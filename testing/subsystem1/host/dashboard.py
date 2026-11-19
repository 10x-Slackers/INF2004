"""Supervisor dashboard for the car's MQTT telemetry (docs/TELEMETRY.md).

Subscribes to csc13/#, decodes each payload with payloads.py and serves a
local page with the latest values and START, STOP and RESET buttons, which
publish the one-byte command on csc13/cmd. Run:

    python3 dashboard.py --broker <broker IP>    # then open http://127.0.0.1:8000
"""

import argparse
import json
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import paho.mqtt.client as mqtt

import payloads

PREFIX = "csc13/"
COMMANDS = {"START": 1, "STOP": 2, "RESET": 4}  # Bits of the cmd byte.
FAULTS = ["STALL_LEFT", "STALL_RIGHT", "ENCODER", "UPSIDE_DOWN", "TILT",
          "NO_ECHO", "AVOID_SENSOR", "AVOID_BLOCKED", "AVOID_MOTION", "WIFI",
          "LIFTED"]  # APP_FAULT_... in app_fault.h, bit 0 first.
PAGE = Path(__file__).with_name("dashboard.html").read_bytes()

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
latest = {}  # Topic name: (time received, decoded fields).


def decode(topic, payload):
    """A topic's name and decoded fields, or None if it is not a payload of
    ours. The telemetry state and faults are given by name."""
    name = topic.removeprefix(PREFIX)
    if (name not in payloads.LAYOUTS
            or len(payload) != (payloads.width(name) + 7) // 8):
        return None
    # unpack() fills its dict from the last field back: show payload order.
    values = dict(reversed(payloads.unpack(name, payload).items()))
    if name == "telemetry":
        state = values["state"]
        values["state"] = (payloads.STATES[state]
                           if state < len(payloads.STATES) else state)
        values["faults"] = [fault for bit, fault in enumerate(FAULTS)
                            if values["faults"] >> bit & 1]
    return name, values


def command(name):
    """The cmd payload for START, STOP or RESET: one byte."""
    return bytes([COMMANDS[name]])


def snapshot():
    """What the page shows: the latest fields of each topic and their age."""
    now = time.time()
    return {"broker": client.is_connected(),
            "topics": {name: {"age_s": round(now - latest[name][0], 1),
                              "values": latest[name][1]}
                       for name in payloads.LAYOUTS if name in latest}}


def on_connect(client, userdata, flags, reason_code, properties):
    client.subscribe(PREFIX + "#")  # Again after every reconnect.


def on_message(client, userdata, message):
    decoded = decode(message.topic, message.payload)
    if decoded:
        latest[decoded[0]] = (time.time(), decoded[1])


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/state":
            self.reply(json.dumps(snapshot()).encode(), "application/json")
        else:
            self.reply(PAGE, "text/html; charset=utf-8")

    def do_POST(self):  # /cmd/START, /cmd/STOP or /cmd/RESET.
        client.publish(PREFIX + "cmd", command(self.path.rsplit("/", 1)[1]))
        self.reply(b"", "text/plain")

    def reply(self, body, content_type):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
        pass  # The page polls twice a second.


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--broker", default="127.0.0.1", help="broker address")
    parser.add_argument("--broker-port", type=int, default=1883)
    parser.add_argument("--port", type=int, default=8000,
                        help="port of the local page")
    args = parser.parse_args()

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect_async(args.broker, args.broker_port)  # Broker may be late.
    client.loop_start()  # Its own thread; it reconnects by itself.
    print(f"Broker {args.broker}:{args.broker_port}, "
          f"page http://127.0.0.1:{args.port}", flush=True)
    ThreadingHTTPServer(("127.0.0.1", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
