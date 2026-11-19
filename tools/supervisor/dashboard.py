"""Supervisor dashboard for the car (docs/TELEMETRY.md).

Subscribes to csc13/#, decodes each payload with payloads.py, keeps an event
log of what changed, and serves a local page with the live state and the
Start, Stop and Reset commands. Run:

    python3 tools/supervisor/dashboard.py --broker <broker IP>
    then open http://127.0.0.1:8000
"""

import argparse
import collections
import itertools
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import paho.mqtt.client as mqtt

import payloads

PREFIX = "csc13/"
COUNTS = ("barcodes", "humps", "bypasses")  # Telemetry counters, which wrap.
PAGE = Path(__file__).with_name("dashboard.html")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
lock = threading.Lock()  # Guards latest and events: MQTT and HTTP threads.
latest = {}  # Topic name: (time received, decoded fields).
events = collections.deque(maxlen=1000)  # {"id", "t", "text", "kind"}.
ids = itertools.count(1)  # Not reset by Clear log, so the page's since holds.
RUN = time.time()  # Tells the page this server restarted, ids from 1 again.


def decode(topic, payload):
    """A topic's name and decoded fields, or None if it is not a payload of
    ours. Enumerated fields are given by name, faults as a list of names."""
    name = topic.removeprefix(PREFIX)
    if (
        not topic.startswith(PREFIX)
        or name not in payloads.LAYOUTS
        or len(payload) != payloads.size(name)
    ):
        return None
    values = payloads.unpack(name, payload)
    for field, names in payloads.NAMES.get(name, {}).items():
        if values[field] < len(names):  # A value the C side added later.
            values[field] = names[values[field]]
    if name == "telemetry":
        faults = values["faults"]
        values["faults"] = [
            fault for bit, fault in enumerate(payloads.FAULTS) if faults >> bit & 1
        ]
    return name, values


def telemetry_events(old, new):
    """(text, kind) for each change worth logging between two telemetry
    messages. Counts wrap, so a change is any difference."""
    found = []
    if new["state"] != old["state"]:
        found.append((f"{old['state']} → {new['state']}", "state"))
        if new["state"] == "SAFE_STOP":  # FR12: a run ends on Stop.
            found.append(
                (
                    (
                        f"Run ended: highest hump since power-on "
                        f"{new['hump_max']} mm, "
                        f"faults: {', '.join(new['faults']) or 'none'}"
                    ),
                    "summary",
                )
            )
    if new["barcodes"] != old["barcodes"]:
        found.append((f"Barcode read: {new['barcode']}", "info"))
    if new["humps"] != old["humps"]:
        found.append((f"Hump measured, highest so far {new['hump_max']} mm", "info"))
    if new["bypasses"] != old["bypasses"]:
        found.append(("Bypass complete", "info"))
    found += [
        (f"Fault: {fault}", "fault")
        for fault in new["faults"]
        if fault not in old["faults"]
    ]
    found += [
        (f"Fault cleared: {fault}", "info")
        for fault in old["faults"]
        if fault not in new["faults"]
    ]
    return found


def heartbeat_event(old_count, count):
    """(text, kind) when the heartbeat count shows a gap or a restart. The
    count wraps at 65536; a count that goes back means the car restarted."""
    step = None if old_count is None else (count - old_count) % 65536
    if step is None or step == 1:
        return None
    if step == 0 or step > 32768:
        return ("Car restarted", "state")
    return (f"{step - 1} heartbeats missed", "fault")


def log(text, kind="info"):
    with lock:
        events.append(
            {
                "id": next(ids),
                "t": time.time(),
                "text": text,
                "kind": kind,
            }
        )


def on_connect(client, userdata, flags, reason_code, properties):
    client.subscribe(PREFIX + "#")  # Again after every reconnect.
    log("Dashboard connected to the broker", "state")


def on_disconnect(client, userdata, flags, reason_code, properties):
    log("Dashboard lost the broker; reconnecting", "fault")


def on_message(client, userdata, message):
    decoded = decode(message.topic, message.payload)
    if decoded is None:
        return
    name, values = decoded
    with lock:
        t, old = latest.get(name, (0, None))
        if name == "telemetry" and old and time.time() - t > 2:
            # A gap, maybe a reboot: counts that start again from 0 are not
            # new events, but the state and faults still compare.
            old = dict(old, **{k: values[k] for k in COUNTS})
        latest[name] = (time.time(), values)
    if name == "telemetry":
        found = (
            telemetry_events(old, values)
            if old
            else [(f"Telemetry received: {values['state']}", "state")]
        )
    elif name == "heartbeat":
        found = [heartbeat_event(old and old["count"], values["count"])]
    elif name == "scan":
        found = [
            (
                (
                    f"Obstacle profiled: closest {values['closest_mm']} mm at "
                    f"{values['closest_deg']}°, width {values['width_mm']} mm"
                ),
                "info",
            )
        ]
    else:
        found = []
    for event in filter(None, found):
        log(*event)


def snapshot(since):
    """What the page shows: each topic's latest fields and age, and the
    events after id since."""
    now = time.time()
    with lock:
        return {
            "run": RUN,
            "broker": client.is_connected(),
            "topics": {
                name: {"age_s": round(now - t, 1), "values": v}
                for name, (t, v) in latest.items()
            },
            "events": [e for e in events if e["id"] > since][-200:],
        }


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith("/state"):  # /state?since=<last event id>
            since = int(self.path.partition("since=")[2] or 0)
            self.reply(200, json.dumps(snapshot(since)), "application/json")
        elif self.path == "/log":  # The whole event log, as a text file.
            with lock:
                lines = [
                    time.strftime("%H:%M:%S ", time.localtime(e["t"])) + e["text"]
                    for e in events
                ]
            self.reply(200, "\n".join(lines) + "\n", "text/plain")
        else:
            self.reply(200, PAGE.read_bytes(), "text/html")

    def do_POST(self):  # /clear, or /cmd/START, /cmd/STOP or /cmd/RESET.
        name = self.path.removeprefix("/cmd/")
        if self.path == "/clear":
            with lock:
                events.clear()
            self.reply(200, "", "text/plain")
        elif name not in payloads.COMMANDS:
            self.reply(404, "Unknown command", "text/plain")
        elif not client.is_connected():
            self.reply(503, "Not sent: no broker connection", "text/plain")
        else:
            client.publish(PREFIX + "cmd", bytes([payloads.COMMANDS[name]]))
            log(f"Sent {name.capitalize()}", "command")
            self.reply(200, "", "text/plain")

    def reply(self, status, body, content_type):
        body = body.encode() if isinstance(body, str) else body
        self.send_response(status)
        self.send_header("Content-Type", content_type + "; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
        pass  # The page polls four times a second.


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--broker", default="127.0.0.1", help="broker address")
    parser.add_argument("--broker-port", type=int, default=1883)
    parser.add_argument("--port", type=int, default=8000, help="port of the local page")
    args = parser.parse_args()

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.connect_async(args.broker, args.broker_port)  # Broker may be late.
    client.loop_start()  # Its own thread; it reconnects by itself.
    print(
        f"Broker {args.broker}:{args.broker_port}, page http://127.0.0.1:{args.port}",
        flush=True,
    )
    ThreadingHTTPServer(("127.0.0.1", args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
