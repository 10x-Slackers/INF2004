"""Pack, unpack and size the MQTT payloads of docs/TELEMETRY.md.

Each payload is a bit stream, most significant bit first, like
motion_control_pack(). Signed fields are sent with an offset. The script
round-trips every field at its limits, then compares each payload with the
same data as JSON in the Week 6 design. The JSON carries the Week 6 common
fields (v, robot_id, seq) that the binary drops, so part of the saving is
those fields, not the packing.
Run as in S1-U01 of testing/subsystem1/TEST_LOG.md.
"""

import json

# (name, bits, offset added before packing)
LAYOUTS = {
    "telemetry": [
        ("state", 3, 0), ("latched", 1, 0),
        ("line", 2, 0), ("junction", 1, 0), ("following", 1, 0),
        ("barcode", 3, 0), ("barcodes", 4, 0), ("motion_class", 3, 0),
        ("pitch", 11, 1024), ("roll", 11, 1024),
        ("hump_max", 6, 0), ("humps", 4, 0), ("on_hump", 1, 0),
        ("avoid_state", 3, 0), ("avoid_side", 2, 0), ("bypasses", 4, 0),
        ("forward_mm", 12, 0), ("faults", 10, 0),
    ],
    "motion": [
        ("state", 2, 0), ("command", 3, 0), ("heading_deg", 9, 0), ("left_speed", 11, 1024),
        ("right_speed", 11, 1024), ("left_count", 16, 0),
        ("right_count", 16, 0), ("distance_mm", 24, 0),
    ],
    "scan": [
        ("closest_mm", 12, 0), ("closest_deg", 8, 0), ("right_deg", 8, 0),
        ("left_deg", 8, 0), ("width_mm", 8, 0), ("left_mm", 12, 0),
        ("right_mm", 12, 0),
    ],
    "heartbeat": [("count", 16, 0)],
    "cmd": [("cmd", 8, 0)],  # Event flag bits: 1 START, 2 STOP, 4 RESET.
}

# Typical values, as the car would send them.
SAMPLES = {
    "telemetry": dict(state=2, latched=0, line=1, junction=0,
                      following=1, barcode=1, barcodes=3, motion_class=0,
                      pitch=-12, roll=35, hump_max=21, humps=2, on_hump=0,
                      avoid_state=1, avoid_side=0, bypasses=1,
                      forward_mm=1830, faults=0),
    "motion": dict(state=1, command=5, heading_deg=271,
                   left_speed=198, right_speed=203, left_count=48211,
                   right_count=48902, distance_mm=12450),
    "scan": dict(closest_mm=120, closest_deg=90, right_deg=70, left_deg=105,
                 width_mm=73, left_mm=820, right_mm=140),
    "heartbeat": dict(count=1234),
    "cmd": dict(cmd=1),
}

# motion_control_pack() output for these values, from S2-U21's
# testing/subsystem2/host/test_motion.c, so both sides use one layout.
C_MOTION_VALUES = dict(state=2, command=3, heading_deg=345,
                       left_speed=100, right_speed=-500, left_count=0x1234,
                       right_count=0xABCD, distance_mm=0xEF0123)
C_MOTION_BYTES = bytes.fromhex("9d663220c1234abcdef01230")

# The same data as Week 6 JSON: names spelled out, enums as words.
STATES = ["INIT", "READY", "FOLLOW", "AVOID", "TURN", "SAFE_STOP"]
COMMON = {"v": 1, "robot_id": "CSC13", "seq": 1234}
CMD_JSON = {"v": 1, "robot_id": "CSC13", "session_id": "s1", "command_id": "c7",
            "seq": 7, "cmd": "START", "challenge": "n4"}  # Week 6, verbatim.

# Messages per second (scan: one per obstacle; cmd: on demand).
RATES = {"telemetry": 5, "motion": 5, "heartbeat": 1}


def width(name):
    return sum(bits for _, bits, _ in LAYOUTS[name])


def pack(name, values):
    total = width(name)
    v = 0
    for field, bits, offset in LAYOUTS[name]:
        raw = values[field] + offset
        assert 0 <= raw < (1 << bits), (name, field, values[field])
        v = (v << bits) | raw
    pad = -total % 8
    return (v << pad).to_bytes((total + pad) // 8, "big")


def unpack(name, payload):
    total = width(name)
    v = int.from_bytes(payload, "big") >> (len(payload) * 8 - total)
    out = {}
    for field, bits, offset in reversed(LAYOUTS[name]):
        out[field] = (v & ((1 << bits) - 1)) - offset
        v >>= bits
    return out


def as_json(name, values):
    doc = CMD_JSON if name == "cmd" else {**COMMON, **values}
    if name == "telemetry":
        doc["state"] = STATES[values["state"]]
    return json.dumps(doc, separators=(",", ":")).encode()


def main():
    failures = 0

    def check(ok, what):
        nonlocal failures
        failures += not ok
        print(f"{'PASS' if ok else 'FAIL'}  {what}")

    for name, layout in LAYOUTS.items():
        for label in ("min", "max"):  # Every field at its limits.
            values = {f: (0 if label == "min" else (1 << b) - 1) - o
                      for f, b, o in layout}
            check(unpack(name, pack(name, values)) == values,
                  f"{name} round trip ({label})")
    check(pack("motion", C_MOTION_VALUES) == C_MOTION_BYTES,
          "motion matches motion_control_pack()")

    json_s = binary_s = 0
    print(f"\n{'Topic':<10} {'Bits':>4} {'Binary B':>9} {'JSON B':>7} "
          f"{'Saved':>6} {'Hz':>3} {'JSON B/s':>9} {'Binary B/s':>11}")
    for name, values in SAMPLES.items():
        binary = pack(name, values)
        text = as_json(name, values)
        hz = RATES.get(name, 0)
        json_s += len(text) * hz
        binary_s += len(binary) * hz
        print(f"{name:<10} {width(name):>4} {len(binary):>9} {len(text):>7} "
              f"{1 - len(binary) / len(text):>6.0%} {hz or '-':>3} "
              f"{len(text) * hz:>9} {len(binary) * hz:>11}")
        print(f"           {binary.hex()}  {text.decode()}")
    print(f"\nPeriodic total: JSON {json_s} B/s, binary {binary_s} B/s")
    print("\nALL PASS" if failures == 0 else f"\nFAILED: {failures}")

if __name__ == "__main__":
    main()
