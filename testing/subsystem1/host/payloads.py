"""Pack, unpack and size the MQTT payloads of docs/TELEMETRY.md.

Each payload is a bit stream, most significant bit first, like
motion_control_pack(). Signed fields are sent with an offset. The script
round-trips every field at its limits, then compares each payload with the
same data as JSON in the Week 6 design (common fields v, robot_id, seq).
Run as in S1-U01 of testing/subsystem1/TEST_LOG.md.
"""

import json

# (name, bits, offset added before packing)
LAYOUTS = {
    "telemetry": [
        ("state", 3, 0), ("latched", 1, 0), ("faults", 8, 0),
        ("line", 2, 0), ("junction", 1, 0), ("following", 1, 0),
        ("barcode", 3, 0), ("barcodes", 4, 0), ("motion_class", 3, 0),
        ("pitch", 11, 1024), ("roll", 11, 1024),
        ("hump_max", 6, 0), ("humps", 4, 0), ("on_hump", 1, 0),
        ("avoid_state", 3, 0), ("avoid_side", 2, 0), ("bypasses", 4, 0),
        ("forward_mm", 12, 0),
    ],
    "motion": [
        ("state", 2, 0), ("command", 3, 0), ("faults", 3, 0),
        ("heading_deg", 9, 0), ("left_speed", 11, 1024),
        ("right_speed", 11, 1024), ("left_count", 16, 0),
        ("right_count", 16, 0), ("distance_mm", 24, 0), ("pad", 1, 0),
    ],
    "scan": [
        ("closest_mm", 12, 0), ("closest_deg", 7, 0), ("right_deg", 7, 0),
        ("left_deg", 7, 0), ("width_mm", 7, 0), ("left_mm", 12, 0),
        ("right_mm", 12, 0),
    ],
    "heartbeat": [("count", 16, 0)],
    "cmd": [("cmd", 2, 0), ("pad", 6, 0)],
}

# Typical values, as the car would send them.
SAMPLES = {
    "telemetry": dict(state=2, latched=0, faults=0, line=1, junction=0,
                      following=1, barcode=1, barcodes=3, motion_class=0,
                      pitch=-12, roll=35, hump_max=21, humps=2, on_hump=0,
                      avoid_state=1, avoid_side=0, bypasses=1,
                      forward_mm=1830),
    "motion": dict(state=1, command=5, faults=0, heading_deg=271,
                   left_speed=198, right_speed=203, left_count=48211,
                   right_count=48902, distance_mm=12450, pad=0),
    "scan": dict(closest_mm=310, closest_deg=90, right_deg=84, left_deg=97,
                 width_mm=71, left_mm=820, right_mm=0),
    "heartbeat": dict(count=1234),
    "cmd": dict(cmd=1, pad=0),
}

# The same data as Week 6 JSON: names spelled out, enums as words.
STATES = ["INIT", "READY", "FOLLOW", "AVOID", "TURN", "SAFE_STOP"]
COMMON = {"v": 1, "robot_id": "CSC13", "seq": 1234}

# Messages per second (scan: one per obstacle; cmd: on demand).
RATES = {"telemetry": 5, "motion": 10, "heartbeat": 1}


def pack(name, values):
    total = sum(bits for _, bits, _ in LAYOUTS[name])
    v = 0
    for field, bits, offset in LAYOUTS[name]:
        raw = values[field] + offset
        assert 0 <= raw < (1 << bits), (name, field, values[field])
        v = (v << bits) | raw
    pad = -total % 8
    return (v << pad).to_bytes((total + pad) // 8, "big")


def unpack(name, payload):
    total = sum(bits for _, bits, _ in LAYOUTS[name])
    v = int.from_bytes(payload, "big") >> (len(payload) * 8 - total)
    out = {}
    for field, bits, offset in reversed(LAYOUTS[name]):
        out[field] = (v & ((1 << bits) - 1)) - offset
        v >>= bits
    return out


def as_json(name, values):
    doc = dict(COMMON)
    doc.update({k: x for k, x in values.items() if k != "pad"})
    if name == "telemetry":
        doc["state"] = STATES[values["state"]]
    if name == "cmd":
        doc = {"v": 1, "robot_id": "CSC13", "session_id": "s1",
               "command_id": "c7", "seq": 7, "cmd": "START",
               "challenge": "n4"}  # The Week 6 example, verbatim.
    return json.dumps(doc, separators=(",", ":")).encode()


def main():
    failures = 0
    for name, layout in LAYOUTS.items():
        for pick in (0, 1):  # Every field at its lowest, then highest.
            values = {f: (0 if pick == 0 else (1 << b) - 1) - o
                      for f, b, o in layout}
            ok = unpack(name, pack(name, values)) == values
            failures += not ok
            print(f"{'PASS' if ok else 'FAIL'}  {name} round trip "
                  f"({'min' if pick == 0 else 'max'})")

    per_s = [0, 0]
    print(f"\n{'Topic':<10} {'Bits':>4} {'Binary B':>9} {'JSON B':>7} "
          f"{'Saved':>6} {'Hz':>3} {'JSON B/s':>9} {'Binary B/s':>11}")
    for name, values in SAMPLES.items():
        binary = pack(name, values)
        text = as_json(name, values)
        bits = sum(b for _, b, _ in LAYOUTS[name])
        hz = RATES.get(name, 0)
        print(f"{name:<10} {bits:>4} {len(binary):>9} {len(text):>7} "
              f"{1 - len(binary) / len(text):>6.0%} {hz or '-':>3} "
              f"{len(text) * hz:>9} {len(binary) * hz:>11}")
        print(f"           {binary.hex()}  {text.decode()}")
        per_s = [per_s[0] + len(text) * hz, per_s[1] + len(binary) * hz]
    print(f"\nPeriodic total: JSON {per_s[0]} B/s, binary {per_s[1]} B/s")
    print("\nALL PASS" if failures == 0 else f"\nFAILED: {failures}")


if __name__ == "__main__":
    main()
