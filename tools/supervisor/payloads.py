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
        ("state", 3, 0),
        ("latched", 1, 0),
        ("line", 2, 0),
        ("junction", 1, 0),
        ("following", 1, 0),
        ("barcode", 3, 0),
        ("barcodes", 4, 0),
        ("motion_class", 3, 0),
        ("pitch", 11, 1024),
        ("roll", 11, 1024),
        ("hump_max", 6, 0),
        ("humps", 4, 0),
        ("on_hump", 1, 0),
        ("avoid_state", 3, 0),
        ("avoid_side", 2, 0),
        ("bypasses", 4, 0),
        ("forward_mm", 12, 0),
        ("faults", 11, 0),
    ],
    "motion": [
        ("state", 2, 0),
        ("command", 3, 0),
        ("heading_deg", 9, 0),
        ("left_speed", 11, 1024),
        ("right_speed", 11, 1024),
        ("left_count", 16, 0),
        ("right_count", 16, 0),
        ("distance_mm", 24, 0),
    ],
    "scan": [
        ("closest_mm", 12, 0),
        ("closest_deg", 8, 0),
        ("right_deg", 8, 0),
        ("left_deg", 8, 0),
        ("width_mm", 8, 0),
        ("left_mm", 12, 0),
        ("right_mm", 12, 0),
    ],
    "heartbeat": [("count", 16, 0)],
    "cmd": [("cmd", 8, 0)],  # COMMANDS bits, below.
}

# Names of the enumerated fields by value, from the C headers.
STATES = ["INIT", "READY", "FOLLOW", "AVOID", "TURN", "SAFE_STOP"]
NAMES = {
    "telemetry": {
        "state": STATES,
        "barcode": ["none", "left", "right", "straight", "U-turn"],
        "motion_class": ["steady", "accelerating", "braking", "climbing", "descending"],
        "avoid_state": ["idle", "watching", "profiling", "bypassing", "stopped"],
        "avoid_side": ["none", "left", "right"],
    },
    "motion": {
        "state": ["idle", "running", "complete", "failed"],
        "command": ["none", "forward", "backward", "left", "right", "drive"],
    },
}
FAULTS = [
    "STALL_LEFT",
    "STALL_RIGHT",
    "ENCODER",
    "UPSIDE_DOWN",
    "TILT",
    "NO_ECHO",
    "AVOID_SENSOR",
    "AVOID_BLOCKED",
    "AVOID_MOTION",
    "WIFI",
    "LIFTED",
]  # APP_FAULT_... in app_fault.h, bit 0 first.
COMMANDS = {"START": 1, "STOP": 2, "RESET": 4}  # S1_CMD_... in subsystem1.h.

# Typical values, as the car would send them.
SAMPLES = {
    "telemetry": {
        "state": 2,
        "latched": 0,
        "line": 1,
        "junction": 0,
        "following": 1,
        "barcode": 1,
        "barcodes": 3,
        "motion_class": 0,
        "pitch": -12,
        "roll": 35,
        "hump_max": 21,
        "humps": 2,
        "on_hump": 0,
        "avoid_state": 1,
        "avoid_side": 0,
        "bypasses": 1,
        "forward_mm": 1830,
        "faults": 0,
    },
    "motion": {
        "state": 1,
        "command": 5,
        "heading_deg": 271,
        "left_speed": 198,
        "right_speed": 203,
        "left_count": 48211,
        "right_count": 48902,
        "distance_mm": 12450,
    },
    "scan": {
        "closest_mm": 120,
        "closest_deg": 90,
        "right_deg": 70,
        "left_deg": 105,
        "width_mm": 73,
        "left_mm": 820,
        "right_mm": 140,
    },
    "heartbeat": {"count": 1234},
    "cmd": {"cmd": 1},
}

# motion_control_pack() output for these values, from S2-U21's
# testing/subsystem2/host/test_motion.c, so both sides use one layout.
C_MOTION_VALUES = {
    "state": 2,
    "command": 3,
    "heading_deg": 345,
    "left_speed": 100,
    "right_speed": -500,
    "left_count": 0x1234,
    "right_count": 0xABCD,
    "distance_mm": 0xEF0123,
}
C_MOTION_BYTES = bytes.fromhex("9d663220c1234abcdef01230")

# payload_pack_telemetry() and payload_pack_scan() output for these values,
# from S1-U02's test_payload.c. Every field has its top bit set, so a field
# packed too narrow shows; roll is negative instead, for the offset.
C_TELEMETRY_VALUES = {
    "state": 5,
    "latched": 1,
    "line": 2,
    "junction": 1,
    "following": 1,
    "barcode": 4,
    "barcodes": 9,
    "motion_class": 4,
    "pitch": 900,
    "roll": -900,
    "hump_max": 42,
    "humps": 10,
    "on_hump": 1,
    "avoid_state": 4,
    "avoid_side": 2,
    "bypasses": 9,
    "forward_mm": 4000,
    "faults": 0x600,
}
C_TELEMETRY_BYTES = bytes.fromhex("bb933c207caab29fa0c000")
C_SCAN_VALUES = {
    "closest_mm": 2100,
    "closest_deg": 135,
    "right_deg": 130,
    "left_deg": 135,
    "width_mm": 200,
    "left_mm": 4000,
    "right_mm": 2050,
}
C_SCAN_BYTES = bytes.fromhex("834878287c8fa08020")

# The same data as Week 6 JSON: names spelled out, enums as words.
COMMON = {"v": 1, "robot_id": "CSC13", "seq": 1234}
CMD_JSON = {
    "v": 1,
    "robot_id": "CSC13",
    "session_id": "s1",
    "command_id": "c7",
    "seq": 7,
    "cmd": "START",
    "challenge": "n4",
}  # Week 6, verbatim.

# Messages per second (scan: one per obstacle; cmd: on demand).
RATES = {"telemetry": 5, "motion": 5, "heartbeat": 1}


def width(name):
    return sum(bits for _, bits, _ in LAYOUTS[name])


def size(name):
    """Bytes in a payload: its bits padded with zeros to whole bytes."""
    return (width(name) + 7) // 8


def pack(name, values):
    v = 0
    for field, bits, offset in LAYOUTS[name]:
        raw = values[field] + offset
        assert 0 <= raw < (1 << bits), (name, field, values[field])
        v = (v << bits) | raw
    return (v << (size(name) * 8 - width(name))).to_bytes(size(name), "big")


def unpack(name, payload):
    v = int.from_bytes(payload, "big")
    shift = len(payload) * 8
    out = {}
    for field, bits, offset in LAYOUTS[name]:  # In payload order.
        shift -= bits
        out[field] = (v >> shift & ((1 << bits) - 1)) - offset
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
            values = {
                f: (0 if label == "min" else (1 << b) - 1) - o for f, b, o in layout
            }
            check(
                unpack(name, pack(name, values)) == values,
                f"{name} round trip ({label})",
            )
    check(
        pack("motion", C_MOTION_VALUES) == C_MOTION_BYTES,
        "motion matches motion_control_pack()",
    )
    check(
        pack("telemetry", C_TELEMETRY_VALUES) == C_TELEMETRY_BYTES,
        "telemetry matches payload_pack_telemetry()",
    )
    check(
        pack("scan", C_SCAN_VALUES) == C_SCAN_BYTES, "scan matches payload_pack_scan()"
    )

    json_s = binary_s = 0
    print(
        f"\n{'Topic':<10} {'Bits':>4} {'Binary B':>9} {'JSON B':>7} "
        f"{'Saved':>6} {'Hz':>3} {'JSON B/s':>9} {'Binary B/s':>11}"
    )
    for name, values in SAMPLES.items():
        binary = pack(name, values)
        text = as_json(name, values)
        hz = RATES.get(name, 0)
        json_s += len(text) * hz
        binary_s += len(binary) * hz
        print(
            f"{name:<10} {width(name):>4} {len(binary):>9} {len(text):>7} "
            f"{1 - len(binary) / len(text):>6.0%} {hz or '-':>3} "
            f"{len(text) * hz:>9} {len(binary) * hz:>11}"
        )
        print(f"           {binary.hex()}  {text.decode()}")
    print(f"\nPeriodic total: JSON {json_s} B/s, binary {binary_s} B/s")
    print("\nALL PASS" if failures == 0 else f"\nFAILED: {failures}")


if __name__ == "__main__":
    main()
