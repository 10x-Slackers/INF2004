"""Unit tests of dashboard.py: decoding and the event log.
Run as in S1-U03 of testing/subsystem1/TEST_LOG.md."""

import unittest

import dashboard
import payloads


class DashboardTest(unittest.TestCase):
    def test_decodes_what_the_car_packs(self):
        # The bytes of the C packers' test: top bits, and a negative roll.
        name, values = dashboard.decode("csc13/telemetry", payloads.C_TELEMETRY_BYTES)
        self.assertEqual(name, "telemetry")
        self.assertEqual(values["state"], "SAFE_STOP")
        self.assertEqual(values["barcode"], "U-turn")
        self.assertEqual(
            values["faults"], ["STALL_LEFT", "IMU_SENSOR", "BARCODE_SENSOR"]
        )
        self.assertEqual((values["pitch"], values["roll"]), (900, -900))
        self.assertEqual(
            dashboard.decode("csc13/scan", payloads.C_SCAN_BYTES),
            ("scan", payloads.C_SCAN_VALUES),
        )

    def test_ignores_other_messages(self):
        self.assertIsNone(dashboard.decode("csc13/telemetry", bytes(10)))
        self.assertIsNone(dashboard.decode("csc13/other", b"\x01"))
        self.assertIsNone(dashboard.decode("elsewhere/cmd", b"\x01"))

    def test_logs_what_changed(self):
        old = {
            "state": "FOLLOW",
            "barcode": "left",
            "barcodes": 15,
            "humps": 1,
            "hump_max": 12,
            "bypasses": 0,
            "faults": ["NO_ECHO"],
        }
        new = dict(
            old, state="SAFE_STOP", barcode="right", barcodes=0, faults=["TILT"]
        )  # The barcode count wrapped.
        self.assertEqual(
            dashboard.telemetry_events(old, new),
            [
                ("FOLLOW → SAFE_STOP", "state"),
                (
                    "Run ended: highest hump this run 12 mm, faults: TILT",
                    "summary",
                ),
                ("Barcode read: right", "info"),
                ("Fault: TILT", "fault"),
                ("Fault cleared: NO_ECHO", "info"),
            ],
        )
        self.assertEqual(dashboard.telemetry_events(old, old), [])

    def test_heartbeat_gaps_and_restarts(self):
        self.assertIsNone(dashboard.heartbeat_event(None, 7))
        self.assertIsNone(dashboard.heartbeat_event(65535, 0))  # Wraps.
        self.assertEqual(
            dashboard.heartbeat_event(65535, 2), ("2 heartbeats missed", "fault")
        )
        self.assertEqual(dashboard.heartbeat_event(410, 3), ("Car restarted", "state"))


if __name__ == "__main__":
    unittest.main()
