"""Unit tests of dashboard.py: decoding and the command bytes.
Run as in S1-U03 of testing/subsystem1/TEST_LOG.md."""

import unittest

import dashboard
import payloads


class DashboardTest(unittest.TestCase):
    def test_decodes_what_the_car_packs(self):
        # The bytes of the C packers' test: top bits, and a negative roll.
        name, values = dashboard.decode("csc13/telemetry",
                                        payloads.C_TELEMETRY_BYTES)
        self.assertEqual(name, "telemetry")
        self.assertEqual(values["state"], "SAFE_STOP")
        self.assertEqual(values["faults"], ["WIFI", "LIFTED"])
        self.assertEqual((values["pitch"], values["roll"]), (900, -900))
        self.assertEqual(values["forward_mm"], 4000)
        self.assertEqual(
            dashboard.decode("csc13/scan", payloads.C_SCAN_BYTES),
            ("scan", payloads.C_SCAN_VALUES))

    def test_ignores_other_messages(self):
        self.assertIsNone(dashboard.decode("csc13/telemetry", bytes(10)))
        self.assertIsNone(dashboard.decode("csc13/other", b"\x01"))
        self.assertIsNone(dashboard.decode("elsewhere/cmd", b"\x01"))


if __name__ == "__main__":
    unittest.main()
