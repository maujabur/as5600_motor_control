import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class DiagnosticsContractTest(unittest.TestCase):
    def test_motion_metrics_and_periodic_logs_are_isolated(self):
        source = (ROOT / "lib/diagnostics/MotionTelemetry.cpp").read_text(
            encoding="utf-8")
        for token in ("peak_rpm_abs", "mean_rpm", "peak_pwm_abs",
                      "travelled_deg", "2000", "Serial.printf"):
            self.assertIn(token, source)

    def test_serial_is_output_only(self):
        main = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
        self.assertNotIn("Serial.read", main)
        self.assertNotIn("Serial.available", main)
        self.assertNotIn("g_move_", main)
        self.assertRegex(main, r"MotionTelemetry\s+g_telemetry")


if __name__ == "__main__":
    unittest.main()
