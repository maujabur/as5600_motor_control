import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
MAIN = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
APP_PATH = ROOT / "src" / "MotorControlApplication.cpp"
APP = APP_PATH.read_text(encoding="utf-8") if APP_PATH.exists() else ""


class ApplicationContractTests(unittest.TestCase):
    def test_main_only_forwards_arduino_lifecycle(self):
        self.assertIn("MotorControlApplication application", MAIN)
        self.assertIn("application.setup()", MAIN)
        self.assertIn("application.update(millis())", MAIN)
        self.assertLess(len(MAIN.splitlines()), 30)

    def test_application_preserves_safety_order(self):
        tokens = (
            "network_.update",
            "web_.handleClient",
            "motion_.updateSensor",
            "sequence_.update",
            "motion_.updateControl",
            "telemetry_.update",
        )
        for token in tokens:
            self.assertIn(token, APP)
        order = [APP.index(token) for token in tokens]
        self.assertEqual(order, sorted(order))

    def test_ota_start_stops_sequence_and_motion(self):
        self.assertIn("void MotorControlApplication::onOtaStart", APP)
        body = APP.split("void MotorControlApplication::onOtaStart", 1)[1]
        self.assertIn("setRunning(false", body)
        self.assertIn("motion_.cancelMove()", body)


if __name__ == "__main__":
    unittest.main()
