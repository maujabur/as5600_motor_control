import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
HEADER = ROOT / "lib/motion_control/AngleSensorManager.h"
SOURCE = ROOT / "lib/motion_control/AngleSensorManager.cpp"


class AngleSensorManagerContractTest(unittest.TestCase):
    def test_manager_exposes_state_identity_and_events(self):
        header = HEADER.read_text(encoding="utf-8")
        for token in ("enum class State", "Detecting", "Active", "Lost",
                      "setFailureLimit", "failureCount", "consumeLostEvent",
                      "consumeRecoveredEvent", "sensorName", "sensorAddress"):
            self.assertIn(token, header)

    def test_detection_order_and_interval_are_fixed(self):
        source = SOURCE.read_text(encoding="utf-8")
        positions = [source.index(token) for token in
                     ("As5600Sensor::DEFAULT_I2C_ADDR",
                      "Mt6701Sensor::DEFAULT_I2C_ADDR",
                      "Mt6701Sensor::ALTERNATE_I2C_ADDR")]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("1000", source)

    def test_consecutive_failures_drive_lost_state(self):
        source = SOURCE.read_text(encoding="utf-8")
        self.assertIn("failure_count_ = 0", source)
        self.assertIn("failure_count_ >= failure_limit_", source)
        self.assertIn("State::Lost", source)


if __name__ == "__main__":
    unittest.main()
