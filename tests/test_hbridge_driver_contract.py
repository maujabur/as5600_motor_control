import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
HEADER = ROOT / "lib" / "hardware" / "HBridgeMotorDriver.h"
SOURCE = ROOT / "lib" / "hardware" / "HBridgeMotorDriver.cpp"
MAIN = ROOT / "src" / "main.cpp"


class HBridgeMotorDriverContractTest(unittest.TestCase):
    def test_driver_owns_physical_motor_output(self):
        self.assertTrue(HEADER.exists(), "HBridgeMotorDriver.h must exist")
        self.assertTrue(SOURCE.exists(), "HBridgeMotorDriver.cpp must exist")
        header = HEADER.read_text(encoding="utf-8")
        source = SOURCE.read_text(encoding="utf-8")
        main = MAIN.read_text(encoding="utf-8")

        for token in ("writeSignedPercent", "brake", "stop",
                      "power_limit_percent", "lastAppliedPercent"):
            self.assertIn(token, header)
        for token in ("analogWrite", "analogWriteFrequency",
                      "analogWriteResolution"):
            self.assertIn(token, source)
            self.assertNotIn(token, main)

    def test_serial_is_diagnostic_only(self):
        main = MAIN.read_text(encoding="utf-8")
        self.assertNotIn("parseAndHandleCommand", main)
        self.assertNotIn("processSerialInput", main)
        self.assertNotIn("updateRampControl", main)


if __name__ == "__main__":
    unittest.main()
