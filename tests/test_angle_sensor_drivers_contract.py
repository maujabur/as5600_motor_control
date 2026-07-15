import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class AngleSensorDriversContractTest(unittest.TestCase):
    def test_shared_interface_and_concrete_drivers_exist(self):
        interface = (ROOT / "lib/motion_control/AngleSensor.h").read_text(encoding="utf-8")
        as5600 = (ROOT / "lib/motion_control/As5600Sensor.h").read_text(encoding="utf-8")
        mt6701 = (ROOT / "lib/motion_control/Mt6701Sensor.h").read_text(encoding="utf-8")
        for token in ("enum class AngleSensorType", "virtual bool probe", "readRawAngle",
                      "readAngleDeg", "countsPerTurn"):
            self.assertIn(token, interface)
        self.assertIn("public AngleSensor", as5600)
        self.assertIn("public AngleSensor", mt6701)

    def test_supported_addresses_and_registers_are_explicit(self):
        as5600 = (ROOT / "lib/motion_control/As5600Sensor.h").read_text(encoding="utf-8")
        mt6701 = (ROOT / "lib/motion_control/Mt6701Sensor.h").read_text(encoding="utf-8")
        self.assertIn("0x36", as5600)
        self.assertIn("0x0C", as5600)
        for token in ("0x06", "0x46", "0x03", "0x04"):
            self.assertIn(token, mt6701)

    def test_mt6701_decodes_fourteen_bit_angle(self):
        implementation = (ROOT / "lib/motion_control/Mt6701Sensor.cpp").read_text(encoding="utf-8")
        header = (ROOT / "lib/motion_control/Mt6701Sensor.h").read_text(encoding="utf-8")
        self.assertIn("((uint16_t)high_byte << 6)", implementation)
        self.assertIn("low_byte >> 2", implementation)
        self.assertIn("16384", header)


if __name__ == "__main__":
    unittest.main()
