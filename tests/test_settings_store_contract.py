import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
DEVICE_SETTINGS_H = ROOT / "lib/settings/DeviceSettings.h"
STORE_H = ROOT / "lib/settings/PreferencesSettingsStore.h"
STORE_CPP = ROOT / "lib/settings/PreferencesSettingsStore.cpp"
APPLICATION_H = ROOT / "src/MotorControlApplication.h"
APPLICATION_CPP = ROOT / "src/MotorControlApplication.cpp"


class SettingsStoreContractTest(unittest.TestCase):
    def test_complete_versioned_snapshot_model_exists(self):
        header = DEVICE_SETTINGS_H.read_text(encoding="utf-8")
        for token in ("struct DeviceSettings", "MotionSequenceConfig sequence",
                      "AdrcPositionSettings position", "MotorDriverSettings motor",
                      "SensorSettings sensor", "bool validateDeviceSettings"):
            self.assertIn(token, header)

    def test_store_is_the_only_preferences_boundary(self):
        header = STORE_H.read_text(encoding="utf-8")
        source = STORE_CPP.read_text(encoding="utf-8")
        self.assertIn("class PreferencesSettingsStore", header)
        self.assertIn('preferences_.begin("motor_cfg_v2"', source)
        self.assertIn('putBytes("snapshot"', source)
        self.assertIn('getBytesLength("snapshot")', source)

    def test_application_has_no_legacy_key_level_persistence(self):
        application = APPLICATION_CPP.read_text(encoding="utf-8")
        header = APPLICATION_H.read_text(encoding="utf-8")
        for token in ('getFloat("adrc_wc"', 'putFloat("adrc_wc"',
                      'getBytes("sequence"', 'putBool("running"'):
            self.assertNotIn(token, application)
        self.assertNotIn("Preferences g_repetitive_preferences", application)
        self.assertIn("DeviceSettings settings_", header)
        self.assertIn("PreferencesSettingsStore settings_store_", header)

    def test_run_endpoint_reports_snapshot_save_failure(self):
        application = APPLICATION_CPP.read_text(encoding="utf-8")
        public_handler = application.split(
            "OperationResult MotorControlApplication::setRunning", 1)[1]
        public_handler = public_handler.split(
            "OperationResult MotorControlApplication::manualMove", 1)[0]
        self.assertIn("if (!setRunning(running, true))", public_handler)
        self.assertIn('return {false, 500, "Falha ao salvar configuracao"}',
                      public_handler)
        self.assertLess(public_handler.index("setRunning(running, true)"),
                        public_handler.index("stall_fault_ = false"))


if __name__ == "__main__":
    unittest.main()
