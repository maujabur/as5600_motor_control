import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
DEVICE_SETTINGS_H = ROOT / "lib/settings/DeviceSettings.h"
STORE_H = ROOT / "lib/settings/PreferencesSettingsStore.h"
STORE_CPP = ROOT / "lib/settings/PreferencesSettingsStore.cpp"
MAIN = ROOT / "src/main.cpp"


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

    def test_main_has_no_legacy_key_level_persistence(self):
        main = MAIN.read_text(encoding="utf-8")
        for token in ('getFloat("adrc_wc"', 'putFloat("adrc_wc"',
                      'getBytes("sequence"', 'putBool("running"'):
            self.assertNotIn(token, main)
        self.assertNotIn("Preferences g_repetitive_preferences", main)
        self.assertIn("DeviceSettings g_settings", main)
        self.assertIn("PreferencesSettingsStore g_settings_store", main)


if __name__ == "__main__":
    unittest.main()
