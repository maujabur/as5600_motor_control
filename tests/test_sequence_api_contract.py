import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
MAIN = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
WEB = (ROOT / "lib/web/WebControlServer.cpp").read_text(encoding="utf-8")
PAGE = (ROOT / "lib/web/repetitive_motion_web_page.h").read_text(encoding="utf-8")
SETTINGS = (ROOT / "lib/settings/DeviceSettings.h").read_text(encoding="utf-8")
STORE = (ROOT / "lib/settings/PreferencesSettingsStore.cpp").read_text(encoding="utf-8")


class SequenceApiContractTest(unittest.TestCase):
    def test_versioned_sequence_uses_safe_defaults_without_legacy_migration(self):
        self.assertIn("DEVICE_SETTINGS_VERSION", SETTINGS)
        self.assertIn('getBytes("snapshot"', STORE)
        self.assertIn('putBytes("snapshot"', STORE)
        self.assertNotIn('getFloat("start_deg"', MAIN)
        self.assertIn("value.sequence.startup_step = {", SETTINGS)
        self.assertIn("MotionDirection::Clockwise", SETTINGS)
        self.assertIn("MotionDirection::CounterClockwise", SETTINGS)

    def test_sequence_api_validates_count_steps_and_busy_state(self):
        self.assertIn('server_.on("/api/sequence", HTTP_GET', WEB)
        marker = 'server_.on("/api/sequence", HTTP_POST'
        self.assertIn(marker, WEB)
        endpoint = WEB.split(marker, 1)[1].split('server_.on(', 1)[0]
        self.assertIn("MOTION_SEQUENCE_MAX_STEPS", endpoint)
        self.assertIn("status.running", endpoint)
        self.assertIn("status.move_active", endpoint)
        self.assertIn("status.ota_busy", endpoint)
        self.assertIn("parseSequenceStep", endpoint)

    def test_editor_has_startup_loop_and_variable_step_controls(self):
        for token in ('id="startupStep"', 'id="loopSteps"', 'id="addStep"',
                      'data-action="delete"', 'data-action="up"',
                      'data-action="down"', "steps.length>=16"):
            self.assertIn(token, PAGE)
        for direction in ('value="shortest"', 'value="cw"', 'value="ccw"'):
            self.assertIn(direction, PAGE)
        self.assertIn("setSequenceDisabled", PAGE)


if __name__ == "__main__":
    unittest.main()
