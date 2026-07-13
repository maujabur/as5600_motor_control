import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
MAIN = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
PAGE = (ROOT / "src" / "repetitive_motion_web_page.h").read_text(encoding="utf-8")


class ManualMoveContractTest(unittest.TestCase):
    def test_api_accepts_target_and_rpm_and_uses_shortest_path(self):
        self.assertIn('g_web_server.on("/api/manual-move", HTTP_POST', MAIN)
        self.assertIn('parseWebNumber("target", &target_deg)', MAIN)
        self.assertIn('parseWebNumber("rpm", &rpm)', MAIN)
        self.assertIn('AdrcPositionController::MoveDirection::Shortest', MAIN)

    def test_api_rejects_move_when_controller_is_busy(self):
        marker = 'g_web_server.on("/api/manual-move", HTTP_POST'
        self.assertIn(marker, MAIN)
        endpoint = MAIN.split(marker, 1)[1]
        endpoint = endpoint.split('g_web_server.on(', 1)[0]
        self.assertIn('g_repetitive_motion.running()', endpoint)
        self.assertIn('g_position_servo.isActive()', endpoint)
        self.assertIn('g_ota_update_in_progress', endpoint)

    def test_page_exposes_non_persistent_manual_move_controls(self):
        self.assertIn('id="manualTarget"', PAGE)
        self.assertIn('id="manualRpm"', PAGE)
        self.assertIn("api('/api/manual-move'", PAGE)
        self.assertNotIn('manualTarget', MAIN.split('void saveRepetitiveMotionConfig()', 1)[1]
                         .split('}', 1)[0])


if __name__ == "__main__":
    unittest.main()
