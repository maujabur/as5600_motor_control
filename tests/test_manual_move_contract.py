import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
MAIN = (ROOT / "src" / "main.cpp").read_text(encoding="utf-8")
WEB = (ROOT / "lib/web/WebControlServer.cpp").read_text(encoding="utf-8")
PAGE = (ROOT / "lib/web/repetitive_motion_web_page.h").read_text(encoding="utf-8")


class ManualMoveContractTest(unittest.TestCase):
    def test_api_accepts_target_and_rpm_and_uses_shortest_path(self):
        self.assertIn('server_.on("/api/manual-move", HTTP_POST', WEB)
        self.assertIn('parseNumber("target", &target_deg)', WEB)
        self.assertIn('parseNumber("rpm", &rpm)', WEB)
        self.assertIn('MotionDirection::Shortest', WEB)

    def test_api_rejects_move_when_controller_is_busy(self):
        marker = 'server_.on("/api/manual-move", HTTP_POST'
        self.assertIn(marker, WEB)
        endpoint = WEB.split(marker, 1)[1]
        endpoint = endpoint.split('server_.on(', 1)[0]
        self.assertIn('status.running', endpoint)
        self.assertIn('status.move_active', endpoint)
        self.assertIn('status.ota_busy', endpoint)

    def test_page_exposes_non_persistent_manual_move_controls(self):
        self.assertIn('id="manualTarget"', PAGE)
        self.assertIn('id="manualRpm"', PAGE)
        self.assertIn("api('/api/manual-move'", PAGE)
        self.assertNotIn('manualTarget', MAIN)


if __name__ == "__main__":
    unittest.main()
