import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
WEB_CPP_PATH = ROOT / "lib/web/WebControlServer.cpp"


class WebContractTest(unittest.TestCase):
    def test_routes_and_status_fields_are_owned_by_web_module(self):
        source = WEB_CPP_PATH.read_text(encoding="utf-8")
        for route in (
            'on("/", HTTP_GET', 'on("/settings", HTTP_GET',
            'on("/api/status", HTTP_GET', 'on("/api/settings", HTTP_GET',
            'on("/api/settings", HTTP_POST', 'on("/api/sequence", HTTP_GET',
            'on("/api/sequence", HTTP_POST', 'on("/api/run", HTTP_POST',
            'on("/api/adjust", HTTP_POST', 'on("/api/manual-move", HTTP_POST',
            'on("/api/config", HTTP_POST'):
            self.assertIn(route, source)
        for field in ("unit", "running", "moveActive", "phase", "step",
                      "sensor", "sensorType", "sensorAddress", "sensorState",
                      "sensorFailures", "angle", "maxRpm", "otaBusy", "stall"):
            self.assertIn(f'\\"{field}\\"', source)

    def test_main_has_no_http_routing_or_json_assembly(self):
        main = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
        self.assertNotIn("g_web_server.on", main)
        self.assertNotIn("json +=", main)


if __name__ == "__main__":
    unittest.main()
