import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
NETWORK_CPP_PATH = ROOT / "lib/connectivity/NetworkServices.cpp"


class NetworkServicesContractTest(unittest.TestCase):
    def test_connectivity_module_owns_wifi_button_and_ota(self):
        header = (ROOT / "lib/connectivity/NetworkServices.h").read_text(
            encoding="utf-8")
        source = NETWORK_CPP_PATH.read_text(encoding="utf-8")
        self.assertIn("class NetworkServiceEvents", header)
        self.assertNotIn("class NetworkEvents", header)
        for token in ("WiFi.begin", "WiFi.softAP", "ArduinoOTA.onStart",
                      "ArduinoOTA.handle", "esp_wifi_set_channel",
                      "12000", "1500", "{1, 6, 11}"):
            self.assertIn(token, source)

    def test_application_uses_boundary_instead_of_framework_calls(self):
        application = (ROOT / "src/MotorControlApplication.cpp").read_text(
            encoding="utf-8")
        header = (ROOT / "src/MotorControlApplication.h").read_text(
            encoding="utf-8")
        for token in ("WiFi.begin", "WiFi.softAP", "ArduinoOTA.onStart",
                      "ArduinoOTA.handle", "esp_wifi_set_channel"):
            self.assertNotIn(token, application)
        self.assertIn("NetworkServices network_", header)
        self.assertIn("void MotorControlApplication::onOtaStart", application)


if __name__ == "__main__":
    unittest.main()
