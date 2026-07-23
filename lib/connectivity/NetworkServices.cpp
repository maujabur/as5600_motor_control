#include "NetworkServices.h"

#include <ArduinoOTA.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <string.h>

namespace {
constexpr uint32_t STATION_TIMEOUT_MS = 12000;
constexpr uint32_t BUTTON_HOLD_MS = 1500;
constexpr uint8_t AP_CHANNELS[] = {1, 6, 11};
}

NetworkServices::NetworkServices(
    const NetworkConfig& config, NetworkServiceEvents& events)
    : config_(config), events_(events) {}

void NetworkServices::configureOta() {
  ArduinoOTA.setHostname(hostname_);
  ArduinoOTA.setPassword(config_.ota_password);
  ArduinoOTA.onStart([this]() {
    ota_busy_ = true;
    events_.onOtaStart();
    Serial.println("OTA: atualizacao iniciada; motores desativados");
  });
  ArduinoOTA.onEnd([this]() {
    ota_busy_ = false;
    Serial.println("\nOTA: atualizacao concluida; reiniciando");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    const unsigned int percent = total == 0 ? 0 : (progress * 100U) / total;
    Serial.printf("\rOTA: %u%%", percent);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("\nOTA: erro %u; reinicie a placa antes de operar o motor\n",
                  (unsigned int)error);
  });
  ArduinoOTA.begin();
  web_enabled_ = true;
}

bool NetworkServices::activateAccessPoint() {
  const uint8_t channel = AP_CHANNELS[(config_.unit_number - 1U) % 3U];
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(ap_ssid_, config_.ap_password, channel)) {
    Serial.println("OTA/WEB: falha ao criar ponto de acesso");
    return false;
  }
  if (!WiFi.softAPbandwidth(WIFI_BW_HT20) ||
      esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    Serial.println("OTA/WEB: falha ao fixar canal e largura WiFi");
    WiFi.softAPdisconnect(true);
    return false;
  }

  uint8_t actual_channel = 0;
  wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
  wifi_bandwidth_t bandwidth = WIFI_BW_HT20;
  if (esp_wifi_get_channel(&actual_channel, &secondary) != ESP_OK ||
      esp_wifi_get_bandwidth(WIFI_IF_AP, &bandwidth) != ESP_OK ||
      actual_channel != channel || secondary != WIFI_SECOND_CHAN_NONE ||
      bandwidth != WIFI_BW_HT20) {
    Serial.println("OTA/WEB: configuracao WiFi divergente");
    WiFi.softAPdisconnect(true);
    return false;
  }

  configureOta();
  Serial.printf("OTA/WEB AP: SSID=%s canal=%u largura=20MHz IP=%s OTA=3232 WEB=80\n",
                ap_ssid_, actual_channel, WiFi.softAPIP().toString().c_str());
  return true;
}

bool NetworkServices::begin() {
  pinMode(config_.button_pin, INPUT_PULLUP);
  snprintf(ap_ssid_, sizeof(ap_ssid_), "Motor-Control-%02u", config_.unit_number);
  snprintf(hostname_, sizeof(hostname_), "as5600-motor-%02u", config_.unit_number);
  WiFi.mode(WIFI_OFF);

  if (config_.sta_ssid && strlen(config_.sta_ssid) > 0) {
    Serial.printf("WiFi: conectando a %s", config_.sta_ssid);
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(hostname_);
    WiFi.begin(config_.sta_ssid, config_.sta_password);
    const uint32_t started_ms = millis();
    while (WiFi.status() != WL_CONNECTED &&
           millis() - started_ms < STATION_TIMEOUT_MS) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      configureOta();
      Serial.printf("OTA/WEB STA: SSID=%s IP=%s OTA=3232 WEB=80\n",
                    WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
      return true;
    }
    Serial.println("WiFi: rede nao encontrada; iniciando AP de contingencia");
    WiFi.disconnect(true);
    delay(100);
  } else {
    Serial.println("WiFi: credenciais STA ausentes; iniciando AP de contingencia");
  }
  return activateAccessPoint();
}

void NetworkServices::update(uint32_t now_ms) {
  if (web_enabled_) ArduinoOTA.handle();

  const bool pressed = digitalRead(config_.button_pin) == LOW;
  if (!pressed) {
    button_pressed_ = false;
    return;
  }
  if (!button_pressed_) {
    button_pressed_ = true;
    button_pressed_ms_ = now_ms;
    return;
  }
  if (now_ms - button_pressed_ms_ < BUTTON_HOLD_MS) return;

  button_pressed_ = false;
  Serial.println("OTA/WEB: ativando AP; controle do motor permanece disponivel");
  if (!activateAccessPoint()) Serial.println("OTA/WEB: falha ao ativar AP");
}
