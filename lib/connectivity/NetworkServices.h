#pragma once

#include <Arduino.h>

struct NetworkConfig {
  uint8_t unit_number = 1;
  uint8_t button_pin = 7;
  const char* sta_ssid = "";
  const char* sta_password = "";
  const char* ap_password = "";
  const char* ota_password = "";
};

class NetworkServiceEvents {
 public:
  virtual ~NetworkServiceEvents() = default;
  virtual void onOtaStart() = 0;
};

class NetworkServices {
 public:
  NetworkServices(const NetworkConfig& config, NetworkServiceEvents& events);
  bool begin();
  void update(uint32_t now_ms);
  bool otaBusy() const { return ota_busy_; }
  bool webEnabled() const { return web_enabled_; }

 private:
  bool activateAccessPoint();
  void configureOta();

  NetworkConfig config_;
  NetworkServiceEvents& events_;
  bool ota_busy_ = false;
  bool web_enabled_ = false;
  bool button_pressed_ = false;
  uint32_t button_pressed_ms_ = 0;
  char ap_ssid_[20] = {0};
  char hostname_[28] = {0};
};
