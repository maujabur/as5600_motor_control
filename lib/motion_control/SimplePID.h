#pragma once

#include <Arduino.h>

struct PIDSettings {
  float kp = 1.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float integral_max = 100.0f;    // clamp de integral
  float output_min = -100.0f;     // clamp de saída
  float output_max = 100.0f;
};

class SimplePID {
 public:
  SimplePID() = default;
  explicit SimplePID(const PIDSettings& settings) : settings_(settings) {}

  void setSettings(const PIDSettings& settings) { settings_ = settings; }
  const PIDSettings& settings() const { return settings_; }

  void reset() {
    integral_ = 0.0f;
    last_error_ = 0.0f;
    last_time_ms_ = 0;
  }

  float compute(float error, uint32_t now_ms);

  float getIntegral() const { return integral_; }
  float getLastError() const { return last_error_; }

 private:
  PIDSettings settings_;
  float integral_ = 0.0f;
  float last_error_ = 0.0f;
  uint32_t last_time_ms_ = 0;
};
