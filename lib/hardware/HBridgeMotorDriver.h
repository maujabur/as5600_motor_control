#pragma once

#include <Arduino.h>

struct MotorDriverPins {
  uint8_t a_in1;
  uint8_t a_in2;
  uint8_t b_in1;
  uint8_t b_in2;
};

struct MotorDriverSettings {
  uint32_t pwm_frequency_hz = 500;
  uint8_t pwm_resolution_bits = 8;
  uint8_t power_limit_percent = 100;
};

class HBridgeMotorDriver {
 public:
  explicit HBridgeMotorDriver(const MotorDriverPins& pins);

  bool begin(const MotorDriverSettings& settings);
  bool setSettings(const MotorDriverSettings& settings);
  const MotorDriverSettings& settings() const { return settings_; }

  void writeSignedPercent(float percent);
  void brake();
  void stop();

  int16_t lastAppliedPercent() const { return last_applied_percent_; }

 private:
  void configurePin(uint8_t pin);
  void writeChannel(int16_t signed_duty, uint8_t in1, uint8_t in2);
  uint32_t maxDuty() const;

  MotorDriverPins pins_;
  MotorDriverSettings settings_;
  bool initialized_ = false;
  int16_t last_applied_percent_ = 0;
};
