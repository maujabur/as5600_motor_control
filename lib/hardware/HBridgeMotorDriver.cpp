#include "HBridgeMotorDriver.h"

#include <math.h>

namespace {

float clampPercent(float value) {
  if (value < -100.0f) return -100.0f;
  if (value > 100.0f) return 100.0f;
  return value;
}

}  // namespace

HBridgeMotorDriver::HBridgeMotorDriver(const MotorDriverPins& pins)
    : pins_(pins) {}

bool HBridgeMotorDriver::begin(const MotorDriverSettings& settings) {
  pinMode(pins_.a_in1, OUTPUT);
  pinMode(pins_.a_in2, OUTPUT);
  pinMode(pins_.b_in1, OUTPUT);
  pinMode(pins_.b_in2, OUTPUT);
  initialized_ = true;
  if (!setSettings(settings)) {
    initialized_ = false;
    return false;
  }
  stop();
  return true;
}

bool HBridgeMotorDriver::setSettings(const MotorDriverSettings& settings) {
  if (settings.pwm_frequency_hz < 500 ||
      settings.pwm_frequency_hz > 20000 ||
      settings.pwm_resolution_bits < 1 ||
      settings.pwm_resolution_bits > 16 ||
      settings.power_limit_percent > 100) {
    return false;
  }
  settings_ = settings;
  if (initialized_) {
    configurePin(pins_.a_in1);
    configurePin(pins_.a_in2);
    configurePin(pins_.b_in1);
    configurePin(pins_.b_in2);
  }
  return true;
}

void HBridgeMotorDriver::configurePin(uint8_t pin) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  analogWriteFrequency(pin, settings_.pwm_frequency_hz);
  analogWriteResolution(pin, settings_.pwm_resolution_bits);
#else
  (void)pin;
  analogWriteFrequency(settings_.pwm_frequency_hz);
  analogWriteResolution(settings_.pwm_resolution_bits);
#endif
}

uint32_t HBridgeMotorDriver::maxDuty() const {
  return (1UL << settings_.pwm_resolution_bits) - 1UL;
}

void HBridgeMotorDriver::writeChannel(int16_t signed_duty, uint8_t in1,
                                      uint8_t in2) {
  const uint32_t duty = (uint32_t)abs(signed_duty);
  if (signed_duty > 0) {
    analogWrite(in1, duty);
    analogWrite(in2, 0);
  } else if (signed_duty < 0) {
    analogWrite(in1, 0);
    analogWrite(in2, duty);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

void HBridgeMotorDriver::writeSignedPercent(float percent) {
  const float limited = clampPercent(percent) *
                        ((float)settings_.power_limit_percent / 100.0f);
  last_applied_percent_ = (int16_t)lroundf(limited);
  const int16_t signed_duty = (int16_t)lroundf(
      limited * (float)maxDuty() / 100.0f);
  writeChannel(signed_duty, pins_.a_in1, pins_.a_in2);
  writeChannel(signed_duty, pins_.b_in1, pins_.b_in2);
}

void HBridgeMotorDriver::brake() {
  const uint32_t duty = maxDuty();
  analogWrite(pins_.a_in1, duty);
  analogWrite(pins_.a_in2, duty);
  analogWrite(pins_.b_in1, duty);
  analogWrite(pins_.b_in2, duty);
  last_applied_percent_ = 0;
}

void HBridgeMotorDriver::stop() {
  writeChannel(0, pins_.a_in1, pins_.a_in2);
  writeChannel(0, pins_.b_in1, pins_.b_in2);
  last_applied_percent_ = 0;
}
