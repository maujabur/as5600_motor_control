#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "AngleSensor.h"
#include "As5600Sensor.h"
#include "Mt6701Sensor.h"

class AngleSensorManager {
 public:
  enum class State { Detecting, Active, Lost };
  static constexpr uint32_t REDETECT_INTERVAL_MS = 1000;

  bool begin(TwoWire& wire, uint8_t sda_pin, uint8_t scl_pin,
             uint32_t clock_hz = 400000);
  void update(uint32_t now_ms);
  bool readAngleDeg(float* angle_deg);
  bool readRawAngle(uint16_t* raw_angle);
  void setFailureLimit(uint8_t limit);
  uint8_t failureLimit() const { return failure_limit_; }
  uint8_t failureCount() const { return failure_count_; }
  bool active() const { return state_ == State::Active && active_sensor_; }
  State state() const { return state_; }
  const char* sensorName() const;
  uint8_t sensorAddress() const;
  bool consumeLostEvent();
  bool consumeRecoveredEvent(float* confirmed_angle_deg);

 private:
  bool detect(uint32_t now_ms);
  bool select(AngleSensor& sensor, uint8_t address, float* angle_deg);
  void recordFailure();

  TwoWire* wire_ = nullptr;
  As5600Sensor as5600_;
  Mt6701Sensor mt6701_;
  AngleSensor* active_sensor_ = nullptr;
  State state_ = State::Detecting;
  uint8_t failure_limit_ = 3;
  uint8_t failure_count_ = 0;
  uint32_t last_detect_attempt_ms_ = 0;
  bool lost_event_ = false;
  bool recovered_event_ = false;
  float recovered_angle_deg_ = 0.0f;
};
