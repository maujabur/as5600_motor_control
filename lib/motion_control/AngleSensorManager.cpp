#include "AngleSensorManager.h"

bool AngleSensorManager::begin(TwoWire& wire, uint8_t sda_pin, uint8_t scl_pin,
                               uint32_t clock_hz) {
  wire_ = &wire;
  wire_->begin(sda_pin, scl_pin);
  wire_->setClock(clock_hz);
  state_ = State::Detecting;
  return detect(millis());
}

void AngleSensorManager::update(uint32_t now_ms) {
  if (!active() &&
      (uint32_t)(now_ms - last_detect_attempt_ms_) >= REDETECT_INTERVAL_MS) {
    detect(now_ms);
  }
}

bool AngleSensorManager::readAngleDeg(float* angle_deg) {
  if (!active_sensor_ || !angle_deg) return false;
  if (active_sensor_->readAngleDeg(angle_deg)) {
    failure_count_ = 0;
    return true;
  }
  recordFailure();
  return false;
}

bool AngleSensorManager::readRawAngle(uint16_t* raw_angle) {
  if (!active_sensor_ || !raw_angle) return false;
  if (active_sensor_->readRawAngle(raw_angle)) {
    failure_count_ = 0;
    return true;
  }
  recordFailure();
  return false;
}

void AngleSensorManager::setFailureLimit(uint8_t limit) {
  failure_limit_ = limit;
}

const char* AngleSensorManager::sensorName() const {
  return active_sensor_ ? active_sensor_->name() : "NONE";
}

uint8_t AngleSensorManager::sensorAddress() const {
  return active_sensor_ ? active_sensor_->address() : 0;
}

bool AngleSensorManager::consumeLostEvent() {
  const bool event = lost_event_;
  lost_event_ = false;
  return event;
}

bool AngleSensorManager::consumeRecoveredEvent(float* confirmed_angle_deg) {
  if (!recovered_event_ || !confirmed_angle_deg) return false;
  *confirmed_angle_deg = recovered_angle_deg_;
  recovered_event_ = false;
  return true;
}

bool AngleSensorManager::detect(uint32_t now_ms) {
  last_detect_attempt_ms_ = now_ms;
  const bool was_lost = state_ == State::Lost;
  float angle_deg = 0.0f;
  if (select(as5600_, As5600Sensor::DEFAULT_I2C_ADDR, &angle_deg) ||
      select(mt6701_, Mt6701Sensor::DEFAULT_I2C_ADDR, &angle_deg) ||
      select(mt6701_, Mt6701Sensor::ALTERNATE_I2C_ADDR, &angle_deg)) {
    state_ = State::Active;
    failure_count_ = 0;
    if (was_lost) {
      recovered_angle_deg_ = angle_deg;
      recovered_event_ = true;
    }
    return true;
  }
  active_sensor_ = nullptr;
  state_ = was_lost ? State::Lost : State::Detecting;
  return false;
}

bool AngleSensorManager::select(AngleSensor& sensor, uint8_t address,
                                float* angle_deg) {
  if (!wire_ || !angle_deg || !sensor.probe(*wire_, address) ||
      !sensor.readAngleDeg(angle_deg)) {
    return false;
  }
  active_sensor_ = &sensor;
  return true;
}

void AngleSensorManager::recordFailure() {
  ++failure_count_;
  if (failure_count_ >= failure_limit_) {
    active_sensor_ = nullptr;
    state_ = State::Lost;
    lost_event_ = true;
    last_detect_attempt_ms_ = millis();
  }
}
