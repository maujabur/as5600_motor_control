#pragma once

#include <AdrcPositionController.h>
#include <HBridgeMotorDriver.h>
#include <MotionTypes.h>

#include <math.h>
#include <stdint.h>

constexpr uint32_t DEVICE_SETTINGS_VERSION = 2;

struct SensorSettings {
  uint8_t failure_limit = 3;
};

struct DeviceSettings {
  uint32_t version = DEVICE_SETTINGS_VERSION;
  AdrcPositionSettings position;
  MotionSequenceConfig sequence;
  MotorDriverSettings motor;
  SensorSettings sensor;
  bool running = false;

  static DeviceSettings defaults() {
    DeviceSettings value;
    value.position.control_bandwidth = 25.0f;
    value.position.observer_bandwidth = 80.0f;
    value.position.plant_gain = 250.0f;
    value.position.max_target_rpm = 2.4f;
    value.position.physical_max_rpm = 3.0f;
    value.position.stop_window_deg = 1.0f;
    value.position.accel_ramp_ms = 250;
    value.position.decel_ramp_ms = 220;
    value.position.kick_pwm_percent = 85.0f;
    value.position.kick_ms = 180;
    value.position.samples_to_stop = 3;
    value.position.velocity_window_ms = 400;
    value.position.velocity_num_samples = 8;
    value.position.minimum_drive_pwm_percent = 24.0f;
    value.position.stall_timeout_ms = 1500;
    value.position.stall_velocity_deg_s = 2.0f;

    value.sequence.startup_step = {
      0.0f, 1.0f, 1000, MotionDirection::Shortest};
    value.sequence.step_count = 2;
    value.sequence.steps[0] = {
      180.0f, 1.0f, 2000, MotionDirection::Clockwise};
    value.sequence.steps[1] = {
      0.0f, 1.0f, 1000, MotionDirection::CounterClockwise};

    value.motor.pwm_frequency_hz = 500;
    value.motor.pwm_resolution_bits = 8;
    value.motor.power_limit_percent = 100;
    value.sensor.failure_limit = 3;
    value.running = false;
    return value;
  }
};

inline bool validMotionStep(const MotionStep& step, float max_rpm) {
  return isfinite(step.target_deg) && step.target_deg >= 0.0f &&
         step.target_deg < 360.0f && isfinite(step.rpm) &&
         step.rpm >= 0.1f && step.rpm <= max_rpm &&
         step.dwell_ms <= 3600000UL;
}

inline bool validateDeviceSettings(const DeviceSettings& value) {
  const AdrcPositionSettings& p = value.position;
  if (value.version != DEVICE_SETTINGS_VERSION ||
      !isfinite(p.control_bandwidth) || p.control_bandwidth < 1.0f ||
      p.control_bandwidth > 100.0f || !isfinite(p.observer_bandwidth) ||
      p.observer_bandwidth < 1.0f || p.observer_bandwidth > 300.0f ||
      !isfinite(p.plant_gain) || p.plant_gain < 1.0f || p.plant_gain > 2000.0f ||
      !isfinite(p.max_target_rpm) || p.max_target_rpm < 0.1f ||
      p.max_target_rpm > 10.0f || !isfinite(p.physical_max_rpm) ||
      p.physical_max_rpm < 0.1f || p.physical_max_rpm > 10.0f ||
      p.max_target_rpm > p.physical_max_rpm ||
      !isfinite(p.stop_window_deg) || p.stop_window_deg < 0.2f ||
      p.stop_window_deg > 20.0f || p.accel_ramp_ms < 50 ||
      p.accel_ramp_ms > 2000 || p.decel_ramp_ms < 50 ||
      p.decel_ramp_ms > 2000 || !isfinite(p.kick_pwm_percent) ||
      p.kick_pwm_percent < 0.0f || p.kick_pwm_percent > 100.0f ||
      p.kick_ms > 1000 || p.samples_to_stop < 1 || p.samples_to_stop > 20 ||
      !isfinite(p.minimum_drive_pwm_percent) ||
      p.minimum_drive_pwm_percent < 0.0f ||
      p.minimum_drive_pwm_percent > 45.0f || p.stall_timeout_ms < 100 ||
      p.stall_timeout_ms > 10000 || !isfinite(p.stall_velocity_deg_s) ||
      p.stall_velocity_deg_s < 0.1f || p.stall_velocity_deg_s > 20.0f ||
      p.velocity_window_ms < 20 || p.velocity_window_ms > 1000 ||
      p.velocity_num_samples < 2 || p.velocity_num_samples > 20) {
    return false;
  }

  if (value.sequence.step_count < 1 ||
      value.sequence.step_count > MOTION_SEQUENCE_MAX_STEPS ||
      !validMotionStep(value.sequence.startup_step, p.max_target_rpm)) {
    return false;
  }
  for (uint8_t i = 0; i < value.sequence.step_count; ++i) {
    if (!validMotionStep(value.sequence.steps[i], p.max_target_rpm)) return false;
  }

  return value.motor.pwm_frequency_hz >= 500 &&
         value.motor.pwm_frequency_hz <= 20000 &&
         value.motor.pwm_resolution_bits == 8 &&
         value.motor.power_limit_percent <= 100 &&
         value.sensor.failure_limit >= 1 && value.sensor.failure_limit <= 20;
}
