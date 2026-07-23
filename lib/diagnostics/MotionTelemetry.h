#pragma once

#include <MotionCoordinator.h>

struct MotionTelemetrySummary {
  float peak_rpm_abs = 0.0f;
  float mean_rpm = 0.0f;
  float travelled_deg = 0.0f;
  int16_t peak_pwm_abs = 0;
  uint32_t duration_ms = 0;
};

class MotionTelemetry {
 public:
  void beginMove(const MotionStatus& status, uint32_t now_ms);
  void sample(const MotionStatus& status, uint32_t now_ms);
  MotionTelemetrySummary finishMove(uint32_t now_ms);
  void update(const MotionStatus& status, uint32_t now_ms);

 private:
  bool active_ = false;
  bool previous_angle_valid_ = false;
  float previous_angle_deg_ = 0.0f;
  float rpm_sum_ = 0.0f;
  uint32_t rpm_samples_ = 0;
  uint32_t started_ms_ = 0;
  uint32_t last_log_ms_ = 0;
  MotionTelemetrySummary summary_;
};
