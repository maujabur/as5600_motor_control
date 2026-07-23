#include "MotionTelemetry.h"

#include <AngleMath.h>
#include <Arduino.h>
#include <math.h>

namespace {
constexpr uint32_t DIAGNOSTIC_LOG_PERIOD_MS = 2000;
}

void MotionTelemetry::beginMove(
    const MotionStatus& status, uint32_t now_ms) {
  active_ = true;
  previous_angle_valid_ = status.sensor_available;
  previous_angle_deg_ = status.angle_deg;
  rpm_sum_ = 0.0f;
  rpm_samples_ = 0;
  started_ms_ = now_ms;
  last_log_ms_ = now_ms;
  summary_ = MotionTelemetrySummary{};
}

void MotionTelemetry::sample(
    const MotionStatus& status, uint32_t now_ms) {
  if (status.sensor_available) {
    if (previous_angle_valid_) {
      summary_.travelled_deg += fabsf(
        AngleMath::shortestDelta(previous_angle_deg_, status.angle_deg));
    }
    previous_angle_deg_ = status.angle_deg;
    previous_angle_valid_ = true;
  }
  const float rpm_abs = fabsf(status.measured_rpm);
  if (rpm_abs > summary_.peak_rpm_abs) summary_.peak_rpm_abs = rpm_abs;
  rpm_sum_ += rpm_abs;
  ++rpm_samples_;
  const int16_t pwm_abs = (int16_t)abs(status.pwm_percent);
  if (pwm_abs > summary_.peak_pwm_abs) summary_.peak_pwm_abs = pwm_abs;

  if (now_ms - last_log_ms_ >= DIAGNOSTIC_LOG_PERIOD_MS) {
    last_log_ms_ = now_ms;
    Serial.printf("DBG: move t=%u ms rpm=%.2f rpm_cmd=%.2f pwm=%d%% ang=%.2f\n",
                  now_ms - started_ms_, status.measured_rpm,
                  status.commanded_rpm, status.pwm_percent, status.angle_deg);
  }
}

MotionTelemetrySummary MotionTelemetry::finishMove(uint32_t now_ms) {
  summary_.duration_ms = now_ms - started_ms_;
  summary_.mean_rpm = rpm_samples_ > 0
    ? rpm_sum_ / (float)rpm_samples_ : 0.0f;
  active_ = false;
  previous_angle_valid_ = false;
  Serial.printf(
    "OK: movimento concluido (tempo=%u ms, deslocamento=%.2f deg, "
    "rpm_pico=%.2f, rpm_media=%.2f, pwm_pico=%d%%)\n",
    summary_.duration_ms, summary_.travelled_deg, summary_.peak_rpm_abs,
    summary_.mean_rpm, summary_.peak_pwm_abs);
  return summary_;
}

void MotionTelemetry::update(const MotionStatus& status, uint32_t now_ms) {
  if (status.active && !active_) beginMove(status, now_ms);
  if (status.active && active_) sample(status, now_ms);
  if (!status.active && active_ && !status.paused_for_sensor) finishMove(now_ms);
}
