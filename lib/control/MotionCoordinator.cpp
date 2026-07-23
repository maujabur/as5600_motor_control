#include "MotionCoordinator.h"

MotionCoordinator::MotionCoordinator(AngleSensorManager& sensor,
                                     AdrcPositionController& servo,
                                     HBridgeMotorDriver& motor)
    : sensor_(sensor), servo_(servo), motor_(motor) {}

bool MotionCoordinator::startMove(const MotionRequest& request) {
  status_.stalled = false;
  servo_.startMove(request.target_deg, request.rpm, request.direction);

  float angle = 0.0f;
  if (!sensor_.active() || !sensor_.readAngleDeg(&angle)) {
    motor_.stop();
    status_.sensor_available = false;
    if (!sensor_.active()) markSensorUnavailable(millis());
    refreshStatus();
    return servo_.isActive();
  }

  servo_.primeAccumulatedAngle(angle);
  status_.angle_deg = angle;
  status_.sensor_available = true;
  refreshStatus();
  return true;
}

bool MotionCoordinator::retargetMove(const MotionRequest& request) {
  const bool accepted = servo_.retargetMove(
      request.target_deg, request.rpm, request.direction);
  refreshStatus();
  return accepted;
}

bool MotionCoordinator::isMoveActive() const {
  return servo_.isActive();
}

bool MotionCoordinator::isMoveNearTarget() const {
  return servo_.isWithinStopWindow();
}

void MotionCoordinator::cancelMove() {
  servo_.cancel();
  motor_.stop();
  status_.paused_for_sensor = false;
  status_.stalled = false;
  sensor_pause_started_ms_ = 0;
  recovered_pause_ms_ = 0;
  refreshStatus();
}

void MotionCoordinator::markSensorUnavailable(uint32_t now_ms) {
  if (!status_.paused_for_sensor) sensor_pause_started_ms_ = now_ms;
  if (servo_.isActive()) status_.paused_for_sensor = true;
  status_.sensor_available = false;
  motor_.stop();
}

void MotionCoordinator::updateSensor(uint32_t now_ms) {
  const AngleSensorManager::State previous_state = sensor_.state();
  sensor_.update(now_ms);
  status_.sensor_available = sensor_.active();

  if (previous_state == AngleSensorManager::State::Detecting &&
      sensor_.active()) {
    Serial.printf("%s detectado no endereco 0x%02X\n",
                  sensor_.sensorName(), sensor_.sensorAddress());
  }

  if (sensor_.consumeLostEvent()) {
    markSensorUnavailable(now_ms);
    Serial.printf("Sensor angular perdido apos %u falhas; PWM bloqueado\n",
                  sensor_.failureLimit());
  }

  float recovered_angle = 0.0f;
  if (sensor_.consumeRecoveredEvent(&recovered_angle)) {
    Serial.printf("%s reconectado em 0x%02X\n",
                  sensor_.sensorName(), sensor_.sensorAddress());
    if (status_.paused_for_sensor && servo_.isActive()) {
      servo_.resumeAtAngle(recovered_angle, now_ms);
      recovered_pause_ms_ = now_ms - sensor_pause_started_ms_;
    }
    status_.angle_deg = recovered_angle;
    status_.sensor_available = true;
    status_.paused_for_sensor = false;
    sensor_pause_started_ms_ = 0;
  }
  refreshStatus();
}

void MotionCoordinator::updateControl(uint32_t now_ms) {
  if (status_.paused_for_sensor) {
    motor_.stop();
    refreshStatus();
    return;
  }

  if (!servo_.isActive()) {
    motor_.stop();
    refreshStatus();
    return;
  }

  float angle = 0.0f;
  if (!sensor_.active() || !sensor_.readAngleDeg(&angle)) {
    motor_.stop();
    status_.sensor_available = false;
    if (!sensor_.active()) markSensorUnavailable(now_ms);
    refreshStatus();
    return;
  }

  status_.angle_deg = angle;
  status_.sensor_available = true;
  const float output = servo_.computeOutputPercent(angle, now_ms);
  if (servo_.isStalled()) {
    motor_.stop();
    status_.stalled = true;
    refreshStatus();
    return;
  }
  motor_.writeSignedPercent(output);
  refreshStatus();
}

uint32_t MotionCoordinator::consumeRecoveredPauseMs() {
  const uint32_t elapsed = recovered_pause_ms_;
  recovered_pause_ms_ = 0;
  return elapsed;
}

void MotionCoordinator::refreshStatus() {
  status_.active = servo_.isActive();
  status_.near_target = servo_.isWithinStopWindow();
  status_.target_deg = servo_.targetDeg();
  status_.commanded_rpm = servo_.commandedRpm();
  status_.measured_rpm = servo_.measuredRpm();
  status_.pwm_percent = motor_.lastAppliedPercent();
  status_.sensor_available = sensor_.active();
}
