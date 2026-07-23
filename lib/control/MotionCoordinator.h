#pragma once

#include <Arduino.h>

#include <AdrcPositionController.h>
#include <AngleSensorManager.h>
#include <HBridgeMotorDriver.h>
#include <MotionExecutor.h>

struct MotionStatus {
  bool active = false;
  bool near_target = false;
  bool stalled = false;
  bool sensor_available = false;
  bool paused_for_sensor = false;
  float angle_deg = 0.0f;
  float target_deg = 0.0f;
  float commanded_rpm = 0.0f;
  float measured_rpm = 0.0f;
  int16_t pwm_percent = 0;
};

class MotionCoordinator final : public MotionExecutor {
 public:
  MotionCoordinator(AngleSensorManager& sensor,
                    AdrcPositionController& servo,
                    HBridgeMotorDriver& motor);

  bool startMove(const MotionRequest& request) override;
  bool retargetMove(const MotionRequest& request) override;
  bool isMoveActive() const override;
  bool isMoveNearTarget() const override;
  void cancelMove() override;

  void updateSensor(uint32_t now_ms);
  void updateControl(uint32_t now_ms);
  uint32_t consumeRecoveredPauseMs();
  const MotionStatus& status() const { return status_; }

 private:
  void markSensorUnavailable(uint32_t now_ms);
  void refreshStatus();

  AngleSensorManager& sensor_;
  AdrcPositionController& servo_;
  HBridgeMotorDriver& motor_;
  MotionStatus status_;
  uint32_t sensor_pause_started_ms_ = 0;
  uint32_t recovered_pause_ms_ = 0;
};
