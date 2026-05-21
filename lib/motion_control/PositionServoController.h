#pragma once

#include <Arduino.h>

struct PositionServoSettings {
  float rated_max_rpm = 18.0f;
  float kp_rpm_per_deg = 0.12f;
  float stop_window_deg = 1.0f;
  
  // Rampa trapezoidal de velocidade
  uint16_t accel_ramp_ms = 200;  // tempo para rampar de 0 até vmax
  uint16_t decel_ramp_ms = 200;  // tempo para rampar de vmax até 0
  
  // Kick de posição: pulso inicial para garantir movimento
  float kick_rpm = 8.0f;
  uint16_t kick_ms = 100;
  
  // Histerese na detecção de chegada
  uint16_t samples_to_stop = 5;  // quantas leituras seguidas dentro da janela para parar
};

class PositionServoController {
 public:
  PositionServoController() = default;

  void setSettings(const PositionServoSettings& settings) { settings_ = settings; }
  const PositionServoSettings& settings() const { return settings_; }

  void startMove(float target_deg, float max_speed_rpm);
  void cancel();

  bool isActive() const { return active_; }
  float targetDeg() const { return target_deg_; }
  float maxSpeedRpm() const { return max_speed_rpm_; }
  float lastErrorDeg() const { return last_error_deg_; }
  float commandedRpm() const { return commanded_rpm_; }

  float computeOutputPercent(float current_deg);

 private:
  enum class MovePhase { KICK, ACCEL, CRUISE, DECEL, STOPPING };
  
  static float normalize360(float deg);
  static float shortestErrorDeg(float current_deg, float target_deg);

  PositionServoSettings settings_;
  bool active_ = false;
  float target_deg_ = 0.0f;
  float max_speed_rpm_ = 0.0f;
  float last_error_deg_ = 0.0f;
  float commanded_rpm_ = 0.0f;
  
  // Estado da rampa
  MovePhase phase_ = MovePhase::KICK;
  uint32_t phase_start_ms_ = 0;
  
  // Histerese na chegada
  uint16_t samples_in_window_ = 0;
};
