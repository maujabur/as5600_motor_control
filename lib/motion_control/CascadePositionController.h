#pragma once

#include <Arduino.h>
#include "SimplePID.h"
#include "VelocityEstimator.h"

struct CascadePositionSettings {
  // PID de posição
  PIDSettings pos_pid;
  
  // PID de velocidade
  PIDSettings vel_pid;
  
  // Configuração de movimento
  float max_target_rpm = 18.0f;
  float physical_max_rpm = 9.0f;
  float stop_window_deg = 1.0f;
  
  // Rampa trapezoidal em RPM
  uint16_t accel_ramp_ms = 200;
  uint16_t decel_ramp_ms = 200;
  
  // Kick em PWM (%) para vencer inercia/atrito inicial
  float kick_pwm_percent = 85.0f;
  uint16_t kick_ms = 100;
  
  // Histerese
  uint16_t samples_to_stop = 5;
  
  // Estimador de velocidade
  uint16_t velocity_window_ms = 100;
  uint8_t velocity_num_samples = 5;
};

class CascadePositionController {
 public:
  enum class MoveDirection { Shortest, Clockwise, CounterClockwise };

  CascadePositionController() = default;

  void setSettings(const CascadePositionSettings& settings);
  const CascadePositionSettings& settings() const { return settings_; }

  void startMove(float target_deg, float max_speed_rpm, MoveDirection direction = MoveDirection::Shortest);
  void cancel();

  bool isActive() const { return active_; }
  bool isKicking() const { return active_ && phase_ == MovePhase::KICK; }
    bool isCruising() const { return active_ && (phase_ == MovePhase::CRUISE || phase_ == MovePhase::DECEL); }
  float targetDeg() const { return target_deg_; }
  float maxSpeedRpm() const { return max_speed_rpm_; }
  float lastErrorDeg() const { return last_error_pos_deg_; }
  float commandedRpm() const { return commanded_rpm_; }
  float measuredRpm() const { return velocity_estimator_.getLastRpm(); }
  float measuredRpmRaw() const { return velocity_estimator_.getRawRpm(); }
  float lastVelocityError() const { return last_error_rpm_; }
  int pwmOutput() const { return last_pwm_output_; }
  float accumulatedDeg() const { return current_accumulated_deg_; }
  void primeAccumulatedAngle(float current_deg);

  // Computa saída PWM com cascata PID
  float computeOutputPercent(float current_deg, uint32_t now_ms);

 private:
  enum class MovePhase { KICK, ACCEL, CRUISE, DECEL, STOPPING };

  static float normalize360(float deg);
  static float shortestErrorDeg(float current_deg, float target_deg);
  static float directedErrorDeg(float current_deg, float target_deg, MoveDirection direction);

  CascadePositionSettings settings_;
  
  SimplePID pid_position_;
  SimplePID pid_velocity_;
  VelocityEstimator velocity_estimator_{100, 5};

  bool active_ = false;
  float target_deg_ = 0.0f;
  float target_accumulated_deg_ = 0.0f;  // Alvo sem normalizacao (permite >360)
  float current_accumulated_deg_ = 0.0f;  // Rastreamento de ângulo acumulado (odômetro)
  bool accumulated_angle_initialized_ = false;
  float last_current_deg_normalized_ = 0.0f;  // Última leitura normalizada para detect wrap
  float max_speed_rpm_ = 0.0f;
  MoveDirection direction_ = MoveDirection::Shortest;
  float last_error_pos_deg_ = 0.0f;
  float last_error_rpm_ = 0.0f;
  float commanded_rpm_ = 0.0f;
  int last_pwm_output_ = 0;
  uint32_t last_compute_ms_ = 0;

  // Estado da rampa
  MovePhase phase_ = MovePhase::KICK;
  uint32_t phase_start_ms_ = 0;
  
  // Histerese
  uint16_t samples_in_window_ = 0;
};
