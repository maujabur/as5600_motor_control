#include "CascadePositionController.h"

#include <math.h>

void CascadePositionController::setSettings(const CascadePositionSettings& settings) {
  settings_ = settings;
  pid_position_.setSettings(settings_.pos_pid);
  pid_velocity_.setSettings(settings_.vel_pid);
  velocity_estimator_ = VelocityEstimator(settings_.velocity_window_ms,
                                          settings_.velocity_num_samples);
}

void CascadePositionController::startMove(float target_deg, float max_speed_rpm, MoveDirection direction) {
  target_accumulated_deg_ = target_deg;  // Armazena alvo sem normalizacao (permite >360)
  target_deg_ = normalize360(target_deg);  // Normaliza para status/debug
  const float effective_limit = fminf(settings_.max_target_rpm, settings_.physical_max_rpm);
  max_speed_rpm_ = constrain(max_speed_rpm, 0.0f, effective_limit);
  direction_ = direction;
  active_ = true;
  last_error_pos_deg_ = 0.0f;
  last_error_rpm_ = 0.0f;
  commanded_rpm_ = 0.0f;
  samples_in_window_ = 0;
  last_pwm_output_ = 0;

  pid_position_.reset();
  pid_velocity_.reset();
  velocity_estimator_.reset();

  phase_ = MovePhase::KICK;
  phase_start_ms_ = millis();
  
  // Inicializa rastreamento de ângulo acumulado (será atualizado no primeiro computeOutputPercent)
  current_accumulated_deg_ = 0.0f;
  accumulated_angle_initialized_ = false;
  last_current_deg_normalized_ = 0.0f;
}

void CascadePositionController::cancel() {
  active_ = false;
  max_speed_rpm_ = 0.0f;
  last_error_pos_deg_ = 0.0f;
  last_error_rpm_ = 0.0f;
  commanded_rpm_ = 0.0f;
  samples_in_window_ = 0;
  last_pwm_output_ = 0;
  phase_ = MovePhase::KICK;

  pid_position_.reset();
  pid_velocity_.reset();
  velocity_estimator_.reset();

  current_accumulated_deg_ = 0.0f;
  accumulated_angle_initialized_ = false;
  last_current_deg_normalized_ = 0.0f;
}

float CascadePositionController::computeOutputPercent(float current_deg, uint32_t now_ms) {
  if (!active_) return 0.0f;

  // ── Estima velocidade real a partir de posição ────────────────────────
  velocity_estimator_.update(current_deg, now_ms);
  const float measured_rpm_control = velocity_estimator_.getRawRpm();

  // ── Rastreia ângulo acumulado (odômetro) para suportar múltiplas voltas ────
  if (!accumulated_angle_initialized_) {
    // Primeiro call: inicializa com posição atual
    current_accumulated_deg_ = current_deg;
    last_current_deg_normalized_ = current_deg;
    accumulated_angle_initialized_ = true;
  } else {
    // Atualiza acumulado rastreando wraps de 360->0
    float delta = current_deg - last_current_deg_normalized_;
    
    // Detecta e corrige wrap-around
    if (delta < -180.0f) {
      // Wrapped para frente (360 -> 0)
      delta += 360.0f;
    } else if (delta > 180.0f) {
      // Wrapped para trás (0 -> 360)
      delta -= 360.0f;
    }
    
    current_accumulated_deg_ += delta;
    last_current_deg_normalized_ = current_deg;
  }

  // ── Malha de posição: PID_pos calcula RPM_cmd desejado ────────────────
  last_error_pos_deg_ = target_accumulated_deg_ - current_accumulated_deg_;
  
  // Verifica chegada
  if (fabsf(last_error_pos_deg_) <= settings_.stop_window_deg) {
    samples_in_window_++;
    if (samples_in_window_ >= settings_.samples_to_stop) {
      active_ = false;
      commanded_rpm_ = 0.0f;
      last_pwm_output_ = 0;
      return 0.0f;
    }
  } else {
    samples_in_window_ = 0;
  }

  // ── Máquina de fases para rampa trapezoidal ──────────────────────────
  float rpm_target_from_ramp = 0.0f;
  const float kick_sign = (last_error_pos_deg_ >= 0.0f) ? 1.0f : -1.0f;

  switch (phase_) {
    // ── KICK ──────────────────────────────────────────────────────────
    case MovePhase::KICK: {
      const uint32_t elapsed = now_ms - phase_start_ms_;
      if (elapsed >= settings_.kick_ms) {
        phase_ = MovePhase::ACCEL;
        phase_start_ms_ = now_ms;
      } else {
        float kick_pwm = constrain(settings_.kick_pwm_percent, 0.0f, 100.0f);
        // Evita que kick em PWM muito alto cause overspeed grande quando o alvo de RPM eh baixo.
        if (settings_.physical_max_rpm > 0.1f && max_speed_rpm_ > 0.0f) {
          const float kick_pwm_limit = constrain((max_speed_rpm_ / settings_.physical_max_rpm) * 135.0f,
                                                8.0f, 100.0f);
          kick_pwm = fminf(kick_pwm, kick_pwm_limit);
        }
        commanded_rpm_ = 0.0f;
        last_error_rpm_ = 0.0f;
        pid_velocity_.reset();
        const float out_kick = kick_sign * kick_pwm;
        last_pwm_output_ = (int)out_kick;
        return out_kick;
      }
      break;
    }

    // ── ACCEL ─────────────────────────────────────────────────────────
    case MovePhase::ACCEL: {
      const uint32_t elapsed = now_ms - phase_start_ms_;
      const float progress = (float)elapsed / (float)settings_.accel_ramp_ms;

      if (progress >= 1.0f) {
        phase_ = MovePhase::CRUISE;
        phase_start_ms_ = now_ms;
        rpm_target_from_ramp = (last_error_pos_deg_ >= 0.0f)
                                   ? max_speed_rpm_
                                   : -max_speed_rpm_;
      } else {
        rpm_target_from_ramp = max_speed_rpm_ * progress;
        if (last_error_pos_deg_ < 0.0f) rpm_target_from_ramp = -rpm_target_from_ramp;
      }
      break;
    }

    // ── CRUISE ────────────────────────────────────────────────────────
    case MovePhase::CRUISE: {
      const float decel_distance = max_speed_rpm_ * (float)settings_.decel_ramp_ms / 2000.0f * 60.0f;
      if (fabsf(last_error_pos_deg_) <= decel_distance) {
        phase_ = MovePhase::DECEL;
        phase_start_ms_ = now_ms;
      }
      rpm_target_from_ramp = (last_error_pos_deg_ >= 0.0f)
                                 ? max_speed_rpm_
                                 : -max_speed_rpm_;
      break;
    }

    // ── DECEL ─────────────────────────────────────────────────────────
    case MovePhase::DECEL: {
      const uint32_t elapsed = now_ms - phase_start_ms_;
      const float progress = (float)elapsed / (float)settings_.decel_ramp_ms;

      if (progress >= 1.0f) {
        phase_ = MovePhase::STOPPING;
        phase_start_ms_ = now_ms;
        // Reseta malha de velocidade ao entrar na fase final para reduzir caca
        // em baixa velocidade devido a integral acumulada em CRUISE/DECEL.
        pid_velocity_.reset();
        rpm_target_from_ramp = 0.0f;
      } else {
        const float rpm_remaining = max_speed_rpm_ * (1.0f - progress);
        rpm_target_from_ramp = (last_error_pos_deg_ >= 0.0f) ? rpm_remaining : -rpm_remaining;
      }
      break;
    }

    // ── STOPPING ──────────────────────────────────────────────────────
    case MovePhase::STOPPING: {
      // PID de posição gera comando para os últimos graus
      rpm_target_from_ramp = pid_position_.compute(last_error_pos_deg_, now_ms);
      // Mantem torque util no fechamento para vencer a zona morta do motor lento.
      float kick_equiv_rpm = max_speed_rpm_;
      if (settings_.physical_max_rpm > 0.1f) {
        kick_equiv_rpm = settings_.physical_max_rpm * constrain(settings_.kick_pwm_percent, 0.0f, 100.0f) / 100.0f;
      }
      const float stop_max_rpm = fminf(fmaxf(max_speed_rpm_ * 0.45f, kick_equiv_rpm * 0.6f), 2.5f);
      const float stop_min_rpm = fminf(stop_max_rpm, fmaxf(kick_equiv_rpm * 0.5f, 0.45f));
      rpm_target_from_ramp = constrain(rpm_target_from_ramp, -stop_max_rpm, stop_max_rpm);
      if (fabsf(last_error_pos_deg_) > settings_.stop_window_deg &&
          fabsf(measured_rpm_control) < 0.15f &&
          fabsf(rpm_target_from_ramp) < stop_min_rpm) {
        rpm_target_from_ramp = (last_error_pos_deg_ >= 0.0f) ? stop_min_rpm : -stop_min_rpm;
      }
      break;
    }
  }

  commanded_rpm_ = rpm_target_from_ramp;

  // Limite estrito por movimento: nunca comanda acima de max_speed_rpm_.
  commanded_rpm_ = constrain(commanded_rpm_, -max_speed_rpm_, max_speed_rpm_);

  // ── Malha de velocidade: feedforward + PID_vel para manter RPM_measured ≈ RPM_cmd
  float pwm_feedforward = 0.0f;
  if (settings_.physical_max_rpm > 0.1f) {
    const float ff_base = (commanded_rpm_ / settings_.physical_max_rpm) * 100.0f;
    if (phase_ == MovePhase::STOPPING) {
      // No fechamento final, reduz menos o feedforward para nao cair abaixo do torque minimo.
      const float ff_taper = 0.45f + 0.55f *
        constrain(fabsf(last_error_pos_deg_) / (settings_.stop_window_deg * 3.0f), 0.0f, 1.0f);
      pwm_feedforward = ff_base * ff_taper;
    } else {
      pwm_feedforward = ff_base;
    }
  }

  last_error_rpm_ = commanded_rpm_ - measured_rpm_control;
  float pwm_phase_scale = 1.0f;
  switch (phase_) {
    case MovePhase::CRUISE:
      pwm_phase_scale = 1.0f;
      break;
    case MovePhase::DECEL:
      pwm_phase_scale = 0.65f;
      break;
    case MovePhase::STOPPING:
      pwm_phase_scale = 0.75f;
      break;
    case MovePhase::KICK:
    case MovePhase::ACCEL:
    default:
      pwm_phase_scale = 0.85f;
      break;
  }

  const float pwm_pid_correction = pid_velocity_.compute(last_error_rpm_, now_ms) * pwm_phase_scale;
  float pwm_percent = pwm_feedforward + pwm_pid_correction;

  // Anti-stall em baixa rotacao: garante torque minimo quando a velocidade cai perto de zero.
  if ((phase_ == MovePhase::ACCEL || phase_ == MovePhase::CRUISE) &&
      fabsf(commanded_rpm_) >= 0.35f &&
      fabsf(measured_rpm_control) < 0.08f) {
    const float base_min_pwm = (settings_.physical_max_rpm > 0.1f)
                                 ? (fabsf(commanded_rpm_) * 100.0f / settings_.physical_max_rpm)
                                 : 18.0f;
    const float min_pwm = constrain(base_min_pwm + 8.0f, 18.0f, 45.0f);
    if (fabsf(pwm_percent) < min_pwm) {
      pwm_percent = (commanded_rpm_ >= 0.0f) ? min_pwm : -min_pwm;
    }
  }
  
  // Clamp final
  pwm_percent = constrain(pwm_percent, -100.0f, 100.0f);
  last_pwm_output_ = (int)pwm_percent;

  return pwm_percent;
}

float CascadePositionController::normalize360(float deg) {
  float value = fmodf(deg, 360.0f);
  if (value < 0.0f) value += 360.0f;
  return value;
}

float CascadePositionController::shortestErrorDeg(float current_deg, float target_deg) {
  float c = normalize360(current_deg);
  float t = normalize360(target_deg);

  float error = t - c;
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;
  return error;
}

float CascadePositionController::directedErrorDeg(float current_deg, float target_deg, MoveDirection direction) {
  const float c = normalize360(current_deg);  // Posicao atual normalizada

  switch (direction) {
    case MoveDirection::Clockwise: {
      // CW: calcula diferenca considerando voltas completas
      // Se target > 360, permite multiplas voltas
      float error = target_deg - c;
      // Se o erro for muito negativo, add 360 (proxima volta)
      while (error < -180.0f) error += 360.0f;
      // Se o erro for muito positivo mas target nao permite, normaliza
      while (error > 180.0f && target_deg <= 360.0f) error -= 360.0f;
      return error;
    }
    case MoveDirection::CounterClockwise: {
      // CCW: similar mas invertido
      float error = c - target_deg;
      while (error < -180.0f) error += 360.0f;
      while (error > 180.0f && target_deg >= 0.0f) error -= 360.0f;
      return -error;
    }
    case MoveDirection::Shortest:
    default: {
      float t = normalize360(target_deg);
      float error = t - c;
      while (error > 180.0f) error -= 360.0f;
      while (error < -180.0f) error += 360.0f;
      return error;
    }
  }
}
