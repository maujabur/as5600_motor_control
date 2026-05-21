#include "PositionServoController.h"

#include <math.h>

void PositionServoController::startMove(float target_deg, float max_speed_rpm) {
  target_deg_ = normalize360(target_deg);
  max_speed_rpm_ = max_speed_rpm;
  active_ = true;
  last_error_deg_ = 0.0f;
  commanded_rpm_ = 0.0f;
  samples_in_window_ = 0;
  
  // Inicia na fase de kick
  phase_ = MovePhase::KICK;
  phase_start_ms_ = millis();
}

void PositionServoController::cancel() {
  active_ = false;
  max_speed_rpm_ = 0.0f;
  last_error_deg_ = 0.0f;
  commanded_rpm_ = 0.0f;
  samples_in_window_ = 0;
  phase_ = MovePhase::KICK;
}

float PositionServoController::computeOutputPercent(float current_deg) {
  if (!active_) return 0.0f;

  const float rated = settings_.rated_max_rpm;
  if (rated <= 0.01f) {
    cancel();
    return 0.0f;
  }

  const uint32_t now = millis();
  const float max_rpm = constrain(max_speed_rpm_, 0.0f, rated);
  last_error_deg_ = shortestErrorDeg(current_deg, target_deg_);

  // ── Máquina de fases ──────────────────────────────────────────────────

  switch (phase_) {
    // ── KICK: Pulso inicial para garantir movimento ──────────────────────
    case MovePhase::KICK: {
      const uint32_t elapsed = now - phase_start_ms_;
      if (elapsed >= settings_.kick_ms) {
        // Termina kick, inicia aceleração
        phase_ = MovePhase::ACCEL;
        phase_start_ms_ = now;
        commanded_rpm_ = 0.0f;
      } else {
        // Aplica kick com sinal do erro
        commanded_rpm_ = (last_error_deg_ >= 0.0f)
                           ? settings_.kick_rpm
                           : -settings_.kick_rpm;
      }
      break;
    }

    // ── ACCEL: Rampa de 0 até vmax ──────────────────────────────────────
    case MovePhase::ACCEL: {
      const uint32_t elapsed = now - phase_start_ms_;
      const float progress = (float)elapsed / (float)settings_.accel_ramp_ms;

      if (progress >= 1.0f) {
        // Aceleração concluída, vai para cruise
        phase_ = MovePhase::CRUISE;
        phase_start_ms_ = now;
        commanded_rpm_ = max_rpm;
      } else {
        // Rampa linear de 0 até max_rpm
        commanded_rpm_ = max_rpm * progress;
        if (last_error_deg_ < 0.0f) commanded_rpm_ = -commanded_rpm_;
      }
      break;
    }

    // ── CRUISE: Velocidade constante ─────────────────────────────────────
    case MovePhase::CRUISE: {
      // Verifica se precisa começar a desacelerar
      // Usamos o erro para calcular quando começar a frear
      // Distância de frenagem = vmax * decel_time / 2
      const float decel_distance = max_rpm * (float)settings_.decel_ramp_ms / 2000.0f * 60.0f;
      
      if (fabsf(last_error_deg_) <= decel_distance) {
        phase_ = MovePhase::DECEL;
        phase_start_ms_ = now;
      } else {
        commanded_rpm_ = (last_error_deg_ >= 0.0f) ? max_rpm : -max_rpm;
      }
      break;
    }

    // ── DECEL: Rampa de vmax até 0 ──────────────────────────────────────
    case MovePhase::DECEL: {
      const uint32_t elapsed = now - phase_start_ms_;
      const float progress = (float)elapsed / (float)settings_.decel_ramp_ms;

      if (progress >= 1.0f) {
        phase_ = MovePhase::STOPPING;
        phase_start_ms_ = now;
        commanded_rpm_ = 0.0f;
      } else {
        // Rampa de max_rpm até 0
        const float rpm_remaining = max_rpm * (1.0f - progress);
        commanded_rpm_ = (last_error_deg_ >= 0.0f) ? rpm_remaining : -rpm_remaining;
      }
      break;
    }

    // ── STOPPING: Verificação de chegada com histerese ───────────────────
    case MovePhase::STOPPING: {
      if (fabsf(last_error_deg_) <= settings_.stop_window_deg) {
        samples_in_window_++;
        
        if (samples_in_window_ >= settings_.samples_to_stop) {
          // Chegou! Desativa e retorna 0
          active_ = false;
          commanded_rpm_ = 0.0f;
          return 0.0f;
        }
        // Dentro da janela mas não o suficiente: comando suave
        const float fine_cmd = fabsf(last_error_deg_) * settings_.kp_rpm_per_deg;
        commanded_rpm_ = constrain(fine_cmd, 0.0f, max_rpm * 0.2f);
      } else {
        // Fora da janela: reset contador e continua com mais força
        samples_in_window_ = 0;
        // Comando P direto, pode usar mais potência se precisar
        const float corrective_cmd = fabsf(last_error_deg_) * settings_.kp_rpm_per_deg;
        // Garante mínimo de movimento mesmo com erro pequeno
        commanded_rpm_ = constrain(corrective_cmd, 1.0f, max_rpm);
      }
      
      if (last_error_deg_ < 0.0f) commanded_rpm_ = -commanded_rpm_;
      break;
    }
  }

  // ── Converte RPM comandado para percentual ────────────────────────────
  float out_pct = (fabsf(commanded_rpm_) / rated) * 100.0f;
  out_pct = constrain(out_pct, 0.0f, 100.0f);

  return (commanded_rpm_ >= 0.0f) ? out_pct : -out_pct;
}

float PositionServoController::normalize360(float deg) {
  float value = fmodf(deg, 360.0f);
  if (value < 0.0f) value += 360.0f;
  return value;
}

float PositionServoController::shortestErrorDeg(float current_deg, float target_deg) {
  float c = normalize360(current_deg);
  float t = normalize360(target_deg);

  float error = t - c;
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;
  return error;
}
