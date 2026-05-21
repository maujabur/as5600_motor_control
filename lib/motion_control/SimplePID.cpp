#include "SimplePID.h"

float SimplePID::compute(float error, uint32_t now_ms) {
  // Inicializa tempo na primeira chamada
  if (last_time_ms_ == 0) {
    last_time_ms_ = now_ms;
    last_error_ = error;
    return 0.0f;
  }

  const uint32_t dt_ms = now_ms - last_time_ms_;
  if (dt_ms == 0) return 0.0f;  // Evita divisão por zero
  
  const float dt = (float)dt_ms / 1000.0f;  // Converte para segundos

  // Termo P
  const float p_term = settings_.kp * error;

  // Termo D
  const float derivative = (error - last_error_) / dt;
  const float d_term = settings_.kd * derivative;

  // Termo I com anti-windup e protecao para ki=0
  const float prev_integral = integral_;
  float candidate_integral = integral_;
  if (fabsf(settings_.ki) > 1e-6f) {
    candidate_integral += error * dt;
    const float integral_bound = settings_.integral_max / fabsf(settings_.ki);
    candidate_integral = constrain(candidate_integral, -integral_bound, integral_bound);
  } else {
    candidate_integral = 0.0f;
  }

  float i_term = settings_.ki * candidate_integral;
  float output_unsat = p_term + i_term + d_term;
  float output = constrain(output_unsat, settings_.output_min, settings_.output_max);

  // Se saturou e erro empurra mais para saturacao, nao integra neste ciclo.
  const bool saturated_high = output >= settings_.output_max - 1e-6f;
  const bool saturated_low = output <= settings_.output_min + 1e-6f;
  const bool push_high = saturated_high && error > 0.0f;
  const bool push_low = saturated_low && error < 0.0f;

  if (push_high || push_low) {
    integral_ = prev_integral;
    i_term = settings_.ki * integral_;
    output_unsat = p_term + i_term + d_term;
    output = constrain(output_unsat, settings_.output_min, settings_.output_max);
  } else {
    integral_ = candidate_integral;
  }

  last_error_ = error;
  last_time_ms_ = now_ms;

  return output;
}
