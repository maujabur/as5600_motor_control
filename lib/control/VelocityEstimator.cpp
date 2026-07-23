#include "VelocityEstimator.h"

#include <AngleMath.h>

#include <math.h>

float VelocityEstimator::update(float current_pos_deg, uint32_t now_ms) {
  if (!has_prev_raw_) {
    has_prev_raw_ = true;
    prev_raw_deg_ = current_pos_deg;
    unwrapped_deg_ = current_pos_deg;
  } else {
    unwrapped_deg_ = AngleMath::unwrap(unwrapped_deg_, current_pos_deg);
    prev_raw_deg_ = current_pos_deg;
  }

  // Em loops muito rapidos, varios updates chegam no mesmo millis().
  // Ignorar timestamp repetido evita regressao com dt~0 que derruba RPM para zero.
  if (valid_samples_ > 0 && samples_[valid_samples_ - 1].time_ms == now_ms) {
    return filtered_rpm_;
  }

  // Evita encher o buffer com amostras quase no mesmo instante.
  // Em baixa rotacao isso derruba a base temporal e tende a zerar a regressao.
  // Em baixas rotações, uma cadência mais lenta melhora SNR da estimativa.
  // 20ms evita oversampling com pouca variação angular por amostra.
  if (valid_samples_ > 0 && (now_ms - last_added_sample_ms_) < 20U) {
    return filtered_rpm_;
  }

  // Descarta amostras antigas
  while (valid_samples_ > 0 &&
         (now_ms - samples_[0].time_ms) > (uint32_t)sample_window_ms_) {
    // Shift amostras
    for (uint8_t i = 0; i < valid_samples_ - 1; ++i) {
      samples_[i] = samples_[i + 1];
    }
    valid_samples_--;
  }

  // Adiciona nova amostra
  if (valid_samples_ < num_samples_) {
    samples_[valid_samples_].pos_deg = unwrapped_deg_;
    samples_[valid_samples_].time_ms = now_ms;
    valid_samples_++;
  } else {
    // Substitui amostra mais antiga
    for (uint8_t i = 0; i < valid_samples_ - 1; ++i) {
      samples_[i] = samples_[i + 1];
    }
    samples_[valid_samples_ - 1].pos_deg = unwrapped_deg_;
    samples_[valid_samples_ - 1].time_ms = now_ms;
  }
  last_added_sample_ms_ = now_ms;

  // Calcula RPM usando regressão linear (mais suave que apenas first/last)
  if (valid_samples_ >= 2) {
    // Regressão linear simples: fit velocidade aos últimos 3-5 pontos
    float sum_t = 0.0f, sum_pos = 0.0f, sum_tp = 0.0f, sum_t2 = 0.0f;
    const uint32_t t_ref = samples_[0].time_ms;
    
    for (uint8_t i = 0; i < valid_samples_; ++i) {
      const float t = (float)(samples_[i].time_ms - t_ref) / 1000.0f;  // tempo em segundos
      const float pos = samples_[i].pos_deg;
      sum_t += t;
      sum_pos += pos;
      sum_tp += t * pos;
      sum_t2 += t * t;
    }
    
    const float n = (float)valid_samples_;
    const float denom = n * sum_t2 - sum_t * sum_t;
    
    if (fabsf(denom) > 1e-6f) {
      // Inclinação em graus/segundo
      const float slope_deg_s = (n * sum_tp - sum_t * sum_pos) / denom;
      // Converte para RPM: 360 graus = 1 rotação, 60 segundos = 1 minuto
      last_rpm_ = slope_deg_s / 360.0f * 60.0f;
    } else {
      // Fallback numericamente estavel: usa primeira e ultima amostra.
      const uint32_t dt_ms = samples_[valid_samples_ - 1].time_ms - samples_[0].time_ms;
      if (dt_ms > 0) {
        const float delta_deg = samples_[valid_samples_ - 1].pos_deg - samples_[0].pos_deg;
        last_rpm_ = (delta_deg * 60000.0f) / (360.0f * (float)dt_ms);
      } else {
        last_rpm_ = 0.0f;
      }
    }
  } else {
    last_rpm_ = 0.0f;
  }

  // Suaviza a estimativa para evitar oscilacao audivel causada por quantizacao
  // do encoder e pequenas variacoes de amostragem.
  const float alpha = 0.2f;
  filtered_rpm_ = filtered_rpm_ * (1.0f - alpha) + last_rpm_ * alpha;

  return filtered_rpm_;
}

void VelocityEstimator::reset() {
  valid_samples_ = 0;
  last_rpm_ = 0.0f;
  filtered_rpm_ = 0.0f;
  has_prev_raw_ = false;
  last_added_sample_ms_ = 0;
  prev_raw_deg_ = 0.0f;
  unwrapped_deg_ = 0.0f;
}
