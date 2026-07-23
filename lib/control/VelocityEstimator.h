#pragma once

#include <Arduino.h>

class VelocityEstimator {
 public:
  VelocityEstimator(uint16_t sample_window_ms = 100, uint8_t num_samples = 5)
      : sample_window_ms_(sample_window_ms), num_samples_(num_samples) {}

  // Atualiza estimador com nova medição de posição e retorna RPM estimado
  float update(float current_pos_deg, uint32_t now_ms);

  // Reseta histórico
  void reset();

  float getLastRpm() const { return filtered_rpm_; }
  float getRawRpm() const { return last_rpm_; }

 private:
  struct Sample {
    float pos_deg = 0.0f;
    uint32_t time_ms = 0;
  };

  uint16_t sample_window_ms_;
  uint8_t num_samples_;
  Sample samples_[10];  // Máximo 10 amostras
  uint8_t sample_index_ = 0;
  uint8_t valid_samples_ = 0;
  uint32_t last_added_sample_ms_ = 0;
  float last_rpm_ = 0.0f;
  float filtered_rpm_ = 0.0f;
  bool has_prev_raw_ = false;
  float prev_raw_deg_ = 0.0f;
  float unwrapped_deg_ = 0.0f;
};
