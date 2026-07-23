#pragma once

#include <stdint.h>

constexpr uint8_t MOTION_SEQUENCE_MAX_STEPS = 16;

enum class MotionDirection : uint8_t {
  Shortest = 0,
  Clockwise = 1,
  CounterClockwise = 2,
};

struct MotionRequest {
  float target_deg = 0.0f;
  float rpm = 1.0f;
  MotionDirection direction = MotionDirection::Shortest;
};

struct MotionStep {
  float target_deg = 0.0f;
  float rpm = 1.0f;
  uint32_t dwell_ms = 0;
  MotionDirection direction = MotionDirection::Shortest;
};

struct MotionSequenceConfig {
  MotionStep startup_step;
  MotionStep steps[MOTION_SEQUENCE_MAX_STEPS];
  uint8_t step_count = 1;
};
