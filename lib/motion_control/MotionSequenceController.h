#pragma once

#include <stdint.h>

constexpr uint8_t MOTION_SEQUENCE_MAX_STEPS = 16;

enum class MotionDirection : uint8_t {
  Shortest = 0,
  Clockwise = 1,
  CounterClockwise = 2
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

class MotionSequenceController {
 public:
  using StartMoveFn = void (*)(float, float, MotionDirection);
  using ContinueMoveFn = bool (*)(float, float, MotionDirection);
  using IsMoveActiveFn = bool (*)();
  using IsMoveNearTargetFn = bool (*)();
  using StopMoveFn = void (*)();

  struct Commands {
    StartMoveFn start_move = nullptr;
    ContinueMoveFn continue_move = nullptr;
    IsMoveActiveFn is_move_active = nullptr;
    IsMoveNearTargetFn is_move_near_target = nullptr;
    StopMoveFn stop_move = nullptr;
  };

  enum class Phase : uint8_t { STOPPED, MOVING, DWELLING };

  explicit MotionSequenceController(const Commands& commands);
  void setConfig(const MotionSequenceConfig& config);
  const MotionSequenceConfig& config() const { return config_; }
  void setRunning(bool running, uint32_t now_ms);
  void stop();
  void update(uint32_t now_ms);
  void resumeAfterPause(uint32_t paused_ms);
  bool running() const { return running_; }
  Phase phase() const { return phase_; }
  uint8_t currentStep() const { return current_step_index_; }
  const char* phaseText() const;

 private:
  const MotionStep& stepAt(uint8_t index) const;
  void beginStep(uint8_t index, bool from_step_known, uint8_t from_step_index);
  int8_t moveDirectionSignFromTo(uint8_t from_step_index,
                                 uint8_t to_step_index) const;
  void advanceNextStep();

  Commands commands_;
  MotionSequenceConfig config_;
  bool running_ = false;
  Phase phase_ = Phase::STOPPED;
  uint8_t current_step_index_ = 0;
  uint8_t next_step_index_ = 1;
  int8_t current_move_sign_ = 0;
  uint32_t phase_started_ms_ = 0;
};
