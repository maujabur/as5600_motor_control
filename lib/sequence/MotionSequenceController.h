#pragma once

#include <stdint.h>

#include <MotionExecutor.h>
#include <MotionTypes.h>

class MotionSequenceController {
 public:
  enum class Phase : uint8_t { STOPPED, MOVING, DWELLING };

  explicit MotionSequenceController(MotionExecutor& executor);

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
  void beginStep(uint8_t index, bool from_step_known,
                 uint8_t from_step_index);
  int8_t moveDirectionSignFromTo(uint8_t from_step_index,
                                 uint8_t to_step_index) const;
  void advanceNextStep();

  MotionExecutor& executor_;
  MotionSequenceConfig config_;
  bool running_ = false;
  Phase phase_ = Phase::STOPPED;
  uint8_t current_step_index_ = 0;
  uint8_t next_step_index_ = 1;
  int8_t current_move_sign_ = 0;
  uint32_t phase_started_ms_ = 0;
};
