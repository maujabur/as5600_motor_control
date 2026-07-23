#include "MotionSequenceController.h"

#include <AngleMath.h>

MotionSequenceController::MotionSequenceController(MotionExecutor& executor)
    : executor_(executor) {}

void MotionSequenceController::setConfig(
    const MotionSequenceConfig& config) {
  config_ = config;
  if (config_.step_count < 1) config_.step_count = 1;
  if (config_.step_count > MOTION_SEQUENCE_MAX_STEPS) {
    config_.step_count = MOTION_SEQUENCE_MAX_STEPS;
  }
}

const MotionStep& MotionSequenceController::stepAt(uint8_t index) const {
  return index == 0 ? config_.startup_step : config_.steps[index - 1];
}

int8_t MotionSequenceController::moveDirectionSignFromTo(
    uint8_t from_step_index, uint8_t to_step_index) const {
  const float from_target_deg = stepAt(from_step_index).target_deg;
  const MotionStep& to_step = stepAt(to_step_index);
  switch (to_step.direction) {
    case MotionDirection::Clockwise:
      return 1;
    case MotionDirection::CounterClockwise:
      return -1;
    case MotionDirection::Shortest:
    default: {
      const float delta =
          AngleMath::shortestDelta(from_target_deg, to_step.target_deg);
      if (delta > 0.0f) return 1;
      if (delta < 0.0f) return -1;
      return 0;
    }
  }
}

void MotionSequenceController::advanceNextStep() {
  ++next_step_index_;
  if (next_step_index_ > config_.step_count) next_step_index_ = 1;
}

void MotionSequenceController::beginStep(uint8_t index,
                                         bool from_step_known,
                                         uint8_t from_step_index) {
  current_step_index_ = index;
  phase_ = Phase::MOVING;
  current_move_sign_ = from_step_known
                           ? moveDirectionSignFromTo(from_step_index, index)
                           : 0;
  const MotionStep& step = stepAt(index);
  executor_.startMove({step.target_deg, step.rpm, step.direction});
}

void MotionSequenceController::setRunning(bool running, uint32_t now_ms) {
  if (!running) {
    stop();
    return;
  }
  if (running_) return;
  running_ = true;
  phase_started_ms_ = now_ms;
  next_step_index_ = 1;
  current_move_sign_ = 0;
  beginStep(0, false, 0);
}

void MotionSequenceController::stop() {
  running_ = false;
  phase_ = Phase::STOPPED;
  current_step_index_ = 0;
  next_step_index_ = 1;
  current_move_sign_ = 0;
  executor_.cancelMove();
}

void MotionSequenceController::update(uint32_t now_ms) {
  if (!running_) return;
  if (phase_ == Phase::MOVING) {
    const MotionStep& current_step = stepAt(current_step_index_);
    if (current_step_index_ != 0 && current_step.dwell_ms == 0) {
      const int8_t next_move_sign =
          moveDirectionSignFromTo(current_step_index_, next_step_index_);
      const bool reversal = current_move_sign_ != 0 && next_move_sign != 0 &&
                            current_move_sign_ != next_move_sign;
      if (!reversal && executor_.isMoveNearTarget()) {
        const MotionStep& next_step = stepAt(next_step_index_);
        if (executor_.retargetMove(
                {next_step.target_deg, next_step.rpm, next_step.direction})) {
          current_step_index_ = next_step_index_;
          current_move_sign_ = next_move_sign;
          advanceNextStep();
          return;
        }
      }
    }
  }

  if (phase_ == Phase::MOVING && !executor_.isMoveActive()) {
    phase_ = Phase::DWELLING;
    phase_started_ms_ = now_ms;
    return;
  }
  if (phase_ != Phase::DWELLING) return;
  if (now_ms - phase_started_ms_ < stepAt(current_step_index_).dwell_ms) {
    return;
  }

  const uint8_t from_step_index = current_step_index_;
  beginStep(next_step_index_, true, from_step_index);
  advanceNextStep();
}

void MotionSequenceController::resumeAfterPause(uint32_t paused_ms) {
  if (!running_) return;
  phase_started_ms_ += paused_ms;
}

const char* MotionSequenceController::phaseText() const {
  switch (phase_) {
    case Phase::MOVING:
      return "MOVING";
    case Phase::DWELLING:
      return "DWELLING";
    case Phase::STOPPED:
      return "STOPPED";
  }
  return "STOPPED";
}
