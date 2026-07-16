#include "MotionSequenceController.h"

MotionSequenceController::MotionSequenceController(const Commands& commands)
    : commands_(commands) {}

void MotionSequenceController::setConfig(const MotionSequenceConfig& config) {
  config_ = config;
  if (config_.step_count < 1) config_.step_count = 1;
  if (config_.step_count > MOTION_SEQUENCE_MAX_STEPS) {
    config_.step_count = MOTION_SEQUENCE_MAX_STEPS;
  }
}

const MotionStep& MotionSequenceController::stepAt(uint8_t index) const {
  return index == 0 ? config_.startup_step : config_.steps[index - 1];
}

void MotionSequenceController::beginStep(uint8_t index) {
  current_step_index_ = index;
  phase_ = Phase::MOVING;
  const MotionStep& step = stepAt(index);
  if (commands_.start_move) {
    commands_.start_move(step.target_deg, step.rpm, step.direction);
  }
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
  beginStep(0);
}

void MotionSequenceController::stop() {
  running_ = false;
  phase_ = Phase::STOPPED;
  current_step_index_ = 0;
  next_step_index_ = 1;
  if (commands_.stop_move) commands_.stop_move();
}

void MotionSequenceController::update(uint32_t now_ms) {
  if (!running_ || !commands_.is_move_active) return;
  if (phase_ == Phase::MOVING && !commands_.is_move_active()) {
    phase_ = Phase::DWELLING;
    phase_started_ms_ = now_ms;
    return;
  }
  if (phase_ != Phase::DWELLING) return;
  if (now_ms - phase_started_ms_ < stepAt(current_step_index_).dwell_ms) return;

  beginStep(next_step_index_);
  ++next_step_index_;
  if (next_step_index_ > config_.step_count) next_step_index_ = 1;
}

void MotionSequenceController::resumeAfterPause(uint32_t paused_ms) {
  if (!running_) return;
  phase_started_ms_ += paused_ms;
}

const char* MotionSequenceController::phaseText() const {
  switch (phase_) {
    case Phase::MOVING: return "MOVING";
    case Phase::DWELLING: return "DWELLING";
    case Phase::STOPPED: return "STOPPED";
  }
  return "STOPPED";
}
