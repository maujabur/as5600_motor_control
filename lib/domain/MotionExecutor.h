#pragma once

#include "MotionTypes.h"

class MotionExecutor {
 public:
  virtual ~MotionExecutor() = default;
  virtual bool startMove(const MotionRequest& request) = 0;
  virtual bool retargetMove(const MotionRequest& request) = 0;
  virtual bool isMoveActive() const = 0;
  virtual bool isMoveNearTarget() const = 0;
  virtual void cancelMove() = 0;
};
