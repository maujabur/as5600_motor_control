#pragma once

#include "MotionTypes.h"

namespace AngleMath {

float normalize(float degrees);
float shortestDelta(float from_deg, float to_deg);
float directedDelta(float from_deg, float to_deg, MotionDirection direction);
float unwrap(float previous_accumulated_deg, float current_normalized_deg);

}  // namespace AngleMath
