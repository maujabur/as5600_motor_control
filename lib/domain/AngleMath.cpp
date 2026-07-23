#include "AngleMath.h"

#include <math.h>

namespace AngleMath {

float normalize(float degrees) {
  float value = fmodf(degrees, 360.0f);
  return value < 0.0f ? value + 360.0f : value;
}

float shortestDelta(float from_deg, float to_deg) {
  float delta = normalize(to_deg) - normalize(from_deg);
  if (delta > 180.0f) delta -= 360.0f;
  if (delta < -180.0f) delta += 360.0f;
  return delta;
}

float directedDelta(float from_deg, float to_deg, MotionDirection direction) {
  float delta = normalize(to_deg) - normalize(from_deg);
  if (direction == MotionDirection::Clockwise && delta < 0.0f) {
    delta += 360.0f;
  }
  if (direction == MotionDirection::CounterClockwise && delta > 0.0f) {
    delta -= 360.0f;
  }
  return direction == MotionDirection::Shortest
             ? shortestDelta(from_deg, to_deg)
             : delta;
}

float unwrap(float previous_accumulated_deg, float current_normalized_deg) {
  return previous_accumulated_deg +
         shortestDelta(normalize(previous_accumulated_deg),
                       current_normalized_deg);
}

}  // namespace AngleMath
