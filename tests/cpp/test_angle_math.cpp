#include <cmath>
#include <cstdlib>
#include <iostream>

#include <AngleMath.h>

namespace {

void expectNear(float expected, float actual, const char* message) {
  if (std::fabs(expected - actual) <= 0.001f) return;
  std::cerr << message << ": expected " << expected << ", got " << actual
            << '\n';
  std::exit(1);
}

}  // namespace

int main() {
  expectNear(10.0f, AngleMath::normalize(370.0f),
             "normalize positive wrap");
  expectNear(350.0f, AngleMath::normalize(-10.0f),
             "normalize negative wrap");
  expectNear(0.0f, AngleMath::normalize(360.0f),
             "normalize full turn");

  expectNear(20.0f, AngleMath::shortestDelta(350.0f, 10.0f),
             "shortest delta clockwise across zero");
  expectNear(-20.0f, AngleMath::shortestDelta(10.0f, 350.0f),
             "shortest delta counter-clockwise across zero");

  expectNear(340.0f,
             AngleMath::directedDelta(10.0f, 350.0f,
                                      MotionDirection::Clockwise),
             "forced clockwise delta");
  expectNear(-340.0f,
             AngleMath::directedDelta(350.0f, 10.0f,
                                      MotionDirection::CounterClockwise),
             "forced counter-clockwise delta");

  expectNear(370.0f, AngleMath::unwrap(350.0f, 10.0f),
             "unwrap increasing across zero");
  expectNear(-10.0f, AngleMath::unwrap(10.0f, 350.0f),
             "unwrap decreasing across zero");

  std::cout << "angle math tests passed\n";
  return 0;
}
