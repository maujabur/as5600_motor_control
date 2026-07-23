#include <cmath>
#include <cstdlib>
#include <iostream>

#include <MotionSequenceController.h>

namespace {

void expect(bool condition, const char* message) {
  if (condition) return;
  std::cerr << message << '\n';
  std::exit(1);
}

void expectNear(float expected, float actual, const char* message) {
  expect(std::fabs(expected - actual) <= 0.001f, message);
}

class FakeExecutor final : public MotionExecutor {
 public:
  bool startMove(const MotionRequest& request) override {
    last_request = request;
    active = true;
    cancelled = false;
    return true;
  }

  bool retargetMove(const MotionRequest& request) override {
    last_request = request;
    return retarget_result;
  }

  bool isMoveActive() const override { return active; }
  bool isMoveNearTarget() const override { return near_target; }

  void cancelMove() override {
    active = false;
    cancelled = true;
  }

  MotionRequest last_request;
  bool active = false;
  bool near_target = false;
  bool retarget_result = false;
  bool cancelled = false;
};

MotionSequenceConfig baseConfig() {
  MotionSequenceConfig config;
  config.startup_step = {5.0f, 1.0f, 50, MotionDirection::Shortest};
  config.step_count = 2;
  config.steps[0] = {90.0f, 1.0f, 0, MotionDirection::Clockwise};
  config.steps[1] = {180.0f, 1.0f, 0, MotionDirection::Clockwise};
  return config;
}

void testStartupMoveDwellAndStop() {
  FakeExecutor fake;
  MotionSequenceController controller(fake);
  const MotionSequenceConfig config = baseConfig();
  controller.setConfig(config);
  controller.setRunning(true, 100);

  expect(controller.currentStep() == 0, "startup step index");
  expect(controller.phase() == MotionSequenceController::Phase::MOVING,
         "startup moving phase");
  expectNear(5.0f, fake.last_request.target_deg, "startup target");

  fake.active = false;
  controller.update(200);
  expect(controller.phase() == MotionSequenceController::Phase::DWELLING,
         "startup dwell phase");
  controller.update(200 + config.startup_step.dwell_ms);
  expect(controller.currentStep() == 1, "first cyclic step");

  controller.stop();
  expect(!controller.running(), "controller stopped");
  expect(fake.cancelled, "executor cancelled");
}

void testZeroDwellContinuesWithoutStopping() {
  FakeExecutor fake;
  MotionSequenceController controller(fake);
  MotionSequenceConfig config = baseConfig();
  config.startup_step.dwell_ms = 0;
  controller.setConfig(config);
  controller.setRunning(true, 0);

  fake.active = false;
  controller.update(1);
  controller.update(1);
  expect(controller.currentStep() == 1, "first zero-dwell step");

  fake.active = true;
  fake.near_target = true;
  fake.retarget_result = true;
  controller.update(2);
  expect(controller.currentStep() == 2, "retargeted second step");
  expect(controller.phase() == MotionSequenceController::Phase::MOVING,
         "retarget remains moving");
}

}  // namespace

int main() {
  testStartupMoveDwellAndStop();
  testZeroDwellContinuesWithoutStopping();
  std::cout << "motion sequence tests passed\n";
  return 0;
}
