# Modular Firmware Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refatorar o firmware em módulos por competência, preservando o painel web, os contratos HTTP e o comportamento físico, enquanto remove o console serial interativo, o controlador repetitivo legado e a compatibilidade histórica da NVS.

**Architecture:** O domínio angular e os tipos de movimento serão independentes de Arduino. Uma fachada de movimento coordenará sensor, ADRC e ponte H; o sequenciador consumirá essa fachada; aplicação, web, persistência e conectividade dependerão de interfaces estreitas e snapshots tipados. `main.cpp` terminará apenas encaminhando `setup()` e `loop()` para `MotorControlApplication`.

**Tech Stack:** C++17/Arduino, PlatformIO, ESP32-S3, Arduino `Preferences`, `WebServer`, `WiFi`, `ArduinoOTA`, testes Python `unittest`, testes nativos PlatformIO/Unity.

## Global Constraints

- Preservar aparência, fluxo, URLs, métodos e campos JSON consumidos pelas páginas atuais.
- Preservar passo inicial, 1 a 16 passos cíclicos, movimento avulso, continuidade de passos, STOP, stall e recuperação do sensor.
- Manter a ordem lógica: rede/web, sensor, sequência, controle, saída, telemetria.
- Início de OTA, STOP, sensor inválido e stall devem resultar imediatamente em saída zero.
- Não alterar os defaults nem a sintonia ADRC durante a refatoração.
- Não preservar comandos seriais interativos nem o formato NVS anterior.
- Cada tarefa termina com testes e build do ambiente `waveshare_esp32_s3_zero` passando.
- Não adicionar bibliotecas externas além do framework e do Unity fornecido pelo PlatformIO.

## Mapa final de arquivos

```text
lib/domain/
  AngleMath.h/.cpp             matemática circular compartilhada
  MotionTypes.h               direção, solicitação, passo e sequência
  MotionExecutor.h            interface consumida pelo sequenciador
lib/hardware/
  HBridgeMotorDriver.h/.cpp    única escrita em IN1..IN4/PWM
  AngleSensor*.h/.cpp          drivers e redetecção I2C existentes
lib/control/
  VelocityEstimator.h/.cpp     estimativa de velocidade
  AdrcPositionController.h/.cpp controle de posição
  MotionCoordinator.h/.cpp     sensor + ADRC + driver e recuperação
lib/sequence/
  MotionSequenceController.h/.cpp máquina de estados pura
lib/settings/
  DeviceSettings.h            snapshot persistido e validação
  PreferencesSettingsStore.h/.cpp acesso exclusivo à NVS
lib/web/
  WebControlServer.h/.cpp      rotas, parsing e JSON
  *_web_page.h                 HTML existente sem redesenho
lib/connectivity/
  NetworkServices.h/.cpp       STA, AP, botão e OTA
lib/diagnostics/
  MotionTelemetry.h/.cpp       métricas e logs periódicos
src/
  MotorControlApplication.h/.cpp composição e casos de uso
  main.cpp                     encaminhamento setup/loop
tests/native/
  test_angle_math/test_main.cpp
  test_motion_sequence/test_main.cpp
tests/
  test_web_contract.py
  test_application_contract.py
```

---

### Task 1: Domínio angular e tipos compartilhados

**Files:**
- Create: `lib/domain/AngleMath.h`
- Create: `lib/domain/AngleMath.cpp`
- Create: `lib/domain/MotionTypes.h`
- Create: `lib/domain/MotionExecutor.h`
- Create: `tests/native/test_angle_math/test_main.cpp`
- Modify: `platformio.ini`
- Modify: `lib/motion_control/VelocityEstimator.h`
- Modify: `lib/motion_control/VelocityEstimator.cpp`
- Modify: `lib/motion_control/AdrcPositionController.h`
- Modify: `lib/motion_control/AdrcPositionController.cpp`
- Modify: `lib/motion_control/MotionSequenceController.h`
- Modify: `lib/motion_control/MotionSequenceController.cpp`

**Interfaces:**
- Produces: `AngleMath::normalize(float)`, `AngleMath::shortestDelta(float,float)`, `AngleMath::directedDelta(float,float,MotionDirection)`, `AngleMath::unwrap(float,float)`, `MotionRequest`, `MotionStep`, `MotionSequenceConfig`, and `MotionExecutor`.
- Consumes: no project module; only `<stdint.h>` and `<math.h>`.

- [ ] **Step 1: Add the native test environment and failing angle tests**

Append this environment to `platformio.ini`:

```ini
[platformio]
test_dir = tests/native

[env:native]
platform = native
test_framework = unity
build_src_filter = -<*>
```

Add `test_dir` to the existing `[platformio]` section; do not create a second
section with the same name.

Create `tests/native/test_angle_math/test_main.cpp`:

```cpp
#include <unity.h>
#include <AngleMath.h>

void test_normalize_wraps_both_directions() {
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, AngleMath::normalize(370.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 350.0f, AngleMath::normalize(-10.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, AngleMath::normalize(360.0f));
}

void test_shortest_delta_crosses_zero() {
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f,
                           AngleMath::shortestDelta(350.0f, 10.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -20.0f,
                           AngleMath::shortestDelta(10.0f, 350.0f));
}

void test_directed_delta_obeys_requested_direction() {
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 340.0f,
      AngleMath::directedDelta(10.0f, 350.0f, MotionDirection::Clockwise));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -340.0f,
      AngleMath::directedDelta(350.0f, 10.0f,
                               MotionDirection::CounterClockwise));
}

void test_unwrap_preserves_accumulated_turns() {
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 370.0f, AngleMath::unwrap(350.0f, 10.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -10.0f, AngleMath::unwrap(10.0f, 350.0f));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_normalize_wraps_both_directions);
  RUN_TEST(test_shortest_delta_crosses_zero);
  RUN_TEST(test_directed_delta_obeys_requested_direction);
  RUN_TEST(test_unwrap_preserves_accumulated_turns);
  return UNITY_END();
}
```

- [ ] **Step 2: Run the focused native test and verify failure**

Run: `pio test -e native -f test_angle_math`

Expected: FAIL because `AngleMath.h` and `MotionDirection` do not exist.

- [ ] **Step 3: Implement the shared domain interfaces**

Create `lib/domain/MotionTypes.h`:

```cpp
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
```

Create `lib/domain/MotionExecutor.h`:

```cpp
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
```

Create `lib/domain/AngleMath.h`:

```cpp
#pragma once
#include "MotionTypes.h"

namespace AngleMath {
float normalize(float degrees);
float shortestDelta(float from_deg, float to_deg);
float directedDelta(float from_deg, float to_deg, MotionDirection direction);
float unwrap(float previous_accumulated_deg, float current_normalized_deg);
}
```

Create `lib/domain/AngleMath.cpp`:

```cpp
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
  if (direction == MotionDirection::Clockwise && delta < 0.0f) delta += 360.0f;
  if (direction == MotionDirection::CounterClockwise && delta > 0.0f) delta -= 360.0f;
  return direction == MotionDirection::Shortest
      ? shortestDelta(from_deg, to_deg) : delta;
}

float unwrap(float previous_accumulated_deg, float current_normalized_deg) {
  return previous_accumulated_deg + shortestDelta(
      normalize(previous_accumulated_deg), current_normalized_deg);
}
}
```

- [ ] **Step 4: Replace every private angular implementation**

Include `<AngleMath.h>` and `<MotionTypes.h>` from the estimator, ADRC and
sequencer. Replace their local normalization/delta calls with
`AngleMath::normalize`, `AngleMath::shortestDelta`,
`AngleMath::directedDelta`, and `AngleMath::unwrap`. Delete
`AdrcPositionController::normalize360`,
`AdrcPositionController::shortestDelta`,
`VelocityEstimator::shortestAngleDelta`, the anonymous angle functions in
`MotionSequenceController.cpp`, and `shortestAngleDeltaDeg` in `main.cpp`.
Remove the duplicate `MotionDirection`, `MotionStep`, and
`MotionSequenceConfig` definitions from `MotionSequenceController.h`.

- [ ] **Step 5: Run domain tests, Python contracts, and firmware build**

Run: `pio test -e native -f test_angle_math`

Expected: 4 tests PASS.

Run: `python -m unittest discover -s tests -p "test_*.py"`

Expected: all existing contract tests PASS after updating references to the
public `AngleMath` calls.

Run: `pio run -e waveshare_esp32_s3_zero`

Expected: SUCCESS.

- [ ] **Step 6: Commit the domain extraction**

```powershell
git add platformio.ini lib/domain lib/motion_control src/main.cpp tests
git commit -m "refactor: centralize angular motion domain"
```

### Task 2: Pure motion sequencer and removal of the repetitive controller

**Files:**
- Create: `lib/sequence/MotionSequenceController.h`
- Create: `lib/sequence/MotionSequenceController.cpp`
- Create: `tests/native/test_motion_sequence/test_main.cpp`
- Modify: `src/main.cpp`
- Modify: `tests/test_sequence_controller_contract.py`
- Delete: `lib/motion_control/MotionSequenceController.h`
- Delete: `lib/motion_control/MotionSequenceController.cpp`
- Delete: `lib/motion_control/RepetitiveMotionController.h`
- Delete: `lib/motion_control/RepetitiveMotionController.cpp`

**Interfaces:**
- Consumes: `MotionExecutor` and `MotionSequenceConfig` from Task 1.
- Produces: `MotionSequenceController(MotionExecutor&)`, `setConfig`,
  `setRunning`, `stop`, `update`, `resumeAfterPause`, and immutable state accessors.

- [ ] **Step 1: Write a behavior test with a fake executor**

Create `tests/native/test_motion_sequence/test_main.cpp`:

```cpp
#include <unity.h>
#include <MotionSequenceController.h>

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
  void cancelMove() override { active = false; cancelled = true; }

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

void test_startup_move_dwell_and_stop() {
  FakeExecutor fake;
  MotionSequenceController controller(fake);
  const MotionSequenceConfig config = baseConfig();
  controller.setConfig(config);
  controller.setRunning(true, 100);
  TEST_ASSERT_EQUAL_UINT8(0, controller.currentStep());
  TEST_ASSERT_EQUAL(MotionSequenceController::Phase::MOVING, controller.phase());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f, fake.last_request.target_deg);

  fake.active = false;
  controller.update(200);
  TEST_ASSERT_EQUAL(MotionSequenceController::Phase::DWELLING, controller.phase());
  controller.update(200 + config.startup_step.dwell_ms);
  TEST_ASSERT_EQUAL_UINT8(1, controller.currentStep());

  controller.stop();
  TEST_ASSERT_FALSE(controller.running());
  TEST_ASSERT_TRUE(fake.cancelled);
}

void test_zero_dwell_continues_without_stopping() {
  FakeExecutor fake;
  MotionSequenceController controller(fake);
  MotionSequenceConfig config = baseConfig();
  config.startup_step.dwell_ms = 0;
  controller.setConfig(config);
  controller.setRunning(true, 0);
  fake.active = false;
  controller.update(1);
  controller.update(1);
  TEST_ASSERT_EQUAL_UINT8(1, controller.currentStep());
  fake.active = true;
  fake.near_target = true;
  fake.retarget_result = true;
  controller.update(2);
  TEST_ASSERT_EQUAL_UINT8(2, controller.currentStep());
  TEST_ASSERT_EQUAL(MotionSequenceController::Phase::MOVING, controller.phase());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_startup_move_dwell_and_stop);
  RUN_TEST(test_zero_dwell_continues_without_stopping);
  return UNITY_END();
}
```

- [ ] **Step 2: Verify the new sequencer test fails**

Run: `pio test -e native -f test_motion_sequence`

Expected: FAIL because the sequencer still uses callback structs and is not in
`lib/sequence`.

- [ ] **Step 3: Move and adapt the sequencer**

Use this public constructor and dependency:

```cpp
class MotionSequenceController {
 public:
  enum class Phase : uint8_t { STOPPED, MOVING, DWELLING };
  explicit MotionSequenceController(MotionExecutor& executor);
  void setConfig(const MotionSequenceConfig& config);
  const MotionSequenceConfig& config() const;
  void setRunning(bool running, uint32_t now_ms);
  void stop();
  void update(uint32_t now_ms);
  void resumeAfterPause(uint32_t paused_ms);
  bool running() const;
  Phase phase() const;
  uint8_t currentStep() const;
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
```

Replace callback invocations with `executor_.startMove`,
`executor_.retargetMove`, `executor_.isMoveActive`,
`executor_.isMoveNearTarget`, and `executor_.cancelMove`. Preserve the existing
reversal check and zero-dwell continuation behavior exactly.

- [ ] **Step 4: Remove the obsolete repetitive model from `main.cpp`**

Delete `g_repetitive_motion`, `RepetitiveMotionConfig`,
`startAutomaticPositionMove`, and every serial/web path that edits the old
start/end model. Keep `/api/adjust` by reading the startup step and the relevant
cyclic step directly from `g_sequence_motion.config()`. Store only
`MotionSequenceConfig`; when NVS has no current sequence, create the existing
safe default of startup 0°, step 180° CW, and step 0° CCW.

- [ ] **Step 5: Update contracts and verify**

Update `tests/test_sequence_controller_contract.py` paths to `lib/sequence` and
assert the header contains `MotionExecutor& executor_`. Remove assertions that
require callback typedefs.

Run: `pio test -e native -f test_motion_sequence`

Expected: all sequencer tests PASS.

Run: `python -m unittest discover -s tests -p "test_*.py"`

Expected: all tests PASS.

Run: `pio run -e waveshare_esp32_s3_zero`

Expected: SUCCESS.

- [ ] **Step 6: Commit the sequencer boundary**

```powershell
git add lib/sequence lib/motion_control src/main.cpp tests
git commit -m "refactor: isolate motion sequencing"
```

### Task 3: Physical H-bridge driver and removal of open-loop serial control

**Files:**
- Create: `lib/hardware/HBridgeMotorDriver.h`
- Create: `lib/hardware/HBridgeMotorDriver.cpp`
- Modify: `src/main.cpp`
- Modify: `tests/test_manual_move_contract.py`

**Interfaces:**
- Consumes: Arduino PWM and GPIO APIs.
- Produces: `MotorDriverPins`, `MotorDriverSettings`, and
  `HBridgeMotorDriver::{begin,setSettings,writeSignedPercent,brake,stop}`.

- [ ] **Step 1: Add a source contract that fails before extraction**

Add assertions to `tests/test_manual_move_contract.py`:

```python
DRIVER_H = (ROOT / "lib/hardware/HBridgeMotorDriver.h").read_text(encoding="utf-8")
DRIVER_CPP = (ROOT / "lib/hardware/HBridgeMotorDriver.cpp").read_text(encoding="utf-8")

def test_hbridge_owns_all_physical_output_operations(self):
    for token in ("writeSignedPercent", "brake", "stop", "power_limit_percent"):
        self.assertIn(token, DRIVER_H)
    self.assertIn("ledcWrite", DRIVER_CPP)
    self.assertNotIn("ledcWrite", MAIN)
```

- [ ] **Step 2: Verify the source contract fails**

Run: `python -m unittest tests.test_manual_move_contract -v`

Expected: FAIL because `lib/hardware/HBridgeMotorDriver.*` is absent.

- [ ] **Step 3: Implement the H-bridge driver**

Define:

```cpp
struct MotorDriverPins {
  uint8_t a_in1, a_in2, b_in1, b_in2;
};

struct MotorDriverSettings {
  uint32_t pwm_frequency_hz = 500;
  uint8_t pwm_resolution_bits = 8;
  uint8_t power_limit_percent = 100;
};

class HBridgeMotorDriver {
 public:
  explicit HBridgeMotorDriver(const MotorDriverPins& pins);
  bool begin(const MotorDriverSettings& settings);
  bool setSettings(const MotorDriverSettings& settings);
  const MotorDriverSettings& settings() const;
  void writeSignedPercent(float percent);
  void brake();
  void stop();
  int16_t lastAppliedPercent() const;
};
```

Move `configurePwmOutputs`, `setPwmOutputFrequency`,
`setPwmOutputResolution`, `setMotorSignedPwm`, `setBrakeChannelPins`, physical
parts of `applyMotorOutput`, and `applyBrakeOutput` into the driver. Clamp the
requested value to -100..100 and apply `power_limit_percent` before converting
to duty. `begin` configures all four outputs and calls `stop()`.

- [ ] **Step 4: Delete the open-loop control and serial command parser**

Delete `MotorControlState`, `DrivePhase`, `MotorSelection`,
`updateRampControl`, `parseAndHandleCommand`, `processSerialInput`, prompt/help
functions, serial input buffers, direct PWM commands and motor channel
selection. Replace all legitimate position-control output calls with
`g_motor_driver.writeSignedPercent(output_percent)` and all safety paths with
`g_motor_driver.stop()`. Retain only `Serial.begin` and diagnostic prints.

- [ ] **Step 5: Verify contracts and build**

Run: `python -m unittest discover -s tests -p "test_*.py"`

Expected: all tests PASS after removing serial-specific assertions.

Run: `pio run -e waveshare_esp32_s3_zero`

Expected: SUCCESS and no unresolved references to deleted open-loop functions.

- [ ] **Step 6: Commit the physical driver**

```powershell
git add lib/hardware src/main.cpp tests
git commit -m "refactor: encapsulate h-bridge output"
```

### Task 4: Motion coordinator and sensor recovery boundary

**Files:**
- Move: `lib/motion_control/AngleSensor*` to `lib/hardware/`
- Move: `lib/motion_control/As5600Sensor*` to `lib/hardware/`
- Move: `lib/motion_control/Mt6701Sensor*` to `lib/hardware/`
- Move: `lib/motion_control/VelocityEstimator*` to `lib/control/`
- Move: `lib/motion_control/AdrcPositionController*` to `lib/control/`
- Create: `lib/control/MotionCoordinator.h`
- Create: `lib/control/MotionCoordinator.cpp`
- Modify: `src/main.cpp`
- Modify: `tests/test_sensor_recovery_integration_contract.py`
- Modify: `tests/test_angle_sensor_manager_contract.py`
- Modify: `tests/test_angle_sensor_drivers_contract.py`

**Interfaces:**
- Consumes: `AngleSensorManager`, `AdrcPositionController`,
  `HBridgeMotorDriver`, `MotionRequest`, and `MotionExecutor`.
- Produces: `MotionStatus` and a concrete `MotionExecutor` implementation.

- [ ] **Step 1: Rewrite recovery contracts against the coordinator**

Point sensor test paths to `lib/hardware`. In
`tests/test_sensor_recovery_integration_contract.py`, load
`lib/control/MotionCoordinator.cpp` and assert it contains:

```python
for token in ("consumeLostEvent", "motor_.stop()",
              "consumeRecoveredEvent", "servo_.resumeAtAngle",
              "consumeRecoveredPauseMs", "cancelMove"):
    self.assertIn(token, COORDINATOR_CPP)
```

Also assert `src/main.cpp` no longer contains `g_sensor_pause_active`,
`g_sensor_loss_active`, `resolvePendingNumericDirection`, or
`forceMotorSafeForSensorLoss`.

- [ ] **Step 2: Verify the rewritten contract fails**

Run: `python -m unittest tests.test_sensor_recovery_integration_contract -v`

Expected: FAIL because `MotionCoordinator` does not exist.

- [ ] **Step 3: Define coordinator state and API**

Create this public model:

```cpp
struct MotionStatus {
  bool active = false;
  bool near_target = false;
  bool stalled = false;
  bool sensor_available = false;
  bool paused_for_sensor = false;
  float angle_deg = 0.0f;
  float target_deg = 0.0f;
  float commanded_rpm = 0.0f;
  float measured_rpm = 0.0f;
  int16_t pwm_percent = 0;
};

class MotionCoordinator final : public MotionExecutor {
 public:
  MotionCoordinator(AngleSensorManager& sensor,
                    AdrcPositionController& servo,
                    HBridgeMotorDriver& motor);
  bool startMove(const MotionRequest& request) override;
  bool retargetMove(const MotionRequest& request) override;
  bool isMoveActive() const override;
  bool isMoveNearTarget() const override;
  void cancelMove() override;
  void updateSensor(uint32_t now_ms);
  void updateControl(uint32_t now_ms);
  uint32_t consumeRecoveredPauseMs();
  const MotionStatus& status() const;
};
```

- [ ] **Step 4: Move the recovery and control behavior**

Move logic from `readAngleSensorDeg`, `resolvePendingNumericDirection`,
`updateAngleSensorRecovery`, `updatePositionMoveControl`,
`startSequencePositionMove`, `continueSequencePositionMove`, and
`stopAutomaticPositionMove` into the coordinator. Preserve these invariants:

```cpp
if (!sensor_.active() || !sensor_.readAngleDeg(&angle)) {
  motor_.stop();
  status_.sensor_available = false;
  return;
}

const float output = servo_.computeOutputPercent(angle, now_ms);
if (servo_.isStalled()) {
  motor_.stop();
  status_.stalled = true;
  return;
}
motor_.writeSignedPercent(output);
```

On loss, record the pause start and stop the motor without cancelling the
servo target. On recovery, resolve a pending numeric direction before priming,
call `resumeAtAngle`, and expose the elapsed pause once through
`consumeRecoveredPauseMs`. `cancelMove` clears all pending-resume state and
calls both `servo_.cancel()` and `motor_.stop()`.

- [ ] **Step 5: Wire sequence to the coordinator and verify**

Construct `MotionSequenceController g_sequence_motion(g_motion_coordinator)`.
In the loop, call `updateSensor`, pass any consumed pause to
`g_sequence_motion.resumeAfterPause`, update the sequence only while the sensor
is not paused, then call `updateControl`.

Run: `python -m unittest discover -s tests -p "test_*.py"`

Expected: all recovery and sensor contracts PASS.

Run: `pio test -e native -f test_angle_math -f test_motion_sequence`

Expected: all native tests PASS.

Run: `pio run -e waveshare_esp32_s3_zero`

Expected: SUCCESS.

- [ ] **Step 6: Commit the coordinator**

```powershell
git add lib/control lib/hardware lib/motion_control src/main.cpp tests
git commit -m "refactor: coordinate closed-loop motion"
```

### Task 5: Unified settings model and versioned NVS store

**Files:**
- Create: `lib/settings/DeviceSettings.h`
- Create: `lib/settings/PreferencesSettingsStore.h`
- Create: `lib/settings/PreferencesSettingsStore.cpp`
- Create: `tests/test_settings_store_contract.py`
- Modify: `src/main.cpp`

**Interfaces:**
- Consumes: `AdrcPositionSettings`, `MotionSequenceConfig`,
  `MotorDriverSettings`.
- Produces: `DeviceSettings::defaults`, `validateDeviceSettings`, and
  `PreferencesSettingsStore::{begin,load,save}`.

- [ ] **Step 1: Add failing settings source contracts**

Create `tests/test_settings_store_contract.py` asserting:

```python
self.assertIn("struct DeviceSettings", DEVICE_SETTINGS_H)
self.assertIn("MotionSequenceConfig sequence", DEVICE_SETTINGS_H)
self.assertIn("AdrcPositionSettings position", DEVICE_SETTINGS_H)
self.assertIn("MotorDriverSettings motor", DEVICE_SETTINGS_H)
self.assertIn("bool validateDeviceSettings", DEVICE_SETTINGS_H)
self.assertIn('preferences_.begin("motor_cfg_v2"', STORE_CPP)
self.assertIn('putBytes("snapshot"', STORE_CPP)
self.assertNotIn('getFloat("adrc_wc"', MAIN)
self.assertNotIn('putFloat("adrc_wc"', MAIN)
```

- [ ] **Step 2: Verify the settings contract fails**

Run: `python -m unittest tests.test_settings_store_contract -v`

Expected: FAIL because the settings module is absent.

- [ ] **Step 3: Implement the complete snapshot model**

Define:

```cpp
constexpr uint32_t DEVICE_SETTINGS_VERSION = 2;

struct SensorSettings {
  uint8_t failure_limit = 3;
};

struct DeviceSettings {
  uint32_t version = DEVICE_SETTINGS_VERSION;
  AdrcPositionSettings position;
  MotionSequenceConfig sequence;
  MotorDriverSettings motor;
  SensorSettings sensor;
  bool running = false;
  static DeviceSettings defaults();
};

bool validateDeviceSettings(const DeviceSettings& value);
```

`defaults()` must reproduce the current ADRC values, 500 Hz PWM, 100% power,
sensor failure limit 3, startup step 0°, cyclic step 180° CW and cyclic step 0°
CCW. Validation must enforce every range currently checked by
`/api/settings`, `step_count` 1..16, target 0..<360, RPM 0.1..max target,
dwell at most 3,600,000 ms, PWM 500..20,000 Hz, and power 0..100%.

- [ ] **Step 4: Implement one NVS read/write boundary**

Define:

```cpp
class PreferencesSettingsStore {
 public:
  bool begin();
  DeviceSettings load() const;
  bool save(const DeviceSettings& settings);
 private:
  Preferences preferences_;
  bool ready_ = false;
};
```

Use namespace `motor_cfg_v2`, key `snapshot`, and `getBytesLength` to require
the exact current structure size. `load()` returns `DeviceSettings::defaults()`
when the namespace, size, version, read, or validation fails. `save()` validates
first and succeeds only if `putBytes` returns `sizeof(DeviceSettings)`.

- [ ] **Step 5: Replace legacy Preferences calls and verify**

Replace `loadRepetitiveMotionPreferences`, `saveRepetitiveMotionConfig`,
`saveAdrcSettings`, `saveControlSettings`, and direct `putBool` calls with a
single in-memory `DeviceSettings` and `g_settings_store.save(settings)`.

Run: `python -m unittest discover -s tests -p "test_*.py"`

Expected: all tests PASS.

Run: `pio run -e waveshare_esp32_s3_zero`

Expected: SUCCESS.

- [ ] **Step 6: Commit unified settings**

```powershell
git add lib/settings src/main.cpp tests
git commit -m "refactor: unify persistent device settings"
```

### Task 6: Web control server extraction

**Files:**
- Create: `lib/web/WebControlServer.h`
- Create: `lib/web/WebControlServer.cpp`
- Move: `src/repetitive_motion_web_page.h` to `lib/web/`
- Move: `src/control_settings_web_page.h` to `lib/web/`
- Create: `tests/test_web_contract.py`
- Modify: `src/main.cpp`
- Modify: `tests/test_manual_move_contract.py`
- Modify: `tests/test_sequence_api_contract.py`

**Interfaces:**
- Consumes: `DeviceSettings`, `MotionStatus`, and `MotionSequenceController`
  snapshots through `WebControlActions`.
- Produces: `WebControlServer::{begin,handleClient}` and unchanged HTTP routes.

- [ ] **Step 1: Capture the external route and JSON contract**

Create `tests/test_web_contract.py` and assert `WebControlServer.cpp` contains
all current registrations:

```python
ROUTES = (
    'on("/", HTTP_GET', 'on("/settings", HTTP_GET',
    'on("/api/status", HTTP_GET', 'on("/api/settings", HTTP_GET',
    'on("/api/settings", HTTP_POST', 'on("/api/sequence", HTTP_GET',
    'on("/api/sequence", HTTP_POST', 'on("/api/run", HTTP_POST',
    'on("/api/adjust", HTTP_POST', 'on("/api/manual-move", HTTP_POST',
)
for route in ROUTES:
    self.assertIn(route, WEB_CPP)
for field in ("unit", "running", "moveActive", "phase", "step",
              "sensor", "sensorType", "sensorAddress", "sensorState",
              "sensorFailures", "angle", "maxRpm", "otaBusy", "stall"):
    self.assertIn(f'\\"{field}\\"', WEB_CPP)
```

Assert `src/main.cpp` contains neither `g_web_server.on` nor JSON assembly.

- [ ] **Step 2: Verify the web extraction contract fails**

Run: `python -m unittest tests.test_web_contract -v`

Expected: FAIL because the routes are still in `main.cpp`.

- [ ] **Step 3: Define the web/application boundary**

In `WebControlServer.h`, define request-independent result types and:

```cpp
struct OperationResult {
  bool ok = false;
  int http_status = 500;
  const char* message = "Falha interna";
};

struct ApplicationStatus {
  uint8_t unit = 1;
  bool running = false;
  bool move_active = false;
  const char* phase = "STOPPED";
  uint8_t step = 0;
  bool sensor_available = false;
  const char* sensor_type = "NONE";
  uint8_t sensor_address = 0;
  const char* sensor_state = "DETECTING";
  uint8_t sensor_failures = 0;
  float angle_deg = 0.0f;
  float max_rpm = 0.0f;
  bool ota_busy = false;
  bool stalled = false;
};

class WebControlActions {
 public:
  virtual ~WebControlActions() = default;
  virtual ApplicationStatus status() const = 0;
  virtual DeviceSettings settings() const = 0;
  virtual OperationResult replaceSettings(const DeviceSettings&) = 0;
  virtual OperationResult setRunning(bool running) = 0;
  virtual OperationResult manualMove(const MotionRequest&) = 0;
  virtual OperationResult adjustTo(bool startup_position) = 0;
};

class WebControlServer {
 public:
  WebControlServer(uint16_t port, WebControlActions& actions);
  void begin();
  void handleClient();
 private:
  WebServer server_;
  WebControlActions& actions_;
};
```

Use `OperationResult{true, 200, ""}` for success, 400 for invalid syntax or
range, 409 when motion/sequence/OTA state forbids the request, and 500 when the
validated settings snapshot cannot be persisted.

- [ ] **Step 4: Move handlers without changing external names**

Move `parseWebNumber`, direction parsing/text, JSON helpers,
`sendWebStatus`, `sendWebSettings`, `sendWebSequence`, `sendWebError`, and every
route lambda into `WebControlServer.cpp`. Replace global reads and writes with
`actions_` calls. Return 400 for parse/range errors, 409 for busy-state
rejections and 500 for failed persistence. Serve the two moved HTML constants
unchanged.

- [ ] **Step 5: Verify web contracts and build**

Update old contract tests to load `lib/web/WebControlServer.cpp` instead of
`main.cpp` and keep their assertions about parameter names and busy checks.

Run: `python -m unittest discover -s tests -p "test_*.py"`

Expected: all tests PASS.

Run: `pio run -e waveshare_esp32_s3_zero`

Expected: SUCCESS.

- [ ] **Step 6: Commit the web boundary**

```powershell
git add lib/web src tests
git commit -m "refactor: isolate web control API"
```

### Task 7: Connectivity and OTA extraction

**Files:**
- Create: `lib/connectivity/NetworkServices.h`
- Create: `lib/connectivity/NetworkServices.cpp`
- Create: `tests/test_network_services_contract.py`
- Modify: `src/main.cpp`

**Interfaces:**
- Consumes: unit number, credentials, pins and `NetworkEvents`.
- Produces: `NetworkServices::{begin,update,otaBusy,webEnabled}`.

- [ ] **Step 1: Add a failing network ownership contract**

Create `tests/test_network_services_contract.py` asserting the new source owns
`WiFi.begin`, `WiFi.softAP`, `ArduinoOTA.onStart`, `ArduinoOTA.handle`, the AP
channel mapping 1/6/11, and the 12,000 ms station timeout. Assert `main.cpp`
contains none of those API calls.

- [ ] **Step 2: Verify the contract fails**

Run: `python -m unittest tests.test_network_services_contract -v`

Expected: FAIL because connectivity remains in `main.cpp`.

- [ ] **Step 3: Implement the network boundary**

Define:

```cpp
struct NetworkConfig {
  uint8_t unit_number;
  uint8_t button_pin;
  const char* sta_ssid;
  const char* sta_password;
  const char* ota_password;
};

class NetworkEvents {
 public:
  virtual ~NetworkEvents() = default;
  virtual void onOtaStart() = 0;
};

class NetworkServices {
 public:
  NetworkServices(const NetworkConfig&, NetworkEvents&);
  bool begin();
  void update(uint32_t now_ms);
  bool otaBusy() const;
  bool webEnabled() const;
};
```

Move hostname/SSID formatting, station attempt, AP setup and validation,
button-hold logic, OTA callbacks, and `ArduinoOTA.handle` into this module.
The OTA `onStart` callback must set `ota_busy_` before invoking
`events_.onOtaStart()`.

- [ ] **Step 4: Verify connectivity contracts and build**

Run: `python -m unittest discover -s tests -p "test_*.py"`

Expected: all tests PASS.

Run: `pio run -e waveshare_esp32_s3_zero`

Expected: SUCCESS.

- [ ] **Step 5: Commit connectivity extraction**

```powershell
git add lib/connectivity src/main.cpp tests
git commit -m "refactor: isolate wifi and ota services"
```

### Task 8: Telemetry and diagnostic-only serial

**Files:**
- Create: `lib/diagnostics/MotionTelemetry.h`
- Create: `lib/diagnostics/MotionTelemetry.cpp`
- Create: `tests/test_diagnostics_contract.py`
- Modify: `src/main.cpp`

**Interfaces:**
- Consumes: immutable `MotionStatus` samples.
- Produces: `MotionTelemetry::{beginMove,sample,finishMove,update}` and logs only.

- [ ] **Step 1: Add a failing diagnostic-only contract**

Create `tests/test_diagnostics_contract.py` asserting `MotionTelemetry.cpp`
contains peak RPM, mean RPM, peak PWM, travelled angle and periodic diagnostic
output, while `src/main.cpp` contains neither `Serial.read` nor
`Serial.available`.

- [ ] **Step 2: Verify the contract fails**

Run: `python -m unittest tests.test_diagnostics_contract -v`

Expected: FAIL because telemetry globals remain in `main.cpp`.

- [ ] **Step 3: Move telemetry state into a focused class**

Define:

```cpp
struct MotionTelemetrySummary {
  float peak_rpm_abs = 0.0f;
  float mean_rpm = 0.0f;
  float travelled_deg = 0.0f;
  int16_t peak_pwm_abs = 0;
  uint32_t duration_ms = 0;
};

class MotionTelemetry {
 public:
  void beginMove(const MotionStatus& status, uint32_t now_ms);
  void sample(const MotionStatus& status, uint32_t now_ms);
  MotionTelemetrySummary finishMove(uint32_t now_ms);
  void update(const MotionStatus& status, uint32_t now_ms);
};
```

Move only the useful metrics from `g_move_*`. Use `AngleMath::shortestDelta` for
travel and emit the existing periodic movement diagnostic at 2,000 ms. Delete
unused prompt, echo, input-buffer and command-status state.

- [ ] **Step 4: Verify diagnostic contract and build**

Run: `python -m unittest discover -s tests -p "test_*.py"`

Expected: all tests PASS.

Run: `pio run -e waveshare_esp32_s3_zero`

Expected: SUCCESS.

- [ ] **Step 5: Commit telemetry extraction**

```powershell
git add lib/diagnostics src/main.cpp tests
git commit -m "refactor: isolate motion diagnostics"
```

### Task 9: Application composition and minimal main

**Files:**
- Create: `src/MotorControlApplication.h`
- Create: `src/MotorControlApplication.cpp`
- Create: `tests/test_application_contract.py`
- Modify: `src/main.cpp`
- Modify: `README.md`
- Delete: empty `lib/motion_control/` files or directory entries

**Interfaces:**
- Consumes: every module completed in Tasks 1–8.
- Produces: `MotorControlApplication::{setup,update}` plus
  `WebControlActions` and `NetworkEvents` implementations.

- [ ] **Step 1: Write the minimal-main and update-order contract**

Create `tests/test_application_contract.py`:

```python
def test_main_only_forwards_arduino_lifecycle(self):
    self.assertIn("MotorControlApplication application", MAIN)
    self.assertIn("application.setup()", MAIN)
    self.assertIn("application.update(millis())", MAIN)
    self.assertLess(len(MAIN.splitlines()), 30)

def test_application_preserves_safety_order(self):
    order = [APP.index(token) for token in (
        "network_.update", "web_.handleClient", "motion_.updateSensor",
        "sequence_.update", "motion_.updateControl", "telemetry_.update")]
    self.assertEqual(order, sorted(order))

def test_ota_start_cancels_before_any_future_update(self):
    body = APP.split("void MotorControlApplication::onOtaStart", 1)[1]
    self.assertIn("setRunning(false)", body)
    self.assertIn("motion_.cancelMove()", body)
```

- [ ] **Step 2: Verify the application contract fails**

Run: `python -m unittest tests.test_application_contract -v`

Expected: FAIL because orchestration remains in `main.cpp`.

- [ ] **Step 3: Implement application composition**

Define:

```cpp
class MotorControlApplication final : public WebControlActions,
                                      public NetworkEvents {
 public:
  void setup();
  void update(uint32_t now_ms);
  ApplicationStatus status() const override;
  DeviceSettings settings() const override;
  OperationResult replaceSettings(const DeviceSettings&) override;
  OperationResult setRunning(bool running) override;
  OperationResult manualMove(const MotionRequest&) override;
  OperationResult adjustTo(bool startup_position) override;
  void onOtaStart() override;
 private:
  // Own instances in dependency order: sensor, servo, motor, motion,
  // sequence, settings store, telemetry, network, web.
};
```

`setup()` initializes Serial, safe motor output, settings, sensor, network and
web in that order; applies the validated snapshot; and starts the saved cycle
only after sensor detection is available or marks it pending. `update()` uses
the exact order asserted by the test and skips sequence advancement while the
coordinator reports sensor pause.

Use this final `src/main.cpp`:

```cpp
#include <Arduino.h>
#include "MotorControlApplication.h"

MotorControlApplication application;

void setup() {
  application.setup();
}

void loop() {
  application.update(millis());
}
```

- [ ] **Step 4: Update README structure and operating notes**

Replace the README tree with the final module map. State that serial input is
ignored and serial is diagnostic-only. State that firmware using NVS v2 loads
safe defaults once and must be configured again; remove the historical
conversion description.

- [ ] **Step 5: Run the full verification suite**

Run: `python -m unittest discover -s tests -p "test_*.py"`

Expected: all Python tests PASS.

Run: `pio test -e native`

Expected: all native tests PASS.

Run: `pio run -e waveshare_esp32_s3_zero`

Expected: SUCCESS with no warnings about missing project headers or libraries.

Run: `rg -n "RepetitiveMotionController|DrivePhase|parseAndHandleCommand|processSerialInput|shortestAngleDeltaDeg|g_move_|g_state" src lib tests`

Expected: no matches.

Run: `git diff --check`

Expected: no output.

- [ ] **Step 6: Commit application composition**

```powershell
git add src lib tests README.md platformio.ini
git commit -m "refactor: compose modular motor firmware"
```

### Task 10: Final safety and contract audit

**Files:**
- Modify only files implicated by failed checks from this task.

**Interfaces:**
- Consumes: completed modular firmware.
- Produces: verified external behavior and a clean repository state.

- [ ] **Step 1: Re-run every automated check from a clean process**

Run:

```powershell
python -m unittest discover -s tests -p "test_*.py"
pio test -e native
pio run -e waveshare_esp32_s3_zero
git diff --check
git status --short
```

Expected: all tests/builds succeed; `git diff --check` emits nothing; status
contains only intentional changes made by a correction in this task.

- [ ] **Step 2: Audit the safety call sites**

Run:

```powershell
rg -n "onOtaStart|cancelMove|motor_\.stop|consumeLostEvent|isStalled|setRunning\(false" src lib
rg -n "ledcWrite|digitalWrite" src lib
```

Expected: OTA, STOP, sensor loss and stall each reach `motor_.stop()` through
the application/coordinator; physical output APIs occur only in
`HBridgeMotorDriver.cpp` and sensor/initialization GPIO where appropriate.

- [ ] **Step 3: Audit the web contract against the unchanged frontend**

Run:

```powershell
rg -o '/api/[a-z-]+' lib/web/*_web_page.h | Sort-Object -Unique
rg -o 'on\("/api/[a-z-]+' lib/web/WebControlServer.cpp | Sort-Object -Unique
```

Expected: every frontend API path has a registered backend route; there are no
renamed form fields in the Python web contract.

- [ ] **Step 4: Commit only if the audit required corrections**

If Step 1–3 required source corrections, rerun the complete verification and
commit them:

```powershell
git add src lib tests README.md platformio.ini
git commit -m "fix: preserve modular firmware contracts"
```

If no correction was needed, do not create an empty commit.
