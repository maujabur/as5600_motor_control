# Angle Sensor Autodetection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Autodetect AS5600 and MT6701 sensors over I2C, stop PWM after a configurable consecutive-read failure limit, and safely resume the interrupted movement after periodic redetection.

**Architecture:** Concrete AS5600 and MT6701 drivers implement a small `AngleSensor` interface. `AngleSensorManager` owns detection order, active identity, failure counting, and redetection; `main.cpp` owns motor-safety reactions to manager transitions. The ADRC controller receives a focused resume operation that preserves the move target while resetting time-sensitive estimator state.

**Tech Stack:** C++17/Arduino, ESP32 `TwoWire`, PlatformIO, ESP32 `Preferences`, embedded HTML/JavaScript, Python `unittest` contract tests.

## Global Constraints

- Target board remains `waveshare_esp32_s3_zero`.
- I2C remains on GPIO 5 SDA and GPIO 6 SCL at 400 kHz.
- Detection order is AS5600 `0x36`, MT6701 `0x06`, then MT6701 `0x46`.
- Exactly one angle sensor is expected on the bus.
- Default failure limit is 3 and the accepted persisted range is 1 through 20.
- Redetection interval is fixed at 1000 ms.
- Reaching the failure limit must force both motor channels to PWM zero.
- Runtime sensor loss preserves destination, direction, step, and `running` state.
- Recovery restarts through the normal acceleration ramp and must not resume a move canceled by STOP or OTA.
- No heap allocation is used for sensor drivers or manager selection.
- Analog/PWM, SSI, ABZ, UVW, and EEPROM programming remain out of scope.

## File Structure

- Create `lib/motion_control/AngleSensor.h`: shared model enum and sensor interface.
- Modify `lib/motion_control/As5600Sensor.h`: implement the shared interface without initializing `Wire`.
- Modify `lib/motion_control/As5600Sensor.cpp`: probe and read AS5600 through the supplied bus.
- Create `lib/motion_control/Mt6701Sensor.h`: MT6701 interface declaration.
- Create `lib/motion_control/Mt6701Sensor.cpp`: MT6701 probing and 14-bit register decoding.
- Create `lib/motion_control/AngleSensorManager.h`: detection state, configuration, identity, and transition API.
- Create `lib/motion_control/AngleSensorManager.cpp`: detection order, failure counter, and periodic redetection.
- Modify `lib/motion_control/AdrcPositionController.h`: declare safe movement resume.
- Modify `lib/motion_control/AdrcPositionController.cpp`: reset dynamic controller state while preserving move intent.
- Modify `src/main.cpp`: replace AS5600 coupling, persist settings, gate PWM, and process loss/recovery events.
- Modify `src/control_settings_web_page.h`: expose the failure-limit setting.
- Modify `src/repetitive_motion_web_page.h`: display detection and reconnection states.
- Create `tests/test_angle_sensor_drivers_contract.py`: driver addresses, registers, and conversions.
- Create `tests/test_angle_sensor_manager_contract.py`: manager detection and failure-state contract.
- Create `tests/test_sensor_recovery_integration_contract.py`: ADRC, motor, API, settings, and UI integration contract.
- Modify `README.md`: supported sensors, addresses, configuration, and recovery behavior.

---

### Task 1: Shared sensor interface and concrete drivers

**Files:**
- Create: `lib/motion_control/AngleSensor.h`
- Modify: `lib/motion_control/As5600Sensor.h`
- Modify: `lib/motion_control/As5600Sensor.cpp`
- Create: `lib/motion_control/Mt6701Sensor.h`
- Create: `lib/motion_control/Mt6701Sensor.cpp`
- Create: `tests/test_angle_sensor_drivers_contract.py`

**Interfaces:**
- Consumes: Arduino `TwoWire` read/write operations.
- Produces: `AngleSensorType`, `AngleSensor::probe(TwoWire&, uint8_t)`, `readRawAngle(uint16_t*)`, `readAngleDeg(float*)`, `name()`, `address()`, and `countsPerTurn()`.

- [ ] **Step 1: Write the failing driver contract tests**

Create `tests/test_angle_sensor_drivers_contract.py`:

```python
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class AngleSensorDriversContractTest(unittest.TestCase):
    def test_shared_interface_and_concrete_drivers_exist(self):
        interface = (ROOT / "lib/motion_control/AngleSensor.h").read_text(encoding="utf-8")
        as5600 = (ROOT / "lib/motion_control/As5600Sensor.h").read_text(encoding="utf-8")
        mt6701 = (ROOT / "lib/motion_control/Mt6701Sensor.h").read_text(encoding="utf-8")
        for token in ("enum class AngleSensorType", "virtual bool probe", "readRawAngle",
                      "readAngleDeg", "countsPerTurn"):
            self.assertIn(token, interface)
        self.assertIn("public AngleSensor", as5600)
        self.assertIn("public AngleSensor", mt6701)

    def test_supported_addresses_and_registers_are_explicit(self):
        as5600 = (ROOT / "lib/motion_control/As5600Sensor.h").read_text(encoding="utf-8")
        mt6701 = (ROOT / "lib/motion_control/Mt6701Sensor.h").read_text(encoding="utf-8")
        self.assertIn("0x36", as5600)
        self.assertIn("0x0C", as5600)
        for token in ("0x06", "0x46", "0x03", "0x04"):
            self.assertIn(token, mt6701)

    def test_mt6701_decodes_fourteen_bit_angle(self):
        implementation = (ROOT / "lib/motion_control/Mt6701Sensor.cpp").read_text(encoding="utf-8")
        header = (ROOT / "lib/motion_control/Mt6701Sensor.h").read_text(encoding="utf-8")
        self.assertIn("((uint16_t)high_byte << 6)", implementation)
        self.assertIn("low_byte >> 2", implementation)
        self.assertIn("16384", header)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the new test and verify it fails**

Run:

```powershell
python -m unittest tests.test_angle_sensor_drivers_contract -v
```

Expected: errors for missing `AngleSensor.h` and `Mt6701Sensor.h`.

- [ ] **Step 3: Add the shared interface**

Create `lib/motion_control/AngleSensor.h`:

```cpp
#pragma once

#include <Arduino.h>
#include <Wire.h>

enum class AngleSensorType { None, As5600, Mt6701 };

class AngleSensor {
 public:
  virtual ~AngleSensor() = default;
  virtual bool probe(TwoWire& wire, uint8_t address) = 0;
  virtual bool readRawAngle(uint16_t* raw_angle) = 0;
  virtual const char* name() const = 0;
  virtual AngleSensorType type() const = 0;
  virtual uint8_t address() const = 0;
  virtual uint16_t countsPerTurn() const = 0;

  bool readAngleDeg(float* angle_deg) {
    if (!angle_deg) return false;
    uint16_t raw_angle = 0;
    if (!readRawAngle(&raw_angle)) return false;
    *angle_deg = (float)raw_angle * 360.0f / (float)countsPerTurn();
    return true;
  }
};
```

- [ ] **Step 4: Refactor the AS5600 driver to implement the interface**

Use this public contract in `As5600Sensor.h`:

```cpp
class As5600Sensor final : public AngleSensor {
 public:
  static constexpr uint8_t DEFAULT_I2C_ADDR = 0x36;
  static constexpr uint8_t REG_RAW_ANGLE_H = 0x0C;

  bool begin(TwoWire& wire, uint8_t sda_pin, uint8_t scl_pin,
             uint8_t address = DEFAULT_I2C_ADDR);
  bool probe(TwoWire& wire, uint8_t address) override;
  bool readRawAngle(uint16_t* raw_angle) override;
  bool detected() const { return detected_; }
  const char* name() const override { return "AS5600"; }
  AngleSensorType type() const override { return AngleSensorType::As5600; }
  uint8_t address() const override { return address_; }
  uint16_t countsPerTurn() const override { return 4096; }

 private:
  TwoWire* wire_ = nullptr;
  uint8_t address_ = DEFAULT_I2C_ADDR;
  bool detected_ = false;
};
```

Replace the old `begin()` in `As5600Sensor.cpp` with a probe that does not call `Wire.begin()`:

```cpp
bool As5600Sensor::probe(TwoWire& wire, uint8_t address) {
  wire_ = &wire;
  address_ = address;
  detected_ = true;
  uint16_t raw_angle = 0;
  detected_ = readRawAngle(&raw_angle);
  return detected_;
}
```

Retain the existing `begin(...)`, `detected()`, and `detected_` members as a
temporary compatibility shim so this task leaves the existing `main.cpp`
buildable. `begin(...)` initializes the bus exactly as it does today and then
delegates sensor confirmation to `probe(...)`. Remove this shim in Task 4 after
the manager owns bus initialization. Retain null-pointer, detection,
transmission, and two-byte length checks in `readRawAngle`, and remove the old
out-of-class `readAngleDeg` implementation because the interface now supplies it.

- [ ] **Step 5: Implement the MT6701 driver**

Declare constants and overrides in `Mt6701Sensor.h`:

```cpp
class Mt6701Sensor final : public AngleSensor {
 public:
  static constexpr uint8_t DEFAULT_I2C_ADDR = 0x06;
  static constexpr uint8_t ALTERNATE_I2C_ADDR = 0x46;
  static constexpr uint8_t REG_ANGLE_HIGH = 0x03;
  static constexpr uint8_t REG_ANGLE_LOW = 0x04;

  bool probe(TwoWire& wire, uint8_t address) override;
  bool readRawAngle(uint16_t* raw_angle) override;
  const char* name() const override { return "MT6701"; }
  AngleSensorType type() const override { return AngleSensorType::Mt6701; }
  uint8_t address() const override { return address_; }
  uint16_t countsPerTurn() const override { return 16384; }

 private:
  bool readRegister(uint8_t register_address, uint8_t* value);
  TwoWire* wire_ = nullptr;
  uint8_t address_ = DEFAULT_I2C_ADDR;
};
```

Implement the angle composition in `Mt6701Sensor.cpp`:

```cpp
bool Mt6701Sensor::readRawAngle(uint16_t* raw_angle) {
  if (!wire_ || !raw_angle) return false;
  uint8_t high_byte = 0;
  uint8_t low_byte = 0;
  if (!readRegister(REG_ANGLE_HIGH, &high_byte)) return false;
  if (!readRegister(REG_ANGLE_LOW, &low_byte)) return false;
  *raw_angle = (uint16_t)(((uint16_t)high_byte << 6) |
                         ((uint16_t)low_byte >> 2)) & 0x3FFF;
  return true;
}
```

`probe()` must reject addresses other than `0x06` and `0x46`, attach the supplied `TwoWire`, and confirm one complete raw read. `readRegister()` must use a repeated start, request exactly one byte, and return false on any I2C error.

- [ ] **Step 6: Run driver tests and firmware build**

Run:

```powershell
python -m unittest tests.test_angle_sensor_drivers_contract -v
& 'C:\Users\User\.platformio\penv\Scripts\pio.exe' run -e waveshare_esp32_s3_zero
```

Expected: driver tests pass and PlatformIO reports `SUCCESS`; the temporary
AS5600 compatibility shim keeps the unchanged integration compiling.

- [ ] **Step 7: Commit the drivers**

```powershell
git add lib/motion_control/AngleSensor.h lib/motion_control/As5600Sensor.h lib/motion_control/As5600Sensor.cpp lib/motion_control/Mt6701Sensor.h lib/motion_control/Mt6701Sensor.cpp tests/test_angle_sensor_drivers_contract.py
git commit -m "feat: add generic AS5600 and MT6701 drivers"
```

### Task 2: Detection, failure counting, and periodic redetection

**Files:**
- Create: `lib/motion_control/AngleSensorManager.h`
- Create: `lib/motion_control/AngleSensorManager.cpp`
- Create: `tests/test_angle_sensor_manager_contract.py`

**Interfaces:**
- Consumes: `As5600Sensor`, `Mt6701Sensor`, `TwoWire`, and Arduino `millis()`.
- Produces: `AngleSensorManager::begin`, `update`, `readAngleDeg`, `setFailureLimit`, state/identity accessors, `consumeLostEvent`, and `consumeRecoveredEvent`.

- [ ] **Step 1: Write the failing manager contract tests**

Create `tests/test_angle_sensor_manager_contract.py`:

```python
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
HEADER = ROOT / "lib/motion_control/AngleSensorManager.h"
SOURCE = ROOT / "lib/motion_control/AngleSensorManager.cpp"


class AngleSensorManagerContractTest(unittest.TestCase):
    def test_manager_exposes_state_identity_and_events(self):
        header = HEADER.read_text(encoding="utf-8")
        for token in ("enum class State", "Detecting", "Active", "Lost",
                      "setFailureLimit", "failureCount", "consumeLostEvent",
                      "consumeRecoveredEvent", "sensorName", "sensorAddress"):
            self.assertIn(token, header)

    def test_detection_order_and_interval_are_fixed(self):
        source = SOURCE.read_text(encoding="utf-8")
        positions = [source.index(token) for token in
                     ("As5600Sensor::DEFAULT_I2C_ADDR",
                      "Mt6701Sensor::DEFAULT_I2C_ADDR",
                      "Mt6701Sensor::ALTERNATE_I2C_ADDR")]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("1000", source)

    def test_consecutive_failures_drive_lost_state(self):
        source = SOURCE.read_text(encoding="utf-8")
        self.assertIn("failure_count_ = 0", source)
        self.assertIn("failure_count_ >= failure_limit_", source)
        self.assertIn("State::Lost", source)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the test and verify it fails**

Run `python -m unittest tests.test_angle_sensor_manager_contract -v`.

Expected: error because manager files do not exist.

- [ ] **Step 3: Declare the manager API**

Use this public surface in `AngleSensorManager.h`:

```cpp
class AngleSensorManager {
 public:
  enum class State { Detecting, Active, Lost };
  static constexpr uint32_t REDETECT_INTERVAL_MS = 1000;

  bool begin(TwoWire& wire, uint8_t sda_pin, uint8_t scl_pin,
             uint32_t clock_hz = 400000);
  void update(uint32_t now_ms);
  bool readAngleDeg(float* angle_deg);
  bool readRawAngle(uint16_t* raw_angle);
  void setFailureLimit(uint8_t limit);
  uint8_t failureLimit() const { return failure_limit_; }
  uint8_t failureCount() const { return failure_count_; }
  bool active() const { return state_ == State::Active && active_sensor_; }
  State state() const { return state_; }
  const char* sensorName() const;
  uint8_t sensorAddress() const;
  bool consumeLostEvent();
  bool consumeRecoveredEvent(float* confirmed_angle_deg);

 private:
  bool detect(uint32_t now_ms);
  bool select(AngleSensor& sensor, uint8_t address, float* angle_deg);
  void recordFailure();

  TwoWire* wire_ = nullptr;
  As5600Sensor as5600_;
  Mt6701Sensor mt6701_;
  AngleSensor* active_sensor_ = nullptr;
  State state_ = State::Detecting;
  uint8_t failure_limit_ = 3;
  uint8_t failure_count_ = 0;
  uint32_t last_detect_attempt_ms_ = 0;
  bool lost_event_ = false;
  bool recovered_event_ = false;
  float recovered_angle_deg_ = 0.0f;
};
```

- [ ] **Step 4: Implement detection order and transition behavior**

`begin()` initializes the bus exactly once and immediately calls `detect(millis())`:

```cpp
bool AngleSensorManager::begin(TwoWire& wire, uint8_t sda_pin, uint8_t scl_pin,
                               uint32_t clock_hz) {
  wire_ = &wire;
  wire_->begin(sda_pin, scl_pin);
  wire_->setClock(clock_hz);
  state_ = State::Detecting;
  return detect(millis());
}
```

`detect()` tries the exact ordered candidates and records recovery only when the previous state was `Lost`:

```cpp
bool AngleSensorManager::detect(uint32_t now_ms) {
  last_detect_attempt_ms_ = now_ms;
  const bool was_lost = state_ == State::Lost;
  float angle_deg = 0.0f;
  if (select(as5600_, As5600Sensor::DEFAULT_I2C_ADDR, &angle_deg) ||
      select(mt6701_, Mt6701Sensor::DEFAULT_I2C_ADDR, &angle_deg) ||
      select(mt6701_, Mt6701Sensor::ALTERNATE_I2C_ADDR, &angle_deg)) {
    state_ = State::Active;
    failure_count_ = 0;
    if (was_lost) {
      recovered_angle_deg_ = angle_deg;
      recovered_event_ = true;
    }
    return true;
  }
  active_sensor_ = nullptr;
  state_ = was_lost ? State::Lost : State::Detecting;
  return false;
}
```

`readAngleDeg()` and `readRawAngle()` reset the consecutive count on success and call `recordFailure()` on failure. `recordFailure()` must set `active_sensor_` to null, set `State::Lost`, and raise `lost_event_` when `failure_count_ >= failure_limit_`. `update()` calls `detect(now_ms)` only when not active and at least 1000 ms elapsed.

When `recordFailure()` enters `Lost`, set `last_detect_attempt_ms_ = millis()`
so the first retry occurs after the full 1000 ms interval rather than in the
same control iteration.

- [ ] **Step 5: Run manager and driver tests**

Run:

```powershell
python -m unittest tests.test_angle_sensor_drivers_contract tests.test_angle_sensor_manager_contract -v
```

Expected: all tests pass.

- [ ] **Step 6: Commit the manager**

```powershell
git add lib/motion_control/AngleSensorManager.h lib/motion_control/AngleSensorManager.cpp tests/test_angle_sensor_manager_contract.py
git commit -m "feat: add angle sensor autodetection manager"
```

### Task 3: Safe ADRC resume operation

**Files:**
- Modify: `lib/motion_control/AdrcPositionController.h`
- Modify: `lib/motion_control/AdrcPositionController.cpp`
- Create: `tests/test_sensor_recovery_integration_contract.py`

**Interfaces:**
- Consumes: active ADRC move state and a confirmed recovered angle.
- Produces: `void resumeAtAngle(float current_deg, uint32_t now_ms)`.

- [ ] **Step 1: Write the failing ADRC recovery contract**

Start `tests/test_sensor_recovery_integration_contract.py` with:

```python
import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
ADRC_H = (ROOT / "lib/motion_control/AdrcPositionController.h").read_text(encoding="utf-8")
ADRC_CPP = (ROOT / "lib/motion_control/AdrcPositionController.cpp").read_text(encoding="utf-8")


class SensorRecoveryIntegrationContractTest(unittest.TestCase):
    def test_adrc_resume_preserves_move_and_resets_dynamic_state(self):
        self.assertIn("resumeAtAngle(float current_deg, uint32_t now_ms)", ADRC_H)
        body = ADRC_CPP.split("void AdrcPositionController::resumeAtAngle", 1)[1]
        body = body.split("void AdrcPositionController::", 1)[0]
        for token in ("if (!active_)", "primeAccumulatedAngle(current_deg)",
                      "velocity_estimator_.reset()", "stall_started_ms_ = 0",
                      "profile_velocity_deg_s_ = 0.0f", "last_output_pwm_ = 0.0f"):
            self.assertIn(token, body)
        self.assertNotIn("active_ = false", body)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the ADRC test and verify it fails**

Run `python -m unittest tests.test_sensor_recovery_integration_contract -v`.

Expected: failure because `resumeAtAngle` is absent.

- [ ] **Step 3: Implement safe resume**

Add the declaration to the public section of `AdrcPositionController.h` and this implementation before `computeOutputPercent`:

```cpp
void AdrcPositionController::resumeAtAngle(float current_deg, uint32_t now_ms) {
  if (!active_) return;
  kicking_ = false;
  stalled_ = false;
  samples_in_window_ = 0;
  stall_started_ms_ = 0;
  commanded_rpm_ = 0.0f;
  profile_velocity_deg_s_ = 0.0f;
  last_error_pos_deg_ = 0.0f;
  last_output_pwm_ = 0.0f;
  last_pwm_output_percent_ = 0;
  velocity_estimator_.reset();
  primeAccumulatedAngle(current_deg);
  move_started_ms_ = now_ms;
  last_compute_ms_ = now_ms;
}
```

This deliberately preserves `active_`, `target_deg_`, `direction_`, and `max_speed_rpm_`. `primeAccumulatedAngle()` recalculates the accumulated target from the preserved absolute target and direction.

- [ ] **Step 4: Run the recovery contract and build**

Run:

```powershell
python -m unittest tests.test_sensor_recovery_integration_contract -v
& 'C:\Users\User\.platformio\penv\Scripts\pio.exe' run -e waveshare_esp32_s3_zero
```

Expected: test and build pass.

- [ ] **Step 5: Commit ADRC recovery**

```powershell
git add lib/motion_control/AdrcPositionController.h lib/motion_control/AdrcPositionController.cpp tests/test_sensor_recovery_integration_contract.py
git commit -m "feat: resume ADRC safely after sensor recovery"
```

### Task 4: Firmware integration and motor-safety gate

**Files:**
- Modify: `src/main.cpp`
- Modify: `tests/test_sensor_recovery_integration_contract.py`

**Interfaces:**
- Consumes: `AngleSensorManager` events and `AdrcPositionController::resumeAtAngle`.
- Produces: generic sensor reads throughout the firmware, immediate PWM gating in `LOST`, and automatic continuation of active moves.

- [ ] **Step 1: Extend the failing integration contract**

Add these methods to `SensorRecoveryIntegrationContractTest`:

```python
    def test_main_uses_generic_manager_and_no_as5600_global(self):
        main = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
        self.assertIn("AngleSensorManager g_angle_sensor", main)
        self.assertNotIn("As5600Sensor            g_as5600", main)
        self.assertNotIn("g_as5600.", main)

    def test_loss_forces_zero_and_recovery_resumes_active_move(self):
        main = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
        self.assertIn("consumeLostEvent()", main)
        self.assertIn("applyMotorOutput(0)", main)
        self.assertIn("g_sensor_pause_active", main)
        self.assertIn("consumeRecoveredEvent(&recovered_angle_deg)", main)
        self.assertIn("resumeAtAngle(recovered_angle_deg, now_ms)", main)

    def test_loop_updates_detection_before_motion_and_pwm(self):
        main = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
        loop = main.split("void loop()", 1)[1]
        order = [loop.index(token) for token in
                 ("updateAngleSensorRecovery", "g_sequence_motion.update",
                  "updatePositionMoveControl", "updateRampControl")]
        self.assertEqual(order, sorted(order))
```

- [ ] **Step 2: Run the integration contract and verify it fails**

Run `python -m unittest tests.test_sensor_recovery_integration_contract -v`.

Expected: failures for the missing manager global and recovery helper.

- [ ] **Step 3: Replace the concrete global and initialize the manager**

Replace the AS5600 include/global with:

```cpp
#include <AngleSensorManager.h>

AngleSensorManager       g_angle_sensor;
AdrcPositionController   g_position_servo;
bool                     g_sensor_pause_active = false;
bool                     g_run_when_sensor_detected = false;
```

In `setup()`, after preferences are loaded and before networking, initialize the bus once:

```cpp
g_angle_sensor.begin(Wire, I2C_SDA_PIN, I2C_SCL_PIN, 400000);
```

Replace every sensor availability check with `g_angle_sensor.active()`, every degree read with `g_angle_sensor.readAngleDeg(&value)`, and every raw read with `g_angle_sensor.readRawAngle(&value)`. Replace AS5600-specific errors with `sensor angular` wording.

Now remove the temporary AS5600 `begin(...)`, `detected()`, and `detected_`
compatibility shim from Task 1. The concrete driver must finish this task with
no `Wire.begin()` or `setClock()` call. Remove the `detected_` condition from
`readRawAngle()` at the same time; the manager's active pointer now owns sensor
availability state.

- [ ] **Step 4: Add the loss/recovery coordinator**

Add this helper near `updatePositionMoveControl()`:

```cpp
void updateAngleSensorRecovery(uint32_t now_ms) {
  g_angle_sensor.update(now_ms);
  if (g_angle_sensor.consumeLostEvent()) {
    g_sensor_pause_active = g_position_servo.isActive();
    g_state.target_percent = 0.0f;
    g_state.current_percent = 0.0f;
    g_state.drive_phase = DrivePhase::IDLE;
    applyMotorOutput(0);
    Serial.printf("Sensor angular perdido apos %u falhas; PWM bloqueado\n",
                  g_angle_sensor.failureLimit());
  }

  float recovered_angle_deg = 0.0f;
  if (g_angle_sensor.consumeRecoveredEvent(&recovered_angle_deg)) {
    Serial.printf("%s reconectado em 0x%02X\n",
                  g_angle_sensor.sensorName(), g_angle_sensor.sensorAddress());
    if (g_sensor_pause_active && g_position_servo.isActive() &&
        !g_ota_update_in_progress) {
      g_position_servo.resumeAtAngle(recovered_angle_deg, now_ms);
      g_move_prev_sample_valid = false;
      g_move_rpm_window_delta_deg = 0.0f;
      g_move_rpm_window_dt_ms = 0;
    }
    g_sensor_pause_active = false;
  }

  if (g_run_when_sensor_detected && g_angle_sensor.active() &&
      !g_ota_update_in_progress) {
    g_run_when_sensor_detected = false;
    setRepetitiveRunning(true, false);
    Serial.println("Sensor detectado; iniciando ciclo persistente");
  }
}
```

Call `updateAngleSensorRecovery(millis())` before sequence, position, and ramp updates in `loop()`.

- [ ] **Step 5: Gate both control paths while paused**

At the beginning of `updatePositionMoveControl()` and `updateRampControl()`, after their local time/period checks where applicable, add:

```cpp
if (g_sensor_pause_active ||
    (g_position_servo.isActive() && !g_angle_sensor.active())) {
  g_state.target_percent = 0.0f;
  g_state.current_percent = 0.0f;
  g_state.drive_phase = DrivePhase::IDLE;
  applyMotorOutput(0);
  return;
}
```

In STOP and OTA cancellation paths, set `g_sensor_pause_active = false` after canceling the ADRC controller so later sensor recovery cannot resume the canceled move.

- [ ] **Step 6: Update startup and transition logging**

Use generic startup output:

```cpp
if (g_angle_sensor.active()) {
  Serial.printf("%s detectado no endereco 0x%02X\n",
                g_angle_sensor.sensorName(), g_angle_sensor.sensorAddress());
} else {
  Serial.println("Sensor angular nao detectado; procurando periodicamente");
}
```

If persisted `running=on` and no sensor is present at boot, do not clear the persisted flag. Keep the sequence stopped until initial detection, then start it once from `updateAngleSensorRecovery`; use a dedicated `g_run_when_sensor_detected` boolean set during setup and cleared after starting or after STOP/OTA.

- [ ] **Step 7: Run all contracts and firmware build**

Run:

```powershell
python -m unittest discover -s tests -v
& 'C:\Users\User\.platformio\penv\Scripts\pio.exe' run -e waveshare_esp32_s3_zero
```

Expected: all tests pass and PlatformIO reports `SUCCESS`.

- [ ] **Step 8: Commit firmware integration**

```powershell
git add src/main.cpp tests/test_sensor_recovery_integration_contract.py
git commit -m "feat: pause and resume motion across sensor loss"
```

### Task 5: Persistent setting, status API, and web UI

**Files:**
- Modify: `src/main.cpp`
- Modify: `src/control_settings_web_page.h`
- Modify: `src/repetitive_motion_web_page.h`
- Modify: `tests/test_sensor_recovery_integration_contract.py`

**Interfaces:**
- Consumes: manager identity, state, failure count, and failure-limit setter.
- Produces: NVS key `sensor_fail`, settings field `sensorFailures`, and status fields `sensorType`, `sensorAddress`, `sensorState`, `sensorFailures`.

- [ ] **Step 1: Add failing configuration, API, and UI contracts**

Add these methods to `SensorRecoveryIntegrationContractTest`:

```python
    def test_failure_limit_is_persisted_and_validated(self):
        main = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
        page = (ROOT / "src/control_settings_web_page.h").read_text(encoding="utf-8")
        for token in ('getUInt("sensor_fail"', 'putUInt("sensor_fail"',
                      'parseWebNumber("sensorFailures"',
                      "setFailureLimit((uint8_t)lroundf(sensor_failures))"):
            self.assertIn(token, main)
        self.assertIn('id="sensorFailures"', page)
        self.assertIn("'sensorFailures'", page)

    def test_status_api_exposes_sensor_identity_and_state(self):
        main = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
        for field in ('"sensorType"', '"sensorAddress"', '"sensorState"',
                      '"sensorFailures"'):
            self.assertIn(field, main)

    def test_main_page_distinguishes_detection_and_reconnection(self):
        page = (ROOT / "src/repetitive_motion_web_page.h").read_text(encoding="utf-8")
        self.assertIn("DETECTANDO SENSOR", page)
        self.assertIn("RECONECTANDO SENSOR", page)
        self.assertIn("sensorState", page)
```

- [ ] **Step 2: Run the integration contract and verify it fails**

Run `python -m unittest tests.test_sensor_recovery_integration_contract -v`.

Expected: the three new tests fail.

- [ ] **Step 3: Persist and validate the failure limit**

In preference loading:

```cpp
g_angle_sensor.setFailureLimit((uint8_t)constrain(
  g_repetitive_preferences.getUInt("sensor_fail", 3), 1U, 20U));
```

In `saveControlSettings()`:

```cpp
g_repetitive_preferences.putUInt("sensor_fail", g_angle_sensor.failureLimit());
```

In the settings POST handler, parse `sensorFailures`, require an integer in the inclusive range 1 through 20, and apply:

```cpp
g_angle_sensor.setFailureLimit((uint8_t)lroundf(sensor_failures));
```

Add `sensorFailures` to `sendWebSettings()` using `g_angle_sensor.failureLimit()`.

- [ ] **Step 4: Add generic status fields**

Add helpers in `main.cpp`:

```cpp
const char* angleSensorStateText(AngleSensorManager::State state) {
  switch (state) {
    case AngleSensorManager::State::Active: return "ACTIVE";
    case AngleSensorManager::State::Lost: return "LOST";
    case AngleSensorManager::State::Detecting: return "DETECTING";
  }
  return "DETECTING";
}
```

In `sendWebStatus()`, preserve the existing boolean `sensor` and append model, address, state, and current consecutive-failure count. Reserve at least 420 bytes for the JSON string.

- [ ] **Step 5: Add the advanced setting field**

In `control_settings_web_page.h`, add this field under a new sensor-protection section:

```html
<section><h2>Proteção do sensor angular</h2><div class="grid">
<div class="field"><label for="sensorFailures">Falhas consecutivas do sensor</label><div class="input"><input id="sensorFailures" type="number" min="1" max="20" step="1" required><span class="unit">N</span></div><div class="hint">Ao atingir o limite, o PWM é zerado e a reconexão começa.</div></div>
</div></section>
```

Add `'sensorFailures'` to the JavaScript `ids` array so GET rendering and POST submission use the existing generic loop.

- [ ] **Step 6: Render sensor connection states on the main page**

Inside the existing `render(s)` function, calculate the angle label with:

```javascript
const sensorText=s.sensor?s.angle.toFixed(1)+'°':(s.sensorState==='LOST'?'RECONECTANDO SENSOR':'DETECTANDO SENSOR');
$('angle').textContent=sensorText;
```

Keep movement controls disabled unless `s.sensor` is true. While `LOST`, keep STOP enabled whenever `s.running` or `s.moveActive` is true.

- [ ] **Step 7: Run all tests and build**

Run:

```powershell
python -m unittest discover -s tests -v
& 'C:\Users\User\.platformio\penv\Scripts\pio.exe' run -e waveshare_esp32_s3_zero
```

Expected: all tests pass and firmware build succeeds.

- [ ] **Step 8: Commit settings and UI**

```powershell
git add src/main.cpp src/control_settings_web_page.h src/repetitive_motion_web_page.h tests/test_sensor_recovery_integration_contract.py
git commit -m "feat: expose sensor recovery status and settings"
```

### Task 6: Documentation and final verification

**Files:**
- Modify: `README.md`

**Interfaces:**
- Consumes: final implemented behavior.
- Produces: operator-facing hardware, configuration, and recovery documentation.

- [ ] **Step 1: Update README sensor documentation**

Change the title and introduction to name both AS5600 and MT6701. Update the hardware table to label GPIO 5/6 as shared angle-sensor SDA/SCL. Add this address table:

```markdown
| Sensor | Endereço I²C | Resolução |
|---|---:|---:|
| AS5600 | `0x36` | 12 bits |
| MT6701 | `0x06` ou `0x46` | 14 bits |
```

Document that only one sensor is connected, selection is automatic, `sensorFailures` defaults to 3, redetection runs every 1 second, PWM is blocked while disconnected, and an interrupted move resumes from the same step through a fresh acceleration ramp. State that STOP and OTA cancel pending automatic resume.

- [ ] **Step 2: Check for stale AS5600-only integration text**

Run:

```powershell
rg -n "g_as5600|AS5600 nao detectado|falha ao ler AS5600|AS5600 SDA|AS5600 SCL" src lib README.md
```

Expected: no output. AS5600 references remain only where model-specific documentation or the concrete driver requires them.

- [ ] **Step 3: Run the complete verification suite**

Run:

```powershell
python -m unittest discover -s tests -v
& 'C:\Users\User\.platformio\penv\Scripts\pio.exe' run -e waveshare_esp32_s3_zero
git diff --check
git status --short
```

Expected: all tests pass; PlatformIO reports `SUCCESS`; `git diff --check` emits no errors; only the intended README change is uncommitted at this point.

- [ ] **Step 4: Commit documentation**

```powershell
git add README.md
git commit -m "docs: document AS5600 and MT6701 autodetection"
```

- [ ] **Step 5: Verify the committed branch is clean**

Run:

```powershell
git status --short
git log --oneline -7
```

Expected: clean status and the sensor-driver, manager, ADRC recovery, firmware integration, UI/settings, and documentation commits appear above the design and plan commits.
