# Motion Sequencer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the fixed repetitive two-position cycle with a persisted sequence containing one startup step and one to sixteen looping steps, while retaining manual shortest-path movement.

**Architecture:** Introduce a hardware-independent `MotionSequenceController` that owns a fixed-capacity configuration and runs one move/wait state machine. `main.cpp` adapts it to the existing ADRC servo, serializes its configuration as a versioned Preferences blob, and exposes status/configuration endpoints. The embedded page renders and edits the variable-length sequence only while stopped.

**Tech Stack:** C++17/Arduino, ESP32 Preferences, WebServer form encoding, embedded HTML/CSS/JavaScript, Python contract tests, PlatformIO.

## Global Constraints

- Step 0 runs once after RUN or persisted RUN at boot and is excluded from the loop.
- Loop length is arbitrary from 1 through 16 steps.
- Every step has target angle, RPM, post-arrival dwell, and direction (`shortest`, `cw`, `ccw`).
- After the last loop step and its dwell, execution returns to step 1.
- Sequence editing is blocked while RUN, a position move, or OTA is active.
- Existing saved start/end configuration is migrated when no versioned sequence exists.
- Manual movement remains non-persistent and shortest-path only.

---

### Task 1: Hardware-independent sequence controller

**Files:**
- Create: `lib/motion_control/MotionSequenceController.h`
- Create: `lib/motion_control/MotionSequenceController.cpp`
- Create: `tests/test_sequence_controller_contract.py`

**Interfaces:**
- Produces: `MotionStep`, `MotionSequenceConfig`, and `MotionSequenceController::{setConfig,setRunning,stop,update,running,phase,currentStep}`.
- Consumes callbacks matching `start_move(target_deg, rpm, direction)`, `is_move_active()`, and `stop_move()`.

- [ ] Write a controller contract test proving the fixed capacity, startup step, directions, phases, and loop boundary are present.
- [ ] Run the contract test before implementation and confirm failure because `MotionSequenceController.h` is absent.
- [ ] Implement the fixed-capacity state machine with phases `STOPPED`, `MOVING`, and `DWELLING`.
- [ ] Run the contract test; expect exit code 0.

### Task 2: Firmware integration, persistence, and API

**Files:**
- Modify: `src/main.cpp`
- Delete: `lib/motion_control/RepetitiveMotionController.h`
- Delete: `lib/motion_control/RepetitiveMotionController.cpp`
- Create: `tests/test_sequence_api_contract.py`

**Interfaces:**
- Consumes: `MotionSequenceConfig` with `startup_step`, `steps[16]`, and `step_count`.
- Produces: `GET /api/sequence`, `POST /api/sequence`, and sequence fields in `GET /api/status`.

- [ ] Write failing contract tests for versioned blob persistence, migration, validation, busy-state rejection, three direction values, and status fields.
- [ ] Run tests and confirm failure because sequence endpoints do not exist.
- [ ] Replace the repetitive controller adapter and status/run integration with the sequence controller.
- [ ] Store a versioned packed blob using Preferences `putBytes/getBytes`; if absent, translate current start/end/RPM/wait keys into startup + two loop steps.
- [ ] Parse form fields `count`, `s0_target/rpm/dwell/direction` through `s16_*`; validate count, angle, RPM, dwell, and direction before applying atomically.
- [ ] Run contract tests and expect all pass.

### Task 3: Variable-length web editor

**Files:**
- Modify: `src/repetitive_motion_web_page.h`
- Modify: `tests/test_sequence_api_contract.py`

**Interfaces:**
- Consumes: JSON returned by `/api/sequence` and `/api/status`.
- Produces: add, delete, and reorder controls plus one save action posting the complete sequence.

- [ ] Extend the contract test to require a distinct step-0 card, loop container, add/delete/up/down controls, three direction choices, and disabled editing while busy.
- [ ] Run the test and confirm failure on missing editor elements.
- [ ] Replace fixed start/end/RPM/dwell fields with a startup card and dynamically rendered loop cards.
- [ ] Preserve the gauge, RUN/STOP, manual movement, messages, settings link, and responsive styling.
- [ ] Run all Python contract tests and expect all pass.

### Task 4: Documentation and complete verification

**Files:**
- Modify: `README.md`

- [ ] Document the step model, startup behavior, looping behavior, limit of 16, directions, migration, edit lock, and manual command.
- [ ] Run `python -m unittest discover -s tests -p "test_*.py" -v`; expect zero failures.
- [ ] Run PlatformIO build for `waveshare_esp32_s3_zero`; expect SUCCESS.
- [ ] Run `git diff --check`; expect no errors.
