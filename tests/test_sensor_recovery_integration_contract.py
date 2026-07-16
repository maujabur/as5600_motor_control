import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
ADRC_H = (ROOT / "lib/motion_control/AdrcPositionController.h").read_text(encoding="utf-8")
ADRC_CPP = (ROOT / "lib/motion_control/AdrcPositionController.cpp").read_text(encoding="utf-8")
SEQUENCE_H = (ROOT / "lib/motion_control/MotionSequenceController.h").read_text(encoding="utf-8")
SEQUENCE_CPP = (ROOT / "lib/motion_control/MotionSequenceController.cpp").read_text(encoding="utf-8")


class SensorRecoveryIntegrationContractTest(unittest.TestCase):
    def test_adrc_resume_preserves_move_and_resets_dynamic_state(self):
        self.assertIn("resumeAtAngle(float current_deg, uint32_t now_ms)", ADRC_H)
        body = ADRC_CPP.split("void AdrcPositionController::resumeAtAngle", 1)[1]
        body = body.split("void AdrcPositionController::", 1)[0]
        for token in ("if (!active_)", "if (!accumulated_initialized_)",
                      "primeAccumulatedAngle(current_deg)",
                      "normalize360(current_deg)",
                      "shortestDelta(last_current_deg_normalized_, normalized)",
                      "profiled_target_deg_ = current_accumulated_deg_",
                      "resetObserver(current_accumulated_deg_, now_ms)",
                      "velocity_estimator_.reset()", "stall_started_ms_ = 0",
                      "profile_velocity_deg_s_ = 0.0f", "last_output_pwm_ = 0.0f"):
            self.assertIn(token, body)
        initialized_branch = body.split("else", 1)[1]
        self.assertNotIn("target_accumulated_deg_ =", initialized_branch)
        self.assertNotIn("active_ = false", body)

        start = ADRC_CPP.split("void AdrcPositionController::startMove", 1)[1]
        start = start.split("void AdrcPositionController::", 1)[0]
        self.assertIn("accumulated_initialized_ = false", start)

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

    def test_sequence_state_is_frozen_only_during_sensor_loss(self):
        main = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
        loop = main.split("void loop()", 1)[1]
        sequence_prefix = loop.split("g_sequence_motion.update(now_ms)", 1)[0]
        self.assertIn("if (!g_sensor_loss_active)", sequence_prefix)
        recovery = main.split("void updateAngleSensorRecovery", 1)[1]
        recovery = recovery.split("void updatePositionMoveControl", 1)[0]
        lost_branch = recovery.split("consumeLostEvent()", 1)[1]
        lost_branch = lost_branch.split("consumeRecoveredEvent", 1)[0]
        self.assertIn("forceMotorSafeForSensorLoss()", lost_branch)
        self.assertIn("g_sensor_loss_active = false", recovery)
        self.assertIn("g_run_when_sensor_detected", recovery)
        self.assertIn("resumeAfterPause", recovery)
        self.assertIn("resumeAfterPause(uint32_t paused_ms)", SEQUENCE_H)
        resume = SEQUENCE_CPP.split(
            "void MotionSequenceController::resumeAfterPause", 1)[1]
        resume = resume.split("void MotionSequenceController::", 1)[0]
        self.assertIn("if (!running_)", resume)
        self.assertIn("phase_started_ms_ += paused_ms", resume)

    def test_read_that_reaches_lost_state_forces_pwm_zero_immediately(self):
        main = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
        self.assertIn("void forceMotorSafeForSensorLoss()", main)
        self.assertIn("bool readAngleSensorDeg(float* angle_deg)", main)
        self.assertEqual(main.count("g_angle_sensor.readAngleDeg"), 1)
        position = main.split("void updatePositionMoveControl()", 1)[1]
        position = position.split("void updateRampControl()", 1)[0]
        failed_read = position.split(
            "if (!readAngleSensorDeg(&current_deg))", 1)[1]
        failed_read = failed_read.split("const uint32_t now_ms", 1)[0]
        self.assertIn("return", failed_read)
        read_helper = main.split("bool readAngleSensorDeg(float* angle_deg)", 1)[1]
        read_helper = read_helper.split("void updateAngleSensorRecovery", 1)[0]
        self.assertIn("g_angle_sensor.readAngleDeg(angle_deg)", read_helper)
        self.assertIn("!g_angle_sensor.active()", read_helper)
        self.assertIn("forceMotorSafeForSensorLoss()", read_helper)
        helper = main.split("void forceMotorSafeForSensorLoss()", 1)[1]
        helper = helper.split("void updateAngleSensorRecovery", 1)[0]
        for token in ("g_sensor_pause_active", "g_sensor_loss_active = true",
                      "g_state.target_percent = 0.0f", "applyMotorOutput(0)"):
            self.assertIn(token, helper)

    def test_step_start_loss_keeps_automatic_move_active_and_pending(self):
        main = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
        for function_name in ("startSequencePositionMove",
                              "startAutomaticPositionMove"):
            body = main.rsplit(f"void {function_name}", 1)[1]
            body = body.split("\nvoid ", 1)[0]
            self.assertLess(body.index("g_position_servo.startMove"),
                            body.index("readAngleSensorDeg(&current_deg)"))
            failed_read = body.split("readAngleSensorDeg(&current_deg)", 1)[1]
            lost_branch = failed_read.split("setRepetitiveRunning(false)", 1)[0]
            self.assertIn("if (!g_angle_sensor.active())", lost_branch)
            self.assertIn("forceMotorSafeForSensorLoss()", lost_branch)
            self.assertIn("return", lost_branch)
            self.assertIn("setRepetitiveRunning(false)", failed_read)

    def test_pending_numeric_direction_is_resolved_only_before_prime(self):
        main = (ROOT / "src/main.cpp").read_text(encoding="utf-8")
        self.assertIn("g_pending_numeric_direction", main)
        automatic = main.rsplit("void startAutomaticPositionMove", 1)[1]
        automatic = automatic.split("\nvoid ", 1)[0]
        lost = automatic.split("if (!g_angle_sensor.active())", 1)[1]
        lost = lost.split("return", 1)[0]
        self.assertIn("Direction::ByNumericComparison", lost)
        self.assertIn("g_pending_numeric_direction = true", lost)

        recovery = main.split("consumeRecoveredEvent(&recovered_angle_deg)", 1)[1]
        recovery = recovery.split("g_position_servo.resumeAtAngle", 1)[0]
        self.assertIn("g_pending_numeric_direction", recovery)
        self.assertIn("recovered_angle_deg < g_position_servo.targetDeg()", recovery)
        self.assertIn("setPendingDirection", recovery)
        self.assertIn("MoveDirection::Clockwise", recovery)
        self.assertIn("MoveDirection::CounterClockwise", recovery)

        for cancel_name in ("stopMotorForOta", "stopAutomaticPositionMove"):
            cancel = main.rsplit(f"void {cancel_name}", 1)[1]
            cancel = cancel.split("\nvoid ", 1)[0]
            self.assertIn("g_pending_numeric_direction = false", cancel)

        self.assertIn("setPendingDirection(MoveDirection direction)", ADRC_H)
        setter = ADRC_CPP.split(
            "bool AdrcPositionController::setPendingDirection", 1)[1]
        setter = setter.split("void AdrcPositionController::", 1)[0]
        self.assertIn("!active_ || accumulated_initialized_", setter)
        self.assertIn("direction_ = direction", setter)

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


if __name__ == "__main__":
    unittest.main()
