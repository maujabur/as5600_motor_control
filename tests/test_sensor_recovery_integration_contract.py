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
        for token in ("if (!active_)", "normalize360(current_deg)",
                      "shortestDelta(last_current_deg_normalized_, normalized)",
                      "profiled_target_deg_ = current_accumulated_deg_",
                      "resetObserver(current_accumulated_deg_, now_ms)",
                      "velocity_estimator_.reset()", "stall_started_ms_ = 0",
                      "profile_velocity_deg_s_ = 0.0f", "last_output_pwm_ = 0.0f"):
            self.assertIn(token, body)
        self.assertNotIn("primeAccumulatedAngle", body)
        self.assertNotIn("target_accumulated_deg_ =", body)
        self.assertNotIn("active_ = false", body)

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


if __name__ == "__main__":
    unittest.main()
