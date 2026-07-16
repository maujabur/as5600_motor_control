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


if __name__ == "__main__":
    unittest.main()
