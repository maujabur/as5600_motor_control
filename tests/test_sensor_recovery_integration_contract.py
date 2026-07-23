import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
ADRC_H = (ROOT / "lib/control/AdrcPositionController.h").read_text(encoding="utf-8")
ADRC_CPP = (ROOT / "lib/control/AdrcPositionController.cpp").read_text(encoding="utf-8")
SEQUENCE_H = (ROOT / "lib/sequence/MotionSequenceController.h").read_text(encoding="utf-8")
SEQUENCE_CPP = (ROOT / "lib/sequence/MotionSequenceController.cpp").read_text(encoding="utf-8")
COORDINATOR_H = ROOT / "lib/control/MotionCoordinator.h"
COORDINATOR_CPP = ROOT / "lib/control/MotionCoordinator.cpp"
APPLICATION_H = ROOT / "src/MotorControlApplication.h"
APPLICATION_CPP = ROOT / "src/MotorControlApplication.cpp"


class SensorRecoveryIntegrationContractTest(unittest.TestCase):
    def test_adrc_resume_preserves_move_and_resets_dynamic_state(self):
        self.assertIn("resumeAtAngle(float current_deg, uint32_t now_ms)", ADRC_H)
        body = ADRC_CPP.split("void AdrcPositionController::resumeAtAngle", 1)[1]
        body = body.split("void AdrcPositionController::", 1)[0]
        for token in ("if (!active_)", "if (!accumulated_initialized_)",
                      "primeAccumulatedAngle(current_deg)",
                      "AngleMath::normalize(current_deg)",
                      "AngleMath::unwrap(current_accumulated_deg_, normalized)",
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

    def test_coordinator_owns_sensor_recovery_and_motor_safety(self):
        header = COORDINATOR_H.read_text(encoding="utf-8")
        source = COORDINATOR_CPP.read_text(encoding="utf-8")
        self.assertIn("class MotionCoordinator final : public MotionExecutor", header)
        self.assertIn("struct MotionStatus", header)
        for token in ("consumeLostEvent", "motor_.stop()",
                      "consumeRecoveredEvent", "servo_.resumeAtAngle",
                      "consumeRecoveredPauseMs", "cancelMove"):
            self.assertIn(token, source)

    def test_application_uses_coordinator_and_no_recovery_state(self):
        header = APPLICATION_H.read_text(encoding="utf-8")
        application = APPLICATION_CPP.read_text(encoding="utf-8")
        self.assertIn("AngleSensorManager sensor_", header)
        self.assertRegex(header, r"MotionCoordinator\s+motion_")
        self.assertRegex(
            header, r"MotionSequenceController\s+sequence_")
        self.assertIn("sequence_(motion_)", application)
        self.assertNotIn("As5600Sensor", application)
        self.assertNotIn("g_as5600.", application)
        for token in ("g_sensor_pause_active", "g_sensor_loss_active",
                      "forceMotorSafeForSensorLoss", "readAngleSensorDeg"):
            self.assertNotIn(token, application)

    def test_loop_updates_detection_before_motion_and_pwm(self):
        application = APPLICATION_CPP.read_text(encoding="utf-8")
        loop = application.split("void MotorControlApplication::update", 1)[1]
        order = [loop.index(token) for token in
                 ("motion_.updateSensor", "sequence_.update",
                  "motion_.updateControl")]
        self.assertEqual(order, sorted(order))

    def test_sequence_state_is_frozen_only_during_sensor_loss(self):
        source = COORDINATOR_CPP.read_text(encoding="utf-8")
        application = APPLICATION_CPP.read_text(encoding="utf-8")
        loop = application.split("void MotorControlApplication::update", 1)[1]
        self.assertIn("if (!motion_.status().paused_for_sensor)", loop)
        self.assertIn("consumeRecoveredPauseMs()", loop)
        self.assertIn("resumeAfterPause", loop)
        self.assertIn("paused_for_sensor = true", source)
        self.assertIn("paused_for_sensor = false", source)
        self.assertIn("resumeAfterPause(uint32_t paused_ms)", SEQUENCE_H)
        resume = SEQUENCE_CPP.split(
            "void MotionSequenceController::resumeAfterPause", 1)[1]
        resume = resume.split("void MotionSequenceController::", 1)[0]
        self.assertIn("if (!running_)", resume)
        self.assertIn("phase_started_ms_ += paused_ms", resume)

    def test_invalid_sensor_and_stall_force_output_zero(self):
        source = COORDINATOR_CPP.read_text(encoding="utf-8")
        for token in ("!sensor_.active()", "!sensor_.readAngleDeg(&angle)",
                      "servo_.isStalled()", "status_.stalled = true"):
            self.assertIn(token, source)
        self.assertGreaterEqual(source.count("motor_.stop()"), 3)

    def test_transient_step_start_failure_keeps_move_and_sequence_pending(self):
        source = COORDINATOR_CPP.read_text(encoding="utf-8")
        manager = (ROOT / "lib/hardware/AngleSensorManager.h").read_text(
            encoding="utf-8")
        self.assertIn("failure_limit_ = 3", manager)
        body = source.split("bool MotionCoordinator::startMove", 1)[1]
        body = body.split("bool MotionCoordinator::retargetMove", 1)[0]
        self.assertLess(body.index("servo_.startMove"),
                        body.index("sensor_.readAngleDeg"))
        self.assertNotIn("servo_.cancel()", body)

    def test_failure_limit_is_persisted_and_validated(self):
        application = APPLICATION_CPP.read_text(encoding="utf-8")
        page = (ROOT / "lib/web/control_settings_web_page.h").read_text(encoding="utf-8")
        settings = (ROOT / "lib/settings/DeviceSettings.h").read_text(
            encoding="utf-8")
        web = (ROOT / "lib/web/WebControlServer.cpp").read_text(encoding="utf-8")
        for token in ('parseNumber("sensorFailures"',
                      "candidate.sensor.failure_limit =",
                      "(uint8_t)lroundf(sensor_failures)"):
            self.assertIn(token, web)
        self.assertIn(
            "sensor_.setFailureLimit(settings_.sensor.failure_limit)",
            application)
        self.assertIn("SensorSettings sensor", settings)
        self.assertIn("value.sensor.failure_limit >= 1", settings)
        self.assertIn('id="sensorFailures"', page)
        self.assertIn("'sensorFailures'", page)

    def test_status_api_exposes_sensor_identity_and_state(self):
        web = (ROOT / "lib/web/WebControlServer.cpp").read_text(encoding="utf-8")
        for field in ('"sensorType"', '"sensorAddress"', '"sensorState"',
                      '"sensorFailures"'):
            self.assertIn(field, web.replace('\\"', '"'))

    def test_main_page_distinguishes_detection_and_reconnection(self):
        page = (ROOT / "lib/web/repetitive_motion_web_page.h").read_text(encoding="utf-8")
        self.assertIn("DETECTANDO SENSOR", page)
        self.assertIn("RECONECTANDO SENSOR", page)
        self.assertIn("LEITURA INDISPONIVEL", page)
        self.assertIn("sensorState", page)

    def test_late_detection_logs_sensor_identity(self):
        source = COORDINATOR_CPP.read_text(encoding="utf-8")
        self.assertIn("AngleSensorManager::State::Detecting", source)
        self.assertIn("sensor_.sensorName()", source)
        self.assertIn("sensor_.sensorAddress()", source)


if __name__ == "__main__":
    unittest.main()
