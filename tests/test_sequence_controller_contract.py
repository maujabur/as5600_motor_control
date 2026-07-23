import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
HEADER_PATH = ROOT / "lib" / "sequence" / "MotionSequenceController.h"
SOURCE_PATH = ROOT / "lib" / "sequence" / "MotionSequenceController.cpp"
TYPES_PATH = ROOT / "lib" / "domain" / "MotionTypes.h"


class SequenceControllerContractTest(unittest.TestCase):
    def test_controller_has_startup_and_sixteen_loop_steps(self):
        types = TYPES_PATH.read_text(encoding="utf-8")
        self.assertIn("MOTION_SEQUENCE_MAX_STEPS = 16", types)
        self.assertIn("MotionStep startup_step", types)
        self.assertIn("MotionStep steps[MOTION_SEQUENCE_MAX_STEPS]", types)
        self.assertIn("uint8_t step_count", types)

    def test_controller_supports_three_directions_and_move_wait_phases(self):
        header = HEADER_PATH.read_text(encoding="utf-8")
        source = SOURCE_PATH.read_text(encoding="utf-8")
        types = TYPES_PATH.read_text(encoding="utf-8")
        for name in ("Shortest", "Clockwise", "CounterClockwise"):
            self.assertIn(name, types)
        for phase in ("STOPPED", "MOVING", "DWELLING"):
            self.assertIn(phase, header)
        self.assertIn("MotionExecutor& executor_", header)
        self.assertIn("next_step_index_ = 1", source)
        self.assertIn("next_step_index_ > config_.step_count", source)


if __name__ == "__main__":
    unittest.main()
