#!/usr/bin/env python3
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class MissionDiagnosticsContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (PACKAGE_ROOT / "src" / "d_task_mission_node.cpp").read_text(
            encoding="utf-8"
        )
        cls.config = (PACKAGE_ROOT / "config" / "d_task_uav.yaml").read_text(
            encoding="utf-8"
        )

    def test_control_loop_reuses_one_input_for_control_and_diagnostics(self):
        callback = self.source.split("void controlTimerCallback", 1)[1].split(
            "void executeCommand", 1
        )[0]
        self.assertIn("const MissionInput input = buildInput(now_s);", callback)
        self.assertIn("controller_.update(input)", callback)
        self.assertIn("logDiagnosticSnapshot(input, command", callback)
        self.assertNotIn("controller_.update(buildInput(now_s))", callback)

    def test_logs_state_transitions_faults_and_periodic_snapshot(self):
        self.assertIn("[d_task_mission][STATE]", self.source)
        self.assertIn("[d_task_mission][FAULT]", self.source)
        self.assertIn("[d_task_mission][DIAG]", self.source)
        for field in (
            "search_start_error_m",
            "search_end_error_m",
            "platform_vision",
            "pixel_aligned",
            "apriltag_center",
            "apriltag_fresh",
            "apriltag_age_s",
            "bridge_state_fresh",
            "bridge_state_age_s",
            "control_mode_fresh",
            "raw_detection_measurement_age_s",
            "setpoint_valid",
            "distance_to_d_m",
        ):
            self.assertIn(field, self.source)

    def test_diagnostic_rate_is_configurable_and_enabled_at_one_hz(self):
        self.assertIn('"diagnostics/log_rate_hz"', self.source)
        self.assertIn('"diagnostics/source_timeout_s"', self.source)
        self.assertIn("diagnostics:\n  log_rate_hz: 1.0", self.config)

    def test_fault_deduplication_resets_after_fault_free_control_cycle(self):
        callback = self.source.split("void controlTimerCallback", 1)[1].split(
            "void executeCommand", 1
        )[0]
        self.assertIn("if (command.fault_code == 0)", callback)
        self.assertIn("last_fault_key_.clear();", callback)


if __name__ == "__main__":
    unittest.main()
