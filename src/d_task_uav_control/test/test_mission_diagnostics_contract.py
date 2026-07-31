#!/usr/bin/env python3
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class CompetitionMissionContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node = (PACKAGE_ROOT / "src" / "d_task_mission_node.cpp").read_text(
            encoding="utf-8"
        )
        cls.config = (PACKAGE_ROOT / "config" / "d_task_uav.yaml").read_text(
            encoding="utf-8"
        )

    def test_uses_ground_station_protocol_state_names(self):
        for state in (
            "HOVER_3S",
            "SEARCH_CAR",
            "FOLLOW_CAR",
            "RELEASE",
            "RETURN_HOME",
            "LAND_HOME",
            "COMPLETE",
            "ABORT",
        ):
            self.assertIn(f'"{state}"', self.node)
        for removed in ("FOLLOW_TAG", "HOLD_BEFORE_LAND"):
            self.assertNotIn(removed, self.node)

    def test_drop_gate_uses_follow_alignment_and_release_pulse(self):
        self.assertIn("follow_start_s_", self.node)
        self.assertIn("alignedFor(now_s, alignment_hold_s_)", self.node)
        self.assertIn("payload_release_pulse_s_", self.node)
        self.assertIn("hover_time_s: 3.0", self.config)
        self.assertIn("drop_min_follow_s: 3.0", self.config)

    def test_restores_payload_return_and_terminal_protocol(self):
        for symbol in (
            '"PAYLOAD_RELEASE_STARTED"',
            '"PAYLOAD_RELEASED_SPEED_UP"',
            '"PAYLOAD_RELEASED_RETURN_HOME"',
            '"MISSION_COMPLETE"',
            '"MISSION_ABORTED"',
            'topic("payload_release", "/d_task/payload/release")',
            'topic("local_tracking", "/uav_protocol/local_tracking")',
        ):
            self.assertIn(symbol, self.node)

    def test_payload_and_drop_gate_match_field_configuration(self):
        for setting in (
            "alignment_tolerance_norm: 0.10",
            "alignment_hold_s: 0.50",
            "drop_min_follow_s: 3.0",
            "payload_release_pulse_s: 0.70",
            "wiringpi_pin: 9",
            "active_high: true",
        ):
            self.assertIn(setting, self.config)


if __name__ == "__main__":
    unittest.main()
