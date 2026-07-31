#!/usr/bin/env python3
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class FixedHeightMissionContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node = (PACKAGE_ROOT / "src" / "d_task_mission_node.cpp").read_text(
            encoding="utf-8"
        )
        cls.flow = (
            PACKAGE_ROOT / "src" / "fixed_height_drop_flow.cpp"
        ).read_text(encoding="utf-8")
        cls.config = (PACKAGE_ROOT / "config" / "d_task_uav.yaml").read_text(
            encoding="utf-8"
        )

    def test_uses_ground_station_protocol_state_names(self):
        for state in (
            "MOVE_TO_SEARCH_START",
            "FORWARD_SEARCH",
            "FOLLOW_CAR",
            "RELEASE",
            "RETURN_HOME",
            "LAND_HOME",
            "COMPLETE",
            "ABORT",
        ):
            self.assertIn(f'return "{state}";', self.flow)
        for removed in ("FOLLOW_TAG", "HOLD_BEFORE_LAND", '"LANDING"'):
            self.assertNotIn(removed, self.node)

    def test_drop_progress_is_event_driven_not_fixed_follow_duration(self):
        for removed in (
            "search_timeout_s",
            "follow_duration_s",
            "pre_land_hover_s",
        ):
            self.assertNotIn(removed, self.node)
            self.assertNotIn(removed, self.config)
        self.assertIn("conditionHeld(input.aligned", self.flow)
        self.assertIn("config_.release_duration_s", self.flow)
        self.assertIn("conditionHeld(input.home_reached", self.flow)

    def test_restores_payload_return_and_terminal_protocol(self):
        for symbol in (
            '"PAYLOAD_RELEASE_TRIGGERED"',
            '"PAYLOAD_RELEASE_COMPLETE"',
            '"CAR_SPEEDUP_REQUESTED"',
            '"RETURN_HOME_STARTED"',
            '"MISSION_CONTROLLER_FINISHED"',
            'topic("mission_reset", "/uav_protocol/mission_reset")',
            'topic("local_tracking", "/uav_protocol/local_tracking")',
        ):
            self.assertIn(symbol, self.node)

    def test_payload_and_drop_gate_match_field_configuration(self):
        for setting in (
            "drop_alignment_stable_s: 1.00",
            "enabled: true",
            "backend: gpio",
            "gpio_wiringpi_pin: 9",
            "pulse_duration_s: 5.0",
        ):
            self.assertIn(setting, self.config)


if __name__ == "__main__":
    unittest.main()
