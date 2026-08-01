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
        self.assertIn("drop_min_follow_s: 1.5", self.config)

    def test_new_mission_does_not_accept_stale_hover_as_takeoff_complete(self):
        for symbol in (
            "takeoff_accepted_by_bridge_",
            'bridge_state_ == "PRE_OFFBOARD"',
            'bridge_state_ == "TAKEOFF"',
            "takeoff_height_tolerance_m_",
        ):
            self.assertIn(symbol, self.node)
        self.assertIn("takeoff_height_tolerance_m: 0.10", self.config)

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

    def test_return_home_advances_a_near_position_setpoint(self):
        self.assertIn("simple_follow/return_home_speed_mps", self.node)
        self.assertIn("return_home_speed_mps_", self.node)
        self.assertIn("return_home_speed_mps: 0.65", self.config)
        return_loop = self.node[self.node.index('state_ == "RETURN_HOME"'):
                                self.node.index('state_ == "LAND_HOME"')]
        self.assertIn("return_setpoint_initialized_", return_loop)
        self.assertIn("return_setpoint_x_", return_loop)
        self.assertIn("return_setpoint_y_", return_loop)
        self.assertIn("return_home_speed_mps_ * dt", return_loop)
        self.assertIn("sendPositionCommand(return_setpoint_x_", return_loop)
        self.assertNotIn("return_vx", return_loop)
        self.assertNotIn("return_vy", return_loop)

    def test_home_landing_uses_bridge_limited_descent(self):
        for symbol in (
            '"HOME_REACHED_BRIDGE_LIMITED_LAND"',
            '"BRIDGE_LIMITED_LAND_REQUESTED"',
            'ego_api_->requestLand();',
            'bridge_state_ == "IDLE"',
        ):
            self.assertIn(symbol, self.node)
        self.assertNotIn('request.request.custom_mode = "AUTO.LAND"', self.node)
        self.assertNotIn('px4_set_mode: /mavros/set_mode', self.config)

    def test_payload_and_drop_gate_match_field_configuration(self):
        for setting in (
            "alignment_tolerance_norm: 0.10",
            "alignment_hold_s: 0.50",
            "drop_min_follow_s: 1.5",
            "payload_release_pulse_s: 0.70",
            "wiringpi_pin: 9",
            "active_high: true",
        ):
            self.assertIn(setting, self.config)

    def test_dynamic_landing_skips_takeoff_hover_and_keeps_override_fresh(self):
        self.assertIn('if (isDropMode()) {\n                    setState("HOVER_3S"', self.node)
        self.assertIn('beginSearch(now_s);', self.node)
        self.assertIn('state_ == "PLATFORM_TAKEOFF"', self.node)
        self.assertIn('sendPositionCommand(', self.node)

    def test_dynamic_landing_blind_descend_and_speed_events_are_explicit(self):
        for symbol in (
            '"DESCEND_BLIND"',
            '"DYNAMIC_DESCENT_TAG_LOST_FINAL_SLOW"',
            '"DYNAMIC_PLATFORM_LANDED_SPEED_UP"',
            'blind_descent_x_',
            'platformZ() - landing_press_depth_m_',
        ):
            self.assertIn(symbol, self.node)
        self.assertIn(
            'state_ == "DESCEND_BLIND") {\n            if (bridge_state_ == "PLATFORM_LANDED")',
            self.node,
        )
        self.assertIn(
            '"DYNAMIC_BLIND_PLATFORM_CONTACT_TIMEOUT"',
            self.node,
        )

    def test_dynamic_landing_search_uses_035_mps_velocity_and_acquires_tag_in_motion(self):
        for symbol in (
            'simple_follow/dynamic_search_speed_mps',
            'dynamic_search_speed_mps_',
            'if (!isDropMode() && tagFresh(now_s))',
            'DYNAMIC_TAG_ACQUIRED_DURING_SEARCH',
            'dynamic_search_speed_mps_ * dx / distance',
            'dynamic_search_speed_mps_ * dy / distance',
        ):
            self.assertIn(symbol, self.node)
        self.assertIn('dynamic_search_speed_mps: 0.35', self.config)


if __name__ == "__main__":
    unittest.main()
