#!/usr/bin/env python3
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_SRC = PACKAGE_ROOT.parent


class MissionConfigSenderContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gateway_source = (
            REPOSITORY_SRC
            / "uav_protocol_gateway"
            / "src"
            / "uav_protocol_gateway"
            / "gateway_core.py"
        ).read_text(encoding="utf-8")
        cls.gateway_node_source = (
            REPOSITORY_SRC
            / "uav_protocol_gateway"
            / "src"
            / "uav_protocol_gateway"
            / "gateway_node.py"
        ).read_text(encoding="utf-8")
        cls.supervisor_source = (
            PACKAGE_ROOT
            / "src"
            / "d_task_uav_control"
            / "fastlio_supervisor_node.py"
        ).read_text(encoding="utf-8")
        cls.mission_node_source = (
            PACKAGE_ROOT / "src" / "d_task_mission_node.cpp"
        ).read_text(encoding="utf-8")
        cls.mission_config_source = (
            PACKAGE_ROOT / "config" / "d_task_uav.yaml"
        ).read_text(encoding="utf-8")

    def test_all_local_consumers_authorize_car_mission_config(self):
        self.assertIn(
            'Topics.MISSION_CONFIG: ("mission_config", "car")',
            self.gateway_source,
        )
        self.assertIn('message.sender != "car"', self.supervisor_source)
        self.assertIn(
            'root.get("sender", "").asString() != "car"',
            self.mission_node_source,
        )

    def test_same_mission_new_command_restarts_positioning(self):
        branch = self.gateway_source.split(
            "if self.configured_mission_id == message.mission_id:", 1
        )[1].split("self.configured_mission_id = message.mission_id", 1)[0]
        self.assertIn("self.positioning_ready = False", branch)
        self.assertIn('CoreAction(kind="mission_config"', branch)

    def test_drop_search_path_is_takeoff_relative(self):
        for setting in (
            "initial_offset_distance_m: 0.50",
            "initial_offset_clockwise_deg: 30.0",
            "forward_search_distance_m: 1.00",
        ):
            self.assertIn(setting, self.mission_config_source)
        self.assertIn("home_yaw_", self.mission_node_source)
        self.assertIn("offset_target_x_", self.mission_node_source)
        self.assertIn("forward_target_x_", self.mission_node_source)

    def test_gateway_preserves_fixed_search_states(self):
        self.assertIn('"MOVE_TO_SEARCH_START"', self.gateway_node_source)
        self.assertIn('"FORWARD_SEARCH"', self.gateway_node_source)


if __name__ == "__main__":
    unittest.main()
