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
        cls.supervisor_source = (
            PACKAGE_ROOT
            / "src"
            / "d_task_uav_control"
            / "fastlio_supervisor_node.py"
        ).read_text(encoding="utf-8")
        cls.mission_node_source = (
            PACKAGE_ROOT / "src" / "d_task_mission_node.cpp"
        ).read_text(encoding="utf-8")

    def test_gateway_authorizes_car_mission_config(self):
        self.assertIn(
            'Topics.MISSION_CONFIG: ("mission_config", "car")',
            self.gateway_source,
        )

    def test_fastlio_supervisor_uses_gateway_sender_contract(self):
        callback = self.supervisor_source.split(
            "def _mission_config_cb", 1
        )[1].split("def _start_launch", 1)[0]

        self.assertIn('message.sender != "car"', callback)
        self.assertNotIn('message.sender != "ground"', callback)

    def test_mission_node_uses_gateway_sender_contract(self):
        callback = self.mission_node_source.split(
            "void missionConfigCallback", 1
        )[1].split("void missionStartCallback", 1)[0]

        self.assertIn('root.get("sender", "").asString() != "car"', callback)
        self.assertNotIn('root.get("sender", "").asString() != "ground"', callback)


if __name__ == "__main__":
    unittest.main()
