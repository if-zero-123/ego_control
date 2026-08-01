#!/usr/bin/env python3
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class PlatformLandingContractTests(unittest.TestCase):
    def test_platform_landed_waits_for_disarmed_state_feedback(self):
        node = (PACKAGE_ROOT / "src" / "ego_bridge_node.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("platform_disarm_requested_", node)
        self.assertIn("platform_disarm_request_time_", node)
        self.assertIn(
            "mavros_state_stamp_ >= platform_disarm_request_time_", node)
        self.assertIn("!mavros_state_.armed", node)
        self.assertIn("platform landing waiting for armed=false feedback", node)


if __name__ == "__main__":
    unittest.main()
