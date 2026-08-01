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

    def test_platform_contact_force_disarms_without_auto_land_handoff(self):
        node = (PACKAGE_ROOT / "src" / "ego_bridge_node.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("#include <mavros_msgs/CommandLong.h>", node)
        self.assertIn("bool forceDisarmDrone()", node)
        self.assertIn("req.request.command = 400", node)
        self.assertIn("req.request.param1 = 0.0", node)
        self.assertIn("req.request.param2 = 21196.0", node)
        self.assertIn("platform_contact_height_m", node)
        self.assertIn("platform_contact_height_tolerance_m", node)
        self.assertIn("platform_force_disarm_feedback_timeout_s", node)
        self.assertIn("platform force DISARM feedback timeout", node)

        platform_loop = node[node.index("void runPlatformLanding"):
                             node.index("void runPlatformLanded")]
        self.assertNotIn("setAutoLandMode", platform_loop)
        self.assertIn("forceDisarmDrone()", platform_loop)

        platform_cancel = node[node.index("void platformCancelCb"):
                               node.index("/// 【回调】紧急停止")]
        self.assertIn("platform_disarm_requested_", platform_cancel)


if __name__ == "__main__":
    unittest.main()
