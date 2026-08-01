#!/usr/bin/env python3
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class PlatformLandingContractTests(unittest.TestCase):
    def test_platform_landed_follows_accepted_force_disarm(self):
        node = (PACKAGE_ROOT / "src" / "ego_bridge_node.cpp").read_text(
            encoding="utf-8"
        )
        platform_loop = node[node.index("void runPlatformLanding"):
                             node.index("void runPlatformLanded")]
        force_disarm_branch = platform_loop[platform_loop.index(
            "if (forceDisarmDrone()) {") :]
        self.assertIn("enterPlatformLanded();", force_disarm_branch)
        self.assertNotIn("completePlatformLanding", platform_loop)

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
        platform_loop = node[node.index("void runPlatformLanding"):
                             node.index("void runPlatformLanded")]
        self.assertNotIn("setAutoLandMode", platform_loop)
        self.assertIn("forceDisarmDrone()", platform_loop)
        self.assertNotIn("position_error_z", platform_loop)
        self.assertNotIn("platform_land_pos_deviation", platform_loop)
        self.assertNotIn("platform_min_landing_detect_time", node)


if __name__ == "__main__":
    unittest.main()
