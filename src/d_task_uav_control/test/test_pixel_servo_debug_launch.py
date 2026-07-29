#!/usr/bin/env python3
"""Launch-file contract for the no-control pixel-servo bench test."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class PixelServoDebugLaunchTests(unittest.TestCase):
    def test_launch_starts_vision_and_only_the_debug_servo_node(self):
        launch_path = PACKAGE_ROOT / "launch" / "pixel_servo_debug.launch"
        root = ET.parse(launch_path).getroot()

        includes = root.findall("include")
        self.assertEqual(len(includes), 1)
        self.assertIn("platform_target_test.launch", includes[0].attrib["file"])
        nodes = root.findall("node")
        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].attrib["type"], "pixel_servo_debug_node")
        self.assertNotIn("d_task_mission_node", ET.tostring(root, encoding="unicode"))
        self.assertNotIn("ego_bridge", ET.tostring(root, encoding="unicode"))

    def test_default_configuration_disables_legacy_visual_projection(self):
        config_path = PACKAGE_ROOT / "config" / "d_task_uav.yaml"
        self.assertIn("use_visual_projection: false", config_path.read_text())


if __name__ == "__main__":
    unittest.main()
