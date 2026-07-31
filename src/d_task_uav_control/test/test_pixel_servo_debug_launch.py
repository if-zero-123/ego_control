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

        self.assertEqual(root.findall("include"), [])
        node_types = {
            node.attrib.get("type") for node in root.findall("node")
        }
        self.assertEqual(
            node_types,
            {"usb_cam_node", "apriltag_detector_node", "pixel_servo_debug_node"},
        )
        launch_xml = ET.tostring(root, encoding="unicode")
        self.assertNotIn("metal_ball_rknn", launch_xml)
        self.assertNotIn("d_task_mission_node", ET.tostring(root, encoding="unicode"))
        self.assertNotIn("ego_bridge", ET.tostring(root, encoding="unicode"))

    def test_default_configuration_uses_apriltag_only_world_projection(self):
        config_path = PACKAGE_ROOT / "config" / "d_task_uav.yaml"
        config_text = config_path.read_text()
        self.assertIn("use_visual_projection: true", config_text)
        self.assertIn("use_car_measurements: false", config_text)


if __name__ == "__main__":
    unittest.main()
