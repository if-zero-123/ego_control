#!/usr/bin/env python3

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class Tag0OffsetCalibrationLaunchTests(unittest.TestCase):
    def test_launch_contains_only_camera_and_calibrator(self):
        root = ET.parse(
            PACKAGE_ROOT / "launch" / "tag0_offset_calibration.launch"
        ).getroot()
        node_types = {
            node.attrib.get("type") for node in root.findall(".//node")
        }

        self.assertEqual(
            node_types,
            {"usb_cam_node", "calibrate_tag0_offset.py"},
        )
        launch_xml = ET.tostring(root, encoding="unicode")
        for forbidden in (
            "ego_bridge",
            "d_task_mission_node",
            "uav_protocol_gateway",
            "mavros",
            "fastlio",
        ):
            self.assertNotIn(forbidden, launch_xml)

    def test_calibrator_is_required_and_uses_external_backup_directory(self):
        root = ET.parse(
            PACKAGE_ROOT / "launch" / "tag0_offset_calibration.launch"
        ).getroot()
        calibrator = next(
            node for node in root.findall(".//node")
            if node.attrib.get("type") == "calibrate_tag0_offset.py"
        )
        parameters = {
            parameter.attrib["name"]: parameter.attrib.get("value")
            for parameter in calibrator.findall("param")
        }

        self.assertEqual(calibrator.attrib.get("required"), "true")
        self.assertIn("/.ros/d_task_uav/calibration_backups", parameters["backup_root"])
        self.assertEqual(parameters["config_file"], "$(arg config_file)")


if __name__ == "__main__":
    unittest.main()
