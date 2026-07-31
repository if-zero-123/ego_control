#!/usr/bin/env python3

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_range_debug_launch_is_read_only():
    root = ET.parse(
        PACKAGE_ROOT / 'launch' / 'apriltag_range_debug.launch'
    ).getroot()
    node_types = {
        node.attrib.get('type')
        for node in root.findall('.//node')
    }

    assert 'usb_cam_node' in node_types
    assert 'apriltag_detector_node' in node_types
    assert 'ego_bridge_node' not in node_types
    assert 'd_task_mission_node' not in node_types
    assert 'payload_gpio_test_node.py' not in node_types


def test_range_debug_launch_keeps_physical_tag_size_explicit():
    root = ET.parse(
        PACKAGE_ROOT / 'launch' / 'apriltag_range_debug.launch'
    ).getroot()
    arguments = {
        argument.attrib['name']: argument.attrib.get('default')
        for argument in root.findall('arg')
    }

    assert arguments['apriltag_id'] == '0'
    assert arguments['apriltag_size_m'] == '0.080'
