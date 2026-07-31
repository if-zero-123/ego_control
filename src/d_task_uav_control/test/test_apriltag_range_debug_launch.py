#!/usr/bin/env python3

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


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


def test_range_debug_launch_loads_measured_camera_info():
    root = ET.parse(
        PACKAGE_ROOT / 'launch' / 'apriltag_range_debug.launch'
    ).getroot()
    parameters = {
        parameter.attrib['name']: parameter.attrib.get('value')
        for parameter in root.findall('.//param')
    }

    assert parameters['camera_name'] == 'head_camera'
    assert parameters['camera_info_url'].endswith(
        '/config/decxin_640x480.yaml'
    )


def test_full_task_launch_injects_the_same_camera_info():
    root = ET.parse(PACKAGE_ROOT / 'launch' / 'd_task_uav.launch').getroot()
    parameters = {
        parameter.attrib['name']: parameter.attrib.get('value')
        for parameter in root.findall('.//param')
    }

    assert parameters['/usb_camera_vision/usb_cam/camera_name'] == 'head_camera'
    assert parameters['/usb_camera_vision/usb_cam/camera_info_url'].endswith(
        '/config/decxin_640x480.yaml'
    )


def test_measured_intrinsics_match_half_meter_tag_sample():
    calibration = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'decxin_640x480.yaml').read_text()
    )
    matrix = calibration['camera_matrix']['data']
    distortion = calibration['distortion_coefficients']['data']

    assert calibration['image_width'] == 640
    assert calibration['image_height'] == 480
    assert abs(matrix[0] - 909.375000) < 1e-6
    assert abs(matrix[4] - 909.418101) < 1e-6
    assert matrix[2] == 319.5
    assert matrix[5] == 239.5
    assert distortion == [0.0, 0.0, 0.0, 0.0, 0.0]


def test_apriltag_prediction_is_bounded_and_explicitly_marked():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'd_task_uav.yaml').read_text()
    )
    apriltag = config['apriltag']
    message_definition = (
        PACKAGE_ROOT / 'msg' / 'PlatformDetection.msg'
    ).read_text()

    assert 0.0 < apriltag['prediction_timeout_s'] <= 0.20
    assert 0.0 < apriltag['track_filter_alpha'] <= 1.0
    assert 0.0 <= apriltag['track_filter_beta'] <= 1.0
    assert apriltag['max_velocity_px_s'] > 0.0
    assert apriltag['reacquire_distance_px'] > 0.0
    assert 'bool predicted' in message_definition
    assert 'float32 measurement_age_s' in message_definition
