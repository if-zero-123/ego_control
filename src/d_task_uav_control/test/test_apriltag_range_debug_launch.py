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


def test_range_debug_launch_uses_multitag_layout_from_config():
    root = ET.parse(
        PACKAGE_ROOT / 'launch' / 'apriltag_range_debug.launch'
    ).getroot()
    detector = next(
        node for node in root.findall('.//node')
        if node.attrib.get('type') == 'apriltag_detector_node'
    )
    rosparams = detector.findall('rosparam')
    assert any(
        item.attrib.get('command') == 'load'
        and item.attrib.get('file') == '$(arg config_file)'
        for item in rosparams
    )


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


def test_multitag_layout_image_pipeline_and_range_message_are_explicit():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'd_task_uav.yaml').read_text()
    )
    apriltag = config['apriltag']
    layout = {entry['id']: entry for entry in apriltag['layout']}

    assert layout == {
        0: {'id': 0, 'size_m': 0.045, 'x_m': 0.0, 'y_m': 0.0,
            'yaw_rad': 0.0},
        1: {'id': 1, 'size_m': 0.120, 'x_m': -0.195, 'y_m': 0.195,
            'yaw_rad': 0.0},
        2: {'id': 2, 'size_m': 0.120, 'x_m': 0.195, 'y_m': 0.195,
            'yaw_rad': 0.0},
        3: {'id': 3, 'size_m': 0.120, 'x_m': 0.195, 'y_m': -0.195,
            'yaw_rad': 0.0},
        4: {'id': 4, 'size_m': 0.120, 'x_m': -0.195, 'y_m': -0.195,
            'yaw_rad': 0.0},
    }
    assert apriltag['full_frame_interval'] == 10
    assert apriltag['roi_expand_scale'] >= 1.0
    assert apriltag['clahe_fallback'] is True
    assert 0.0 < apriltag['range_filter_alpha'] <= 1.0

    range_message = (PACKAGE_ROOT / 'msg' / 'AprilTagRange.msg').read_text()
    assert 'bool center_tag_visible' in range_message
    assert 'int32[] visible_tag_ids' in range_message
    assert 'int32[] used_tag_ids' in range_message
    assert 'float64 raw_plane_distance_m' in range_message


def test_full_task_launch_starts_camera_and_apriltag_without_yolo_or_mux():
    root = ET.parse(PACKAGE_ROOT / 'launch' / 'd_task_uav.launch').getroot()
    node_types = {
        node.attrib.get('type')
        for node in root.findall('.//node')
    }
    include_files = {
        include.attrib.get('file', '')
        for include in root.findall('.//include')
    }

    assert 'usb_cam_node' in node_types
    assert 'apriltag_detector_node' in node_types
    assert 'platform_detection_mux_node' not in node_types
    assert not any('metal_ball_rknn' in path for path in include_files)
