#!/usr/bin/env python3
"""Measure rectified Tag 0 centre and atomically save the servo offset."""

from __future__ import annotations

from collections import deque
import json
import math
import sys
from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from d_task_uav_control.tag0_offset_calibration import (
    UnstableCalibrationError,
    calculate_offset,
    update_offset_config,
)


class Tag0OffsetCalibrationNode:
    def __init__(self) -> None:
        self.succeeded = False
        self.failed = False
        self.bridge = CvBridge()
        self.tag_id = int(rospy.get_param("~tag_id", 0))
        self.sample_count = int(rospy.get_param("~sample_count", 90))
        self.max_stddev_px = float(rospy.get_param("~max_stddev_px", 1.5))
        self.min_tag_side_px = float(rospy.get_param("~min_tag_side_px", 8.0))
        self.timeout_s = float(rospy.get_param("~timeout_s", 45.0))
        self.config_file = Path(rospy.get_param("~config_file")).expanduser()
        self.backup_root = Path(rospy.get_param("~backup_root")).expanduser()
        if self.sample_count < 10:
            raise ValueError("sample_count must be at least 10")
        if self.max_stddev_px <= 0.0 or self.timeout_s <= 0.0:
            raise ValueError("stability and timeout parameters must be positive")

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_APRILTAG_36h11
        )
        self.detector_parameters = cv2.aruco.DetectorParameters_create()
        self.detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector_parameters.minMarkerPerimeterRate = 0.02
        self.camera_matrix = None
        self.distortion = None
        self.camera_width = 0
        self.camera_height = 0
        self.samples = deque(maxlen=self.sample_count)
        self.principal = None
        self.started_at = rospy.Time.now()

        image_topic = rospy.get_param(
            "~image_topic", "/tag0_offset_calibration/usb_cam/image_raw"
        )
        camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/tag0_offset_calibration/usb_cam/camera_info"
        )
        self.result_publisher = rospy.Publisher(
            "~result", String, queue_size=1, latch=True
        )
        rospy.Subscriber(camera_info_topic, CameraInfo, self._camera_info_callback, queue_size=1)
        rospy.Subscriber(image_topic, Image, self._image_callback, queue_size=1)
        self.timeout_timer = rospy.Timer(rospy.Duration(0.2), self._timeout_callback)
        rospy.loginfo(
            "[tag0_offset_calibration] waiting for ID %d: samples=%d max_stddev=%.2fpx",
            self.tag_id,
            self.sample_count,
            self.max_stddev_px,
        )

    def _camera_info_callback(self, message: CameraInfo) -> None:
        matrix = np.asarray(message.K, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(message.D, dtype=np.float64)
        if (
            message.width <= 0
            or message.height <= 0
            or not np.all(np.isfinite(matrix))
            or not np.all(np.isfinite(distortion))
            or matrix[0, 0] <= 0.0
            or matrix[1, 1] <= 0.0
        ):
            rospy.logwarn_throttle(2.0, "[tag0_offset_calibration] invalid CameraInfo")
            return
        self.camera_matrix = matrix
        self.distortion = distortion
        self.camera_width = int(message.width)
        self.camera_height = int(message.height)

    def _image_callback(self, message: Image) -> None:
        if self.succeeded or self.failed or self.camera_matrix is None:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, "[tag0_offset_calibration] image error: %s", error)
            return
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_parameters
        )
        if ids is None:
            rospy.loginfo_throttle(1.0, "[tag0_offset_calibration] waiting for ID %d", self.tag_id)
            return

        selected = None
        for index, detected_id in enumerate(ids.flatten().tolist()):
            if detected_id == self.tag_id:
                selected = np.asarray(corners[index], dtype=np.float32).reshape(4, 2)
                break
        if selected is None:
            return
        sides = [
            float(np.linalg.norm(selected[(index + 1) % 4] - selected[index]))
            for index in range(4)
        ]
        if min(sides) < self.min_tag_side_px:
            rospy.logwarn_throttle(1.0, "[tag0_offset_calibration] ID %d is too small", self.tag_id)
            return

        image_height, image_width = gray.shape[:2]
        scale_x = float(image_width) / float(self.camera_width)
        scale_y = float(image_height) / float(self.camera_height)
        scaled_matrix = self.camera_matrix.copy()
        scaled_matrix[0, 0] *= scale_x
        scaled_matrix[0, 2] *= scale_x
        scaled_matrix[1, 1] *= scale_y
        scaled_matrix[1, 2] *= scale_y
        centre = np.mean(selected, axis=0).reshape(1, 1, 2)
        rectified = cv2.undistortPoints(
            centre,
            scaled_matrix,
            self.distortion,
            P=scaled_matrix,
        ).reshape(2)
        principal = (float(scaled_matrix[0, 2]), float(scaled_matrix[1, 2]))
        if self.principal is not None and (
            not math.isclose(self.principal[0], principal[0], abs_tol=1e-6)
            or not math.isclose(self.principal[1], principal[1], abs_tol=1e-6)
        ):
            self.samples.clear()
        self.principal = principal
        self.samples.append((float(rectified[0]), float(rectified[1])))
        rospy.loginfo_throttle(
            1.0,
            "[tag0_offset_calibration] collecting %d/%d rectified=(%.2f, %.2f)",
            len(self.samples),
            self.sample_count,
            rectified[0],
            rectified[1],
        )
        if len(self.samples) < self.sample_count:
            return
        try:
            result = calculate_offset(
                self.samples,
                principal_u=self.principal[0],
                principal_v=self.principal[1],
                max_stddev_px=self.max_stddev_px,
            )
        except UnstableCalibrationError as error:
            rospy.logwarn_throttle(1.0, "[tag0_offset_calibration] %s; keep camera still", error)
            return
        try:
            backup_path = update_offset_config(
                self.config_file,
                offset_u_px=result.offset_u_px,
                offset_v_px=result.offset_v_px,
                backup_root=self.backup_root,
            )
        except (OSError, ValueError) as error:
            self.failed = True
            rospy.logerr("[tag0_offset_calibration] config update failed: %s", error)
            rospy.signal_shutdown("calibration config update failed")
            return

        payload = {
            "sample_count": result.sample_count,
            "observed_u": result.observed_u,
            "observed_v": result.observed_v,
            "offset_u_px": result.offset_u_px,
            "offset_v_px": result.offset_v_px,
            "stddev_u_px": result.stddev_u_px,
            "stddev_v_px": result.stddev_v_px,
            "config_file": str(self.config_file),
            "backup_file": str(backup_path),
        }
        self.result_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))
        self.succeeded = True
        rospy.loginfo("[tag0_offset_calibration] saved: %s", json.dumps(payload, sort_keys=True))
        rospy.signal_shutdown("tag offset calibration complete")

    def _timeout_callback(self, _event) -> None:
        if self.succeeded or self.failed:
            return
        if (rospy.Time.now() - self.started_at).to_sec() <= self.timeout_s:
            return
        self.failed = True
        rospy.logerr(
            "[tag0_offset_calibration] timeout after %.1fs with %d/%d samples",
            self.timeout_s,
            len(self.samples),
            self.sample_count,
        )
        rospy.signal_shutdown("tag offset calibration timeout")


def main() -> int:
    rospy.init_node("tag0_offset_calibration")
    try:
        node = Tag0OffsetCalibrationNode()
    except (KeyError, OSError, ValueError) as error:
        rospy.logfatal("[tag0_offset_calibration] startup failed: %s", error)
        return 2
    rospy.spin()
    return 0 if node.succeeded or not node.failed else 2


if __name__ == "__main__":
    sys.exit(main())
