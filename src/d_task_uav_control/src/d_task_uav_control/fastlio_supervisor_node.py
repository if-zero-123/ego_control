"""Restart one owned FAST-LIO launch and publish readiness for mission start."""

from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time
from typing import List, Optional

import rosnode
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from d_task_protocol import ProtocolCodec, ProtocolError

from .positioning_readiness import (
    PositioningReadiness,
    PositioningReadinessConfig,
    build_roslaunch_command,
)


class FastlioSupervisor:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._armed = False
        self._process: Optional[subprocess.Popen] = None
        self._last_status_json = ""
        self._launch_package = rospy.get_param(
            "~positioning/launch_package", "lidar_to_mavros"
        )
        self._launch_file = rospy.get_param(
            "~positioning/launch_file", "fastlio_to_px4_mid360_direct.launch"
        )
        self._launch_args = list(
            rospy.get_param(
                "~positioning/launch_args", ["rviz:=false", "zero_origin:=true"]
            )
        )
        self._workspace_env = rospy.get_param(
            "~positioning/workspace_env", "/home/orangepi/ros_ws/devel/env.sh"
        )
        self._stop_timeout_s = float(
            rospy.get_param("~positioning/stop_timeout_s", 5.0)
        )
        self._conflict_nodes = set(
            rospy.get_param(
                "~positioning/conflict_nodes", ["laserMapping", "lidar_to_mavros"]
            )
        )
        self._readiness = PositioningReadiness(
            PositioningReadinessConfig(
                startup_timeout_s=float(
                    rospy.get_param("~positioning/startup_timeout_s", 15.0)
                ),
                stable_time_s=float(
                    rospy.get_param("~positioning/stable_time_s", 3.0)
                ),
                message_timeout_s=float(
                    rospy.get_param("~positioning/message_timeout_s", 0.3)
                ),
                max_xy_drift_m=float(
                    rospy.get_param("~positioning/max_xy_drift_m", 0.05)
                ),
                max_z_drift_m=float(
                    rospy.get_param("~positioning/max_z_drift_m", 0.05)
                ),
            )
        )

        self._status_pub = rospy.Publisher(
            rospy.get_param("~topics/status", "/d_task/positioning/status"),
            String,
            queue_size=5,
            latch=True,
        )
        rospy.Subscriber(
            rospy.get_param("~topics/mission_config", "/uav_protocol/mission_config"),
            String,
            self._mission_config_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~topics/fastlio_odom", "/Odometry"),
            Odometry,
            self._fastlio_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~topics/vision_pose", "/mavros/vision_pose/pose"),
            PoseStamped,
            self._vision_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~topics/px4_odom", "/mavros/local_position/odom"),
            Odometry,
            self._px4_odom_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~topics/mavros_state", "/mavros/state"),
            State,
            self._mavros_state_cb,
            queue_size=5,
        )
        self._timer = rospy.Timer(rospy.Duration(0.05), self._timer_cb)
        rospy.on_shutdown(self.shutdown)
        self._publish_status(force=True)

    def _mission_config_cb(self, raw: String) -> None:
        try:
            message = ProtocolCodec.decode(raw.data)
            if message.type != "mission_config" or message.sender != "car":
                raise ProtocolError("expected car mission_config")
        except (ProtocolError, TypeError, UnicodeDecodeError) as exc:
            rospy.logwarn("[fastlio_supervisor] invalid mission config: %s", exc)
            return

        with self._lock:
            self._readiness.begin(
                message.mission_id, message.payload["mode"], rospy.Time.now().to_sec()
            )
            self._publish_status(force=True)
            if self._armed:
                self._readiness.fail(1303, "cannot_restart_positioning_while_armed")
                self._publish_status(force=True)
                return
            self._stop_owned_launch()
            if not self._wait_for_conflicts_clear():
                self._readiness.fail(1302, "fastlio_node_conflict")
                self._publish_status(force=True)
                return
            try:
                self._start_launch()
            except (OSError, ValueError) as exc:
                self._readiness.fail(1304, "fastlio_launch_failed: " + str(exc))
                self._publish_status(force=True)

    def _start_launch(self) -> None:
        if self._workspace_env and not os.access(self._workspace_env, os.X_OK):
            raise OSError(
                "positioning workspace env is not executable: "
                + self._workspace_env
            )
        command = build_roslaunch_command(
            self._workspace_env,
            self._launch_package,
            self._launch_file,
            self._launch_args,
        )
        rospy.loginfo("[fastlio_supervisor] starting: %s", " ".join(command))
        self._process = subprocess.Popen(command, start_new_session=True)

    def _stop_owned_launch(self) -> None:
        process = self._process
        self._process = None
        if process is None or process.poll() is not None:
            return
        rospy.loginfo("[fastlio_supervisor] stopping previous FAST-LIO launch")
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=self._stop_timeout_s)
        except subprocess.TimeoutExpired:
            rospy.logwarn("[fastlio_supervisor] SIGINT timeout, sending SIGTERM")
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                rospy.logerr("[fastlio_supervisor] SIGTERM timeout, sending SIGKILL")
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=1.0)

    def _wait_for_conflicts_clear(self) -> bool:
        deadline = time.monotonic() + self._stop_timeout_s
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if not self._active_conflicts():
                return True
            rospy.sleep(0.1)
        return not self._active_conflicts()

    def _active_conflicts(self) -> List[str]:
        try:
            names = rosnode.get_node_names()
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "[fastlio_supervisor] ROS node query failed: %s", exc)
            return ["ros_master_unavailable"]
        return [
            name for name in names if name.rsplit("/", 1)[-1] in self._conflict_nodes
        ]

    def _fastlio_cb(self, message: Odometry) -> None:
        with self._lock:
            self._readiness.observe_fastlio(self._stamp(message.header.stamp))

    def _vision_cb(self, message: PoseStamped) -> None:
        with self._lock:
            self._readiness.observe_vision(self._stamp(message.header.stamp))

    def _px4_odom_cb(self, message: Odometry) -> None:
        position = message.pose.pose.position
        with self._lock:
            self._readiness.observe_px4(
                self._stamp(message.header.stamp), position.x, position.y, position.z
            )

    def _mavros_state_cb(self, message: State) -> None:
        with self._lock:
            self._armed = bool(message.armed)

    def _timer_cb(self, _event) -> None:
        with self._lock:
            if self._process is not None and self._process.poll() is not None:
                code = self._process.returncode
                self._process = None
                self._readiness.fail(1305, "fastlio_launch_exited: " + str(code))
            self._readiness.evaluate(rospy.Time.now().to_sec())
            self._publish_status()

    def _publish_status(self, force: bool = False) -> None:
        payload = self._readiness.status.to_dict()
        raw = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        if force or raw != self._last_status_json:
            self._status_pub.publish(String(data=raw))
            self._last_status_json = raw

    @staticmethod
    def _stamp(stamp: rospy.Time) -> float:
        return rospy.Time.now().to_sec() if stamp.is_zero() else stamp.to_sec()

    def shutdown(self) -> None:
        with self._lock:
            self._stop_owned_launch()


def main() -> None:
    rospy.init_node("fastlio_supervisor")
    FastlioSupervisor()
    rospy.spin()
