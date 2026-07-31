#!/usr/bin/env python3
"""ROS1 <-> MQTT gateway for the D-task UAV agent."""

from __future__ import annotations

import json
import math
import queue
import threading
import time
from typing import Any, Dict, Optional

import rospy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Image
from std_msgs.msg import Bool, String, UInt8

from d_task_protocol import (
    EnvelopeFactory,
    MessageGuard,
    Mode,
    ProtocolEndpoint,
    build_event,
    build_fault,
    build_health,
    build_heartbeat,
    build_tracking,
    build_uav_state,
)
from d_task_protocol.mqtt_bus import MqttBus, MqttConfig
from d_task_protocol.topics import Topics

from .gateway_core import (
    CoreAction,
    UavGatewayCore,
    follow_established_after_state,
    normalise_local_event,
)


_UAV_STATES = {
    "NOT_READY",
    "POSITIONING_INIT",
    "POSITIONING_FAULT",
    "WAIT_START",
    "READY",
    "TAKEOFF",
    "HOVER_3S",
    "MOVE_TO_SEARCH_START",
    "FORWARD_SEARCH",
    "SEARCH_CAR",
    "LOCK_CAR",
    "FOLLOW_CAR",
    "DROP_DESCEND",
    "RELEASE",
    "RETURN_HOME",
    "DESCEND_HIGH",
    "DESCEND_LOW",
    "LAND_ON_PLATFORM",
    "PLATFORM_HOLD",
    "PLATFORM_TAKEOFF",
    "CLIMB_TO_CRUISE",
    "LAND_HOME",
    "COMPLETE",
    "ABORT",
}

_EGO_BRIDGE_STATE_MAP = {
    "IDLE": "READY",
    "PRE_OFFBOARD": "TAKEOFF",
    "TAKEOFF": "TAKEOFF",
    "HOVER": "READY",
    "TRACKING": "FOLLOW_CAR",
    "LANDING": "LAND_HOME",
    "PLATFORM_LANDING": "LAND_ON_PLATFORM",
    "PLATFORM_LANDED": "PLATFORM_HOLD",
    "PLATFORM_PRE_OFFBOARD": "PLATFORM_TAKEOFF",
    "PLATFORM_TAKEOFF": "PLATFORM_TAKEOFF",
}


def _now_ms() -> int:
    return time.time_ns() // 1_000_000


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    message = Quaternion()
    message.z = math.sin(yaw * 0.5)
    message.w = math.cos(yaw * 0.5)
    return message


def _json_object(raw: str) -> Dict[str, Any]:
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise ValueError("JSON value must be an object")
    if isinstance(value.get("payload"), dict):
        return dict(value["payload"])
    return value


class UavProtocolGateway:
    """Own ROS subscriptions, MQTT transport, and periodic UAV telemetry."""

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.started_monotonic = time.monotonic()

        # The role is intentionally explicit: all outbound messages originate
        # from one process-stable UAV envelope factory.
        self.factory = EnvelopeFactory("uav")
        self.guard = MessageGuard()
        self.car_pose_timeout_ms = int(rospy.get_param("~car_pose_timeout_ms", 300))
        self.core = UavGatewayCore(
            factory=self.factory,
            guard=self.guard,
            car_pose_timeout_ms=self.car_pose_timeout_ms,
        )

        self.mqtt_config = MqttConfig(
            host=rospy.get_param("~mqtt_host", "192.168.0.198"),
            port=int(rospy.get_param("~mqtt_port", 1883)),
            keepalive=int(rospy.get_param("~mqtt_keepalive", 10)),
            client_id=rospy.get_param("~mqtt_client_id", "d-task-uav"),
            username=rospy.get_param("~mqtt_username", "") or None,
            password=rospy.get_param("~mqtt_password", "") or None,
        )

        self.state = "NOT_READY"
        self.mode = Mode.DROP.value
        self.control_mode = "EGO"
        self.px4_state = "UNKNOWN"
        self.armed = False
        self.battery_percent = 0.0
        self.odom: Optional[Odometry] = None
        self.last_odom_ms: Optional[int] = None
        self.last_px4_state_ms: Optional[int] = None
        self.last_camera_ms: Optional[int] = None
        self.last_tracking_ms: Optional[int] = None
        self.last_task_state_ms: Optional[int] = None
        self.follow_established = False
        self.last_camera_health: Optional[bool] = None
        self.last_vision_health: Optional[bool] = None
        self.task_health: Dict[str, Any] = {}
        self.tracking: Dict[str, Any] = self._default_tracking()
        self.last_car_health: Dict[str, Any] = {}
        self.last_car_heartbeat_ms: Optional[int] = None
        self.fault_code = 0
        self.fault_text = ""
        self._stale_pose_fault_sent = False
        self._last_positioning_state = ""
        self._mqtt_inbox = queue.SimpleQueue()
        self._mqtt_outbox = queue.SimpleQueue()

        self._advertise_ros()
        self._subscribe_ros()
        self._connect_mqtt()

        self.mqtt_timer = rospy.Timer(
            rospy.Duration(0.01), self._mqtt_io_timer_cb)
        self.state_timer = rospy.Timer(rospy.Duration(0.2), self._state_timer_cb)
        self.tracking_timer = rospy.Timer(rospy.Duration(0.2), self._tracking_timer_cb)
        self.health_timer = rospy.Timer(rospy.Duration(1.0), self._health_timer_cb)
        self.heartbeat_timer = rospy.Timer(rospy.Duration(0.5), self._heartbeat_timer_cb)
        self.safety_timer = rospy.Timer(
            rospy.Duration(float(rospy.get_param("~safety_gate_period", 0.02))),
            self._safety_timer_cb,
        )
        rospy.on_shutdown(self.shutdown)

    def _advertise_ros(self) -> None:
        self.pub_mission_config = rospy.Publisher(
            rospy.get_param("~mission_config_ros_topic", "/uav_protocol/mission_config"),
            String,
            queue_size=5,
            latch=True,
        )
        self.pub_mission_start = rospy.Publisher(
            rospy.get_param("~mission_start_ros_topic", "/uav_protocol/mission_start"),
            String,
            queue_size=5,
            latch=True,
        )
        self.pub_car_pose = rospy.Publisher(
            rospy.get_param("~car_pose_ros_topic", "/uav_protocol/car/pose"),
            Odometry,
            queue_size=5,
        )
        self.pub_car_pose_meta = rospy.Publisher(
            rospy.get_param("~car_pose_meta_ros_topic", "/uav_protocol/car/pose_meta"),
            String,
            queue_size=5,
        )
        self.pub_car_event = rospy.Publisher(
            rospy.get_param("~car_event_ros_topic", "/uav_protocol/car/event"),
            String,
            queue_size=5,
        )
        self.pub_car_health = rospy.Publisher(
            rospy.get_param("~car_health_ros_topic", "/uav_protocol/car/health"),
            String,
            queue_size=5,
        )
        self.pub_car_heartbeat = rospy.Publisher(
            rospy.get_param("~car_heartbeat_ros_topic", "/uav_protocol/car/heartbeat"),
            String,
            queue_size=5,
        )
        self.pub_dynamic_descent_allowed = rospy.Publisher(
            rospy.get_param(
                "~dynamic_descent_allowed_topic",
                "/uav_protocol/dynamic_descent_allowed",
            ),
            Bool,
            queue_size=1,
            latch=True,
        )
        self.pub_safety_hold = rospy.Publisher(
            rospy.get_param("~safety_hold_topic", "/uav_protocol/safety_hold"),
            Bool,
            queue_size=1,
            latch=True,
        )

    def _subscribe_ros(self) -> None:
        rospy.Subscriber(
            rospy.get_param("~odom_topic", "/mavros/local_position/odom"),
            Odometry,
            self._odom_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~mavros_state_topic", "/mavros/state"),
            State,
            self._mavros_state_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~battery_topic", "/mavros/battery"),
            BatteryState,
            self._battery_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~flight_state_topic", "/ego_bridge/flight_state"),
            String,
            self._flight_state_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~control_mode_topic", "/ego_bridge/control_mode"),
            UInt8,
            self._control_mode_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param(
                "~positioning_status_topic", "/d_task/positioning/status"
            ),
            String,
            self._positioning_status_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~task_state_topic", "/uav_protocol/task_state"),
            String,
            self._task_state_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~tracking_topic", "/uav_protocol/local_tracking"),
            String,
            self._tracking_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~event_topic", "/uav_protocol/local_event"),
            String,
            self._event_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~fault_topic", "/uav_protocol/local_fault"),
            String,
            self._fault_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~health_topic", "/uav_protocol/local_health"),
            String,
            self._health_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~mission_reset_topic", "/uav_protocol/mission_reset"),
            String,
            self._mission_reset_cb,
            queue_size=5,
        )

        camera_topic = rospy.get_param(
            "~camera_image_topic", "/usb_camera_vision/usb_cam/image_raw"
        )
        if camera_topic:
            rospy.Subscriber(camera_topic, Image, self._camera_cb, queue_size=1)

        camera_health_topic = rospy.get_param("~camera_health_topic", "")
        if camera_health_topic:
            rospy.Subscriber(camera_health_topic, Bool, self._camera_health_cb, queue_size=1)
        vision_health_topic = rospy.get_param("~vision_health_topic", "")
        if vision_health_topic:
            rospy.Subscriber(vision_health_topic, Bool, self._vision_health_cb, queue_size=1)

    def _connect_mqtt(self) -> None:
        self.bus = MqttBus(self.mqtt_config)
        self.endpoint = ProtocolEndpoint(self.bus, guard=self.guard)
        self.endpoint.subscribe(
            Topics.MISSION_CONFIG,
            self._mqtt_message_cb,
            qos=1,
            include_duplicates=True,
            include_rejected=True,
        )
        self.endpoint.subscribe(
            Topics.MISSION_START,
            self._mqtt_message_cb,
            qos=1,
            include_duplicates=True,
            include_rejected=True,
        )
        self.endpoint.subscribe(Topics.CAR_POSE, self._mqtt_message_cb, qos=0)
        self.endpoint.subscribe(Topics.CAR_EVENT, self._mqtt_message_cb, qos=0)
        self.endpoint.subscribe(Topics.CAR_HEALTH, self._mqtt_message_cb, qos=0)
        self.endpoint.subscribe(Topics.CAR_HEARTBEAT, self._mqtt_message_cb, qos=0)
        try:
            self.bus.connect_async(
                min_retry_delay=int(rospy.get_param("~mqtt_reconnect_min_delay", 1)),
                max_retry_delay=int(rospy.get_param("~mqtt_reconnect_max_delay", 10)),
            )
            self.bus.start()
            rospy.loginfo(
                "[uav_protocol_gateway] MQTT async connect started for %s:%d",
                self.mqtt_config.host,
                self.mqtt_config.port,
            )
        except Exception as exc:
            rospy.logerr("[uav_protocol_gateway] MQTT adapter start failed: %s", exc)

    def _mqtt_message_cb(self, topic, message, decision) -> None:
        # Paho invokes this on its network thread.  Do not publish ROS or MQTT
        # messages here: a high-rate car pose stream can otherwise block the
        # keepalive loop through cross-thread lock ordering.
        self._mqtt_inbox.put((topic, message, decision))

    def _mqtt_io_timer_cb(self, _event) -> None:
        # ROS state transitions are handled first.  All resulting MQTT writes
        # are queued, so no Paho call can run while the gateway state lock is
        # held.  This prevents lock inversion under the 20 Hz car pose stream.
        for _ in range(50):
            try:
                topic, message, decision = self._mqtt_inbox.get_nowait()
            except queue.Empty:
                break
            self._handle_mqtt_message(topic, message, decision)

        for _ in range(100):
            try:
                topic, message, qos, retain = self._mqtt_outbox.get_nowait()
            except queue.Empty:
                break
            try:
                self.endpoint.publish(
                    topic,
                    message,
                    qos=qos,
                    retain=retain,
                )
            except Exception as exc:
                with self.lock:
                    self.fault_code = 1101
                    self.fault_text = "mqtt_publish_failed: " + str(exc)
                rospy.logwarn_throttle(
                    2.0,
                    "[uav_protocol_gateway] MQTT publish failed: %s",
                    exc,
                )

    def _handle_mqtt_message(self, topic, message, decision) -> None:
        with self.lock:
            result = self.core.receive_checked(
                topic,
                message,
                decision.status,
                _now_ms(),
                decision.reason,
            )
            for action in result.actions:
                if action.kind == "mission_config":
                    self.pub_mission_config.publish(String(data=message.to_json()))
                    self.mode = message.payload["mode"]
                    self.state = "POSITIONING_INIT"
                    self.fault_code = 0
                    self.fault_text = ""
                    self.last_task_state_ms = None
                    self.follow_established = False
                    self._publish_event("MISSION_CONFIG_ACCEPTED", mode=self.mode)
                elif action.kind == "mission_start":
                    self.pub_mission_start.publish(String(data=message.to_json()))
                    self.mode = message.payload["mode"]
                    self._publish_event("MISSION_START_ACCEPTED", mode=self.mode)
                else:
                    self._publish_action(action)

            if result.rejected:
                rospy.logwarn_throttle(2.0, "[uav_protocol_gateway] rejected %s: %s", topic, result.reason)
                return
            if message.type == "car_pose":
                self._publish_car_pose(message)
            elif message.type == "event":
                self.pub_car_event.publish(String(data=message.to_json()))
            elif message.type == "health":
                self.last_car_health = dict(message.payload)
                self.pub_car_health.publish(String(data=message.to_json()))
            elif message.type == "heartbeat":
                self.last_car_heartbeat_ms = _now_ms()
                self.pub_car_heartbeat.publish(String(data=message.to_json()))

    def _publish_action(self, action: CoreAction) -> None:
        if not action.topic:
            return
        self._mqtt_outbox.put(
            (action.topic, action.message, action.qos, action.retain)
        )

    def _publish_car_pose(self, message) -> None:
        payload = message.payload
        odom = Odometry()
        odom.header.stamp = rospy.Time.from_sec(message.sent_at_ms / 1000.0)
        odom.header.frame_id = payload["frame_id"]
        odom.child_frame_id = payload["source_frame"]
        odom.pose.pose.position.x = payload["x"]
        odom.pose.pose.position.y = payload["y"]
        odom.pose.pose.orientation = _yaw_to_quaternion(payload["yaw"])
        odom.twist.twist.linear.x = payload["vx"]
        odom.twist.twist.linear.y = payload["vy"]
        odom.twist.twist.angular.z = payload["yaw_rate"]
        self.pub_car_pose.publish(odom)
        self.pub_car_pose_meta.publish(String(data=message.to_json()))

    def _odom_cb(self, message: Odometry) -> None:
        with self.lock:
            self.odom = message
            self.last_odom_ms = _now_ms()

    def _mavros_state_cb(self, message: State) -> None:
        with self.lock:
            self.px4_state = message.mode or "UNKNOWN"
            self.armed = bool(message.armed)
            self.last_px4_state_ms = _now_ms()

    def _battery_cb(self, message: BatteryState) -> None:
        if math.isfinite(message.percentage) and message.percentage >= 0.0:
            with self.lock:
                self.battery_percent = max(0.0, min(100.0, message.percentage * 100.0))

    def _flight_state_cb(self, message: String) -> None:
        with self.lock:
            task_state_age = self._age_ms(self.last_task_state_ms)
            if (
                message.data
                and self.core.mission_id
                and (task_state_age is None or task_state_age > 500)
            ):
                self.state = self._normalise_state(message.data)

    def _control_mode_cb(self, message: UInt8) -> None:
        with self.lock:
            self.control_mode = "OVERRIDE" if message.data else "EGO"

    def _positioning_status_cb(self, message: String) -> None:
        try:
            data = _json_object(message.data)
            mission_id = str(data["mission_id"])
            state = str(data.get("state", "POSITIONING_INIT"))
            ready = bool(data.get("ready", False))
        except (KeyError, ValueError, TypeError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(
                2.0, "[uav_protocol_gateway] invalid positioning status: %s", exc
            )
            return

        with self.lock:
            if not self.core.set_positioning_ready(mission_id, ready):
                rospy.logwarn_throttle(
                    2.0,
                    "[uav_protocol_gateway] ignored positioning status for %s",
                    mission_id,
                )
                return
            normalised = self._normalise_state(state)
            self.state = "WAIT_START" if ready else normalised
            if state == "POSITIONING_FAULT":
                self.fault_code = int(data.get("fault_code", 1301))
                self.fault_text = str(data.get("fault_text", "positioning_failed"))
            if self.state != self._last_positioning_state:
                self._publish_event(
                    "POSITIONING_READY" if ready else state,
                    positioning_state=state,
                )
                self._last_positioning_state = self.state

    def _task_state_cb(self, message: String) -> None:
        try:
            data = _json_object(message.data)
            state = data.get("state")
            mode = data.get("mode")
            mission_id = str(data.get("mission_id", ""))
        except (ValueError, TypeError, json.JSONDecodeError):
            state = message.data
            mode = None
            mission_id = ""
        with self.lock:
            expected_ids = {
                value
                for value in (self.core.mission_id, self.core.configured_mission_id)
                if value
            }
            if not expected_ids or (mission_id and mission_id not in expected_ids):
                return
            if state:
                self.state = self._normalise_state(str(state))
                self.follow_established = follow_established_after_state(
                    self.follow_established,
                    str(mode or self.mode),
                    self.state,
                )
                self.last_task_state_ms = _now_ms()
            if mode in (Mode.DROP.value, Mode.DYNAMIC_LANDING.value):
                self.mode = mode

    def _tracking_cb(self, message: String) -> None:
        try:
            data = _json_object(message.data)
            self.tracking = self._normalise_tracking(data)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(2.0, "[uav_protocol_gateway] invalid local tracking: %s", exc)
            with self.lock:
                self.tracking = self._default_tracking()
        with self.lock:
            self.last_tracking_ms = _now_ms()

    def _event_cb(self, message: String) -> None:
        try:
            data = _json_object(message.data)
        except (ValueError, TypeError, json.JSONDecodeError):
            data = {"event": message.data}
        try:
            event, details = normalise_local_event(
                data, self._context_mission_id()
            )
        except ValueError as exc:
            rospy.logwarn_throttle(
                2.0, "[uav_protocol_gateway] rejected local event: %s", exc
            )
            return
        self._publish_event(event, **details)

    def _fault_cb(self, message: String) -> None:
        try:
            data = _json_object(message.data)
        except (ValueError, TypeError, json.JSONDecodeError):
            data = {"fault_code": 1099, "severity": "ERROR", "fault_text": message.data}
        try:
            fault_code = int(data.pop("fault_code", 1099))
        except (TypeError, ValueError):
            fault_code = 1099
        self._publish_fault(
            fault_code,
            str(data.pop("severity", "ERROR")),
            str(data.pop("fault_text", "local_fault")),
            **data,
        )

    def _health_cb(self, message: String) -> None:
        try:
            data = _json_object(message.data)
        except (ValueError, TypeError, json.JSONDecodeError):
            data = {}
        with self.lock:
            self.task_health.update(data)

    def _mission_reset_cb(self, message: String) -> None:
        try:
            data = _json_object(message.data)
        except (ValueError, TypeError, json.JSONDecodeError):
            data = {"mission_id": message.data, "final_state": "COMPLETE"}
        mission_id = str(data.get("mission_id", ""))
        final_state = str(data.get("final_state", ""))
        if final_state not in {"COMPLETE", "ABORT"}:
            rospy.logwarn("[uav_protocol_gateway] mission reset requires COMPLETE or ABORT")
            return
        with self.lock:
            if not self.core.mission_id or mission_id != self.core.mission_id:
                rospy.logwarn("[uav_protocol_gateway] mission reset id mismatch: %s", mission_id)
                return
            self._publish_event("MISSION_FINISHED", final_state=final_state)
            self.core.finish_mission(mission_id)
            self.pub_mission_config.publish(String(data=""))
            self.pub_mission_start.publish(String(data=""))
            self.state = "NOT_READY"
            self.mode = Mode.DROP.value
            self._stale_pose_fault_sent = False
            self.last_task_state_ms = None
            self.follow_established = False

    def _camera_cb(self, _message: Image) -> None:
        with self.lock:
            self.last_camera_ms = _now_ms()

    def _camera_health_cb(self, message: Bool) -> None:
        with self.lock:
            self.last_camera_health = bool(message.data)

    def _vision_health_cb(self, message: Bool) -> None:
        with self.lock:
            self.last_vision_health = bool(message.data)

    def _state_timer_cb(self, _event) -> None:
        with self.lock:
            mission_id = self._context_mission_id()
            payload = self._uav_state_payload()
            message = build_uav_state(self.factory, mission_id, **payload)
            self._safe_publish(Topics.UAV_STATE, message, qos=0)

    def _tracking_timer_cb(self, _event) -> None:
        with self.lock:
            mission_id = self._context_mission_id()
            payload = dict(self.tracking)
            transport_age = self._age_ms(self.last_tracking_ms)
            source_age = int(payload.get("vision_age_ms", 2**31 - 1))
            if transport_age is None:
                payload["vision_age_ms"] = 2**31 - 1
            else:
                payload["vision_age_ms"] = min(
                    2**31 - 1, max(0, source_age) + transport_age
                )
            payload["landing_gate"] = bool(payload.get("landing_gate", False)) and self.core.dynamic_descent_allowed()
            try:
                message = build_tracking(self.factory, mission_id, **payload)
            except (TypeError, ValueError) as exc:
                rospy.logwarn_throttle(2.0, "[uav_protocol_gateway] tracking payload rejected: %s", exc)
                message = build_tracking(self.factory, mission_id, **self._default_tracking())
            self._safe_publish(Topics.UAV_TRACKING, message, qos=0)

    def _health_timer_cb(self, _event) -> None:
        with self.lock:
            mission_id = self._context_mission_id()
            odom_ok = self._fresh(self.last_odom_ms, 500)
            px4_ok = self._fresh(self.last_px4_state_ms, 1000)
            camera_ok = self.last_camera_health if self.last_camera_health is not None else self._fresh(self.last_camera_ms, 1000)
            vision_ok = self.last_vision_health if self.last_vision_health is not None else self._fresh(self.last_tracking_ms, 1000)
            payload = {
                "online": True,
                "mqtt_connected": bool(self.bus.connected),
                "localization_ok": odom_ok,
                "positioning_ready": bool(
                    self.core.positioning_ready
                    and self.core.configured_mission_id == mission_id
                ),
                "camera_ok": bool(camera_ok),
                "vision_ok": bool(vision_ok),
                "fault_code": self.fault_code,
                "fault_text": self.fault_text,
                "px4_connected": px4_ok,
                "car_pose_fresh": self.core.car_pose_fresh(),
                "car_pose_age_ms": self.core.car_pose_age_ms(),
            }
            for key, value in self.task_health.items():
                if key in payload:
                    if isinstance(value, bool):
                        payload[key] = value
                else:
                    payload[key] = value
            message = build_health(self.factory, mission_id, **payload)
            self._safe_publish(Topics.UAV_HEALTH, message, qos=0)

    def _heartbeat_timer_cb(self, _event) -> None:
        with self.lock:
            mission_id = self._context_mission_id()
            message = build_heartbeat(
                self.factory,
                mission_id,
                online=True,
                uptime_ms=int((time.monotonic() - self.started_monotonic) * 1000),
            )
            self._safe_publish(Topics.UAV_HEARTBEAT, message, qos=0)

    def _safety_timer_cb(self, _event) -> None:
        with self.lock:
            descent_allowed = self.core.dynamic_descent_allowed()
            low_states = {
                "DESCEND_HIGH",
                "DESCEND_LOW",
                "LAND_ON_PLATFORM",
            }
            hold_required = (
                self.mode == Mode.DYNAMIC_LANDING.value
                and self.state in low_states
                and not descent_allowed
            )
            self.pub_dynamic_descent_allowed.publish(Bool(data=descent_allowed))
            self.pub_safety_hold.publish(Bool(data=hold_required))
            if hold_required and not self._stale_pose_fault_sent:
                self._publish_fault(1201, "ERROR", "car_pose_stale_over_300ms")
                self._stale_pose_fault_sent = True
            elif not hold_required:
                self._stale_pose_fault_sent = False

    def _uav_state_payload(self) -> Dict[str, Any]:
        odom = self.odom
        pose_age_ms = self._age_ms(self.last_odom_ms)
        pose_valid = (
            odom is not None
            and pose_age_ms is not None
            and pose_age_ms <= 500
        )
        if odom is None:
            frame_id = "unknown"
            pose_age_ms = 2**31 - 1
            x = y = z = yaw = vx = vy = vz = 0.0
        else:
            frame_id = odom.header.frame_id or "unknown"
            assert pose_age_ms is not None
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            z = odom.pose.pose.position.z
            q = odom.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            vx = odom.twist.twist.linear.x
            vy = odom.twist.twist.linear.y
            vz = odom.twist.twist.linear.z
        elapsed = 0
        if self.core.mission_started_ms is not None:
            elapsed = max(0, _now_ms() - self.core.mission_started_ms)
        return {
            "state": self._normalise_state(self.state),
            "mode": self.mode,
            "control_mode": self.control_mode,
            "px4_mode": self.px4_state,
            "armed": self.armed,
            "frame_id": frame_id,
            "pose_valid": pose_valid,
            "pose_age_ms": pose_age_ms,
            "x": x,
            "y": y,
            "z": z,
            "yaw": yaw,
            "vx": vx,
            "vy": vy,
            "vz": vz,
            "battery_percent": self.battery_percent,
            "mission_elapsed_ms": elapsed,
            "follow_established": self.follow_established,
        }

    @staticmethod
    def _normalise_state(state: str) -> str:
        value = _EGO_BRIDGE_STATE_MAP.get(state, state)
        return value if value in _UAV_STATES else "READY"

    def _default_tracking(self) -> Dict[str, Any]:
        return {
            "track_state": "INVALID",
            "detected": False,
            "confidence": 0.0,
            "pixel_center": {"u": 0.0, "v": 0.0},
            "relative_position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "relative_velocity": {"x": 0.0, "y": 0.0},
            "vision_age_ms": 2**31 - 1,
            "filter_mode": "INVALID",
            "release_gate": False,
            "landing_gate": False,
        }

    def _normalise_tracking(self, data: Dict[str, Any]) -> Dict[str, Any]:
        payload = self._default_tracking()
        payload.update(data)
        payload["track_state"] = str(payload.get("track_state", "INVALID"))
        payload["detected"] = bool(payload.get("detected", False))
        try:
            payload["confidence"] = float(payload.get("confidence", 0.0))
        except (TypeError, ValueError):
            payload["confidence"] = 0.0
        for key in ("pixel_center", "relative_position", "relative_velocity"):
            if not isinstance(payload.get(key), dict):
                payload[key] = self._default_tracking()[key]
        payload["release_gate"] = bool(payload.get("release_gate", False))
        payload["landing_gate"] = bool(payload.get("landing_gate", False))
        try:
            payload["vision_age_ms"] = max(
                0, int(payload.get("vision_age_ms", 2**31 - 1))
            )
        except (TypeError, ValueError):
            payload["vision_age_ms"] = 2**31 - 1
        if payload.get("filter_mode") not in {"MEASURED", "PREDICTED", "STALE", "INVALID"}:
            payload["filter_mode"] = "INVALID"
        return payload

    def _publish_event(self, event: str, **details: Any) -> None:
        with self.lock:
            message = build_event(self.factory, self._context_mission_id(), event, **details)
            # The payload-complete event triggers a one-shot speed increase on
            # the moving car.  Deliver it at least once; the car latches it by
            # mission ID and ignores duplicates.
            qos = 1 if event == "PAYLOAD_RELEASED_SPEED_UP" else 0
            self._safe_publish(Topics.UAV_EVENT, message, qos=qos)

    def _publish_fault(self, code: int, severity: str, text: str, **details: Any) -> None:
        self.fault_code = code
        self.fault_text = text
        message = build_fault(
            self.factory,
            self._context_mission_id(),
            fault_code=code,
            severity=severity,
            fault_text=text,
            **details,
        )
        self._safe_publish(Topics.SAFETY_FAULT, message, qos=1)

    def _safe_publish(self, topic: str, message, qos: int = 0) -> None:
        self._mqtt_outbox.put((topic, message, qos, False))

    def _age_ms(self, stamp: Optional[int]) -> Optional[int]:
        if stamp is None:
            return None
        return max(0, _now_ms() - stamp)

    def _fresh(self, stamp: Optional[int], timeout_ms: int) -> bool:
        age = self._age_ms(stamp)
        return age is not None and age <= timeout_ms

    def _context_mission_id(self) -> str:
        return self.core.mission_id or self.core.configured_mission_id or "idle"

    def shutdown(self) -> None:
        if getattr(self, "bus", None) is not None:
            try:
                self.bus.stop()
            except Exception as exc:
                rospy.logwarn("[uav_protocol_gateway] MQTT stop failed: %s", exc)


def main() -> None:
    rospy.init_node("uav_protocol_gateway")
    UavProtocolGateway()
    rospy.spin()


if __name__ == "__main__":
    main()
