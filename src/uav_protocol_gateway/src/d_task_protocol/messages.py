"""Canonical message types, payload builders, and lightweight validators."""

from __future__ import annotations

import math
from enum import Enum
from typing import Any, Dict, Mapping, Optional, Union

from .envelope import Envelope, EnvelopeFactory, ProtocolError, now_ms


class Sender(str, Enum):
    GROUND = "ground"
    CAR = "car"
    UAV = "uav"


class Mode(str, Enum):
    DROP = "DROP"
    DYNAMIC_LANDING = "DYNAMIC_LANDING"


class GroundCommandAction(str, Enum):
    SELECT = "SELECT"
    START = "START"
    START_CAR_ONLY = "START_CAR_ONLY"
    ABORT = "ABORT"
    RESET_LOCALIZATION = "RESET_LOCALIZATION"


class AckResult(str, Enum):
    ACCEPTED = "accepted"
    REJECTED = "rejected"
    DUPLICATE = "duplicate"
    BUSY = "busy"
    EXPIRED = "expired"


class RouteSegment(str, Enum):
    AB = "AB"
    BC = "BC"
    CD = "CD"
    DA = "DA"
    FINISHED = "FINISHED"


class CarState(str, Enum):
    NOT_READY = "NOT_READY"
    LOCALIZATION_INIT = "LOCALIZATION_INIT"
    READY = "READY"
    RUN_AB = "RUN_AB"
    RUN_BC = "RUN_BC"
    RUN_CD = "RUN_CD"
    RUN_DA = "RUN_DA"
    COMPLETE = "COMPLETE"
    ABORT = "ABORT"
    FAULT = "FAULT"


class UavState(str, Enum):
    NOT_READY = "NOT_READY"
    POSITIONING_INIT = "POSITIONING_INIT"
    POSITIONING_FAULT = "POSITIONING_FAULT"
    WAIT_START = "WAIT_START"
    READY = "READY"
    TAKEOFF = "TAKEOFF"
    HOVER_3S = "HOVER_3S"
    SEARCH_CAR = "SEARCH_CAR"
    LOCK_CAR = "LOCK_CAR"
    FOLLOW_CAR = "FOLLOW_CAR"
    DROP_DESCEND = "DROP_DESCEND"
    RELEASE = "RELEASE"
    RETURN_HOME = "RETURN_HOME"
    DESCEND_HIGH = "DESCEND_HIGH"
    DESCEND_LOW = "DESCEND_LOW"
    LAND_ON_PLATFORM = "LAND_ON_PLATFORM"
    PLATFORM_LANDED = "PLATFORM_LANDED"
    PLATFORM_HOLD = "PLATFORM_HOLD"
    PLATFORM_TAKEOFF = "PLATFORM_TAKEOFF"
    CLIMB_TO_CRUISE = "CLIMB_TO_CRUISE"
    LAND_HOME = "LAND_HOME"
    COMPLETE = "COMPLETE"
    ABORT = "ABORT"


class FilterMode(str, Enum):
    MEASURED = "MEASURED"
    PREDICTED = "PREDICTED"
    STALE = "STALE"
    INVALID = "INVALID"


def _require(payload: Mapping[str, Any], *fields: str) -> None:
    missing = [field for field in fields if field not in payload]
    if missing:
        raise ProtocolError("missing payload fields: " + ", ".join(missing))


def _number(value: Any, field: str) -> None:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        raise ProtocolError(f"{field} must be a finite number")


def _bool(value: Any, field: str) -> None:
    if not isinstance(value, bool):
        raise ProtocolError(f"{field} must be boolean")


def _pose(payload: Mapping[str, Any], prefix: str = "") -> None:
    for key in ("x", "y", "yaw"):
        name = f"{prefix}{key}"
        _require(payload, name)
        _number(payload[name], name)


def validate_payload(message_type: str, payload: Mapping[str, Any]) -> None:
    if not isinstance(payload, Mapping):
        raise ProtocolError("payload must be an object")
    if message_type == "ground_start_request":
        _require(payload, "mode", "requested_at_ms")
        if payload["mode"] not in {item.value for item in Mode}:
            raise ProtocolError("invalid mission mode")
        if not isinstance(payload["requested_at_ms"], int):
            raise ProtocolError("requested_at_ms must be integer")
        action = payload.get("action", GroundCommandAction.SELECT.value)
        if action not in {item.value for item in GroundCommandAction}:
            raise ProtocolError("invalid ground command action")
    elif message_type == "mission_config":
        _require(payload, "mode", "selected_by", "config_reason", "requested_at_ms")
        if payload["mode"] not in {item.value for item in Mode}:
            raise ProtocolError("invalid mission mode")
        if payload["selected_by"] not in {"car_button", "ground_web"}:
            raise ProtocolError("invalid mission config source")
        if not isinstance(payload["requested_at_ms"], int):
            raise ProtocolError("requested_at_ms must be integer")
    elif message_type == "mission_start":
        _require(payload, "mode", "car_start_pose", "start_reason")
        if payload["mode"] not in {item.value for item in Mode}:
            raise ProtocolError("invalid mission mode")
        if not isinstance(payload["car_start_pose"], Mapping):
            raise ProtocolError("car_start_pose must be an object")
        _require(payload["car_start_pose"], "frame_id", "x", "y", "yaw")
        _number(payload["car_start_pose"]["x"], "car_start_pose.x")
        _number(payload["car_start_pose"]["y"], "car_start_pose.y")
        _number(payload["car_start_pose"]["yaw"], "car_start_pose.yaw")
    elif message_type == "config_ack":
        _require(payload, "command", "result", "reason_code")
        if payload["command"] != "mission_config":
            raise ProtocolError("unsupported ACK command")
        if payload["result"] not in {item.value for item in AckResult}:
            raise ProtocolError("invalid ACK result")
    elif message_type == "start_ack":
        _require(payload, "command", "result", "reason_code")
        if payload["command"] != "mission_start":
            raise ProtocolError("unsupported ACK command")
        if payload["result"] not in {item.value for item in AckResult}:
            raise ProtocolError("invalid ACK result")
    elif message_type == "car_pose":
        _require(
            payload,
            "frame_id",
            "source_frame",
            "x",
            "y",
            "yaw",
            "vx",
            "vy",
            "yaw_rate",
            "speed",
            "route_segment",
            "route_progress_m",
            "distance_to_d_m",
            "moving",
            "odom_valid",
            "pose_quality",
        )
        for key in (
            "x",
            "y",
            "yaw",
            "vx",
            "vy",
            "yaw_rate",
            "speed",
            "route_progress_m",
            "distance_to_d_m",
            "pose_quality",
        ):
            _number(payload[key], key)
        _bool(payload["moving"], "moving")
        _bool(payload["odom_valid"], "odom_valid")
        if payload["route_segment"] not in {item.value for item in RouteSegment}:
            raise ProtocolError("invalid route_segment")
    elif message_type == "car_state":
        _require(
            payload,
            "state",
            "mode",
            "localization_state",
            "route_segment",
            "start_allowed",
            "start_block_reason",
            "mission_elapsed_ms",
            "trigger_source",
        )
        if payload["state"] not in {item.value for item in CarState}:
            raise ProtocolError("invalid car state")
        if payload["mode"] not in {item.value for item in Mode}:
            raise ProtocolError("invalid mode")
        if payload["route_segment"] not in {item.value for item in RouteSegment}:
            raise ProtocolError("invalid route_segment")
        _bool(payload["start_allowed"], "start_allowed")
        _number(payload["mission_elapsed_ms"], "mission_elapsed_ms")
    elif message_type == "uav_state":
        _require(
            payload,
            "state",
            "mode",
            "control_mode",
            "px4_mode",
            "armed",
            "frame_id",
            "pose_valid",
            "pose_age_ms",
            "x",
            "y",
            "z",
            "yaw",
            "vx",
            "vy",
            "vz",
            "battery_percent",
            "mission_elapsed_ms",
        )
        if payload["state"] not in {item.value for item in UavState}:
            raise ProtocolError("invalid uav state")
        if payload["mode"] not in {item.value for item in Mode}:
            raise ProtocolError("invalid mode")
        _bool(payload["armed"], "armed")
        if not isinstance(payload["frame_id"], str) or not payload["frame_id"]:
            raise ProtocolError("frame_id must be a non-empty string")
        _bool(payload["pose_valid"], "pose_valid")
        for key in (
            "pose_age_ms",
            "x",
            "y",
            "z",
            "yaw",
            "vx",
            "vy",
            "vz",
            "battery_percent",
            "mission_elapsed_ms",
        ):
            _number(payload[key], key)
    elif message_type == "uav_tracking":
        _require(
            payload,
            "track_state",
            "detected",
            "confidence",
            "pixel_center",
            "relative_position",
            "relative_velocity",
            "vision_age_ms",
            "filter_mode",
            "release_gate",
            "landing_gate",
        )
        _bool(payload["detected"], "detected")
        _number(payload["confidence"], "confidence")
        if payload["filter_mode"] not in {item.value for item in FilterMode}:
            raise ProtocolError("invalid filter_mode")
        _bool(payload["release_gate"], "release_gate")
        _bool(payload["landing_gate"], "landing_gate")
    elif message_type == "health":
        _require(
            payload,
            "online",
            "mqtt_connected",
            "localization_ok",
            "camera_ok",
            "vision_ok",
            "fault_code",
            "fault_text",
        )
        for key in ("online", "mqtt_connected", "localization_ok", "camera_ok", "vision_ok"):
            _bool(payload[key], key)
    elif message_type == "heartbeat":
        _require(payload, "online", "uptime_ms")
        _bool(payload["online"], "online")
        if not isinstance(payload["uptime_ms"], (int, float)):
            raise ProtocolError("uptime_ms must be numeric")
    elif message_type == "event":
        _require(payload, "event")
    elif message_type == "fault":
        _require(payload, "fault_code", "severity", "fault_text")
    else:
        raise ProtocolError(f"unsupported message type: {message_type}")


def _make(
    factory: EnvelopeFactory,
    message_type: str,
    mission_id: str,
    payload: Dict[str, Any],
    ttl_ms: int,
    command_id: Optional[str] = None,
    reply_to: Optional[str] = None,
) -> Envelope:
    validate_payload(message_type, payload)
    return factory.make(
        message_type,
        mission_id,
        payload,
        ttl_ms,
        command_id=command_id,
        reply_to=reply_to,
    )


def build_ground_start_request(
    factory: EnvelopeFactory,
    mission_id: str,
    mode: Union[Mode, str],
    command_id: str,
    requested_at_ms: Optional[int] = None,
    action: Union[GroundCommandAction, str] = GroundCommandAction.SELECT,
) -> Envelope:
    return _make(
        factory,
        "ground_start_request",
        mission_id,
        {
            "mode": mode.value if isinstance(mode, Mode) else mode,
            "requested_at_ms": now_ms() if requested_at_ms is None else requested_at_ms,
            "action": action.value if isinstance(action, GroundCommandAction) else action,
        },
        ttl_ms=3000,
        command_id=command_id,
    )


def build_mission_config(
    factory: EnvelopeFactory,
    mission_id: str,
    mode: Union[Mode, str],
    selected_by: str,
    config_reason: str,
    command_id: str,
    requested_at_ms: Optional[int] = None,
) -> Envelope:
    return _make(
        factory,
        "mission_config",
        mission_id,
        {
            "mode": mode.value if isinstance(mode, Mode) else mode,
            "selected_by": selected_by,
            "config_reason": config_reason,
            "requested_at_ms": now_ms() if requested_at_ms is None else requested_at_ms,
        },
        ttl_ms=5000,
        command_id=command_id,
    )


def build_mission_start(
    factory: EnvelopeFactory,
    mission_id: str,
    mode: Union[Mode, str],
    car_start_pose: Dict[str, Any],
    start_reason: str = "car_button",
    command_id: Optional[str] = None,
) -> Envelope:
    return _make(
        factory,
        "mission_start",
        mission_id,
        {
            "mode": mode.value if isinstance(mode, Mode) else mode,
            "car_start_pose": car_start_pose,
            "start_reason": start_reason,
        },
        ttl_ms=3000,
        command_id=command_id,
    )


def build_start_ack(
    factory: EnvelopeFactory,
    mission_id: str,
    result: Union[AckResult, str],
    reason_code: str = "",
    reply_to: Optional[str] = None,
) -> Envelope:
    return _make(
        factory,
        "start_ack",
        mission_id,
        {
            "command": "mission_start",
            "result": result.value if isinstance(result, AckResult) else result,
            "reason_code": reason_code,
        },
        ttl_ms=3000,
        reply_to=reply_to,
    )


def build_config_ack(
    factory: EnvelopeFactory,
    mission_id: str,
    result: Union[AckResult, str],
    reason_code: str = "",
    reply_to: Optional[str] = None,
) -> Envelope:
    return _make(
        factory,
        "config_ack",
        mission_id,
        {
            "command": "mission_config",
            "result": result.value if isinstance(result, AckResult) else result,
            "reason_code": reason_code,
        },
        ttl_ms=3000,
        reply_to=reply_to,
    )


def build_car_pose(factory: EnvelopeFactory, mission_id: str, **payload: Any) -> Envelope:
    return _make(factory, "car_pose", mission_id, payload, ttl_ms=500)


def build_car_state(factory: EnvelopeFactory, mission_id: str, **payload: Any) -> Envelope:
    return _make(factory, "car_state", mission_id, payload, ttl_ms=1000)


def build_uav_state(factory: EnvelopeFactory, mission_id: str, **payload: Any) -> Envelope:
    return _make(factory, "uav_state", mission_id, payload, ttl_ms=500)


def build_tracking(factory: EnvelopeFactory, mission_id: str, **payload: Any) -> Envelope:
    return _make(factory, "uav_tracking", mission_id, payload, ttl_ms=300)


def build_health(factory: EnvelopeFactory, mission_id: str, **payload: Any) -> Envelope:
    return _make(factory, "health", mission_id, payload, ttl_ms=2000)


def build_heartbeat(factory: EnvelopeFactory, mission_id: str, **payload: Any) -> Envelope:
    return _make(factory, "heartbeat", mission_id, payload, ttl_ms=2000)


def build_event(
    factory: EnvelopeFactory,
    mission_id: str,
    event: str,
    **details: Any,
) -> Envelope:
    return _make(
        factory,
        "event",
        mission_id,
        {"event": event, **details},
        ttl_ms=5000,
    )


def build_fault(
    factory: EnvelopeFactory,
    mission_id: str,
    fault_code: int,
    severity: str,
    fault_text: str,
    **details: Any,
) -> Envelope:
    return _make(
        factory,
        "fault",
        mission_id,
        {
            "fault_code": fault_code,
            "severity": severity,
            "fault_text": fault_text,
            **details,
        },
        ttl_ms=5000,
    )
