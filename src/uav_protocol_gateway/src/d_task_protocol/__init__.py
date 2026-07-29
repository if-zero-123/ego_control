"""Reusable protocol implementation for the D-task car/UAV system."""

from .envelope import Envelope, EnvelopeFactory, ProtocolError
from .messages import (
    AckResult,
    CarState,
    FilterMode,
    Mode,
    RouteSegment,
    Sender,
    UavState,
    build_car_pose,
    build_config_ack,
    build_event,
    build_fault,
    build_ground_start_request,
    build_health,
    build_heartbeat,
    build_mission_start,
    build_mission_config,
    build_start_ack,
    build_tracking,
    build_uav_state,
    validate_payload,
)
from .codec import ProtocolCodec
from .dedupe import DeliveryStatus, GuardDecision, MessageGuard
from .topics import Topics, topic_accepts_message
from .endpoint import ProtocolEndpoint

__all__ = [
    "AckResult",
    "CarState",
    "DeliveryStatus",
    "Envelope",
    "EnvelopeFactory",
    "FilterMode",
    "GuardDecision",
    "MessageGuard",
    "Mode",
    "ProtocolCodec",
    "ProtocolError",
    "ProtocolEndpoint",
    "RouteSegment",
    "Sender",
    "Topics",
    "UavState",
    "build_car_pose",
    "build_config_ack",
    "build_event",
    "build_fault",
    "build_ground_start_request",
    "build_health",
    "build_heartbeat",
    "build_mission_start",
    "build_mission_config",
    "build_start_ack",
    "build_tracking",
    "build_uav_state",
    "validate_payload",
    "topic_accepts_message",
]
