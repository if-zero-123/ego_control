#!/usr/bin/env python3
"""Protocol-only core for the ROS1 UAV MQTT gateway.

This module deliberately has no ROS or MQTT dependency.  It owns wire
validation, delivery guards, mission-start idempotency, and the car-pose
freshness gate used by the dynamic-landing task.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Union

from d_task_protocol import (
    AckResult,
    DeliveryStatus,
    Envelope,
    EnvelopeFactory,
    MessageGuard,
    ProtocolCodec,
    ProtocolError,
    build_config_ack,
    build_fault,
    build_start_ack,
)
from d_task_protocol.envelope import now_ms
from d_task_protocol.topics import Topics, topic_accepts_message


@dataclass(frozen=True)
class CoreAction:
    """An outbound MQTT action or an accepted local dispatch event."""

    kind: str
    topic: str
    message: Envelope
    qos: int = 0
    retain: bool = False


@dataclass
class ReceiveResult:
    """Result of processing one raw MQTT message."""

    guard_status: Optional[DeliveryStatus]
    message: Optional[Envelope] = None
    actions: List[CoreAction] = field(default_factory=list)
    rejected: bool = False
    reason: str = ""


class UavGatewayCore:
    """Pure protocol state used by the ROS1 gateway node."""

    _AUTHORIZED_START_REASONS = {"car_button", "ground_web"}

    _EXPECTED = {
        Topics.MISSION_CONFIG: ("mission_config", "car"),
        Topics.MISSION_START: ("mission_start", "car"),
        Topics.CAR_POSE: ("car_pose", "car"),
        Topics.CAR_EVENT: ("event", "car"),
        Topics.CAR_HEALTH: ("health", "car"),
        Topics.CAR_HEARTBEAT: ("heartbeat", "car"),
    }

    def __init__(
        self,
        factory: EnvelopeFactory,
        guard: Optional[MessageGuard] = None,
        clock_ms: Callable[[], int] = now_ms,
        car_pose_timeout_ms: int = 300,
    ) -> None:
        if factory.sender != "uav":
            raise ValueError("UavGatewayCore requires EnvelopeFactory('uav')")
        self.factory = factory
        self.guard = guard or MessageGuard()
        self.clock_ms = clock_ms
        self.car_pose_timeout_ms = car_pose_timeout_ms
        self.configured_mission_id = ""
        self.configured_mode = ""
        self.positioning_ready = False
        self.mission_id = ""
        self.mode = ""
        self.mission_started_ms: Optional[int] = None
        self.last_car_pose_rx_ms: Optional[int] = None
        self.last_car_pose_sent_ms: Optional[int] = None
        self.last_car_pose: Optional[Envelope] = None

    def receive(
        self,
        topic: str,
        raw: Union[bytes, str],
        now_ms: Optional[int] = None,
    ) -> ReceiveResult:
        """Decode and guard one MQTT delivery before dispatching it."""

        current = self.clock_ms() if now_ms is None else now_ms
        try:
            message = ProtocolCodec.decode(raw)
        except (ProtocolError, UnicodeDecodeError, TypeError) as exc:
            return ReceiveResult(
                guard_status=None,
                rejected=True,
                reason="decode_error: " + str(exc),
                actions=[self._fault("PROTOCOL_DECODE", str(exc))],
            )

        decision = self.guard.check(message, current_ms=current)
        return self.receive_checked(topic, message, decision.status, current, decision.reason)

    def receive_checked(
        self,
        topic: str,
        message: Envelope,
        guard_status: DeliveryStatus,
        now_ms: Optional[int] = None,
        guard_reason: str = "",
    ) -> ReceiveResult:
        """Dispatch an already decoded and guarded message.

        ``ProtocolEndpoint`` uses this entry point so the shared protocol
        library remains the single place that performs decode and guard
        checks.  ``receive`` is retained for deterministic unit tests and
        adapters that receive raw bytes directly.
        """

        current = self.clock_ms() if now_ms is None else now_ms
        result = ReceiveResult(
            guard_status=guard_status,
            message=message,
            reason=guard_reason,
        )

        expected = self._EXPECTED.get(topic)
        if expected is None:
            result.rejected = True
            result.reason = "topic_not_subscribed"
            return result

        expected_type, expected_sender = expected
        if not topic_accepts_message(topic, message.type):
            result.rejected = True
            result.reason = "topic_type_mismatch"
            return result
        if message.type != expected_type or message.sender != expected_sender:
            result.rejected = True
            result.reason = "unexpected_sender_or_type"
            return result

        if guard_status is DeliveryStatus.DUPLICATE:
            if topic in (Topics.MISSION_CONFIG, Topics.MISSION_START) and message.command_id:
                result.actions.append(self._ack(message, AckResult.DUPLICATE, "duplicate_command"))
            return result

        if guard_status is not DeliveryStatus.ACCEPTED:
            if topic in (Topics.MISSION_CONFIG, Topics.MISSION_START) and message.command_id:
                ack_result = (
                    AckResult.EXPIRED
                    if guard_status is DeliveryStatus.EXPIRED
                    else AckResult.REJECTED
                )
                result.actions.append(self._ack(message, ack_result, guard_status.value))
            return result

        if topic == Topics.MISSION_CONFIG:
            self._handle_config(result)
        elif topic == Topics.MISSION_START:
            self._handle_start(result, current)
        elif topic == Topics.CAR_POSE:
            if self.mission_id and message.mission_id != self.mission_id:
                result.rejected = True
                result.reason = "mission_id_mismatch"
                return result
            self.last_car_pose_rx_ms = current
            self.last_car_pose_sent_ms = message.sent_at_ms
            self.last_car_pose = message
        return result

    def car_pose_age_ms(self, now_ms: Optional[int] = None) -> Optional[int]:
        if self.last_car_pose_rx_ms is None:
            return None
        current = self.clock_ms() if now_ms is None else now_ms
        receive_age = max(0, current - self.last_car_pose_rx_ms)
        source_age = 0
        if self.last_car_pose_sent_ms is not None:
            source_age = max(0, current - self.last_car_pose_sent_ms)
        return max(receive_age, source_age)

    def car_pose_fresh(self, now_ms: Optional[int] = None) -> bool:
        age = self.car_pose_age_ms(now_ms)
        return age is not None and age <= self.car_pose_timeout_ms

    def dynamic_descent_allowed(self, now_ms: Optional[int] = None) -> bool:
        if not self.mission_id:
            return False
        if not self.car_pose_fresh(now_ms):
            return False
        if self.last_car_pose is None:
            return False
        if self.last_car_pose.mission_id != self.mission_id:
            return False
        payload = self.last_car_pose.payload
        return bool(payload.get("odom_valid", False)) and bool(payload.get("moving", False))

    def finish_mission(self, mission_id: str) -> bool:
        """Clear one completed mission so a new physical start can be accepted."""

        if not self.mission_id or mission_id != self.mission_id:
            return False
        self.mission_id = ""
        self.mode = ""
        self.configured_mission_id = ""
        self.configured_mode = ""
        self.positioning_ready = False
        self.mission_started_ms = None
        self.last_car_pose_rx_ms = None
        self.last_car_pose_sent_ms = None
        self.last_car_pose = None
        return True

    def set_positioning_ready(self, mission_id: str, ready: bool) -> bool:
        if not mission_id or mission_id != self.configured_mission_id:
            return False
        self.positioning_ready = bool(ready)
        return True

    def _handle_config(self, result: ReceiveResult) -> None:
        message = result.message
        assert message is not None
        if not message.command_id:
            result.rejected = True
            result.reason = "missing_command_id"
            result.actions.append(self._ack(message, AckResult.REJECTED, result.reason))
            return
        if self.mission_id:
            result.rejected = True
            result.reason = "mission_busy"
            result.actions.append(self._ack(message, AckResult.BUSY, result.reason))
            return
        if self.configured_mission_id == message.mission_id:
            if self.configured_mode != message.payload["mode"]:
                result.rejected = True
                result.reason = "mission_id_mode_conflict"
                result.actions.append(self._ack(message, AckResult.REJECTED, result.reason))
            else:
                result.actions.append(
                    self._ack(message, AckResult.DUPLICATE, "mission_already_configured")
                )
            return

        self.configured_mission_id = message.mission_id
        self.configured_mode = message.payload["mode"]
        self.positioning_ready = False
        result.actions.append(self._ack(message, AckResult.ACCEPTED, ""))
        result.actions.append(CoreAction(kind="mission_config", topic="", message=message))

    def _handle_start(self, result: ReceiveResult, current_ms: int) -> None:
        message = result.message
        assert message is not None
        payload = message.payload
        if not message.command_id:
            result.rejected = True
            result.reason = "missing_command_id"
            result.actions.append(self._ack(message, AckResult.REJECTED, result.reason))
            return
        if payload.get("start_reason") not in self._AUTHORIZED_START_REASONS:
            result.rejected = True
            result.reason = "start_reason_not_authorized"
            result.actions.append(self._ack(message, AckResult.REJECTED, result.reason))
            return
        if message.mission_id != self.configured_mission_id:
            result.rejected = True
            result.reason = "mission_not_configured"
            result.actions.append(self._ack(message, AckResult.REJECTED, result.reason))
            return
        if payload["mode"] != self.configured_mode:
            result.rejected = True
            result.reason = "configured_mode_mismatch"
            result.actions.append(self._ack(message, AckResult.REJECTED, result.reason))
            return
        if not self.positioning_ready:
            result.rejected = True
            result.reason = "uav_not_ready"
            result.actions.append(self._ack(message, AckResult.BUSY, result.reason))
            return
        if self.mission_id and self.mission_id != message.mission_id:
            result.rejected = True
            result.reason = "mission_busy"
            result.actions.append(self._ack(message, AckResult.BUSY, result.reason))
            return
        if self.mission_id == message.mission_id:
            result.actions.append(self._ack(message, AckResult.DUPLICATE, "mission_already_started"))
            return

        self.mission_id = message.mission_id
        self.mode = payload["mode"]
        self.mission_started_ms = current_ms
        self.last_car_pose_rx_ms = None
        self.last_car_pose_sent_ms = None
        self.last_car_pose = None
        result.actions.append(self._ack(message, AckResult.ACCEPTED, ""))
        result.actions.append(
            CoreAction(
                kind="mission_start",
                topic="",
                message=message,
            )
        )

    def _ack(self, message: Envelope, result: AckResult, reason: str) -> CoreAction:
        builder = build_config_ack if message.type == "mission_config" else build_start_ack
        ack = builder(
            self.factory, message.mission_id, result,
            reason_code=reason, reply_to=message.command_id,
        )
        return CoreAction(kind="ack", topic=Topics.UAV_ACK, message=ack, qos=1)

    def _fault(self, code: str, text: str) -> CoreAction:
        fault = build_fault(
            self.factory,
            self.mission_id or "idle",
            fault_code=1001,
            severity="ERROR",
            fault_text=f"{code}: {text}",
        )
        return CoreAction(kind="fault", topic=Topics.SAFETY_FAULT, message=fault, qos=1)
