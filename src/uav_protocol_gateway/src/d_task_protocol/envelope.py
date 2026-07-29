"""Protocol envelope and sequence-producing factory.

The core module intentionally has no MQTT or ROS dependency.
"""

from __future__ import annotations

import json
import re
import time
import uuid
from dataclasses import dataclass
from typing import Any, Callable, Dict, Mapping, Optional, Union


SCHEMA = "d_task/v1"
_NAME_RE = re.compile(r"^[a-z][a-z0-9_]*$")
_SENDERS = {"ground", "car", "uav"}


class ProtocolError(ValueError):
    """Raised when a message violates the wire protocol."""


def now_ms() -> int:
    return time.time_ns() // 1_000_000


def _require_string(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value:
        raise ProtocolError(f"{field} must be a non-empty string")
    return value


def _require_int(value: Any, field: str, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ProtocolError(f"{field} must be an integer >= {minimum}")
    return value


@dataclass(frozen=True)
class Envelope:
    schema: str
    type: str
    sender: str
    boot_id: str
    mission_id: str
    seq: int
    sent_at_ms: int
    ttl_ms: int
    payload: Dict[str, Any]
    command_id: Optional[str] = None
    reply_to: Optional[str] = None

    def __post_init__(self) -> None:
        if self.schema != SCHEMA:
            raise ProtocolError(f"unsupported schema: {self.schema}")
        if not _NAME_RE.match(self.type):
            raise ProtocolError(f"invalid type: {self.type}")
        if self.sender not in _SENDERS:
            raise ProtocolError(f"invalid sender: {self.sender}")
        _require_string(self.boot_id, "boot_id")
        _require_string(self.mission_id, "mission_id")
        _require_int(self.seq, "seq", 1)
        _require_int(self.sent_at_ms, "sent_at_ms", 0)
        _require_int(self.ttl_ms, "ttl_ms", 1)
        if not isinstance(self.payload, dict):
            raise ProtocolError("payload must be an object")
        if self.command_id is not None:
            _require_string(self.command_id, "command_id")
        if self.reply_to is not None:
            _require_string(self.reply_to, "reply_to")

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "Envelope":
        if not isinstance(value, Mapping):
            raise ProtocolError("message must be a JSON object")
        required = (
            "schema",
            "type",
            "sender",
            "boot_id",
            "mission_id",
            "seq",
            "sent_at_ms",
            "ttl_ms",
            "payload",
        )
        missing = [key for key in required if key not in value]
        if missing:
            raise ProtocolError("missing fields: " + ", ".join(missing))
        payload = value["payload"]
        if not isinstance(payload, Mapping):
            raise ProtocolError("payload must be an object")
        return cls(
            schema=value["schema"],
            type=value["type"],
            sender=value["sender"],
            boot_id=value["boot_id"],
            mission_id=value["mission_id"],
            seq=value["seq"],
            sent_at_ms=value["sent_at_ms"],
            ttl_ms=value["ttl_ms"],
            payload=dict(payload),
            command_id=value.get("command_id"),
            reply_to=value.get("reply_to"),
        )

    @classmethod
    def from_json(cls, raw: Union[str, bytes]) -> "Envelope":
        try:
            value = json.loads(raw)
        except (TypeError, json.JSONDecodeError) as exc:
            raise ProtocolError(f"invalid JSON: {exc}") from exc
        return cls.from_dict(value)

    def to_dict(self) -> Dict[str, Any]:
        value: Dict[str, Any] = {
            "schema": self.schema,
            "type": self.type,
            "sender": self.sender,
            "boot_id": self.boot_id,
            "mission_id": self.mission_id,
            "seq": self.seq,
            "sent_at_ms": self.sent_at_ms,
            "ttl_ms": self.ttl_ms,
            "payload": self.payload,
        }
        if self.command_id is not None:
            value["command_id"] = self.command_id
        if self.reply_to is not None:
            value["reply_to"] = self.reply_to
        return value

    def to_json(self) -> str:
        return json.dumps(
            self.to_dict(), ensure_ascii=False, separators=(",", ":"), sort_keys=True
        )

    def age_ms(self, current_ms: Optional[int] = None) -> int:
        current = now_ms() if current_ms is None else current_ms
        return max(0, current - self.sent_at_ms)

    def is_expired(self, current_ms: Optional[int] = None) -> bool:
        return self.age_ms(current_ms) > self.ttl_ms


class EnvelopeFactory:
    """Creates envelopes with a process-stable boot_id and monotonic seq."""

    def __init__(
        self,
        sender: str,
        boot_id: Optional[str] = None,
        clock_ms: Callable[[], int] = now_ms,
    ) -> None:
        if sender not in _SENDERS:
            raise ProtocolError(f"invalid sender: {sender}")
        self.sender = sender
        self.boot_id = boot_id or f"{sender}-{clock_ms()}-{uuid.uuid4().hex[:8]}"
        self._clock_ms = clock_ms
        self._seq = 0

    @property
    def seq(self) -> int:
        return self._seq

    def make(
        self,
        message_type: str,
        mission_id: str,
        payload: Dict[str, Any],
        ttl_ms: int,
        command_id: Optional[str] = None,
        reply_to: Optional[str] = None,
    ) -> Envelope:
        self._seq += 1
        return Envelope(
            schema=SCHEMA,
            type=message_type,
            sender=self.sender,
            boot_id=self.boot_id,
            mission_id=mission_id,
            seq=self._seq,
            sent_at_ms=self._clock_ms(),
            ttl_ms=ttl_ms,
            payload=payload,
            command_id=command_id,
            reply_to=reply_to,
        )
