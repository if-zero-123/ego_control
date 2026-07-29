"""TTL and duplicate/out-of-order protection for MQTT delivery."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional, Tuple

from .envelope import Envelope, now_ms


class DeliveryStatus(str, Enum):
    ACCEPTED = "accepted"
    DUPLICATE = "duplicate"
    OUT_OF_ORDER = "out_of_order"
    EXPIRED = "expired"


@dataclass(frozen=True)
class GuardDecision:
    status: DeliveryStatus
    reason: str = ""

    @property
    def accepted(self) -> bool:
        return self.status is DeliveryStatus.ACCEPTED


class MessageGuard:
    """Accept each sender boot/stream sequence once and reject stale TTLs.

    Command messages are additionally deduplicated by command_id. This makes
    QoS1 retries safe while still allowing a new command after a reboot.
    """

    def __init__(self, command_cache_ms: int = 120_000) -> None:
        self._last_seq: Dict[Tuple[str, str, str], int] = {}
        self._commands: Dict[Tuple[str, str, str, str], int] = {}
        self._command_cache_ms = command_cache_ms

    def check(self, message: Envelope, current_ms: Optional[int] = None) -> GuardDecision:
        current = now_ms() if current_ms is None else current_ms
        self._purge(current)
        if message.is_expired(current):
            return GuardDecision(DeliveryStatus.EXPIRED, "ttl_expired")

        if message.command_id is not None:
            command_key = (
                message.sender,
                message.boot_id,
                message.mission_id,
                message.command_id,
            )
            if command_key in self._commands:
                return GuardDecision(DeliveryStatus.DUPLICATE, "command_id_seen")

        stream_key = (message.sender, message.boot_id, message.type)
        previous = self._last_seq.get(stream_key)
        if previous is not None and message.seq <= previous:
            return GuardDecision(DeliveryStatus.OUT_OF_ORDER, "seq_not_newer")

        self._last_seq[stream_key] = message.seq
        if message.command_id is not None:
            self._commands[
                (
                    message.sender,
                    message.boot_id,
                    message.mission_id,
                    message.command_id,
                )
            ] = current
        return GuardDecision(DeliveryStatus.ACCEPTED)

    def _purge(self, current_ms: int) -> None:
        cutoff = current_ms - self._command_cache_ms
        self._commands = {
            key: seen_at for key, seen_at in self._commands.items() if seen_at >= cutoff
        }
