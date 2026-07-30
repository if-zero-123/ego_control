"""MQTT endpoint that applies protocol validation and delivery guards."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional

from .dedupe import DeliveryStatus, GuardDecision, MessageGuard
from .envelope import Envelope
from .mqtt_bus import MqttBus, _topic_matches
from .topics import topic_accepts_message


Handler = Callable[[str, Envelope, GuardDecision], None]


@dataclass
class _Subscription:
    topic_filter: str
    handler: Handler
    include_duplicates: bool
    include_rejected: bool


class ProtocolEndpoint:
    """Connect a role to MQTT without exposing duplicate/stale messages."""

    def __init__(self, bus: MqttBus, guard: Optional[MessageGuard] = None) -> None:
        self.bus = bus
        self.guard = guard or MessageGuard()
        self._subscriptions: List[_Subscription] = []
        self._all_topics_subscribed = False
        self._all_topics_qos = -1

    def subscribe(
        self,
        topic_filter: str,
        handler: Handler,
        qos: int = 0,
        include_duplicates: bool = False,
        include_rejected: bool = False,
    ) -> None:
        if qos not in (0, 1, 2):
            raise ValueError("qos must be 0, 1, or 2")
        subscription = _Subscription(
            topic_filter,
            handler,
            include_duplicates,
            include_rejected,
        )
        self._subscriptions.append(subscription)
        if not self._all_topics_subscribed or qos > self._all_topics_qos:
            # One callback prevents duplicate dispatch when several filters
            # match the same MQTT message. Re-subscribe when a command filter
            # needs a higher QoS so the internal wildcard cannot downgrade it.
            self.bus.subscribe("#", self._dispatch, qos=qos)
            self._all_topics_subscribed = True
            self._all_topics_qos = qos

    def publish(self, topic: str, message: Envelope, qos: int = 0, retain: bool = False) -> None:
        self.bus.publish(topic, message, qos=qos, retain=retain)

    def _dispatch(self, topic: str, message: Envelope) -> None:
        if not topic_accepts_message(topic, message.type):
            return
        decision = self.guard.check(message)
        for subscription in self._subscriptions:
            if _topic_matches(subscription.topic_filter, topic):
                if decision.accepted or (
                    decision.status is DeliveryStatus.DUPLICATE
                    and subscription.include_duplicates
                ) or (
                    not decision.accepted
                    and decision.status is not DeliveryStatus.DUPLICATE
                    and subscription.include_rejected
                ):
                    subscription.handler(topic, message, decision)
