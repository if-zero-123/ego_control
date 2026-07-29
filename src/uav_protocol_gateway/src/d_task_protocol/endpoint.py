"""MQTT endpoint that applies protocol validation and delivery guards."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Set

from .dedupe import DeliveryStatus, GuardDecision, MessageGuard
from .envelope import Envelope
from .mqtt_bus import MqttBus, _topic_matches


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
        self._bus_topic_filters: Set[str] = set()

    def subscribe(
        self,
        topic_filter: str,
        handler: Handler,
        qos: int = 0,
        include_duplicates: bool = False,
        include_rejected: bool = False,
    ) -> None:
        subscription = _Subscription(
            topic_filter,
            handler,
            include_duplicates,
            include_rejected,
        )
        self._subscriptions.append(subscription)
        if topic_filter not in self._bus_topic_filters:
            self.bus.subscribe(topic_filter, self._dispatch, qos=qos)
            self._bus_topic_filters.add(topic_filter)

    def publish(self, topic: str, message: Envelope, qos: int = 0, retain: bool = False) -> None:
        self.bus.publish(topic, message, qos=qos, retain=retain)

    def _dispatch(self, topic: str, message: Envelope) -> None:
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
