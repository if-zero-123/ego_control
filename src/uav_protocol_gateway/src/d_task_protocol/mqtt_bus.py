"""Optional paho-mqtt adapter.

Install paho-mqtt only on devices that need the network adapter. The protocol
codec and tests do not import this module.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from .codec import ProtocolCodec
from .envelope import Envelope, ProtocolError
from .topics import topic_accepts_message


def _topic_matches(subscription: str, topic: str) -> bool:
    sub_parts = subscription.split("/")
    topic_parts = topic.split("/")
    for index, sub in enumerate(sub_parts):
        if sub == "#":
            return index <= len(topic_parts)
        if index >= len(topic_parts):
            return False
        if sub != "+" and sub != topic_parts[index]:
            return False
    return len(sub_parts) == len(topic_parts)


@dataclass
class MqttConfig:
    host: str = "192.168.0.198"
    port: int = 1883
    keepalive: int = 10
    client_id: str = "d-task-client"
    username: Optional[str] = None
    password: Optional[str] = None


class MqttBus:
    """Small MQTT wrapper with JSON envelope decoding and topic callbacks."""

    def __init__(self, config: MqttConfig) -> None:
        try:
            import paho.mqtt.client as mqtt
        except ImportError as exc:
            raise RuntimeError(
                "MqttBus requires paho-mqtt; install with 'pip install paho-mqtt'"
            ) from exc

        self._mqtt = mqtt
        self.config = config
        self._client = mqtt.Client(client_id=config.client_id)
        if config.username is not None:
            self._client.username_pw_set(config.username, config.password)
        self._callbacks: List[Tuple[str, Callable[[str, Envelope], None]]] = []
        self._subscriptions: Dict[str, int] = {}
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.on_disconnect = self._on_disconnect
        self.last_error: Optional[str] = None
        self.connected = False

    def connect(self) -> None:
        self._client.connect(self.config.host, self.config.port, self.config.keepalive)

    def connect_async(
        self,
        min_retry_delay: int = 1,
        max_retry_delay: int = 10,
    ) -> None:
        """Connect without blocking and keep retrying while the loop runs."""

        if min_retry_delay < 1 or max_retry_delay < min_retry_delay:
            raise ValueError("invalid MQTT reconnect delay range")
        self._client.reconnect_delay_set(
            min_delay=min_retry_delay,
            max_delay=max_retry_delay,
        )
        self._client.connect_async(
            self.config.host,
            self.config.port,
            self.config.keepalive,
        )

    def start(self) -> None:
        self._client.loop_start()

    def stop(self) -> None:
        self._client.loop_stop()
        self._client.disconnect()

    def subscribe(
        self,
        topic_filter: str,
        callback: Callable[[str, Envelope], None],
        qos: int = 0,
    ) -> None:
        if qos not in (0, 1, 2):
            raise ValueError("qos must be 0, 1, or 2")
        callback_entry = (topic_filter, callback)
        if callback_entry not in self._callbacks:
            self._callbacks.append(callback_entry)
        self._subscriptions[topic_filter] = qos
        if self.connected:
            self._client.subscribe(topic_filter, qos=qos)

    def publish(
        self,
        topic: str,
        message: Envelope,
        qos: int = 0,
        retain: bool = False,
    ) -> None:
        if not topic_accepts_message(topic, message.type):
            raise ProtocolError(
                f"message type {message.type!r} is not accepted by topic {topic!r}"
            )
        if retain and topic.endswith("/start"):
            raise ProtocolError("START messages must never be retained")
        result = self._client.publish(
            topic, ProtocolCodec.encode(message), qos=qos, retain=retain
        )
        if result.rc != self._mqtt.MQTT_ERR_SUCCESS:
            raise ConnectionError(f"MQTT publish failed with rc={result.rc}")

    def _on_connect(self, client, userdata, flags, rc, properties=None) -> None:
        self.connected = rc == 0
        if not self.connected:
            self.last_error = f"connect_rc={rc}"
            return
        for topic_filter, qos in self._subscriptions.items():
            client.subscribe(topic_filter, qos=qos)

    def _on_disconnect(self, client, userdata, rc, properties=None) -> None:
        self.connected = False
        if rc != 0:
            self.last_error = f"disconnect_rc={rc}"

    def _on_message(self, client, userdata, msg) -> None:
        try:
            message = ProtocolCodec.decode(msg.payload)
        except (ProtocolError, UnicodeDecodeError) as exc:
            self.last_error = f"decode_error={exc}"
            return
        if not topic_accepts_message(msg.topic, message.type):
            self.last_error = (
                f"topic_type_mismatch={msg.topic}:{message.type}"
            )
            return
        for topic_filter, callback in self._callbacks:
            if _topic_matches(topic_filter, msg.topic):
                callback(msg.topic, message)
