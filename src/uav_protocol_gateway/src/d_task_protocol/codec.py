"""Wire codec with payload validation."""

from __future__ import annotations

from typing import Any, Mapping, Union

from .envelope import Envelope, ProtocolError
from .messages import validate_payload


class ProtocolCodec:
    @staticmethod
    def encode(message: Envelope) -> bytes:
        validate_payload(message.type, message.payload)
        return message.to_json().encode("utf-8")

    @staticmethod
    def decode(raw: Union[bytes, str], validate: bool = True) -> Envelope:
        message = Envelope.from_json(raw)
        if validate:
            validate_payload(message.type, message.payload)
        return message

    @staticmethod
    def decode_dict(value: Mapping[str, Any], validate: bool = True) -> Envelope:
        message = Envelope.from_dict(value)
        if validate:
            validate_payload(message.type, message.payload)
        return message
