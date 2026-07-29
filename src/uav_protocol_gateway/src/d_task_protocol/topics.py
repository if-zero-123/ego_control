"""Canonical MQTT topic names."""


class Topics:
    GROUND_START = "d_task/v1/ground/command/start"
    MISSION_START = "d_task/v1/mission/start"
    UAV_ACK = "d_task/v1/uav/ack"
    CAR_POSE = "d_task/v1/car/pose"
    UAV_STATE = "d_task/v1/uav/state"
    UAV_TRACKING = "d_task/v1/uav/tracking"
    CAR_HEALTH = "d_task/v1/car/health"
    UAV_HEALTH = "d_task/v1/uav/health"
    CAR_EVENT = "d_task/v1/car/event"
    UAV_EVENT = "d_task/v1/uav/event"
    MISSION_EVENT = "d_task/v1/mission/event"
    SAFETY_FAULT = "d_task/v1/safety/fault"
    CAR_HEARTBEAT = "d_task/v1/car/heartbeat"
    UAV_HEARTBEAT = "d_task/v1/uav/heartbeat"


# A topic may accept more than one sender, but the message type remains fixed.
TOPIC_MESSAGE_TYPES = {
    Topics.GROUND_START: {"ground_start_request"},
    Topics.MISSION_START: {"mission_start"},
    Topics.UAV_ACK: {"start_ack"},
    Topics.CAR_POSE: {"car_pose"},
    Topics.UAV_STATE: {"uav_state"},
    Topics.UAV_TRACKING: {"uav_tracking"},
    Topics.CAR_HEALTH: {"health"},
    Topics.UAV_HEALTH: {"health"},
    Topics.CAR_EVENT: {"event"},
    Topics.UAV_EVENT: {"event"},
    Topics.MISSION_EVENT: {"event"},
    Topics.SAFETY_FAULT: {"fault"},
    Topics.CAR_HEARTBEAT: {"heartbeat"},
    Topics.UAV_HEARTBEAT: {"heartbeat"},
}


def topic_accepts_message(topic: str, message_type: str) -> bool:
    """Return whether a concrete topic accepts an envelope type."""
    return message_type in TOPIC_MESSAGE_TYPES.get(topic, set())
