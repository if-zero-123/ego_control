#!/usr/bin/env python3
import os
import sys
import types
import unittest
from unittest import mock


PACKAGE_SRC = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "src")
)
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from d_task_protocol import (  # noqa: E402
    DeliveryStatus,
    EnvelopeFactory,
    MessageGuard,
    Mode,
    ProtocolCodec,
    Sender,
    build_car_pose,
    build_mission_config,
    build_mission_start,
    build_uav_state,
)
from d_task_protocol.endpoint import ProtocolEndpoint  # noqa: E402
from d_task_protocol.mqtt_bus import MqttBus, MqttConfig, _topic_matches  # noqa: E402
from d_task_protocol.topics import Topics  # noqa: E402
from uav_protocol_gateway.gateway_core import (  # noqa: E402
    UavGatewayCore,
    follow_established_after_state,
)


class GatewayCoreTests(unittest.TestCase):
    def setUp(self):
        self.clock = [1_720_000_000_000]

        def clock_ms():
            return self.clock[0]

        self.clock_ms = clock_ms
        self.car_factory = EnvelopeFactory(
            Sender.CAR.value, boot_id="car-boot", clock_ms=clock_ms
        )
        self.ground_factory = EnvelopeFactory(
            Sender.GROUND.value, boot_id="ground-boot", clock_ms=clock_ms
        )
        self.uav_factory = EnvelopeFactory(
            Sender.UAV.value, boot_id="uav-boot", clock_ms=clock_ms
        )
        self.core = UavGatewayCore(
            factory=self.uav_factory,
            guard=MessageGuard(),
            clock_ms=clock_ms,
        )

    def mission_start(self, command_id="cmd-1", start_reason="car_button"):
        return build_mission_start(
            self.car_factory,
            "mission-0001",
            Mode.DROP,
            {"frame_id": "competition_world", "x": 0.0, "y": 0.0, "yaw": 0.0},
            start_reason=start_reason,
            command_id=command_id,
        )

    def mission_config(
        self,
        mission_id="mission-0001",
        mode=Mode.DROP,
        command_id="config-1",
    ):
        return build_mission_config(
            self.car_factory,
            mission_id,
            mode,
            selected_by="car_button",
            config_reason="button_mode_select",
            command_id=command_id,
        )

    def prepare(self, mission_id="mission-0001", mode=Mode.DROP):
        config = self.mission_config(mission_id=mission_id, mode=mode)
        self.core.receive(
            Topics.MISSION_CONFIG,
            ProtocolCodec.encode(config),
            now_ms=self.clock[0],
        )
        self.assertTrue(self.core.set_positioning_ready(mission_id, True))

    def test_accepts_mission_config_and_requests_one_positioning_reset(self):
        message = self.mission_config()

        result = self.core.receive(
            Topics.MISSION_CONFIG,
            ProtocolCodec.encode(message),
            now_ms=self.clock[0],
        )

        self.assertEqual([action.kind for action in result.actions], ["ack", "mission_config"])
        self.assertEqual(result.actions[0].message.payload["command"], "mission_config")
        self.assertEqual(result.actions[0].message.payload["result"], "accepted")
        self.assertEqual(self.core.configured_mission_id, "mission-0001")
        self.assertFalse(self.core.positioning_ready)

    def test_rejects_ground_as_the_mission_config_authority(self):
        message = build_mission_config(
            self.ground_factory,
            "mission-ground",
            Mode.DROP,
            selected_by="ground_web",
            config_reason="web_select",
            command_id="config-ground",
        )

        result = self.core.receive(
            Topics.MISSION_CONFIG,
            ProtocolCodec.encode(message),
            now_ms=self.clock[0],
        )

        self.assertTrue(result.rejected)
        self.assertEqual(result.reason, "unexpected_sender_or_type")
        self.assertEqual(self.core.configured_mission_id, "")

    def test_drop_descend_is_a_valid_uav_state(self):
        message = build_uav_state(
            self.uav_factory,
            "mission-0001",
            state="DROP_DESCEND",
            mode="DROP",
            control_mode="OVERRIDE",
            px4_mode="OFFBOARD",
            armed=True,
            frame_id="map",
            pose_valid=True,
            pose_age_ms=20,
            x=0.0,
            y=0.0,
            z=1.0,
            yaw=0.0,
            vx=0.0,
            vy=0.0,
            vz=-0.2,
            battery_percent=80.0,
            mission_elapsed_ms=1000,
            follow_established=True,
        )

        self.assertEqual(message.payload["state"], "DROP_DESCEND")
        self.assertTrue(message.payload["follow_established"])

    def test_follow_established_latches_only_after_stable_follow_states(self):
        self.assertFalse(
            follow_established_after_state(False, "DROP", "FOLLOW_CAR"))
        self.assertTrue(
            follow_established_after_state(False, "DROP", "DROP_DESCEND"))
        self.assertTrue(
            follow_established_after_state(True, "DROP", "RETURN_HOME"))
        self.assertTrue(
            follow_established_after_state(
                False, "DYNAMIC_LANDING", "DESCEND_HIGH"))

    def test_duplicate_mission_config_only_emits_duplicate_ack(self):
        first = self.mission_config(command_id="config-duplicate")
        second = self.mission_config(command_id="config-duplicate")
        self.core.receive(
            Topics.MISSION_CONFIG,
            ProtocolCodec.encode(first),
            now_ms=self.clock[0],
        )

        result = self.core.receive(
            Topics.MISSION_CONFIG,
            ProtocolCodec.encode(second),
            now_ms=self.clock[0],
        )

        self.assertEqual([action.kind for action in result.actions], ["ack"])
        self.assertEqual(result.actions[0].message.payload["result"], "duplicate")

    def test_start_before_positioning_ready_returns_busy(self):
        config = self.mission_config()
        self.core.receive(
            Topics.MISSION_CONFIG,
            ProtocolCodec.encode(config),
            now_ms=self.clock[0],
        )

        result = self.core.receive(
            Topics.MISSION_START,
            ProtocolCodec.encode(self.mission_start()),
            now_ms=self.clock[0],
        )

        self.assertEqual([action.kind for action in result.actions], ["ack"])
        self.assertEqual(result.actions[0].message.payload["result"], "busy")
        self.assertEqual(result.actions[0].message.payload["reason_code"], "uav_not_ready")
        self.assertEqual(self.core.mission_id, "")

    def test_start_mode_must_match_prepared_mode(self):
        self.prepare(mode=Mode.DROP)
        message = build_mission_start(
            self.car_factory,
            "mission-0001",
            Mode.DYNAMIC_LANDING,
            {"frame_id": "competition_world", "x": 0.0, "y": 0.0, "yaw": 0.0},
            start_reason="car_button",
            command_id="cmd-mode-mismatch",
        )

        result = self.core.receive(
            Topics.MISSION_START,
            ProtocolCodec.encode(message),
            now_ms=self.clock[0],
        )

        self.assertEqual(result.actions[0].message.payload["result"], "rejected")
        self.assertEqual(result.actions[0].message.payload["reason_code"], "configured_mode_mismatch")
        self.assertEqual(self.core.mission_id, "")

    def test_accepts_car_button_start_and_emits_one_mission_event(self):
        self.prepare()
        result = self.core.receive(
            Topics.MISSION_START,
            ProtocolCodec.encode(self.mission_start()),
            now_ms=self.clock[0],
        )

        self.assertEqual(result.guard_status, DeliveryStatus.ACCEPTED)
        self.assertEqual([action.kind for action in result.actions], ["ack", "mission_start"])
        self.assertEqual(result.actions[0].message.payload["result"], "accepted")
        self.assertEqual(result.actions[1].message.mission_id, "mission-0001")
        self.assertEqual(self.core.mission_id, "mission-0001")

    def test_accepts_ground_web_start_relayed_by_car(self):
        self.prepare()
        result = self.core.receive(
            Topics.MISSION_START,
            ProtocolCodec.encode(
                self.mission_start("cmd-ground-web", "ground_web")
            ),
            now_ms=self.clock[0],
        )

        self.assertEqual([action.kind for action in result.actions], ["ack", "mission_start"])
        self.assertEqual(result.actions[0].message.payload["result"], "accepted")
        self.assertEqual(self.core.mission_id, "mission-0001")

    def test_duplicate_command_only_emits_duplicate_ack(self):
        self.prepare()
        first = self.mission_start("cmd-duplicate")
        second = self.mission_start("cmd-duplicate")

        self.core.receive(
            Topics.MISSION_START,
            ProtocolCodec.encode(first),
            now_ms=self.clock[0],
        )
        result = self.core.receive(
            Topics.MISSION_START,
            ProtocolCodec.encode(second),
            now_ms=self.clock[0],
        )

        self.assertEqual(result.guard_status, DeliveryStatus.DUPLICATE)
        self.assertEqual([action.kind for action in result.actions], ["ack"])
        self.assertEqual(result.actions[0].message.payload["result"], "duplicate")

    def test_rejects_unknown_start_reason_without_starting(self):
        self.prepare(mission_id="mission-0002", mode=Mode.DYNAMIC_LANDING)
        message = build_mission_start(
            self.car_factory,
            "mission-0002",
            Mode.DYNAMIC_LANDING,
            {"frame_id": "competition_world", "x": 0.0, "y": 0.0, "yaw": 0.0},
            start_reason="ground_command",
            command_id="cmd-ground",
        )

        result = self.core.receive(
            Topics.MISSION_START,
            ProtocolCodec.encode(message),
            now_ms=self.clock[0],
        )

        self.assertEqual([action.kind for action in result.actions], ["ack"])
        self.assertEqual(result.actions[0].message.payload["result"], "rejected")
        self.assertEqual(self.core.mission_id, "")

    def test_car_pose_older_than_300ms_disables_dynamic_descent(self):
        self.prepare()
        self.core.receive(
            Topics.MISSION_START,
            ProtocolCodec.encode(self.mission_start()),
            now_ms=self.clock[0],
        )
        pose = build_car_pose(
            self.car_factory,
            "mission-0001",
            frame_id="competition_world",
            source_frame="car_odom",
            x=1.0,
            y=2.0,
            yaw=1.57,
            vx=0.2,
            vy=0.0,
            yaw_rate=0.0,
            speed=0.2,
            route_segment="AB",
            route_progress_m=0.5,
            distance_to_d_m=4.0,
            moving=True,
            odom_valid=True,
            pose_quality=0.9,
        )
        self.core.receive(
            Topics.CAR_POSE,
            ProtocolCodec.encode(pose),
            now_ms=self.clock[0],
        )

        self.assertTrue(self.core.car_pose_fresh(self.clock[0]))
        self.assertFalse(self.core.car_pose_fresh(self.clock[0] + 301))
        self.assertFalse(self.core.dynamic_descent_allowed(self.clock[0] + 301))

    def test_wrong_topic_type_is_rejected_before_dispatch(self):
        message = self.mission_start()
        result = self.core.receive(
            Topics.CAR_POSE,
            ProtocolCodec.encode(message),
            now_ms=self.clock[0],
        )

        self.assertEqual(result.guard_status, DeliveryStatus.ACCEPTED)
        self.assertTrue(result.rejected)
        self.assertEqual(result.actions, [])

    def test_other_mission_pose_does_not_open_current_descent_gate(self):
        self.prepare()
        self.core.receive(
            Topics.MISSION_START,
            ProtocolCodec.encode(self.mission_start()),
            now_ms=self.clock[0],
        )
        pose = self.car_pose("mission-other")

        result = self.core.receive(
            Topics.CAR_POSE,
            ProtocolCodec.encode(pose),
            now_ms=self.clock[0],
        )

        self.assertTrue(result.rejected)
        self.assertEqual(result.reason, "mission_id_mismatch")
        self.assertFalse(self.core.dynamic_descent_allowed(self.clock[0]))

    def test_delayed_pose_uses_source_timestamp_for_300ms_gate(self):
        pose = self.car_pose("mission-0001")
        self.clock[0] += 350

        self.core.receive(
            Topics.CAR_POSE,
            ProtocolCodec.encode(pose),
            now_ms=self.clock[0],
        )

        self.assertEqual(self.core.car_pose_age_ms(self.clock[0]), 350)
        self.assertFalse(self.core.dynamic_descent_allowed(self.clock[0]))

    def test_finish_mission_allows_a_new_mission(self):
        self.prepare()
        self.core.receive(
            Topics.MISSION_START,
            ProtocolCodec.encode(self.mission_start()),
            now_ms=self.clock[0],
        )
        self.assertTrue(self.core.finish_mission("mission-0001"))
        second_config = self.mission_config(
            mission_id="mission-0002",
            mode=Mode.DYNAMIC_LANDING,
            command_id="config-2",
        )
        self.core.receive(
            Topics.MISSION_CONFIG,
            ProtocolCodec.encode(second_config),
            now_ms=self.clock[0],
        )
        self.assertTrue(self.core.set_positioning_ready("mission-0002", True))
        second = build_mission_start(
            self.car_factory,
            "mission-0002",
            Mode.DYNAMIC_LANDING,
            {"frame_id": "competition_world", "x": 0.0, "y": 0.0, "yaw": 0.0},
            start_reason="car_button",
            command_id="cmd-2",
        )

        result = self.core.receive(
            Topics.MISSION_START,
            ProtocolCodec.encode(second),
            now_ms=self.clock[0],
        )

        self.assertEqual(result.actions[0].message.payload["result"], "accepted")
        self.assertEqual(self.core.mission_id, "mission-0002")

    def car_pose(self, mission_id):
        return build_car_pose(
            self.car_factory,
            mission_id,
            frame_id="competition_world",
            source_frame="car_odom",
            x=1.0,
            y=2.0,
            yaw=1.57,
            vx=0.2,
            vy=0.0,
            yaw_rate=0.0,
            speed=0.2,
            route_segment="AB",
            route_progress_m=0.5,
            distance_to_d_m=4.0,
            moving=True,
            odom_valid=True,
            pose_quality=0.9,
        )


class FakeBus:
    def __init__(self):
        self.subscriptions = []

    def subscribe(self, topic_filter, callback, qos=0):
        self.subscriptions.append((topic_filter, callback, qos))

    def publish(self, topic, message, qos=0, retain=False):
        del topic, message, qos, retain

    def emit(self, topic, message):
        for topic_filter, callback, _qos in list(self.subscriptions):
            if _topic_matches(topic_filter, topic):
                callback(topic, message)


class EndpointTests(unittest.TestCase):
    def test_subscribes_once_to_wildcard_at_the_highest_qos(self):
        bus = FakeBus()
        endpoint = ProtocolEndpoint(bus)
        endpoint.subscribe(Topics.MISSION_START, lambda *_args: None, qos=1)
        endpoint.subscribe(Topics.CAR_POSE, lambda *_args: None, qos=0)

        self.assertEqual(
            [(item[0], item[2]) for item in bus.subscriptions],
            [("#", 1)],
        )

    def test_can_dispatch_expired_start_for_expired_ack(self):
        clock = [1_720_000_000_000]
        factory = EnvelopeFactory(
            Sender.CAR.value,
            boot_id="car-endpoint",
            clock_ms=lambda: clock[0],
        )
        start = build_mission_start(
            factory,
            "mission-expired",
            Mode.DROP,
            {"frame_id": "competition_world", "x": 0.0, "y": 0.0, "yaw": 0.0},
            command_id="cmd-expired",
        )
        decisions = []
        bus = FakeBus()
        guard = MessageGuard()
        endpoint = ProtocolEndpoint(bus, guard=guard)
        endpoint.subscribe(
            Topics.MISSION_START,
            lambda _topic, _message, decision: decisions.append(decision.status),
            qos=1,
            include_rejected=True,
        )
        clock[0] += start.ttl_ms + 1

        original_check = guard.check
        guard.check = lambda message: original_check(message, current_ms=clock[0])
        bus.emit(Topics.MISSION_START, start)

        self.assertEqual(decisions, [DeliveryStatus.EXPIRED])


class MqttBusTests(unittest.TestCase):
    def test_connect_async_retries_when_broker_starts_late(self):
        client = mock.Mock()
        mqtt_module = types.ModuleType("paho.mqtt.client")
        mqtt_module.Client = mock.Mock(return_value=client)
        mqtt_module.MQTT_ERR_SUCCESS = 0
        paho_module = types.ModuleType("paho")
        paho_mqtt_module = types.ModuleType("paho.mqtt")

        with mock.patch.dict(
            sys.modules,
            {
                "paho": paho_module,
                "paho.mqtt": paho_mqtt_module,
                "paho.mqtt.client": mqtt_module,
            },
        ):
            bus = MqttBus(MqttConfig(host="broker.local", port=1883))
            bus.connect_async(min_retry_delay=1, max_retry_delay=8)

        client.reconnect_delay_set.assert_called_once_with(min_delay=1, max_delay=8)
        client.connect_async.assert_called_once_with("broker.local", 1883, 10)


if __name__ == "__main__":
    unittest.main()
