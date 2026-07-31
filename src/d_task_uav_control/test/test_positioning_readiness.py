#!/usr/bin/env python3
import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from d_task_uav_control.positioning_readiness import (  # noqa: E402
    PositioningReadiness,
    PositioningReadinessConfig,
    build_roslaunch_command,
)


class PositioningReadinessTests(unittest.TestCase):
    def setUp(self):
        self.readiness = PositioningReadiness(
            PositioningReadinessConfig(
                startup_timeout_s=5.0,
                stable_time_s=1.0,
                message_timeout_s=0.3,
                max_xy_drift_m=0.05,
                max_z_drift_m=0.05,
            )
        )
        self.readiness.begin("mission-1", "DROP", 10.0)

    def feed(self, stamp, x=1.0, y=2.0, z=0.1):
        self.readiness.observe_fastlio(stamp)
        self.readiness.observe_vision(stamp)
        self.readiness.observe_px4(stamp, x, y, z)

    def test_requires_all_three_position_streams(self):
        self.readiness.observe_fastlio(10.0)
        self.readiness.observe_px4(10.0, 1.0, 2.0, 0.1)

        status = self.readiness.evaluate(10.1)

        self.assertFalse(status.ready)
        self.assertEqual(status.state, "POSITIONING_INIT")

    def test_becomes_ready_after_continuous_stable_window(self):
        for index in range(11):
            stamp = 10.0 + index * 0.1
            self.feed(stamp, x=1.0 + index * 0.001)
            status = self.readiness.evaluate(stamp)

        self.assertTrue(status.ready)
        self.assertEqual(status.state, "WAIT_START")
        self.assertEqual(status.mission_id, "mission-1")
        self.assertAlmostEqual(status.home_x, 1.01)
        self.assertAlmostEqual(status.home_y, 2.0)
        self.assertAlmostEqual(status.home_z, 0.1)

    def test_position_jump_restarts_stability_timer(self):
        self.feed(10.0)
        self.readiness.evaluate(10.0)
        self.feed(10.8, x=1.2)
        self.readiness.evaluate(10.8)
        self.feed(11.1, x=1.2)

        status = self.readiness.evaluate(11.1)

        self.assertFalse(status.ready)

    def test_stale_stream_resets_stability(self):
        self.feed(10.0)
        self.readiness.evaluate(10.0)

        stale = self.readiness.evaluate(10.31)
        self.feed(10.4)
        recovered = self.readiness.evaluate(10.4)

        self.assertFalse(stale.ready)
        self.assertFalse(recovered.ready)

    def test_startup_timeout_reports_fault(self):
        status = self.readiness.evaluate(15.01)

        self.assertFalse(status.ready)
        self.assertEqual(status.state, "POSITIONING_FAULT")
        self.assertEqual(status.fault_text, "positioning_startup_timeout")

    def test_ready_state_is_not_overwritten_by_startup_timeout(self):
        for index in range(11):
            stamp = 10.0 + index * 0.1
            self.feed(stamp)
            self.readiness.evaluate(stamp)
        self.assertTrue(self.readiness.status.ready)

        self.feed(30.0, x=1.01)
        status = self.readiness.evaluate(30.0)

        self.assertTrue(status.ready)
        self.assertEqual(status.state, "WAIT_START")
        self.assertEqual(status.fault_code, 0)
        self.assertEqual(status.fault_text, "")

    def test_new_configuration_clears_previous_ready_and_home(self):
        for index in range(11):
            stamp = 10.0 + index * 0.1
            self.feed(stamp)
            self.readiness.evaluate(stamp)
        self.assertTrue(self.readiness.status.ready)

        self.readiness.begin("mission-2", "DYNAMIC_LANDING", 20.0)

        self.assertFalse(self.readiness.status.ready)
        self.assertEqual(self.readiness.status.mission_id, "mission-2")
        self.assertIsNone(self.readiness.status.home_x)

    def test_external_launch_failure_sets_fault_status(self):
        self.readiness.fail(1302, "fastlio_node_conflict")
        status = self.readiness.evaluate(10.1)

        self.assertFalse(status.ready)
        self.assertEqual(status.state, "POSITIONING_FAULT")
        self.assertEqual(status.fault_code, 1302)
        self.assertEqual(status.fault_text, "fastlio_node_conflict")

    def test_roslaunch_uses_the_positioning_workspace_environment(self):
        command = build_roslaunch_command(
            "/home/orangepi/ros_ws/devel/env.sh",
            "lidar_to_mavros",
            "fastlio_to_px4_mid360_direct.launch",
            ["rviz:=false", "zero_origin:=true"],
        )

        self.assertEqual(
            command,
            [
                "/home/orangepi/ros_ws/devel/env.sh",
                "roslaunch",
                "lidar_to_mavros",
                "fastlio_to_px4_mid360_direct.launch",
                "rviz:=false",
                "zero_origin:=true",
            ],
        )


if __name__ == "__main__":
    unittest.main()
