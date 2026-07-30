#!/usr/bin/env python3
"""Manual high/low GPIO test node for the payload driver input."""

from __future__ import annotations

import subprocess

import rospy
from std_msgs.msg import Bool

from d_task_uav_control.payload_gpio import GpioOutput


class PayloadGpioTestNode:
    def __init__(self) -> None:
        self._pin = int(rospy.get_param("~wiringpi_pin", 9))
        self._active_high = bool(rospy.get_param("~active_high", True))
        self._topic = rospy.get_param("~topic", "/payload_gpio_test/set")
        self._output = GpioOutput(self._pin, self._active_high)
        self._output.initialize()
        self._subscriber = rospy.Subscriber(
            self._topic, Bool, self._set_callback, queue_size=1
        )
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "[payload_gpio_test] ready: topic=%s wPi=%d active_high=%s",
            self._topic,
            self._pin,
            self._active_high,
        )

    def _set_callback(self, message: Bool) -> None:
        try:
            self._output.set_release(message.data)
            rospy.loginfo(
                "[payload_gpio_test] output %s",
                "HIGH" if message.data else "LOW",
            )
        except (OSError, subprocess.CalledProcessError) as exc:
            rospy.logerr("[payload_gpio_test] GPIO write failed: %s", exc)

    def shutdown(self) -> None:
        try:
            self._output.safe_off()
        except (OSError, subprocess.CalledProcessError) as exc:
            rospy.logerr("[payload_gpio_test] GPIO safe-off failed: %s", exc)


def main() -> None:
    rospy.init_node("payload_gpio_test")
    PayloadGpioTestNode()
    rospy.spin()


if __name__ == "__main__":
    main()
