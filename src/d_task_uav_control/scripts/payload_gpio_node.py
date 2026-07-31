#!/usr/bin/env python3
"""Safe ROS wrapper for the payload-release GPIO output.

The mission node owns pulse timing and publishes ``Bool`` on the configured
topic.  This node only drives the electrical output and always returns it to
the non-release level at startup and shutdown.
"""

from __future__ import annotations

import subprocess

import rospy
from std_msgs.msg import Bool

from d_task_uav_control.payload_gpio import GpioOutput


class PayloadGpioNode:
    def __init__(self) -> None:
        pin = int(rospy.get_param("~payload/wiringpi_pin", 9))
        active_high = bool(rospy.get_param("~payload/active_high", True))
        topic = str(rospy.get_param("~topics/payload_release", "/d_task/payload/release"))
        self._output = GpioOutput(pin, active_high)
        try:
            self._output.initialize()
        except (OSError, subprocess.CalledProcessError) as exc:
            raise rospy.ROSException("payload GPIO initialisation failed: %s" % exc)
        self._subscriber = rospy.Subscriber(topic, Bool, self._callback, queue_size=1)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo(
            "[payload_gpio] ready: topic=%s wPi=%d active_high=%s",
            topic,
            pin,
            active_high,
        )

    def _callback(self, message: Bool) -> None:
        try:
            self._output.set_release(message.data)
        except (OSError, subprocess.CalledProcessError) as exc:
            rospy.logerr("[payload_gpio] GPIO write failed: %s", exc)

    def _shutdown(self) -> None:
        try:
            self._output.safe_off()
        except (OSError, subprocess.CalledProcessError) as exc:
            rospy.logerr("[payload_gpio] GPIO safe-off failed: %s", exc)


def main() -> None:
    rospy.init_node("payload_gpio")
    PayloadGpioNode()
    rospy.spin()


if __name__ == "__main__":
    main()
