#!/usr/bin/env python3
import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from d_task_uav_control.payload_gpio import GpioOutput  # noqa: E402


class GpioOutputTests(unittest.TestCase):
    def test_initialization_and_release_use_expected_wiringpi_commands(self):
        commands = []
        output = GpioOutput(9, active_high=True, runner=commands.append)

        output.initialize()
        output.set_release(True)
        output.set_release(True)
        output.set_release(False)

        self.assertEqual(
            commands,
            [
                ["/usr/bin/gpio", "mode", "9", "out"],
                ["/usr/bin/gpio", "write", "9", "0"],
                ["/usr/bin/gpio", "write", "9", "1"],
                ["/usr/bin/gpio", "write", "9", "0"],
            ],
        )

    def test_active_low_output_inverts_the_release_level(self):
        commands = []
        output = GpioOutput(9, active_high=False, runner=commands.append)

        output.initialize()
        output.set_release(True)
        output.set_release(False)

        self.assertEqual(
            commands,
            [
                ["/usr/bin/gpio", "mode", "9", "out"],
                ["/usr/bin/gpio", "write", "9", "1"],
                ["/usr/bin/gpio", "write", "9", "0"],
                ["/usr/bin/gpio", "write", "9", "1"],
            ],
        )


if __name__ == "__main__":
    unittest.main()
