"""GPIO output helper for the Orange Pi payload actuator and bench test node."""

from __future__ import annotations

import subprocess
from typing import Callable, List, Optional


CommandRunner = Callable[[List[str]], object]


class GpioOutput:
    """Set a WiringOP-numbered GPIO output through the board-provided gpio tool."""

    def __init__(
        self,
        wiringpi_pin: int,
        active_high: bool,
        runner: Optional[CommandRunner] = None,
    ) -> None:
        if wiringpi_pin < 0:
            raise ValueError("wiringpi_pin must be non-negative")
        self._pin = wiringpi_pin
        self._active_high = bool(active_high)
        self._runner = runner or self._run
        self._level_known = False
        self._level_high = False

    def initialize(self) -> None:
        self._execute("mode", "out")
        self.set_release(False, force=True)

    def set_release(self, release: bool, force: bool = False) -> None:
        level_high = bool(release) if self._active_high else not bool(release)
        if not force and self._level_known and self._level_high == level_high:
            return
        self._execute("write", "1" if level_high else "0")
        self._level_known = True
        self._level_high = level_high

    def safe_off(self) -> None:
        self.set_release(False, force=True)

    def _execute(self, command: str, value: str) -> None:
        self._runner(["/usr/bin/gpio", command, str(self._pin), value])

    @staticmethod
    def _run(command: List[str]) -> None:
        subprocess.run(command, check=True)
