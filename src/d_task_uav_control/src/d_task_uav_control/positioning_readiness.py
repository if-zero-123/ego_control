"""Pure positioning-readiness state used by the ROS supervisor."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Dict, Iterable, List, Optional


def build_roslaunch_command(
    workspace_env: str,
    package: str,
    launch_file: str,
    launch_args: Iterable[str],
) -> List[str]:
    """Build a roslaunch command under the positioning workspace environment."""

    command = ["roslaunch", package, launch_file]
    command.extend(str(value) for value in launch_args)
    return [workspace_env, *command] if workspace_env else command


@dataclass(frozen=True)
class PositioningReadinessConfig:
    startup_timeout_s: float = 15.0
    stable_time_s: float = 3.0
    message_timeout_s: float = 0.3
    max_xy_drift_m: float = 0.05
    max_z_drift_m: float = 0.05


@dataclass
class PositioningStatus:
    mission_id: str = ""
    mode: str = ""
    state: str = "NOT_READY"
    ready: bool = False
    fault_code: int = 0
    fault_text: str = ""
    home_x: Optional[float] = None
    home_y: Optional[float] = None
    home_z: Optional[float] = None

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)


class PositioningReadiness:
    def __init__(self, config: PositioningReadinessConfig) -> None:
        self.config = config
        self.status = PositioningStatus()
        self._started_s: Optional[float] = None
        self._stable_started_s: Optional[float] = None
        self._reference_position = None
        self._fastlio_stamp_s: Optional[float] = None
        self._vision_stamp_s: Optional[float] = None
        self._px4_stamp_s: Optional[float] = None
        self._px4_position = None

    def begin(self, mission_id: str, mode: str, now_s: float) -> None:
        self.status = PositioningStatus(
            mission_id=mission_id,
            mode=mode,
            state="POSITIONING_INIT",
        )
        self._started_s = now_s
        self._stable_started_s = None
        self._reference_position = None
        self._fastlio_stamp_s = None
        self._vision_stamp_s = None
        self._px4_stamp_s = None
        self._px4_position = None

    def observe_fastlio(self, stamp_s: float) -> None:
        self._fastlio_stamp_s = stamp_s

    def observe_vision(self, stamp_s: float) -> None:
        self._vision_stamp_s = stamp_s

    def observe_px4(self, stamp_s: float, x: float, y: float, z: float) -> None:
        self._px4_stamp_s = stamp_s
        self._px4_position = (x, y, z)

    def fail(self, fault_code: int, fault_text: str) -> PositioningStatus:
        self.status.state = "POSITIONING_FAULT"
        self.status.ready = False
        self.status.fault_code = fault_code
        self.status.fault_text = fault_text
        return self.status

    def evaluate(self, now_s: float) -> PositioningStatus:
        if self._started_s is None:
            return self.status
        if self.status.state == "POSITIONING_FAULT":
            return self.status
        if self.status.ready:
            return self.status
        if now_s - self._started_s > self.config.startup_timeout_s:
            self.status.state = "POSITIONING_FAULT"
            self.status.ready = False
            self.status.fault_code = 1301
            self.status.fault_text = "positioning_startup_timeout"
            return self.status

        if not self._streams_fresh(now_s):
            self._stable_started_s = None
            self._reference_position = None
            self.status.state = "POSITIONING_INIT"
            self.status.ready = False
            return self.status

        assert self._px4_position is not None
        if self._stable_started_s is None:
            self._stable_started_s = now_s
            self._reference_position = self._px4_position
        elif self._drift_exceeded(self._px4_position):
            self._stable_started_s = now_s
            self._reference_position = self._px4_position

        if now_s - self._stable_started_s + 1e-9 >= self.config.stable_time_s:
            self.status.state = "WAIT_START"
            self.status.ready = True
            self.status.home_x, self.status.home_y, self.status.home_z = self._px4_position
        return self.status

    def _streams_fresh(self, now_s: float) -> bool:
        stamps = (self._fastlio_stamp_s, self._vision_stamp_s, self._px4_stamp_s)
        return all(
            stamp is not None and 0.0 <= now_s - stamp <= self.config.message_timeout_s
            for stamp in stamps
        )

    def _drift_exceeded(self, position) -> bool:
        assert self._reference_position is not None
        dx = position[0] - self._reference_position[0]
        dy = position[1] - self._reference_position[1]
        dz = abs(position[2] - self._reference_position[2])
        return (
            math.hypot(dx, dy) > self.config.max_xy_drift_m
            or dz > self.config.max_z_drift_m
        )
