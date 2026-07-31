"""Stable Tag 0 offset calculation and atomic mission-config update."""

from __future__ import annotations

import math
import os
import re
import shutil
import statistics
import tempfile
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, Optional, Tuple

import yaml


class UnstableCalibrationError(ValueError):
    """Raised when the observed tag centre is not stable enough to save."""


@dataclass(frozen=True)
class OffsetCalibrationResult:
    observed_u: float
    observed_v: float
    offset_u_px: float
    offset_v_px: float
    stddev_u_px: float
    stddev_v_px: float
    sample_count: int


def calculate_offset(
    samples: Iterable[Tuple[float, float]],
    principal_u: float,
    principal_v: float,
    max_stddev_px: float,
) -> OffsetCalibrationResult:
    values = list(samples)
    if not values:
        raise ValueError("at least one calibration sample is required")
    if max_stddev_px <= 0.0 or not math.isfinite(max_stddev_px):
        raise ValueError("max_stddev_px must be finite and positive")
    if not math.isfinite(principal_u) or not math.isfinite(principal_v):
        raise ValueError("principal point must be finite")
    if any(
        not math.isfinite(u) or not math.isfinite(v)
        for u, v in values
    ):
        raise ValueError("calibration samples must be finite")

    u_values = [value[0] for value in values]
    v_values = [value[1] for value in values]
    observed_u = statistics.median(u_values)
    observed_v = statistics.median(v_values)
    stddev_u = statistics.pstdev(u_values)
    stddev_v = statistics.pstdev(v_values)
    if stddev_u > max_stddev_px or stddev_v > max_stddev_px:
        raise UnstableCalibrationError(
            "tag centre is unstable: stddev=(%.3f, %.3f)px limit=%.3fpx"
            % (stddev_u, stddev_v, max_stddev_px)
        )
    return OffsetCalibrationResult(
        observed_u=observed_u,
        observed_v=observed_v,
        offset_u_px=observed_u - principal_u,
        offset_v_px=observed_v - principal_v,
        stddev_u_px=stddev_u,
        stddev_v_px=stddev_v,
        sample_count=len(values),
    )


def _format_float(value: float) -> str:
    if not math.isfinite(value):
        raise ValueError("offset must be finite")
    if abs(value) < 0.0000005:
        value = 0.0
    return ("%.6f" % value).rstrip("0").rstrip(".") + (
        ".0" if float(value).is_integer() else ""
    )


def _replace_scalar(text: str, key: str, value: float) -> str:
    pattern = re.compile(r"^(\s*" + re.escape(key) + r"\s*:\s*).*$", re.MULTILINE)
    matches = list(pattern.finditer(text))
    if len(matches) != 1:
        raise ValueError("expected exactly one %s entry" % key)
    return pattern.sub(r"\g<1>" + _format_float(value), text, count=1)


def update_offset_config(
    config_path: Path,
    offset_u_px: float,
    offset_v_px: float,
    backup_root: Path,
    timestamp: Optional[str] = None,
) -> Path:
    config_path = Path(config_path)
    backup_root = Path(backup_root)
    original = config_path.read_text(encoding="utf-8")
    parsed = yaml.safe_load(original)
    simple_follow = parsed.get("simple_follow") if isinstance(parsed, dict) else None
    if not isinstance(simple_follow, dict):
        raise ValueError("config is missing simple_follow mapping")
    for key in ("target_offset_u_px", "target_offset_v_px"):
        if key not in simple_follow:
            raise ValueError("config is missing simple_follow.%s" % key)

    updated = _replace_scalar(original, "target_offset_u_px", offset_u_px)
    updated = _replace_scalar(updated, "target_offset_v_px", offset_v_px)
    verified = yaml.safe_load(updated)
    verified_follow = verified["simple_follow"]
    serialized_u = float(_format_float(offset_u_px))
    serialized_v = float(_format_float(offset_v_px))
    if float(verified_follow["target_offset_u_px"]) != serialized_u:
        raise ValueError("failed to verify target_offset_u_px")
    if float(verified_follow["target_offset_v_px"]) != serialized_v:
        raise ValueError("failed to verify target_offset_v_px")

    backup_root.mkdir(parents=True, exist_ok=True)
    timestamp = timestamp or datetime.now().strftime("%Y%m%d-%H%M%S")
    backup_path = backup_root / (config_path.stem + "-" + timestamp + config_path.suffix)
    counter = 1
    while backup_path.exists():
        backup_path = backup_root / (
            config_path.stem + "-" + timestamp + "-%d" % counter + config_path.suffix
        )
        counter += 1
    shutil.copy2(str(config_path), str(backup_path))

    descriptor, temporary_name = tempfile.mkstemp(
        prefix=config_path.name + ".", suffix=".tmp", dir=str(config_path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as temporary_file:
            temporary_file.write(updated)
            temporary_file.flush()
            os.fsync(temporary_file.fileno())
        os.chmod(temporary_name, config_path.stat().st_mode)
        os.replace(temporary_name, str(config_path))
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
    return backup_path
