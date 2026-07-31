#!/usr/bin/env python3

import tempfile
import unittest
from pathlib import Path

from d_task_uav_control.tag0_offset_calibration import (
    UnstableCalibrationError,
    calculate_offset,
    update_offset_config,
)


class Tag0OffsetCalibrationTests(unittest.TestCase):
    def test_calculates_median_offset_from_stable_rectified_samples(self):
        result = calculate_offset(
            [(345.0, 237.1), (345.2, 237.0), (344.8, 237.2)],
            principal_u=322.8,
            principal_v=237.3,
            max_stddev_px=0.5,
        )

        self.assertAlmostEqual(result.observed_u, 345.0)
        self.assertAlmostEqual(result.observed_v, 237.1)
        self.assertAlmostEqual(result.offset_u_px, 22.2)
        self.assertAlmostEqual(result.offset_v_px, -0.2)

    def test_rejects_unstable_samples(self):
        with self.assertRaises(UnstableCalibrationError):
            calculate_offset(
                [(320.0, 240.0), (330.0, 240.0), (340.0, 240.0)],
                principal_u=322.8,
                principal_v=237.3,
                max_stddev_px=1.0,
            )

    def test_updates_only_offsets_and_creates_external_backup(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            config_path = root / "d_task_uav.yaml"
            backup_root = root / "backups"
            original = (
                "simple_follow:\n"
                "  target_offset_u_px: 0.0\n"
                "  target_offset_v_px: 0.0\n"
                "  pid:\n"
                "    kp: 0.65\n"
            )
            config_path.write_text(original, encoding="utf-8")

            backup_path = update_offset_config(
                config_path,
                offset_u_px=22.21,
                offset_v_px=-0.24,
                backup_root=backup_root,
                timestamp="20260801-120000",
            )

            updated = config_path.read_text(encoding="utf-8")
            self.assertIn("target_offset_u_px: 22.21", updated)
            self.assertIn("target_offset_v_px: -0.24", updated)
            self.assertIn("kp: 0.65", updated)
            self.assertEqual(backup_path.read_text(encoding="utf-8"), original)

    def test_refuses_config_without_both_offset_keys(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            config_path = root / "d_task_uav.yaml"
            original = "simple_follow:\n  target_offset_u_px: 0.0\n"
            config_path.write_text(original, encoding="utf-8")

            with self.assertRaises(ValueError):
                update_offset_config(
                    config_path,
                    offset_u_px=1.0,
                    offset_v_px=2.0,
                    backup_root=root / "backups",
                    timestamp="20260801-120000",
                )

            self.assertEqual(config_path.read_text(encoding="utf-8"), original)

    def test_accepts_offsets_rounded_to_config_precision(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            config_path = root / "d_task_uav.yaml"
            config_path.write_text(
                "simple_follow:\n"
                "  target_offset_u_px: 0.0\n"
                "  target_offset_v_px: 0.0\n",
                encoding="utf-8",
            )

            update_offset_config(
                config_path,
                offset_u_px=37.01484351,
                offset_v_px=180.54268049,
                backup_root=root / "backups",
                timestamp="20260801-120000",
            )

            updated = config_path.read_text(encoding="utf-8")
            self.assertIn("target_offset_u_px: 37.014844", updated)
            self.assertIn("target_offset_v_px: 180.54268", updated)


if __name__ == "__main__":
    unittest.main()
