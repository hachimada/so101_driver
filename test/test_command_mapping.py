import math
import tempfile
import unittest
from pathlib import Path

from so101_driver.command_mapping import (
    AbsoluteTeleop,
    JointSnapshot,
    load_follower_model,
)
from so101_driver.profile import JOINTS


class CommandMappingTests(unittest.TestCase):
    def test_gui_and_teleop_require_calibration_profile(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "follower.yaml"
            for params in ('calibration_profile: ""', 'calibration_file: "/legacy.json"'):
                config.write_text(f"/follower/so101_follower:\n  ros__parameters:\n    {params}\n")
                with self.assertRaisesRegex(ValueError, "calibration_profile"):
                    load_follower_model(config)

    def test_snapshot_reorders_names_and_rejects_invalid_or_old_data(self):
        snapshot = JointSnapshot()
        self.assertTrue(
            snapshot.update(
                list(reversed(JOINTS)),
                list(range(6)),
                1_000_000_000,
                ros_now_ns=1_000_000_000,
                now=10,
            )
        )
        self.assertEqual(snapshot.positions, list(reversed(range(6))))
        for names, values, stamp in (
            (JOINTS, [math.nan] * 6, 1_100_000_000),
            ([JOINTS[0]] * 6, [0.0] * 6, 1_100_000_000),
            (JOINTS, [0.0] * 6, 1_200_000_000),
            (JOINTS, [0.0] * 6, 1_000_000_000),
        ):
            self.assertFalse(
                snapshot.update(names, values, stamp, ros_now_ns=1_100_000_000, now=10.1)
            )
        self.assertTrue(snapshot.fresh(ros_now_ns=1_100_000_000, now=10.1))
        self.assertFalse(snapshot.fresh(ros_now_ns=1_600_000_000, now=10.6))
        self.assertFalse(snapshot.fresh(ros_now_ns=900_000_000, now=10.1))

    def test_absolute_mapping_scales_angles_and_clips_each_joint(self):
        mapping = AbsoluteTeleop([(-1, 1)] * 6, [1, -1, 2, 0, 1, 1])
        self.assertEqual(mapping.target([0.3] * 6), [0.3, -0.3, 0.6, 0, 0.3, 0.3])
        result = mapping.target([0.5, 0.5, 3, 3, -3, 3])
        self.assertEqual(result, [0.5, -0.5, 1, 0, -1, 1])

    def test_alignment_rejects_unreachable_targets_instead_of_clipping(self):
        mapping = AbsoluteTeleop([(-1, 1)] * 6, [1] * 6)
        for i, name in enumerate(JOINTS):
            leader = [0.3] * 6
            leader[i] = 1.1
            with self.assertRaisesRegex(ValueError, name):
                mapping.target(leader, require_in_range=True)
        self.assertEqual(mapping.target([1.0] * 6, require_in_range=True), [1.0] * 6)

    def test_nonfinite_targets_are_rejected(self):
        mapping = AbsoluteTeleop([(-1, 1)] * 6, [1] * 6)
        for leader in ([math.nan] * 6, [math.inf] * 6, [0.1] * 5):
            with self.assertRaises(ValueError):
                mapping.target(leader)

    def test_alignment_tolerance_near_limits_never_expands_motor_commands(self):
        mapping = AbsoluteTeleop([(-1, 1)] * 6, [1, -1, 1, 1, 1, 1])
        leader = [1.04] * 6
        self.assertEqual(
            mapping.target(leader, require_in_range=True, range_tolerance_rad=0.05),
            [1, -1, 1, 1, 1, 1],
        )
        self.assertEqual(mapping.leader_angles(leader), [1.04, -1.04, 1.04, 1.04, 1.04, 1.04])
        with self.assertRaisesRegex(ValueError, "by more than"):
            mapping.target([1.051] * 6, require_in_range=True, range_tolerance_rad=0.05)

    def test_invalid_scales_are_rejected(self):
        for scales in ([1] * 5, [float("inf")] * 6):
            with self.assertRaises(ValueError):
                AbsoluteTeleop([(-1, 1)] * 6, scales)
