import json
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from calibration_fixtures import make_profile

from so101_driver.profile import (
    GRIPPER_CLOSED_RAD,
    GRIPPER_OPEN_RAD,
    CalibrationProfile,
    gripper_conversion,
)
from so101_driver.so101_follower import SO101Follower
from so101_driver.so101_leader import SO101Leader


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "profile.json"
        self.hardware = {
            i: {"range_min": 1000, "range_max": 3000, "homing_offset": 0, "operating_mode": 0}
            for i in range(1, 7)
        }

    def write(self, role, **gripper):
        profile = make_profile(self.hardware, role=role)
        profile["gripper"].update(gripper)
        self.path.write_text(json.dumps(profile))
        return self.path

    def test_gripper_endpoints_map_in_either_direction(self):
        for closed, opened in ((1000, 3000), (3000, 1000)):
            with self.subTest(closed=closed, opened=opened):
                c = gripper_conversion(closed, opened)
                self.assertAlmostEqual(c.to_radians(closed), GRIPPER_CLOSED_RAD)
                self.assertAlmostEqual(c.to_radians(opened), GRIPPER_OPEN_RAD)
                self.assertAlmostEqual(
                    c.to_radians(2000), (GRIPPER_CLOSED_RAD + GRIPPER_OPEN_RAD) / 2
                )
                beyond_open = opened + (opened - closed) // 10
                self.assertGreater(c.to_radians(beyond_open), GRIPPER_OPEN_RAD)
                self.assertAlmostEqual(c.to_clamped_radians(beyond_open), GRIPPER_OPEN_RAD)
                self.assertEqual(c.to_ticks(GRIPPER_OPEN_RAD), opened)

    def test_invalid_gripper_angles_are_rejected(self):
        for angles in ((0.0, 0.0), (math.nan, 1.0), (0.0, math.inf)):
            with self.subTest(angles=angles):
                path = self.write("leader", closed_angle_rad=angles[0], open_angle_rad=angles[1])
                with self.assertRaisesRegex(ValueError, "finite and differ"):
                    CalibrationProfile(path, role="leader")

    def test_leader_and_follower_share_one_conversion(self):
        # Within the recorded travel both give the same angles. Past the open position
        # the leader gripper stays fully open, while the follower reports what it measures.
        bus = SimpleNamespace(read_positions=lambda ids: SimpleNamespace(positions_ticks=raw))
        for gripper, leader_gripper in ((2000, None), (2900, None), (3100, GRIPPER_OPEN_RAD)):
            raw = {1: 2100, 2: 1900, 3: 2000, 4: 2050, 5: 1950, 6: gripper}
            with self.subTest(gripper=gripper):
                leader = SO101Leader(bus, self.write("leader"))
                follower = SO101Follower(bus, self.write("follower"))
                measured = follower.to_radians(raw)
                read = leader.read_joint_positions()
                self.assertEqual(read[:5], measured[:5])
                self.assertAlmostEqual(read[5], leader_gripper or measured[5])


if __name__ == "__main__":
    unittest.main()
