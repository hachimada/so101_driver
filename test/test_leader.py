import json
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from calibration_fixtures import make_profile

from so101_driver.profile import JOINTS
from so101_driver.so101_follower import SO101Follower
from so101_driver.so101_leader import SO101Leader


class LeaderTests(unittest.TestCase):
    def test_recorded_zero_direction_and_units_without_double_homing(self):
        hardware = {
            i: dict(range_min=1000, range_max=3000, homing_offset=500, operating_mode=0)
            for i in range(1, 7)
        }
        profile = make_profile(hardware, role="leader", zero_ticks=2300)
        profile["joints"]["shoulder_lift"]["direction"] = -1
        bus = SimpleNamespace(
            ping=lambda ids: {i: 777 for i in ids},
            read_configuration=lambda ids: hardware,
            read_positions=lambda ids: SimpleNamespace(
                positions_ticks={1: 2300, 2: 2400, 3: 2300, 4: 2300, 5: 2300, 6: 2000}
            ),
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "cal.json"
            path.write_text(json.dumps(profile))
            leader = SO101Leader(bus, path)
            leader.validate()
            positions = leader.read_joint_positions()
        self.assertEqual(positions[0], 0)
        self.assertAlmostEqual(positions[1], -100 * 2 * math.pi / 4096)
        self.assertAlmostEqual(positions[5], (-0.174533 + 1.74533) / 2)  # Halfway open.

    def test_lerobot_json_is_rejected_by_both_models_before_bus_access(self):
        legacy = {
            name: dict(id=i, range_min=1000, range_max=3000, homing_offset=0, drive_mode=0)
            for i, name in enumerate(JOINTS, 1)
        }
        bus = Mock()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "legacy.json"
            path.write_text(json.dumps(legacy))
            for model in (SO101Leader, SO101Follower):
                with self.subTest(model=model.__name__):
                    with self.assertRaisesRegex(ValueError, "so101_calibrate"):
                        model(bus, path)
        self.assertEqual(bus.mock_calls, [])
