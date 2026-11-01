import json
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from so101_driver.calibration import CalibrationSession, save_profile
from so101_driver.profile import CalibrationProfile
from so101_driver.so101_leader import SO101Leader


def hardware():
    return {
        i: dict(
            homing_offset=100, range_min=500, range_max=3500, operating_mode=0, torque_enabled=0
        )
        for i in range(1, 7)
    }


def complete_session():
    s = CalibrationSession(hardware())
    s.accept({i: 2000 for i in range(1, 7)})
    s.start_from_reference()
    for step in range(11):
        raw = {i: 2000 + step * 10 + i for i in range(1, 7)}
        raw[6] = 2000 - step * 50
        s.accept(raw)
    s.end_ranges()
    return s


class CalibrationTests(unittest.TestCase):
    def test_role_must_match_and_is_required(self):
        profile = complete_session().profile()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profile.json"
            save_profile(path, profile)
            with self.assertRaisesRegex(ValueError, "role mismatch"):
                CalibrationProfile(path, role="follower")
            profile["robot_role"] = "follower"
            save_profile(path, profile, overwrite=True)
            CalibrationProfile(path, role="follower")
            with self.assertRaisesRegex(ValueError, "role mismatch"):
                CalibrationProfile(path, role="leader")
            del profile["robot_role"]
            save_profile(path, profile, overwrite=True)
            for role in ("leader", "follower"):
                with self.assertRaisesRegex(ValueError, "role mismatch.*got None"):
                    CalibrationProfile(path, role=role)

    def test_invalid_or_stale_samples_not_captured(self):
        s = CalibrationSession(hardware())
        with self.assertRaises(ValueError):
            s.start_from_reference()
        s.accept({i: 2000 for i in range(1, 7)})
        s.fail("voltage fault")
        with self.assertRaises(ValueError):
            s.start_from_reference()
        s.accept({i: 2000 for i in range(1, 7)}, now=10.0)
        with self.assertRaises(ValueError):
            s.capture(now=11.0)
        with self.assertRaises(ValueError):
            s.accept({i: -1 for i in range(1, 7)})

    def test_too_little_motion_does_not_pass(self):
        s = CalibrationSession(hardware())
        s.accept({i: 2000 for i in range(1, 7)})
        s.begin_ranges()
        with self.assertRaises(ValueError):
            s.end_ranges()

    def test_simultaneous_collection_and_resume_preserve_all_ranges(self):
        s = CalibrationSession(hardware())
        s.accept({i: 2000 for i in range(1, 7)})
        s.start_from_reference()
        for tick in range(2000, 2101, 10):
            s.accept({i: tick if i != 4 else 2000 for i in range(1, 7)})
        with self.assertRaisesRegex(ValueError, "wrist_flex"):
            s.end_ranges()
        self.assertEqual(s.ranges[1]["max"], 2100)
        s.begin_ranges()
        for tick in range(2000, 2201, 20):
            s.accept({i: tick if i == 4 else 2000 for i in range(1, 7)})
        s.end_ranges()
        self.assertTrue(s.ranges_complete)
        self.assertEqual(s.ranges[1]["max"], 2100)
        self.assertEqual(s.ranges[4]["max"], 2200)
        self.assertEqual(set(s.ranges), set(range(1, 7)))
        self.assertEqual(s.ranges[5]["max"], 2100)
        profile = s.profile()
        self.assertEqual(profile["joints"]["wrist_roll"]["range_min"], 0)
        self.assertEqual(profile["joints"]["wrist_roll"]["range_max"], 4095)
        # One turn is 4096 ticks.
        for joint in profile["joints"].values():
            self.assertAlmostEqual(joint["radians_per_tick"] * 4096, 2 * math.pi)

    def test_atomic_overwrite_requires_explicit_authorization(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "profile.json"
            save_profile(path, {"version": 1})
            with self.assertRaises(FileExistsError):
                save_profile(path, {"version": 2})
            self.assertEqual(json.loads(path.read_text()), {"version": 1})
            save_profile(path, {"version": 2}, overwrite=True)
            self.assertEqual(json.loads(path.read_text()), {"version": 2})
            self.assertEqual(list(Path(tmp).iterdir()), [path])

    def test_profile_round_trip_direction_and_hardware_mismatch(self):
        profile = complete_session().profile()
        bus = SimpleNamespace(
            ping=lambda ids: {i: 777 for i in ids},
            read_configuration=lambda ids: hardware(),
            read_positions=lambda ids: SimpleNamespace(positions_ticks={i: 2000 for i in ids}),
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "profile.json"
            save_profile(path, profile)
            with self.assertRaises(FileExistsError):
                save_profile(path, profile)
            leader = SO101Leader(bus, path)
            leader.validate()
            positions = leader.read_joint_positions()
            self.assertEqual(positions[:5], [0.0] * 5)
            self.assertAlmostEqual(positions[5], -0.174533)  # Closed.
            # Beyond the recorded open position, the leader gripper stays fully open.
            bus.read_positions = lambda ids: SimpleNamespace(
                positions_ticks={**{i: 2000 for i in ids}, 6: 1400}
            )
            self.assertAlmostEqual(leader.read_joint_positions()[5], 1.74533)
            changed = hardware()
            changed[1]["homing_offset"] = 101
            bus.read_configuration = lambda ids: changed
            with self.assertRaisesRegex(ValueError, "Hardware changed"):
                leader.validate()
            # Calibration file stayed independent of hardware writes.
            self.assertEqual(
                json.loads(path.read_text())["hardware_configuration"]["1"]["homing_offset"], 100
            )

    def test_one_pass_gripper_endpoints_work_in_both_directions(self):
        for direction in (-1, 1):
            s = CalibrationSession(hardware())
            s.accept({i: 2000 for i in range(1, 7)})
            s.start_from_reference()
            for step in range(11):
                raw = {i: 2000 + 10 * step for i in range(1, 7)}
                raw[6] = 2000 + direction * 50 * step
                s.accept(raw)
            s.end_ranges()
            # Later faults cannot invalidate already completed healthy measurements.
            s.fail("voltage fault after recording")
            profile = s.profile()
            self.assertEqual(profile["gripper"]["closed_ticks"], 2000)
            self.assertEqual(profile["gripper"]["open_ticks"], 2000 + direction * 500)
