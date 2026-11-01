import json
import math
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from calibration_fixtures import make_profile

from so101_driver.calibration import CalibrationSession
from so101_driver.follower_control import FollowerControl
from so101_driver.hardware.feetech_bus import BusError
from so101_driver.profile import JOINTS
from so101_driver.so101_follower import SO101Follower

# 1.57 rad/s: 2*pi/4096 rad/tick on the arm, 1.92 rad over 2000 ticks on the gripper.
SPEEDS = {i: 1023 for i in range(1, 6)} | {6: 1635}


class FollowerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "cal.json"
        self.raw = {i: 2000 for i in range(1, 7)}
        self.hardware = {
            i: dict(
                torque_enabled=0, operating_mode=0, homing_offset=0, range_min=1000, range_max=3000
            )
            for i in range(1, 7)
        }
        self.path.write_text(json.dumps(make_profile(self.hardware)))
        self.bus = Mock()
        self.bus.ping.return_value = {i: 777 for i in range(1, 7)}
        self.bus.read_configuration.return_value = self.hardware
        self.bus.read_positions.side_effect = lambda ids: SimpleNamespace(
            positions_ticks=self.raw.copy()
        )
        self.bus.read_motion_status.side_effect = lambda ids: {
            i: {"goal_ticks": self.raw[i], "moving": False} for i in ids
        }
        self.robot = SO101Follower(self.bus, self.path)
        self.now = 0.0

    def controller(self, **kwargs):
        return FollowerControl(self.robot, clock=lambda: self.now, **kwargs)

    def test_inverse_round_trip_and_strict_limits(self):
        self.robot.validate_control()
        positions = self.robot.to_radians(self.raw)
        self.assertEqual(self.robot.to_ticks(positions), self.raw)
        for bad in ([0.0] * 5, [math.nan] * 6, [2.1] * 6, [1.6] * 6):
            with self.assertRaises(ValueError):
                self.robot.to_ticks(bad)
        observed = self.raw | {1: 3500}
        self.assertGreater(self.robot.to_radians(observed)[0], 2.0)  # No state clamp.

    def test_software_profile_direction_and_reversed_gripper(self):
        profile = dict(
            format_version=1,
            kind="so101_software_calibration",
            robot_role="follower",
            hardware_configuration={str(i): v for i, v in self.hardware.items()},
            joints={
                name: dict(
                    id=i,
                    zero_ticks=2000,
                    direction=-1,
                    radians_per_tick=2 * math.pi / 4096,
                    range_min=1000,
                    range_max=3000,
                )
                for i, name in enumerate(JOINTS[:5], 1)
            },
            gripper=dict(
                id=6, closed_ticks=3000, open_ticks=1000, closed_angle_rad=-0.1, open_angle_rad=1.7
            ),
        )
        self.path.write_text(json.dumps(profile))
        robot = SO101Follower(self.bus, self.path)
        self.assertEqual(robot.to_ticks(robot.to_radians(self.raw)), self.raw)
        self.assertAlmostEqual(robot.to_radians(self.raw | {1: 2100})[0], -100 * 2 * math.pi / 4096)
        self.assertEqual(robot.to_ticks([0.0] * 5 + [-0.1])[6], 3000)
        self.assertEqual(robot.to_ticks([0.0] * 5 + [1.7])[6], 1000)

    def test_observed_travel_beyond_servo_limits_does_not_block_valid_commands(self):
        session = CalibrationSession(self.hardware, robot_role="follower")
        session.accept(self.raw)
        session.start_from_reference()
        for step in range(11):
            session.accept({i: 500 + step * 300 for i in range(1, 7)})
        session.end_ranges()
        profile = session.profile()
        profile["joints"]["shoulder_lift"]["zero_ticks"] = 2500
        self.path.write_text(json.dumps(profile))
        robot = SO101Follower(self.bus, self.path)
        robot.validate_control()
        self.assertEqual(profile["joints"]["shoulder_pan"]["range_min"], 500)
        self.assertEqual(robot.conversions["shoulder_pan"].tick_min, 1000)
        self.assertEqual(robot.conversions["shoulder_pan"].tick_max, 3000)
        self.assertEqual(robot.to_ticks(robot.to_radians(self.raw)), self.raw)
        beyond_old_model_limit = self.raw | {2: 1100}
        positions = robot.to_radians(beyond_old_model_limit)
        self.assertLess(positions[1], math.radians(-100))
        self.assertEqual(robot.to_ticks(positions), beyond_old_model_limit)
        self.assertLess(robot.command_limits()[1][0], math.radians(-100))
        too_far = robot.to_radians(self.raw | {1: 999})
        with self.assertRaisesRegex(ValueError, "calibrated tick range"):
            robot.to_ticks(too_far)
        self.bus.write_positions.assert_not_called()
        self.bus.set_torque_enabled.assert_not_called()

    def test_gripper_calibrated_endpoints_accept_roundoff_without_relaxing_real_limits(self):
        robot = SO101Follower(self.bus, self.path)
        for ticks in (1000, 3000):
            raw = self.raw | {6: ticks}
            self.assertEqual(robot.to_ticks(robot.to_radians(raw)), raw)
        for angle in (-0.174533 - 1e-5, 1.74533 + 1e-5):
            with self.assertRaisesRegex(
                ValueError, "gripper: Target exceeds calibrated tick range"
            ):
                robot.to_ticks([0.0] * 5 + [angle])

    def test_saved_home_uses_this_calibration_and_rejects_invalid_targets(self):
        self.assertIsNone(self.robot.home_positions())
        session = CalibrationSession(self.hardware, robot_role="follower")
        session.accept(self.raw)
        session.start_from_reference()
        for step in range(11):
            session.accept({i: 1000 + step * 200 for i in range(1, 7)})
        session.end_ranges()
        profile = session.profile()
        ticks = {name: 2100 if i != 6 else 1800 for i, name in enumerate(JOINTS, 1)}
        profile["home_position"] = {"positions_ticks": ticks}
        self.path.write_text(json.dumps(profile))
        robot = SO101Follower(self.bus, self.path)
        expected = {i: ticks[name] for i, name in enumerate(JOINTS, 1)}
        self.assertEqual(robot.to_ticks(robot.home_positions()), expected)
        for invalid in (
            dict(ticks, shoulder_pan=999),
            dict(ticks, shoulder_pan=True),
            {"gripper": 1800},
        ):
            profile["home_position"] = {"positions_ticks": invalid}
            self.path.write_text(json.dumps(profile))
            robot = SO101Follower(self.bus, self.path)
            with self.assertRaises(ValueError):
                robot.home_positions()
        self.bus.write_positions.assert_not_called()

    def test_readonly_never_writes_even_when_enable_or_stop_called(self):
        control = self.controller()
        with self.assertRaisesRegex(ValueError, "Writes are disabled"):
            control.enable(1_000_000_000)
        with self.assertRaisesRegex(ValueError, "Writes are disabled"):
            control.release_torque()
        control.stop()
        control.update(self.raw)
        self.bus.write_positions.assert_not_called()
        self.bus.set_torque_enabled.assert_not_called()

    def test_release_disables_all_axes_and_verifies_torque_without_position_writes(self):
        control = self.controller(allow_writes=True)
        control.active = control.engaged = True
        for h in self.hardware.values():
            h["torque_enabled"] = 1

        def set_torque(ids, enabled):
            for motor_id in ids:
                self.hardware[motor_id]["torque_enabled"] = int(enabled)

        self.bus.set_torque_enabled.side_effect = set_torque
        control.release_torque()
        self.assertFalse(control.active)
        self.assertFalse(control.engaged)
        self.assertFalse(control.release_failed)
        self.assertTrue(all(h["torque_enabled"] == 0 for h in self.hardware.values()))
        self.assertEqual(self.bus.set_torque_enabled.call_count, 6)
        self.bus.write_positions.assert_not_called()
        self.bus.read_configuration.assert_called_once()

    def test_partial_release_failure_attempts_every_axis_and_keeps_control_disabled(self):
        control = self.controller(allow_writes=True)
        control.active = control.engaged = True
        for h in self.hardware.values():
            h["torque_enabled"] = 1

        def set_torque(ids, enabled):
            for motor_id in ids:
                if motor_id == 3:
                    raise BusError("ID=3 write failed")
                self.hardware[motor_id]["torque_enabled"] = int(enabled)

        self.bus.set_torque_enabled.side_effect = set_torque
        with self.assertRaisesRegex(ValueError, "Torque still enabled on IDs: \\[3\\]"):
            control.release_torque()
        self.assertEqual(self.bus.set_torque_enabled.call_count, 6)
        self.assertTrue(control.release_failed)
        self.assertFalse(control.active)
        with self.assertRaisesRegex(ValueError, "release torque again"):
            control.enable(1_000_000_000)
        self.bus.write_positions.assert_not_called()

    def test_release_readback_failure_is_not_reported_as_success(self):
        control = self.controller(allow_writes=True)
        self.bus.read_configuration.side_effect = BusError("readback timeout")
        with self.assertRaisesRegex(ValueError, "Torque release not confirmed: "):
            control.release_torque()
        self.assertTrue(control.release_failed)
        self.assertFalse(control.active)

    def test_enable_preloads_all_goals_before_torque_and_rejects_wrong_mode(self):
        control = self.controller(allow_writes=True, gripper_torque_limit_pct=18.5)
        self.hardware[6]["operating_mode"] = 1
        with self.assertRaisesRegex(ValueError, "operating_mode"):
            control.enable(1_000_000_000)
        self.bus.write_positions.assert_not_called()
        self.bus.set_torque_limit.assert_not_called()
        self.hardware[6]["operating_mode"] = 0
        control.enable(1_000_000_000)
        writes = [
            c
            for c in self.bus.method_calls
            if c[0] in ("write_positions", "set_torque_limit", "set_torque_enabled")
        ]
        self.assertEqual(writes[0].args[0], self.raw)
        self.assertEqual(writes[1][0], "set_torque_limit")
        self.assertEqual(writes[1].args, (6, 185))
        self.assertEqual(writes[2][0], "set_torque_enabled")
        self.assertTrue(control.active)

    def test_enable_can_hold_outside_observed_range_but_commands_stay_bounded(self):
        self.robot.conversions["elbow_flex"] = replace(
            self.robot.conversions["elbow_flex"], tick_max=2990
        )
        self.raw[3] = 2999  # Outside recorded travel, still within the servo's 3000 limit.
        control = self.controller(allow_writes=True)
        control.enable(1_000_000_000)
        self.assertTrue(control.active)
        self.assertEqual(self.bus.write_positions.call_args.args[0], self.raw)
        with self.assertRaisesRegex(ValueError, "elbow_flex: Target exceeds calibrated"):
            control.submit(
                self.robot.to_radians(self.raw),
                stamp_ns=1_010_000_000,
                ros_now_ns=1_010_000_000,
            )
        target = self.raw | {3: 2980}
        control.submit(
            self.robot.to_radians(target), stamp_ns=1_020_000_000, ros_now_ns=1_020_000_000
        )
        control.update(self.raw)
        self.assertEqual(self.bus.write_positions.call_args.args[0], target)

    def test_enable_rejects_holding_outside_actual_servo_limits_before_any_writes(self):
        for ticks in (999, 3001):
            with self.subTest(ticks=ticks):
                self.raw[3] = ticks
                control = self.controller(allow_writes=True)
                with self.assertRaisesRegex(ValueError, "elbow_flex: position.*servo range"):
                    control.enable(1_000_000_000)
                self.assertFalse(control.active)
                self.bus.write_positions.assert_not_called()
                self.bus.set_torque_enabled.assert_not_called()
                self.bus.set_torque_limit.assert_not_called()

    def test_torque_limit_failure_keeps_control_disabled_before_enabling_torque(self):
        control = self.controller(allow_writes=True)
        self.bus.set_torque_limit.side_effect = BusError("Torque limit readback mismatch ID=6")
        with self.assertRaisesRegex(BusError, "readback mismatch ID=6"):
            control.enable(1_000_000_000)
        self.assertFalse(control.active)
        self.bus.set_torque_enabled.assert_not_called()
        self.assertEqual(self.bus.write_positions.call_args.args[0], self.raw)

    def test_torque_limit_is_applied_on_each_start_but_not_each_position_command(self):
        control = self.controller(allow_writes=True, gripper_torque_limit_pct=21.26)
        control.enable(1_000_000_000)
        control.submit(
            self.robot.to_radians(self.raw), stamp_ns=1_010_000_000, ros_now_ns=1_010_000_000
        )
        control.update(self.raw)
        control.update(self.raw)
        self.bus.set_torque_limit.assert_called_once_with(6, 213)
        control.stop()
        control.enable(2_000_000_000)
        self.assertEqual(self.bus.set_torque_limit.call_count, 2)
        self.bus.set_torque_limit.assert_called_with(6, 213)

    def test_torque_limit_rejects_invalid_percentages_before_writes(self):
        for value in (-0.1, 100.1, math.nan, math.inf, True, "20"):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "gripper_torque_limit_pct"):
                    self.controller(allow_writes=True, gripper_torque_limit_pct=value)
        self.bus.set_torque_limit.assert_not_called()
        self.bus.set_torque_enabled.assert_not_called()

    def test_command_stamp_limits_and_direct_target(self):
        control = self.controller(allow_writes=True)
        control.enable(1_000_000_000)
        target = self.robot.to_radians(self.raw)
        target[0] = 0.5
        for stamp in (0, 999_999_999, 2_000_000_001):
            with self.assertRaises(ValueError):
                control.submit(target, stamp_ns=stamp, ros_now_ns=2_000_000_000)
        control.submit(target, stamp_ns=2_000_000_000, ros_now_ns=2_000_000_000)
        self.now = 0.04
        control.update(self.raw)
        sent = self.bus.write_positions.call_args.args[0]
        self.assertEqual(sent, self.robot.to_ticks(target))
        self.assertTrue(control.active)  # A target 0.5 rad away is a normal position command.
        for stamp in (2_000_000_000, 1_999_999_999):
            with self.assertRaises(ValueError):
                control.submit(target, stamp_ns=stamp, ros_now_ns=2_000_000_001)

    def test_latest_target_replaces_previous_without_waiting_for_motion(self):
        control = self.controller(
            allow_writes=True, max_velocity_rad_s=1.57, servo_acceleration_raw=20
        )
        control.enable(1_000_000_000)
        self.bus.write_positions.reset_mock()
        for angle, stamp in ((0.4, 1_010_000_000), (0.8, 1_020_000_000)):
            target = self.robot.to_radians(self.raw)
            target[0] = angle
            control.submit(target, stamp_ns=stamp, ros_now_ns=stamp)
        self.now = 0.03
        control.update(self.raw)
        self.bus.write_positions.assert_called_once_with(
            self.robot.to_ticks(target), speeds=SPEEDS, acceleration=20
        )
        # Feedback has not reached the old goal, but a new goal can reverse the move.
        target[0] = -0.6
        control.submit(target, stamp_ns=1_040_000_000, ros_now_ns=1_040_000_000)
        for now in (0.04, 0.10, 0.20):
            self.now = now
            control.update(self.raw)
            self.assertEqual(
                self.bus.write_positions.call_args.args[0], self.robot.to_ticks(target)
            )
            self.assertTrue(control.active)
        # Stopping holds the measured position, not the distant target.
        self.raw[1] = 1950
        control.stop()
        self.bus.write_positions.assert_called_with(self.raw, speeds=SPEEDS, acceleration=20)
        self.assertFalse(control.active)

    def test_every_joint_including_the_gripper_moves_at_the_set_velocity(self):
        # Below about 1251 ticks of gripper travel, the gripper has more rad/tick than the
        # arm. One shared servo speed then slowed the arm joints.
        for closed, opened in ((1000, 3000), (2000, 3200), (3000, 1800)):
            with self.subTest(closed=closed, opened=opened):
                profile = make_profile(self.hardware)
                profile["gripper"].update(closed_ticks=closed, open_ticks=opened)
                self.path.write_text(json.dumps(profile))
                robot = SO101Follower(self.bus, self.path)
                control = FollowerControl(robot, max_velocity_rad_s=1.57)
                self.assertEqual([control.speeds[i] for i in range(1, 6)], [1023] * 5)
                for motor_id, name in enumerate(JOINTS, 1):
                    step = abs(robot.conversions[name].radians_per_tick)
                    velocity = control.speeds[motor_id] * step
                    self.assertLessEqual(velocity, 1.57)
                    self.assertGreater(velocity, 1.57 - step)

    def test_invalid_servo_acceleration_is_rejected_before_writes(self):
        for value in (-1, 0, 255, 1.5, True, math.nan):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, "servo_acceleration_raw"):
                    self.controller(allow_writes=True, servo_acceleration_raw=value)
        self.bus.write_positions.assert_not_called()
        self.bus.set_torque_enabled.assert_not_called()

    def test_new_controller_can_adopt_an_existing_hold_without_toggling_torque(self):
        for h in self.hardware.values():
            h["torque_enabled"] = 1
        control = self.controller(allow_writes=True)
        control.enable(1_000_000_000)
        self.assertTrue(control.active)
        self.assertTrue(control.engaged)
        self.assertEqual(self.bus.write_positions.call_args.args[0], self.raw)
        self.bus.set_torque_enabled.assert_not_called()

    def test_unknown_mixed_torque_state_is_rejected_before_writing(self):
        self.hardware[1]["torque_enabled"] = 1
        control = self.controller(allow_writes=True)
        with self.assertRaisesRegex(ValueError, "only some joints"):
            control.enable(1_000_000_000)
        self.bus.write_positions.assert_not_called()
        self.bus.set_torque_enabled.assert_not_called()

    def test_moving_or_unreached_existing_goal_cannot_be_adopted(self):
        for h in self.hardware.values():
            h["torque_enabled"] = 1
        for goal, moving in ((2000, True), (2006, False)):
            self.bus.read_motion_status.side_effect = lambda ids: {
                i: {"goal_ticks": goal, "moving": moving} for i in ids
            }
            control = self.controller(allow_writes=True)
            with self.assertRaisesRegex(ValueError, "cannot confirm it is holding"):
                control.enable(1_000_000_000)
            self.bus.write_positions.assert_not_called()
            self.bus.set_torque_enabled.assert_not_called()

    def test_timeout_holds_measured_position_and_requires_new_enable(self):
        control = self.controller(allow_writes=True)
        control.enable(1_000_000_000)
        target = self.robot.to_radians(self.raw)
        target[0] += 0.3
        control.submit(target, stamp_ns=1_000_000_001, ros_now_ns=1_000_000_001)
        self.now = 0.51
        self.raw[1] = 2005
        control.update(self.raw)
        self.assertFalse(control.active)
        self.assertEqual(self.bus.write_positions.call_args.args[0], self.raw)
        with self.assertRaisesRegex(ValueError, "disabled"):
            control.submit(target, stamp_ns=2_000_000_000, ros_now_ns=2_000_000_000)
        self.assertNotIn(False, [c.args[1] for c in self.bus.set_torque_enabled.call_args_list])

    def test_late_callback_cannot_revive_and_source_age_shortens_deadline(self):
        control = self.controller(allow_writes=True)
        control.enable(1_000_000_000)
        target = self.robot.to_radians(self.raw)
        control.submit(target, stamp_ns=1_000_000_000, ros_now_ns=1_400_000_000)
        self.now = 0.11
        with self.assertRaisesRegex(ValueError, "timeout"):
            control.submit(target, stamp_ns=1_600_000_000, ros_now_ns=1_600_000_000)
        self.assertFalse(control.active)

    def test_write_failure_disarms_and_attempts_hold(self):
        control = self.controller(allow_writes=True)
        control.enable(1_000_000_000)
        control.submit(
            self.robot.to_radians(self.raw), stamp_ns=1_000_000_001, ros_now_ns=1_000_000_001
        )
        self.bus.write_positions.side_effect = [BusError("write timeout"), None]
        self.now = 0.03
        with self.assertRaises(BusError):
            control.update(self.raw)
        self.assertFalse(control.active)
        self.assertEqual(self.bus.write_positions.call_args.args[0], self.raw)

    def test_failed_hold_is_visible_and_control_stays_disabled(self):
        control = self.controller(allow_writes=True)
        control.enable(1_000_000_000)
        self.bus.read_positions.side_effect = BusError("disconnected")
        with self.assertRaises(BusError):
            control.stop()
        self.assertFalse(control.active)
        self.assertTrue(control.hold_failed)
        self.assertIn("NOT confirmed", control.message)

    def test_partial_enable_failure_attempts_hold_without_releasing_torque(self):
        control = self.controller(allow_writes=True)
        self.bus.set_torque_enabled.side_effect = BusError("ID=3 enable failed")
        with self.assertRaises(BusError):
            control.enable(1_000_000_000)
        self.assertFalse(control.active)
        self.assertTrue(control.engaged)
        self.assertEqual(self.bus.write_positions.call_count, 2)
        self.assertIn("Torque enabling failed", control.message)
