"""ROS callbacks with a simulated bus. No physical motor writes."""

import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, call, patch

from calibration_fixtures import make_profile

try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticStatus
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import JointState
    from std_srvs.srv import SetBool, Trigger

    from so101_driver.follower_node import FollowerNode
    from so101_driver.hardware.feetech_bus import BusError
    from so101_driver.profile import JOINTS

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "Requires ROS2 Humble")
class FollowerNodeTests(unittest.TestCase):
    def setUp(self):
        resolver = patch("so101_driver.follower_node.resolve_device_id", return_value="/fake")
        self.resolve = resolver.start()
        self.addCleanup(resolver.stop)
        rclpy.init()
        self.addCleanup(rclpy.try_shutdown)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "cal.json"
        hardware = {
            i: dict(
                torque_enabled=0, operating_mode=0, range_min=1000, range_max=3000, homing_offset=0
            )
            for i in range(1, 7)
        }
        path.write_text(json.dumps(make_profile(hardware)))
        self.parameters = [
            Parameter("device_id", value="fake-device"),
            Parameter("calibration_profile", value=str(path)),
        ]
        self.bus = Mock(port_name="/fake")
        self.raw = {i: 2000 for i in range(1, 7)}
        self.bus.read_positions.side_effect = lambda ids: SimpleNamespace(
            positions_ticks=self.raw.copy()
        )
        self.bus.ping.return_value = {i: 777 for i in range(1, 7)}
        self.bus.read_configuration.return_value = hardware

    def node(self, allow_writes=False, *, release_on_shutdown=False, cleanup=True):
        with patch("so101_driver.follower_node.FeetechBus", return_value=self.bus):
            node = FollowerNode(
                parameter_overrides=self.parameters
                + [
                    Parameter("allow_writes", value=allow_writes),
                    Parameter("release_torque_on_shutdown", value=release_on_shutdown),
                ]
            )
        if cleanup:
            self.addCleanup(node.destroy_node)
        node.joints = Mock()
        node.diagnostics = Mock()
        return node

    def command(self, node):
        msg = JointState()
        msg.name = list(reversed(JOINTS))
        msg.position = list(reversed(node.robot.to_radians(self.raw)))
        msg.header.stamp = node.get_clock().now().to_msg()
        return msg

    def test_missing_connection_or_calibration_fails_before_hardware_access(self):
        for overrides, message in (
            ([], "params_file"),
            ([Parameter("device_id", value="fake-device")], "calibration_profile"),
        ):
            with patch("so101_driver.follower_node.FeetechBus") as bus:
                with self.assertRaisesRegex(ValueError, message):
                    FollowerNode(parameter_overrides=overrides)
                bus.return_value.open.assert_not_called()

    def test_default_rejects_enable_and_commands_without_writes(self):
        node = self.node()
        response = node.enable_control(SetBool.Request(data=True), SetBool.Response())
        self.assertFalse(response.success)
        self.assertIn("Writes are disabled", response.message)
        node.receive_command(self.command(node))
        node.publish_state()
        self.assertEqual(node.joints.publish.call_count, 1)
        self.assertEqual(node.rejected_commands, 1)
        self.assertTrue(node.describe_parameter("allow_writes").read_only)
        response = node.release_torque(Trigger.Request(), Trigger.Response())
        self.assertFalse(response.success)
        self.bus.write_positions.assert_not_called()
        self.bus.set_torque_enabled.assert_not_called()
        self.bus.set_torque_limit.assert_not_called()

    def test_invalid_gripper_torque_limit_fails_before_opening_port(self):
        for value in (-0.1, 100.1, float("nan"), float("inf")):
            with patch("so101_driver.follower_node.FeetechBus", return_value=self.bus):
                with self.assertRaisesRegex(ValueError, "gripper_torque_limit_pct"):
                    FollowerNode(
                        parameter_overrides=self.parameters
                        + [Parameter("gripper_torque_limit_pct", value=value)]
                    )
                self.bus.open.assert_not_called()
                self.bus.set_torque_limit.assert_not_called()

    def test_publish_rate_must_keep_state_fresh_and_is_checked_before_opening(self):
        for rate in (0.0, 4.9, 100.1, float("nan")):
            with patch("so101_driver.follower_node.FeetechBus") as bus:
                with self.assertRaisesRegex(ValueError, r"publish_rate_hz must be in \[5, 100\]"):
                    FollowerNode(
                        parameter_overrides=self.parameters
                        + [Parameter("publish_rate_hz", value=rate)]
                    )
                bus.assert_not_called()
        self.parameters.append(Parameter("publish_rate_hz", value=5.0))
        self.assertEqual(self.node().timer.timer_period_ns, 200_000_000)

    def test_yaml_integers_and_integral_floats_are_normalized(self):
        self.parameters += [
            Parameter("publish_rate_hz", value=10),
            Parameter("motor_state_rate_hz", value=0),
            Parameter("command_timeout_s", value=1),
            Parameter("max_velocity_rad_s", value=2),
            Parameter("servo_acceleration_raw", value=20.0),
            Parameter("gripper_torque_limit_pct", value=30),
        ]
        node = self.node()
        self.assertEqual(node.timer.timer_period_ns, 100_000_000)
        self.assertIsNone(node.motor_telemetry)
        control = node.control
        self.assertEqual((control.timeout, control.gripper_torque_limit_raw), (1.0, 300))
        self.assertIs(type(control.acceleration), int)
        self.assertEqual(control.acceleration, 20)
        node.publish_health(DiagnosticStatus.OK, "ok")
        status = node.diagnostics.publish.call_args.args[0].status[0]
        # The GUI detects disabled monitoring by this exact text.
        self.assertIn(("motor_state_rate_hz", "0.0"), [(v.key, v.value) for v in status.values])

    def test_non_numeric_or_fractional_integer_parameters_fail_before_opening(self):
        for name, value, message in (
            ("publish_rate_hz", "fast", "must be a number"),
            ("gripper_torque_limit_pct", True, "must be a number"),
            ("servo_acceleration_raw", 20.5, "must be an integer"),
        ):
            with patch("so101_driver.follower_node.FeetechBus") as bus:
                with self.assertRaisesRegex(ValueError, f"{name} {message}"):
                    FollowerNode(
                        parameter_overrides=self.parameters + [Parameter(name, value=value)]
                    )
                bus.assert_not_called()

    def test_motor_monitor_can_be_disabled_and_rejects_invalid_rates_before_opening(self):
        for rate in (-1.0, 11.0, float("nan")):
            with patch("so101_driver.follower_node.FeetechBus") as bus:
                with self.assertRaisesRegex(ValueError, "motor_state_rate_hz"):
                    FollowerNode(
                        parameter_overrides=self.parameters
                        + [Parameter("motor_state_rate_hz", value=rate)]
                    )
                bus.assert_not_called()
        self.parameters.append(Parameter("motor_state_rate_hz", value=0.0))
        node = self.node()
        self.assertIsNone(node.motor_telemetry)
        node.publish_state()
        self.assertEqual(node.joints.publish.call_count, 1)
        self.bus.read_motor_status.assert_not_called()
        self.bus.read_motor_info.assert_not_called()

    def test_named_command_reorders_and_rejects_unsupported_fields(self):
        node = self.node(allow_writes=True)
        self.assertTrue(node.describe_parameter("use_sim_time").read_only)
        response = node.enable_control(SetBool.Request(data=True), SetBool.Response())
        self.assertTrue(response.success)
        msg = self.command(node)
        node.receive_command(msg)
        self.assertEqual(node.control.target, node.robot.to_radians(self.raw))
        last_stamp = node.control.last_stamp
        for mutate in (
            lambda m: setattr(m, "name", [JOINTS[0]] * 6),
            lambda m: setattr(m, "velocity", [0.0] * 6),
            lambda m: setattr(m, "position", [float("nan")] * 6),
        ):
            bad = self.command(node)
            mutate(bad)
            node.receive_command(bad)
        self.assertEqual(node.rejected_commands, 3)
        self.assertEqual(node.control.last_stamp, last_stamp)
        node.enable_control(SetBool.Request(data=False), SetBool.Response())
        self.assertFalse(node.control.active)

    def test_read_failure_disarms_and_read_recovery_does_not_resume_control(self):
        node = self.node(allow_writes=True)
        node.enable_control(SetBool.Request(data=True), SetBool.Response())
        self.bus.read_positions.side_effect = BusError("disconnected")
        node.publish_state()
        self.assertFalse(node.control.active)
        self.assertTrue(node.control.hold_failed)
        node.joints.publish.assert_not_called()
        self.bus.read_positions.side_effect = lambda ids: SimpleNamespace(
            positions_ticks=self.raw.copy()
        )
        node.publish_state()
        self.assertEqual(node.joints.publish.call_count, 1)
        self.assertFalse(node.control.active)
        status = node.diagnostics.publish.call_args.args[0].status[0]
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        self.assertIn("NOT confirmed", status.message)
        response = node.enable_control(SetBool.Request(data=False), SetBool.Response())
        self.assertTrue(response.success)  # Explicit hold retry, without resuming commands.
        self.assertFalse(node.control.hold_failed)
        self.assertFalse(node.control.active)

    def test_shutdown_releases_all_axes_before_closing_even_after_ros_shutdown(self):
        node = self.node(allow_writes=True, release_on_shutdown=True, cleanup=False)
        try:
            response = node.enable_control(SetBool.Request(data=True), SetBool.Response())
            self.assertTrue(response.success)
            self.assertTrue(node.describe_parameter("release_torque_on_shutdown").read_only)
            self.bus.reset_mock()
            rclpy.shutdown()
        finally:
            node.destroy_node()
        self.assertEqual(
            self.bus.mock_calls,
            [call.set_torque_enabled([i], False) for i in range(1, 7)]
            + [call.read_configuration(range(1, 7)), call.close()],
        )
        self.assertFalse(node.control.active)
        self.assertFalse(node.control.engaged)

    def test_shutdown_releases_torque_even_when_control_is_stopped(self):
        node = self.node(allow_writes=True, release_on_shutdown=True, cleanup=False)
        try:
            node.enable_control(SetBool.Request(data=True), SetBool.Response())
            node.enable_control(SetBool.Request(data=False), SetBool.Response())
            self.assertFalse(node.control.active)
            self.assertTrue(node.control.engaged)
            self.bus.reset_mock()
        finally:
            node.destroy_node()
        self.assertEqual(
            self.bus.set_torque_enabled.call_args_list,
            [call([i], False) for i in range(1, 7)],
        )
        self.bus.write_positions.assert_not_called()

    def test_shutdown_release_failure_attempts_all_axes_reports_error_and_closes(self):
        node = self.node(allow_writes=True, release_on_shutdown=True, cleanup=False)
        self.bus.set_torque_enabled.side_effect = BusError("disconnected")
        self.bus.read_configuration.side_effect = BusError("disconnected")
        with patch("builtins.print") as output:
            node.destroy_node()
        self.assertEqual(
            self.bus.set_torque_enabled.call_args_list,
            [call([i], False) for i in range(1, 7)],
        )
        self.assertTrue(node.control.release_failed)
        output.assert_called_once()
        self.assertIn("Torque release at shutdown not confirmed", output.call_args.args[0])
        self.assertEqual(output.call_args.kwargs, dict(file=sys.stderr, flush=True))
        self.bus.close.assert_called_once()

    def test_read_only_shutdown_never_writes_even_with_release_requested(self):
        node = self.node(release_on_shutdown=True, cleanup=False)
        node.destroy_node()
        self.bus.set_torque_enabled.assert_not_called()
        self.bus.write_positions.assert_not_called()
        self.bus.close.assert_called_once()

    def test_default_shutdown_keeps_the_existing_hold_behavior(self):
        node = self.node(allow_writes=True, cleanup=False)
        try:
            node.enable_control(SetBool.Request(data=True), SetBool.Response())
            self.bus.reset_mock()
        finally:
            node.destroy_node()
        self.bus.write_positions.assert_called_once()
        self.bus.set_torque_enabled.assert_not_called()
        self.bus.close.assert_called_once()
