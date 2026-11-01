"""ROS callback fault/recovery tests; no hardware access."""

import unittest
from unittest.mock import Mock, patch

try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticStatus
    from rclpy.parameter import Parameter

    from so101_driver.hardware.feetech_bus import BusError
    from so101_driver.leader_node import LeaderNode

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "Requires ROS2 Humble")
class LeaderNodeTests(unittest.TestCase):
    def setUp(self):
        resolver = patch("so101_driver.leader_node.resolve_device_id", return_value="/fake")
        self.resolve = resolver.start()
        self.addCleanup(resolver.stop)

    def test_missing_connection_or_calibration_fails_before_hardware_access(self):
        rclpy.init()
        try:
            for overrides, message in (
                ([], "params_file"),
                ([Parameter("device_id", value="fake-device")], "calibration_profile"),
            ):
                with patch("so101_driver.leader_node.FeetechBus") as bus:
                    with self.assertRaisesRegex(ValueError, message):
                        LeaderNode(parameter_overrides=overrides)
                    bus.return_value.open.assert_not_called()
        finally:
            rclpy.shutdown()

    def test_publish_rate_must_keep_state_fresh_and_is_checked_before_opening(self):
        rclpy.init()
        try:
            for rate in (0.0, 4.9, 100.1, float("nan")):
                with patch("so101_driver.leader_node.FeetechBus") as bus:
                    with self.assertRaisesRegex(
                        ValueError, r"publish_rate_hz must be in \[5, 100\]"
                    ):
                        LeaderNode(
                            parameter_overrides=[
                                Parameter("device_id", value="fake-device"),
                                Parameter("calibration_profile", value="/fake/leader.json"),
                                Parameter("publish_rate_hz", value=rate),
                            ]
                        )
                    bus.assert_not_called()
        finally:
            rclpy.shutdown()

    def test_integer_publish_rate_from_yaml_is_accepted(self):
        rclpy.init()
        node = None
        try:
            with (
                patch("so101_driver.leader_node.FeetechBus"),
                patch("so101_driver.leader_node.SO101Leader"),
            ):
                node = LeaderNode(
                    parameter_overrides=[
                        Parameter("device_id", value="fake-device"),
                        Parameter("calibration_profile", value="/fake/leader.json"),
                        Parameter("publish_rate_hz", value=10),
                    ]
                )
            self.assertEqual(node.timer.timer_period_ns, 100_000_000)
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()

    def test_fault_drops_sample_and_next_success_recovers(self):
        rclpy.init()
        node = None
        try:
            with (
                patch("so101_driver.leader_node.FeetechBus"),
                patch("so101_driver.leader_node.SO101Leader"),
            ):
                node = LeaderNode(
                    parameter_overrides=[
                        Parameter("device_id", value="fake-device"),
                        Parameter("calibration_profile", value="/fake/leader.json"),
                    ]
                )
            node.joints = Mock()
            node.diagnostics = Mock()
            node.bus.port_name = "/fake"
            node.leader.read_joint_positions.side_effect = [
                BusError("ReadPos ID=6 servo_error=0x01"),
                [0.1] * 5 + [(-0.174533 + 1.74533) / 2],
                OSError("disconnected"),
            ]
            node.publish_state()
            node.joints.publish.assert_not_called()
            error = node.diagnostics.publish.call_args.args[0].status[0]
            self.assertEqual(error.level, DiagnosticStatus.ERROR)
            self.assertIn("ID=6", error.message)
            node.publish_state()
            self.assertEqual(node.joints.publish.call_count, 1)
            msg = node.joints.publish.call_args.args[0]
            self.assertEqual(msg.name[-1], "gripper")
            self.assertEqual(len(msg.position), 6)
            self.assertAlmostEqual(msg.position[-1], (-0.174533 + 1.74533) / 2)
            self.assertEqual(node.consecutive_failures, 0)
            self.assertEqual(node.failed_samples, 1)
            self.assertEqual(
                node.diagnostics.publish.call_args.args[0].status[0].level, DiagnosticStatus.OK
            )
            node.publish_state()
            self.assertEqual(node.joints.publish.call_count, 1)
            self.assertEqual(node.failed_samples, 2)
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()
