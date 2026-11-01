"""Expected startup errors end the node with one log line and exit code 1."""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

try:
    import rclpy

    from so101_driver import follower_node, leader_node, teleop_node

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "Requires ROS2 Humble")
class StartupErrorTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)
        logger = patch("so101_driver.startup.rclpy.logging.get_logger")
        self.get_logger = logger.start()
        self.addCleanup(logger.stop)

    def logged(self):
        self.get_logger.return_value.error.assert_called_once()
        return (
            self.get_logger.call_args.args[0],
            self.get_logger.return_value.error.call_args.args[0],
        )

    def test_missing_device_is_logged_as_one_line(self):
        for module, name in ((follower_node, "so101_follower"), (leader_node, "so101_leader")):
            with self.subTest(node=name):
                self.get_logger.reset_mock()
                code = module.main(
                    args=[
                        name,
                        "--ros-args",
                        "-p",
                        "device_id:=not-connected",
                        "-p",
                        "calibration_profile:=/fake/profile.json",
                    ]
                )
                self.assertEqual(code, 1)
                self.assertEqual(
                    self.logged(),
                    (
                        name,
                        "USB device ID 'not-connected' not found; check it with so101_list_devices",
                    ),
                )
                self.assertFalse(rclpy.ok())

    def test_wrong_parameter_type_is_logged_as_one_line(self):
        code = follower_node.main(args=["follower_node", "--ros-args", "-p", "device_id:=123"])
        self.assertEqual(code, 1)
        self.assertIn("expecting type 'STRING'", self.logged()[1])

    def test_missing_yaml_key_names_the_key(self):
        config = self.path / "follower.yaml"
        config.write_text("/**:\n  ros__parameters:\n    calibration_profile: follower.json\n")
        code = teleop_node.main(
            args=["teleop_node", "--ros-args", "-p", f"follower_config:={config}"]
        )
        self.assertEqual(code, 1)
        self.assertEqual(
            self.logged(),
            (
                "so101_teleop",
                "Missing key '/follower/so101_follower' in a settings or calibration file",
            ),
        )

    def test_unparsable_params_file_is_logged_as_one_line(self):
        # rcl rejects a list mixing integers and decimals before any node code runs.
        params = self.path / "teleop.yaml"
        params.write_text("/so101_teleop:\n  ros__parameters:\n    joint_scales: [-1, 1.0]\n")
        code = teleop_node.main(args=["teleop_node", "--ros-args", "--params-file", str(params)])
        self.assertEqual(code, 1)
        self.assertIn("Couldn't parse params file", self.logged()[1])
        self.assertFalse(rclpy.ok())

    def test_unexpected_errors_keep_the_traceback(self):
        with (
            patch("so101_driver.follower_node.FollowerNode", side_effect=TypeError("bug")),
            self.assertRaisesRegex(TypeError, "bug"),
        ):
            follower_node.main(args=["follower_node"])
        self.get_logger.return_value.error.assert_not_called()
        self.assertFalse(rclpy.ok())


if __name__ == "__main__":
    unittest.main()
