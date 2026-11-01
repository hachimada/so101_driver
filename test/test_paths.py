import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from so101_driver.paths import calibration_profile_path, data_directory, default_config_path


class PathTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)

    def test_data_directory_follows_ros_home_and_defaults_to_dot_ros(self):
        with patch.dict("os.environ", {"ROS_HOME": str(self.root)}):
            self.assertEqual(data_directory(), self.root / "so101_driver")
        with patch.dict("os.environ", {"ROS_HOME": "~/custom"}):
            self.assertEqual(data_directory(), Path.home() / "custom/so101_driver")
        for environment in ({}, {"ROS_HOME": ""}):
            with patch.dict("os.environ", environment, clear=True):
                self.assertEqual(data_directory(), Path.home() / ".ros/so101_driver")

    def test_relative_profile_does_not_depend_on_install_or_working_directory(self):
        with (
            patch.dict("os.environ", {"ROS_HOME": str(self.root / "ros")}),
            patch("so101_driver.paths.__file__", str(self.root / "installed/paths.py")),
            patch("so101_driver.paths.Path.cwd", return_value=self.root / "elsewhere"),
        ):
            self.assertEqual(
                calibration_profile_path("calibration/follower.json"),
                self.root / "ros/so101_driver/calibration/follower.json",
            )

    def test_explicit_absolute_profile_is_used_as_is(self):
        profile = self.root / "custom.json"
        with patch("so101_driver.paths.data_directory", side_effect=AssertionError):
            self.assertEqual(calibration_profile_path(profile), profile)

    def test_machine_settings_take_precedence_over_installed_template(self):
        data = self.root / "ros/so101_driver"
        config = data / "config/follower.yaml"
        config.parent.mkdir(parents=True)
        config.write_text("device_id: private-device")
        share = self.root / "installed/share/so101_driver"
        with patch("so101_driver.paths.data_directory", return_value=data):
            self.assertEqual(default_config_path("follower", share), config)
            self.assertEqual(
                default_config_path("leader", share), share / "config/leader.template.yaml"
            )
