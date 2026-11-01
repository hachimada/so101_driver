"""Provide the follower ROS control interface."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from so101_driver.paths import default_config_path


def generate_launch_description():
    share = Path(get_package_share_directory("so101_driver"))
    follower = Node(
        package="so101_driver",
        executable="follower_node",
        namespace="follower",
        name="so101_follower",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "allow_writes": ParameterValue(
                    LaunchConfiguration("allow_writes"), value_type=bool
                ),
                "release_torque_on_shutdown": ParameterValue(
                    LaunchConfiguration("release_torque_on_shutdown"), value_type=bool
                ),
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_config_path("follower", share)),
                description="Follower settings YAML (device ID, calibration, motion)",
            ),
            DeclareLaunchArgument(
                "allow_writes", default_value="false", description="Allow enabling control"
            ),
            DeclareLaunchArgument(
                "release_torque_on_shutdown",
                default_value="false",
                description="Release torque on all 6 joints at shutdown",
            ),
            LogInfo(msg=["Follower settings: ", LaunchConfiguration("params_file")]),
            follower,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=follower,
                    on_exit=[EmitEvent(event=Shutdown(reason="Follower stopped"))],
                )
            ),
        ]
    )
