"""Start the follower interface and six-joint slider GUI."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription
from so101_driver.paths import default_config_path


def generate_launch_description():
    share = Path(get_package_share_directory("so101_driver"))
    gui = Node(
        package="so101_driver",
        executable="slider_gui",
        output="screen",
        parameters=[{"follower_config": LaunchConfiguration("params_file")}],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_config_path("follower", share)),
                description="Follower settings YAML (device ID, calibration, motion)",
            ),
            DeclareLaunchArgument(
                "allow_writes",
                default_value="true",
                description="Allow starting control from the GUI",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / "launch/follower_control.launch.py")),
                launch_arguments={
                    "params_file": LaunchConfiguration("params_file"),
                    "allow_writes": LaunchConfiguration("allow_writes"),
                }.items(),
            ),
            gui,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gui,
                    on_exit=[EmitEvent(event=Shutdown(reason="Slider GUI closed"))],
                )
            ),
        ]
    )
