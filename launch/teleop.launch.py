"""Align the follower to the leader before starting absolute teleoperation.

With cameras:=true, the USB cameras in cameras.yaml are started as well.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from so101_driver.paths import config_path, default_config_path


def generate_launch_description():
    share = Path(get_package_share_directory("so101_driver"))
    leader = Node(
        package="so101_driver",
        executable="leader_node",
        namespace="leader",
        name="so101_leader",
        parameters=[LaunchConfiguration("leader_params_file")],
        output="screen",
    )
    teleop = Node(
        package="so101_driver",
        executable="teleop_node",
        name="so101_teleop",
        output="screen",
        parameters=[
            LaunchConfiguration("teleop_params_file"),
            {
                "follower_config": LaunchConfiguration("follower_params_file"),
                "autostart": ParameterValue(LaunchConfiguration("autostart"), value_type=bool),
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "leader_params_file",
                default_value=str(default_config_path("leader", share)),
                description="Leader settings YAML (device ID, calibration)",
            ),
            DeclareLaunchArgument(
                "follower_params_file",
                default_value=str(default_config_path("follower", share)),
                description="Follower settings YAML (device ID, calibration, motion)",
            ),
            DeclareLaunchArgument(
                "teleop_params_file",
                default_value=str(default_config_path("teleop", share)),
                description="Teleoperation settings YAML (alignment, joint scales)",
            ),
            DeclareLaunchArgument(
                "allow_writes", default_value="true", description="Allow follower control"
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Align and start teleoperation once both arms report state",
            ),
            DeclareLaunchArgument(
                "cameras",
                default_value="false",
                description="Also start the USB cameras listed in cameras_file",
            ),
            DeclareLaunchArgument(
                "cameras_file",
                default_value=str(config_path("cameras")),
                description="USB camera list (name, device, size, fps)",
            ),
            # Before the arms, so a camera setting error starts nothing.
            # A camera that stops later does not stop teleoperation.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / "launch/cameras.launch.py")),
                launch_arguments={"cameras_file": LaunchConfiguration("cameras_file")}.items(),
                condition=IfCondition(LaunchConfiguration("cameras")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / "launch/follower_control.launch.py")),
                launch_arguments={
                    "params_file": LaunchConfiguration("follower_params_file"),
                    "allow_writes": LaunchConfiguration("allow_writes"),
                    "release_torque_on_shutdown": "true",
                }.items(),
            ),
            LogInfo(msg=["Leader settings: ", LaunchConfiguration("leader_params_file")]),
            LogInfo(msg=["Teleop settings: ", LaunchConfiguration("teleop_params_file")]),
            leader,
            teleop,
            *[
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=process,
                        on_exit=[EmitEvent(event=Shutdown(reason="Teleoperation stopped"))],
                    )
                )
                for process in (leader, teleop)
            ],
        ]
    )
