"""Start the USB cameras listed in cameras.yaml with usb_cam, under /cameras/<name>."""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription
from so101_driver.cameras import load_cameras
from so101_driver.paths import config_path


def camera_nodes(context):
    path = Path(LaunchConfiguration("cameras_file").perform(context)).expanduser()
    try:
        get_package_share_directory("usb_cam")
    except PackageNotFoundError:
        raise RuntimeError(
            "usb_cam is not installed: sudo apt install ros-humble-usb-cam"
        ) from None
    if not path.is_file():
        if path == config_path("cameras"):
            raise RuntimeError(
                f"{path} not found. Create it and find the camera paths with: "
                "ros2 run so101_driver so101_list_cameras"
            )
        share = Path(get_package_share_directory("so101_driver"))
        raise RuntimeError(
            f"{path} not found. Create it with: cp {share}/config/cameras.template.yaml {path}"
        )
    cameras = load_cameras(path)
    if not cameras:
        raise RuntimeError(f"No cameras are listed in {path}")
    return [LogInfo(msg=f"Camera settings: {path}")] + [
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            namespace=f"cameras/{name}",
            name="usb_cam",
            output="screen",
            parameters=[parameters],
        )
        for name, parameters in cameras.items()
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cameras_file",
                default_value=str(config_path("cameras")),
                description="USB camera list (name, device, size, fps)",
            ),
            OpaqueFunction(function=camera_nodes),
        ]
    )
