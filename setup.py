from pathlib import Path

from setuptools import find_packages, setup

sdk_directory = Path("vendor/FTServo_Python")
if not (Path(__file__).resolve().parent / sdk_directory / "scservo_sdk/__init__.py").is_file():
    raise RuntimeError(
        "The official Feetech SDK submodule is missing. "
        "Run 'git submodule update --init --recursive' in the so101_driver repository."
    )

setup(
    name="so101_driver",
    version="0.1.0",
    # The pip feetech-servo-sdk also installs a top-level scservo_sdk, so keep this one private.
    packages=find_packages(include=["so101_driver", "so101_driver.*"])
    + ["so101_driver._vendor.scservo_sdk"],
    package_dir={"so101_driver._vendor.scservo_sdk": str(sdk_directory / "scservo_sdk")},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/so101_driver"]),
        ("share/so101_driver", ["package.xml", "README.md"]),
        ("share/so101_driver/licenses/FTServo_Python", [str(sdk_directory / "LICENSE")]),
        ("share/so101_driver/docs", sorted(str(p) for p in Path("docs").glob("*.md"))),
        (
            "share/so101_driver/config",
            [
                "config/leader.template.yaml",
                "config/follower.template.yaml",
                "config/teleop.template.yaml",
                "config/cameras.template.yaml",
                "config/calibration.rviz",
                "config/confirmation.qss",
            ],
        ),
        (
            "share/so101_driver/launch",
            [
                "launch/follower_control.launch.py",
                "launch/follower_slider_control.launch.py",
                "launch/teleop.launch.py",
                "launch/cameras.launch.py",
            ],
        ),
    ]
    + [
        (
            str(Path("share/so101_driver") / directory),
            [str(p) for p in Path(directory).iterdir() if p.is_file()],
        )
        for directory in ("description", "description/so101", "description/so101/assets")
    ],
    install_requires=["setuptools"],
    description="ROS 2 driver for the SO-ARM101: calibration, control, GUI and teleoperation",
    license="MIT",
    maintainer="nao",
    maintainer_email="60519909+hachimada@users.noreply.github.com",
    entry_points={
        "console_scripts": [
            "leader_node = so101_driver.leader_node:main",
            "follower_node = so101_driver.follower_node:main",
            "so101_calibrate = so101_driver.calibration_cli:main",
            "so101_list_devices = so101_driver.hardware.devices:main",
            "so101_list_cameras = so101_driver.cameras:main",
            "slider_gui = so101_driver.slider_gui:main",
            "teleop_node = so101_driver.teleop_node:main",
        ]
    },
)
