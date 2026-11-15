# so101_driver

[![CI](https://github.com/hachimada/so101_driver/actions/workflows/ci.yml/badge.svg)](https://github.com/hachimada/so101_driver/actions/workflows/ci.yml)

**English** | [日本語](README.ja.md)

A ROS 2 package for calibrating and controlling the SO-ARM101 (SO-101) robot arm through the official Feetech SDK.
Teleoperation can also stream USB camera images on ROS topics.

## Features

| Feature | Description |
|---|---|
| Calibration | A terminal guide and an RViz reference model record the reference pose and joint ranges of the leader and follower arms |
| Follower control interface | Six-joint position commands on a ROS topic, and services to start and stop control |
| Slider GUI | Move each joint and check motor state such as current, voltage and temperature |
| Teleoperation | The follower aligns to the leader pose, then follows the leader |
| Cameras (optional) | Publish USB camera images alongside teleoperation, using [usb_cam](https://index.ros.org/p/usb_cam/) |

## Safety

> [!WARNING]
> - Launching with control allowed moves the real arm. Before starting, make sure no people or objects are within reach of the arm.
> - Releasing torque lets the arm fall under its own weight. Support the arm before releasing torque or shutting down.
> - Turning off the servo power also releases torque. If the software cannot stop the arm, support the arm before turning off the power.

## Requirements

- Ubuntu 22.04, ROS 2 Humble, Python 3.10
- A ROS 2 development environment with Git, colcon and rosdep
- An SO-ARM101 (six STS3215 servos with motor IDs 1–6 at 1 Mbps) connected over USB serial.
  The driver never changes the motor IDs or the baud rate, so set them beforehand.
  See [Assign motor IDs and the baud rate](docs/so101_setup.md#assign-motor-ids-and-the-baud-rate).
- For cameras: UVC USB cameras and `ros-humble-usb-cam` (not a dependency of so101_driver)

## Installation

Run in the root of your ROS 2 workspace:

```bash
git clone --recurse-submodules https://github.com/hachimada/so101_driver.git src/so101_driver
rosdep install --from-paths src/so101_driver --ignore-src --rosdistro humble -r -y
colcon build --packages-select so101_driver
source install/setup.bash
```

The commands below work from any directory in a terminal that has sourced `install/setup.bash`.

## Quick start

### 1. Set the device IDs

Run `so101_list_devices` and unplug each arm when prompted to find its device ID (USB serial number).
If the ID is not in any settings file yet, the tool creates the missing files in `~/.ros/so101_driver/config/` from the templates.
Write the ID in `device_id` of the file it shows.
See [Setup](docs/so101_setup.md) for details.

```bash
ros2 run so101_driver so101_list_devices
```

### 2. Calibrate

```bash
ros2 run so101_driver so101_calibrate --role leader
ros2 run so101_driver so101_calibrate --role follower
```

Match the reference model's pose, close the gripper and press Enter to start.
Move every joint, then press Enter to finish and save.
See [Calibration](docs/so101_calibration.md).

### 3. Run

All of the following move the real arm. Before the first run, read the start and stop steps in each guide.

Slider GUI ([guide](docs/so101_gui.md)):

```bash
ros2 launch so101_driver follower_slider_control.launch.py
```

To quit, support the arm and press **Release torque and quit**.

Teleoperation ([guide](docs/so101_teleop.md)):

```bash
ros2 launch so101_driver teleop.launch.py
```

The follower moves to the leader pose, then follows the leader.
To quit, support the follower arm and press Ctrl+C. Torque is released on all six joints before the driver exits.

To also stream USB camera images, add `cameras:=true`.
First find the camera paths with `so101_list_cameras` and write them in `cameras.yaml` ([Cameras](docs/so101_cameras.md)).

```bash
ros2 launch so101_driver teleop.launch.py cameras:=true
```

Control from your own program through ROS topics and services ([guide](docs/so101_follower.md)):

```bash
ros2 launch so101_driver follower_control.launch.py allow_writes:=true
```

## Usage

### Command-line tools

| Command | Description |
|---|---|
| `ros2 run so101_driver so101_list_devices` | Find an arm's device ID by unplugging it, and create missing settings files. See [Setup](docs/so101_setup.md#find-the-device-id) |
| `ros2 run so101_driver so101_calibrate` | Calibrate the leader or follower. See [Calibration](docs/so101_calibration.md) |
| `ros2 run so101_driver so101_list_cameras` | Find a camera's USB port path by unplugging it, and create `cameras.yaml`. See [Cameras](docs/so101_cameras.md#find-the-camera-path) |

### Launch files

| Launch file | Starts | Arguments (default) |
|---|---|---|
| `follower_control.launch.py` | Follower node | `params_file`, `allow_writes` (`false`), `release_torque_on_shutdown` (`false`) |
| `follower_slider_control.launch.py` | Follower node and slider GUI | `params_file`, `allow_writes` (`true`) |
| `teleop.launch.py` | Leader, follower and teleoperation nodes, and the cameras with `cameras:=true` | `leader_params_file`, `follower_params_file`, `teleop_params_file`, `allow_writes` (`true`), `autostart` (`true`), `cameras` (`false`), `cameras_file` |
| `cameras.launch.py` | One usb_cam node per camera | `cameras_file` |

- The `*params_file` arguments default to the matching YAML in `~/.ros/so101_driver/config/`, or to the installed template when that file does not exist. See [Choose settings files at launch](docs/so101_setup.md#choose-settings-files-at-launch).
- `cameras_file` defaults to `~/.ros/so101_driver/config/cameras.yaml`.
- `teleop.launch.py` always releases the follower torque on shutdown (with the default `allow_writes:=true`).
- If one of the arm nodes or the slider GUI exits, the whole launch shuts down. A camera node that stops does not.

### Nodes

| Node | Executable | Started by |
|---|---|---|
| `/follower/so101_follower` | `follower_node` | `follower_control.launch.py`, `follower_slider_control.launch.py`, `teleop.launch.py` |
| `/leader/so101_leader` | `leader_node` | `teleop.launch.py` |
| `/so101_teleop` | `teleop_node` | `teleop.launch.py` |
| `/so101_slider_control` | `slider_gui` | `follower_slider_control.launch.py` |
| `/cameras/<name>/usb_cam` | `usb_cam_node_exe` (usb_cam) | `cameras.launch.py`, `teleop.launch.py cameras:=true` |

## ROS interfaces

Topics and services provided by the nodes of this package. See the linked pages for QoS and field details.

| Name | Type | Node side | Description | Details |
|---|---|---|---|---|
| `/follower/joint_command` | sensor_msgs/msg/JointState | Subscribe | Position targets for the six follower joints (rad) | [Follower](docs/so101_follower.md#ros-interface) |
| `/follower/joint_states` | sensor_msgs/msg/JointState | Publish (BestEffort) | Measured angles of the six follower joints (rad) | [Follower](docs/so101_follower.md#ros-interface) |
| `/follower/diagnostics` | diagnostic_msgs/msg/DiagnosticArray | Publish | Follower control state and errors | [Follower](docs/so101_follower.md#diagnostics) |
| `/follower/motor_states` | diagnostic_msgs/msg/DiagnosticArray | Publish | Current, voltage, temperature and more for each motor | [Motor status](docs/so101_motor_monitor.md#ros-topic) |
| `/follower/enable_control` | std_srvs/srv/SetBool | Service | `true` starts control; `false` holds and stops accepting commands | [Follower](docs/so101_follower.md#ros-interface) |
| `/follower/release_torque` | std_srvs/srv/Trigger | Service | Release torque on all follower joints | [Follower](docs/so101_follower.md#ros-interface) |
| `/leader/joint_states` | sensor_msgs/msg/JointState | Publish (BestEffort) | Angles of the six leader joints (rad) | [Teleoperation](docs/so101_teleop.md#ros-interface) |
| `/leader/diagnostics` | diagnostic_msgs/msg/DiagnosticArray | Publish | Leader read state | [Teleoperation](docs/so101_teleop.md#ros-interface) |
| `/so101_teleop/enable` | std_srvs/srv/SetBool | Service | Start or stop teleoperation | [Teleoperation](docs/so101_teleop.md#ros-interface) |
| `/cameras/<name>/image_raw` | sensor_msgs/msg/Image | Publish | Camera image | [Cameras](docs/so101_cameras.md#topics) |
| `/cameras/<name>/image_raw/compressed` | sensor_msgs/msg/CompressedImage | Publish | JPEG-compressed camera image | [Cameras](docs/so101_cameras.md#topics) |
| `/cameras/<name>/camera_info` | sensor_msgs/msg/CameraInfo | Publish | Camera resolution and calibration | [Cameras](docs/so101_cameras.md#topics) |

The available topics and services depend on the launch file:

| Topics and services | Launch file |
|---|---|
| `/follower/...` | `follower_control.launch.py`, `follower_slider_control.launch.py`, `teleop.launch.py` |
| `/leader/...`, `/so101_teleop/...` | `teleop.launch.py` |
| `/cameras/...` | `teleop.launch.py cameras:=true`, `cameras.launch.py` |

Subscribe to the BestEffort `joint_states` topics with BestEffort QoS.
Subscribe to the camera images with Reliable QoS.

## Configuration

Machine-specific settings and calibration results are stored outside the repository, in `~/.ros/so101_driver` (`$ROS_HOME/so101_driver` when `ROS_HOME` is set).

| File | Contents |
|---|---|
| `config/leader.yaml` | Leader device ID and calibration file |
| `config/follower.yaml` | Follower device ID, calibration file and motion settings |
| `config/teleop.yaml` | Teleoperation alignment and joint scales |
| `config/cameras.yaml` | USB camera list (only when using cameras) |
| `calibration/leader.json`, `calibration/follower.json` | Calibration results |

Changes take effect when the node restarts; no rebuild is needed.
See [Setup](docs/so101_setup.md#settings-files) for every setting.

## Documentation

| Guide | Contents |
|---|---|
| [Setup](docs/so101_setup.md) | Motor IDs, device IDs, settings files and where they are stored |
| [Calibration](docs/so101_calibration.md) | Recording the reference pose and joint ranges, and how the results are saved and used |
| [Follower control interface](docs/so101_follower.md) | ROS API, starting and stopping control, motion settings |
| [Slider GUI](docs/so101_gui.md) | Moving joints, saving and returning to a home pose |
| [Motor status](docs/so101_motor_monitor.md) | Displayed values, units and faults |
| [Teleoperation](docs/so101_teleop.md) | Alignment, following, starting and stopping |
| [Cameras](docs/so101_cameras.md) | USB camera setup, streaming with teleoperation, recording |
| [Feetech SDK](docs/feetech_sdk_source.md) | Source and installation of the bundled official SDK |

## Development

Lint and format with [Ruff](https://docs.astral.sh/ruff/) (settings in `ruff.toml`) in the repository directory:

```bash
ruff check .
ruff format .
```

Run the tests from the workspace root:

```bash
colcon test --packages-select so101_driver
colcon test-result --verbose
```

GitHub Actions ([`.github/workflows/ci.yml`](.github/workflows/ci.yml)) runs the same checks on every pull request and on every push to `main`:

- **Lint:** `ruff check` and `ruff format --check` with Ruff 0.17.5
- **Build and test:** `rosdep install`, `colcon build` and `colcon test` in the `ros:humble-ros-base-jammy` container

Use the same Ruff version locally, for example `uvx ruff@0.17.5 check .`. Older versions sort the `launch` imports differently.

## License

This package is released under the [Apache License 2.0](LICENSE).
The following components are under their own licenses:

| Component | License | Source |
|---|---|---|
| Official Feetech Python SDK (submodule) | MIT | [Feetech SDK](docs/feetech_sdk_source.md) |
| Robot model (URDF and meshes) | Apache-2.0 | [Robot model](description/README.md) |
