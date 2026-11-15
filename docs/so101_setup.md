# Setup

**English** | [日本語](ja/so101_setup.md)

This guide prepares each arm and its settings files after installation.
To install the package, see [Installation in the README](../README.md#installation).

1. [Assign motor IDs and the baud rate](#assign-motor-ids-and-the-baud-rate) (once, when assembling the arm)
2. [Find the device ID](#find-the-device-id) of each arm
3. [Edit the settings files](#settings-files)

## Assign motor IDs and the baud rate

This driver never writes the servo settings (EEPROM).
Set the motor ID and the baud rate (1 Mbps) of each servo when you assemble the arm.
The motor IDs map to the joints as follows, the same assignment as the LeRobot SO-101:

| Motor ID | Joint |
|---:|---|
| 1 | `shoulder_pan` |
| 2 | `shoulder_lift` |
| 3 | `elbow_flex` |
| 4 | `wrist_flex` |
| 5 | `wrist_roll` |
| 6 | `gripper` |

You can set them with LeRobot's `lerobot-setup-motors` (tested with LeRobot 0.6.1).
Replace `--robot.port` and `--teleop.port` with the ports you are using.

```bash
lerobot-setup-motors --robot.type=so101_follower --robot.port=/dev/ttyACM0
lerobot-setup-motors --teleop.type=so101_leader --teleop.port=/dev/ttyACM1
```

Follow the prompts: connect one servo at a time and press Enter, starting from the gripper (ID 6).

## Find the device ID

Each arm is selected by its device ID, the serial number of its USB serial adapter.
Unlike the port name (`/dev/ttyACM0`), it does not change when you replug the USB cable, so the settings stay valid.
It is a different identifier from the servo motor IDs (1–6).

Connect the arm over USB and run:

```bash
ros2 run so101_driver so101_list_devices
```

The tool first shows the settings directory and the connected devices.
The `CURRENT CONFIG` column shows the role (`leader` or `follower`) whose `device_id` already holds that ID.

1. When prompted, unplug the USB cable of one arm and press Enter.
2. Note the `DEVICE ID` and the guidance shown, then reconnect the cable.
3. Repeat for the leader and the follower.

If the ID is not in any settings file yet, the tool creates the missing [settings files](#settings-files) (`leader.yaml`, `follower.yaml`, `teleop.yaml`) from the templates and shows where to write the ID.
Open the file of the arm you unplugged and write the ID in `device_id`.
Existing files are never overwritten.

If no device or more than one device disappears, unplug only the target cable and press Enter again.
A device without a USB serial number, or one whose serial number is shared with another connected device, cannot be selected by ID.
Press `q` or Ctrl+C to quit.

## Settings files

Settings and calibration results for each arm are stored in `~/.ros/so101_driver`.
If the `ROS_HOME` environment variable is set, they are stored in `$ROS_HOME/so101_driver` instead; read `~/.ros` below accordingly.
Because the files live outside the repository, the same files are used regardless of where you cloned the repository or which directory you run from.

The paths below are relative to `~/.ros/so101_driver`.

| File | Contents |
|---|---|
| `config/leader.yaml` | Leader device ID and calibration file |
| `config/follower.yaml` | Follower device ID, calibration file and motion settings |
| `config/teleop.yaml` | Teleoperation alignment and joint scales |
| `config/cameras.yaml` | USB camera list, created by `so101_list_cameras` when you use cameras. See [Cameras](so101_cameras.md) |
| `calibration/leader.json`, `calibration/follower.json` | Calibration results, created by [calibration](so101_calibration.md) |

Set `device_id` in `leader.yaml` and `follower.yaml` to the IDs you found.
Quote the ID even if it contains only digits.

```yaml
device_id: "your-device-serial"
```

`calibration_profile` already points to `calibration/leader.json` and `calibration/follower.json` respectively.

`so101_list_devices` creates `config/leader.yaml`, `config/follower.yaml` and `config/teleop.yaml`.
To create them by hand, copy the installed templates:

```bash
mkdir -p ~/.ros/so101_driver/config
share="$(ros2 pkg prefix --share so101_driver)"
cp -n "$share/config/leader.template.yaml" ~/.ros/so101_driver/config/leader.yaml
cp -n "$share/config/follower.template.yaml" ~/.ros/so101_driver/config/follower.yaml
cp -n "$share/config/teleop.template.yaml" ~/.ros/so101_driver/config/teleop.yaml
```

### Main settings

| Setting | Purpose | Details |
|---|---|---|
| `device_id` | USB serial number of the arm to connect | [Find the device ID](#find-the-device-id) |
| `calibration_profile` | Calibration JSON to use | [Calibration](so101_calibration.md#output-files) |
| `publish_rate_hz` | State publish rate (leader and follower) | [Motion settings](so101_follower.md#motion-settings), [Leader node](so101_teleop.md#leader-node-leaderso101_leader) |
| `command_timeout_s` | How long a follower command stays valid | [Motion settings](so101_follower.md#motion-settings) |
| `max_velocity_rad_s` / `servo_acceleration_raw` | Follower speed and acceleration | [Motion settings](so101_follower.md#motion-settings) |
| `gripper_torque_limit_pct` | Gripper output limit | [Motion settings](so101_follower.md#motion-settings) |
| `motor_state_rate_hz` | Motor status update rate | [Motor status](so101_motor_monitor.md#update-rate) |
| `alignment_*` / `joint_scales` | Teleoperation alignment and joint scales | [Teleoperation settings](so101_teleop.md#settings) |

### Rules for values

- Changes take effect when the node restarts. No rebuild is needed.
- The valid values of each setting are listed on its details page. With an out-of-range value, the node logs an error at startup and exits.
- Numbers can be written as `30` or `30.0`. In lists such as `joint_scales`, however, write every value with a decimal point.
- A relative `calibration_profile` path is resolved from `~/.ros/so101_driver`. Absolute paths and `~` also work.
- Environment variables such as `$HOME` or `$PWD` in the YAML are not expanded.

## Choose settings files at launch

| Feature | Launch file | Settings file arguments |
|---|---|---|
| ROS control interface | `follower_control.launch.py` | `params_file` |
| Slider GUI | `follower_slider_control.launch.py` | `params_file` |
| Teleoperation | `teleop.launch.py` | `leader_params_file`, `follower_params_file`, `teleop_params_file` |

When an argument is omitted, the matching YAML in `~/.ros/so101_driver/config/` is used.
If that file does not exist, the installed template is used instead, so set the device IDs as described above before using the arms.
The settings files in use are printed in the terminal at startup.

To use another settings file, pass its absolute path:

```bash
ros2 launch so101_driver follower_slider_control.launch.py params_file:=/path/to/follower.yaml
```

Keep the top-level node keys of the templates (`/leader/so101_leader:`, `/follower/so101_follower:`, `/so101_teleop:`).
The slider GUI and teleoperation also read the follower calibration from the `/follower/so101_follower:` key.

For the launch commands, see [Run in the README](../README.md#3-run).
