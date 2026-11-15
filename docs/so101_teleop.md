# Teleoperation

**English** | [日本語](ja/so101_teleop.md)

Teleoperation first moves the follower to the leader pose, then makes it follow the leader.

## Start

Complete [setup](so101_setup.md) and [calibration](so101_calibration.md) for both arms first.

```bash
ros2 launch so101_driver teleop.launch.py
```

1. Once the state of both arms is available, the follower automatically moves to the leader pose.
2. While the terminal shows `Aligning`, keep the leader still.
3. When it shows `Aligned; teleoperation running`, move the leader to operate the follower.

If you move the leader during alignment, the target follows the new pose.
Alignment compares the calibrated angles of both arms, so calibrate both with the same reference pose and joint directions.

To also stream USB camera images, add `cameras:=true`. See [Cameras](so101_cameras.md) for the setup.

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `leader_params_file` | `~/.ros/so101_driver/config/leader.yaml` | Leader settings YAML |
| `follower_params_file` | `~/.ros/so101_driver/config/follower.yaml` | Follower settings YAML |
| `teleop_params_file` | `~/.ros/so101_driver/config/teleop.yaml` | Teleoperation settings YAML |
| `allow_writes` | `true` | Allow follower control |
| `autostart` | `true` | Align and start once both arms report their state |
| `cameras` | `false` | Also start the USB cameras in `cameras_file` |
| `cameras_file` | `~/.ros/so101_driver/config/cameras.yaml` | USB camera list |

When a `*_params_file` YAML does not exist, the installed template is used instead.

## Start and stop manually

To disable the automatic start, add `autostart:=false` to the launch command.

Start:

```bash
ros2 service call /so101_teleop/enable std_srvs/srv/SetBool '{data: true}'
```

Stop:

```bash
ros2 service call /so101_teleop/enable std_srvs/srv/SetBool '{data: false}'
```

The start service aligns the follower and returns success after the poses match.
Every restart begins with alignment again.

The stop service makes the follower hold its current position.

The automatic start is attempted only once. If it fails, for example because the leader pose is outside the follower range, fix the cause and use the start service.

## Shut down

Support the follower arm and press Ctrl+C in the terminal that started teleoperation.
The driver turns off torque on all six joints, confirms it by readback and then closes the connection.
It is complete when `Torque released on all 6 joints` is shown.

If the release cannot be confirmed, for example because of a communication error, an error is shown at shutdown (`Torque release at shutdown not confirmed`).
Torque may still be on in that case, so check the connection.

## Settings

Edit `~/.ros/so101_driver/config/teleop.yaml` and restart teleoperation to apply the changes.

```yaml
/so101_teleop:
  ros__parameters:
    joint_scales: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    alignment_tolerance_rad: 0.05
    alignment_settle_time_s: 0.2
    alignment_timeout_s: 30.0
```

| Setting | Default | Meaning |
|---|---|---|
| `alignment_tolerance_rad` | 0.05 rad (about 2.9°) | Allowed difference between the target and the measured angle of each joint (greater than 0). Smaller values require a closer match |
| `alignment_settle_time_s` | 0.2 s | How long all six joints must stay within the tolerance (greater than 0 and less than `alignment_timeout_s`) |
| `alignment_timeout_s` | 30 s | Time limit for alignment (greater than 0). Increase it for slow motion settings |
| `joint_scales` | 1.0 for every joint | Factor applied to each leader joint angle (six finite values) |

With an out-of-range value, teleoperation logs an error at startup and exits.

The order of `joint_scales` is `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`.
-1.0 reverses the direction, and 0.0 fixes the target at 0 rad.
Write every value with a decimal point, such as `-1.0`. A list that mixes integers and decimals, such as `[-1, 1.0, ...]`, cannot be loaded and startup fails.
Both alignment and following use "target angle = joint scale × current leader angle".

The speed and acceleration are set in `~/.ros/so101_driver/config/follower.yaml`.
See [Motion settings](so101_follower.md#motion-settings).
The state publish rate is `publish_rate_hz` in `leader.yaml` and `follower.yaml` (default 30 Hz, 5–100 Hz).

## ROS interface

### Leader node (`/leader/so101_leader`)

Started together by the teleoperation launch command. It never writes to the motors.

| Name | Type | Node QoS | Description |
|---|---|---|---|
| `/leader/joint_states` | sensor_msgs/msg/JointState | Publish: **BestEffort**, KeepLast 5 | Angles of the six joints, rad. The gripper is limited to the closed–open range |
| `/leader/diagnostics` | diagnostic_msgs/msg/DiagnosticArray | Publish: Reliable, KeepLast 10 | Read state, and the failure counts `failed_samples` and `consecutive_failures` |

Subscribe to `/leader/joint_states` with BestEffort QoS.
A failed read publishes no joint state; the diagnostics report it as an error.

The leader settings are in `~/.ros/so101_driver/config/leader.yaml`. Restart the node to apply changes.

| Setting | Default | Description |
|---|---|---|
| `device_id` | None | USB serial number. See [Find the device ID](so101_setup.md#find-the-device-id) |
| `calibration_profile` | `calibration/leader.json` | Calibration result. Relative paths start at `~/.ros/so101_driver` |
| `publish_rate_hz` | 30.0 | Rate of angle reads and publishing (5–100 Hz) |

### Teleoperation node (`/so101_teleop`)

| Name | Type | Description |
|---|---|---|
| `/leader/joint_states` | sensor_msgs/msg/JointState | Subscribes to the leader angles (BestEffort) |
| `/so101_teleop/enable` | std_srvs/srv/SetBool | `true` starts with alignment; `false` stops |

Commands to the follower and the start and stop of follower control use the [follower control interface](so101_follower.md#ros-interface).

## Troubleshooting

| Situation | What to check |
|---|---|
| Waiting for state | The connection and settings of the arm named in the terminal |
| Alignment cannot start (`Cannot align`) | Whether the leader pose is within the follower's range |
| Alignment timed out | The angle difference of the joint shown, the speed settings and the time limit |
| Communication error or lost state | The connection and the motor fault display |

The targets sent to the follower are limited to its command range, which comes from the calibration.
If a joint cannot get within the tolerance even at the edge of the range, alignment is aborted.

Teleoperation also stops when the leader or follower state is more than 0.5 seconds old.
Fix the cause, then resume with the start service. It does not resume automatically.
