# Follower control interface

**English** | [日本語](ja/so101_follower.md)

The follower node takes six-joint position commands on a ROS topic, and starts and stops control and releases torque through services.
The [slider GUI](so101_gui.md) and [teleoperation](so101_teleop.md) also use this interface.

## Start

Complete [setup](so101_setup.md) and [calibration](so101_calibration.md) first.

```bash
ros2 launch so101_driver follower_control.launch.py allow_writes:=true
```

`allow_writes:=true` allows control. Without it, the node only reads state.
`allow_writes` is set only by this launch argument, not in `follower.yaml`.
Control takes effect only after you call the [start service](#start-and-stop-control).

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `params_file` | `~/.ros/so101_driver/config/follower.yaml`, or the installed template if it does not exist | Follower settings YAML |
| `allow_writes` | `false` | Allow starting control and releasing torque |
| `release_torque_on_shutdown` | `false` | Release torque on all six joints when the node shuts down |

Only one process at a time can use an arm's USB port.
If another node or `so101_calibrate` is using the same arm, the node prints `is in use by another process` and does not start.
The slider GUI and teleoperation also start this node, so they cannot run at the same time on the same arm.

## ROS interface

| Name | Type | Node QoS | Description |
|---|---|---|---|
| `/follower/joint_command` | sensor_msgs/msg/JointState | Subscribe: Reliable, KeepLast 1 | Position targets for the six joints, rad |
| `/follower/joint_states` | sensor_msgs/msg/JointState | Publish: **BestEffort**, KeepLast 5 | Measured angles of the six joints, rad |
| `/follower/diagnostics` | diagnostic_msgs/msg/DiagnosticArray | Publish: Reliable, KeepLast 10 | Control state and errors. See [Diagnostics](#diagnostics) |
| `/follower/motor_states` | diagnostic_msgs/msg/DiagnosticArray | Publish: Reliable, KeepLast 10 | State of each motor. See [Motor status](so101_motor_monitor.md#ros-topic) |
| `/follower/enable_control` | std_srvs/srv/SetBool | — | `true` starts control; `false` holds and stops accepting commands |
| `/follower/release_torque` | std_srvs/srv/Trigger | — | Release torque on all joints |

`/follower/joint_states` is published with BestEffort QoS, so subscribe with BestEffort as well.
A subscriber with the default Reliable QoS receives nothing. With `ros2 topic echo`, add `--qos-reliability best_effort`.

### Command format

Send a [JointState](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/JointState.msg) on `/follower/joint_command` as follows:

| Field | Value |
|---|---|
| `name` | Each of `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper` exactly once, in any order |
| `position` | Six finite values matching `name`. The gripper is also in rad |
| `header.stamp` | ROS time when sent. Update it for every command |
| `velocity` / `effort` / `header.frame_id` | Empty |

Send all six joints in every command.
A command is rejected if its stamp is zero, in the future, before control started, not newer than the previous command, or older than `command_timeout_s`.
A command with a target outside the [command range](#command-range) is also rejected.
Keep the clocks of the sender and the driver in sync, and use `use_sim_time=false`; the node refuses to start with `allow_writes` and simulated time.

The subscription QoS is Reliable / Volatile / KeepLast 1, so publish with Reliable as well.
There is no reply when a target is reached; read the current position from `/follower/joint_states`.

### Start and stop control

Send `data: true` to `/follower/enable_control` to start control and `data: false` to stop it.
For example commands, see [Control from the terminal](#control-from-the-terminal).

Before starting, have a program ready that keeps sending targets for all six joints.
At start, the current position becomes the holding target.
Commands sent before the start are not used, so keep sending commands with new stamps after the start.
Each accepted command keeps control alive until `command_timeout_s` after its stamp; if no valid command arrives in time, control stops.

The start is accepted when:

- the current position of every joint is within its servo position limits, and
- torque is off on all joints, or on with every servo stationary and holding its position.

If torque is on for only some joints right after startup, or holding cannot be confirmed, the start is rejected with the reason.

Stopping sends a hold command at the current position and stops accepting commands. Torque stays on.
Send `data: true` to resume; the current position becomes the initial target again.
When this node shuts down (for example with Ctrl+C) while in control, it also tries to hold the position.
With `release_torque_on_shutdown:=true`, it releases torque on all six joints at shutdown instead.
[Teleoperation](so101_teleop.md#shut-down) enables the torque release at shutdown automatically.

### Release torque

Support the arm, then run:

```bash
ros2 service call /follower/release_torque std_srvs/srv/Trigger '{}'
```

The node stops accepting commands, confirms by readback that torque is off on all six joints and then returns success.
If the release fails, the reason is returned. Check the connection and try again.
This service also requires `allow_writes:=true`.

## Control from the terminal

With the [start command](#start) running in terminal 1, use the following steps.
Terminal 2 sends position commands, and terminal 3 starts and stops control.

### 1. Read the current angles

In terminal 2, read the joint names and angles once:

```bash
ros2 topic echo /follower/joint_states sensor_msgs/msg/JointState \
  --once --qos-reliability best_effort
```

`position` lists the angles (rad) in the same order as `name`.
To move only one joint, use the current angles as the targets of the other five.

### 2. Send the target angles

Run the following in terminal 2 and leave it running.
This example sets `shoulder_pan` to 0.1 rad (about 5.7°) and the other five joints to 0 rad.
Replace the six `position` values with targets within your arm's calibrated range.

```bash
ros2 topic pub /follower/joint_command sensor_msgs/msg/JointState \
  '{header: auto,
    name: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper],
    position: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]}' \
  --rate 30 --qos-reliability reliable
```

`header: auto` stamps each message with the current time, and `--rate 30` sends 30 messages per second.
The commands are rejected until control starts, and accepted once you start control in the next step.
Keep sending after the target is reached. If sending stops, control stops after 0.5 seconds by default.

### 3. Start control

In terminal 3, run:

```bash
ros2 service call /follower/enable_control std_srvs/srv/SetBool '{data: true}'
```

Check for `success: true`. The follower moves to the target being sent.
You can then watch the current angles. Press Ctrl+C to stop watching.

```bash
ros2 topic echo /follower/joint_states sensor_msgs/msg/JointState \
  --field position --qos-reliability best_effort
```

### 4. Stop, or change the target

In terminal 3, stop watching the angles and call the stop service:

```bash
ros2 service call /follower/enable_control std_srvs/srv/SetBool '{data: false}'
```

After stopping, press Ctrl+C in terminal 2 to stop sending.
To move to another target, change `position` in step 2, send it again and start control as in step 3.
To also turn off torque, support the arm and run the [release torque](#release-torque) command.

## Motion settings

The node sends the latest target to the servos and updates it even while they are moving.
The servos handle the speed, acceleration and tracking of the target.
These settings are shared by the GUI, the home move and teleoperation.

Edit `~/.ros/so101_driver/config/follower.yaml` and restart the node to apply them.

| Setting | Default | Description |
|---|---|---|
| `publish_rate_hz` | 30.0 | Rate of state reads and target updates (5–100 Hz) |
| `command_timeout_s` | 0.5 | How long a command stays valid, and how long to wait for the first command after the start (seconds, greater than 0) |
| `max_velocity_rad_s` | 1.57 | Speed of every joint (rad/s, greater than 0; 1.57 is about 90°/s). Higher values move faster |
| `servo_acceleration_raw` | 20 | Acceleration setting (integer, 1–254). Higher values change speed more abruptly |
| `gripper_torque_limit_pct` | 20.0 | Gripper output limit (0–100%, in steps of 0.1%) |
| `motor_state_rate_hz` | 2.0 | Motor status rate per joint (0–10 Hz). 0 disables it |

With an out-of-range value, the node logs an error at startup and exits.

The GUI and teleoperation stop control when the joint states or diagnostics are more than 0.5 seconds old.
That is why `publish_rate_hz` must be at least 5 Hz: a single failed read then does not stop control.

### Command range

The command range of each joint is the intersection of the range observed during calibration and the servo position limits recorded at calibration.
Targets outside this range are rejected. The URDF joint limits are not used for control.

### Speed and acceleration

The speed is converted to servo units (ticks/s) for each joint, so all six joints, gripper included, move at `max_velocity_rad_s`.
The gripper angle maps the calibrated closed and open positions to -0.17 to 1.75 rad.
At the default 1.57 rad/s, the gripper moves from closed to open in about 1.2 seconds (excluding acceleration).

Acceleration is defined on the servo angle. One unit of `servo_acceleration_raw` is 8.7°/s², so the default of 20 is 174°/s².
For the gripper, the acceleration in joint angle differs from this value depending on the calibrated opening width.

### Gripper output limit

A higher `gripper_torque_limit_pct` grips more firmly but can increase heating and overload.
A lower value reduces the load but may let objects slip.
100% is the maximum servo output and 0% is no output; it is not a grip force in newtons.
It is applied and verified at every control start, and is not kept after a power cycle.
The output limits and overload protection of the five arm joints are not changed.

For the register specifications, see the [official Feetech STS memory table](https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/).

## Communication errors and command loss

If no valid command arrives in time, or reading positions or sending targets fails, the node tries to hold the position and stops control.
State publishing resumes when communication recovers, but control resumes only after you call the start service again.

`Hold NOT confirmed` means the hold command could not be confirmed.
Check the connection; you can retry the hold with the stop service (`data: false`).
A lost connection or a failed write to some joints may leave the joints not holding.

## Diagnostics

`/follower/diagnostics` publishes one `DiagnosticStatus` named `/follower/so101_follower/hardware`, with the device ID as `hardware_id`.

| Level | Meaning |
|---|---|
| OK | Normal operation |
| WARN | Control stopped while torque is on (holding) |
| ERROR | A position read or target write failed, or a hold or torque release was not confirmed |

| Key | Description |
|---|---|
| `allow_writes` | Whether control is allowed |
| `control_active` | Whether control is running |
| `control_status` | Control state message |
| `failed_samples` | Number of failed position reads |
| `rejected_commands` | Number of rejected commands |
| `motor_state_rate_hz` | Motor status rate per joint |
