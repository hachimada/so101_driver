# Calibration

**English** | [日本語](ja/so101_calibration.md)

`so101_calibrate` records the reference pose, the joint ranges and the closed and open gripper positions of the leader or follower arm.
The result is saved as JSON and used to convert joint angles and to limit control targets.
Calibration is software only: it never changes the homing offsets or position limits stored in the servos.

## Before you start

- [Find the device ID](so101_setup.md#find-the-device-id) of the arm.
- Stop the driver launches (control interface, slider GUI or teleoperation) and any other `so101_calibrate`. Calibration exits without starting while a leader, follower or calibration node is running.
- Calibrate one arm at a time.

## Run

```bash
ros2 run so101_driver so101_calibrate --role leader --device-id "your-leader-serial"
ros2 run so101_driver so101_calibrate --role follower --device-id "your-follower-serial"
```

Without `--device-id`, the `device_id` in the [settings file](so101_setup.md#settings-files) (`leader.yaml` or `follower.yaml` in `~/.ros/so101_driver/config/`) is used.

## Steps

1. Put the arm in the same pose as the blue reference model in RViz, close the gripper and press Enter.
2. Move every joint and the gripper through its range. Any order works, and you can move several joints at once.
3. Press Enter to finish recording and save.

While recording, the terminal shows the minimum and maximum of each joint in ticks.
If a joint does not have enough data, the joints to move are listed and you can resume recording.
Press `q` or Ctrl+C to cancel.

Each joint except `wrist_roll` needs at least 10 valid samples and 20 ticks of travel.
A range wider than 4000 ticks is rejected because the joint may have crossed the encoder wrap-around.

### RViz preview

Before you start, RViz shows only the blue reference model.
After you record the reference pose with Enter, a gray model of the current pose appears next to it.
The blue model stays fixed, and the gray model follows the real arm. You can change the view with the mouse.
During a communication error the gray model may keep its last pose, so also check the status in the terminal.

### Gripper and wrist roll

The gripper position at the start is treated as closed, and the recorded position farthest from it as open.
`wrist_roll` is saved with the full turn (0–4095 ticks) as its range.

## When torque is enabled

If torque is enabled on any joint, calibration does not start.
To release torque and calibrate, add `--disable-torque`:

```bash
ros2 run so101_driver so101_calibrate --role follower \
  --device-id "your-follower-serial" --disable-torque
```

When prompted, support the arm and press Enter; torque is released and calibration continues.
Press `q` to quit before torque is released. Released torque stays off after calibration finishes or is canceled.
The option works for the leader as well.

## Output files

| Role | Default file (in `~/.ros/so101_driver`) |
|---|---|
| Leader | `calibration/leader.json` |
| Follower | `calibration/follower.json` |

If the file already exists, the terminal asks whether to overwrite it right after startup.
Enter `y` or `yes` to continue; press Enter or `n` to exit.
The existing file is not changed until the new result has been saved.

The settings YAML files created from the templates point to these files.
If you save elsewhere, also change `calibration_profile` in the corresponding YAML.

```yaml
calibration_profile: calibration/follower.json
```

A relative `calibration_profile` path is resolved from `~/.ros/so101_driver`,
whereas a relative `--output` path is resolved from the current directory.

## Options

| Option | Description |
|---|---|
| `--role leader\|follower` | Arm to calibrate. Default: `leader` |
| `--device-id` | USB serial number. Default: the value in the role's YAML |
| `--output` | Output file |
| `--no-rviz` | Run without the reference model preview (RViz and robot_state_publisher) |
| `--disable-torque` | Release enabled torque after confirmation |

## Troubleshooting

| Message or situation | What to check |
|---|---|
| `Stop the running driver nodes and other calibration sessions first.` | Stop the driver nodes and other `so101_calibrate` processes |
| `... is in use by another process` | Another process is using the same arm's USB port |
| `Torque is enabled on some ... joints` | Rerun with `--disable-torque` |
| `Some servos are not in position mode` / `Found a servo that is not an STS3215` | The servo settings or models do not match the SO-101. No settings were changed |
| `Not enough data recorded` | Move the listed joints further, then resume |

### When a calibration result cannot be used

Using the leader file for the follower, or vice versa, is an error at startup (`Calibration role mismatch`).
If the servo homing offset, position limits or operating mode differ from when the calibration was recorded, the node reports the mismatch (`Hardware changed since calibration`) and does not start.
Check the arm and its settings, and calibrate again if you changed the servo settings.
