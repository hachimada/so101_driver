# Slider GUI

**English** | [日本語](ja/so101_gui.md)

The slider GUI moves the six follower joints and shows the state of each motor.

## Start

Complete [setup](so101_setup.md) and [calibration](so101_calibration.md) first.

```bash
ros2 launch so101_driver follower_slider_control.launch.py
```

This also starts the [follower node](so101_follower.md) with control allowed (`allow_writes` defaults to `true`).
Use `params_file:=/path/to/follower.yaml` to choose another settings file.

## Controls

1. Wait until the current angles appear and the status shows **Ready to start**.
2. Press **Start control**. The current pose becomes the initial target.
3. Set target angles with the sliders or the number boxes. Angles are shown in degrees.
4. Press **Stop** to stop. The arm holds its current position.
5. To quit, support the arm and press **Release torque and quit**.

The current and target angles are shown separately.
The range of each slider comes from the range recorded during calibration and the servo position limits.

**Stop** and closing the window keep torque on.
**Release torque and quit** closes the window after confirming that torque is released on all joints.
If the release fails, the reason is shown; check the connection and try again.
Closing the GUI shuts down the follower node as well.

## Home pose

| Button | Action |
|---|---|
| Set current pose as home | Save the measured pose at that moment. Replaces a previously saved home pose |
| Go home | After starting control, move to the saved pose |

You can save the home pose before starting control. All joint positions must be available and within the slider ranges.
The home pose is saved in the calibration JSON and is available in later sessions.
After you calibrate again, save the home pose again.

**Stop** interrupts a move to home.
The speed and acceleration are the same as for normal slider moves.
See [Motion settings](so101_follower.md#motion-settings).

## Motor status

The lower part of the window shows torque state, speed, current, duty cycle, voltage, temperature and faults.
Select a joint row to see details such as the servo goal position, model information and protection settings.
The status is shown before control starts as well.

For the meaning and units of each value, see [Motor status](so101_motor_monitor.md).

## Troubleshooting

| Message or situation | What to do |
|---|---|
| The current pose is outside the command range | Check the listed joints and move them into the slider range |
| Control fails to start | Check the reason shown in the window and the terminal |
| Sliders are disabled after a communication error | Press **Start control** after the connection recovers |
| **Go home** is disabled | Save a home pose and start control. The button tooltip shows the reason |
| `Follower control is not allowed` | The follower was started with `allow_writes:=false` |

For the start conditions and stop behavior, see the [follower control interface](so101_follower.md).
