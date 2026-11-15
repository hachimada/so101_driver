# Motor status

**English** | [日本語](ja/so101_motor_monitor.md)

The **Motor status** panel of the slider GUI shows information read through the official Feetech Python SDK.
It is shown before control starts as well. Select a joint row to see the details of that motor.

## Table columns

| Column | Meaning |
|---|---|
| ID | Servo motor ID (1–6) |
| Joint | Joint name, the same as the sliders |
| Torque | Servo output state: OFF / ON / Damping |
| Motion | Moving state reported by the servo: Stopped / Moving |
| Speed (rpm) | Motor speed |
| Current (mA) | Motor phase current |
| Duty (%) | PWM duty cycle: on time ÷ period × 100 |
| Voltage (V) | Voltage reported by the servo |
| Temp (°C) | Internal servo temperature |
| Status | OK, the fault names, Communication failed, Stale (no recent update) or No data (not read yet) |

The signs of current, duty cycle and speed show the direction on the servo side.
A duty cycle of 100% means the PWM output is on for the whole period.
**Stopped** is the moving state relative to the servo's internal goal, so it can be shown while you move the arm by hand.

If the servo's phase setting does not provide current feedback, the current is shown as **N/A**.
Until the setting needed for the speed conversion has been read, the speed is shown as **—**.

## Details of each joint

For the selected joint, the following are shown. All settings are read-only.

- Current angle, and the current and servo goal positions in ticks
- Raw current, voltage and speed values, the torque limit setting and the fault bits
- Model, firmware and operating mode
- Position range, and the temperature, voltage and current protection settings
- Settings such as the position PID gains

The servo goal position is the last position target sent to that servo.
Commands and the status display are updated at different times, so the display can lag just after the target changes.

## Faults and read failures

Voltage, Encoder, Overheat, Overcurrent and Overload faults are shown in red.
On a communication failure or when updates stop, the values are shown as **—**. They reappear once reads recover.
The window also shows the total number of position read failures, status read failures and rejected commands.

Control that has stopped does not restart automatically when the status display recovers.
Fix the cause, then press **Start control** in the GUI to resume.

## Update rate

`motor_state_rate_hz` in `~/.ros/so101_driver/config/follower.yaml` sets the update rate per joint.
The default is 2 Hz, and values from 0 to 10 Hz are accepted. `0.0` disables motor monitoring.
Model information and protection settings are read about every 10 seconds.

```yaml
/follower/so101_follower:
  ros__parameters:
    motor_state_rate_hz: 2.0
```

## ROS topic

| Topic | Type | Description |
|---|---|---|
| `/follower/motor_states` | diagnostic_msgs/msg/DiagnosticArray | The state of one joint per message, cycling through the six joints |

`DiagnosticStatus.name` is `/follower/so101_follower/motor/<joint_name>`,
and `hardware_id` is `<device ID>:<motor ID>`.
The main `values` keys are:

| Key | Description |
|---|---|
| `sample_valid` | Whether the status read succeeded |
| `current_ma` / `voltage_v` / `temperature_c` | Current (mA), voltage (V) and temperature (°C) |
| `position_ticks` / `speed_raw` | Raw position and speed |
| `packet_error` / `servo_status` | Fault bits of the response packet and of the servo |
| `rate_hz` / `read_failures` | Update rate, and the number of failed status reads for that joint |

## Unit conversion and sources

The current is shown as the signed value × 6.5 mA, the voltage as the value × 0.1 V, and the duty cycle as the signed value × 0.1%.
The speed is converted at 0.0146 rpm or 0.732 rpm per unit, depending on the servo phase setting.

For the specifications, see the [official Feetech STS memory table](https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/)
and the [STS3215 feedback description](https://www.feetechrc.com/20210430-56680.html).
The SDK in use is described in [Feetech SDK](feetech_sdk_source.md).
