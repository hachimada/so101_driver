"""Forward the latest position target; the servos handle speed and acceleration."""

import math
from time import monotonic

from .profile import JOINTS, MOTOR_IDS


class FollowerControl:
    def __init__(
        self,
        robot,
        *,
        allow_writes=False,
        command_timeout_s=0.5,
        max_velocity_rad_s=1.57,
        servo_acceleration_raw=20,
        gripper_torque_limit_pct=20.0,
        clock=monotonic,
    ):
        for value in (command_timeout_s, max_velocity_rad_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Timeout and velocity must be finite and positive")
        if type(servo_acceleration_raw) is not int or not 1 <= servo_acceleration_raw <= 254:
            raise ValueError("servo_acceleration_raw must be an integer in [1, 254]")
        if (
            type(gripper_torque_limit_pct) not in (int, float)
            or not math.isfinite(gripper_torque_limit_pct)
            or not 0 <= gripper_torque_limit_pct <= 100
        ):
            raise ValueError("gripper_torque_limit_pct must be a finite number in [0, 100]")
        self.robot = robot
        self.bus = robot.bus
        self.allow_writes = allow_writes
        self.timeout = command_timeout_s
        self.clock = clock
        # The servo speed is in ticks/s and rad/tick differs between the arm and the gripper,
        # so convert per joint to move every joint at max_velocity_rad_s.
        self.speeds = {
            motor_id: math.floor(max_velocity_rad_s / abs(robot.conversions[name].radians_per_tick))
            for motor_id, name in enumerate(JOINTS, 1)
        }
        if not all(1 <= speed <= 32767 for speed in self.speeds.values()):
            raise ValueError("Velocity exceeds SDK speed range")
        self.acceleration = servo_acceleration_raw
        self.gripper_torque_limit_raw = round(gripper_torque_limit_pct * 10)
        self.active = False
        self.engaged = False
        self.hold_failed = False
        self.release_failed = False
        self.message = "Read only" if not allow_writes else "Control disabled"
        self.target = None
        self.last_stamp = 0
        self.enabled_stamp = 0
        self.deadline = 0.0

    def _write(self, raw):
        self.bus.write_positions(raw, speeds=self.speeds, acceleration=self.acceleration)

    def enable(self, ros_now_ns):
        if not self.allow_writes:
            raise ValueError("Writes are disabled; restart with allow_writes:=true")
        if self.release_failed:
            raise ValueError("Torque release not confirmed; release torque again")
        if self.active:
            return
        hardware = self.robot.validate_control()
        enabled = [i for i, h in hardware.items() if h["torque_enabled"] != 0]
        if enabled and len(enabled) != 6 and not self.engaged:
            raise ValueError(
                "Torque is enabled on only some joints; enable or release all six first"
            )
        motion = self.bus.read_motion_status(enabled) if enabled else {}
        raw = self.robot.read_raw()
        # Preloading the measured pose causes no movement. It may be outside observed
        # calibration travel, but must still be inside the servo's actual limits.
        # Subsequent movement targets remain subject to the calibrated command limits.
        for motor_id, name in enumerate(JOINTS, 1):
            h = hardware[motor_id]
            if not h["range_min"] <= raw[motor_id] <= h["range_max"]:
                raise ValueError(
                    f"{name}: position {raw[motor_id]} is outside the servo range "
                    f"{h['range_min']} to {h['range_max']} ticks"
                )
        for motor_id, state in motion.items():
            if state["moving"] or abs(state["goal_ticks"] - raw[motor_id]) > 5:
                raise ValueError(
                    f"ID={motor_id}: cannot confirm it is holding "
                    f"(position={raw[motor_id]}, goal={state['goal_ticks']}, "
                    f"moving={state['moving']})"
                )
        if enabled:
            self.engaged = True  # Existing torque also needs a hold attempt if takeover fails.
        try:
            self._write(raw)  # Preload current goals on ALL axes before enabling any torque.
            self.engaged = True  # A partial torque-write failure still requires a hold attempt.
            self.bus.set_torque_limit(6, self.gripper_torque_limit_raw)
            disabled = [i for i in MOTOR_IDS if i not in enabled]
            if disabled:
                self.bus.set_torque_enabled(disabled, True)
        except Exception:
            self.stop("Torque enabling failed; check hardware")
            raise
        self.active = True
        self.hold_failed = False
        self.target = None
        self.deadline = self.clock() + self.timeout
        self.enabled_stamp = ros_now_ns
        self.last_stamp = 0
        self.message = "Control enabled; waiting for a fresh command"

    def submit(self, positions, *, stamp_ns, ros_now_ns):
        if not self.active:
            raise ValueError("Control is disabled")
        if not (stamp_ns > 0 and self.enabled_stamp <= stamp_ns <= ros_now_ns):
            raise ValueError("Command stamp must follow enable time and must not be in the future")
        if ros_now_ns - stamp_ns > self.timeout * 1e9 or stamp_ns <= self.last_stamp:
            raise ValueError("Command is stale or out of order")
        self.robot.to_ticks(positions)
        now = self.clock()
        # A delayed callback must not revive control after its deadline.
        if now > self.deadline:
            self.stop("Command timeout; enable control again")
            raise ValueError(self.message)
        self.target = list(positions)
        self.last_stamp = stamp_ns
        self.deadline = now + self.timeout - (ros_now_ns - stamp_ns) / 1e9

    def update(self, raw):
        if not self.active:
            return
        now = self.clock()
        if now > self.deadline:
            self.stop("Command timeout; enable control again", raw=raw)
            return
        if self.target is None:
            return
        # A distant target is expected during a move. Send it directly and let
        # the servo's position controller apply the speed/acceleration settings.
        try:
            self._write(self.robot.to_ticks(self.target))
        except Exception:
            self.stop("Position write failed; check hardware")
            raise
        self.message = "Following position commands"

    def stop(self, reason="Holding position; control disabled", *, raw=None):
        self.active = False
        self.target = None
        self.message = reason
        self.hold_failed = False
        if self.engaged:
            try:
                self._write(self.robot.read_raw() if raw is None else raw)
            except Exception as exc:
                self.hold_failed = True
                self.message = f"Hold NOT confirmed: {exc}. {reason}"
                raise
        # Torque remains enabled to avoid dropping the arm. No automatic resume.

    def release_torque(self):
        if not self.allow_writes:
            raise ValueError("Writes are disabled; restart with allow_writes:=true")
        self.active = False
        self.target = None
        self.release_failed = True
        self.message = "Torque release not confirmed"
        errors = []
        # Try every axis even if one acknowledgement is lost. Readback decides success.
        for motor_id in MOTOR_IDS:
            try:
                self.bus.set_torque_enabled([motor_id], False)
            except Exception as exc:
                errors.append(str(exc))
        try:
            hardware = self.bus.read_configuration(MOTOR_IDS)
            remaining = [i for i in MOTOR_IDS if hardware[i]["torque_enabled"] != 0]
            if remaining:
                raise ValueError(f"Torque still enabled on IDs: {remaining}")
        except Exception as exc:
            self.message = f"Torque release not confirmed: {exc}"
            if errors:
                self.message += "; " + "; ".join(errors)
            raise ValueError(self.message) from exc
        self.engaged = False
        self.hold_failed = False
        self.release_failed = False
        self.message = "Torque released on all 6 joints"
