"""SO-101 joints, the calibration profile written by so101_calibrate, and tick/angle conversion."""

import json
import math
from dataclasses import dataclass

from .hardware.feetech_bus import MAX_TICK, STS3215_MODEL_NUMBER, TICKS_PER_TURN
from .paths import calibration_profile_path

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
ARM_JOINTS = JOINTS[:5]
MOTOR_IDS = range(1, 7)  # Servo IDs 1-6 drive JOINTS in order.

# Written to new profiles. Profiles saved before 2026-11 hold 2*pi/4095, as LeRobot's degree
# conversion uses, and are read with their stored value.
RADIANS_PER_TICK = 2 * math.pi / TICKS_PER_TURN

# URDF limits of the gripper joint. The calibrated closed and open ticks map onto them.
GRIPPER_CLOSED_RAD = -0.174533
GRIPPER_OPEN_RAD = 1.74533

# Smallest recorded travel accepted for a joint or for the gripper.
MIN_TRAVEL_TICKS = 20


@dataclass(frozen=True)
class JointConversion:
    """Linear map between servo ticks and a joint angle, with the ticks allowed as targets."""

    zero_ticks: float
    radians_per_tick: float
    tick_min: int
    tick_max: int

    def to_radians(self, ticks):
        return (ticks - self.zero_ticks) * self.radians_per_tick

    def to_clamped_radians(self, ticks):
        return self.to_radians(min(max(ticks, self.tick_min), self.tick_max))

    def to_ticks(self, radians):
        raw = radians / self.radians_per_tick + self.zero_ticks
        if not self.tick_min - 1e-7 <= raw <= self.tick_max + 1e-7:
            raise ValueError("Target exceeds calibrated tick range")
        return round(raw)


def gripper_conversion(
    closed_ticks, open_ticks, closed_rad=GRIPPER_CLOSED_RAD, open_rad=GRIPPER_OPEN_RAD
):
    """Map the closed and open ticks onto the gripper angles, whatever the travel in ticks."""
    scale = (open_rad - closed_rad) / (open_ticks - closed_ticks)
    return JointConversion(
        closed_ticks - closed_rad / scale,
        scale,
        min(closed_ticks, open_ticks),
        max(closed_ticks, open_ticks),
    )


class CalibrationProfile:
    """A validated calibration JSON. Conversion limits are the ticks observed while calibrating."""

    def __init__(self, profile_file, *, role):
        if not profile_file:
            raise ValueError("Set calibration_profile")
        self.path = calibration_profile_path(profile_file)
        self.data = json.loads(self.path.read_text())
        p = self.data
        if (
            not isinstance(p, dict)
            or p.get("format_version") != 1
            or p.get("kind") != "so101_software_calibration"
        ):
            raise ValueError("Use a calibration JSON created by so101_calibrate")
        if p.get("robot_role") != role:
            raise ValueError(
                f"Calibration role mismatch: expected {role}, got {p.get('robot_role')}"
            )
        self.conversions = {}
        for i, name in enumerate(ARM_JOINTS, 1):
            j = p["joints"][name]
            if j["id"] != i or j["direction"] not in (-1, 1):
                raise ValueError(f"Invalid joint calibration: {name}")
            if not (
                math.isfinite(j["zero_ticks"])
                and 0 <= j["zero_ticks"] <= MAX_TICK
                and math.isfinite(j["radians_per_tick"])
                and j["radians_per_tick"] > 0
                and 0 <= j["range_min"] < j["range_max"] <= MAX_TICK
            ):
                raise ValueError(f"Invalid joint values: {name}")
            self.conversions[name] = JointConversion(
                j["zero_ticks"],
                j["direction"] * j["radians_per_tick"],
                j["range_min"],
                j["range_max"],
            )
        g = p["gripper"]
        if g["id"] != 6 or not all(
            type(g[k]) is int and 0 <= g[k] <= MAX_TICK for k in ("closed_ticks", "open_ticks")
        ):
            raise ValueError("Invalid gripper endpoints")
        if abs(g["open_ticks"] - g["closed_ticks"]) < MIN_TRAVEL_TICKS:
            raise ValueError("Gripper endpoints too close")
        angles = (g["closed_angle_rad"], g["open_angle_rad"])
        if not all(math.isfinite(v) for v in angles) or angles[0] == angles[1]:
            raise ValueError("Gripper angle endpoints must be finite and differ")
        self.conversions["gripper"] = gripper_conversion(
            g["closed_ticks"], g["open_ticks"], *angles
        )

    def validate_hardware(self, bus):
        """Reject servos whose settings changed since this calibration was recorded."""
        if any(v != STS3215_MODEL_NUMBER for v in bus.ping(MOTOR_IDS).values()):
            raise ValueError("Expected STS3215 motors")
        actual = bus.read_configuration(MOTOR_IDS)
        for i in MOTOR_IDS:
            expected = self.data["hardware_configuration"][str(i)]
            for key in ("homing_offset", "range_min", "range_max", "operating_mode"):
                if actual[i][key] != expected[key]:
                    raise ValueError(f"Hardware changed since calibration: ID={i} {key}")

    def to_radians(self, raw):
        """Joint angles of a six-motor sample, not clamped to the observed ticks."""
        return [self.conversions[name].to_radians(raw[i]) for i, name in enumerate(JOINTS, 1)]
