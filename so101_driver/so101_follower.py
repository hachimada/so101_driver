"""Follower coordinates and inverse mapping, independent of ROS."""

import json
import math
from dataclasses import replace
from datetime import datetime, timezone

from .calibration import save_profile
from .hardware.feetech_bus import MAX_TICK
from .profile import JOINTS, MOTOR_IDS, CalibrationProfile

ANGLE_EPSILON = 1e-9  # Floating-point roundoff only, far below one encoder tick.


class SO101Follower:
    def __init__(self, bus, profile_file):
        self.bus = bus
        self.profile = CalibrationProfile(profile_file, role="follower")
        self.conversions = {}
        for i, name in enumerate(JOINTS, 1):
            c = self.profile.conversions[name]
            # A freely moved arm can exceed the configured servo travel.
            # Preserve the observations; only commands use the intersection.
            h = self.profile.data["hardware_configuration"][str(i)]
            c = replace(
                c,
                tick_min=max(c.tick_min, h["range_min"]),
                tick_max=min(c.tick_max, h["range_max"]),
            )
            self.conversions[name] = c
            if not 0 <= c.tick_min < c.tick_max <= MAX_TICK:
                raise ValueError(f"No valid calibrated command range: {name}")

    def validate(self):
        self.profile.validate_hardware(self.bus)

    def validate_control(self):
        self.validate()
        hardware = self.bus.read_configuration(MOTOR_IDS)
        for i, name in enumerate(JOINTS, 1):
            c = self.conversions[name]
            h = hardware[i]
            if h["operating_mode"] != 0:
                raise ValueError(f"Position mode is required: {name}")
            if not h["range_min"] <= c.tick_min < c.tick_max <= h["range_max"]:
                raise ValueError(f"Calibration exceeds hardware limits: {name}")
        return hardware

    def read_raw(self):
        raw = self.bus.read_positions(MOTOR_IDS).positions_ticks
        if set(raw) != set(MOTOR_IDS) or any(
            type(v) is not int or not 0 <= v <= MAX_TICK for v in raw.values()
        ):
            raise ValueError("Invalid position sample")
        return raw

    def to_radians(self, raw):
        # Observed positions are not clamped to command limits.
        return [self.conversions[name].to_radians(raw[i]) for i, name in enumerate(JOINTS, 1)]

    def to_ticks(self, positions):
        if len(positions) != 6 or not all(math.isfinite(v) for v in positions):
            raise ValueError("Six finite positions are required")
        result = {}
        for i, (name, position) in enumerate(zip(JOINTS, positions), 1):
            try:
                result[i] = self.conversions[name].to_ticks(position)
            except ValueError as exc:
                raise ValueError(f"{name}: {exc}") from exc
        return result

    def command_limits(self):
        """Convert the profile's observed/configured tick range into joint angles."""
        limits = []
        for name in JOINTS:
            c = self.conversions[name]
            endpoints = (c.to_radians(c.tick_min), c.to_radians(c.tick_max))
            limits.append((min(endpoints), max(endpoints)))
        return limits

    def home_positions(self):
        """Convert an optional saved hardware pose using this calibration."""
        home = self.profile.data.get("home_position")
        if home is None:
            return None
        if not isinstance(home, dict):
            raise ValueError("Invalid home position format")
        ticks = home.get("positions_ticks")
        if (
            not isinstance(ticks, dict)
            or set(ticks) != set(JOINTS)
            or any(type(value) is not int or not 0 <= value <= MAX_TICK for value in ticks.values())
        ):
            raise ValueError("Home position needs valid ticks for all 6 joints")
        raw = {i: ticks[name] for i, name in enumerate(JOINTS, 1)}
        positions = self.to_radians(raw)
        self.to_ticks(positions)
        return positions

    def save_home_positions(self, positions):
        """Save a measured pose in the selected profile without commanding hardware."""
        ticks = self.to_ticks(positions)
        profile = json.loads(self.profile.path.read_text())
        if not isinstance(profile, dict):
            raise ValueError("Invalid calibration JSON format")
        if {key: value for key, value in profile.items() if key != "home_position"} != {
            key: value for key, value in self.profile.data.items() if key != "home_position"
        }:
            raise ValueError("Calibration file changed; restart the GUI")
        profile["home_position"] = {
            "recorded_at": datetime.now(timezone.utc).isoformat(),
            "positions_ticks": {name: ticks[i] for i, name in enumerate(JOINTS, 1)},
        }
        save_profile(self.profile.path, profile, overwrite=True)
        self.profile.data = profile
        return self.to_radians(ticks)
