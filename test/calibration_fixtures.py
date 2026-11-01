"""Calibration profiles for simulated arms."""

import math

from so101_driver.profile import JOINTS


def make_profile(hardware, *, role="follower", zero_ticks=2000):
    return {
        "format_version": 1,
        "kind": "so101_software_calibration",
        "robot_role": role,
        "hardware_configuration": {str(i): dict(values) for i, values in hardware.items()},
        "joints": {
            name: dict(
                id=i,
                zero_ticks=zero_ticks,
                direction=1,
                radians_per_tick=2 * math.pi / 4096,
                range_min=hardware[i]["range_min"],
                range_max=hardware[i]["range_max"],
            )
            for i, name in enumerate(JOINTS[:5], 1)
        },
        "gripper": dict(
            id=6,
            closed_ticks=hardware[6]["range_min"],
            open_ticks=hardware[6]["range_max"],
            closed_angle_rad=-0.174533,
            open_angle_rad=1.74533,
        ),
    }
