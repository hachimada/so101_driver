"""Command ranges, fresh joint snapshots and absolute teleoperation mapping."""

import math
from pathlib import Path

import yaml

from .profile import JOINTS
from .so101_follower import ANGLE_EPSILON, SO101Follower

# GUI and teleop treat joint states and diagnostics older than this as lost.
STATE_TIMEOUT_S = 0.5
# Publishing at least this often keeps state fresh across one dropped sample.
MIN_PUBLISH_RATE_HZ = 5.0


def load_follower_model(config_file):
    params = yaml.safe_load(Path(config_file).expanduser().read_text())
    params = params["/follower/so101_follower"]["ros__parameters"]
    return SO101Follower(None, params.get("calibration_profile", ""))


def load_command_limits(config_file):
    return load_follower_model(config_file).command_limits()


class JointSnapshot:
    def __init__(self, timeout=STATE_TIMEOUT_S):
        self.timeout = timeout
        self.positions = None
        self.stamp_ns = 0
        self.received = 0.0

    def update(self, names, positions, stamp_ns, *, ros_now_ns, now):
        if len(names) != 6 or set(names) != set(JOINTS) or len(positions) != 6:
            return False
        if not all(math.isfinite(v) for v in positions):
            return False
        if not 0 < stamp_ns <= ros_now_ns or stamp_ns <= self.stamp_ns:
            return False
        if ros_now_ns - stamp_ns > self.timeout * 1e9:
            return False
        values = dict(zip(names, positions))
        self.positions = [values[name] for name in JOINTS]
        self.stamp_ns = stamp_ns
        self.received = now
        return True

    def fresh(self, *, ros_now_ns, now):
        return (
            self.positions is not None
            and 0 <= now - self.received <= self.timeout
            and 0 <= ros_now_ns - self.stamp_ns <= self.timeout * 1e9
        )


class AbsoluteTeleop:
    def __init__(self, limits, scales):
        if len(scales) != 6 or not all(math.isfinite(v) for v in scales):
            raise ValueError("joint_scales must contain six finite values")
        self.limits = limits
        self.scales = list(scales)

    def leader_angles(self, leader):
        """Scaled leader angles before clipping, also used for alignment verification."""
        if len(leader) != 6:
            raise ValueError("Teleoperation targets must contain six finite values")
        targets = [scale * value for scale, value in zip(self.scales, leader)]
        if not all(math.isfinite(value) for value in targets):
            raise ValueError("Teleoperation targets must contain six finite values")
        return targets

    def target(self, leader, *, require_in_range=False, range_tolerance_rad=0.0):
        targets = self.leader_angles(leader)
        if require_in_range:
            if not math.isfinite(range_tolerance_rad) or range_tolerance_rad < 0:
                raise ValueError("Range tolerance must be finite and nonnegative")
            for name, value, (lo, hi) in zip(JOINTS, targets, self.limits):
                if not (
                    lo - range_tolerance_rad - ANGLE_EPSILON
                    <= value
                    <= hi + range_tolerance_rad + ANGLE_EPSILON
                ):
                    raise ValueError(
                        f"Cannot align: {name} target {math.degrees(value):.1f}° is outside "
                        f"the follower range {math.degrees(lo):.1f} to {math.degrees(hi):.1f}° "
                        f"by more than {math.degrees(range_tolerance_rad):.1f}°"
                    )
        return [min(hi, max(lo, value)) for value, (lo, hi) in zip(targets, self.limits)]
