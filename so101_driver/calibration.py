"""Software calibration records; never write servo registers."""

import json
import os
import tempfile
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

from .hardware.feetech_bus import MAX_TICK
from .profile import (
    GRIPPER_CLOSED_RAD,
    GRIPPER_OPEN_RAD,
    JOINTS,
    MIN_TRAVEL_TICKS,
    MOTOR_IDS,
    RADIANS_PER_TICK,
)

RANGE_IDS = (1, 2, 3, 4, 6)  # wrist_roll uses the full turn.


class CalibrationSession:
    def __init__(
        self, hardware_configuration, *, robot_role="leader", source_device_id="", source_port=""
    ):
        if robot_role not in ("leader", "follower"):
            raise ValueError("robot_role must be leader or follower")
        self.robot_role = robot_role
        self.source_port = source_port
        self.source_device_id = source_device_id
        self.hardware = hardware_configuration
        self.lock = threading.RLock()
        self.latest = None
        self.latest_time = 0.0
        self.error = "Waiting for first sample"
        self.zero = None
        self.ranges = {}
        self.recording_ranges = False
        self.ranges_complete = False
        self.closed = None
        self.opened = None

    def accept(self, positions, now=None):
        if set(positions) != set(MOTOR_IDS):
            raise ValueError("A complete six-motor sample is required")
        if any(type(v) is not int or not 0 <= v <= MAX_TICK for v in positions.values()):
            raise ValueError(f"Position outside single-turn range 0..{MAX_TICK}")
        with self.lock:
            self.latest = dict(positions)
            self.latest_time = time.monotonic() if now is None else now
            self.error = ""
            if self.recording_ranges:
                for motor_id in MOTOR_IDS:
                    value = positions[motor_id]
                    record = self.ranges[motor_id]
                    record["min"] = min(record["min"], value)
                    record["max"] = max(record["max"], value)
                    record["samples"] += 1

    def fail(self, message):
        with self.lock:
            self.error = str(message)

    def capture(self, now=None):
        with self.lock:
            now = time.monotonic() if now is None else now
            if self.latest is None or self.error or now - self.latest_time > 0.5:
                raise ValueError(
                    "No fresh healthy sample. Wait for recovery and press Enter again."
                )
            return dict(self.latest)

    def start_from_reference(self):
        """Record the reference and start collecting ranges on the same sample."""
        with self.lock:
            self.zero = self.capture()
            self.closed = self.zero[6]
            self.begin_ranges()

    def begin_ranges(self):
        """Start/resume collecting every joint together, preserving prior observations."""
        with self.lock:
            positions = self.capture()
            for motor_id in MOTOR_IDS:
                value = positions[motor_id]
                self.ranges.setdefault(motor_id, {"min": value, "max": value, "samples": 0})
            self.recording_ranges = True
            self.ranges_complete = False

    def end_ranges(self):
        with self.lock:
            self.recording_ranges = False
            self.capture()
            issues = []
            for motor_id in RANGE_IDS:
                record = self.ranges[motor_id]
                span = record["max"] - record["min"]
                if record["samples"] < 10 or span < MIN_TRAVEL_TICKS:
                    issues.append(f"{JOINTS[motor_id - 1]}: not enough movement or valid samples")
                if span > 4000:
                    issues.append(
                        f"{JOINTS[motor_id - 1]}: may have crossed the encoder wrap-around"
                    )
            if issues:
                raise ValueError("; ".join(issues))
            if self.closed is None:
                raise ValueError("Record the reference pose with the gripper closed first")
            gripper = self.ranges[6]
            candidates = (gripper["min"], gripper["max"])
            # The user starts with the gripper closed. No direction assumption needed.
            self.opened = max(candidates, key=lambda value: abs(value - self.closed))
            if abs(self.opened - self.closed) < MIN_TRAVEL_TICKS:
                raise ValueError("gripper: not enough opening movement")
            self.ranges_complete = True

    def profile(self):
        with self.lock:
            if self.zero is None or self.closed is None or self.opened is None:
                raise ValueError("Reference and both gripper endpoints must be recorded")
            if abs(self.opened - self.closed) < MIN_TRAVEL_TICKS:
                raise ValueError("Gripper open/closed endpoints are too close")
            if not self.ranges_complete:
                raise ValueError("Complete range recording for all joints first")
            return {
                "format_version": 1,
                "kind": "so101_software_calibration",
                "robot_role": self.robot_role,
                "source_device_id": self.source_device_id,
                "source_port": self.source_port,
                "created_at": datetime.now(timezone.utc).isoformat(),
                "hardware_configuration": {str(i): dict(v) for i, v in self.hardware.items()},
                "joints": {
                    name: {
                        "id": i,
                        "zero_ticks": self.zero[i],
                        # SO-101 servos turn the same way as the URDF joints. Profiles
                        # still accept -1 so that a hand-edited joint can be reversed.
                        "direction": 1,
                        "radians_per_tick": RADIANS_PER_TICK,
                        "range_min": self.ranges[i]["min"] if i in RANGE_IDS else 0,
                        "range_max": self.ranges[i]["max"] if i in RANGE_IDS else MAX_TICK,
                    }
                    for i, name in enumerate(JOINTS[:5], 1)
                },
                "gripper": {
                    "id": 6,
                    "closed_ticks": self.closed,
                    "open_ticks": self.opened,
                    "open_position_source": "furthest_observed_from_initial_closed",
                    "range_min": self.ranges[6]["min"],
                    "range_max": self.ranges[6]["max"],
                    "closed_angle_rad": GRIPPER_CLOSED_RAD,
                    "open_angle_rad": GRIPPER_OPEN_RAD,
                },
                "notes": "Software reference only. Homing offsets and servo limits unchanged.",
            }


def save_profile(path, profile, *, overwrite=False):
    """Atomically save; replacement requires explicit caller authorization."""
    path = Path(path).expanduser()
    content = json.dumps(profile, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary = Path(temporary)
    try:
        with os.fdopen(fd, "w") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        if overwrite:
            os.replace(temporary, path)
        else:
            os.link(temporary, path)  # Fails if another file appeared before saving.
    finally:
        temporary.unlink(missing_ok=True)
