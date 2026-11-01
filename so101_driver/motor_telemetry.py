"""Motor diagnostics over ROS, separate from the position control feedback."""

import math
from dataclasses import dataclass
from time import monotonic

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .hardware.feetech_bus import BusError
from .messages import stamp_ns
from .profile import JOINTS, MOTOR_IDS

FAULTS = {1: "Voltage", 2: "Encoder", 4: "Overheat", 8: "Overcurrent", 32: "Overload"}


def fault_text(bits):
    labels = [label for mask, label in FAULTS.items() if bits & mask]
    unknown = bits & ~sum(FAULTS)
    if unknown:
        labels.append(f"Unknown fault 0x{unknown:02x}")
    return " / ".join(labels) if labels else "OK"


class MotorTelemetryPublisher:
    def __init__(self, node, bus, rate_hz):
        self.node, self.bus, self.rate = node, bus, rate_hz
        self.publisher = node.create_publisher(DiagnosticArray, "motor_states", 10)
        self.index = 0
        self.info = {}
        self.info_read_at = {}
        self.failures = {i: 0 for i in MOTOR_IDS}
        # Spread reads across the control cycle instead of polling six motors at once.
        self.timer = node.create_timer(1 / (6 * rate_hz), self.poll)

    def poll(self):
        motor_id = self.index + 1
        self.index = (self.index + 1) % 6
        stamp = self.node.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = self.node.get_fully_qualified_name() + "/motor/" + JOINTS[motor_id - 1]
        status.hardware_id = self.node.device_id + f":{motor_id}"
        values = {"motor_id": motor_id, "sample_valid": False, "rate_hz": self.rate}
        try:
            sample = self.bus.read_motor_status([motor_id])[motor_id]
            values.update(sample, sample_valid=True)
            if monotonic() - self.info_read_at.get(motor_id, -math.inf) >= 10:
                try:
                    self.info[motor_id] = self.bus.read_motor_info([motor_id])[motor_id]
                    self.info_read_at[motor_id] = monotonic()
                except (BusError, OSError, ValueError) as exc:
                    values["info_error"] = str(exc)
            values.update(self.info.get(motor_id, {}))
            if "phase" in values and "speed_raw" in sample:
                values["speed_rpm"] = sample["speed_raw"] * (
                    0.0146 if values["phase"] & 4 else 0.732
                )
            if motor_id in self.info_read_at:
                values["info_age_s"] = monotonic() - self.info_read_at[motor_id]
            faults = sample["packet_error"] | sample["servo_status"]
            status.level = DiagnosticStatus.ERROR if faults else DiagnosticStatus.OK
            status.message = fault_text(faults)
        except (BusError, OSError, ValueError) as exc:
            self.failures[motor_id] += 1
            status.level = DiagnosticStatus.ERROR
            status.message = f"Communication failed: {exc}"
        values["read_failures"] = self.failures[motor_id]
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        msg = DiagnosticArray()
        msg.header.stamp = stamp
        msg.status = [status]
        self.publisher.publish(msg)


@dataclass
class MotorRecord:
    values: dict
    level: int
    message: str
    stamp_ns: int
    received: float

    def number(self, key):
        try:
            value = float(self.values[key])
            return value if math.isfinite(value) else None
        except (KeyError, ValueError, TypeError):
            return None

    def fresh(self, ros_now_ns, now):
        rate = self.number("rate_hz") or 2.0
        timeout = max(1.5, 3 / rate) if rate > 0 else 1.5
        return (
            0 <= now - self.received <= timeout and 0 <= ros_now_ns - self.stamp_ns <= timeout * 1e9
        )

    @property
    def valid(self):
        return self.values.get("sample_valid", "").lower() == "true"


class MotorMonitor:
    def __init__(self, node, *, clock=monotonic):
        self.node, self.clock = node, clock
        self.records = {}
        self.subscription = node.create_subscription(
            DiagnosticArray, "/follower/motor_states", self.receive, 10
        )

    def receive(self, msg):
        stamp = stamp_ns(msg.header.stamp)
        if not 0 < stamp <= self.node.get_clock().now().nanoseconds:
            return
        for status in msg.status:
            name = status.name.rsplit("/motor/", 1)[-1]
            if name not in JOINTS:
                continue
            previous = self.records.get(name)
            if previous is not None and stamp <= previous.stamp_ns:
                continue
            level = status.level
            if isinstance(level, bytes):
                level = int.from_bytes(level, "little")
            self.records[name] = MotorRecord(
                {v.key: v.value for v in status.values},
                level,
                status.message,
                stamp,
                self.clock(),
            )

    def fresh(self, record):
        return record.fresh(self.node.get_clock().now().nanoseconds, self.clock())
