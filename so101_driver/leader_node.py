"""Publish calibrated leader positions without hardware writes."""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from .command_mapping import MIN_PUBLISH_RATE_HZ
from .hardware.devices import resolve_device_id
from .hardware.feetech_bus import BusError, FeetechBus
from .messages import hardware_diagnostics
from .parameters import declare_number
from .paths import config_path
from .profile import JOINTS
from .so101_leader import SO101Leader
from .startup import run_node


class LeaderNode(Node):
    def __init__(self, **kwargs):
        super().__init__("so101_leader", **kwargs)
        self.declare_parameter("device_id", "")
        self.declare_parameter("calibration_profile", "")
        rate = declare_number(self, "publish_rate_hz", 30.0)
        self.device_id = self.get_parameter("device_id").value
        self.bus = None
        try:
            if not self.device_id.strip():
                raise ValueError(
                    f"device_id is not set; write it in {config_path('leader')} "
                    "or pass another YAML with params_file"
                )
            if not self.get_parameter("calibration_profile").value:
                raise ValueError("Set calibration_profile")
            if not MIN_PUBLISH_RATE_HZ <= rate <= 100:
                raise ValueError(f"publish_rate_hz must be in [{MIN_PUBLISH_RATE_HZ:g}, 100]")
            self.bus = FeetechBus(resolve_device_id(self.device_id))
            profile = self.get_parameter("calibration_profile").value
            self.leader = SO101Leader(self.bus, profile)
            self.bus.open()
            self.leader.validate()
            self.joints = self.create_publisher(JointState, "joint_states", qos_profile_sensor_data)
            self.diagnostics = self.create_publisher(DiagnosticArray, "diagnostics", 10)
            self.failed_samples = 0
            self.consecutive_failures = 0
            self.timer = self.create_timer(1 / rate, self.publish_state)
        except Exception:
            if self.bus is not None:
                self.bus.close()
            raise

    def publish_state(self):
        try:
            positions = self.leader.read_joint_positions()
        except (BusError, OSError) as exc:
            self.failed_samples += 1
            self.consecutive_failures += 1
            # Do not publish a partial sample, stale values, or fabricated zeros.
            if self.consecutive_failures == 1 or self.consecutive_failures % 30 == 0:
                self.get_logger().warning(f"State sample dropped: {exc}")
            self.publish_health(DiagnosticStatus.ERROR, str(exc))
            return
        if self.consecutive_failures:
            self.get_logger().info(
                f"State reading recovered after {self.consecutive_failures} failed samples"
            )
        self.consecutive_failures = 0
        self.publish_health(DiagnosticStatus.OK, "Reading positions")
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = positions
        self.joints.publish(msg)

    def publish_health(self, level, message):
        values = (
            ("failed_samples", self.failed_samples),
            ("consecutive_failures", self.consecutive_failures),
        )
        self.diagnostics.publish(hardware_diagnostics(self, self.device_id, level, message, values))

    def destroy_node(self):
        self.bus.close()
        return super().destroy_node()


def main(args=None):
    return run_node("so101_leader", LeaderNode, args)
