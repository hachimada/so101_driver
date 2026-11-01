"""Follower state and explicitly enabled streaming commands."""

import math
import sys

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from .command_mapping import MIN_PUBLISH_RATE_HZ
from .follower_control import FollowerControl
from .hardware.devices import resolve_device_id
from .hardware.feetech_bus import BusError, FeetechBus
from .messages import hardware_diagnostics, stamp_ns
from .motor_telemetry import MotorTelemetryPublisher
from .parameters import declare_number
from .paths import config_path
from .profile import JOINTS
from .so101_follower import SO101Follower
from .startup import run_node


class FollowerNode(Node):
    def __init__(self, **kwargs):
        super().__init__("so101_follower", **kwargs)
        for name, default in (
            ("device_id", ""),
            ("calibration_profile", ""),
            ("allow_writes", False),
            ("release_torque_on_shutdown", False),
        ):
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))
        numbers = {
            name: declare_number(self, name, default)
            for name, default in (
                ("publish_rate_hz", 30.0),
                ("motor_state_rate_hz", 2.0),
                ("command_timeout_s", 0.5),
                ("max_velocity_rad_s", 1.57),
                ("servo_acceleration_raw", 20),
                ("gripper_torque_limit_pct", 20.0),
            )
        }
        self.motor_state_rate_hz = numbers["motor_state_rate_hz"]
        self.device_id = self.get_parameter("device_id").value
        self.bus = None
        self.control = None
        try:
            if not self.device_id.strip():
                raise ValueError(
                    f"device_id is not set; write it in {config_path('follower')} "
                    "or pass another YAML with params_file"
                )
            if not self.get_parameter("calibration_profile").value:
                raise ValueError("Set calibration_profile")
            rate = numbers["publish_rate_hz"]
            if not MIN_PUBLISH_RATE_HZ <= rate <= 100:
                raise ValueError(f"publish_rate_hz must be in [{MIN_PUBLISH_RATE_HZ:g}, 100]")
            motor_rate = self.motor_state_rate_hz
            if not math.isfinite(motor_rate) or not 0 <= motor_rate <= 10:
                raise ValueError("motor_state_rate_hz must be in [0, 10]")
            allow_writes = self.get_parameter("allow_writes").value
            if allow_writes and self.get_parameter("use_sim_time").value:
                raise ValueError("Hardware writes require use_sim_time=false")
            if allow_writes:
                self.set_descriptor(
                    "use_sim_time",
                    ParameterDescriptor(
                        type=self.describe_parameter("use_sim_time").type, read_only=True
                    ),
                )
            self.bus = FeetechBus(resolve_device_id(self.device_id))
            self.robot = SO101Follower(
                self.bus,
                self.get_parameter("calibration_profile").value,
            )
            self.get_logger().info("Calibration: " + str(self.robot.profile.path))
            self.control = FollowerControl(
                self.robot,
                allow_writes=allow_writes,
                command_timeout_s=numbers["command_timeout_s"],
                max_velocity_rad_s=numbers["max_velocity_rad_s"],
                servo_acceleration_raw=numbers["servo_acceleration_raw"],
                gripper_torque_limit_pct=numbers["gripper_torque_limit_pct"],
            )
            self.bus.open()
            self.robot.validate()
            self.joints = self.create_publisher(JointState, "joint_states", qos_profile_sensor_data)
            self.diagnostics = self.create_publisher(DiagnosticArray, "diagnostics", 10)
            self.commands = self.create_subscription(
                JointState,
                "joint_command",
                self.receive_command,
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
            self.enable_service = self.create_service(
                SetBool, "enable_control", self.enable_control
            )
            self.release_service = self.create_service(
                Trigger, "release_torque", self.release_torque
            )
            self.failed_samples = 0
            self.rejected_commands = 0
            self.last_error = ""
            self.timer = self.create_timer(1 / rate, self.publish_state)
            self.motor_telemetry = (
                MotorTelemetryPublisher(self, self.bus, motor_rate) if motor_rate else None
            )
            self.get_logger().info(self.control.message)
        except Exception:
            if self.bus is not None:
                self.bus.close()
            raise

    def enable_control(self, request, response):
        try:
            if request.data:
                self.control.enable(self.get_clock().now().nanoseconds)
            elif self.control.engaged:
                self.control.stop()
            response.success = True
            response.message = self.control.message
        except (BusError, OSError, ValueError) as exc:
            self.last_error = str(exc)
            self.get_logger().warning(f"Control request rejected: {exc}")
            response.success = False
            response.message = str(exc)
        return response

    def receive_command(self, msg):
        try:
            if len(msg.name) != 6 or set(msg.name) != set(JOINTS) or len(msg.position) != 6:
                raise ValueError("Command must name each of the six joints exactly once")
            if msg.velocity or msg.effort or msg.header.frame_id:
                raise ValueError(
                    "Only positions are supported; velocity, effort and frame_id must be empty"
                )
            positions = dict(zip(msg.name, msg.position))
            self.control.submit(
                [positions[name] for name in JOINTS],
                stamp_ns=stamp_ns(msg.header.stamp),
                ros_now_ns=self.get_clock().now().nanoseconds,
            )
        except (BusError, OSError, ValueError) as exc:
            self.rejected_commands += 1
            if self.rejected_commands == 1 or self.rejected_commands % 30 == 0:
                self.get_logger().warning(f"Command rejected: {exc}")

    def release_torque(self, request, response):
        try:
            self.control.release_torque()
            response.success = True
            response.message = self.control.message
            self.get_logger().info(response.message)
        except (BusError, OSError, ValueError) as exc:
            self.last_error = str(exc)
            response.success = False
            response.message = str(exc)
            self.get_logger().warning(response.message)
        return response

    def publish_state(self):
        try:
            raw = self.robot.read_raw()
        except (BusError, OSError, ValueError) as exc:
            self.failed_samples += 1
            self.last_error = str(exc)
            if self.control.active:
                self.hold_after_error(str(exc))
            self.publish_health(DiagnosticStatus.ERROR, self.last_error)
            return
        positions = self.robot.to_radians(raw)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = positions
        self.joints.publish(msg)
        try:
            self.control.update(raw)
        except (BusError, OSError, ValueError) as exc:
            if self.control.active:
                self.hold_after_error(str(exc))
            self.last_error = str(exc)
            self.publish_health(DiagnosticStatus.ERROR, self.last_error)
            return
        self.last_error = ""
        level = (
            DiagnosticStatus.ERROR
            if self.control.hold_failed or self.control.release_failed
            else DiagnosticStatus.WARN
            if self.control.engaged and not self.control.active
            else DiagnosticStatus.OK
        )
        self.publish_health(level, self.control.message)

    def hold_after_error(self, reason):
        try:
            self.control.stop(reason)
        except (BusError, OSError, ValueError) as exc:
            self.get_logger().error(f"Hold not confirmed: {exc}")

    def publish_health(self, level, message):
        values = (
            ("allow_writes", self.control.allow_writes),
            ("control_active", self.control.active),
            ("control_status", self.control.message),
            ("failed_samples", self.failed_samples),
            ("rejected_commands", self.rejected_commands),
            ("motor_state_rate_hz", self.motor_state_rate_hz),
        )
        self.diagnostics.publish(hardware_diagnostics(self, self.device_id, level, message, values))

    def destroy_node(self):
        try:
            if self.control is not None:
                if (
                    self.get_parameter("release_torque_on_shutdown").value
                    and self.control.allow_writes
                ):
                    # Release directly over serial, even after the ROS context stops.
                    try:
                        self.control.release_torque()
                    except (BusError, OSError, ValueError) as exc:
                        print(
                            f"Torque release at shutdown not confirmed: {exc}",
                            file=sys.stderr,
                            flush=True,
                        )
                    else:
                        # rosout is unavailable once the ROS context has shut down.
                        print(self.control.message, flush=True)
                elif self.control.active:
                    self.hold_after_error("Node stopping; holding position")
        finally:
            self.bus.close()
        return super().destroy_node()


def main(args=None):
    return run_node("so101_follower", FollowerNode, args)
