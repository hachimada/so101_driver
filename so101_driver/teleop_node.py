"""Align the follower, then follow absolute leader angles through the ROS interface."""

import math
from time import monotonic

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from .command_mapping import AbsoluteTeleop, JointSnapshot, load_command_limits
from .control_client import FollowerClient
from .messages import stamp_ns
from .parameters import declare_number
from .profile import JOINTS
from .startup import run_node


class TeleopNode(Node):
    def __init__(self, **kwargs):
        super().__init__("so101_teleop", **kwargs)
        self.declare_parameter("follower_config", "")
        self.declare_parameter("autostart", True)
        scales = declare_number(self, "joint_scales", [1.0] * 6)
        for name, default in (
            ("alignment_tolerance_rad", 0.05),
            ("alignment_settle_time_s", 0.2),
            ("alignment_timeout_s", 30.0),
        ):
            value = declare_number(self, name, default)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
            setattr(self, name, value)
        if self.alignment_settle_time_s >= self.alignment_timeout_s:
            raise ValueError("alignment_settle_time_s must be less than alignment_timeout_s")
        limits = load_command_limits(self.get_parameter("follower_config").value)
        self.control = FollowerClient(self, limits)
        self.leader = JointSnapshot()
        self.mapping = AbsoluteTeleop(limits, scales)
        self.phase = "stopped"
        self.start_result = None
        self.arm_result = None
        self.stop_result = None
        self.alignment_started = 0.0
        self.aligned_since = None
        self.alignment_stamps = (0, 0)
        self.leader_subscription = self.create_subscription(
            JointState, "/leader/joint_states", self.receive_leader, qos_profile_sensor_data
        )
        self.enable_service = self.create_service(
            SetBool, "~/enable", self.enable, callback_group=ReentrantCallbackGroup()
        )
        self.autostart_attempted = not self.get_parameter("autostart").value
        self.last_message = ""
        self.timer = self.create_timer(1 / 30, self.update)

    def receive_leader(self, msg):
        self.leader.update(
            msg.name,
            msg.position,
            stamp_ns(msg.header.stamp),
            ros_now_ns=self.get_clock().now().nanoseconds,
            now=monotonic(),
        )

    def leader_fresh(self):
        return self.leader.fresh(ros_now_ns=self.get_clock().now().nanoseconds, now=monotonic())

    def start(self):
        if self.phase in ("arming", "aligning"):
            return self.start_result
        if self.phase == "tracking":
            return self.control.completed(True, "Teleoperation is running")
        if self.control.pending is not None or (
            self.stop_result is not None and not self.stop_result.done()
        ):
            return self.control.completed(False, "Wait for the current start or stop to finish")
        if not self.leader_fresh():
            return self.control.completed(False, "Leader state is not available")
        try:
            target = self.mapping.target(
                self.leader.positions,
                require_in_range=True,
                range_tolerance_rad=self.alignment_tolerance_rad,
            )
        except ValueError as exc:
            self.control.error_message = self.control.message = str(exc)
            return self.control.completed(False, str(exc))
        self.phase = "arming"
        self.start_result = Future()
        self.arm_result = self.control.start(target=target)
        return self.start_result

    def finish_start(self, success, message):
        if self.start_result is not None and not self.start_result.done():
            self.start_result.set_result(SetBool.Response(success=success, message=message))

    def stop(self, message="Teleoperation stopped", *, failed=False):
        self.phase = "stopped"
        self.aligned_since = None
        self.finish_start(False, message)
        self.stop_result = self.control.stop(message, failed=failed)
        return self.stop_result

    async def enable(self, request, response):
        self.autostart_attempted = True
        future = self.start() if request.data else self.stop()
        try:
            result = await future
            response.success, response.message = result.success, result.message
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def update(self):
        if not self.autostart_attempted:
            reason = (
                self.control.readiness_reason(require_in_range=False)
                if self.leader_fresh()
                else "Waiting for leader joint states (/leader/joint_states)"
            )
            if reason:
                self.control.message = reason
            else:
                self.autostart_attempted = True
                self.start()
        if self.phase != "stopped":
            if not self.leader_fresh():
                self.stop("Lost leader state", failed=True)
            elif self.phase == "arming" and self.arm_result.done():
                result = self.arm_result.result()
                if result.success and self.control.active:
                    self.phase = "aligning"
                    self.alignment_started = monotonic()
                    self.aligned_since = None
                    self.alignment_stamps = (self.leader.stamp_ns, self.control.state.stamp_ns)
                    self.control.message = "Aligning: moving the follower to the leader pose"
                else:
                    self.phase = "stopped"
                    self.finish_start(False, result.message)
            if self.phase in ("aligning", "tracking"):
                try:
                    self.control.target = self.mapping.target(
                        self.leader.positions,
                        require_in_range=self.phase == "aligning",
                        range_tolerance_rad=self.alignment_tolerance_rad,
                    )
                except ValueError as exc:
                    self.stop(str(exc), failed=True)
        self.control.tick()
        if self.phase in ("aligning", "tracking"):
            if not self.control.active:
                self.phase = "stopped"
                self.finish_start(False, self.control.message)
            elif self.phase == "aligning":
                self.check_alignment()
        if self.control.message != self.last_message:
            self.last_message = self.control.message
            self.get_logger().info(self.last_message)

    def check_alignment(self):
        now = monotonic()
        errors = [
            abs(target - actual)
            # Compare with the actual leader angles, not the clipped servo command.
            for target, actual in zip(
                self.mapping.leader_angles(self.leader.positions), self.control.state.positions
            )
        ]
        worst = max(range(6), key=errors.__getitem__)
        if now - self.alignment_started >= self.alignment_timeout_s:
            self.stop(
                f"Alignment timed out: {JOINTS[worst]} is off by "
                f"{math.degrees(errors[worst]):.1f}° "
                f"(tolerance {math.degrees(self.alignment_tolerance_rad):.1f}°)",
                failed=True,
            )
            return
        if errors[worst] > self.alignment_tolerance_rad:
            self.aligned_since = None
            return
        stamps = (self.leader.stamp_ns, self.control.state.stamp_ns)
        # Require new feedback from both arms; repeated snapshots cannot complete alignment.
        if (
            any(stamp <= previous for stamp, previous in zip(stamps, self.alignment_stamps))
            or self.control.state.stamp_ns <= self.control.enabled_stamp_ns
        ):
            return
        self.alignment_stamps = stamps
        if self.aligned_since is None:
            self.aligned_since = now
        elif now - self.aligned_since >= self.alignment_settle_time_s:
            self.phase = "tracking"
            self.control.message = "Aligned; teleoperation running"
            self.finish_start(True, self.control.message)

    def destroy_node(self):
        if rclpy.ok() and (self.control.active or self.control.pending is not None):
            self.stop("Teleoperation ended")
        return super().destroy_node()


def main(args=None):
    return run_node("so101_teleop", TeleopNode, args)
