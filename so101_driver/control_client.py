"""ROS client shared by the slider GUI and teleoperation command source."""

import math
from time import monotonic

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from .command_mapping import STATE_TIMEOUT_S, JointSnapshot
from .messages import stamp_ns
from .profile import JOINTS
from .so101_follower import ANGLE_EPSILON


class FollowerClient:
    def __init__(self, node, limits, *, clock=monotonic):
        self.node = node
        self.limits = limits
        self.clock = clock
        self.state = JointSnapshot()
        self.target = None
        self.active = False
        self.message = "Waiting for follower state"
        self.error_message = ""
        self.writes_allowed = False
        self.control_active = False
        self.diagnostic_time = 0.0
        self.diagnostic_level = DiagnosticStatus.STALE
        self.diagnostic_message = ""
        self.hardware_id = ""
        self.hardware_details = {}
        self.enabled_stamp_ns = 0
        self.diagnostic_stamp_ns = 0
        self.pending = None
        self.pending_result = None
        self.pending_at = 0.0
        self.stop_requested = False
        self.release_requested = False
        self.releasing = False
        self.torque_released = False
        self.release_started_at = 0.0
        self.publisher = node.create_publisher(
            JointState, "/follower/joint_command", QoSProfile(depth=1)
        )
        self.state_subscription = node.create_subscription(
            JointState, "/follower/joint_states", self.receive_state, qos_profile_sensor_data
        )
        self.diagnostic_subscription = node.create_subscription(
            DiagnosticArray, "/follower/diagnostics", self.receive_diagnostics, 10
        )
        self.service = node.create_client(SetBool, "/follower/enable_control")
        self.release_service = node.create_client(Trigger, "/follower/release_torque")

    def receive_state(self, msg):
        self.state.update(
            msg.name,
            msg.position,
            stamp_ns(msg.header.stamp),
            ros_now_ns=self.node.get_clock().now().nanoseconds,
            now=self.clock(),
        )

    def receive_diagnostics(self, msg):
        for status in msg.status:
            if not status.name.endswith("/hardware"):
                continue
            values = {v.key: v.value for v in status.values}
            self.hardware_id = status.hardware_id
            self.hardware_details = values
            self.writes_allowed = values.get("allow_writes", "false").lower() == "true"
            self.control_active = values.get("control_active", "false").lower() == "true"
            self.diagnostic_level = status.level
            self.diagnostic_message = status.message
            self.diagnostic_time = self.clock()
            self.diagnostic_stamp_ns = stamp_ns(msg.header.stamp)

    def fresh(self):
        return self.state.fresh(
            ros_now_ns=self.node.get_clock().now().nanoseconds, now=self.clock()
        )

    def readiness_reason(self, *, require_in_range=True):
        if self.pending is not None:
            return "Waiting for the follower to confirm control start"
        if self.release_requested:
            return "Torque was released; restart to control again"
        if not self.fresh():
            return "Waiting for follower joint states (/follower/joint_states)"
        if not self.service.service_is_ready():
            return "Waiting for the follower control service (/follower/enable_control)"
        if self.clock() - self.diagnostic_time > STATE_TIMEOUT_S:
            return "Waiting for follower diagnostics (/follower/diagnostics)"
        if not self.writes_allowed:
            return "Follower control is not allowed (allow_writes=false)"
        if self.diagnostic_level >= DiagnosticStatus.ERROR:
            return "Check the follower: " + self.diagnostic_message
        if require_in_range and not self.in_range():
            return self.range_error()
        return ""

    def ready(self, *, require_in_range=True):
        return not self.readiness_reason(require_in_range=require_in_range)

    def in_range(self):
        return self.state.positions is not None and not self.range_violations()

    def range_violations(self):
        if self.state.positions is None:
            return []
        return [
            (name, value, lo, hi)
            for name, value, (lo, hi) in zip(JOINTS, self.state.positions, self.limits)
            if not lo - ANGLE_EPSILON <= value <= hi + ANGLE_EPSILON
        ]

    def range_error(self):
        details = [
            f"{name}: {math.degrees(value):.1f}° "
            f"(range {math.degrees(lo):.1f} to {math.degrees(hi):.1f}°)"
            for name, value, lo, hi in self.range_violations()
        ]
        return "Current pose is outside the command range\n" + "\n".join(details)

    @staticmethod
    def completed(success, message):
        future = Future()
        future.set_result(SetBool.Response(success=success, message=message))
        return future

    def start(self, *, target=None):
        """Hold the current pose, or start towards an explicitly supplied in-range target."""
        if self.active:
            return self.completed(True, "Already in control")
        self.error_message = ""
        if target is not None and (
            len(target) != 6
            or not all(
                math.isfinite(value) and lo - ANGLE_EPSILON <= value <= hi + ANGLE_EPSILON
                for value, (lo, hi) in zip(target, self.limits)
            )
        ):
            self.message = "The start target needs six angles within the command range"
            return self.completed(False, self.message)
        reason = self.readiness_reason(require_in_range=target is None)
        if reason:
            self.message = reason
            return self.completed(False, self.message)
        # The driver verifies the measured holding pose against actual servo limits.
        # An explicit target lets teleop recover from a pose just outside observed travel.
        self.target = list(self.state.positions if target is None else target)
        self.stop_requested = False
        self.pending_at = self.clock()
        self.message = "Starting control"
        self.pending_result = Future()
        self.pending = self.service.call_async(SetBool.Request(data=True))
        self.pending.add_done_callback(self._started)
        return self.pending_result

    def _started(self, future):
        self.pending = None
        result = self.pending_result
        try:
            response = future.result()
        except Exception as exc:
            response = SetBool.Response(success=False, message=str(exc))
        if response.success and (self.stop_requested or result.done()):
            reason = self.error_message or "Start canceled"
            self.stop(reason, failed=bool(self.error_message))
            response = SetBool.Response(success=False, message=reason)
        elif response.success:
            self.active = True
            self.enabled_stamp_ns = self.node.get_clock().now().nanoseconds
            self.message = "In control"
        else:
            self.error_message = self.message = "Failed to start control: " + response.message
            self.node.get_logger().warning(self.message)
        if not result.done():
            result.set_result(response)

    def stop(self, reason="Stopped", *, failed=False):
        self.active = False
        self.stop_requested = True
        self.message = reason
        self.error_message = reason if failed else ""
        if not self.service.service_is_ready():
            self.error_message = self.message = "Cannot send the hold command"
            return self.completed(False, self.message)
        future = self.service.call_async(SetBool.Request(data=False))
        future.add_done_callback(self._stopped)
        return future

    def _stopped(self, future):
        try:
            response = future.result()
            if not response.success:
                self.error_message = self.message = response.message
        except Exception as exc:
            self.error_message = self.message = str(exc)

    def release_torque(self):
        if self.pending is not None:
            return self.completed(False, "Wait for control start to finish before releasing torque")
        if self.releasing:
            return self.completed(False, "Confirming torque release")
        self.active = False
        self.target = None
        self.stop_requested = True
        self.release_requested = True
        self.error_message = ""
        if not self.release_service.service_is_ready():
            self.error_message = self.message = "Cannot reach the torque release service"
            return self.completed(False, self.message)
        self.releasing = True
        self.release_started_at = self.clock()
        self.message = "Releasing torque"
        future = self.release_service.call_async(Trigger.Request())
        future.add_done_callback(self._released)
        return future

    def _released(self, future):
        self.releasing = False
        try:
            response = future.result()
            self.torque_released = response.success
            self.message = response.message
            self.error_message = "" if response.success else response.message
        except Exception as exc:
            self.error_message = self.message = f"Torque release not confirmed: {exc}"

    def tick(self):
        now = self.clock()
        if self.releasing and now - self.release_started_at > 5.0:
            self.error_message = self.message = (
                "No response to torque release; release not confirmed"
            )
        if self.pending is not None and now - self.pending_at > 2.0:
            self.stop_requested = True
            self.message = "Control start timed out"
            self.error_message = self.message
            if not self.pending_result.done():
                self.pending_result.set_result(
                    SetBool.Response(success=False, message=self.message)
                )
        if not self.active:
            return
        if not self.fresh() or now - self.diagnostic_time > STATE_TIMEOUT_S:
            self.stop("Lost follower state", failed=True)
            return
        if self.diagnostic_stamp_ns > self.enabled_stamp_ns and (
            not self.control_active or self.diagnostic_level >= DiagnosticStatus.ERROR
        ):
            detail = self.diagnostic_message
            reason = "Follower control stopped" + (f": {detail}" if detail else "")
            self.stop(reason, failed=True)
            return
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = list(self.target)
        self.publisher.publish(msg)
