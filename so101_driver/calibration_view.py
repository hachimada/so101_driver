"""ROS preview and terminal readings for a calibration session."""

import os
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
from contextlib import contextmanager
from pathlib import Path

import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

from .description import display_description
from .hardware.feetech_bus import BusError
from .profile import (
    GRIPPER_CLOSED_RAD,
    JOINTS,
    MIN_TRAVEL_TICKS,
    MOTOR_IDS,
    RADIANS_PER_TICK,
    gripper_conversion,
)


class CalibrationView(Node):
    def __init__(self, bus, session, terminal=None):
        super().__init__("so101_calibration")
        self.bus, self.session = bus, session
        self.terminal = terminal
        self.live = self.create_publisher(
            JointState, "/calibration/live/joint_states", qos_profile_sensor_data
        )
        self.target = self.create_publisher(
            JointState, "/calibration/target/joint_states", qos_profile_sensor_data
        )
        description_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.description = None
        self.live_visible = False
        self.visible_description = self.create_publisher(
            String, "/calibration/live/preview_robot_description", description_qos
        )
        self.description_subscription = self.create_subscription(
            String,
            "/calibration/live/robot_description",
            self.receive_description,
            description_qos,
        )
        self.static = StaticTransformBroadcaster(self)
        self.transforms = []
        for label, x in (("live", -0.25), ("target", 0.25)):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "calibration/world"
            t.child_frame_id = f"calibration/{label}/base_link"
            t.transform.translation.x = x
            t.transform.rotation.w = 1.0
            self.transforms.append(t)
        # Connect only the reference model until a measured zero pose is available.
        # The description is also withheld: missing TF alone renders error geometry.
        self.static.sendTransform(self.transforms[1:])
        self.last_warning = 0.0
        self.timer = self.create_timer(0.1, self.update)

    def receive_description(self, msg):
        self.description = msg
        if self.live_visible:
            self.visible_description.publish(msg)

    def joint_message(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = positions
        return msg

    def update(self):
        s = self.session
        try:
            s.accept(self.bus.read_positions(MOTOR_IDS).positions_ticks)
        except (BusError, OSError, ValueError) as exc:
            s.fail(str(exc))
            if self.terminal is None and time.monotonic() - self.last_warning > 2:
                self.get_logger().warning(f"Calibration sample discarded: {exc}")
                self.last_warning = time.monotonic()
        with s.lock:
            valid = s.latest is not None and not s.error
            raw = s.latest or {}
            if valid and s.zero is not None:
                positions = [(raw[i] - s.zero[i]) * RADIANS_PER_TICK for i in MOTOR_IDS[:5]]
                endpoint = s.opened
                if endpoint is None and 6 in s.ranges:
                    r = s.ranges[6]
                    endpoint = max((r["min"], r["max"]), key=lambda value: abs(value - s.closed))
                if endpoint is not None and abs(endpoint - s.closed) >= MIN_TRAVEL_TICKS:
                    gripper = gripper_conversion(s.closed, endpoint)
                    positions.append(gripper.to_clamped_radians(raw[6]))
                else:
                    positions.append(GRIPPER_CLOSED_RAD)
                self.live.publish(self.joint_message(positions))
                if not self.live_visible:
                    self.static.sendTransform(self.transforms)
                    self.live_visible = True
                    if self.description is not None:
                        self.visible_description.publish(self.description)
            target = [0.0] * 5 + [GRIPPER_CLOSED_RAD]
            self.target.publish(self.joint_message(target))
            readings = []
            if s.ranges:
                readings.append(f"{'Joint':18} {'Min':>6} {'Max':>6} (ticks)")
                for i, name in enumerate(JOINTS, 1):
                    r = s.ranges[i]
                    readings.append(f"{name:18} {r['min']:6d} {r['max']:6d}")
            if s.error:
                readings.append("Communication error (sample not recorded): " + s.error)
            if self.terminal is not None:
                self.terminal.update(readings)


def _stop_children(children):
    for child in children:
        if child.poll() is None:
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
    for child in children:
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGTERM)
            child.wait(timeout=3)


@contextmanager
def calibration_preview(share, *, show_rviz=True):
    """Own preview processes and their temporary configurations as one resource."""
    if not show_rviz:
        # Only RViz reads the robot_state_publisher output, so start nothing. A stopped
        # preview process aborts calibration, which would then depend on unused processes.
        yield []
        return
    children = []
    with tempfile.TemporaryDirectory(prefix="so101-calibration-") as directory:
        try:
            description = display_description(
                (share / "description/so101/so101_new_calib.urdf").read_text()
            )
            for label in ("live", "target"):
                model = ET.fromstring(description)
                if label == "target":
                    for color in model.findall(".//material/color"):
                        color.set("rgba", "0.25 0.65 1.0 1.0")
                params = Path(directory) / f"{label}.yaml"
                params.write_text(
                    yaml.safe_dump(
                        {
                            "/**": {
                                "ros__parameters": {
                                    "robot_description": ET.tostring(model, encoding="unicode"),
                                    "frame_prefix": f"calibration/{label}/",
                                }
                            }
                        }
                    )
                )
                # Keep the wizard terminal clear; preserve subprocess diagnostics in files.
                log = open(Path(directory) / f"{label}.log", "w")
                try:
                    children.append(
                        subprocess.Popen(
                            [
                                "ros2",
                                "run",
                                "robot_state_publisher",
                                "robot_state_publisher",
                                "--ros-args",
                                "-r",
                                f"__ns:=/calibration/{label}",
                                "--params-file",
                                str(params),
                            ],
                            stdout=log,
                            stderr=subprocess.STDOUT,
                            start_new_session=True,
                        )
                    )
                finally:
                    log.close()
            with open(Path(directory) / "rviz.log", "w") as log:
                children.append(
                    subprocess.Popen(
                        [
                            "rviz2",
                            "-d",
                            str(share / "config/calibration.rviz"),
                            "-stylesheet",
                            str(share / "config/confirmation.qss"),
                        ],
                        stdout=log,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                    )
                )
            yield children
        finally:
            _stop_children(children)
