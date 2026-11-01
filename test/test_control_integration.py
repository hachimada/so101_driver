"""Real ROS services/topics and offscreen Qt, with a simulated motor bus only."""

import json
import os
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    import rclpy
    import yaml
    from PyQt5 import QtWidgets
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_srvs.srv import SetBool

    from so101_driver.follower_control import FollowerControl
    from so101_driver.follower_node import FollowerNode
    from so101_driver.profile import JOINTS
    from so101_driver.slider_gui import SliderNode, SliderWindow
    from so101_driver.so101_follower import SO101Follower
    from so101_driver.teleop_node import TeleopNode

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

# 1.57 rad/s: 2*pi/4096 rad/tick on the arm, 1.92 rad over 2000 ticks on the gripper.
SPEEDS = {i: 1023 for i in range(1, 6)} | {6: 1635}


class SimulatedBus:
    port_name = "/simulated"

    def __init__(self):
        self.raw = {i: 2000 for i in range(1, 7)}
        self.torque = False
        self.torque_limits = {i: 1000 for i in range(1, 7)}
        self.writes = []
        self.write_settings = []
        self.follow_positions = True
        self.motor_fault = 0
        self.telemetry_failure = False

    def read_motor_info(self, ids):
        return {
            i: dict(
                model_number=777,
                firmware_version="3.9",
                phase=0,
                hardware_min_ticks=1000,
                hardware_max_ticks=3000,
                temperature_limit_c=70,
                min_voltage_v=4.0,
                max_voltage_v=14.0,
                operating_mode=0,
            )
            for i in ids
        }

    def read_motor_status(self, ids):
        if self.telemetry_failure:
            raise OSError("simulated timeout")
        return {
            i: dict(
                torque_enabled=int(self.torque),
                torque_limit_pct=self.torque_limits[i] / 10,
                moving=0,
                position_ticks=self.raw[i],
                goal_ticks=self.raw[i],
                speed_raw=-12,
                current_ma=130.0,
                load_pct=-25.0,
                voltage_v=12.1,
                temperature_c=35,
                packet_error=self.motor_fault,
                servo_status=0,
            )
            for i in ids
        }

    def open(self):
        pass

    def close(self):
        pass

    def ping(self, ids):
        return {i: 777 for i in ids}

    def read_configuration(self, ids):
        return {
            i: dict(
                torque_enabled=int(self.torque),
                operating_mode=0,
                homing_offset=0,
                range_min=1000,
                range_max=3000,
            )
            for i in ids
        }

    def read_positions(self, ids):
        return SimpleNamespace(positions_ticks={i: self.raw[i] for i in ids})

    def write_positions(self, targets, **kwargs):
        self.writes.append(dict(targets))
        self.write_settings.append(kwargs)
        if self.follow_positions:
            self.raw.update(targets)

    def set_torque_enabled(self, ids, enabled):
        self.torque = enabled

    def set_torque_limit(self, motor_id, limit_raw):
        self.torque_limits[motor_id] = limit_raw

    def read_motion_status(self, ids):
        return {i: {"goal_ticks": self.raw[i], "moving": False} for i in ids}


@unittest.skipUnless(ROS_AVAILABLE, "Requires ROS2 Humble and PyQt5")
class ControlIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        inherited_domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
        domains = [30 + (os.getpid() + i) % 60 for i in range(60)]
        cls.test_domains = iter(domain for domain in domains if domain != inherited_domain)

    def setUp(self):
        # Separate every test from normal robot nodes and other test contexts.
        rclpy.init(domain_id=next(self.test_domains))
        self.nodes = []
        self.windows = []
        self.executor = SingleThreadedExecutor()
        self.directory = tempfile.TemporaryDirectory()
        path = Path(self.directory.name)
        self.home_ticks = {1: 2100, 2: 1900, 3: 2200, 4: 1850, 5: 2050, 6: 1800}
        self.profile_path = path / "profile.json"
        self.profile_path.write_text(
            json.dumps(
                {
                    "format_version": 1,
                    "kind": "so101_software_calibration",
                    "robot_role": "follower",
                    "hardware_configuration": {
                        str(i): dict(
                            homing_offset=0, range_min=1000, range_max=3000, operating_mode=0
                        )
                        for i in range(1, 7)
                    },
                    "joints": {
                        name: dict(
                            id=i,
                            zero_ticks=2000,
                            direction=1,
                            radians_per_tick=2 * 3.141592653589793 / 4096,
                            range_min=1000,
                            range_max=3000,
                        )
                        for i, name in enumerate(JOINTS[:5], 1)
                    },
                    "gripper": dict(
                        id=6,
                        closed_ticks=1000,
                        open_ticks=3000,
                        closed_angle_rad=-0.174533,
                        open_angle_rad=1.74533,
                    ),
                    "home_position": {
                        "positions_ticks": {
                            name: self.home_ticks[i] for i, name in enumerate(JOINTS, 1)
                        }
                    },
                }
            )
        )
        params = dict(
            device_id="simulated-device",
            calibration_profile=str(self.profile_path),
            allow_writes=True,
            max_velocity_rad_s=1.57,
            servo_acceleration_raw=20,
            gripper_torque_limit_pct=18.5,
        )
        self.config = path / "follower.yaml"
        self.config.write_text(
            yaml.safe_dump({"/follower/so101_follower": {"ros__parameters": params}})
        )
        self.bus = SimulatedBus()
        with (
            patch("so101_driver.follower_node.FeetechBus", return_value=self.bus),
            patch("so101_driver.follower_node.resolve_device_id", return_value="/simulated"),
        ):
            self.follower = self.add(
                FollowerNode(
                    namespace="/follower",
                    parameter_overrides=[
                        Parameter("device_id", value=params["device_id"]),
                        Parameter("calibration_profile", value=params["calibration_profile"]),
                        Parameter("allow_writes", value=True),
                        Parameter("max_velocity_rad_s", value=params["max_velocity_rad_s"]),
                        Parameter("servo_acceleration_raw", value=params["servo_acceleration_raw"]),
                        Parameter(
                            "gripper_torque_limit_pct", value=params["gripper_torque_limit_pct"]
                        ),
                    ],
                )
            )
        self.source = None
        self.leader_positions = [0.3] * 6

    def add(self, node):
        self.nodes.append(node)
        self.executor.add_node(node)
        return node

    def pump(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
            if isinstance(self.source, SliderNode):
                self.source.control.tick()
            self.app.processEvents()
            if predicate():
                return
        self.fail("ROS/Qt condition did not complete before timeout")

    def slider(self):
        self.source = self.add(
            SliderNode(parameter_overrides=[Parameter("follower_config", value=str(self.config))])
        )
        window = SliderWindow(self.source)
        self.windows.append(window)
        self.pump(lambda: self.source.control.ready())
        window.refresh()
        return window

    def teleop(self, autostart=False, **parameters):
        self.source = self.add(
            TeleopNode(
                parameter_overrides=[
                    Parameter("follower_config", value=str(self.config)),
                    Parameter("autostart", value=autostart),
                    Parameter("joint_scales", value=[1.0, -1.0, 1.0, 1.0, 1.0, 1.0]),
                    *[Parameter(name, value=value) for name, value in parameters.items()],
                ]
            )
        )
        publisher_node = self.add(rclpy.create_node("simulated_leader"))
        publisher = publisher_node.create_publisher(
            JointState, "/leader/joint_states", qos_profile_sensor_data
        )

        def publish():
            msg = JointState()
            msg.header.stamp = publisher_node.get_clock().now().to_msg()
            msg.name = list(JOINTS)
            msg.position = self.leader_positions
            publisher.publish(msg)

        self.leader_timer = publisher_node.create_timer(1 / 30, publish)
        self.pump(lambda: self.source.leader_fresh() and self.source.control.fresh())
        client = publisher_node.create_client(SetBool, "/so101_teleop/enable")
        self.pump(client.service_is_ready)
        return client

    def tearDown(self):
        for window in self.windows:
            window.close()
        for node in reversed(self.nodes):
            self.executor.remove_node(node)
            node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()
        self.directory.cleanup()

    def test_slider_start_move_and_hold_use_the_ros_interface(self):
        window = self.slider()
        self.assertEqual(len(window.sliders), 6)
        self.assertFalse(self.bus.writes)
        self.assertEqual(self.bus.torque_limits, {i: 1000 for i in range(1, 7)})
        self.assertTrue(window.start_button.isEnabled())
        window.start_button.click()
        self.pump(lambda: self.source.control.active)
        self.assertEqual(
            self.bus.torque_limits, {1: 1000, 2: 1000, 3: 1000, 4: 1000, 5: 1000, 6: 185}
        )
        self.assertEqual(self.bus.writes[0], {i: 2000 for i in range(1, 7)})
        window.refresh()
        self.assertTrue(window.sliders[0].isEnabled())
        self.bus.follow_positions = False
        before = len(self.bus.writes)
        window.sliders[0].setValue(300)  # 30 degrees; exceeds the former tracking-error limit.
        expected = self.follower.robot.to_ticks(self.source.control.target)
        self.pump(lambda: expected in self.bus.writes[before:])
        changed = [goal for goal in self.bus.writes[before:] if goal != self.bus.raw]
        self.assertEqual(changed[0], expected)
        self.assertEqual(self.bus.write_settings[-1], dict(speeds=SPEEDS, acceleration=20))
        self.assertTrue(self.follower.control.active)
        self.assertEqual(self.bus.raw, {i: 2000 for i in range(1, 7)})
        window.stop_button.click()
        self.pump(lambda: not self.follower.control.active)
        self.assertFalse(self.source.control.active)
        self.assertEqual(self.bus.writes[-1], self.bus.raw)
        self.assertTrue(self.bus.torque)  # Hold does not release the arm.

    def test_gui_stops_when_feedback_disappears(self):
        window = self.slider()
        window.start_button.click()
        self.pump(lambda: self.source.control.active)
        self.follower.timer.cancel()
        self.pump(lambda: not self.source.control.active, timeout=2)
        self.pump(lambda: not self.follower.control.active)
        window.refresh()
        self.assertFalse(window.sliders[0].isEnabled())

    def test_motor_dashboard_receives_all_axes_and_tracks_torque_without_extra_writes(self):
        window = self.slider()
        panel = window.motor_panel
        self.pump(lambda: len(panel.monitor.records) == 6)
        window.refresh()
        self.assertEqual(panel.table.item(0, 2).text(), "OFF")
        self.assertEqual(panel.table.item(0, 4).text(), "-8.78")
        self.assertEqual(panel.table.item(0, 5).text(), "130")
        self.assertEqual(panel.table.item(0, 7).text(), "12.1")
        self.assertIn("6 / 6", panel.summary.text())
        self.assertFalse(self.bus.writes)
        self.assertFalse(self.bus.torque)
        window.start_button.click()
        self.pump(lambda: self.source.control.active)
        self.pump(lambda: panel.monitor.records[JOINTS[0]].number("torque_enabled") == 1)
        window.refresh()
        self.assertEqual(panel.table.item(0, 2).text(), "ON")
        panel.table.selectRow(5)
        panel.refresh()
        self.assertEqual(panel.detail_title.text(), "gripper / ID 6")

    def test_motor_dashboard_distinguishes_faults_communication_loss_and_recovery(self):
        window = self.slider()
        panel = window.motor_panel
        self.bus.motor_fault = 5
        self.pump(
            lambda: (
                JOINTS[0] in panel.monitor.records and panel.monitor.records[JOINTS[0]].level == 2
            )
        )
        window.refresh()
        self.assertIn("Voltage", panel.table.item(0, 9).text())
        self.assertIn("Overheat", panel.table.item(0, 9).text())
        self.assertEqual(panel.table.item(0, 7).text(), "12.1")
        self.bus.telemetry_failure = True
        self.pump(lambda: not panel.monitor.records[JOINTS[0]].valid)
        window.refresh()
        self.assertIn("Communication failed", panel.table.item(0, 9).text())
        self.assertEqual(panel.table.item(0, 7).text(), "—")
        self.bus.telemetry_failure = False
        self.bus.motor_fault = 0
        self.pump(
            lambda: (
                panel.monitor.records[JOINTS[0]].valid
                and panel.monitor.records[JOINTS[0]].level == 0
            )
        )
        window.refresh()
        self.assertEqual(panel.table.item(0, 9).text(), "OK")
        self.follower.motor_telemetry.timer.cancel()
        panel.monitor.records[JOINTS[0]].received -= 2.0
        window.refresh()
        self.assertEqual(panel.table.item(0, 9).text(), "Stale")
        self.assertEqual(panel.table.item(0, 5).text(), "—")
        self.assertFalse(self.bus.writes)

    def test_home_button_sends_saved_pose_directly_with_servo_motion_settings(self):
        window = self.slider()
        self.assertFalse(window.home_button.isEnabled())
        window.home_button.click()
        self.assertFalse(self.bus.writes)
        window.start_button.click()
        self.pump(lambda: self.source.control.active)
        window.refresh()
        self.assertEqual(self.bus.raw, {i: 2000 for i in range(1, 7)})
        self.assertTrue(window.home_button.isEnabled())
        self.bus.follow_positions = False
        before = len(self.bus.writes)
        window.home_button.click()
        self.pump(lambda: self.home_ticks in self.bus.writes[before:])
        changed = [goal for goal in self.bus.writes[before:] if goal != self.bus.raw]
        self.assertEqual(changed[0], self.home_ticks)
        self.assertEqual(self.bus.write_settings[-1], dict(speeds=SPEEDS, acceleration=20))
        self.assertTrue(self.follower.control.active)
        self.assertEqual(self.bus.raw, {i: 2000 for i in range(1, 7)})

    def test_missing_home_disables_only_the_home_button(self):
        profile = json.loads(self.profile_path.read_text())
        del profile["home_position"]
        self.profile_path.write_text(json.dumps(profile))
        window = self.slider()
        window.start_button.click()
        self.pump(lambda: self.source.control.active)
        window.refresh()
        self.assertFalse(window.home_button.isEnabled())
        self.assertIn("No home position saved", window.home_button.toolTip())
        self.assertTrue(window.sliders[0].isEnabled())

    def test_set_home_saves_actual_pose_before_control_and_survives_restart(self):
        self.bus.raw = {1: 2150, 2: 1800, 3: 2210, 4: 1770, 5: 2080, 6: 1650}
        window = self.slider()
        original = json.loads(self.profile_path.read_text())
        self.assertTrue(window.set_home_button.isEnabled())
        window.set_home_button.click()
        saved = json.loads(self.profile_path.read_text())
        self.assertEqual(
            saved["home_position"]["positions_ticks"],
            {name: self.bus.raw[i] for i, name in enumerate(JOINTS, 1)},
        )
        self.assertTrue(saved["home_position"]["recorded_at"])
        self.assertEqual(
            {k: v for k, v in saved.items() if k != "home_position"},
            {k: v for k, v in original.items() if k != "home_position"},
        )
        restarted = self.add(
            SliderNode(
                namespace="/restarted",
                parameter_overrides=[Parameter("follower_config", value=str(self.config))],
            )
        )
        self.assertEqual(restarted.home_positions, self.source.home_positions)
        self.assertEqual(restarted.robot.to_ticks(restarted.home_positions), self.bus.raw)
        window.refresh()
        self.assertIn("Saved the current pose as home", window.status.text())
        self.assertFalse(self.source.control.active)
        self.assertFalse(self.bus.torque)
        self.assertFalse(self.bus.writes)

    def test_set_home_during_control_uses_feedback_and_preserves_target(self):
        window = self.slider()
        window.start_button.click()
        self.pump(lambda: self.source.control.active)
        window.refresh()
        window.sliders[0].setValue(200)  # A target still far from the actual pose.
        target = list(self.source.control.target)
        measured = list(self.source.control.state.positions)
        writes_before = len(self.bus.writes)
        self.assertNotEqual(target, measured)
        window.set_home_button.click()
        self.assertEqual(self.source.home_positions, measured)
        self.assertEqual(self.source.control.target, target)
        self.assertEqual(len(self.bus.writes), writes_before)
        self.assertTrue(self.source.control.active)

    def test_set_home_registers_missing_home_and_preserves_it_on_write_failure(self):
        profile = json.loads(self.profile_path.read_text())
        del profile["home_position"]
        self.profile_path.write_text(json.dumps(profile))
        window = self.slider()
        self.assertIsNone(self.source.home_positions)
        window.set_home_button.click()
        self.assertIsNotNone(self.source.home_positions)
        original = self.profile_path.read_bytes()
        home = list(self.source.home_positions)
        self.bus.raw[1] += 100
        self.pump(lambda: self.source.control.state.positions != home)
        with patch("so101_driver.so101_follower.save_profile", side_effect=OSError("read only")):
            window.set_home_button.click()
        self.assertEqual(self.profile_path.read_bytes(), original)
        self.assertEqual(self.source.home_positions, home)
        self.assertIn("read only", window.status.text())
        self.assertTrue(window.set_home_button.isEnabled())
        window.set_home_button.click()
        self.assertIn("Saved the current pose as home", window.status.text())
        window.start_button.click()
        self.pump(lambda: self.source.control.active)
        window.refresh()
        self.assertTrue(window.home_button.isEnabled())

    def test_set_home_rejects_stale_out_of_range_or_changed_calibration(self):
        window = self.slider()
        original = self.profile_path.read_bytes()
        self.source.control.state.received -= 1.0
        window.refresh()
        self.assertFalse(window.set_home_button.isEnabled())
        window.set_home()
        self.assertEqual(self.profile_path.read_bytes(), original)
        self.pump(lambda: self.source.control.fresh())
        self.source.control.state.positions[0] = self.source.control.limits[0][1] + 0.1
        window.refresh()
        self.assertFalse(window.set_home_button.isEnabled())
        window.set_home()
        self.assertEqual(self.profile_path.read_bytes(), original)
        self.pump(lambda: self.source.control.in_range())
        changed = json.loads(original)
        changed["joints"]["shoulder_pan"]["zero_ticks"] += 10
        self.profile_path.write_text(json.dumps(changed))
        window.refresh()
        window.set_home_button.click()
        self.assertIn("restart the GUI", window.status.text())
        self.assertEqual(json.loads(self.profile_path.read_text()), changed)
        self.assertFalse(self.bus.writes)

    def test_gui_rejects_missing_calibration_profile_before_starting_control(self):
        config = yaml.safe_load(self.config.read_text())
        config["/follower/so101_follower"]["ros__parameters"]["calibration_profile"] = ""
        self.config.write_text(yaml.safe_dump(config))
        with self.assertRaisesRegex(ValueError, "calibration_profile"):
            self.slider()
        self.assertFalse(self.bus.writes)

    def test_gui_can_start_after_controller_restart_with_torque_still_enabled(self):
        window = self.slider()
        window.start_button.click()
        self.pump(lambda: self.source.control.active)
        window.stop_button.click()
        self.pump(lambda: not self.follower.control.active)
        self.assertTrue(self.bus.torque)
        self.follower.control = FollowerControl(self.follower.robot, allow_writes=True)
        self.assertFalse(self.follower.control.engaged)
        self.pump(lambda: self.source.control.ready())
        window.refresh()
        window.start_button.click()
        self.pump(lambda: self.source.control.active)
        window.refresh()
        self.assertTrue(window.sliders[0].isEnabled())
        self.assertTrue(self.follower.control.active)

    def test_gui_preserves_start_failure_until_an_explicit_retry(self):
        window = self.slider()
        with patch.object(
            self.follower.control, "enable", side_effect=ValueError("cannot confirm it is holding")
        ):
            window.start_button.click()
            self.pump(lambda: self.source.control.pending is None)
        self.pump(lambda: self.source.control.ready())
        window.refresh()
        self.assertIn("cannot confirm it is holding", window.status.text())
        self.assertFalse(self.source.control.active)
        self.assertTrue(window.start_button.isEnabled())
        window.start_button.click()
        self.pump(lambda: self.source.control.active)
        window.refresh()
        self.assertFalse(self.source.control.error_message)
        self.assertTrue(window.sliders[0].isEnabled())

    def test_release_and_exit_closes_only_after_torque_off_confirmation(self):
        window = self.slider()
        window.show()
        window.start_button.click()
        self.pump(lambda: self.source.control.active)
        window.refresh()
        self.assertTrue(self.bus.torque)
        writes_before = len(self.bus.writes)
        window.release_button.click()
        self.assertTrue(window.isVisible())
        self.assertFalse(self.source.control.active)
        self.pump(lambda: not window.isVisible())
        self.assertFalse(self.bus.torque)
        self.assertTrue(self.source.control.torque_released)
        self.assertFalse(self.follower.control.active)
        self.assertFalse(self.follower.control.engaged)
        self.assertEqual(len(self.bus.writes), writes_before)

    def test_failed_release_keeps_the_window_open_and_can_be_retried(self):
        window = self.slider()
        window.show()
        window.start_button.click()
        self.pump(lambda: self.source.control.active)
        window.refresh()
        with patch.object(
            self.bus, "read_configuration", side_effect=RuntimeError("readback timeout")
        ):
            window.release_button.click()
            self.pump(lambda: window.release_future is None)
        self.assertTrue(window.isVisible())
        self.assertFalse(self.source.control.torque_released)
        self.source.control.state.positions[0] = self.source.control.limits[0][1] + 0.1
        window.refresh()
        self.assertIn("Torque release not confirmed", window.status.text())
        self.assertFalse(window.start_button.isEnabled())
        self.assertTrue(window.release_button.isEnabled())
        window.release_button.click()
        self.pump(lambda: not window.isVisible())
        self.assertTrue(self.source.control.torque_released)

    def test_teleop_service_mapping_stale_leader_and_rearm(self):
        client = self.teleop()
        self.pump(lambda: self.source.control.ready())
        future = client.call_async(SetBool.Request(data=True))
        self.pump(future.done)
        self.assertTrue(future.result().success, future.result().message)
        self.pump(lambda: self.source.control.active)
        self.assertEqual(
            self.bus.torque_limits, {1: 1000, 2: 1000, 3: 1000, 4: 1000, 5: 1000, 6: 185}
        )
        aligned = self.follower.robot.to_ticks([0.3, -0.3, 0.3, 0.3, 0.3, 0.3])
        self.assertEqual(self.source.phase, "tracking")
        self.assertEqual(self.bus.raw, aligned)
        self.bus.follow_positions = False
        before = len(self.bus.writes)
        self.leader_positions = [0.8, 0.8, 0.3, 0.3, 0.3, 0.3]
        target = [0.8, -0.8, 0.3, 0.3, 0.3, 0.3]
        expected = self.follower.robot.to_ticks(target)
        self.pump(lambda: expected in self.bus.writes[before:])
        changed = [goal for goal in self.bus.writes[before:] if goal != self.bus.raw]
        self.assertEqual(changed[0], expected)
        self.assertTrue(self.follower.control.active)
        self.assertEqual(self.bus.raw, aligned)
        self.leader_timer.cancel()
        self.pump(lambda: not self.source.control.active, timeout=2)
        self.pump(lambda: not self.follower.control.active)
        self.assertEqual(self.bus.writes[-1], self.bus.raw)
        self.leader_positions = [0.8] * 6
        self.leader_timer.reset()
        self.pump(lambda: self.source.leader_fresh() and self.source.control.ready())
        self.assertFalse(self.source.control.active)
        self.bus.follow_positions = True
        future = client.call_async(SetBool.Request(data=True))
        self.pump(future.done)
        self.assertTrue(future.result().success)
        self.assertEqual(
            self.bus.raw, self.follower.robot.to_ticks([0.8, -0.8, 0.8, 0.8, 0.8, 0.8])
        )
        future = client.call_async(SetBool.Request(data=False))
        self.pump(future.done)
        self.assertTrue(future.result().success)
        self.assertFalse(self.follower.control.active)

    def test_teleop_autostart_waits_for_both_arms(self):
        self.bus.follow_positions = False
        self.teleop(autostart=True)
        self.pump(lambda: self.source.phase == "aligning")
        self.assertTrue(self.follower.control.active)
        self.assertEqual(self.bus.raw, {i: 2000 for i in range(1, 7)})
        self.assertFalse(self.source.start_result.done())
        self.bus.follow_positions = True
        self.pump(lambda: self.source.phase == "tracking")
        self.assertTrue(self.source.start_result.result().success)
        self.assertEqual(
            self.bus.raw, self.follower.robot.to_ticks([0.3, -0.3, 0.3, 0.3, 0.3, 0.3])
        )

    def test_teleop_alignment_uses_latest_target_and_all_measured_joints(self):
        self.bus.follow_positions = False
        client = self.teleop()
        self.pump(lambda: self.source.control.ready())
        future = client.call_async(SetBool.Request(data=True))
        self.pump(lambda: self.source.phase == "aligning")
        self.leader_positions = [0.6] * 6
        expected = self.follower.robot.to_ticks([0.6, -0.6, 0.6, 0.6, 0.6, 0.6])
        self.pump(lambda: expected in self.bus.writes)
        self.assertFalse(future.done())
        # Five matching joints are insufficient: the gripper must also reach its target.
        self.bus.raw.update({i: expected[i] for i in range(1, 6)})
        deadline = time.monotonic() + 0.3
        self.pump(lambda: time.monotonic() >= deadline)
        self.assertEqual(self.source.phase, "aligning")
        self.assertFalse(future.done())
        self.bus.raw[6] = expected[6]
        self.pump(lambda: self.source.aligned_since is not None)
        self.assertFalse(future.done())
        # A brief match must not skip the settling interval.
        self.bus.raw[6] += 200
        self.pump(lambda: self.source.aligned_since is None)
        self.assertFalse(future.done())
        self.bus.raw.update(expected)
        self.pump(future.done)
        self.assertTrue(future.result().success)
        self.assertEqual(self.source.phase, "tracking")
        self.assertEqual(self.source.control.target, [0.6, -0.6, 0.6, 0.6, 0.6, 0.6])

    def test_teleop_alignment_timeout_holds_and_does_not_restart(self):
        self.bus.follow_positions = False
        client = self.teleop(alignment_timeout_s=0.4)
        self.pump(lambda: self.source.control.ready())
        future = client.call_async(SetBool.Request(data=True))
        self.pump(future.done)
        self.assertFalse(future.result().success)
        self.assertIn("timed out", future.result().message)
        self.pump(lambda: not self.follower.control.active)
        self.assertEqual(self.source.phase, "stopped")
        self.assertEqual(self.bus.writes[-1], self.bus.raw)
        self.assertTrue(self.bus.torque)
        self.bus.follow_positions = True
        self.pump(lambda: self.source.control.ready())
        deadline = time.monotonic() + 0.3
        self.pump(lambda: time.monotonic() >= deadline)
        self.assertFalse(self.source.control.active)

    def test_teleop_stop_cancels_alignment_and_duplicate_start_requests(self):
        self.bus.follow_positions = False
        client = self.teleop()
        self.pump(lambda: self.source.control.ready())
        first = client.call_async(SetBool.Request(data=True))
        self.pump(lambda: self.source.phase == "aligning")
        second = client.call_async(SetBool.Request(data=True))
        # Confirm that an in-progress alignment is not reported as an immediate success.
        deadline = time.monotonic() + 0.15
        self.pump(lambda: time.monotonic() >= deadline)
        self.assertFalse(second.done())
        stopped = client.call_async(SetBool.Request(data=False))
        self.pump(lambda: first.done() and second.done() and stopped.done())
        self.assertFalse(first.result().success)
        self.assertFalse(second.result().success)
        self.assertTrue(stopped.result().success)
        self.assertFalse(self.source.control.active)
        self.assertFalse(self.follower.control.active)
        self.assertEqual(self.bus.writes[-1], self.bus.raw)

    def test_teleop_stop_during_arming_ignores_late_enable_response(self):
        client = self.teleop()
        self.pump(lambda: self.source.control.ready())
        with patch.object(self.source.control.service, "call_async") as call:
            from rclpy.task import Future

            delayed = Future()
            call.return_value = delayed
            started = self.source.start()
        stopped = client.call_async(SetBool.Request(data=False))
        self.pump(stopped.done)
        self.assertTrue(stopped.result().success)
        delayed.set_result(SetBool.Response(success=True, message="enabled"))
        self.pump(lambda: self.source.control.pending is None)
        self.assertFalse(started.result().success)
        self.assertFalse(self.source.control.active)
        self.assertEqual(self.source.phase, "stopped")
        self.assertFalse(self.bus.torque)

    def test_teleop_rejects_unreachable_start_pose_without_motor_writes(self):
        self.leader_positions = [3.0] * 6
        client = self.teleop()
        self.pump(lambda: self.source.control.ready())
        future = client.call_async(SetBool.Request(data=True))
        self.pump(future.done)
        self.assertFalse(future.result().success)
        self.assertIn("shoulder_pan", future.result().message)
        self.assertIn("the follower range", future.result().message)
        self.assertFalse(self.bus.writes)
        self.assertFalse(self.bus.torque)

    def test_teleop_stops_if_alignment_target_becomes_unreachable(self):
        self.bus.follow_positions = False
        client = self.teleop()
        self.pump(lambda: self.source.control.ready())
        future = client.call_async(SetBool.Request(data=True))
        self.pump(lambda: self.source.phase == "aligning")
        self.leader_positions[5] = 3.0
        self.pump(future.done)
        self.assertFalse(future.result().success)
        self.assertIn("gripper", future.result().message)
        self.pump(lambda: not self.follower.control.active)
        self.assertEqual(self.bus.writes[-1], self.bus.raw)

    def test_teleop_alignment_stops_when_leader_feedback_disappears(self):
        self.bus.follow_positions = False
        client = self.teleop()
        self.pump(lambda: self.source.control.ready())
        future = client.call_async(SetBool.Request(data=True))
        self.pump(lambda: self.source.phase == "aligning")
        self.leader_timer.cancel()
        self.pump(future.done)
        self.assertFalse(future.result().success)
        self.assertIn("Lost leader state", future.result().message)
        self.pump(lambda: not self.follower.control.active)

    def test_teleop_alignment_requires_new_feedback_throughout_settling(self):
        client = self.teleop()
        self.pump(lambda: self.source.control.ready())
        future = client.call_async(SetBool.Request(data=True))
        self.pump(lambda: self.source.aligned_since is not None)
        self.source.destroy_subscription(self.source.control.state_subscription)
        self.pump(future.done)
        self.assertFalse(future.result().success)
        self.assertIn("Lost follower state", future.result().message)
        self.pump(lambda: not self.follower.control.active)

    def test_teleop_autostart_recovers_from_just_outside_observed_travel(self):
        # Reproduce the measured failure: actual elbow 9 ticks beyond calibration,
        # inside servo travel, and leader less than one degree beyond that limit.
        profile = json.loads(self.profile_path.read_text())
        profile["joints"]["elbow_flex"]["range_max"] = 2990
        self.profile_path.write_text(json.dumps(profile))
        self.follower.robot = SO101Follower(self.bus, self.profile_path)
        self.follower.control.robot = self.follower.robot
        self.bus.raw[3] = 2999
        initial_raw = self.bus.raw.copy()
        self.leader_positions[2] = self.follower.robot.conversions["elbow_flex"].to_radians(2998.5)
        self.teleop(autostart=True)
        self.pump(lambda: self.source.phase == "tracking")
        self.assertTrue(self.source.start_result.result().success)
        self.assertEqual(self.bus.writes[0], initial_raw)  # Preload the measured hold first.
        self.assertTrue(all(goal[3] <= 2990 for goal in self.bus.writes[1:]))
        self.assertEqual(self.bus.raw[3], 2990)
        self.assertLess(
            abs(self.source.leader.positions[2] - self.source.control.state.positions[2]),
            self.source.alignment_tolerance_rad,
        )

    def test_teleop_near_limit_alignment_checks_unclipped_leader_angle(self):
        self.bus.follow_positions = False
        upper = self.follower.robot.command_limits()[2][1]
        self.leader_positions[2] = upper + 0.04
        client = self.teleop()
        self.pump(lambda: self.source.control.ready())
        future = client.call_async(SetBool.Request(data=True))
        self.pump(lambda: self.source.phase == "aligning")
        commanded = self.follower.robot.to_ticks(self.source.control.target)
        self.bus.raw.update(commanded)
        self.bus.raw[3] -= 13  # About 0.02 rad from command, 0.06 rad from leader.
        deadline = time.monotonic() + 0.3
        self.pump(lambda: time.monotonic() >= deadline)
        self.assertFalse(future.done())
        self.assertEqual(self.source.phase, "aligning")
        self.bus.raw.update(commanded)
        self.pump(future.done)
        self.assertTrue(future.result().success)
        self.assertEqual(self.bus.raw[3], 3000)

    def test_teleop_autostart_reports_hardware_range_failure_without_writes(self):
        self.bus.raw[3] = 3001
        self.teleop(autostart=True)
        self.pump(lambda: self.source.start_result is not None and self.source.start_result.done())
        self.assertFalse(self.source.start_result.result().success)
        self.assertIn("elbow_flex", self.source.start_result.result().message)
        self.assertIn("servo range", self.source.control.message)
        self.assertEqual(self.source.phase, "stopped")
        self.assertFalse(self.bus.writes)
        self.assertFalse(self.bus.torque)

    def test_teleop_accepts_integers_where_decimals_are_expected(self):
        config = Parameter("follower_config", value=str(self.config))
        node = self.add(
            TeleopNode(
                parameter_overrides=[
                    config,
                    Parameter("autostart", value=False),
                    Parameter("joint_scales", value=[-1, 1, 1, 1, 1, 0]),
                    Parameter("alignment_timeout_s", value=30),
                ]
            )
        )
        self.assertEqual(node.mapping.scales, [-1.0, 1.0, 1.0, 1.0, 1.0, 0.0])
        self.assertTrue(all(type(value) is float for value in node.mapping.scales))
        self.assertEqual(node.alignment_timeout_s, 30.0)
        for name, value in (("joint_scales", "fast"), ("alignment_timeout_s", "30")):
            with self.assertRaisesRegex(ValueError, f"{name} must be"):
                TeleopNode(parameter_overrides=[config, Parameter(name, value=value)])

    def test_teleop_autostart_explains_why_it_is_waiting(self):
        self.follower.control.allow_writes = False
        self.teleop(autostart=True)
        self.pump(lambda: "allow_writes=false" in self.source.control.message)
        self.assertFalse(self.source.autostart_attempted)
        self.assertFalse(self.bus.writes)
        self.leader_timer.cancel()
        self.pump(lambda: "leader joint states" in self.source.control.message)
        self.assertFalse(self.bus.writes)
        self.follower.control.allow_writes = True
        self.leader_timer.reset()
        self.pump(lambda: self.source.phase == "tracking")

    def test_gui_start_without_target_still_requires_an_in_range_holding_pose(self):
        window = self.slider()
        self.bus.raw[3] = 3001
        self.pump(lambda: not self.source.control.in_range())
        window.refresh()
        self.assertFalse(window.start_button.isEnabled())
        result = self.source.control.start().result()
        self.assertFalse(result.success)
        self.assertIn("elbow_flex", result.message)
        self.assertFalse(self.bus.writes)
        for target in ([0.0] * 5, [float("nan")] * 6, [0.0, 0.0, 3.0, 0.0, 0.0, 0.0]):
            result = self.source.control.start(target=target).result()
            self.assertFalse(result.success)
        self.assertFalse(self.bus.writes)
