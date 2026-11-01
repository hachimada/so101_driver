import math
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

try:
    import rclpy
    from std_msgs.msg import String

    from so101_driver.calibration import CalibrationSession
    from so101_driver.calibration_view import CalibrationView, calibration_preview

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "Requires ROS2 Humble")
class CalibrationViewTests(unittest.TestCase):
    def test_live_model_appears_only_after_measuring_reference_not_range_midpoint(self):
        hardware = {i: dict(homing_offset=0, range_min=1000, range_max=3000) for i in range(1, 7)}
        hardware[3].update(range_min=833, range_max=3061)
        reference = {i: 2000 for i in range(1, 7)} | {3: 2055}
        session = CalibrationSession(hardware)
        bus = Mock()
        bus.read_positions.return_value = SimpleNamespace(positions_ticks=reference)
        rclpy.init()
        view = None
        try:
            with patch("so101_driver.calibration_view.StaticTransformBroadcaster") as broadcaster:
                view = CalibrationView(bus, session, terminal=Mock())
            view.live = Mock()
            view.target = Mock()
            view.visible_description = Mock()
            description = String(data='<robot name="test"><link name="base_link"/></robot>')
            view.receive_description(description)
            view.update()
            view.live.publish.assert_not_called()
            view.visible_description.publish.assert_not_called()
            self.assertIsNone(session.zero)
            frames = broadcaster.return_value.sendTransform.call_args.args[0]
            self.assertEqual([t.child_frame_id for t in frames], ["calibration/target/base_link"])
            target = list(view.target.publish.call_args.args[0].position)

            session.start_from_reference()
            view.update()
            self.assertEqual(list(view.live.publish.call_args.args[0].position), target)
            view.visible_description.publish.assert_called_once_with(description)
            frames = broadcaster.return_value.sendTransform.call_args.args[0]
            self.assertEqual(
                {t.child_frame_id for t in frames},
                {"calibration/live/base_link", "calibration/target/base_link"},
            )

            bus.read_positions.return_value = SimpleNamespace(positions_ticks=reference | {3: 2155})
            view.update()
            positions = list(view.live.publish.call_args.args[0].position)
            self.assertAlmostEqual(positions[2], 100 * 2 * math.pi / 4096)
            self.assertEqual(list(view.target.publish.call_args.args[0].position), target)
            self.assertEqual(broadcaster.return_value.sendTransform.call_count, 2)
            view.visible_description.publish.assert_called_once_with(description)
            view.receive_description(description)
            self.assertEqual(view.visible_description.publish.call_count, 2)
            bus.write_positions.assert_not_called()
            bus.set_torque_enabled.assert_not_called()
        finally:
            if view is not None:
                view.destroy_node()
            rclpy.shutdown()

    def test_description_arriving_after_reference_is_forwarded_to_rviz(self):
        hardware = {i: dict(homing_offset=0, range_min=1000, range_max=3000) for i in range(1, 7)}
        raw = {i: 2000 for i in range(1, 7)}
        session = CalibrationSession(hardware)
        session.accept(raw)
        session.start_from_reference()
        bus = Mock()
        bus.read_positions.return_value = SimpleNamespace(positions_ticks=raw)
        rclpy.init()
        view = None
        try:
            view = CalibrationView(bus, session, terminal=Mock())
            view.visible_description = Mock()
            view.update()
            view.visible_description.publish.assert_not_called()
            description = String(data='<robot name="test"><link name="base_link"/></robot>')
            view.receive_description(description)
            view.visible_description.publish.assert_called_once_with(description)
        finally:
            if view is not None:
                view.destroy_node()
            rclpy.shutdown()

    def test_readings_are_sent_to_terminal_not_rviz(self):
        hardware = {i: dict(homing_offset=0, range_min=1000, range_max=3000) for i in range(1, 7)}
        session = CalibrationSession(hardware)
        bus = Mock()
        bus.read_positions.return_value = SimpleNamespace(
            positions_ticks={i: 2000 for i in range(1, 7)}
        )
        terminal = Mock()
        rclpy.init()
        view = None
        try:
            view = CalibrationView(bus, session, terminal=terminal)
            view.target = Mock()
            view.update()
            reference = list(view.target.publish.call_args.args[0].position)
            self.assertNotIn(
                "/calibration/instructions", [publisher.topic_name for publisher in view.publishers]
            )
            readings = "\n".join(terminal.update.call_args.args[0])
            self.assertEqual(readings, "")  # No internal status or values before recording.
            session.start_from_reference()
            bus.read_positions.return_value = SimpleNamespace(
                positions_ticks={i: 2100 for i in range(1, 7)}
            )
            view.update()
            self.assertEqual(list(view.target.publish.call_args.args[0].position), reference)
            readings = "\n".join(terminal.update.call_args.args[0])
            self.assertIn("Min", readings)
            self.assertIn("Max", readings)
            self.assertIn("shoulder_pan", readings)
            self.assertIn("wrist_roll", readings)
            self.assertIn("gripper", readings)
            self.assertIn("2000", readings)
            self.assertIn("2100", readings)
            self.assertNotIn("angle", readings)
            self.assertNotIn("stage", readings.lower())
            self.assertNotIn("communication", readings.lower())
            bus.read_positions.side_effect = OSError("disconnected")
            view.update()
            readings = "\n".join(terminal.update.call_args.args[0])
            self.assertIn("disconnected", readings)
            self.assertIn("sample not recorded", readings)
            self.assertEqual(session.ranges[6]["min"], 2000)
            self.assertEqual(session.ranges[6]["max"], 2100)
        finally:
            if view is not None:
                view.destroy_node()
            rclpy.shutdown()

    def test_preview_releases_processes_and_temporary_files(self):
        share = Path(__file__).parents[1]
        children = [Mock(pid=101), Mock(pid=102), Mock(pid=103)]
        for child in children:
            child.poll.return_value = None
        with (
            patch("so101_driver.calibration_view.subprocess.Popen", side_effect=children) as spawn,
            patch("so101_driver.calibration_view.os.killpg") as kill,
        ):
            with calibration_preview(share) as running:
                self.assertEqual(running, children)
                commands = [c.args[0] for c in spawn.call_args_list]
                self.assertEqual(
                    [c[2] if c[0] == "ros2" else c[0] for c in commands],
                    ["robot_state_publisher", "robot_state_publisher", "rviz2"],
                )
                params = Path(commands[0][-1])
                self.assertTrue(params.exists())
            self.assertFalse(params.exists())
            self.assertEqual(kill.call_count, 3)
            for child in children:
                child.wait.assert_called_once()

    def test_no_rviz_starts_no_preview_process(self):
        share = Path(__file__).parents[1]
        with patch("so101_driver.calibration_view.subprocess.Popen") as spawn:
            with calibration_preview(share, show_rviz=False) as running:
                self.assertEqual(running, [])
        spawn.assert_not_called()

    def test_partial_preview_startup_does_not_leave_process_running(self):
        share = Path(__file__).parents[1]
        child = Mock(pid=101)
        child.poll.return_value = None
        with (
            patch(
                "so101_driver.calibration_view.subprocess.Popen",
                side_effect=[child, OSError("failed to start preview")],
            ),
            patch("so101_driver.calibration_view.os.killpg") as kill,
        ):
            with self.assertRaisesRegex(OSError, "failed to start preview"):
                with calibration_preview(share):
                    self.fail("Preview should not start")
            kill.assert_called_once()
            child.wait.assert_called_once()
