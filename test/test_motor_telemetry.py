"""Diagnostic faults, failed packets and stale data must remain distinguishable."""

import unittest
from unittest.mock import Mock, call

try:
    from builtin_interfaces.msg import Time
    from diagnostic_msgs.msg import DiagnosticStatus

    from so101_driver.hardware.feetech_bus import BusError
    from so101_driver.motor_telemetry import (
        MotorMonitor,
        MotorTelemetryPublisher,
        fault_text,
    )
    from so101_driver.profile import JOINTS

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "Requires ROS2 Humble")
class MotorTelemetryTests(unittest.TestCase):
    def setUp(self):
        self.node = Mock(device_id="fake-device")
        self.node.get_fully_qualified_name.return_value = "/follower/so101_follower"
        now = self.node.get_clock.return_value.now.return_value
        now.nanoseconds = 1_000_000_000
        now.to_msg.return_value = Time(sec=1)
        self.bus = Mock()
        self.bus.read_motor_status.side_effect = lambda ids: {
            i: dict(packet_error=0, servo_status=0, voltage_v=0.0) for i in ids
        }
        self.bus.read_motor_info.side_effect = lambda ids: {i: dict(model_number=777) for i in ids}
        self.publisher = MotorTelemetryPublisher(self.node, self.bus, 2.0)

    def published(self):
        return self.publisher.publisher.publish.call_args.args[0]

    def test_poll_distributes_reads_and_caches_info_without_motor_writes(self):
        for _ in range(7):
            self.publisher.poll()
        self.assertEqual(
            self.bus.read_motor_status.call_args_list, [call([i]) for i in (1, 2, 3, 4, 5, 6, 1)]
        )
        self.assertEqual(self.bus.read_motor_info.call_count, 6)
        self.assertEqual(len(self.published().status), 1)
        self.assertTrue(self.published().status[0].name.endswith("/motor/shoulder_pan"))
        self.bus.write_positions.assert_not_called()
        self.bus.set_torque_enabled.assert_not_called()

    def test_servo_faults_and_failed_transport_are_not_normal_zero_readings(self):
        self.bus.read_motor_status.return_value = None
        self.bus.read_motor_status.side_effect = lambda ids: {
            i: dict(packet_error=4, servo_status=1, voltage_v=0.0) for i in ids
        }
        self.publisher.poll()
        msg = self.published()
        status = msg.status[0]
        values = {v.key: v.value for v in status.values}
        self.assertEqual(status.level, DiagnosticStatus.ERROR)
        self.assertIn("Voltage", status.message)
        self.assertIn("Overheat", status.message)
        self.assertEqual(values["sample_valid"], "True")
        self.assertEqual(values["voltage_v"], "0.0")  # A valid zero stays visible.
        self.bus.read_motor_status.side_effect = BusError("timeout")
        self.publisher.poll()
        status = self.published().status[0]
        values = {v.key: v.value for v in status.values}
        self.assertEqual(values["sample_valid"], "False")
        self.assertNotIn("voltage_v", values)
        self.assertEqual(values["read_failures"], "1")
        self.assertIn("Communication failed", status.message)
        self.assertIn("Unknown fault", fault_text(16))

    def test_info_failure_preserves_live_feedback_and_can_be_retried(self):
        self.bus.read_motor_info.side_effect = OSError("settings timeout")
        self.publisher.poll()
        values = {v.key: v.value for v in self.published().status[0].values}
        self.assertEqual(values["sample_valid"], "True")
        self.assertIn("settings timeout", values["info_error"])
        self.assertNotIn("model_number", values)
        self.bus.read_motor_info.side_effect = lambda ids: {i: dict(model_number=777) for i in ids}
        self.publisher.index = 0
        self.publisher.poll()
        values = {v.key: v.value for v in self.published().status[0].values}
        self.assertEqual(values["model_number"], "777")
        self.assertNotIn("info_error", values)

    def test_speed_units_follow_each_motors_read_phase_setting(self):
        self.bus.read_motor_status.side_effect = lambda ids: {
            i: dict(packet_error=0, servo_status=0, speed_raw=-100) for i in ids
        }
        self.bus.read_motor_info.side_effect = lambda ids: {
            i: dict(phase=4 if i == 1 else 0) for i in ids
        }
        for expected in (-1.46, -73.2):
            self.publisher.poll()
            values = {v.key: v.value for v in self.published().status[0].values}
            self.assertAlmostEqual(float(values["speed_rpm"]), expected)

    def test_monitor_rejects_old_or_future_samples_and_marks_stale_feedback(self):
        clock = [10.0]
        monitor = MotorMonitor(self.node, clock=lambda: clock[0])
        self.publisher.poll()
        msg = self.published()
        monitor.receive(msg)
        record = monitor.records[JOINTS[0]]
        self.assertTrue(record.valid)
        self.assertTrue(monitor.fresh(record))
        msg.status[0].message = "outdated"
        monitor.receive(msg)
        self.assertEqual(monitor.records[JOINTS[0]].message, "OK")
        msg.header.stamp = Time(sec=2)
        monitor.receive(msg)
        self.assertEqual(monitor.records[JOINTS[0]].stamp_ns, 1_000_000_000)
        clock[0] = 12.0
        self.assertFalse(monitor.fresh(record))
        self.assertIsNone(record.number("unknown"))
        record.values["voltage_v"] = "nan"
        self.assertIsNone(record.number("voltage_v"))
        # Source age is checked as well as receipt time.
        clock[0] = 10.0
        self.node.get_clock.return_value.now.return_value.nanoseconds = 3_000_000_000
        self.assertFalse(monitor.fresh(record))
