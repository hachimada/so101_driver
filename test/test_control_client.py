import unittest
from unittest.mock import Mock

try:
    from diagnostic_msgs.msg import DiagnosticStatus
    from rclpy.task import Future
    from rclpy.time import Time
    from std_srvs.srv import SetBool, Trigger

    from so101_driver.control_client import FollowerClient
    from so101_driver.profile import JOINTS

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "Requires ROS2 Humble")
class ControlClientTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.node = Mock()
        self.node.create_client.side_effect = [Mock(), Mock()]
        self.node.get_clock.return_value.now.return_value = Time(nanoseconds=1_000_000_000)
        self.client = FollowerClient(self.node, [(-1, 1)] * 6, clock=lambda: self.now)
        self.client.state.update(
            JOINTS, [0.0] * 6, 1_000_000_000, ros_now_ns=1_000_000_000, now=self.now
        )
        self.client.writes_allowed = True
        self.client.diagnostic_level = DiagnosticStatus.OK
        self.client.diagnostic_time = self.now
        self.client.service.service_is_ready.return_value = True

    def test_stop_during_enable_never_activates_and_repeats_hold_after_late_reply(self):
        started, stopped, held_late = Future(), Future(), Future()
        self.client.service.call_async.side_effect = [started, stopped, held_late]
        result = self.client.start()
        self.client.stop()
        stopped.set_result(SetBool.Response(success=True))
        started.set_result(SetBool.Response(success=True))
        self.assertFalse(self.client.active)
        self.assertFalse(result.result().success)
        requests = [call.args[0].data for call in self.client.service.call_async.call_args_list]
        self.assertEqual(requests, [True, False, False])
        held_late.set_result(SetBool.Response(success=True))

    def test_enable_timeout_resolves_request_and_late_success_is_stopped(self):
        started, hold = Future(), Future()
        self.client.service.call_async.side_effect = [started, hold]
        result = self.client.start()
        self.now += 2.1
        self.client.tick()
        self.assertFalse(result.result().success)
        started.set_result(SetBool.Response(success=True))
        self.assertFalse(self.client.active)
        self.assertFalse(self.client.service.call_async.call_args.args[0].data)
        hold.set_result(SetBool.Response(success=True))

    def test_old_disabled_diagnostic_does_not_stop_newly_enabled_control(self):
        started = Future()
        self.client.service.call_async.return_value = started
        self.client.start()
        started.set_result(SetBool.Response(success=True))
        self.client.diagnostic_stamp_ns = 900_000_000
        self.client.control_active = False
        self.client.tick()
        self.assertTrue(self.client.active)
        self.client.publisher.publish.assert_called_once()

    def test_stale_feedback_stops_publication(self):
        started, stopped = Future(), Future()
        self.client.service.call_async.side_effect = [started, stopped]
        self.client.start()
        started.set_result(SetBool.Response(success=True))
        self.now += 0.51
        self.client.tick()
        self.assertFalse(self.client.active)
        self.client.publisher.publish.assert_not_called()
        stopped.set_result(SetBool.Response(success=True))

    def test_current_pose_outside_limits_cannot_enable_control(self):
        self.client.state.positions[0] = 2.0
        self.assertFalse(self.client.ready())
        result = self.client.start()
        self.assertFalse(result.result().success)
        self.assertIn("outside the command range", result.result().message)
        self.assertIn("shoulder_pan", result.result().message)
        self.assertIn(": 114.6°", result.result().message)
        self.assertIn("range -57.3 to 57.3°", result.result().message)
        self.client.service.call_async.assert_not_called()

    def test_only_numeric_roundoff_at_a_limit_is_accepted(self):
        self.client.state.positions[0] = -1.0 - 1e-12
        self.assertTrue(self.client.in_range())
        self.client.state.positions[0] = -1.0 - 1e-4
        self.assertFalse(self.client.in_range())

    def test_start_rejection_is_preserved_while_the_connection_is_ready(self):
        started = Future()
        self.client.service.call_async.return_value = started
        self.client.start()
        started.set_result(SetBool.Response(success=False, message="cannot confirm it is holding"))
        self.assertTrue(self.client.ready())
        self.assertIn("cannot confirm it is holding", self.client.error_message)
        self.assertFalse(self.client.active)

    def test_release_stops_commands_and_waits_for_confirmation(self):
        released = Future()
        self.client.release_service.call_async.return_value = released
        self.client.release_service.service_is_ready.return_value = True
        self.client.active = True
        result = self.client.release_torque()
        self.assertFalse(self.client.active)
        self.assertFalse(self.client.ready())
        self.assertTrue(self.client.releasing)
        self.assertFalse(self.client.torque_released)
        self.client.tick()
        self.client.publisher.publish.assert_not_called()
        released.set_result(
            Trigger.Response(success=True, message="Torque released on all 6 joints")
        )
        self.assertTrue(result.result().success)
        self.assertTrue(self.client.torque_released)

    def test_release_cannot_race_with_an_in_flight_enable(self):
        self.client.pending = Future()
        result = self.client.release_torque()
        self.assertFalse(result.result().success)
        self.client.release_service.call_async.assert_not_called()

    def test_no_release_reply_does_not_count_as_torque_off(self):
        self.client.release_service.call_async.return_value = Future()
        self.client.release_torque()
        self.now += 5.1
        self.client.tick()
        self.assertFalse(self.client.torque_released)
        self.assertIn("release not confirmed", self.client.error_message)
