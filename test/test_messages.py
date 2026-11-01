import unittest

try:
    from builtin_interfaces.msg import Time
    from diagnostic_msgs.msg import DiagnosticStatus

    from so101_driver.messages import hardware_diagnostics, stamp_ns

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "Requires ROS2 Humble")
class MessageTests(unittest.TestCase):
    def test_stamp_ns_keeps_invalid_stamps_for_the_freshness_checks(self):
        self.assertEqual(stamp_ns(Time(sec=3, nanosec=5)), 3_000_000_005)
        # rclpy.time.Time.from_msg raises here, which would stop a subscriber.
        self.assertEqual(stamp_ns(Time(sec=-1, nanosec=0)), -1_000_000_000)

    def test_hardware_diagnostics_names_the_node_and_stringifies_values(self):
        class Node:
            def get_fully_qualified_name(self):
                return "/follower/so101_follower"

            def get_clock(self):
                return self

            def now(self):
                return self

            def to_msg(self):
                return Time(sec=7)

        msg = hardware_diagnostics(
            Node(), "SERIAL", DiagnosticStatus.WARN, "check", (("count", 2), ("active", True))
        )
        status = msg.status[0]
        self.assertEqual(msg.header.stamp.sec, 7)
        self.assertEqual(status.name, "/follower/so101_follower/hardware")
        self.assertEqual(
            (status.hardware_id, status.level, status.message),
            ("SERIAL", DiagnosticStatus.WARN, "check"),
        )
        self.assertEqual(
            [(v.key, v.value) for v in status.values], [("count", "2"), ("active", "True")]
        )


if __name__ == "__main__":
    unittest.main()
