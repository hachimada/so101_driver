"""Helpers for the ROS messages that the driver nodes read and publish."""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def stamp_ns(stamp):
    # Unlike rclpy.time.Time.from_msg, this never raises. A negative stamp from another
    # node must fail the freshness checks, not stop the subscribing node.
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def hardware_diagnostics(node, hardware_id, level, message, values):
    """A DiagnosticArray holding the node's "<node>/hardware" status."""
    status = DiagnosticStatus(
        name=node.get_fully_qualified_name() + "/hardware",
        hardware_id=hardware_id,
        level=level,
        message=message,
        values=[KeyValue(key=key, value=str(value)) for key, value in values],
    )
    msg = DiagnosticArray(status=[status])
    msg.header.stamp = node.get_clock().now().to_msg()
    return msg
