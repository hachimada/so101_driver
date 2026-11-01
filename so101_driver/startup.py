"""Run driver nodes: expected startup errors become one log line, shutdown is shielded."""

import signal

import rclpy
import yaml
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.executors import ExternalShutdownException

from .hardware.feetech_bus import BusError

# Caused by settings, calibration files or the connection rather than by bugs.
STARTUP_ERRORS = (
    ValueError,
    KeyError,
    OSError,
    BusError,
    yaml.YAMLError,
    InvalidParameterTypeException,
)


def report_startup_error(node_name, exc):
    if isinstance(exc, KeyError):
        message = f"Missing key {exc} in a settings or calibration file"
    else:
        message = str(exc)
    rclpy.logging.get_logger(node_name).error(message)


def init_rclpy(node_name, args):
    """Initialize rclpy; return False after logging an unreadable --params-file YAML."""
    try:
        rclpy.init(args=args)
    except RuntimeError as exc:
        report_startup_error(node_name, exc)
        return False
    return True


def run_node(node_name, create, args=None):
    """Spin the node made by create(). Return 1 after a logged startup error."""
    if not init_rclpy(node_name, args):
        return 1
    node = None
    try:
        node = create()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except STARTUP_ERRORS as exc:
        if node is not None:
            raise
        report_startup_error(node_name, exc)
        return 1
    finally:
        # launch may forward SIGINT after the terminal has already delivered it.
        # Finish releasing the serial descriptor before accepting another one.
        previous_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            signal.signal(signal.SIGINT, previous_handler)
