"""Six-joint slider control through the follower ROS interface."""

import math

import rclpy
from PyQt5 import QtCore, QtWidgets
from rclpy.node import Node

from .command_mapping import load_follower_model
from .control_client import FollowerClient
from .motor_panel import MotorPanel
from .profile import JOINTS
from .startup import STARTUP_ERRORS, init_rclpy, report_startup_error


class SliderNode(Node):
    def __init__(self, **kwargs):
        super().__init__("so101_slider_control", **kwargs)
        self.declare_parameter("follower_config", "")
        self.robot = load_follower_model(self.get_parameter("follower_config").value)
        self.control = FollowerClient(self, self.robot.command_limits())
        self.home_error = "No home position saved"
        try:
            self.home_positions = self.robot.home_positions()
        except ValueError as exc:
            self.home_positions = None
            self.home_error = f"Cannot use the home position: {exc}"

    def can_set_home(self):
        return (
            self.control.fresh()
            and self.control.in_range()
            and self.control.pending is None
            and not self.control.release_requested
        )

    def set_home_from_current_pose(self):
        if not self.can_set_home():
            raise ValueError("Wait for a fresh, valid pose before setting home")
        self.home_positions = self.robot.save_home_positions(list(self.control.state.positions))
        self.home_error = ""


class SliderWindow(QtWidgets.QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.control = node.control
        self.release_future = None
        self.home_notice = ""
        self.setWindowTitle("SO101 Follower Control")
        self.resize(1180, 820)
        layout = QtWidgets.QGridLayout(self)
        for column, text in enumerate(("Joint", "Target", "Target (deg)", "Current (deg)")):
            layout.addWidget(QtWidgets.QLabel(text), 0, column)
        self.sliders = []
        self.spins = []
        self.actual = []
        for i, (name, (lo, hi)) in enumerate(zip(JOINTS, self.control.limits)):
            layout.addWidget(QtWidgets.QLabel(name), i + 1, 0)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(math.ceil(math.degrees(lo) * 10), math.floor(math.degrees(hi) * 10))
            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(1)
            spin.setSingleStep(0.1)
            spin.setRange(math.degrees(lo), math.degrees(hi))
            actual = QtWidgets.QLabel("—")
            slider.valueChanged.connect(lambda value, joint=i: self.set_joint(joint, value / 10))
            spin.valueChanged.connect(lambda value, joint=i: self.set_joint(joint, value))
            self.sliders.append(slider)
            self.spins.append(spin)
            self.actual.append(actual)
            layout.addWidget(slider, i + 1, 1)
            layout.addWidget(spin, i + 1, 2)
            layout.addWidget(actual, i + 1, 3)
        self.start_button = QtWidgets.QPushButton("Start control")
        self.home_button = QtWidgets.QPushButton("Go home")
        self.set_home_button = QtWidgets.QPushButton("Set current pose as home")
        self.stop_button = QtWidgets.QPushButton("Stop")
        self.release_button = QtWidgets.QPushButton("Release torque and quit")
        self.start_button.clicked.connect(self.start)
        self.home_button.clicked.connect(self.go_home)
        self.set_home_button.clicked.connect(self.set_home)
        self.stop_button.clicked.connect(self.stop)
        self.release_button.clicked.connect(self.release_and_exit)
        self.status = QtWidgets.QLabel()
        self.status.setWordWrap(True)
        buttons = QtWidgets.QHBoxLayout()
        for button in (
            self.start_button,
            self.home_button,
            self.set_home_button,
            self.stop_button,
            self.release_button,
        ):
            buttons.addWidget(button)
        layout.addLayout(buttons, 7, 0, 1, 4)
        layout.addWidget(self.status, 8, 0, 1, 4)
        layout.addWidget(QtWidgets.QLabel("Support the arm before releasing torque."), 9, 0, 1, 4)
        self.motor_panel = MotorPanel(node)
        layout.addWidget(self.motor_panel, 10, 0, 1, 4)
        self.refresh()

    def display_joint(self, index, degrees):
        slider, spin = self.sliders[index], self.spins[index]
        slider.blockSignals(True)
        spin.blockSignals(True)
        slider.setValue(round(degrees * 10))
        spin.setValue(degrees)
        slider.blockSignals(False)
        spin.blockSignals(False)

    def set_joint(self, index, degrees):
        if not self.control.active or self.control.target is None:
            return
        self.home_notice = ""
        lo, hi = self.control.limits[index]
        value = min(hi, max(lo, math.radians(degrees)))
        self.control.target[index] = value
        self.display_joint(index, math.degrees(value))

    def start(self):
        self.home_notice = ""
        self.control.start()
        if self.control.target is not None:
            for i, position in enumerate(self.control.target):
                self.display_joint(i, math.degrees(position))
        self.refresh()

    def go_home(self):
        if (
            not self.control.active
            or not self.control.fresh()
            or self.node.home_positions is None
            or self.control.release_requested
        ):
            return
        self.home_notice = ""
        self.control.target = list(self.node.home_positions)
        for i, position in enumerate(self.control.target):
            self.display_joint(i, math.degrees(position))

    def set_home(self):
        try:
            self.node.set_home_from_current_pose()
        except (OSError, ValueError) as exc:
            self.home_notice = f"Cannot save the home position: {exc}"
        else:
            self.home_notice = "Saved the current pose as home"
        self.refresh()

    def stop(self):
        self.home_notice = ""
        self.control.stop()
        self.refresh()

    def release_and_exit(self):
        self.release_future = self.control.release_torque()
        self.release_future.add_done_callback(
            lambda future: QtCore.QTimer.singleShot(0, self.finish_release)
        )
        self.refresh()

    def finish_release(self):
        future, self.release_future = self.release_future, None
        try:
            if future.result().success:
                self.close()
                return
        except Exception as exc:
            self.control.error_message = str(exc)
        self.refresh()

    def refresh(self):
        control = self.control
        fresh = control.fresh()
        if fresh:
            for i, position in enumerate(control.state.positions):
                self.actual[i].setText(f"{math.degrees(position):.1f}")
                if not control.active and control.pending is None:
                    self.display_joint(i, math.degrees(position))
        else:
            for label in self.actual:
                label.setText("—")
        for widget in self.sliders + self.spins:
            widget.setEnabled(control.active and fresh)
        self.start_button.setEnabled(control.ready() and not control.active)
        self.home_button.setEnabled(
            control.active
            and fresh
            and self.node.home_positions is not None
            and not control.release_requested
        )
        self.home_button.setToolTip(
            self.node.home_error
            if self.node.home_positions is None
            else "Move to the saved home position (after starting control)"
        )
        self.set_home_button.setEnabled(self.node.can_set_home())
        self.set_home_button.setToolTip(
            "Save the current pose as the home position for later sessions"
        )
        self.stop_button.setEnabled(
            control.service.service_is_ready() and not control.release_requested
        )
        self.release_button.setEnabled(
            control.release_service.service_is_ready()
            and control.writes_allowed
            and control.pending is None
            and not control.releasing
        )
        if control.release_requested:
            text = control.error_message or control.message
        elif fresh and not control.in_range():
            text = control.range_error()
        elif control.error_message:
            text = control.error_message
        elif self.home_notice:
            text = self.home_notice
        elif control.ready() and not control.active:
            text = "Ready to start"
        elif fresh and not control.writes_allowed:
            text = "Follower control is not allowed"
        else:
            text = control.message
        self.status.setText(text)
        self.motor_panel.refresh()


def main(args=None):
    if not init_rclpy("so101_slider_control", args):
        return 1
    app = QtWidgets.QApplication([])
    node = None
    try:
        node = SliderNode()
        window = SliderWindow(node)
        timer = QtCore.QTimer()

        def update():
            if not rclpy.ok():
                app.quit()
                return
            for _ in range(4):
                rclpy.spin_once(node, timeout_sec=0)
            node.control.tick()
            window.refresh()

        timer.timeout.connect(update)
        timer.start(33)
        window.show()
        app.exec_()
        timer.stop()
    except KeyboardInterrupt:
        pass
    except STARTUP_ERRORS as exc:
        if node is not None:
            raise
        report_startup_error("so101_slider_control", exc)
        return 1
    finally:
        if node is not None:
            if rclpy.ok() and not node.control.torque_released:
                future = node.control.stop()
                rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
