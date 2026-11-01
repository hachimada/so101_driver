"""Read-only motor dashboard for the slider GUI."""

import math

from PyQt5 import QtCore, QtGui, QtWidgets

from .hardware.feetech_bus import STS3215_MODEL_NUMBER
from .motor_telemetry import MotorMonitor, fault_text
from .profile import JOINTS

TORQUE_LABELS = {0: "OFF", 1: "ON", 2: "Damping"}
MODE_LABELS = {0: "Position", 1: "Constant speed", 2: "PWM", 3: "Step"}
# Detail group that starts expanded.
LIVE_VALUES = "Live values"


class MotorPanel(QtWidgets.QGroupBox):
    def __init__(self, node):
        super().__init__("Motor status")
        self.node = node
        self.monitor = MotorMonitor(node)
        self.selected = 0
        layout = QtWidgets.QVBoxLayout(self)
        self.summary = QtWidgets.QLabel("Waiting for motor data")
        self.connection = QtWidgets.QLabel()
        self.connection.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.summary)
        layout.addWidget(self.connection)
        self.table = QtWidgets.QTableWidget(6, 10)
        self.table.setHorizontalHeaderLabels(
            (
                "ID",
                "Joint",
                "Torque",
                "Motion",
                "Speed (rpm)",
                "Current (mA)",
                "Duty (%)",
                "Voltage (V)",
                "Temp (°C)",
                "Status",
            )
        )
        self.table.verticalHeader().hide()
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(9, QtWidgets.QHeaderView.Stretch)
        self.table.setAlternatingRowColors(True)
        self.table.horizontalHeader().setStyleSheet("QHeaderView::section { padding: 4px 8px; }")
        tips = {
            0: "Motor ID.",
            2: "Torque output state read from the motor.",
            3: "Servo moving flag. It does not show movement by hand.",
            4: "Speed in rpm, converted using the servo phase setting. Details show the raw value.",
            5: "Motor phase current. The sign shows the direction (1 unit = 6.5 mA).",
            6: "PWM duty cycle (on time ÷ period × 100). The sign shows the drive direction.",
            7: "Voltage reported by the servo (register value × 0.1 V).",
            8: "Internal servo temperature.",
            9: "Communication state and servo fault bits.",
        }
        for column, tip in tips.items():
            self.table.horizontalHeaderItem(column).setToolTip(tip)
        for row, name in enumerate(JOINTS):
            for column, text in enumerate((str(row + 1), name) + ("—",) * 8):
                item = QtWidgets.QTableWidgetItem(text)
                item.setToolTip(tips.get(column, ""))
                if column == 0:
                    item.setTextAlignment(QtCore.Qt.AlignCenter)
                self.table.setItem(row, column, item)
        self.table.setFixedHeight(
            self.table.horizontalHeader().sizeHint().height()
            + sum(self.table.rowHeight(row) for row in range(6))
            + 2 * self.table.frameWidth()
        )
        self.table.itemSelectionChanged.connect(self.select_motor)
        self.table.selectRow(0)
        layout.addWidget(self.table)
        self.detail_title = QtWidgets.QLabel()
        layout.addWidget(self.detail_title)
        self.details = QtWidgets.QTreeWidget()
        self.details.setHeaderLabels(("Item", "Value"))
        self.details.setRootIsDecorated(True)
        self.details.setMinimumHeight(150)
        self.details.header().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.detail_items = {}
        layout.addWidget(self.details)

    def select_motor(self):
        if self.table.currentRow() >= 0:
            self.selected = self.table.currentRow()

    @staticmethod
    def number(record, key, digits=1, suffix=""):
        value = record.number(key) if record is not None else None
        return "—" if value is None else f"{value:.{digits}f}{suffix}"

    def refresh(self):
        valid_count = 0
        errors = 0
        failures = 0
        for row, name in enumerate(JOINTS):
            record = self.monitor.records.get(name)
            fresh = record is not None and self.monitor.fresh(record)
            valid = fresh and record.valid
            if record:
                failures += int(record.number("read_failures") or 0)
            if valid:
                valid_count += 1
                torque = record.number("torque_enabled")
                moving = record.number("moving")
                current = (
                    "N/A"
                    if int(record.number("phase") or 0) & 32
                    else self.number(record, "current_ma", 0)
                )
                text = (
                    TORQUE_LABELS.get(torque, "Unknown"),
                    {0: "Stopped", 1: "Moving"}.get(moving, "Unknown"),
                    self.number(record, "speed_rpm", 2),
                    current,
                    self.number(record, "load_pct"),
                    self.number(record, "voltage_v"),
                    self.number(record, "temperature_c", 0),
                    record.message,
                )
            else:
                state = "No data" if record is None else record.message if fresh else "Stale"
                text = ("—",) * 7 + (state,)
            fault = fresh and (not record.valid or record.level >= 2)
            errors += int(bool(fault))
            color = QtGui.QColor("#a61b1b" if fault else "#666666" if not fresh else "#164d2d")
            for column, value in enumerate(text, 2):
                item = self.table.item(row, column)
                item.setText(value)
                item.setForeground(color if column == 9 or not valid else self.palette().text())
                if column == 9:
                    item.setToolTip(value)
        disabled = self.node.control.hardware_details.get("motor_state_rate_hz") == "0.0"
        text = "Motor monitoring is disabled" if disabled else f"Reading: {valid_count} / 6 joints"
        if errors:
            text += f"  Faults: {errors} joints"
        self.summary.setText(text)
        self.summary.setStyleSheet("color: #a61b1b; font-weight: bold;" if errors else "")
        control = self.node.control
        self.connection.setText(
            f"Device: {control.hardware_id or 'not connected'}  |  "
            f"Position read failures: {control.hardware_details.get('failed_samples', '—')}  "
            f"Status read failures: {failures}  "
            f"Rejected commands: {control.hardware_details.get('rejected_commands', '—')}"
        )
        self.refresh_details()

    def refresh_details(self):
        name = JOINTS[self.selected]
        record = self.monitor.records.get(name)
        self.detail_title.setText(f"{name} / ID {self.selected + 1}")
        if record is None:
            for item in self.detail_items.values():
                item.setText(1, "—")
            return
        valid = self.monitor.fresh(record) and record.valid

        def value(key, digits=1, suffix=""):
            return self.number(record, key, digits, suffix)

        def live(key, digits=1, suffix=""):
            return value(key, digits, suffix) if valid else "—"

        def angle(key):
            ticks = record.number(key)
            if not valid or ticks is None:
                return "—"
            return f"{math.degrees(self.node.robot.conversions[name].to_radians(ticks)):.1f}°"

        age = max(0, self.monitor.clock() - record.received)
        groups = {
            LIVE_VALUES: (
                ("Last update", f"{age:.1f} s ago"),
                ("Position", f"{angle('position_ticks')} / {live('position_ticks', 0)} ticks"),
                ("Servo goal position", f"{angle('goal_ticks')} / {live('goal_ticks', 0)} ticks"),
                ("Voltage register", live("voltage_raw", 0)),
                ("Current register (signed)", live("current_raw", 0)),
                ("Speed register (signed)", live("speed_raw", 0)),
                ("Torque limit setting", live("torque_limit_pct", 1, " %")),
                ("Packet error bits", live("packet_error", 0)),
                ("Servo fault bits", live("servo_status", 0)),
            ),
            "Info and settings (read about every 10 s)": (
                (
                    "Model",
                    "STS3215"
                    if record.number("model_number") == STS3215_MODEL_NUMBER
                    else value("model_number", 0),
                ),
                ("Model number", value("model_number", 0)),
                ("Firmware", record.values.get("firmware_version", "—")),
                ("Operating mode", MODE_LABELS.get(record.number("operating_mode"), "Unknown")),
                (
                    "Hardware position range",
                    f"{value('hardware_min_ticks', 0)} to {value('hardware_max_ticks', 0)} ticks",
                ),
                ("Temperature limit", value("temperature_limit_c", 0, " °C")),
                ("Voltage range", f"{value('min_voltage_v')} to {value('max_voltage_v')} V"),
                ("Protection current", value("current_limit_ma", 0, " mA")),
                (
                    "Position PID (register values)",
                    f"P={value('position_p', 0)} / I={value('position_i', 0)} / "
                    f"D={value('position_d', 0)}",
                ),
                ("Phase setting (register)", value("phase", 0)),
                (
                    "Enabled protections",
                    fault_text(int(record.number("protection_mask")))
                    if record.number("protection_mask")
                    else "None"
                    if record.number("protection_mask") == 0
                    else "—",
                ),
                (
                    "Settings last read",
                    f"{record.number('info_age_s') + age:.1f} s ago"
                    if record.number("info_age_s") is not None
                    else "—",
                ),
                (
                    "Settings read status",
                    record.values.get(
                        "info_error", "Read" if "model_number" in record.values else "Not read"
                    ),
                ),
            ),
        }
        for title, rows in groups.items():
            parent = self.detail_items.get(title)
            if parent is None:
                parent = QtWidgets.QTreeWidgetItem(self.details, [title])
                self.detail_items[title] = parent
                parent.setExpanded(title == LIVE_VALUES)
            for label, text in rows:
                key = (title, label)
                if key not in self.detail_items:
                    self.detail_items[key] = QtWidgets.QTreeWidgetItem(parent, [label])
                self.detail_items[key].setText(1, text)
