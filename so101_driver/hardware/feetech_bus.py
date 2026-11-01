"""ROS-independent access to a single STS bus. Opening never configures servos."""

import fcntl
import importlib
import os
from dataclasses import dataclass
from threading import RLock

# Single-turn position range and model number of the SO-101's STS3215 servos.
TICKS_PER_TURN = 4096  # Positions 0..MAX_TICK; MAX_TICK and 0 are one tick apart.
MAX_TICK = TICKS_PER_TURN - 1
STS3215_MODEL_NUMBER = 777


class BusError(RuntimeError):
    """Transport failure or a servo-reported fault; never a valid sample."""


@dataclass(frozen=True)
class PositionSample:
    positions_ticks: dict[int, int]


def load_sdk():
    # The bundled official SDK is installed under this package, so the top-level
    # scservo_sdk from pip feetech-servo-sdk (a LeRobot dependency) cannot replace it.
    return importlib.import_module("so101_driver._vendor.scservo_sdk")


class FeetechBus:
    def __init__(self, port: str, baudrate: int = 1_000_000, *, sdk=None):
        self.port_name = port
        self.baudrate = baudrate
        self._sdk = sdk
        self._port = None
        self._handler = None
        self._claim = None
        self._lock = RLock()

    @staticmethod
    def _ids(ids):
        ids = tuple(ids)
        if not ids or any(type(i) is not int or not 1 <= i <= 252 for i in ids):
            raise ValueError("Motor IDs must be integers from 1 to 252")
        if len(set(ids)) != len(ids):
            raise ValueError("Duplicate motor IDs")
        return ids

    def _claim_port(self):
        # The SDK reconfigures the port and flushes its input as soon as it opens it, which
        # would disturb a process already using it. Lock the device first so that a second
        # driver process stops before touching the port. The lock ends when fd is closed.
        try:
            fd = os.open(self.port_name, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as exc:
            raise BusError(f"Cannot open {self.port_name}: {exc}") from exc
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            os.close(fd)
            if isinstance(exc, BlockingIOError):
                raise BusError(
                    f"{self.port_name} is in use by another process "
                    "(another driver node or so101_calibrate)"
                ) from None
            raise BusError(f"Cannot lock {self.port_name}: {exc}") from exc
        return fd

    def open(self):
        with self._lock:
            if self._port is not None:
                raise BusError("Port is already open")
            sdk = self._sdk or load_sdk()
            claim = self._claim_port()
            port = None
            try:
                port = sdk.PortHandler(self.port_name)
                if not port.setBaudRate(self.baudrate):
                    raise BusError(f"Unsupported baud rate: {self.baudrate}")
                handler = sdk.sms_sts(port)
            except Exception as exc:
                if port is not None and port.is_open:
                    port.closePort()
                os.close(claim)
                raise BusError(f"Cannot open {self.port_name}: {exc}") from exc
            self._sdk, self._port, self._handler, self._claim = sdk, port, handler, claim

    def close(self):
        with self._lock:
            if self._port is not None:
                port, claim = self._port, self._claim
                self._port, self._handler, self._claim = None, None, None
                try:
                    port.closePort()
                finally:
                    os.close(claim)

    def _check(self, motor_id, operation, comm, error):
        if comm != self._sdk.COMM_SUCCESS:
            raise BusError(
                f"{operation} ID={motor_id} comm={comm}: {self._handler.getTxRxResult(comm)}"
            )
        if error:
            raise BusError(f"{operation} ID={motor_id} servo_error=0x{error:02x}")

    def _require_open(self):
        if self._handler is None:
            raise BusError("Port is not open")

    def ping(self, ids):
        ids = self._ids(ids)
        with self._lock:
            self._require_open()
            models = {}
            for motor_id in ids:
                value, comm, error = self._handler.ping(motor_id)
                self._check(motor_id, "ping", comm, error)
                models[motor_id] = value
            return models

    def read_positions(self, ids):
        ids = self._ids(ids)
        with self._lock:
            self._require_open()
            positions = {}
            for motor_id in ids:
                value, comm, error = self._handler.ReadPos(motor_id)
                self._check(motor_id, "ReadPos", comm, error)
                positions[motor_id] = value
            return PositionSample(positions)

    def read_configuration(self, ids):
        """Read existing settings without changing torque or calibration."""
        ids = self._ids(ids)
        registers = {
            "torque_enabled": (40, 1),
            "operating_mode": (33, 1),
            "max_voltage_raw": (14, 1),
            "min_voltage_raw": (15, 1),
            "homing_offset_encoded": (31, 2),
            "range_min": (9, 2),
            "range_max": (11, 2),
        }
        with self._lock:
            self._require_open()
            settings = {}
            for motor_id in ids:
                values = {}
                for name, (address, length) in registers.items():
                    read = (
                        self._handler.read1ByteTxRx if length == 1 else self._handler.read2ByteTxRx
                    )
                    value, comm, error = read(motor_id, address)
                    self._check(motor_id, f"read {name}", comm, error)
                    values[name] = value
                encoded = values.pop("homing_offset_encoded")
                values["homing_offset"] = -(encoded & 0x7FF) if encoded & 0x800 else encoded
                settings[motor_id] = values
            return settings

    def read_motion_status(self, ids):
        """Read the existing SRAM goal and moving flag without changing either."""
        ids = self._ids(ids)
        with self._lock:
            self._require_open()
            status = {}
            for motor_id in ids:
                goal, comm, error = self._handler.read2ByteTxRx(motor_id, 42)
                self._check(motor_id, "read goal position", comm, error)
                moving, comm, error = self._handler.ReadMoving(motor_id)
                self._check(motor_id, "read moving", comm, error)
                if not 0 <= goal <= MAX_TICK or moving not in (0, 1):
                    raise BusError(f"Invalid single-turn motion status: ID={motor_id}")
                status[motor_id] = {"goal_ticks": goal, "moving": bool(moving)}
            return status

    def _read_block(self, motor_id, address, length):
        """Diagnostic reads retain servo faults but reject failed/truncated packets."""
        data, comm, error = self._handler.readTxRx(motor_id, address, length)
        if comm != self._sdk.COMM_SUCCESS:
            self._check(motor_id, "read motor state", comm, error)
        if len(data) != length or any(type(v) is not int or not 0 <= v <= 255 for v in data):
            raise BusError(f"Incomplete motor state: ID={motor_id}")
        return data, error

    def read_motor_info(self, ids):
        """Read identification and EEPROM settings, without modifying registers."""
        ids = self._ids(ids)
        with self._lock:
            self._require_open()
            result = {}
            for motor_id in ids:
                data, error = self._read_block(motor_id, 0, 40)

                def word(address):
                    return data[address] | data[address + 1] << 8

                result[motor_id] = {
                    "model_number": word(3),
                    "firmware_version": f"{data[0]}.{data[1]}",
                    "operating_mode": data[33],
                    "phase": data[18],
                    "hardware_min_ticks": word(9),
                    "hardware_max_ticks": word(11),
                    "temperature_limit_c": data[13],
                    "max_voltage_v": data[14] * 0.1,
                    "min_voltage_v": data[15] * 0.1,
                    "current_limit_ma": word(28) * 6.5,
                    "position_p": data[21],
                    "position_d": data[22],
                    "position_i": data[23],
                    "protection_mask": data[19],
                    "info_packet_error": error,
                }
            return result

    def read_motor_status(self, ids):
        """Read live SRAM feedback in one official SDK readTxRx per motor.

        Fault bits are retained for display. These diagnostics must never be
        substituted for the strict position samples used by the controller.
        """
        ids = self._ids(ids)
        with self._lock:
            self._require_open()
            result = {}
            for motor_id in ids:
                data, error = self._read_block(motor_id, 40, 31)

                def word(address):
                    offset = address - 40
                    return data[offset] | data[offset + 1] << 8

                result[motor_id] = {
                    "torque_enabled": data[0],
                    "acceleration_raw": data[1],
                    "goal_ticks": self._handler.scs_tohost(word(42), 15),
                    "goal_speed_raw": self._handler.scs_tohost(word(46), 15),
                    "torque_limit_pct": word(48) * 0.1,
                    "eeprom_locked": data[15],
                    "position_ticks": self._handler.scs_tohost(word(56), 15),
                    "speed_raw": self._handler.scs_tohost(word(58), 15),
                    "load_pct": self._handler.scs_tohost(word(60), 10) * 0.1,
                    "voltage_v": data[22] * 0.1,
                    "voltage_raw": data[22],
                    "temperature_c": data[23],
                    "servo_status": data[25],
                    "moving": data[26],
                    "current_raw": self._handler.scs_tohost(word(69), 15),
                    "current_ma": self._handler.scs_tohost(word(69), 15) * 6.5,
                    "packet_error": error,
                }
            return result

    def write_positions(self, positions_ticks, *, speeds, acceleration=10):
        """Acknowledged, sequential SRAM goals; success does not mean arrival.

        speeds maps each target motor ID to its raw goal speed. Validate the entire
        request before sending anything. A later transport failure can still leave
        earlier motors with updated goals.
        """
        targets = dict(positions_ticks)
        ids = self._ids(targets)
        if any(type(v) is not int or not 0 <= v <= MAX_TICK for v in targets.values()):
            raise ValueError(f"Position targets must be integer ticks in [0, {MAX_TICK}]")
        if any(type(speeds.get(i)) is not int or not 1 <= speeds[i] <= 32767 for i in ids):
            raise ValueError(
                "speeds must give each target an integer in [1, 32767]; zero is not a stop"
            )
        if type(acceleration) is not int or not 1 <= acceleration <= 254:
            raise ValueError("acceleration must be an integer in [1, 254]")
        with self._lock:
            self._require_open()
            for motor_id in ids:
                comm, error = self._handler.WritePosEx(
                    motor_id, targets[motor_id], speeds[motor_id], acceleration
                )
                self._check(motor_id, "WritePosEx", comm, error)

    def set_torque_limit(self, motor_id, limit_raw):
        """Write and verify SRAM Torque Limit (48); 1000 is full output."""
        self._ids([motor_id])
        if type(limit_raw) is not int or not 0 <= limit_raw <= 1000:
            raise ValueError("limit_raw must be an integer in [0, 1000]")
        with self._lock:
            self._require_open()
            comm, error = self._handler.write2ByteTxRx(motor_id, 48, limit_raw)
            self._check(motor_id, "write torque limit", comm, error)
            actual, comm, error = self._handler.read2ByteTxRx(motor_id, 48)
            self._check(motor_id, "read torque limit", comm, error)
            if actual != limit_raw:
                raise BusError(
                    f"Torque limit readback mismatch ID={motor_id}: "
                    f"requested={limit_raw}, actual={actual}"
                )

    def set_torque_enabled(self, ids, enabled):
        """Explicit SRAM write, without changing EEPROM or operating mode."""
        ids = self._ids(ids)
        if type(enabled) is not bool:
            raise ValueError("enabled must be bool")
        with self._lock:
            self._require_open()
            for motor_id in ids:
                comm, error = self._handler.write1ByteTxRx(motor_id, 40, int(enabled))
                self._check(motor_id, "torque", comm, error)

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()
