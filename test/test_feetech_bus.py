import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from so101_driver.hardware.feetech_bus import BusError, FeetechBus

# Holds the same advisory lock as FeetechBus.open() from a separate process.
HOLD_LOCK = """
import fcntl, os, sys
fd = os.open(sys.argv[1], os.O_RDONLY)
fcntl.flock(fd, fcntl.LOCK_EX)
print("locked", flush=True)
sys.stdin.read()
"""


class Port:
    is_open = False

    def __init__(self, name):
        self.name = name

    def setBaudRate(self, baud):
        self.is_open = True
        return True

    def closePort(self):
        self.is_open = False


class Handler:
    def __init__(self, port):
        self.port = port
        self.failure = None
        self.writes = []
        self.memory = [0] * 256

    @staticmethod
    def scs_tohost(value, bit):
        return -(value & ~(1 << bit)) if value & (1 << bit) else value

    def readTxRx(self, i, address, length):
        if self.failure == "comm":
            return [], -6, 0
        data = self.memory[address : address + length]
        if self.failure == "short":
            data.pop()
        return data, 0, 4 if self.failure == "servo" else 0

    def ping(self, i):
        return 777, 0, 0

    def ReadPos(self, i):
        return (
            (0, -6, 0)
            if self.failure == "comm"
            else ((0, 0, 4) if self.failure == "servo" else (100 + i, 0, 0))
        )

    def getTxRxResult(self, comm):
        return "timeout"

    def WritePosEx(self, i, position, speed, acceleration):
        self.writes.append(("position", i, position, speed, acceleration))
        return (
            (-6, 0) if self.failure == "comm" else ((0, 4) if self.failure == "servo" else (0, 0))
        )

    def write1ByteTxRx(self, i, address, value):
        self.writes.append(("register", i, address, value))
        return (0, 4) if self.failure == "servo" else (0, 0)

    def read1ByteTxRx(self, i, address):
        return 0, 0, 0

    def write2ByteTxRx(self, i, address, value):
        self.writes.append(("register2", i, address, value))
        self.memory[address : address + 2] = [value & 255, value >> 8]
        return 0, 0

    def read2ByteTxRx(self, i, address):
        if address == 48:
            return self.memory[48] | self.memory[49] << 8, 0, 0
        if address == 42:
            return (
                2000 + i,
                -6 if self.failure == "comm" else 0,
                4 if self.failure == "servo" else 0,
            )
        return {31: 0x800 | 585, 9: 806, 11: 3053}[address], 0, 0

    def ReadMoving(self, i):
        return 0, 0, 0


class BusTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        # A real file stands in for the device so the inter-process lock can be exercised.
        self.port = Path(directory.name) / "ttyFAKE"
        self.port.touch()
        self.sdk = SimpleNamespace(PortHandler=Port, sms_sts=Handler, COMM_SUCCESS=0)
        self.bus = FeetechBus(str(self.port), sdk=self.sdk)

    def untouched_bus(self, port=None):
        sdk = SimpleNamespace(PortHandler=Mock(side_effect=Port), sms_sts=Handler, COMM_SUCCESS=0)
        return FeetechBus(str(port or self.port), sdk=sdk), sdk.PortHandler

    def test_second_bus_on_the_same_port_stops_before_the_sdk_touches_it(self):
        other, port_handler = self.untouched_bus()
        with self.bus:
            with self.assertRaisesRegex(BusError, "in use by another process"):
                other.open()
            port_handler.assert_not_called()
            self.assertIsNone(other._port)
        other.open()  # Closing the first bus releases the port.
        other.close()

    def test_port_held_by_another_process_is_rejected_until_released(self):
        holder = subprocess.Popen(
            [sys.executable, "-c", HOLD_LOCK, str(self.port)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
        )
        self.addCleanup(holder.kill)
        self.assertEqual(holder.stdout.readline().strip(), "locked")
        other, port_handler = self.untouched_bus()
        with self.assertRaisesRegex(BusError, "in use by another process"):
            other.open()
        port_handler.assert_not_called()
        holder.stdin.close()
        holder.wait(timeout=5)
        with self.bus:
            self.assertEqual(self.bus.ping([1]), {1: 777})

    def test_failed_open_releases_the_port(self):
        with patch.object(Port, "setBaudRate", return_value=False):
            with self.assertRaisesRegex(BusError, "Unsupported baud rate"):
                self.bus.open()
        with self.untouched_bus()[0]:
            pass

    def test_missing_port_fails_before_creating_the_sdk_port(self):
        bus, port_handler = self.untouched_bus(self.port.parent / "absent")
        with self.assertRaisesRegex(BusError, "Cannot open"):
            bus.open()
        port_handler.assert_not_called()

    def test_read_and_cleanup_without_writes(self):
        with self.bus as bus:
            self.assertEqual(bus.ping([1, 2]), {1: 777, 2: 777})
            sample = bus.read_positions([1, 2])
            self.assertEqual(sample.positions_ticks, {1: 101, 2: 102})
            port = bus._port
            self.assertEqual(bus._handler.writes, [])
        self.assertFalse(port.is_open)
        self.bus.close()

    def test_failures_never_return_zero_sample(self):
        for failure in ("comm", "servo"):
            with self.bus as bus:
                bus._handler.failure = failure
                with self.assertRaisesRegex(BusError, "ID=1"):
                    bus.read_positions([1])

    def test_invalid_ids_before_access(self):
        for ids in ([], [1, 1], [0], [True]):
            with self.assertRaises(ValueError):
                self.bus.read_positions(ids)

    def test_requires_open(self):
        with self.assertRaises(BusError):
            self.bus.read_positions([1])

    def test_configuration_sign_magnitude_offset(self):
        with self.bus as bus:
            config = bus.read_configuration([3])[3]
            self.assertEqual(config["homing_offset"], -585)
            self.assertEqual(config["range_min"], 806)
            self.assertEqual(config["torque_enabled"], 0)

    def test_existing_goal_and_moving_are_read_without_writes(self):
        with self.bus as bus:
            self.assertEqual(
                bus.read_motion_status([1, 2]),
                {
                    1: {"goal_ticks": 2001, "moving": False},
                    2: {"goal_ticks": 2002, "moving": False},
                },
            )
            self.assertEqual(bus._handler.writes, [])
            for failure in ("comm", "servo"):
                bus._handler.failure = failure
                with self.assertRaises(BusError):
                    bus.read_motion_status([1])

    def test_motor_feedback_decodes_signed_values_and_units_without_writes(self):
        with self.bus as bus:
            memory = bus._handler.memory
            for address, value in {
                42: 2100,
                48: 750,
                56: 2000,
                58: 0x8000 | 120,
                60: 0x400 | 500,
                69: 0x8000 | 20,
            }.items():
                memory[address : address + 2] = [value & 255, value >> 8]
            memory[40] = 1
            memory[62], memory[63], memory[65], memory[66] = 121, 43, 1, 1
            bus._handler.failure = "servo"
            record = bus.read_motor_status([6])[6]
            self.assertEqual(record["position_ticks"], 2000)
            self.assertEqual(record["speed_raw"], -120)
            self.assertEqual(record["load_pct"], -50.0)
            self.assertEqual(record["current_ma"], -130.0)
            self.assertAlmostEqual(record["voltage_v"], 12.1)
            self.assertEqual(record["temperature_c"], 43)
            self.assertEqual(record["torque_enabled"], 1)
            self.assertEqual(record["torque_limit_pct"], 75)
            self.assertEqual(record["packet_error"], 4)
            self.assertEqual(record["servo_status"], 1)
            self.assertEqual(bus._handler.writes, [])
            with self.assertRaises(BusError):
                bus.read_positions([6])  # Faults remain fatal for control feedback.

    def test_motor_diagnostics_reject_timeouts_and_incomplete_packets(self):
        with self.bus as bus:
            for failure in ("comm", "short"):
                bus._handler.failure = failure
                for read in (bus.read_motor_status, bus.read_motor_info):
                    with self.assertRaises(BusError):
                        read([1])
            self.assertEqual(bus._handler.writes, [])

    def test_motor_info_reads_protection_limits_and_identity_without_writes(self):
        with self.bus as bus:
            memory = bus._handler.memory
            memory[0:2] = [3, 9]
            for address, value in {3: 777, 9: 800, 11: 3200, 28: 400}.items():
                memory[address : address + 2] = [value & 255, value >> 8]
            memory[13:16] = [70, 140, 40]
            record = bus.read_motor_info([1])[1]
            self.assertEqual(record["model_number"], 777)
            self.assertEqual(record["firmware_version"], "3.9")
            self.assertEqual(record["hardware_min_ticks"], 800)
            self.assertEqual(record["hardware_max_ticks"], 3200)
            self.assertEqual(record["temperature_limit_c"], 70)
            self.assertEqual(record["max_voltage_v"], 14)
            self.assertEqual(record["current_limit_ma"], 2600)
            self.assertEqual(bus._handler.writes, [])

    def test_write_checks_entire_batch_before_access(self):
        with self.bus as bus:
            for targets, speeds in (
                ({1: 100, 2: -1}, {1: 10, 2: 10}),
                ({1: 100}, {1: 0}),
                ({1: True}, {1: 10}),
                ({1: 100}, {1: True}),
                ({1: 100, 2: 100}, {1: 10}),  # No speed for ID 2.
            ):
                with self.assertRaises(ValueError):
                    bus.write_positions(targets, speeds=speeds)
            self.assertEqual(bus._handler.writes, [])
            bus.write_positions({2: 1500, 6: 2100}, speeds={2: 120, 6: 90})
            bus.set_torque_enabled([2, 6], True)
            self.assertEqual(
                bus._handler.writes,
                [
                    ("position", 2, 1500, 120, 10),
                    ("position", 6, 2100, 90, 10),
                    ("register", 2, 40, 1),
                    ("register", 6, 40, 1),
                ],
            )

    def test_write_failure_does_not_continue_to_next_motor(self):
        with self.bus as bus:
            for failure in ("comm", "servo"):
                bus._handler.failure = failure
                bus._handler.writes.clear()
                with self.assertRaisesRegex(BusError, "ID=1"):
                    bus.write_positions({1: 1500, 2: 1500}, speeds={1: 100, 2: 100})
                self.assertEqual(len(bus._handler.writes), 1)

    def test_torque_limit_writes_two_byte_sram_and_reads_it_back(self):
        with self.bus as bus:
            for value in (0, 200, 500, 1000):
                bus.set_torque_limit(6, value)
                status = bus.read_motor_status([6])[6]
                self.assertEqual(status["torque_limit_pct"], value / 10)
            self.assertEqual(
                bus._handler.writes,
                [("register2", 6, 48, value) for value in (0, 200, 500, 1000)],
            )

    def test_invalid_torque_limit_is_rejected_before_access(self):
        for motor_id, value in ((0, 200), (True, 200), (6, -1), (6, 1001), (6, 20.5), (6, True)):
            with self.subTest(motor_id=motor_id, value=value):
                with self.assertRaises(ValueError):
                    self.bus.set_torque_limit(motor_id, value)

    def test_torque_limit_requires_successful_write_and_matching_readback(self):
        with self.bus as bus:
            for result in ((-7, 0), (0, 32)):
                with (
                    patch.object(bus._handler, "write2ByteTxRx", return_value=result),
                    patch.object(bus._handler, "read2ByteTxRx") as read,
                ):
                    with self.assertRaisesRegex(BusError, "write torque limit ID=6"):
                        bus.set_torque_limit(6, 200)
                    read.assert_not_called()
            for result, message in (
                ((200, -6, 0), "read torque limit ID=6"),
                ((200, 0, 32), "read torque limit ID=6"),
                ((500, 0, 0), "readback mismatch ID=6"),
            ):
                with patch.object(bus._handler, "read2ByteTxRx", return_value=result):
                    with self.assertRaisesRegex(BusError, message):
                        bus.set_torque_limit(6, 200)


if __name__ == "__main__":
    unittest.main()
