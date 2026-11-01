"""Terminal-guided calibration with independent live and target RViz models."""

import argparse
import sys
import threading
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from .calibration import CalibrationSession, save_profile
from .calibration_view import CalibrationView, calibration_preview
from .hardware.devices import ROLES, configured_device_id, resolve_device_id
from .hardware.feetech_bus import MAX_TICK, STS3215_MODEL_NUMBER, BusError, FeetechBus
from .paths import config_path, data_directory
from .profile import MOTOR_IDS
from .terminal import TerminalReadings


def default_output(role):
    return data_directory() / "calibration" / f"{role}.json"


def default_device_id(role):
    device_id = configured_device_id(role)
    if not device_id:
        raise ValueError(f"Pass --device-id or set device_id in {config_path(role)}")
    return device_id


def check_running_nodes(nodes):
    busy = {"so101_leader", "so101_follower", "so101_calibration"}
    if any(name in busy for name, _ in nodes):
        raise ValueError("Stop the running driver nodes and other calibration sessions first.")


def prepare_hardware(bus, role, *, disable_torque=False, reader=None):
    hardware = bus.read_configuration(MOTOR_IDS)
    if any(v["operating_mode"] != 0 for v in hardware.values()):
        raise ValueError("Some servos are not in position mode. No settings were changed.")
    if any(not 0 <= v["range_min"] < v["range_max"] <= MAX_TICK for v in hardware.values()):
        raise ValueError("Invalid servo position limits")
    active = [i for i, v in hardware.items() if v["torque_enabled"] != 0]
    if active:
        if not disable_torque:
            raise ValueError(
                f"Torque is enabled on some {role} joints. Rerun with --disable-torque "
                "to release it. No settings were changed."
            )
        wait_enter(
            f"Support the {role} arm.\nEnter: release torque / q: quit > ",
            reader=reader,
        )
        bus.set_torque_enabled(active, False)
        hardware = bus.read_configuration(MOTOR_IDS)
        if any(v["torque_enabled"] != 0 for v in hardware.values()):
            raise ValueError("Torque release not confirmed. Calibration aborted.")
    return hardware


def wait_enter(text, *, reader=None):
    while True:
        answer = (reader or input)(text).strip().lower()
        if answer == "q":
            raise KeyboardInterrupt
        if not answer:
            return


def prompt(action, text, *, button="start", reader=None):
    while True:
        wait_enter(text + f"\nEnter: {button} / q: quit > ", reader=reader)
        try:
            return action()
        except ValueError as exc:
            print(f"Cannot record: {exc}", flush=True)


def confirm_overwrite(path, *, reader=None):
    return (reader or input)(f"{path} already exists. Overwrite? [y/N]> ").strip().lower() in (
        "y",
        "yes",
    )


def save_with_confirmation(output, profile, *, overwrite_approved=False, reader=None):
    while True:
        overwrite = Path(output).exists()
        if overwrite and not overwrite_approved and not confirm_overwrite(output, reader=reader):
            return False
        try:
            save_profile(output, profile, overwrite=overwrite)
            return True
        except FileExistsError:
            # A file appeared between the existence check and atomic creation.
            continue


def run_wizard(session, output, *, overwrite_approved=False, reader=None):
    role = session.robot_role
    print("\nBlue: reference pose (fixed) / Gray: current pose after start", flush=True)
    prompt(
        session.start_from_reference,
        f"Put the {role} arm in the blue reference pose and close the gripper.",
        reader=reader,
    )
    while True:
        wait_enter(
            "Recording. Move every joint and the gripper through its comfortable range.\n"
            "Enter: finish and save / q: quit > ",
            reader=reader,
        )
        try:
            session.end_ranges()
            break
        except ValueError as exc:
            print(f"Not enough data recorded: {exc}", flush=True)
            prompt(
                session.begin_ranges,
                "Record more movement for the joints listed above.",
                button="resume",
                reader=reader,
            )
    profile = session.profile()
    if not save_with_confirmation(
        output, profile, overwrite_approved=overwrite_approved, reader=reader
    ):
        print("Overwrite canceled. The existing file was not changed.")
        return
    print(f"Saved: {output}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--role",
        choices=ROLES,
        default="leader",
        help="Arm to calibrate (default: leader)",
    )
    parser.add_argument(
        "--device-id",
        help=f"USB serial number from so101_list_devices (default: {config_path('<role>')})",
    )
    parser.add_argument(
        "--output",
        help=f"Output file (default: {data_directory() / 'calibration'}/<role>.json)",
    )
    parser.add_argument(
        "--disable-torque",
        action="store_true",
        help="Release enabled torque after confirmation; it stays released after calibration",
    )
    parser.add_argument(
        "--no-rviz", action="store_true", help="Calibrate without opening the RViz preview"
    )
    args = parser.parse_args(argv)
    try:
        output = (
            (Path(args.output) if args.output is not None else default_output(args.role))
            .expanduser()
            .resolve()
        )
    except (OSError, ValueError) as exc:
        print(f"ERROR: Cannot determine the output file: {exc}", file=sys.stderr)
        return 1
    print(f"Calibrating: {args.role}", flush=True)
    print(f"Calibration file: {output}", flush=True)
    overwrite_approved = output.exists()
    try:
        if overwrite_approved and not confirm_overwrite(output):
            print("Overwrite canceled. The existing file was not changed.")
            return 0
    except (KeyboardInterrupt, EOFError):
        print("\nCalibration canceled.")
        return 130
    view = None
    stop = threading.Event()
    spin_errors = []
    try:
        device_id = args.device_id if args.device_id is not None else default_device_id(args.role)
        port = resolve_device_id(device_id)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        print(f"ERROR: Cannot read the device settings: {exc}", file=sys.stderr)
        return 1
    print(f"Device ID: {device_id} / port: {port}", flush=True)
    bus = FeetechBus(port)
    rclpy.init(args=[])
    try:
        guard = Node("so101_calibration_guard")
        try:
            for _ in range(5):
                rclpy.spin_once(guard, timeout_sec=0.1)
            check_running_nodes(guard.get_node_names_and_namespaces())
        finally:
            guard.destroy_node()
        bus.open()
        if any(v != STS3215_MODEL_NUMBER for v in bus.ping(MOTOR_IDS).values()):
            raise ValueError("Found a servo that is not an STS3215")
        hardware = prepare_hardware(bus, args.role, disable_torque=args.disable_torque)
        session = CalibrationSession(
            hardware, robot_role=args.role, source_device_id=device_id, source_port=port
        )
        terminal = TerminalReadings()
        view = CalibrationView(bus, session, terminal=terminal)
        share = Path(get_package_share_directory("so101_driver"))
        with calibration_preview(share, show_rviz=not args.no_rviz) as children:

            def spin():
                try:
                    while not stop.is_set() and rclpy.ok():
                        rclpy.spin_once(view, timeout_sec=0.1)
                        if any(p.poll() is not None for p in children):
                            session.fail(
                                "A visualization process stopped; cancel and restart calibration."
                            )
                            return
                except Exception as exc:
                    spin_errors.append(exc)
                    session.fail(f"Visualization stopped: {exc}")

            thread = threading.Thread(target=spin, daemon=True)
            thread.start()
            try:
                run_wizard(
                    session, output, overwrite_approved=overwrite_approved, reader=terminal.read
                )
            finally:
                stop.set()
                thread.join(timeout=3)
        if spin_errors:
            raise spin_errors[0]
    except (KeyboardInterrupt, EOFError):
        print("\nCalibration canceled.")
        return 130
    except (BusError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if view is not None:
            view.destroy_node()
        bus.close()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
