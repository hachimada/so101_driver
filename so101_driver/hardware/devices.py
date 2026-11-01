"""Discover USB serial adapters and resolve their serial numbers without opening them."""

import argparse
import shutil
import unicodedata
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from serial.tools import list_ports

from ..paths import config_directory, config_path

ROLES = ("leader", "follower")
# Created from the installed templates when missing (see docs/so101_setup.md).
CONFIG_NAMES = ("leader", "follower", "teleop")


@dataclass(frozen=True)
class UsbSerialDevice:
    device_id: str | None
    port: str
    description: str
    vid: int
    pid: int


def list_devices():
    devices = [
        UsbSerialDevice(p.serial_number or None, p.device, p.description or "", p.vid, p.pid)
        for p in list_ports.comports()
        if p.vid is not None and p.pid is not None
    ]
    return sorted(devices, key=lambda d: (d.device_id or "", d.port))


def resolve_device_id(device_id):
    if not isinstance(device_id, str) or not device_id.strip():
        raise ValueError("Set device_id to the USB serial number")
    device_id = device_id.strip()
    matches = [device for device in list_devices() if device.device_id == device_id]
    if not matches:
        raise ValueError(f"USB device ID {device_id!r} not found; check it with so101_list_devices")
    if len(matches) != 1:
        ports = ", ".join(device.port for device in matches)
        raise ValueError(f"USB device ID {device_id!r} is shared by several ports: {ports}")
    return matches[0].port


def configured_device_id(role):
    """device_id in the machine-specific YAML, or None when the file has not been created."""
    path = config_path(role)
    if not path.is_file():
        return None
    config = yaml.safe_load(path.read_text())
    device_id = config[f"/{role}/so101_{role}"]["ros__parameters"]["device_id"]
    return device_id.strip() if isinstance(device_id, str) else ""


def read_role_configs():
    """Map each role to (usable device_id or "", status shown to the user)."""
    configs = {}
    for role in ROLES:
        try:
            device_id = configured_device_id(role)
        except (OSError, KeyError, TypeError, yaml.YAMLError):
            configs[role] = ("", "unreadable; check the YAML syntax")
            continue
        if device_id is None:
            status = "not created"
        elif device_id:
            status = f'device_id: "{device_id}"'
        else:
            status = "device_id not set"
        configs[role] = (device_id or "", status)
    return configs


def create_missing_configs():
    """Copy templates only for files that do not exist yet; an existing file is never replaced."""
    templates = Path(get_package_share_directory("so101_driver")) / "config"
    created = []
    for name in CONFIG_NAMES:
        path = config_path(name)
        if path.exists():
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        try:
            with (
                (templates / f"{name}.template.yaml").open("rb") as source,
                path.open("xb") as target,
            ):
                shutil.copyfileobj(source, target)
        except FileExistsError:
            continue
        created.append(path)
    return created


def display_width(text):
    # Full-width characters such as Japanese take two terminal columns.
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in text)


def pad(text, width):
    return text + " " * (width - display_width(text))


def print_devices(devices, configs=None):
    counts = Counter(d.device_id for d in devices if d.device_id)
    roles = {}
    for role, (device_id, _) in (configs or {}).items():
        if device_id:
            roles.setdefault(device_id, []).append(role)
    rows = [("DEVICE ID", "PORT", "USB VID:PID", "CURRENT CONFIG", "DEVICE")]
    for device in devices:
        identity = device.device_id or "(no serial number)"
        if device.device_id and counts[device.device_id] > 1:
            identity += " (duplicate, unusable)"
        rows.append(
            (
                identity,
                device.port,
                f"{device.vid:04X}:{device.pid:04X}",
                ", ".join(roles.get(device.device_id, ["-"])),
                device.description,
            )
        )
    widths = [max(display_width(row[i]) for row in rows) for i in range(4)]
    for row in rows:
        print("  ".join(pad(value, width) for value, width in zip(row[:4], widths)) + "  " + row[4])


def print_config_guidance(device_id, configs):
    """Point to the YAML holding this ID, or prepare the files and ask to write it by hand."""
    registered = [role for role, (configured, _) in configs.items() if configured == device_id]
    if len(registered) == 1:
        role = registered[0]
        print(f"This ID is set in the {role} config: {config_path(role)}")
        return
    if registered:
        print("This ID is set in both the leader and follower configs.")
        print("Fix device_id in the config of the arm that you did not unplug.")
    else:
        print("This ID is not in any config file yet.")
        try:
            created = create_missing_configs()
        except OSError as exc:
            print(f"Cannot create config files: {exc}")
            created = []
        if created:
            print("Created config files from the templates:")
            for path in created:
                print(f"   {path}")
            configs = read_role_configs()
        print("Open the config of the unplugged arm and write this ID in device_id:")
    labels = {role: f"{role}:" for role in configs}
    width = max(map(display_width, labels.values()))
    for role, (_, status) in configs.items():
        print(f"   {pad(labels[role], width)} {config_path(role)} ({status})")
    if not registered:
        print(f'   Line to write: device_id: "{device_id}"')


def find_removed(before, after):
    # A remaining adapter can get a new port number. Compare USB identities,
    # retaining counts so removal of one of several duplicate IDs is detected.
    def identity(device):
        return device.device_id, device.vid, device.pid

    missing = Counter(map(identity, before)) - Counter(map(identity, after))
    removed = []
    for device in before:
        key = identity(device)
        if missing[key] > 0:
            removed.append(device)
            missing[key] -= 1
    return removed


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Find a device ID by comparing USB devices before and after unplugging one"
    )
    parser.parse_args(argv)
    before = list_devices()
    if not before:
        print("No USB serial devices found. Connect the arm and run again.")
        return 1
    configs = read_role_configs()
    print(f"Config directory: {config_directory()}\n")
    print_devices(before, configs)
    try:
        while True:
            answer = (
                input(
                    "\nUnplug the USB cable of the arm to identify.\nEnter: check ID / q: quit > "
                )
                .strip()
                .lower()
            )
            if answer == "q":
                print("Canceled.")
                return 130
            if answer:
                continue
            removed = find_removed(before, list_devices())
            if not removed:
                print("No device was unplugged. Check the cable and press Enter again.")
                continue
            if len(removed) != 1:
                print("More than one device was unplugged. Reconnect the other devices.")
                continue
            device = removed[0]
            if not device.device_id:
                print("The unplugged device has no USB serial number and cannot be selected by ID.")
                return 1
            if sum(d.device_id == device.device_id for d in before) != 1:
                print(f"The unplugged device ID {device.device_id!r} is duplicated and unusable.")
                return 1
            print(f"\nUnplugged device ID: {device.device_id}")
            print_config_guidance(device.device_id, configs)
            print("Reconnect the USB cable.")
            return 0
    except (KeyboardInterrupt, EOFError):
        print("\nCanceled.")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
