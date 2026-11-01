"""USB cameras: the camera list in cameras.yaml, and so101_list_cameras to find their paths."""

import argparse
import fcntl
import math
import re
import shutil
import struct
from dataclasses import dataclass
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from .hardware.devices import display_width, pad
from .paths import config_path

KEYS = {"name", "device", "width", "height", "fps", "parameters"}
NAME = re.compile(r"[A-Za-z][A-Za-z0-9_]*\Z")
# Paths named after the USB port: identical cameras stay distinct, unlike /dev/v4l/by-id.
BY_PATH = Path("/dev/v4l/by-path")

# struct v4l2_capability is 104 bytes; capabilities and device_caps follow driver, card,
# bus_info and version.
VIDIOC_QUERYCAP = 0x80685600
V4L2_CAP_VIDEO_CAPTURE = 0x00000001
V4L2_CAP_DEVICE_CAPS = 0x80000000


def _captures_video(device):
    buffer = bytearray(104)
    with open(device, "rb", buffering=0) as stream:
        fcntl.ioctl(stream, VIDIOC_QUERYCAP, buffer)
    capabilities, device_caps = struct.unpack_from("<II", buffer, 84)
    if capabilities & V4L2_CAP_DEVICE_CAPS:
        capabilities = device_caps
    return bool(capabilities & V4L2_CAP_VIDEO_CAPTURE)


def video_devices():
    """Each /dev/videoN: whether it captures video, and the camera model name."""
    devices = {}
    for path in sorted(Path("/dev").glob("video*")):
        try:
            capture = _captures_video(path)
        except OSError as exc:
            devices[str(path)] = {"error": str(exc)}
            continue
        try:
            model = (Path("/sys/class/video4linux") / path.name / "name").read_text().strip()
        except OSError:
            model = ""
        devices[str(path)] = {"capture": capture, "model": model}
    return devices


def _positive(name, key, value, kinds):
    if type(value) not in kinds or not math.isfinite(value) or value <= 0:
        raise ValueError(f"Camera {name!r}: {key} must be a positive number, got {value!r}")
    return value


def load_cameras(path, devices=None, by_path=BY_PATH):
    """usb_cam parameters for each camera listed in a cameras.yaml file, keyed by name.

    devices maps /dev/videoN to video_devices() records; it is read from the system if omitted.
    """
    data = yaml.safe_load(Path(path).read_text()) or {}
    entries = data.get("cameras") if isinstance(data, dict) else None
    if not isinstance(data, dict) or not isinstance(entries or [], list):
        raise ValueError(f"{path}: list the cameras under 'cameras:'")
    if entries and devices is None:
        devices = video_devices()
    cameras = {}
    for entry in entries or []:
        if not isinstance(entry, dict):
            raise ValueError(f"{path}: each camera needs a name and a device")
        name = entry.get("name")
        if not isinstance(name, str) or not NAME.match(name):
            raise ValueError(
                f"Camera name must start with a letter and use letters, digits or _, got {name!r}"
            )
        if name in cameras:
            raise ValueError(f"Camera name {name!r} is used twice")
        unknown = set(entry) - KEYS
        if unknown:
            raise ValueError(f"Camera {name!r}: unknown keys {', '.join(sorted(unknown))}")
        device = entry.get("device")
        if not isinstance(device, str) or Path(device).parent != by_path:
            raise ValueError(
                f"Camera {name!r}: set device to the USB port path under {by_path}/ "
                f"shown by so101_list_cameras, got {device!r}"
            )
        try:
            # usb_cam 0.8.1 mishandles the relative links in /dev/v4l.
            video_device = str(Path(device).resolve(strict=True))
        except OSError:
            raise ValueError(
                f"Camera {name!r}: {device} not found. Connect the camera to the same USB port, "
                "or check the path with so101_list_cameras"
            ) from None
        info = devices.get(video_device)
        if info is None:
            raise ValueError(f"Camera {name!r}: {device} is not a video device")
        if "error" in info:
            raise ValueError(f"Camera {name!r}: cannot open {device}: {info['error']}")
        if not info["capture"]:
            raise ValueError(
                f"Camera {name!r}: {device} does not capture video; "
                "use a path shown by so101_list_cameras"
            )
        for other, parameters in cameras.items():
            if parameters["video_device"] == video_device:
                raise ValueError(f"Cameras {other!r} and {name!r} both use {video_device}")
        extra = entry.get("parameters") or {}
        if not isinstance(extra, dict):
            raise ValueError(f"Camera {name!r}: parameters must be a mapping")
        cameras[name] = {
            "video_device": video_device,
            "image_width": _positive(name, "width", entry.get("width", 640), (int,)),
            "image_height": _positive(name, "height", entry.get("height", 480), (int,)),
            "framerate": float(_positive(name, "fps", entry.get("fps", 30), (int, float))),
            # camera_info_manager then reads $ROS_HOME/camera_info/<name>.yaml if it exists.
            "camera_name": name,
            "frame_id": f"{name}_optical_frame",
            **extra,
        }
    return cameras


@dataclass(frozen=True)
class CameraPort:
    path: str  # /dev/v4l/by-path/... for cameras.yaml
    device: str  # /dev/videoN it points to now
    model: str


def camera_ports(by_path=BY_PATH, devices=None):
    """Connected devices that capture video, with one USB port path each."""
    devices = video_devices() if devices is None else devices
    ports = {}
    for link in sorted(by_path.glob("*")):
        device = str(link.resolve())
        info = devices.get(device, {})
        if not info.get("capture"):
            continue
        # udev makes both a usb- and a usbv2- link for each port; show one of them.
        if device not in ports or "-usbv2-" in ports[device].path:
            ports[device] = CameraPort(str(link), device, info.get("model", ""))
    return sorted(ports.values(), key=lambda port: port.path)


def configured_names(path, ports):
    """Camera names from cameras.yaml for the ports they point to, without validating it.

    Only USB port paths count; any other path is rejected when the cameras start.
    """
    try:
        data = yaml.safe_load(Path(path).read_text()) or {}
    except (OSError, yaml.YAMLError):
        return {}
    entries = data.get("cameras") if isinstance(data, dict) else None
    folders = {Path(port.path).parent for port in ports}
    devices = {}
    for entry in entries if isinstance(entries, list) else []:
        device = entry.get("device") if isinstance(entry, dict) else None
        if isinstance(device, str) and Path(device).parent in folders:
            devices.setdefault(str(Path(device).resolve()), []).append(str(entry.get("name")))
    return {port.path: ", ".join(devices[port.device]) for port in ports if port.device in devices}


def create_camera_config(path):
    """Copy the template; an existing file is never replaced."""
    template = Path(get_package_share_directory("so101_driver")) / "config/cameras.template.yaml"
    path.parent.mkdir(parents=True, exist_ok=True)
    with template.open("rb") as source, path.open("xb") as target:
        shutil.copyfileobj(source, target)


def print_cameras(ports, names):
    rows = [("PORT PATH (device:)", "CAMERA", "NAME IN CONFIG")]
    rows += [(port.path, port.model, names.get(port.path, "-")) for port in ports]
    widths = [max(display_width(row[i]) for row in rows) for i in range(2)]
    for path, model, name in rows:
        print(f"{pad(path, widths[0])}  {pad(model, widths[1])}  {name}")


def print_config_guidance(port, names, config):
    name = names.get(port.path)
    if name:
        print(f"This camera is set as {name!r} in {config}")
        return
    print("This camera is not in the camera list yet.")
    if not config.exists():
        try:
            create_camera_config(config)
        except OSError as exc:
            print(f"Cannot create the config file: {exc}")
        else:
            print(f"Created the config file from the template: {config}")
    print(f"Open {config} and add these lines under cameras:")
    print("  - name: <name>")
    print(f"    device: {port.path}")
    print("Replace <name> with a name for this camera, such as front or wrist.")


def main(argv=None):
    parser = argparse.ArgumentParser(description="Find a camera's USB port path by unplugging it")
    parser.parse_args(argv)
    config = config_path("cameras")
    before = camera_ports()
    if not before:
        print("No USB cameras found. Connect the camera and run again.")
        return 1
    names = configured_names(config, before)
    print(f"Config file: {config}{'' if config.exists() else ' (not created yet)'}\n")
    print_cameras(before, names)
    try:
        while True:
            answer = (
                input(
                    "\nUnplug the USB cable of the camera to identify.\nEnter: check / q: quit > "
                )
                .strip()
                .lower()
            )
            if answer == "q":
                print("Canceled.")
                return 130
            if answer:
                continue
            remaining = {port.path for port in camera_ports()}
            removed = [port for port in before if port.path not in remaining]
            if not removed:
                print("No camera was unplugged. Check the cable and press Enter again.")
                continue
            if len(removed) != 1:
                print("More than one camera was unplugged. Reconnect the other cameras.")
                continue
            port = removed[0]
            print(f"\nUnplugged camera: {port.model}")
            print(f"USB port path: {port.path}")
            print_config_guidance(port, names, config)
            print("Reconnect the camera to the same USB port; another port has another path.")
            return 0
    except (KeyboardInterrupt, EOFError):
        print("\nCanceled.")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
