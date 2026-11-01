"""Locations of machine-specific settings and calibration files."""

import os
from pathlib import Path


def data_directory():
    # Like camera_info_manager, keep per-machine files under ROS_HOME (default ~/.ros).
    # They then do not depend on the clone location, working directory or install type.
    return Path(os.environ.get("ROS_HOME") or "~/.ros").expanduser() / "so101_driver"


def config_directory():
    return data_directory() / "config"


def config_path(name):
    return config_directory() / f"{name}.yaml"


def calibration_profile_path(path):
    path = Path(path).expanduser()
    return path if path.is_absolute() else data_directory() / path


def default_config_path(name, share_directory):
    """Use the machine-specific settings when present, otherwise the installed template."""
    config = config_path(name)
    if config.is_file():
        return config
    return Path(share_directory) / "config" / f"{name}.template.yaml"
