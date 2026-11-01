"""Read leader joint positions using a profile created by so101_calibrate."""

from .profile import MOTOR_IDS, CalibrationProfile


class SO101Leader:
    def __init__(self, bus, profile_file):
        self.bus = bus
        self.profile = CalibrationProfile(profile_file, role="leader")

    def validate(self):
        self.profile.validate_hardware(self.bus)

    def read_joint_positions(self):
        raw = self.bus.read_positions(MOTOR_IDS).positions_ticks
        positions = self.profile.to_radians(raw)
        # The leader gripper sets the follower target, so keep it between closed and open.
        positions[-1] = self.profile.conversions["gripper"].to_clamped_radians(raw[6])
        return positions
