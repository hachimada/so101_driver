import contextlib
import io
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from so101_driver.hardware.devices import display_width, list_devices, main, resolve_device_id


def port(device, serial_number, *, usb=True):
    return SimpleNamespace(
        device=device,
        serial_number=serial_number,
        vid=0x1A86 if usb else None,
        pid=0x55D3 if usb else None,
        description="USB Single Serial" if usb else "n/a",
    )


class DeviceTests(unittest.TestCase):
    def setUp(self):
        ports = patch("so101_driver.hardware.devices.list_ports.comports")
        self.ports = ports.start()
        self.addCleanup(ports.stop)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.data = Path(directory.name) / "so101_driver"
        data = patch("so101_driver.paths.data_directory", return_value=self.data)
        data.start()
        self.addCleanup(data.stop)
        # The repository's config directory holds the same templates as the installed share.
        self.templates = Path(__file__).resolve().parents[1] / "config"
        share = patch(
            "so101_driver.hardware.devices.get_package_share_directory",
            return_value=str(self.templates.parent),
        )
        share.start()
        self.addCleanup(share.stop)

    def write_config(self, name, text):
        path = self.data / "config" / f"{name}.yaml"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
        return path

    def write_device_id(self, role, device_id):
        return self.write_config(
            role, f'/{role}/so101_{role}:\n  ros__parameters:\n    device_id: "{device_id}"\n'
        )

    def unplug(self, before, after):
        self.ports.side_effect = [before, after]
        with (
            patch("builtins.input", return_value=""),
            contextlib.redirect_stdout(io.StringIO()) as output,
        ):
            self.assertEqual(main([]), 0)
        return output.getvalue()

    def test_enumerates_usb_only_and_keeps_devices_without_serial_visible(self):
        self.ports.return_value = [
            port("/dev/ttyS0", None, usb=False),
            port("/dev/ttyACM1", "follower-id"),
            port("/dev/ttyACM0", "leader-id"),
            port("/dev/ttyUSB0", None),
        ]
        devices = list_devices()
        self.assertEqual([d.device_id for d in devices], [None, "follower-id", "leader-id"])
        self.assertEqual(devices[1].vid, 0x1A86)

    def test_exact_serial_resolves_again_after_port_number_changes(self):
        self.ports.return_value = [port("/dev/ttyACM0", "leader-id")]
        self.assertEqual(resolve_device_id("leader-id"), "/dev/ttyACM0")
        self.ports.return_value = [port("/dev/ttyACM4", "leader-id")]
        self.assertEqual(resolve_device_id("leader-id"), "/dev/ttyACM4")

    def test_missing_or_partial_id_never_falls_back_to_another_device(self):
        self.ports.return_value = [port("/dev/ttyACM0", "leader-id")]
        for identity in ("leader", "follower-id", "/dev/ttyACM0"):
            with self.assertRaisesRegex(ValueError, "not found"):
                resolve_device_id(identity)

    def test_duplicate_serial_or_multiple_interfaces_are_rejected(self):
        self.ports.return_value = [
            port("/dev/ttyACM0", "same-id"),
            port("/dev/ttyACM1", "same-id"),
        ]
        with self.assertRaisesRegex(ValueError, "shared by several ports"):
            resolve_device_id("same-id")

    def test_empty_or_non_string_id_is_rejected_before_enumeration(self):
        for identity in ("", "  ", None, 1234):
            with self.assertRaisesRegex(ValueError, "USB serial number"):
                resolve_device_id(identity)
        self.ports.assert_not_called()

    def test_listing_shows_ids_ports_and_unusable_ids_without_opening_devices(self):
        self.ports.return_value = [
            port("/dev/ttyACM0", "same-id"),
            port("/dev/ttyACM1", "same-id"),
            port("/dev/ttyUSB0", None),
        ]
        output = io.StringIO()
        with (
            patch("serial.Serial") as serial,
            patch("builtins.input", return_value="q"),
            contextlib.redirect_stdout(output),
        ):
            self.assertEqual(main([]), 130)
            serial.assert_not_called()
        for text in ("same-id", "/dev/ttyACM1", "1A86:55D3", "duplicate", "no serial number"):
            self.assertIn(text, output.getvalue())

    def test_scans_before_prompt_and_identifies_the_removed_device_without_opening_ports(self):
        leader = port("/dev/ttyACM0", "leader-id")
        follower = port("/dev/ttyACM1", "follower-id")
        self.ports.side_effect = [[leader, follower], [leader]]

        def unplug(prompt):
            self.assertEqual(self.ports.call_count, 1)
            self.assertIn("Unplug the USB cable", prompt)
            self.assertIn("Enter", prompt)
            return ""

        with (
            patch("builtins.input", side_effect=unplug),
            patch("serial.Serial") as serial,
            contextlib.redirect_stdout(io.StringIO()) as output,
        ):
            self.assertEqual(main([]), 0)
            serial.assert_not_called()
        self.assertIn("Unplugged device ID: follower-id", output.getvalue())
        self.assertNotIn("Unplugged device ID: leader-id", output.getvalue())
        self.assertIn("Reconnect the USB cable", output.getvalue())

    def test_remaining_port_renumbering_and_new_devices_do_not_change_identification(self):
        self.ports.side_effect = [
            [port("/dev/ttyACM0", "leader-id"), port("/dev/ttyACM1", "follower-id")],
            [port("/dev/ttyACM0", "follower-id"), port("/dev/ttyACM2", "new-id")],
        ]
        with (
            patch("builtins.input", return_value=""),
            contextlib.redirect_stdout(io.StringIO()) as output,
        ):
            self.assertEqual(main([]), 0)
        self.assertIn("Unplugged device ID: leader-id", output.getvalue())

    def test_no_change_can_be_retried_against_the_initial_scan(self):
        device = port("/dev/ttyACM0", "leader-id")
        self.ports.side_effect = [[device], [device], []]
        with (
            patch("builtins.input", return_value="") as prompt,
            contextlib.redirect_stdout(io.StringIO()) as output,
        ):
            self.assertEqual(main([]), 0)
        self.assertEqual(prompt.call_count, 2)
        self.assertIn("No device was unplugged", output.getvalue())
        self.assertIn("Unplugged device ID: leader-id", output.getvalue())

    def test_multiple_removals_require_reconnecting_the_other_device(self):
        leader = port("/dev/ttyACM0", "leader-id")
        follower = port("/dev/ttyACM1", "follower-id")
        self.ports.side_effect = [[leader, follower], [], [follower]]
        with (
            patch("builtins.input", return_value="") as prompt,
            contextlib.redirect_stdout(io.StringIO()) as output,
        ):
            self.assertEqual(main([]), 0)
        self.assertEqual(prompt.call_count, 2)
        self.assertIn("More than one device", output.getvalue())
        self.assertIn("Unplugged device ID: leader-id", output.getvalue())

    def test_missing_or_duplicate_serial_is_not_reported_as_usable(self):
        duplicate = port("/dev/ttyACM1", "same-id")
        for before, after, message in (
            ([port("/dev/ttyUSB0", None)], [], "no USB serial number"),
            ([port("/dev/ttyACM0", "same-id"), duplicate], [duplicate], "duplicated"),
        ):
            self.ports.side_effect = [before, after]
            with (
                patch("builtins.input", return_value=""),
                contextlib.redirect_stdout(io.StringIO()) as output,
            ):
                self.assertEqual(main([]), 1)
            self.assertIn(message, output.getvalue())
            self.assertNotIn("Unplugged device ID:", output.getvalue())

    def test_interrupt_or_end_of_input_cancels_without_a_second_scan(self):
        self.ports.return_value = [port("/dev/ttyACM0", "leader-id")]
        for interruption in (KeyboardInterrupt, EOFError):
            self.ports.reset_mock()
            with (
                patch("builtins.input", side_effect=interruption),
                contextlib.redirect_stdout(io.StringIO()) as output,
            ):
                self.assertEqual(main([]), 130)
            self.ports.assert_called_once()
            self.assertIn("Canceled", output.getvalue())

    def test_unregistered_id_creates_files_from_templates_and_asks_to_write_the_id(self):
        leader, follower = port("/dev/ttyACM0", "leader-id"), port("/dev/ttyACM1", "follower-id")
        output = self.unplug([leader, follower], [leader])
        config = self.data / "config"
        self.assertIn(f"Config directory: {config}", output)
        self.assertIn("not in any config file yet", output)
        self.assertIn("Created config files from the templates", output)
        for name in ("leader", "follower", "teleop"):
            self.assertIn(f"   {config}/{name}.yaml\n", output)
            self.assertEqual(
                (config / f"{name}.yaml").read_text(),
                (self.templates / f"{name}.template.yaml").read_text(),
            )
        self.assertIn("write this ID in device_id", output)
        self.assertIn(f"{config}/follower.yaml (device_id not set)", output)
        self.assertIn('Line to write: device_id: "follower-id"', output)

    def test_existing_files_are_kept_and_show_their_current_ids(self):
        leader_config = self.write_device_id("leader", "leader-id")
        teleop_config = self.write_config("teleop", "/so101_teleop:\n  ros__parameters: {}\n")
        before = leader_config.read_text(), teleop_config.read_text()
        leader, follower = port("/dev/ttyACM0", "leader-id"), port("/dev/ttyACM1", "follower-id")
        output = self.unplug([leader, follower], [leader])
        self.assertEqual((leader_config.read_text(), teleop_config.read_text()), before)
        created = output.split("templates:\n", 1)[1].split("Open the config", 1)[0]
        self.assertEqual(created.split(), [str(self.data / "config/follower.yaml")])
        self.assertIn(f'{self.data}/config/leader.yaml (device_id: "leader-id")', output)
        # The role column keeps the remaining columns aligned despite full-width labels.
        self.assertIn("CURRENT CONFIG", output)
        rows = [line for line in output.splitlines() if line.endswith("USB Single Serial")]
        self.assertEqual(len(rows), 2)
        self.assertIn("  leader  ", next(row for row in rows if row.startswith("leader-id")))
        self.assertEqual(
            len({display_width(row[: row.index("USB Single Serial")]) for row in rows}), 1
        )

    def test_registered_id_points_to_its_file_instead_of_setup_steps(self):
        path = self.write_device_id("leader", "leader-id")
        leader, follower = port("/dev/ttyACM0", "leader-id"), port("/dev/ttyACM1", "follower-id")
        output = self.unplug([leader, follower], [follower])
        self.assertIn(f"This ID is set in the leader config: {path}", output)
        self.assertNotIn("Created config files", output)
        self.assertNotIn("Line to write", output)
        self.assertFalse((self.data / "config/follower.yaml").exists())

    def test_unset_or_unreadable_files_are_reported(self):
        self.write_device_id("leader", "")
        broken = self.write_config("follower", "/follower/so101_follower: [unclosed\n")
        device = port("/dev/ttyACM0", "leader-id")
        output = self.unplug([device], [])
        self.assertIn("leader.yaml (device_id not set)", output)
        self.assertIn("follower.yaml (unreadable", output)
        self.assertEqual(broken.read_text(), "/follower/so101_follower: [unclosed\n")
        self.assertIn('Line to write: device_id: "leader-id"', output)

    def test_id_written_for_both_roles_asks_to_fix_one(self):
        self.write_device_id("leader", "same-arm")
        self.write_device_id("follower", "same-arm")
        output = self.unplug([port("/dev/ttyACM0", "same-arm")], [])
        self.assertIn("both the leader and follower", output)
        self.assertIn("leader, follower", output)

    def test_copy_failure_is_reported_and_the_target_files_are_still_shown(self):
        with patch(
            "so101_driver.hardware.devices.get_package_share_directory",
            return_value=str(self.data / "missing-share"),
        ):
            output = self.unplug([port("/dev/ttyACM0", "leader-id")], [])
        self.assertIn("Cannot create config files", output)
        self.assertIn("leader.yaml (not created)", output)
        self.assertFalse((self.data / "config/leader.yaml").exists())

    def test_empty_list_has_a_clear_message(self):
        self.ports.return_value = []
        with contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(main([]), 1)
        self.assertIn("No USB serial devices found", output.getvalue())
