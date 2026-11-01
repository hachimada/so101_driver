import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

try:
    from so101_driver.calibration import CalibrationSession
    from so101_driver.calibration_cli import (
        check_running_nodes,
        confirm_overwrite,
        default_device_id,
        default_output,
        main,
        prepare_hardware,
        run_wizard,
        save_with_confirmation,
        wait_enter,
    )

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, "Requires ROS2 Humble")
class WizardTests(unittest.TestCase):
    def setUp(self):
        resolver = patch("so101_driver.calibration_cli.resolve_device_id", return_value="/fake")
        self.resolve = resolver.start()
        self.addCleanup(resolver.stop)

    def test_missing_or_duplicate_device_does_not_initialize_ros_or_hardware(self):
        with tempfile.TemporaryDirectory() as directory:
            self.resolve.side_effect = ValueError(
                "USB device ID not found or shared by several ports"
            )
            with (
                patch("so101_driver.calibration_cli.FeetechBus") as bus,
                patch("so101_driver.calibration_cli.rclpy.init") as init,
                contextlib.redirect_stdout(io.StringIO()),
                contextlib.redirect_stderr(io.StringIO()),
            ):
                self.assertEqual(
                    main(["--device-id", "absent", "--output", str(Path(directory) / "new.json")]),
                    1,
                )
                bus.assert_not_called()
                init.assert_not_called()

    def test_guided_flow_saves_closed_open_and_reference_without_motor_writes(self):
        hardware = {
            i: dict(
                homing_offset=10, range_min=500, range_max=3500, torque_enabled=0, operating_mode=0
            )
            for i in range(1, 7)
        }
        s = CalibrationSession(hardware)
        prompts = []
        instructions = []

        def answer(text):
            prompts.append(s.recording_ranges)
            instructions.append(text)
            raw = {i: 2000 for i in range(1, 7)}
            if s.recording_ranges:
                for tick in range(2000, 2110, 10):
                    s.accept({i: tick + i for i in range(1, 7)})
            else:
                s.accept(raw)
            return ""

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "calibration.json"
            with (
                patch("builtins.input", side_effect=answer),
                contextlib.redirect_stdout(io.StringIO()),
            ):
                run_wizard(s, path)
            profile = json.loads(path.read_text())
            self.assertEqual(profile["gripper"]["closed_ticks"], 2000)
            self.assertEqual(profile["gripper"]["open_ticks"], 2106)
            self.assertEqual(profile["joints"]["shoulder_pan"]["zero_ticks"], 2000)
            self.assertEqual(profile["hardware_configuration"]["1"]["homing_offset"], 10)
            self.assertEqual(prompts, [False, True])
            self.assertIn("Enter: start", instructions[0])
            self.assertIn("Enter: finish and save", instructions[1])

    def test_follower_wizard_records_its_role_and_preserves_simple_flow(self):
        from so101_driver.so101_follower import SO101Follower

        hardware = {
            i: dict(
                homing_offset=0, range_min=0, range_max=4095, torque_enabled=0, operating_mode=0
            )
            for i in range(1, 7)
        }
        session = CalibrationSession(
            hardware, robot_role="follower", source_device_id="follower-id", source_port="/follower"
        )
        prompts = []

        def answer(text):
            prompts.append(text)
            if session.recording_ranges:
                for step in range(11):
                    session.accept({i: 2000 + step * 10 for i in range(1, 7)})
            else:
                session.accept({i: 2000 for i in range(1, 7)})
            return ""

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "calibration/follower.json"
            with contextlib.redirect_stdout(io.StringIO()):
                run_wizard(session, path, reader=answer)
            profile = json.loads(path.read_text())
            self.assertEqual(profile["robot_role"], "follower")
            self.assertEqual(profile["source_port"], "/follower")
            self.assertEqual(profile["source_device_id"], "follower-id")
            self.assertEqual(len(prompts), 2)
            self.assertIn("Put the follower arm in the blue reference pose", prompts[0])
            bus = Mock()
            bus.ping.return_value = {i: 777 for i in range(1, 7)}
            bus.read_configuration.return_value = hardware
            robot = SO101Follower(bus, path)
            robot.validate_control()
            bus.write_positions.assert_not_called()
            bus.set_torque_enabled.assert_not_called()

    def test_role_defaults_use_sibling_json_files_in_the_data_directory(self):
        data = Path("/ros_home/so101_driver")
        with patch("so101_driver.calibration_cli.data_directory", return_value=data):
            self.assertEqual(default_output("leader"), data / "calibration/leader.json")
            self.assertEqual(default_output("follower"), data / "calibration/follower.json")

    def test_missing_config_requires_an_explicit_device_id(self):
        with (
            tempfile.TemporaryDirectory() as directory,
            patch("so101_driver.paths.data_directory", return_value=Path(directory)),
        ):
            for role in ("leader", "follower"):
                with self.assertRaisesRegex(ValueError, "--device-id"):
                    default_device_id(role)

    def test_role_device_id_lookup_uses_config_without_hardware_specific_constants(self):
        with tempfile.TemporaryDirectory() as directory:
            share = Path(directory)
            (share / "config").mkdir()
            for role in ("leader", "follower"):
                (share / "config" / f"{role}.yaml").write_text(
                    f"/{role}/so101_{role}:\n  ros__parameters:\n    device_id: device-{role}\n"
                )
            with patch("so101_driver.paths.data_directory", return_value=share):
                self.assertEqual(default_device_id("leader"), "device-leader")
                self.assertEqual(default_device_id("follower"), "device-follower")

    def test_both_normal_nodes_and_other_calibration_block_startup(self):
        for name in ("so101_leader", "so101_follower", "so101_calibration"):
            with self.assertRaisesRegex(ValueError, "Stop the running"):
                check_running_nodes([(name, "/custom")])
        check_running_nodes([("so101_calibration_guard", "/"), ("rviz2", "/")])

    def test_torque_release_requires_option_and_confirmation_before_write(self):
        hardware = {
            i: dict(torque_enabled=int(i == 2), operating_mode=0, range_min=0, range_max=4095)
            for i in range(1, 7)
        }
        bus = Mock()
        bus.read_configuration.return_value = hardware
        with self.assertRaisesRegex(ValueError, "--disable-torque"):
            prepare_hardware(bus, "follower")
        bus.set_torque_enabled.assert_not_called()
        with self.assertRaises(KeyboardInterrupt):
            prepare_hardware(bus, "follower", disable_torque=True, reader=lambda text: "q")
        bus.set_torque_enabled.assert_not_called()

        def confirm(text):
            self.assertIn("Support the follower arm", text)
            bus.set_torque_enabled.assert_not_called()
            return ""

        released = {i: v | {"torque_enabled": 0} for i, v in hardware.items()}
        bus.read_configuration.side_effect = [hardware, released]
        self.assertEqual(
            prepare_hardware(bus, "follower", disable_torque=True, reader=confirm), released
        )
        bus.set_torque_enabled.assert_called_once_with([2], False)

    def test_torque_readback_failure_and_invalid_mode_do_not_start_calibration(self):
        hardware = {
            i: dict(torque_enabled=1, operating_mode=0, range_min=0, range_max=4095)
            for i in range(1, 7)
        }
        bus = Mock()
        bus.read_configuration.return_value = hardware
        with self.assertRaisesRegex(ValueError, "Torque release not confirmed"):
            prepare_hardware(bus, "follower", disable_torque=True, reader=lambda text: "")
        bus.reset_mock()
        hardware[3]["operating_mode"] = 1
        with self.assertRaisesRegex(ValueError, "position mode"):
            prepare_hardware(bus, "follower", disable_torque=True, reader=lambda text: "")
        bus.set_torque_enabled.assert_not_called()

    def test_quit_during_range_prompt(self):
        with patch("builtins.input", return_value="q"):
            with self.assertRaises(KeyboardInterrupt):
                wait_enter("range> ")

    def test_nonempty_input_does_not_confirm_torque_release(self):
        bus = Mock()
        bus.read_configuration.return_value = {
            i: dict(torque_enabled=1, operating_mode=0, range_min=0, range_max=4095)
            for i in range(1, 7)
        }
        with self.assertRaises(KeyboardInterrupt):
            prepare_hardware(
                bus, "follower", disable_torque=True, reader=Mock(side_effect=["n", "q"])
            )
        bus.set_torque_enabled.assert_not_called()

    def test_main_refuses_running_follower_before_opening_port(self):
        with tempfile.TemporaryDirectory() as directory:
            guard = Mock()
            guard.get_node_names_and_namespaces.return_value = [("so101_follower", "/follower")]
            error = io.StringIO()
            with (
                patch("so101_driver.calibration_cli.Node", return_value=guard),
                patch("so101_driver.calibration_cli.rclpy.init"),
                patch("so101_driver.calibration_cli.rclpy.spin_once"),
                patch("so101_driver.calibration_cli.rclpy.ok", return_value=False),
                patch("so101_driver.calibration_cli.FeetechBus") as bus,
                contextlib.redirect_stdout(io.StringIO()),
                contextlib.redirect_stderr(error),
            ):
                result = main(
                    [
                        "--role",
                        "follower",
                        "--device-id",
                        "/fake",
                        "--output",
                        str(Path(directory) / "profile.json"),
                    ]
                )
                self.assertEqual(result, 1)
                bus.return_value.open.assert_not_called()
                bus.return_value.close.assert_called_once()
                guard.destroy_node.assert_called_once()
                self.assertIn("driver nodes", error.getvalue())

    def test_overwrite_decline_and_accept(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "profile.json"
            path.write_text('{"original": true}')
            with patch("so101_driver.calibration_cli.confirm_overwrite", return_value=False):
                self.assertFalse(save_with_confirmation(path, {"new": True}))
            self.assertEqual(json.loads(path.read_text()), {"original": True})
            with patch("so101_driver.calibration_cli.confirm_overwrite", return_value=True):
                self.assertTrue(save_with_confirmation(path, {"new": True}))
            self.assertEqual(json.loads(path.read_text()), {"new": True})

    def test_terminal_confirmation_defaults_to_no(self):
        with patch("builtins.input", return_value=""):
            self.assertFalse(confirm_overwrite("/fake/profile.json"))
        with patch("builtins.input", return_value="yes"):
            self.assertTrue(confirm_overwrite("/fake/profile.json"))

    def test_startup_decline_does_not_initialize_ros_or_hardware(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "profile.json"
            path.write_text('{"original": true}')
            with (
                patch("builtins.input", return_value="n") as prompt,
                patch("so101_driver.calibration_cli.FeetechBus") as bus,
                patch("so101_driver.calibration_cli.rclpy.init") as init,
                contextlib.redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(main(["--device-id", "/fake", "--output", str(path)]), 0)
                prompt.assert_called_once()
                bus.assert_not_called()
                init.assert_not_called()
            self.assertEqual(json.loads(path.read_text()), {"original": True})

    def test_startup_approval_is_reused_at_save(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "profile.json"
            path.write_text('{"original": true}')
            with patch("so101_driver.calibration_cli.confirm_overwrite") as prompt:
                self.assertTrue(
                    save_with_confirmation(path, {"new": True}, overwrite_approved=True)
                )
                prompt.assert_not_called()
            self.assertEqual(json.loads(path.read_text()), {"new": True})

    def test_follower_default_overwrite_prompt_precedes_device_lookup_and_ros_init(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "calibration/follower.json"
            path.parent.mkdir()
            path.write_text('{"original": true}')
            with (
                patch("so101_driver.calibration_cli.default_output", return_value=path),
                patch("so101_driver.calibration_cli.default_device_id") as port,
                patch("so101_driver.calibration_cli.rclpy.init") as init,
                patch("so101_driver.calibration_cli.FeetechBus") as bus,
                patch("builtins.input", return_value="n") as prompt,
                contextlib.redirect_stdout(io.StringIO()),
            ):
                self.assertEqual(main(["--role", "follower"]), 0)
                prompt.assert_called_once()
                port.assert_not_called()
                init.assert_not_called()
                bus.assert_not_called()
            self.assertEqual(json.loads(path.read_text()), {"original": True})

    def test_follower_default_device_id_is_used_when_no_override_given(self):
        with tempfile.TemporaryDirectory() as tmp:
            with (
                patch(
                    "so101_driver.calibration_cli.default_output",
                    return_value=Path(tmp) / "profile.json",
                ),
                patch(
                    "so101_driver.calibration_cli.default_device_id", return_value="follower-id"
                ) as port,
                patch(
                    "so101_driver.calibration_cli.FeetechBus",
                    side_effect=RuntimeError("before access"),
                ) as bus,
                contextlib.redirect_stdout(io.StringIO()),
            ):
                with self.assertRaisesRegex(RuntimeError, "before access"):
                    main(["--role", "follower"])
                port.assert_called_once_with("follower")
                bus.assert_called_once_with("/fake")
                self.resolve.assert_called_once_with("follower-id")

    def test_startup_accept_precedes_hardware_access_and_keeps_old_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "profile.json"
            path.write_text('{"original": true}')

            def construct_bus(*args):
                prompt.assert_called_once()
                self.assertEqual(json.loads(path.read_text()), {"original": True})
                raise RuntimeError("stop before hardware access")

            with (
                patch("builtins.input", return_value="y") as prompt,
                patch("so101_driver.calibration_cli.FeetechBus", side_effect=construct_bus),
                contextlib.redirect_stdout(io.StringIO()),
            ):
                with self.assertRaisesRegex(RuntimeError, "stop before hardware access"):
                    main(["--device-id", "/fake", "--output", str(path)])
