import contextlib
import io
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml

from so101_driver.cameras import CameraPort, camera_ports, load_cameras, main

MODEL = "Innomaker-U20CAM-1080p-S1: Inno"


class CameraFixture(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        root = Path(self.directory.name)
        # Like /dev/v4l, stable names are relative links to the /dev/videoN devices.
        # Two identical cameras on USB ports 9 and 10; video1 is the metadata node of port 9.
        for video in ("video0", "video1", "video4"):
            (root / video).touch()
        for folder, links in (
            (
                "by-path",
                {
                    "pci-usb-0:9:1.0-video-index0": "video0",
                    "pci-usbv2-0:9:1.0-video-index0": "video0",
                    "pci-usb-0:9:1.0-video-index1": "video1",
                    "pci-usb-0:10:1.0-video-index0": "video4",
                },
            ),
            ("by-id", {"usb-Innomaker_SN0001-video-index0": "video0"}),
        ):
            (root / folder).mkdir()
            for name, video in links.items():
                (root / folder / name).symlink_to(f"../{video}")
        self.root = root
        self.by_path = root / "by-path"
        self.port9 = str(self.by_path / "pci-usb-0:9:1.0-video-index0")
        self.port10 = str(self.by_path / "pci-usb-0:10:1.0-video-index0")
        self.video0 = str((root / "video0").resolve())
        self.video4 = str((root / "video4").resolve())
        self.devices = {
            self.video0: {"capture": True, "model": MODEL},
            str((root / "video1").resolve()): {"capture": False, "model": MODEL},
            self.video4: {"capture": True, "model": MODEL},
        }


class CameraListTests(CameraFixture):
    def setUp(self):
        super().setUp()
        self.path = self.root / "cameras.yaml"

    def load(self, data, devices=None):
        self.path.write_text(data if isinstance(data, str) else yaml.safe_dump(data))
        devices = self.devices if devices is None else devices
        return load_cameras(self.path, devices, by_path=self.by_path)

    def camera(self, **entry):
        return {"cameras": [{"name": "front", "device": self.port9, **entry}]}

    def test_defaults_and_resolved_device(self):
        self.assertEqual(
            self.load(self.camera()),
            {
                "front": {
                    "video_device": self.video0,
                    "image_width": 640,
                    "image_height": 480,
                    "framerate": 30.0,
                    "camera_name": "front",
                    "frame_id": "front_optical_frame",
                }
            },
        )

    def test_size_fps_and_usb_cam_parameters(self):
        front = self.load(
            self.camera(width=1280, height=720, fps=15, parameters={"pixel_format": "mjpeg2rgb"})
        )["front"]
        self.assertEqual((front["image_width"], front["image_height"]), (1280, 720))
        self.assertEqual(front["framerate"], 15.0)
        self.assertEqual(front["pixel_format"], "mjpeg2rgb")

    def test_identical_cameras_are_told_apart_by_usb_port(self):
        cameras = self.load(
            {
                "cameras": [
                    {"name": "front", "device": self.port9},
                    {"name": "wrist", "device": self.port10},
                ]
            }
        )
        self.assertEqual(cameras["front"]["video_device"], self.video0)
        self.assertEqual(cameras["wrist"]["video_device"], self.video4)

    def test_no_cameras(self):
        template = Path(__file__).resolve().parents[1] / "config/cameras.template.yaml"
        for text in ("", "cameras: []\n", "cameras:\n", template.read_text()):
            with self.subTest(text=text):
                self.assertEqual(self.load(text), {})

    def test_invalid_lists_are_rejected_with_the_reason(self):
        two = {"cameras": [self.camera()["cameras"][0]] * 2}
        for data, message in (
            ({"cameras": {"name": "front"}}, "under 'cameras:'"),
            ({"cameras": ["front"]}, "name and a device"),
            (self.camera(name="1cam"), "start with a letter"),
            (self.camera(name="front cam"), "start with a letter"),
            (two, "used twice"),
            (self.camera(fsp=30), "unknown keys fsp"),
            ({"cameras": [{"name": "front"}]}, "USB port path"),
            (self.camera(device=str(self.by_path / "missing")), "not found.*same USB port"),
            (self.camera(width=0), "width must be a positive"),
            (self.camera(width=640.5), "width must be a positive"),
            (self.camera(height=True), "height must be a positive"),
            (self.camera(fps="30"), "fps must be a positive"),
            (self.camera(parameters=["pixel_format"]), "must be a mapping"),
        ):
            with self.subTest(message=message, data=data):
                with self.assertRaisesRegex(ValueError, message):
                    self.load(data)

    def test_only_usb_port_paths_are_accepted(self):
        by_id = str(self.root / "by-id/usb-Innomaker_SN0001-video-index0")
        for device in (by_id, str(self.root / "video0")):
            with self.subTest(device=device):
                with self.assertRaisesRegex(ValueError, "USB port path.*so101_list_cameras"):
                    self.load(self.camera(device=device))

    def test_devices_that_cannot_deliver_images_are_rejected(self):
        (self.by_path / "pci-usb-0:11:1.0-video-index0").symlink_to("../video0-gone")
        (self.root / "video0-gone").touch()
        same_camera = {
            "cameras": [
                {"name": "front", "device": self.port9},
                {"name": "wrist", "device": str(self.by_path / "pci-usbv2-0:9:1.0-video-index0")},
            ]
        }
        metadata = str(self.by_path / "pci-usb-0:9:1.0-video-index1")
        unknown = str(self.by_path / "pci-usb-0:11:1.0-video-index0")
        for data, devices, message in (
            (self.camera(device=metadata), None, "does not capture video"),
            (self.camera(device=unknown), None, "is not a video device"),
            (self.camera(), {self.video0: {"error": "Permission denied"}}, "cannot open"),
            (same_camera, None, "'front' and 'wrist' both use"),
        ):
            with self.subTest(message=message):
                with self.assertRaisesRegex(ValueError, message):
                    self.load(data, devices)


class ListCamerasTests(CameraFixture):
    def setUp(self):
        super().setUp()
        self.data = self.root / "so101_driver"
        self.config = self.data / "config/cameras.yaml"
        data = patch("so101_driver.paths.data_directory", return_value=self.data)
        data.start()
        self.addCleanup(data.stop)
        # The repository's config directory holds the same template as the installed share.
        self.templates = Path(__file__).resolve().parents[1] / "config"
        share = patch(
            "so101_driver.cameras.get_package_share_directory",
            return_value=str(self.templates.parent),
        )
        share.start()
        self.addCleanup(share.stop)
        self.front = CameraPort(self.port9, self.video0, MODEL)
        self.wrist = CameraPort(self.port10, self.video4, MODEL)

    def run_main(self, scans, answers):
        with (
            patch("so101_driver.cameras.camera_ports", side_effect=scans),
            patch("builtins.input", side_effect=answers),
            contextlib.redirect_stdout(io.StringIO()) as output,
        ):
            code = main([])
        return code, output.getvalue()

    def test_lists_capture_devices_once_per_usb_port(self):
        self.assertEqual(
            camera_ports(self.by_path, self.devices),
            [
                CameraPort(self.port10, self.video4, MODEL),
                CameraPort(self.port9, self.video0, MODEL),
            ],
        )

    def test_unplugged_camera_gets_a_new_config_and_the_lines_to_write(self):
        code, output = self.run_main([[self.front, self.wrist], [self.front]], [""])
        self.assertEqual(code, 0)
        self.assertIn("(not created yet)", output)
        self.assertIn(f"USB port path: {self.port10}", output)
        self.assertIn(f"Created the config file from the template: {self.config}", output)
        self.assertIn(f"  - name: <name>\n    device: {self.port10}\n", output)
        self.assertIn("same USB port", output)
        template = (self.templates / "cameras.template.yaml").read_text()
        self.assertEqual(self.config.read_text(), template)

    def test_written_camera_shows_its_name_and_the_file_is_kept(self):
        self.config.parent.mkdir(parents=True)
        text = f"cameras:\n  - name: wrist\n    device: {self.port10}\n"
        self.config.write_text(text)
        code, output = self.run_main([[self.front, self.wrist], [self.front]], [""])
        self.assertEqual(code, 0)
        self.assertRegex(output, r"pci-usb-0:10:1\.0-video-index0 +.+ +wrist")
        self.assertIn(f"This camera is set as 'wrist' in {self.config}", output)
        self.assertEqual(self.config.read_text(), text)

    def test_names_written_with_other_paths_are_not_shown(self):
        self.config.parent.mkdir(parents=True)
        by_id = self.root / "by-id/usb-Innomaker_SN0001-video-index0"
        self.config.write_text(f"cameras:\n  - name: front\n    device: {by_id}\n")
        code, output = self.run_main([[self.front, self.wrist], [self.wrist]], [""])
        self.assertEqual(code, 0)
        self.assertNotRegex(output, r"video-index0 +.+ +front")
        self.assertIn("This camera is not in the camera list yet", output)
        self.assertNotIn("Created the config file", output)

    def test_no_change_or_several_changes_ask_again(self):
        code, output = self.run_main(
            [[self.front, self.wrist], [self.front, self.wrist], [], [self.wrist]], ["", "", ""]
        )
        self.assertEqual(code, 0)
        self.assertIn("No camera was unplugged", output)
        self.assertIn("More than one camera was unplugged", output)
        self.assertIn(f"USB port path: {self.port9}", output)

    def test_no_camera_quit_and_end_of_input(self):
        self.assertEqual(self.run_main([[]], [])[0], 1)
        self.assertEqual(self.run_main([[self.front]], ["q"])[0], 130)
        self.assertEqual(self.run_main([[self.front]], EOFError())[0], 130)
        self.assertFalse(self.config.exists())


if __name__ == "__main__":
    unittest.main()
