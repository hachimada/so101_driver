import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from so101_driver.description import display_description


class DescriptionTests(unittest.TestCase):
    def test_display_preserves_movable_joints_and_resolves_meshes(self):
        path = Path(__file__).parents[1] / "description/so101/so101_new_calib.urdf"
        original = path.read_text()
        root = ET.fromstring(display_description(original))
        self.assertEqual(root.find("joint[@name='gripper']").get("type"), "revolute")
        self.assertEqual(len([j for j in root.findall("joint") if j.get("type") == "revolute"]), 6)
        self.assertTrue(
            all(m.get("filename").startswith("package://") for m in root.findall(".//mesh"))
        )
        self.assertEqual(path.read_text(), original)
