"""Resolve mesh references and display materials while retaining movable joints."""

import xml.etree.ElementTree as ET


def display_description(urdf_text):
    root = ET.fromstring(urdf_text)
    for color in root.findall(".//material/color"):
        color.set("rgba", "0.6 0.6 0.6 1.0")
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith("assets/"):
            mesh.set("filename", "package://so101_driver/description/so101/" + filename)
    return ET.tostring(root, encoding="unicode")
