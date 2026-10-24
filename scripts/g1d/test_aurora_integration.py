"""CPU acceptance of the Aurora body registration and its frozen source assets."""

import ast
import hashlib
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
BODY = "g1d_aurora_lite_ws"
MODEL = ROOT / "real2sim" / "robot" / BODY


class AuroraIntegrationTests(unittest.TestCase):
    def test_frozen_model_and_meshes_are_present(self):
        hashes = json.loads((MODEL / "sha256.json").read_text())
        for relative, expected in hashes.items():
            with self.subTest(file=relative):
                path = (MODEL / relative).resolve()
                self.assertTrue(path.is_relative_to(MODEL.resolve()))
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), expected)
        robot = ET.parse(MODEL / f"{BODY}.urdf").getroot()
        for mesh in robot.findall(".//mesh"):
            self.assertTrue((MODEL / mesh.attrib["filename"]).is_file(), mesh.attrib)
        names = {link.attrib["name"] for link in robot.findall("link")}
        for side in ("left", "right"):
            for sign in ("positive", "negative"):
                self.assertIn(f"{side}_aurora_{sign}_gel_link", names)

    def test_body_maps_and_action_dimensions(self):
        source = (ROOT / "env/robot_manager/robot_manager.py").read_text()
        self.assertTrue('"classes": ("G1DAuroraLeft", "G1DAuroraRight")' in source)
        self.assertTrue(f'"{BODY}": "{BODY}"' in source)
        info = json.loads((ROOT / "env_cfg/robot/_robot_info.json").read_text())
        self.assertEqual(info[BODY], {"arm_dim": [7, 7], "ee_dim": [1, 1]})
        self.assertIn("g1d_mobile", info)

    def test_existing_layout_appearance_is_preserved(self):
        source = (ROOT / "scripts/g1d/build_asset.py").read_text()
        self.assertTrue(f'"{BODY}":' in source)
        self.assertTrue("appearance.apply(shifted" in source)

    def test_cpu_checks_include_aurora_tests(self):
        check = (ROOT / "scripts/check.sh").read_text()
        self.assertTrue("scripts/g1d/xense_gripper" in check)
        self.assertTrue("scripts/g1d/test_aurora_integration.py" in check)

    def test_variant_config_and_registration_exist(self):
        import yaml

        config = yaml.safe_load((ROOT / "env_cfg" / f"{BODY}.yml").read_text())
        self.assertEqual(config["config"]["robot"], BODY)
        for subdirectory in ("robot_class", "robot_config"):
            path = ROOT / "env/robot_manager" / subdirectory / f"{BODY}.py"
            ast.parse(path.read_text())
        registration = ROOT / "real2sim/embodiments" / BODY / "registration.json"
        self.assertEqual(json.loads(registration.read_text())["embodiment"], BODY)


if __name__ == "__main__":
    unittest.main()
