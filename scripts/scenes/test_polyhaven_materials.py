"""CPU checks for the Poly Haven material import (polyhaven_materials.py): the selection filter and the MDL text."""

from pathlib import Path
import re
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import polyhaven_materials as pm  # noqa: E402


def texture(surface="floor", categories=(), name="X", aerial=False):
    return {"name": name, "categories": list(categories), "attributes": {"surface_use": surface, "aerial": aerial}}


class Select(unittest.TestCase):
    def test_filter(self):
        assets = {
            "oak_floor": texture(categories=["wood", "indoor"]),
            "brick_wall_01": texture("wall", ["brick"]),
            "rusty_metal": texture("object", ["metal"]),
            "forest_ground": texture("ground"),
            "cobble": texture("paving"),
            "aerial_rocks": texture(aerial=True),
            "rocky_trail": texture(categories=["terrain"]),
            "red_bark": texture("object", ["bark"]),
            "stone_wall_03": texture("wall", name="Stone Wall 03"),
            "plywood": texture("object", name="Plywood"),
            "no_attributes": {"name": "N", "categories": []},
        }
        self.assertEqual(pm.select(assets, ["Stone_Wall", "Plywood"]), ["brick_wall_01", "oak_floor", "rusty_metal"])

    def test_family(self):
        self.assertEqual(pm.family("Stone_Wall_02"), pm.family("stone wall"))
        self.assertNotEqual(pm.family("Marble_Tile_12"), pm.family("Marble"))


class Mdl(unittest.TestCase):
    def test_name_is_an_identifier(self):
        for key in ("oak_floor", "3d-tiles", "rock.face 02"):
            self.assertRegex(pm.material_name(key), r"^ph_[A-Za-z0-9_]+$")

    def test_text(self):
        text = pm.mdl_text("ph_oak_floor")
        self.assertTrue(text.startswith("mdl 1.4;"))
        self.assertIn("export material ph_oak_floor(*)", text)
        self.assertEqual(
            re.findall(r'texture_2d\("([^"]+)", ::tex::(\w+)\)', text),
            [
                ("./ph_oak_floor_BaseColor.jpg", "gamma_srgb"),
                ("./ph_oak_floor_ORM.jpg", "gamma_linear"),
                ("./ph_oak_floor_Normal.jpg", "gamma_linear"),
            ],
        )
        self.assertIn("enable_ORM_texture: true", text)
        self.assertEqual(text.count("("), text.count(")"))


if __name__ == "__main__":
    unittest.main()
