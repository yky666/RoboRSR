"""Render built objects of any kind, deformable ones included, alone at their rest shape without simulating them.

check_object.py loads objects as rigid bodies and drops them, which FEM, cloth and rope assets cannot do. This puts each
object's USD on the same floor, under the same lights and camera as check_object.py's solo view, and writes
<image-dir>/<type>/<category>/<index>/preview/00_model.png: the cover the Real2Sim platform shows for the object.

Usage:
    OMNI_KIT_ACCEPT_EULA=YES python scripts/objects/render_model.py DIR [DIR ...] --image-dir real2sim/objects
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Render objects alone at their rest shape, without simulating them.")
parser.add_argument("dirs", nargs="+", help="object dirs containing object.usdz")
parser.add_argument("--image-dir", required=True, help="writes <dir>/<type>/<category>/<index>/preview/00_model.png")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = True
app = AppLauncher(args).app

import os  # noqa: E402
from pathlib import Path  # noqa: E402
import sys  # noqa: E402
import traceback  # noqa: E402

import carb  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

SPACING = 0.6  # m between objects along Y: neighbours stay outside each object's view
VIEW = np.array([0.82, -0.25, 0.5]) / np.linalg.norm([0.82, -0.25, 0.5])  # check_object.py SOLO_VIEW


def main():
    render = sim_utils.RenderCfg(  # check_object.py's settings, so covers of rigid and deformable objects match
        enable_translucency=True,
        enable_reflections=True,
        enable_global_illumination=True,
        rendering_mode="quality",
    )
    carb.settings.get_settings().set_float("/rtx/translucency/worldEps", 0.0)
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(device=args.device, render=render))
    stage = omni.usd.get_context().get_stage()
    row = (len(args.dirs) - 1) * SPACING
    floor = sim_utils.CuboidCfg(
        size=(2.0, row + 2.0, 0.02), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.36, 0.38))
    )
    floor.func("/World/floor", floor, translation=(0.0, row / 2, -0.01))
    dome = sim_utils.DomeLightCfg(intensity=300.0)
    dome.func("/World/dome", dome)
    sun = sim_utils.DistantLightCfg(intensity=2500.0, angle=5.0)
    sun.func("/World/sun", sun, orientation=(0.9397, 0.2418, 0.2418, 0.0))

    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    shots = []
    for k, d in enumerate(map(Path, args.dirs)):
        place = UsdGeom.Xform.Define(stage, f"/World/place_{k}")
        model = stage.DefinePrim(f"/World/place_{k}/model")
        model.GetReferences().AddReference(str(d / "object.usdz"))
        box = bounds.ComputeWorldBound(model).ComputeAlignedRange()
        lo, hi = np.array(box.GetMin()), np.array(box.GetMax())
        size = hi - lo
        place.AddTranslateOp().Set((-(lo[0] + hi[0]) / 2, k * SPACING - (lo[1] + hi[1]) / 2, -lo[2] + 0.0005))
        centre = np.array([0.0, k * SPACING, size[2] / 2])
        cam = Camera(prim_path=f"/World/camera_{k}", resolution=(640, 480))
        # pose before sim.reset(): later USD xform edits do not reach the renderer with Fabric on
        eye = centre + VIEW * max(3.7 * float(np.linalg.norm(size)), 0.03)
        set_camera_view(tuple(eye), tuple(centre), camera_prim_path=cam.prim_path)
        out = Path(args.image_dir) / d.parent.parent.name / d.parent.name / d.name / "preview" / "00_model.png"
        shots.append((cam, out))
        print(f"{d}: bbox {np.round(size * 1000, 1).tolist()} mm")

    sim.reset()  # never stepped: each object stays at the rest shape its USD was authored in
    for cam, _ in shots:
        cam.initialize()
        cam.set_clipping_range(0.001, 100.0)
    for _ in range(30):  # let RTX accumulate
        sim.render()
    for cam, path in shots:
        path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(cam.get_rgba()[:, :, :3]).save(path)
        print(f"image: {path}")


try:
    main()
    failed = False
except Exception:
    traceback.print_exc()
    failed = True
sys.stdout.flush()
sys.stderr.flush()
os._exit(1 if failed else 0)  # app.close() hangs once a camera render product exists, and Kit eats exit codes
