"""Render a scene twin with Isaac RTX and its own USD lights, for the scene cards of the Real2Sim platform.

Loads real2sim/scenes/<dir>/usd/scene.usda (or its preview.usda, export_glb.scene_stage) alone (no robot) and frames
everything outside /Background and /Lights (the table and what stands on it) from the robot's side, front-right and
above, the same view the platform's 3D
preview opens with; the camera is kept inside the room. Writes <dir>/preview/render/sim.jpg (preview/ is not in git;
sync it to the platform machine's checkout). One scene per process: each room brings its own dome light.

A benchmark scene config (env_cfg/scene/<name>.yml) is spawned with the env's own Room, Table and Background classes
from Assets/, framed on the table (or the room centre at table height when it has none), and written to
real2sim/benchmark_scenes/<name>/preview/render/sim.jpg.

--photos renders a twin from each reference photo's fitted camera instead, for comparing the lights, exposure and colour
with the photo: <dir>/preview/render/sim_photo_<photo>.jpg is photo | RTX render. The cameras come from
<dir>/preview/photo_cameras.json (written by the scene's preview.py: {photo: {"image", "eye", "R" (rows: camera right,
down, forward in the scene frame), "hfov_deg", "size"}}), else from reference/cameras_scene.json (SfM, photos p*),
moved by the room shift of usd/build_params.json so the room sits where it was photographed.

Usage (GPU server):
    OMNI_KIT_ACCEPT_EULA=YES python scripts/scenes/render_scenes.py real2sim/scenes/real_scene5_shenzhen_lab
    OMNI_KIT_ACCEPT_EULA=YES python scripts/scenes/render_scenes.py real2sim/scenes/real_scene5_shenzhen_lab --photos
    OMNI_KIT_ACCEPT_EULA=YES python scripts/scenes/render_scenes.py env_cfg/scene/default.yml
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Render a scene twin for its platform card.")
parser.add_argument("scene", help="scene directory, e.g. real2sim/scenes/real_scene3_lab, or env_cfg/scene/<name>.yml")
parser.add_argument("--photos", action="store_true", help="render from the reference photos' cameras")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = True
app = AppLauncher(args).app

import json  # noqa: E402
import os  # noqa: E402
from pathlib import Path  # noqa: E402
import sys  # noqa: E402
import traceback  # noqa: E402

import carb  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageOps  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402

VIEW = np.array([0.55, -1.0, 0.6]) / np.linalg.norm([0.55, -1.0, 0.6])  # front-right, above; the robot stands at -y
MARGIN = 0.15  # m the camera keeps from the walls and the ceiling
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts" / "scenes"))
from export_glb import scene_stage  # noqa: E402


def quat(m):
    """(w, x, y, z) of a rotation matrix."""
    w = np.sqrt(max(1e-12, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    return np.array([w, (m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w)])


def look_at(eye, target):
    """(w, x, y, z) of a camera at `eye` looking at `target`, Isaac "world" camera axes (+X forward, +Z up)."""
    forward = (target - eye) / np.linalg.norm(target - eye)
    left = np.cross([0.0, 0.0, 1.0], forward)
    left /= np.linalg.norm(left)
    up = np.cross(forward, left)
    return quat(np.stack([forward, left, up], axis=1))


def photo_cameras(scene_dir):
    """{photo: camera} as in preview/photo_cameras.json (see the module docstring)."""
    path = scene_dir / "preview" / "photo_cameras.json"
    if path.is_file():
        return json.loads(path.read_text())
    params = json.loads((scene_dir / "usd" / "build_params.json").read_text())
    shift = np.array(params.get("room_shift_unscaled", [0, 0, 0])) * params.get("scale_correction", 1.0)
    out = {}
    for name, c in json.loads((scene_dir / "reference" / "cameras_scene.json").read_text()).items():
        if name.startswith("p"):
            out[name] = dict(
                image=f"reference/{name}.jpg",
                eye=(np.array(c["center_scene"]) + shift).tolist(),
                R=c["R_cam_from_scene"],
                hfov_deg=float(np.degrees(2 * np.arctan(c["width"] / 2 / c["fx"]))),
                size=[c["width"], c["height"]],
            )
    return out


def bounds(prims):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    box = Gf.Range3d()
    for prim in prims:
        box.UnionWith(cache.ComputeWorldBound(prim).ComputeAlignedRange())
    return np.array(box.GetMin()), np.array(box.GetMax())


def benchmark_scene(sim, path):
    """Spawn env_cfg/scene/<name>.yml as a task would; returns (focus box, room box, output image)."""
    sys.path.insert(0, str(ROOT))
    import yaml

    from env.global_configs import ASSETS_PATH
    from env.scene_manager.objects.background import Background
    from env.scene_manager.objects.room import Room
    from env.scene_manager.objects.table import Table

    cfg, assets = yaml.safe_load(path.read_text(encoding="utf-8")), Path(ASSETS_PATH)
    room_dir = assets / "Room" / cfg["Room"]["default"]
    Room("/World/Room/room", str(next(room_dir.glob("*.usd"))), dict(cfg["Room"]))
    if bg := cfg.get("Background"):
        hdr = str(assets / "Background" / bg["default"])
        Background("/World/Background", {"intensity": bg["intensity"], "texture_path": hdr})
    focus = None
    if table := cfg.get("Table"):
        mdl = next((assets / "Material" / table["default"]).glob("*.mdl"))
        Table("/World/Table/table", str(mdl), dict(table), mdl_name=mdl.stem)
        focus = bounds([sim.stage.GetPrimAtPath("/World/Table/table")])
    room_box = bounds([sim.stage.GetPrimAtPath("/World/Room/room")])
    if focus is None:  # no table: a 1 m box at table height in the middle of the room
        c = np.array([*(room_box[0][:2] + room_box[1][:2]) / 2, 0.75])
        focus = (c - 0.5, c + 0.5)
    return focus, room_box, ROOT / "real2sim" / "benchmark_scenes" / path.stem / "preview" / "render" / "sim.jpg"


def main():
    scene_dir = Path(args.scene).resolve()
    render = sim_utils.RenderCfg(enable_reflections=True, enable_global_illumination=True, rendering_mode="quality")
    carb.settings.get_settings().set_float("/rtx/translucency/worldEps", 0.0)
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 60, device=args.device, render=render))
    if scene_dir.suffix == ".yml":
        (lo, hi), (room_lo, room_hi), out = benchmark_scene(sim, scene_dir)
        centre, size = (lo + hi) / 2, float(np.linalg.norm(hi - lo))
        eye = centre + VIEW * (1.1 * size + 0.8)
        eye[:2] = np.clip(eye[:2], room_lo[:2] + MARGIN, room_hi[:2] - MARGIN)
        eye[2] = min(eye[2], room_hi[2] - MARGIN)
        shoot(sim, eye, centre, lo, hi, out)
        return
    room = sim.stage.DefinePrim("/World/Scene", "Xform")
    room.GetReferences().AddReference(str(scene_stage(scene_dir)))
    parts = {p.GetName(): p for p in room.GetChildren()}
    if "PhysicsScene" in parts:
        parts["PhysicsScene"].SetActive(False)  # the SimulationContext brings its own
    if args.photos:
        shoot_photos(sim, scene_dir, photo_cameras(scene_dir))
        return
    focus = [p for name, p in parts.items() if name not in ("Background", "Lights", "PhysicsScene")]
    lo, hi = bounds(focus)
    centre, size = (lo + hi) / 2, float(np.linalg.norm(hi - lo))
    eye = centre + VIEW * (1.1 * size + 0.8)
    floor = sim.stage.GetPrimAtPath("/World/Scene/Background/Floor")
    if floor:  # keep the camera inside the room: within the floor's footprint and under the ceiling
        room_lo, room_hi = bounds([floor])
        room_top = bounds([parts["Background"]])[1][2]
        eye[:2] = np.clip(eye[:2], room_lo[:2] + MARGIN, room_hi[:2] - MARGIN)
        eye[2] = min(eye[2], room_top - MARGIN)
    shoot(sim, eye, centre, lo, hi, scene_dir / "preview" / "render" / "sim.jpg")


def shoot(sim, eye, centre, lo, hi, out):
    cam = Camera(prim_path="/World/camera", resolution=(1280, 720))
    UsdGeom.Camera(sim.stage.GetPrimAtPath("/World/camera")).GetFocalLengthAttr().Set(16.0)  # ~66 deg horizontal
    sim.reset()
    cam.initialize()
    cam.set_world_pose(eye, look_at(eye, centre), camera_axes="world")
    print(f"focus {np.round(lo, 2).tolist()} .. {np.round(hi, 2).tolist()}, camera at {np.round(eye, 2).tolist()}")
    for _ in range(60):  # let RTX accumulate
        sim.render()
    out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(cam.get_rgba()[:, :, :3]).save(out, quality=90)
    print(f"image: {out}")


def shoot_photos(sim, scene_dir, cameras):
    """One camera per photo, at most 1024 px on its long side; all accumulate together."""
    shots = []
    for i, (name, c) in enumerate(sorted(cameras.items())):
        w, h = c["size"]
        k = 1024 / max(w, h)
        size = (round(w * k), round(h * k))
        cam = Camera(prim_path=f"/World/photo_{i}", resolution=size)
        usd_cam = UsdGeom.Camera(sim.stage.GetPrimAtPath(f"/World/photo_{i}"))
        usd_cam.GetFocalLengthAttr().Set(16.0)
        aperture = 2 * 16.0 * np.tan(np.radians(c["hfov_deg"]) / 2)
        usd_cam.GetHorizontalApertureAttr().Set(float(aperture))
        usd_cam.GetVerticalApertureAttr().Set(float(aperture * size[1] / size[0]))
        shots.append((name, c, cam, size))
    sim.reset()
    for name, c, cam, _ in shots:
        cam.initialize()
        cam.set_world_pose(np.array(c["eye"]), quat(np.array(c["R"]).T), camera_axes="ros")
    for _ in range(60):
        sim.render()
    out_dir = scene_dir / "preview" / "render"
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, c, cam, size in shots:
        photo = ImageOps.exif_transpose(Image.open(scene_dir / c["image"])).convert("RGB").resize(size)
        pair = Image.new("RGB", (2 * size[0], size[1]))
        pair.paste(photo, (0, 0))
        pair.paste(Image.fromarray(cam.get_rgba()[:, :, :3]), (size[0], 0))
        out = out_dir / f"sim_photo_{Path(name).stem}.jpg"
        pair.save(out, quality=90)
        print(f"image: {out}")


try:
    main()
    failed = False
except Exception:
    traceback.print_exc()
    failed = True
sys.stdout.flush()
sys.stderr.flush()
os._exit(1 if failed else 0)  # app.close() hangs once a camera render product exists, and Kit eats exit codes
