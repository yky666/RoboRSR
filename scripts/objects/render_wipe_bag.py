"""Neutral rest-shape views and approximate same-view photo comparisons; no physics claim."""

import argparse
from pathlib import Path
import traceback


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--assets", type=Path, required=True)
    p.add_argument("--registration", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "fast_shutdown": True})
    try:
        import carb

        carb.settings.get_settings().set_bool("/isaaclab/render/offscreen", True)
        import isaaclab.sim as sim_utils
        from isaacsim.core.utils.stage import get_current_stage
        from isaacsim.core.utils.viewports import set_camera_view
        from isaacsim.sensors.camera import Camera
        import numpy as np
        from PIL import Image, ImageDraw
        from pxr import UsdGeom

        sim = sim_utils.SimulationContext(
            sim_utils.SimulationCfg(
                device="cuda:0",
                render=sim_utils.RenderCfg(
                    rendering_mode="quality", enable_reflections=True, enable_global_illumination=True
                ),
            )
        )
        stage = get_current_stage()
        floor = sim_utils.CuboidCfg(
            size=(2, 3, 0.02), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.54, 0.55, 0.57))
        )
        floor.func("/World/Floor", floor, translation=(0, 0.7, -0.01))
        light = sim_utils.DomeLightCfg(intensity=650)
        light.func("/World/Dome", light)
        sun = sim_utils.DistantLightCfg(intensity=1800, angle=25)
        sun.func("/World/Key", sun, orientation=(0.9239, 0.3827, 0, 0))
        for i in range(4):
            place = UsdGeom.Xform.Define(stage, f"/World/place{i}")
            place.AddTranslateOp().Set((0, 0.5 * i, 0.019))
            model = stage.DefinePrim(f"/World/place{i}/asset")
            model.GetReferences().AddReference(str((a.assets / f"{i:05d}" / "object.usdz").resolve()))
        camera = Camera("/World/Camera", resolution=(1600, 1200))
        camera.set_clipping_range(0.001, 0.48)
        camera.prim.GetAttribute("focalLength").Set(28.0)
        set_camera_view((0.20, -0.28, 0.25), (0, 0, 0.019), camera_prim_path=camera.prim_path)
        sim.reset()
        camera.initialize()
        shots = []
        for i in range(4):
            out = a.output / "Rigid/wipe_bag" / f"{i:05d}" / "preview"
            out.mkdir(parents=True, exist_ok=True)
            centre = np.array([0, 0.5 * i, 0.019])
            views = [
                ("00_model", (0.20, -0.28, 0.23)),
                ("view_front", (0, -0.29, 0.07)),
                ("view_side", (0.30, 0, 0.08)),
                ("view_back", (0, 0.29, 0.07)),
                ("view_top", (0.001, 0, 0.32)),
            ]
            if i == 0:
                views += [
                    ("reference_1", (0, -0.29, 0.095)),
                    ("reference_2", (0.17, -0.24, 0.235)),
                    ("reference_3", (0.015, -0.17, 0.31)),
                ]
            for name, delta in views:
                set_camera_view(centre + np.array(delta), centre, camera_prim_path=camera.prim_path)
                for _ in range(35):
                    sim.render()
                frame = Image.fromarray(camera.get_rgba()).convert("RGB")
                frame.save(out / (name + ".png"))
                if i == 0 and name.startswith("reference_"):
                    shots.append((name, frame.copy()))
        refs = [
            "31c71df6cf84f97880e007a08224aff0.jpg",
            "6d1daf69774d20ff5f43cf1888fb2239.jpg",
            "72757b43e310bc58a2b3916b07e9755a.jpg",
        ]
        for (name, frame), ref in zip(shots, refs):
            photo = Image.open(a.registration / "reference" / ref).convert("RGB").resize((1200, 900))
            frame = frame.resize((1200, 900))
            pair = Image.new("RGB", (2400, 950), "white")
            pair.paste(photo, (0, 50))
            pair.paste(frame, (1200, 50))
            draw = ImageDraw.Draw(pair)
            draw.text((15, 15), "REFERENCE PHOTO", fill="black")
            draw.text((1215, 15), "MODEL - approximate viewpoint; camera not calibrated", fill="black")
            pair.save(a.output / "Rigid/wipe_bag/00000/preview" / (name + "_comparison.jpg"), quality=95)
        covers = [
            Image.open(a.output / "Rigid/wipe_bag" / f"{i:05d}" / "preview/00_model.png").resize((800, 600))
            for i in range(4)
        ]
        sheet = Image.new("RGB", (1600, 1200), "white")
        for i, im in enumerate(covers):
            sheet.paste(im, ((i % 2) * 800, (i // 2) * 600))
        sheet.save(a.output / "Rigid/wipe_bag/00000/preview/variants.png")
        print("RENDERED", a.output, flush=True)
    except Exception:
        import os

        traceback.print_exc()
        os._exit(1)
    app.close(skip_cleanup=True)


if __name__ == "__main__":
    main()
