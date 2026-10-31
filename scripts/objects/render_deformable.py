"""RTX render of saved deformable nodes and rigid poses, with no duplicate physics simulation.

Run in Isaac Sim's Python; the solver remains in its separately pinned environment.
"""

import argparse
from pathlib import Path

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--run", type=Path, required=True)
p.add_argument("--output", type=Path, required=True)
p.add_argument("--gpu", type=int, default=0)
p.add_argument("--stride", type=int, default=1)
p.add_argument("--speed", type=float, default=0.5)
a = p.parse_args()
if a.stride < 1 or not 0 < a.speed <= 4:
    p.error("stride must be positive and speed in (0, 4]")

from isaacsim import SimulationApp

app = SimulationApp(
    {
        "headless": True,
        "width": 1280,
        "height": 720,
        "active_gpu": a.gpu,
        "physics_gpu": a.gpu,
        "multi_gpu": False,
        "renderer": "RaytracedLighting",
    }
)

import json
import os
import subprocess
import traceback

import cv2
from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera
import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, Vt


def main():
    a.output.mkdir(parents=True, exist_ok=False)
    data = np.load(a.run / "nodes.npz")
    config = json.loads((a.run / "config.json").read_text())
    points, faces, poses = data["points"], data["faces"], data["bodies"]
    world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)
    stage = world.stage
    UsdGeom.SetStageUpAxis(stage, "Z")

    def material(name, color, rough=0.55, metal=0.0):
        m = UsdShade.Material.Define(stage, "/World/Materials/" + name)
        s = UsdShade.Shader.Define(stage, str(m.GetPath()) + "/Shader")
        s.CreateIdAttr("UsdPreviewSurface")
        s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        s.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface")
        return m

    cloth = config["objects"][0]["kind"] == "cloth"
    mat = material("Deformable", (0.025, 0.23, 0.62) if cloth else (0.07, 0.48, 0.30), 0.72 if cloth else 0.45)
    mesh = UsdGeom.Mesh.Define(stage, "/World/Object")
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points[0].astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr([3] * len(faces))
    mesh.CreateFaceVertexIndicesAttr(faces.ravel().tolist())
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.SetNormalsInterpolation("vertex")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    sphere_ops = []
    for i, body in enumerate(config["bodies"]):
        sphere = UsdGeom.Sphere.Define(stage, f"/World/Body{i}")
        sphere.CreateRadiusAttr(body["radius_m"])
        UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim()).Bind(
            material(f"Sphere{i}", (0.65, 0.13, 0.035), 0.22, 0.25)
        )
        sphere_ops.append(sphere.AddTranslateOp())
    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    floor.AddTranslateOp().Set(Gf.Vec3f(0, 0, -0.01))
    floor.AddScaleOp().Set(Gf.Vec3f(20, 20, 0.02))
    UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(material("Floor", (0.19, 0.21, 0.24), 0.7))
    dome = UsdLux.DomeLight.Define(stage, "/World/Dome")
    dome.CreateIntensityAttr(450)
    key = UsdLux.DistantLight.Define(stage, "/World/Key")
    key.CreateIntensityAttr(1800)
    key.AddRotateXYZOp().Set(Gf.Vec3f(25, -35, -20))
    key.CreateAngleAttr(12)
    cam = Camera(prim_path="/World/Camera", resolution=(1280, 720))
    UsdGeom.Camera(stage.GetPrimAtPath("/World/Camera")).GetFocalLengthAttr().Set(18.0)
    UsdGeom.Camera(stage.GetPrimAtPath("/World/Camera")).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))
    world.reset()
    cam.initialize()
    eye = Gf.Vec3d(0.8, -0.95, 0.75) if cloth else Gf.Vec3d(0.48, -0.67, 0.46)
    target = Gf.Vec3d(0, 0, 0.22)
    matrix = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse()
    rot = matrix.ExtractRotationQuat()
    cam.set_world_pose(np.array(eye), np.array([rot.GetReal(), *rot.GetImaginary()]), camera_axes="usd")
    for i, op in enumerate(sphere_ops):
        op.Set(Gf.Vec3d(*poses[0, i, :3]))
    for _ in range(40):
        world.step(render=True)
    fps = a.speed / (config["frame_dt"] * a.stride)
    cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        "1280x720",
        "-r",
        str(fps),
        "-i",
        "pipe:0",
        "-an",
        "-c:v",
        "libx264",
        "-crf",
        "18",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(a.output / "demo.mp4"),
    ]
    encoder = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    indices = list(range(0, len(points), a.stride))
    peak = int(np.argmax(data["deformation_rms_m"]))
    peak_sample = min(indices, key=lambda i: abs(i - peak))
    for index in indices:
        q = points[index].astype(np.float32)
        mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(q))
        normals = np.zeros_like(q)
        fn = np.cross(q[faces[:, 1]] - q[faces[:, 0]], q[faces[:, 2]] - q[faces[:, 0]])
        for k in range(3):
            np.add.at(normals, faces[:, k], fn)
        normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
        mesh.CreateNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(normals))
        mesh.CreateExtentAttr().Set(Vt.Vec3fArray.FromNumpy(np.array([q.min(0), q.max(0)])))
        for i, op in enumerate(sphere_ops):
            op.Set(Gf.Vec3d(*poses[index, i, :3]))
        for _ in range(3):
            world.step(render=True)
        image = np.ascontiguousarray(cam.get_rgba()[:, :, :3]).copy()
        title = "Cloth draping over a sphere" if cloth else "Sphere impact on a soft block"
        cv2.rectangle(image, (0, 0), (1280, 98), (15, 20, 28), -1)
        cv2.putText(image, title, (24, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (240, 245, 250), 2, cv2.LINE_AA)
        label = "t {:.2f} s | shape RMS {:.1f} mm | height {:.1f} mm".format(
            (index + 1) * config["frame_dt"],
            data["deformation_rms_m"][index] * 1000,
            np.ptp(q[:, 2]) * 1000,
        )
        label += f" | playback {a.speed:g}x"
        cv2.putText(image, label, (24, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (210, 225, 240), 1, cv2.LINE_AA)
        cv2.putText(
            image,
            "Newton VBD | saved physical trajectory / RTX rendering | candidate material",
            (24, 697),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.51,
            (235, 240, 245),
            1,
            cv2.LINE_AA,
        )
        if index == indices[0]:
            for _ in range(round(fps)):
                encoder.stdin.write(image.tobytes())
        encoder.stdin.write(image.tobytes())
        if index in (indices[0], indices[len(indices) // 2], indices[-1], peak_sample):
            from PIL import Image

            Image.fromarray(image).save(a.output / (f"frame_{index:04d}.png"))
        if index % 60 == 0:
            print("rendered", index, flush=True)
    for _ in range(round(fps * 2)):
        encoder.stdin.write(image.tobytes())
    encoder.stdin.close()
    if encoder.wait() != 0:
        raise RuntimeError("Video encoding failed")
    (a.output / "render.json").write_text(
        json.dumps(
            {
                "renderer": "Isaac Sim RTX",
                "physics_rerun": False,
                "source_frames": len(points),
                "rendered_frames": len(indices),
                "fps": fps,
                "playback_speed": a.speed,
                "video": "demo.mp4",
            },
            indent=2,
        )
    )


try:
    main()
    code = 0
except Exception:
    traceback.print_exc()
    code = 1
# Camera render-product teardown can hang in this Isaac version. The solver is a separate, already closed process.
import sys

sys.stdout.flush()
sys.stderr.flush()
os._exit(code)
