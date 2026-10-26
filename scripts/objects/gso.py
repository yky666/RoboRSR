"""Import Google Scanned Objects (CC-BY 4.0, fuel.gazebosim.org/GoogleResearch) as RoboDojo rigid task-object
instances for the RSI pools (rsi/AGENTS.md), each turned into the official instances' frame convention and scaled
into their size range.

    shoe  (pack_objects_into_box): long axis on x with the heel at +x (all 18 official Rigid/shoe have the heel, the
          higher end, at +x; the task wants the shoe's +x along world +x in the box), length into SHOE_LENGTH_M,
          dropped when its height is then outside SHOE_HEIGHT_M (flip-flops, boots)

    convert  <model zips> -> <out>/Object/RoboDojo/Rigid/<category>/<index>/object.usdz (/World: the scanned mesh
             with its texture, standing on z = 0, plus a decimated convex-decomposition collision mesh),
             metadata.json (source: GSO model name, licence, attribution, tasks), description.json

Needs usd-core, numpy, trimesh, pillow and fast-simplification (no Isaac):
    python scripts/objects/gso.py convert --category shoe --zips gso/zips/*.zip --assets Assets --out extra_gso
A model whose mesh has the face count of a RoboDojo object is taken for a copy of it and skipped.
"""

import argparse
import json
from pathlib import Path
import sys
import tempfile
import uuid
import zipfile

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import manitwin  # noqa: E402

SHOE_LENGTH_M = (0.15, 0.175)  # official Rigid/shoe 0.151-0.189 long; at 0.19 a scan jams in the box and kicks it
# off the table, the official ones are smoother
SHOE_HEIGHT_M = (0.045, 0.095)  # official 0.070-0.092: no flip-flops, no boots
FIRST_INDEX = 900
TASKS = {"shoe": ["pack_objects_into_box"]}
LICENSE = "CC-BY-4.0"
ATTRIBUTION = "Google Scanned Objects, Google LLC (Downs et al., 2022), via fuel.gazebosim.org/GoogleResearch"


def load(zip_path):
    """(points (V, 3), faces (F, 3), uv (V, 2), texture png bytes or None, model.pbtxt text) of a GSO model zip."""
    import trimesh

    with zipfile.ZipFile(zip_path) as z, tempfile.TemporaryDirectory() as tmp:
        z.extractall(tmp)
        mesh = trimesh.load(Path(tmp) / "meshes" / "model.obj", force="mesh", process=False)
        tex = Path(tmp) / "materials" / "textures" / "texture.png"
        meta = Path(tmp) / "metadata.pbtxt"
        uv = getattr(mesh.visual, "uv", None)
        return (
            np.asarray(mesh.vertices, float),
            np.asarray(mesh.faces, np.int64),
            np.zeros((len(mesh.vertices), 2)) if uv is None else np.asarray(uv, float),
            tex.read_bytes() if tex.is_file() else None,
            meta.read_text(encoding="utf-8") if meta.is_file() else "",
        )


def shoe_pose(points):
    """(3x3 rotation about z, uniform scale) that puts a shoe's long axis on x with the heel (the higher end) at +x
    and its length into SHOE_LENGTH_M; None when its height is then outside SHOE_HEIGHT_M."""
    xy = points[:, :2] - points[:, :2].mean(0)
    axis = np.linalg.eigh(np.cov(xy.T))[1][:, -1]
    rot = np.array([[axis[0], axis[1], 0.0], [-axis[1], axis[0], 0.0], [0.0, 0.0, 1.0]])  # long axis -> x
    p = points @ rot.T
    lo, hi = p.min(0), p.max(0)
    length = hi[0] - lo[0]
    ends = [p[p[:, 0] > hi[0] - 0.25 * length, 2].max(), p[p[:, 0] < lo[0] + 0.25 * length, 2].max()]
    if ends[1] > ends[0]:  # the heel is at -x: half a turn
        rot = np.diag([-1.0, -1.0, 1.0]) @ rot
    s = float(np.clip(1.0, SHOE_LENGTH_M[0] / length, SHOE_LENGTH_M[1] / length))
    return (rot, s) if SHOE_HEIGHT_M[0] <= (hi[2] - lo[2]) * s <= SHOE_HEIGHT_M[1] else None


def write_object(target, points, faces, uv, texture, src):
    """object.usdz (+ metadata.json, description.json) of a mesh already posed: standing on z = 0, centred in xy."""
    import fast_simplification
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, UsdUtils, Vt

    col_pts, col_faces = points, faces
    if len(faces) > manitwin.COLLISION_FACES:
        col_pts, col_faces = fast_simplification.simplify(
            points.astype(np.float32), faces, target_reduction=1 - manitwin.COLLISION_FACES / len(faces)
        )
    with tempfile.TemporaryDirectory() as tmp:
        stage = Usd.Stage.CreateNew(str(Path(tmp) / "object.usda"))
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        world = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(world.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(world.GetPrim())
        mesh = UsdGeom.Mesh.Define(stage, "/World/visual")
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(faces.ravel().astype(np.int32)))
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        st.Set(Vt.Vec2fArray.FromNumpy(uv.astype(np.float32)))
        material = UsdShade.Material.Define(stage, "/World/Looks/scan")
        shader = UsdShade.Shader.Define(stage, "/World/Looks/scan/surface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.7)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        if texture:
            (Path(tmp) / "texture.png").write_bytes(texture)
            reader = UsdShade.Shader.Define(stage, "/World/Looks/scan/st")
            reader.CreateIdAttr("UsdPrimvarReader_float2")
            reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            image = UsdShade.Shader.Define(stage, "/World/Looks/scan/texture")
            image.CreateIdAttr("UsdUVTexture")
            image.CreateInput("file", Sdf.ValueTypeNames.Asset).Set("./texture.png")
            image.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
            image.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
                image.ConnectableAPI(), "rgb"
            )
        else:
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.5, 0.5, 0.5))
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
        col = UsdGeom.Mesh.Define(stage, "/World/collision")
        col.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(col_pts, np.float32)))
        col.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(col_faces)))
        col.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.asarray(col_faces).ravel().astype(np.int32)))
        UsdPhysics.CollisionAPI.Apply(col.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(col.GetPrim()).CreateApproximationAttr("convexDecomposition")
        col.MakeInvisible()
        stage.GetRootLayer().Save()
        target.mkdir(parents=True, exist_ok=True)
        if not UsdUtils.CreateNewUsdzPackage(
            Sdf.AssetPath(str(Path(tmp) / "object.usda")), str(target / "object.usdz")
        ):
            raise RuntimeError(f"packaging {src} failed")
    meta = manitwin.metadata(points, int(len(faces)), int(len(points)), src)
    meta["uuid"] = str(uuid.uuid5(uuid.NAMESPACE_URL, f"gso:{src}"))
    return meta


def description(pbtxt):
    """The model's description lines from metadata.pbtxt."""
    for line in pbtxt.splitlines():
        if line.startswith("description:"):
            text = line.split(":", 1)[1].strip().strip('"')
            return [t.strip() for t in text.split("\\n") if t.strip()]
    return []


def convert(category, zips, assets, out):
    official = manitwin.robodojo_faces(assets)
    done, skipped = [], []
    index = FIRST_INDEX
    for zip_path in sorted(zips):
        src = Path(zip_path).stem
        try:
            points, faces, uv, texture, pbtxt = load(zip_path)
        except Exception as e:  # a broken download
            skipped.append({"src": src, "reason": f"unreadable: {repr(e)[:80]}"})
            continue
        if "Shoe" not in pbtxt and category == "shoe":
            skipped.append({"src": src, "reason": "not in GSO category Shoe"})
            continue
        if (same := official.get(len(faces))) is not None:
            skipped.append({"src": src, "reason": f"mesh of {same}"})
            continue
        pose = shoe_pose(points)
        if pose is None:
            skipped.append({"src": src, "reason": f"height outside {SHOE_HEIGHT_M} m at the official length"})
            continue
        rot, s = pose
        p = (points @ rot.T) * s
        lo, hi = p.min(0), p.max(0)
        p -= np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])
        target = Path(out) / "Object" / "RoboDojo" / "Rigid" / category / f"{index:05d}"
        meta = write_object(target, p, faces, uv, texture, src)
        meta["source"] = {"dataset": "Google Scanned Objects", "object": src, "license": LICENSE}
        meta["source"] |= {"attribution": ATTRIBUTION, "scale": round(s, 4), "tasks": TASKS[category]}
        (target / "metadata.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
        desc = {"description": description(pbtxt)}
        (target / "description.json").write_text(json.dumps(desc, ensure_ascii=False), encoding="utf-8")
        done.append({"id": f"Rigid/{category}/{index:05d}", "source": src, "scale": round(s, 4)})
        index += 1
    return {"converted": done, "skipped": skipped}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("convert")
    c.add_argument("--category", choices=sorted(TASKS), required=True)
    c.add_argument("--zips", type=Path, nargs="+", required=True)
    c.add_argument("--assets", type=Path, required=True)
    c.add_argument("--out", type=Path, required=True)
    c.add_argument("--report", type=Path)
    args = ap.parse_args(argv)
    result = convert(args.category, args.zips, args.assets, args.out)
    if args.report:
        args.report.write_text(json.dumps(result, indent=1), encoding="utf-8")
    reasons = {}
    for skip in result["skipped"]:
        key = skip["reason"].split(":")[0].split(" of ")[0]
        reasons[key] = reasons.get(key, 0) + 1
    print(f"converted {len(result['converted'])}; skipped {reasons}")


if __name__ == "__main__":
    main()
