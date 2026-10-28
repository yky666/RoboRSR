"""Import ManiTwin-100K objects (Apache-2.0, huggingface.co/datasets/ManiTwin/ManiTwin-100K) as RoboDojo rigid
objects, for the RSI distractor and instance pools (rsi/AGENTS.md). RoboDojo's own clutter library was built in a
ManiTwin workspace (its clutter.yml source_root), so a ManiTwin object the size of any RoboDojo object is dropped as a
likely copy.

    select     <annotations root> + Assets/ -> candidates.json: tabletop sized, not a near-duplicate, --per-category
    instances  the downloaded models of INSTANCE_SOURCES -> candidates.json of new task-object instances (bowls, handled
               mugs): shape-checked, scaled into the size range of the official instances, a mug turned so its handle
               points +x with the official handle frames; dropped only when its mesh is an official one (same face
               count: sizes and shapes of bowls and mugs overlap too much to tell copies by)
    convert    candidates.json + the downloaded base_rescaled.usdz ->
               <out>/Object/RoboDojo/<section>/<category>/<index>/object.usdz (/World: the ManiTwin model referenced
               whole, its own dome light switched off, standing on z = 0, plus a decimated convex-decomposition
               collision mesh), metadata.json, description.json

Needs usd-core, numpy and fast-simplification (no Isaac):
    python scripts/objects/manitwin.py select --annotations manitwin/ManiTwin-100K --assets Assets --out candidates.json
    python scripts/objects/manitwin.py instances --source manitwin/ManiTwin-100K --assets Assets --out instances.json
    python scripts/objects/manitwin.py convert --candidates candidates.json --source manitwin/ManiTwin-100K --out extra
Then drop-test with scripts/objects/check_object.py, link the tree into a run's Assets (rsi_assets.py link) and rescan.
"""

import argparse
import json
from pathlib import Path
import re
import shutil
import tempfile
import uuid

import numpy as np

SIZE_M = (0.03, 0.30)  # longest side of a tabletop object
THIN_M = 0.005  # shortest side
DUP_ABS, DUP_REL = 0.003, 0.03  # sorted extents this close to a RoboDojo object: treated as the same asset
SHAPE_REL = 0.05  # aspect ratios this close, with a shared category word: treated as the same asset, rescaled
FIRST_INDEX = 900  # new instances sit above every official index
COLLISION_FACES = 2000
MASS_KG = (0.02, 0.5)
DENSITY = 250.0  # kg/m^3 of the bounding box, a hollow household object
# RoboDojo category -> ManiTwin categories, and the tasks a converted instance is checked for (x5_layouts only swaps a
# new instance into those)
INSTANCE_SOURCES = {
    "bowl": (("cereal_bowl", "dessert_bowl", "dip_bowl"), ("stack_bowls",)),
    "mug": (("coffee_mug", "cup"), ("hang_mugs",)),
}
BOWL_DIAMETER_M, BOWL_HEIGHT_M = (0.11, 0.18), 0.08  # official Rigid/bowl: 0.11-0.18 wide, 0.05-0.08 tall
MUG_HEIGHT_M, MUG_LENGTH_M = (0.069, 0.082), 0.146  # official Rigid/mug: 0.069-0.082 tall, at most 0.146 with handle
HANDLE_REACH = (1.4, 2.3)  # handle tip / rim radius of a handled mug (1.0: no handle)
HANDLE_INSET_M = 0.003  # official handle frames sit this far inside the handle tip
# official Rigid/mug handle frames (wxyz): both turn the frame z axis onto the mug's +x, the way the handle points
HANDLE_QUATS = ([0.5, 0.5, 0.5, 0.5], [0.5, -0.5, 0.5, -0.5])


def robodojo_objects(assets):
    """(id, category, sorted extents) of every object in an Assets tree (both object libraries)."""
    found = []
    for meta in Path(assets).glob("Object/*/*/*/*/metadata.json"):
        geometry = json.loads(meta.read_text(encoding="utf-8")).get("geometry") or {}
        extents = (geometry.get("aligned_bbox") or geometry.get("oriented_bbox") or {}).get("extents")
        if extents:
            section, category, index = meta.parts[-4:-1]
            found.append((f"{section}/{category}/{index}", category, sorted(extents)))
    return found


def duplicate_of(category, dims, known):
    """The RoboDojo object a ManiTwin object most likely is, or None."""
    words = set(category.split("_"))
    for oid, cat, ext in known:
        if all(abs(a - b) <= max(DUP_ABS, DUP_REL * b) for a, b in zip(dims, ext)):
            return oid
        if words & set(cat.split("_")):
            ra, rb = np.asarray(dims) / dims[-1], np.asarray(ext) / ext[-1]
            if np.all(np.abs(ra - rb) <= SHAPE_REL):
                return oid
    return None


def prim_name(category):
    """A ManiTwin category as a USD prim name (the loaders build prim paths from it): chef's_knife -> chef_s_knife."""
    name = re.sub(r"[^0-9A-Za-z_]", "_", category)
    return f"n{name}" if name[0].isdigit() else name


def select(annotations, assets, per_category):
    known, chosen, dropped = robodojo_objects(assets), [], []
    taken = {}
    for path in sorted(Path(annotations).glob("*/*/manipulation_annotations.json")):
        raw, obj = path.parts[-3], path.parts[-2]
        if raw.endswith("_tmp"):  # ManiTwin's working copies of another category
            continue
        category = prim_name(raw)
        box = json.loads(path.read_text(encoding="utf-8")).get("bounding_box") or {}
        dims = sorted(box.get("dimensions") or [0, 0, 0])
        src = f"{raw}/{obj}"
        if not (SIZE_M[0] <= dims[-1] <= SIZE_M[1] and dims[0] >= THIN_M):
            dropped.append({"src": src, "reason": "size", "dims": dims})
        elif dup := duplicate_of(category, dims, known):
            dropped.append({"src": src, "reason": f"like {dup}", "dims": dims})
        elif taken.get(category, 0) < per_category:
            index = FIRST_INDEX + taken.get(category, 0)
            taken[category] = taken.get(category, 0) + 1
            chosen.append({"src": src, "section": "Clutter", "category": category, "index": index})
    return {"source": "ManiTwin/ManiTwin-100K", "license": "Apache-2.0", "chosen": chosen, "dropped": dropped}


def bowl_scale(points):
    """Uniform scale that brings an open, round bowl into the official size range, or None if it is not one."""
    lo, hi = points.min(0), points.max(0)
    (ex, ey, h), d = hi - lo, (hi - lo)[:2].mean()
    center = (lo[:2] + hi[:2]) / 2
    inner = points[np.linalg.norm(points[:, :2] - center, axis=1) < 0.1 * d]
    if abs(ex - ey) > 0.1 * max(ex, ey) or h > 0.75 * d or not len(inner) or inner[:, 2].max() > lo[2] + 0.5 * h:
        return None
    s_lo, s_hi = BOWL_DIAMETER_M[0] / d, min(BOWL_DIAMETER_M[1] / d, BOWL_HEIGHT_M / h)
    return None if s_lo > s_hi else float(np.clip(1.0, s_lo, s_hi))


def mug_pose(points):
    """(yaw deg that turns the handle to +x, uniform scale into the official size range) of an open mug with one
    handle, or None."""
    lo, hi = points.min(0), points.max(0)
    h = hi[2] - lo[2]
    rim = points[points[:, 2] > hi[2] - 0.05 * h]
    center = (rim[:, :2].min(0) + rim[:, :2].max(0)) / 2  # the body axis: a handle rarely reaches the rim
    rim_r = np.linalg.norm(rim[:, :2] - center, axis=1).max()
    band = points[(points[:, 2] > lo[2] + 0.2 * h) & (points[:, 2] < hi[2] - 0.1 * h)]
    r = np.linalg.norm(band[:, :2] - center, axis=1)
    inner = points[np.linalg.norm(points[:, :2] - center, axis=1) < 0.5 * rim_r]
    if not len(band) or not (HANDLE_REACH[0] <= r.max() / rim_r <= HANDLE_REACH[1]):
        return None
    if not len(inner) or inner[:, 2].max() > lo[2] + 0.6 * h:  # closed top
        return None
    tip = band[r.argmax(), :2] - center
    yaw = -np.degrees(np.arctan2(tip[1], tip[0]))
    length = rim_r + r.max()  # along the handle once it points +x
    s = min(float(np.clip(1.0, MUG_HEIGHT_M[0] / h, MUG_HEIGHT_M[1] / h)), MUG_LENGTH_M / length)
    return (float(yaw), s) if s * h >= 0.95 * MUG_HEIGHT_M[0] else None


def handle_frames(points):
    """Official-style handle frames at the tip of a handle that points +x (points already posed)."""
    tip = points[points[:, 0] > points[:, 0].max() - 0.01]
    pos = [float(points[:, 0].max() - HANDLE_INSET_M), float(np.median(tip[:, 1])), float(np.median(tip[:, 2]))]
    return {"handle": {"frame": [pos + q for q in HANDLE_QUATS]}}


def robodojo_faces(assets):
    """{face count: object} of every object in both libraries (metadata geometry.faces, one copy of the visual mesh)."""
    found = {}
    for meta in Path(assets).glob("Object/*/*/*/*/metadata.json"):
        faces = (json.loads(meta.read_text(encoding="utf-8")).get("geometry") or {}).get("faces")
        if faces:
            found[int(faces)] = "/".join(meta.parts[-4:-1])
    return found


def instances(source, assets):
    from pxr import Usd

    official, chosen, dropped = robodojo_faces(assets), [], []
    for category, (sources, tasks) in INSTANCE_SOURCES.items():
        index = FIRST_INDEX
        for usdz in sorted(p for raw in sources for p in Path(source).glob(f"{raw}/*/base_rescaled.usdz")):
            src = f"{usdz.parts[-3]}/{usdz.parts[-2]}"
            if usdz.stat().st_size < 1024:  # a git-lfs pointer: not downloaded
                dropped.append({"src": src, "reason": "not downloaded"})
                continue
            points, faces = mesh_arrays(Usd.Stage.Open(str(usdz)))
            if (same := official.get(len(faces))) is not None:
                dropped.append({"src": src, "reason": f"mesh of {same}"})
                continue
            item = {"src": src, "section": "Rigid", "category": category, "tasks": list(tasks)}
            if category == "bowl" and (s := bowl_scale(points)):
                item["scale"] = round(s, 4)
            elif category == "mug" and (pose := mug_pose(points)):
                item["yaw_deg"], item["scale"] = round(pose[0], 2), round(pose[1], 4)
            else:
                dropped.append({"src": src, "reason": f"not a {category}"})
                continue
            chosen.append({**item, "index": index})
            index += 1
    return {"source": "ManiTwin/ManiTwin-100K", "license": "Apache-2.0", "chosen": chosen, "dropped": dropped}


def posed(points, item):
    """Points turned by the item's yaw and scaled uniformly, as the visual xform ops do."""
    t = np.radians(item.get("yaw_deg", 0.0))
    rz = np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1]])
    return (points * item.get("scale", 1.0)) @ rz.T


def mesh_arrays(stage):
    """World-space points and triangle faces of every mesh under /root except lights."""
    from pxr import Usd, UsdGeom

    points, faces, offset = [], [], 0
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        p = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
        m = np.asarray(cache.GetLocalToWorldTransform(prim), dtype=float)  # row-vector convention
        points.append(p @ m[:3, :3] + m[3, :3])
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
        idx = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()) + offset
        start = np.concatenate([[0], np.cumsum(counts)[:-1]])
        for s, c in zip(start, counts):  # fan-triangulate
            faces.extend([idx[s], idx[s + k], idx[s + k + 1]] for k in range(1, c - 1))
        offset += len(p)
    return np.concatenate(points), np.asarray(faces, dtype=np.int64)


def metadata(points, faces_total, vertices_total, src):
    lo, hi = points.min(0), points.max(0)
    corners = [[float(x), float(y), float(z)] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    extents = [float(v) for v in hi - lo]
    center = (lo[:2] + hi[:2]) / 2
    radius = float(np.linalg.norm(points[:, :2] - center, axis=1).max())
    base = points[points[:, 2] <= lo[2] + 0.005]
    contact = float(np.linalg.norm(base[:, :2] - center, axis=1).max()) if len(base) else radius / 2
    circle = [float(center[0]), float(center[1]), 0.0, 1.0, 0.0, 0.0, 0.0]
    mass = float(np.clip(DENSITY * np.prod(extents), *MASS_KG))
    bbox = {"vertices": corners, "extents": extents}
    return {
        "uuid": str(uuid.uuid5(uuid.NAMESPACE_URL, f"manitwin:{src}")),
        "geometry": {
            "faces": faces_total,
            "vertices": vertices_total,
            "aligned_bbox": bbox,
            "oriented_bbox": bbox,
            "radius": radius,
        },
        "active": {
            "place": {
                "up": {
                    "orientation": [1.0, 0.0, 0.0, 0.0],
                    "projection_circle": {"center": circle, "radius": radius},
                    "contact_circle": {"center": circle, "radius": contact},
                }
            }
        },
        "physics": {"mass": round(mass, 4)},
        "source": {"dataset": "ManiTwin/ManiTwin-100K", "object": src, "license": "Apache-2.0"},
    }


def convert_one(item, source, out):
    import fast_simplification
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

    usdz = Path(source) / item["src"] / "base_rescaled.usdz"
    target = Path(out) / "Object" / "RoboDojo" / item["section"] / item["category"] / f"{item['index']:05d}"
    src_stage = Usd.Stage.Open(str(usdz))
    points, faces = mesh_arrays(src_stage)
    points = posed(points, item)
    lo, hi = points.min(0), points.max(0)
    shift = np.array([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
    points = points + shift
    col_pts, col_faces = points, faces
    if len(faces) > COLLISION_FACES:
        col_pts, col_faces = fast_simplification.simplify(
            points, faces, target_reduction=1 - COLLISION_FACES / len(faces)
        )
    with tempfile.TemporaryDirectory() as tmp:
        shutil.copy(usdz, Path(tmp) / "model.usdz")
        stage = Usd.Stage.CreateNew(str(Path(tmp) / "object.usda"))
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        world = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(world.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(world.GetPrim())
        visual = UsdGeom.Xform.Define(stage, "/World/visual")
        visual.GetPrim().GetReferences().AddReference("./model.usdz", Sdf.Path("/root"))
        visual.AddTranslateOp().Set(tuple(float(v) for v in shift))
        if item.get("yaw_deg"):
            visual.AddRotateZOp().Set(float(item["yaw_deg"]))
        if item.get("scale", 1.0) != 1.0:
            visual.AddScaleOp().Set((float(item["scale"]),) * 3)
        stage.OverridePrim("/World/visual/env_light").SetActive(False)  # every object would add a dome light
        col = UsdGeom.Mesh.Define(stage, "/World/collision")
        col.CreatePointsAttr([tuple(map(float, p)) for p in col_pts])
        col.CreateFaceVertexCountsAttr([3] * len(col_faces))
        col.CreateFaceVertexIndicesAttr([int(i) for i in np.asarray(col_faces).ravel()])
        UsdPhysics.CollisionAPI.Apply(col.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(col.GetPrim()).CreateApproximationAttr("convexDecomposition")
        col.MakeInvisible()
        stage.GetRootLayer().Save()
        target.mkdir(parents=True, exist_ok=True)
        if not UsdUtils.CreateNewUsdzPackage(
            Sdf.AssetPath(str(Path(tmp) / "object.usda")), str(target / "object.usdz")
        ):
            raise RuntimeError(f"packaging {usdz} failed")
    meta = metadata(points, int(len(faces)), int(len(points)), item["src"])
    if item["category"] == "mug":
        meta["active"]["functional"] = handle_frames(points)
    if item.get("tasks"):
        meta["source"]["tasks"] = item["tasks"]
    (target / "metadata.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
    caption = Path(source) / item["src"] / "caption.json"
    seen = json.loads(caption.read_text(encoding="utf-8")).get("seen", []) if caption.is_file() else []
    (target / "description.json").write_text(json.dumps({"description": seen}, ensure_ascii=False), encoding="utf-8")
    return target


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("select")
    s.add_argument("--annotations", type=Path, required=True)
    s.add_argument("--assets", type=Path, required=True)
    s.add_argument("--per-category", type=int, default=2)
    s.add_argument("--out", type=Path, required=True)
    i = sub.add_parser("instances")
    i.add_argument("--source", type=Path, required=True)
    i.add_argument("--assets", type=Path, required=True)
    i.add_argument("--out", type=Path, required=True)
    c = sub.add_parser("convert")
    c.add_argument("--candidates", type=Path, required=True)
    c.add_argument("--source", type=Path, required=True)
    c.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    if args.cmd == "select":
        result = select(args.annotations, args.assets, args.per_category)
        args.out.write_text(json.dumps(result, indent=1), encoding="utf-8")
        reasons = {}
        for d in result["dropped"]:
            key = "size" if d["reason"] == "size" else "duplicate"
            reasons[key] = reasons.get(key, 0) + 1
        categories = len({c["category"] for c in result["chosen"]})
        print(f"chosen {len(result['chosen'])} from {categories} categories; dropped {reasons}")
    elif args.cmd == "instances":
        result = instances(args.source, args.assets)
        args.out.write_text(json.dumps(result, indent=1), encoding="utf-8")
        reasons = {}
        for d in result["dropped"]:
            reasons[d["reason"].split(" of ")[0]] = reasons.get(d["reason"].split(" of ")[0], 0) + 1
        per = {}
        for c in result["chosen"]:
            per[c["category"]] = per.get(c["category"], 0) + 1
        print(f"chosen {per}; dropped {reasons}")
    else:
        failed = []
        for item in json.loads(args.candidates.read_text(encoding="utf-8"))["chosen"]:
            done = args.out / "Object" / "RoboDojo" / item["section"] / item["category"] / f"{item['index']:05d}"
            if (done / "metadata.json").is_file():
                continue
            try:
                convert_one(item, args.source, args.out)
            except Exception as e:  # keep going; report at the end
                failed.append((item["src"], repr(e)[:200]))
        print(f"failed {len(failed)}: {failed[:10]}")


if __name__ == "__main__":
    main()
