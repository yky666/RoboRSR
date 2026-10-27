"""Offline checks for the ShenZhen lab desk USD package (real_scene5). Writes check_report.json; prints a summary.

Only authoring / geometry checks against the tape measurements in registration.json. PhysX settling, IK reach,
grasping and RTX rendering need Isaac Sim and are NOT covered.
"""

import json
from pathlib import Path
import shutil
import tempfile

import cv2
import numpy as np
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, UsdUtils
from shapely.geometry import Polygon, box
from shapely.ops import unary_union

HERE = Path(__file__).resolve().parent
USD = HERE / "usd"
ROOM = "real_scene5"
# G1-D footprint as in real_scene3 (stand cylinder r = 0.22 m around the root, x = 0); root 0.13 m behind the table
# edge (env_cfg/robot/g1d_shenzhen_lab.yml: root y = -0.73, edge at -0.60).
ROBOT_STAND_R, ROOT_BEHIND_EDGE = 0.22, 0.13
HULL_MAX_VERTS, FACE_MAX_VERTS = 64, 32  # PhysX convex hulls that still collide with particles


def bounds(prim, cache):
    pts = np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=np.float64)
    m = np.array(cache.GetLocalToWorldTransform(prim)).T
    w = pts @ m[:3, :3].T + m[:3, 3]
    return w.min(0), w.max(0)


def main():
    rep = {
        "checks": [],
        "not_covered": [
            "PhysX settling / stability (needs Isaac Sim)",
            "IK reach and grasping",
            "RTX rendering, exposure and colour match against the photos",
            "robot stand (not measured), light positions and brightness (not recorded), friction (estimates)",
        ],
    }

    def add(name, ok, detail):
        rep["checks"].append(dict(name=name, ok=bool(ok), detail=detail))
        print(("PASS " if ok else "FAIL ") + name + ": " + str(detail))

    for lp in sorted(USD.glob("*.usda")):
        st = Usd.Stage.Open(str(lp))
        add(
            f"stage metadata {lp.name}",
            UsdGeom.GetStageUpAxis(st) == UsdGeom.Tokens.z
            and UsdGeom.GetStageMetersPerUnit(st) == 1.0
            and st.GetDefaultPrim().IsValid(),
            dict(upAxis=UsdGeom.GetStageUpAxis(st), defaultPrim=str(st.GetDefaultPrim().GetPath())),
        )

    _, assets, unresolved = UsdUtils.ComputeAllDependencies(str(HERE / f"{ROOM}.usd"))
    add(f"dependencies resolve (Room entry {ROOM}.usd)", not unresolved, dict(unresolved=[str(u) for u in unresolved]))
    with tempfile.TemporaryDirectory() as td:
        dst = Path(td) / ROOM
        shutil.copytree(USD, dst / "usd")
        shutil.copy(HERE / f"{ROOM}.usd", dst)
        _, _, unres2 = UsdUtils.ComputeAllDependencies(str(dst / f"{ROOM}.usd"))
        add("relocated copy resolves", not unres2, dict(unresolved=[str(u) for u in unres2]))

    st = Usd.Stage.Open(str(USD / "scene.usda"))
    cache = UsdGeom.XformCache()
    t = json.loads((USD / "build_params.json").read_text())["table"]
    tape = {k: v["value_m"] for k, v in json.loads((HERE / "registration.json").read_text())["measurements"].items()}

    meshes = [p for p in st.Traverse() if p.IsA(UsdGeom.Mesh)]
    render = [p for p in meshes if UsdGeom.Imageable(p).GetPurposeAttr().Get() != UsdGeom.Tokens.guide]
    colliders = [p for p in meshes if p.HasAPI(UsdPhysics.CollisionAPI)]
    unbound = [str(p.GetPath()) for p in render if not UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]]
    add("every render mesh has a bound material", not unbound, dict(render_meshes=len(render), unbound=unbound))
    tris = sum(len(UsdGeom.Mesh(p).GetFaceVertexIndicesAttr().Get()) // 3 for p in render)
    add("render triangle budget (< 100k)", tris < 100_000, dict(triangles=tris))
    bad = [str(p.GetPath()) for p in colliders if UsdGeom.Imageable(p).GetPurposeAttr().Get() != UsdGeom.Tokens.guide]
    add("colliders are guide purpose (not rendered)", not bad, dict(colliders=len(colliders), rendered=bad))
    rigid = [str(p.GetPath()) for p in st.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    add("scene is static (no RigidBodyAPI)", not rigid, dict(rigid=rigid))
    approx = {str(p.GetPath()): UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get() for p in colliders}
    over = {}
    for p in colliders:
        # the hull can only have fewer vertices than the mesh's unique points
        pts = np.unique(np.round(np.array(UsdGeom.Mesh(p).GetPointsAttr().Get()), 6), axis=0)
        if len(pts) > HULL_MAX_VERTS or max(UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get()) > FACE_MAX_VERTS:
            over[str(p.GetPath())] = len(pts)
    add(
        f"colliders: convex hulls <= {HULL_MAX_VERTS} vertices, faces <= {FACE_MAX_VERTS} vertices",
        not over and set(approx.values()) == {"convexHull"},
        dict(over=over, approximations=sorted(set(approx.values()))),
    )

    cb = {str(p.GetPath()).rsplit("/", 1)[-1]: bounds(p, cache) for p in colliders}
    in_table = {str(p.GetPath()).rsplit("/", 1)[-1] for p in colliders if str(p.GetPath()).startswith("/World/Table/")}
    lo, hi = cb["top"]
    model = dict(table_top_z=float(hi[2]), table_length=float(hi[0] - lo[0]), table_width=float(hi[1] - lo[1]))
    diffs = {k: round(abs(model[k] - tape[k]), 4) for k in model}
    add("tabletop collider matches the tape (< 1 cm)", all(d < 0.01 for d in diffs.values()), diffs)
    sz = np.array(UsdGeom.Mesh(st.GetPrimAtPath("/World/Table/Top/slab")).GetPointsAttr().Get())[:, 2]
    add("visible tabletop coincides with the collider", abs(sz.max() - t["top_z"]) < 1e-4, dict(z=float(sz.max())))
    table_parts = {n: b for n, b in cb.items() if n in in_table}
    above = [n for n, (a, b) in table_parts.items() if n != "top" and b[2] > t["top_z"] + 1e-4]
    add("nothing of the table sticks out above the tabletop", not above, dict(above=above))
    feet = {n: round(float(b[0][2]), 4) for n, b in table_parts.items() if n.endswith("_foot")}
    add("two feet stand on the floor", len(feet) == 2 and all(abs(z) < 1e-4 for z in feet.values()), feet)
    back = max(b[1][1] for b in table_parts.values())
    wall = cb["wall_back"][0][1]
    add(
        "table back edge touches the wall (registered 0 cm)",
        abs(wall - back) < 1e-3,
        dict(back=round(float(back), 4), wall=round(float(wall), 4)),
    )
    cab = [b for n, b in cb.items() if n.startswith("Cabinet")][0]
    clash = [n for n, (a, b) in table_parts.items() if (a < cab[1]).all() and (cab[0] < b).all()]
    add(
        "cabinet stands on the floor against the wall, clear of the table",
        abs(cab[0][2]) < 1e-4 and abs(cab[1][1] - wall) < 1e-3 and not clash,
        dict(x=[round(float(cab[0][0]), 3), round(float(cab[1][0]), 3)], size=(cab[1] - cab[0]).round(3).tolist()),
    )

    root = np.array([0.0, t["front_y"] - ROOT_BEHIND_EDGE])
    hits = [
        n
        for n, (a, b) in table_parts.items()
        if np.linalg.norm(np.clip(root, a[:2], b[:2]) - root) < ROBOT_STAND_R and a[2] < 0.70
    ]
    add(
        "G1-D stance (root 0.13 m behind the edge) clears the legs and feet",
        not hits,
        dict(root=root.tolist(), hits=hits),
    )

    bp = json.loads((USD / "build_params.json").read_text())
    room = Polygon([a for _, a, _ in bp["outline"]])
    walls = [
        p
        for p in render
        if str(p.GetPath()).startswith("/World/Background/Walls/") and "skirting" not in str(p.GetPath())
    ]
    add(
        "one wall per outline segment",
        len(walls) == len(bp["outline"]),
        dict(walls=len(walls), segments=len(bp["outline"])),
    )
    cover = unary_union(
        [
            box(
                *bounds(st.GetPrimAtPath(f"/World/Background/{n}"), cache)[0][:2],
                *bounds(st.GetPrimAtPath(f"/World/Background/{n}"), cache)[1][:2],
            )
            for n in ("Ceiling", "Soffit/back", "Soffit/elevator_side")
        ]  # fmt: skip
    )
    columns = room.convex_hull.intersection(box(-99, bp["room"]["y_col"], 99, 99)).difference(cover)
    gap = room.difference(cover.union(columns)).area
    add("ceiling and soffits close the room (no sky above the floor)", gap < 1e-3, dict(uncovered_m2=round(gap, 4)))

    def group(path):
        parts = path.split("/")
        if path.startswith("/World/Background/Colliders/"):
            name = parts[-1]
            return (
                name.rsplit("_", 1)[0]
                if name.startswith("chair_")
                else name.split("_")[1]
                if name.startswith("Furniture_")
                else None
            )
        if path.startswith("/World/Background/Furniture/"):
            return parts[5] if parts[4] == "Chairs" else parts[4]
        return None

    groups = {}
    for p in meshes:
        g = group(str(p.GetPath()))
        if g:
            a, b = bounds(p, cache)
            groups.setdefault(g, []).append((p.HasAPI(UsdPhysics.CollisionAPI), a, b))
    floor = {g: round(float(min(a[2] for _, a, _ in v)), 4) for g, v in groups.items()}
    add("every furniture piece stands on the floor", all(abs(z) < 1e-3 for z in floor.values()), floor)
    outside = [
        g
        for g, v in groups.items()
        for c, a, b in v
        if c and not room.buffer(1e-3).contains(box(a[0], a[1], b[0], b[1]))
    ]
    add("furniture inside the room outline", not outside, dict(outside=sorted(set(outside))))
    solid = {g: [(a, b) for c, a, b in v if c] for g, v in groups.items()}
    solid |= {"Table": list(table_parts.values()), "Cabinet": [cab]}
    clash = sorted(
        {
            f"{g}/{h}"
            for g in solid
            for h in solid
            if g < h
            for a, b in solid[g]
            for c, d in solid[h]
            if (np.minimum(b, d) - np.maximum(a, c) > 1e-3).all()
        }  # fmt: skip
    )
    add("furniture pieces do not overlap each other, the desk or the cabinet", not clash, dict(overlaps=clash))
    work = box(-t["length"] / 2, root[1] - ROBOT_STAND_R, t["length"] / 2, bp["room"]["y_n"]).buffer(1.0)
    near = sorted(
        {
            g
            for g, v in solid.items()
            if g not in ("Table", "Cabinet")
            for a, b in v
            if work.intersects(box(a[0], a[1], b[0], b[1]))
        }
    )
    add("furniture keeps 1 m from the desk and the robot stance", not near, dict(near=near))

    black = {
        f.name: round(float((cv2.imread(str(f)).max(2) < 8).mean()), 4)
        for f in sorted((USD / "textures").glob("*.png"))
    }
    add("textures have no unfilled (black) areas (< 1%)", all(v < 0.01 for v in black.values()), black)
    lights = [p for p in st.Traverse() if p.HasAPI(UsdLux.LightAPI)]
    add("lights present", len(lights) >= 2, dict(lights=[str(p.GetPath()) for p in lights]))
    add("room has a ceiling", st.GetPrimAtPath("/World/Background/Ceiling").IsValid(), {})

    rep["passed"] = sum(c["ok"] for c in rep["checks"])
    rep["total"] = len(rep["checks"])
    (HERE / "check_report.json").write_text(json.dumps(rep, indent=1, default=str), encoding="utf-8")
    print(f"{rep['passed']}/{rep['total']} checks passed")


if __name__ == "__main__":
    main()
