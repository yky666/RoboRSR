"""Generate the deformable (PhysX FEM volume) paper cup Dynamic/paper_cup/00000.

The cup is a closed thin-walled solid of revolution (outer wall, rolled rim bead, inner wall, recessed floor and
skirt) with exact measured outer dimensions. Deformable physics is authored only through the official IsaacLab
schema helpers (``define_mass_properties``, ``define_deformable_body_properties``, ``DeformableBodyMaterialCfg``),
so PhysX cooks the collision and simulation tetrahedral meshes itself. Real paperboard (~0.35 mm) cannot be
tetrahedralised at a usable resolution, so the wall uses an effective thickness and an effective Young's modulus;
the material is not calibrated against the real cup.

Run inside Isaac Sim (headless) from the repository root:
    python scripts/objects/make_paper_cup_fem.py --output Assets/Object/Real2Sim/Dynamic
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import uuid

import numpy as np

MOUTH_OUTER_DIAMETER_MM = 87.9
BOTTOM_DIAMETER_MM = 60.5
HEIGHT_MM = 136.0
MASS_KG = 0.022
REAL_WALL_MM = 0.35
REAL_YOUNGS_PA = 4.0e9
BLACK = (0.02, 0.02, 0.022)
WHITE = (0.93, 0.92, 0.88)


def cup_profile(wall_mm, bead_mm, floor_recess_mm):
    """Closed (r, z, tag) polygon in mm, counter-clockwise in the r-z plane; tag 1 marks black outer faces."""
    r_bot = BOTTOM_DIAMETER_MM / 2
    bead_r = bead_mm / 2
    bead_c = (MOUTH_OUTER_DIAMETER_MM / 2 - bead_r, HEIGHT_MM - bead_r)
    z_wall_top = bead_c[1] - bead_r
    slope = (bead_c[0] - r_bot) / z_wall_top
    normal_offset = wall_mm * math.sqrt(1 + slope**2)

    def outer(z):
        return r_bot + slope * z

    def inner(z):
        return outer(z) - normal_offset

    pts = []
    for z in np.linspace(0, z_wall_top, 40, endpoint=False):
        pts.append((outer(z), z, 1))
    for a in np.linspace(-math.pi / 2, math.pi, 13):
        pts.append((bead_c[0] + bead_r * math.cos(a), bead_c[1] + bead_r * math.sin(a), 0))
    z_floor_top = floor_recess_mm + wall_mm
    z_inner_top = bead_c[1] - bead_r * 0.6
    for z in np.linspace(z_inner_top, z_floor_top, 38):
        pts.append((inner(z), z, 0))
    for r in np.linspace(inner(z_floor_top), 0, 9)[1:]:
        pts.append((r, z_floor_top, 0))
    for r in np.linspace(0, inner(floor_recess_mm), 9):
        pts.append((r, floor_recess_mm, 0))
    for z in np.linspace(floor_recess_mm, 0, 4)[1:]:
        pts.append((inner(z), z, 1 if z == 0 else 0))
    return pts


def revolve(profile, segments):
    """Revolve a closed profile into a watertight triangle mesh (metres); axis points become single poles."""
    points, ring_index = [], []
    for r, z, _ in profile:
        if r < 1e-9:
            ring_index.append([len(points)] * segments)
            points.append((0.0, 0.0, z * 1e-3))
            continue
        ids = []
        for k in range(segments):
            a = 2 * math.pi * k / segments
            ids.append(len(points))
            points.append((r * math.cos(a) * 1e-3, r * math.sin(a) * 1e-3, z * 1e-3))
        ring_index.append(ids)
    faces, tags = [], []
    n = len(profile)
    for i in range(n):
        j = (i + 1) % n
        tag = profile[i][2] if profile[j][2] == profile[i][2] else 0
        a_ring, b_ring = ring_index[i], ring_index[j]
        for k in range(segments):
            k1 = (k + 1) % segments
            a0, a1, b0, b1 = a_ring[k], a_ring[k1], b_ring[k], b_ring[k1]
            for tri in ((a0, a1, b1), (a0, b1, b0)):
                if len(set(tri)) == 3:
                    faces.append(tri)
                    tags.append(tag)
    points = np.asarray(points, dtype=np.float64)
    faces = np.asarray(faces, dtype=np.int64)
    a, b, c = points[faces[:, 0]], points[faces[:, 1]], points[faces[:, 2]]
    volume = float(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6)
    if volume < 0:
        faces = faces[:, ::-1].copy()
        volume = -volume
    return points, faces, np.asarray(tags), volume


def check_watertight(faces):
    edges = {}
    for f in faces:
        for e in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0])):
            key = tuple(sorted(e))
            edges[key] = edges.get(key, 0) + 1
    bad = sum(1 for v in edges.values() if v != 2)
    if bad:
        raise RuntimeError(f"Cup mesh is not watertight: {bad} non-manifold edges")


def measure(points):
    p = points * 1e3
    r = np.linalg.norm(p[:, :2], axis=1)
    z = p[:, 2]
    return {
        "开口外径": round(float(2 * r[z > z.max() - 6].max()), 2),
        "底部直径": round(float(2 * r[z < z.min() + 0.5].max()), 2),
        "高度": round(float(z.max() - z.min()), 2),
    }


def preview_material(stage, path, color, roughness):
    from pxr import Gf, Sdf, UsdShade

    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def build(args):
    import isaaclab.sim as sim_utils
    from isaaclab.sim import schemas
    import omni.usd
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, UsdUtils, Vt

    profile = cup_profile(args.wall_mm, args.bead_mm, args.floor_recess_mm)
    points, faces, tags, volume = revolve(profile, args.segments)
    check_watertight(faces)
    points[:, 2] -= points[:, 2].min()

    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/paper_cup")
    stage.SetDefaultPrim(root.GetPrim())
    mesh_path = "/paper_cup/geometry/mesh"
    UsdGeom.Xform.Define(stage, "/paper_cup/geometry")
    mesh = UsdGeom.Mesh.Define(stage, mesh_path)
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in points]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(faces.flatten().tolist()))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*points.min(0)), Gf.Vec3f(*points.max(0))]))

    looks = "/paper_cup/Looks"
    white = preview_material(stage, f"{looks}/paper_white", WHITE, 0.7)
    black = preview_material(stage, f"{looks}/paper_black", BLACK, 0.55)
    binding = UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
    binding.Bind(white)
    outer = UsdGeom.Subset.CreateGeomSubset(
        mesh, "outer_wall", UsdGeom.Tokens.face, Vt.IntArray(np.flatnonzero(tags == 1).tolist()), "materialBind"
    )
    UsdShade.MaterialBindingAPI.Apply(outer.GetPrim()).Bind(black)
    inner = UsdGeom.Subset.CreateGeomSubset(
        mesh, "inner_and_rim", UsdGeom.Tokens.face, Vt.IntArray(np.flatnonzero(tags == 0).tolist()), "materialBind"
    )
    UsdShade.MaterialBindingAPI.Apply(inner.GetPrim()).Bind(white)

    bending_equivalent = REAL_YOUNGS_PA * (REAL_WALL_MM / args.wall_mm) ** 3
    youngs = args.youngs_pa or bending_equivalent
    schemas.define_mass_properties(mesh_path, sim_utils.MassPropertiesCfg(mass=MASS_KG), stage=stage)
    deformable_cfg = sim_utils.DeformableBodyPropertiesCfg(
        rest_offset=0.0,
        contact_offset=args.contact_offset_m,
        solver_position_iteration_count=args.solver_iterations,
        simulation_hexahedral_resolution=args.resolution,
        collision_simplification=False,
        self_collision=False,
        vertex_velocity_damping=args.vertex_damping,
        sleep_damping=10.0,
        settling_threshold=0.01,
        sleep_threshold=0.002,
    )
    schemas.define_deformable_body_properties(mesh_path, deformable_cfg, stage=stage)
    material_cfg = sim_utils.DeformableBodyMaterialCfg(
        density=MASS_KG / volume,
        youngs_modulus=youngs,
        poissons_ratio=args.poisson,
        dynamic_friction=0.5,
        elasticity_damping=args.elasticity_damping,
    )
    material_path = "/paper_cup/physicsMaterial"
    material_cfg.func(material_path, material_cfg)
    sim_utils.bind_physics_material(mesh_path, material_path, stage=stage)

    out = args.output / "paper_cup" / "00000"
    out.mkdir(parents=True, exist_ok=True)
    flat = Usd.Stage.Open(stage.Flatten())
    flat.GetRootLayer().Export(str(out / "object.usdc"))
    usdz = out / "object.usdz"
    usdz.unlink(missing_ok=True)
    if not UsdUtils.CreateNewUsdzPackage(Sdf.AssetPath(str(out / "object.usdc")), str(usdz)):
        raise RuntimeError("usdz packaging failed")

    lo, hi = points.min(0), points.max(0)
    center = ((lo + hi) / 2).tolist()
    radius = BOTTOM_DIAMETER_MM / 2e3
    corners = [[float(x), float(y), float(z)] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    box = {"vertices": corners, "extents": (hi - lo).tolist()}
    metadata = {
        "uuid": str(uuid.uuid5(uuid.NAMESPACE_URL, "Real2Sim/Dynamic/paper_cup/00000")),
        "physics": {"mass": MASS_KG, "size": (hi - lo).tolist(), "backend": "isaac_physx_fem"},
        "geometry": {
            "aligned_bbox": box,
            "oriented_bbox": box,
            "bbox": (hi - lo).tolist(),
            "center": center,
            "bounds_m": [lo.tolist(), hi.tolist()],
        },
        "active": {
            "place": {
                "up": {
                    "orientation": [1, 0, 0, 0],
                    "contact_circle": {"center": [0, 0, 0, 1, 0, 0, 0], "radius": radius},
                }
            }
        },
        "passive": {},
        "backend": "isaac_physx_fem",
        "schema": "PhysxSchema.PhysxDeformableBodyAPI (legacy FEM volume, driven by isaaclab.assets.DeformableObject)",
        "requires": "GPU physics pipeline (PhysX FEM soft bodies are GPU-only)",
        "render_mesh_vertices": len(points),
        "render_mesh_triangles": len(faces),
        "dimensions_mm": {
            "mouth_outer_diameter": MOUTH_OUTER_DIAMETER_MM,
            "bottom_diameter": BOTTOM_DIAMETER_MM,
            "height": HEIGHT_MM,
        },
        "dimensions_source": "measured on the real cup (Clutter/paper_cup/00000 spec)",
        "measurements_mm": measure(points),
        "real_material_calibrated": False,
        "candidate_material": {
            "model": "linear-elastic FEM volume with effective wall thickness",
            "real_wall_thickness_mm_assumed": REAL_WALL_MM,
            "real_youngs_modulus_pa_assumed": REAL_YOUNGS_PA,
            "effective_wall_thickness_mm": args.wall_mm,
            "youngs_modulus_pa": youngs,
            "bending_equivalent_youngs_pa": bending_equivalent,
            "youngs_rule": "E_eff = E_real*(t_real/t_eff)^3 (equal bending stiffness)"
            if not args.youngs_pa
            else "about 3x the bending-equivalent E_eff: the PhysX FEM solver under-converges on this thin shell at "
            "120 iterations, so the nominal value gave a softer squeeze than a thin-ring estimate (~1 N at 20 mm)",
            "poissons_ratio": args.poisson,
            "elasticity_damping": args.elasticity_damping,
            "density_kg_m3": MASS_KG / volume,
            "mass_kg": MASS_KG,
            "rim_bead_diameter_mm": args.bead_mm,
            "floor_recess_mm": args.floor_recess_mm,
        },
        "simulation": {
            "simulation_hexahedral_resolution": args.resolution,
            "solver_position_iteration_count": args.solver_iterations,
            "collision_simplification": False,
            "contact_offset_m": args.contact_offset_m,
            "vertex_velocity_damping": args.vertex_damping,
        },
        "limits": [
            "material not calibrated; linear elastic, no plasticity or creasing",
            "rim bead is part of the same material (stiffer only through geometry)",
        ],
        "object_sha256": hashlib.sha256((out / "object.usdc").read_bytes()).hexdigest(),
    }
    (out / "metadata.json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n")
    description = {
        "caption": {"name": ["paper_cup", "可形变纸杯"]},
        "description": [
            "Single-wall paper cup, black outside, white rim and inside; PhysX FEM volume deformable with "
            "effective wall thickness, uncalibrated material. Deformable companion of Clutter/paper_cup/00000."
        ],
    }
    (out / "description.json").write_text(json.dumps(description, indent=2, ensure_ascii=False) + "\n")
    return metadata


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, default=Path("Assets/Object/Real2Sim/Dynamic"))
    p.add_argument("--wall-mm", type=float, default=3.0)
    p.add_argument("--bead-mm", type=float, default=3.5)
    p.add_argument("--floor-recess-mm", type=float, default=9.0)
    p.add_argument("--segments", type=int, default=48)
    p.add_argument("--resolution", type=int, default=28)
    p.add_argument("--solver-iterations", type=int, default=120)
    p.add_argument("--youngs-pa", type=float, default=19.0e6, help="0 selects the bending-equivalent E_eff")
    p.add_argument("--poisson", type=float, default=0.3)
    p.add_argument("--elasticity-damping", type=float, default=0.005)
    p.add_argument("--vertex-damping", type=float, default=0.0)
    p.add_argument("--contact-offset-m", type=float, default=0.001)
    args = p.parse_args()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        metadata = build(args)
        print(json.dumps({k: metadata[k] for k in ("measurements_mm", "candidate_material", "simulation")}))
    finally:
        app.close()


if __name__ == "__main__":
    main()
