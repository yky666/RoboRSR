"""Inspect and tessellate source STEP geometry without inventing robot kinematics.

This is a CAD inspection tool, not a STEP-to-articulated-URDF converter. Lengths
from OpenCascade are millimetres; exported meshes use metres. No density, mass,
inertia, joint, wrist transform, or tactile response is inferred here.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re


def product_names(text):
    def decode(value):
        return re.sub(
            r"\\X2\\([0-9A-Fa-f]+)\\X0\\",
            lambda match: bytes.fromhex(match[1]).decode("utf-16-be"),
            value,
        )

    names = []
    for entity, body in re.findall(r"#(\d+)\s*=\s*PRODUCT\s*\((.*?)\)\s*;", text, re.S):
        match = re.search(r"'((?:''|[^'])*)'", body)
        names.append({"entity": int(entity), "name": decode(match[1]) if match else None})
    return names


def inspect(source, out, tessellate=False):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_Reader
    from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer

    source, out = source.resolve(), out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    data = source.read_bytes()
    if not data.startswith(b"ISO-10303-21;"):
        raise ValueError("Input is not a STEP Part 21 file")
    reader = STEPControl_Reader()
    if reader.ReadFile(str(source)) != IFSelect_RetDone:
        raise ValueError("OpenCascade could not parse the STEP source")
    transferred = reader.TransferRoots()
    shape = reader.OneShape()
    if shape.IsNull():
        raise ValueError("STEP has no transferred geometry")
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    minimum, maximum = box.CornerMin(), box.CornerMax()
    bounds = [minimum.X(), minimum.Y(), minimum.Z(), maximum.X(), maximum.Y(), maximum.Z()]

    def count(kind):
        explorer, total = TopExp_Explorer(shape, kind), 0
        while explorer.More():
            total += 1
            explorer.Next()
        return total

    record = {
        "source": source.name,
        "source_sha256": hashlib.sha256(data).hexdigest(),
        "source_bytes": len(data),
        "source_kind": "STEP CAD, not URDF",
        "transferred_roots": transferred,
        "solids": count(TopAbs_SOLID),
        "faces": count(TopAbs_FACE),
        "bbox_min_max_mm": bounds,
        "extents_mm": [bounds[i + 3] - bounds[i] for i in range(3)],
        "products": product_names(data.decode("utf-8", errors="replace")),
        "generated_kinematics": False,
        "generated_dynamics": False,
        "assembly_to_g1d_verified": False,
        "simulation_acceptance": False,
    }
    if tessellate:
        import numpy as np
        from OCP.BRep import BRep_Tool
        from OCP.BRepMesh import BRepMesh_IncrementalMesh
        from OCP.TopAbs import TopAbs_REVERSED
        from OCP.TopLoc import TopLoc_Location
        from OCP.TopoDS import TopoDS
        import trimesh

        BRepMesh_IncrementalMesh(shape, 0.15, False, 0.2, False).Perform()
        explorer = TopExp_Explorer(shape, TopAbs_FACE)
        vertices, faces = [], []
        while explorer.More():
            cast_face = getattr(TopoDS, "Face_s", None) or TopoDS.Face
            face = cast_face(explorer.Current())
            location = TopLoc_Location()
            mesh = BRep_Tool.Triangulation_s(face, location)
            if mesh is not None:
                offset = len(vertices)
                for i in range(1, mesh.NbNodes() + 1):
                    point = mesh.Node(i).Transformed(location.Transformation())
                    vertices.append([point.X() * 0.001, point.Y() * 0.001, point.Z() * 0.001])
                for i in range(1, mesh.NbTriangles() + 1):
                    triangle = mesh.Triangle(i)
                    a, b, c = (triangle.Value(j) for j in (1, 2, 3))
                    if face.Orientation() == TopAbs_REVERSED:
                        b, c = c, b
                    faces.append([offset + a - 1, offset + b - 1, offset + c - 1])
            explorer.Next()
        model = trimesh.Trimesh(vertices=np.array(vertices), faces=np.array(faces), process=False)
        if not len(model.vertices) or not np.isfinite(model.vertices).all():
            raise ValueError("Tessellation is empty or non-finite")
        model.export(out / "cad_geometry.glb")
        record.update(
            {
                "tessellation": {"linear_deflection_mm": 0.15, "angular_deflection_rad": 0.2},
                "mesh_unit": "metre",
                "mesh_vertices": len(model.vertices),
                "mesh_triangles": len(model.faces),
                "mesh_bbox_min_max_m": model.bounds.tolist(),
                "mesh_sha256": hashlib.sha256((out / "cad_geometry.glb").read_bytes()).hexdigest(),
            }
        )
    (out / "inspection.json").write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n")
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--tessellate", action="store_true")
    args = parser.parse_args()
    result = inspect(args.source, args.out, args.tessellate)
    print(json.dumps({k: result[k] for k in ("source", "solids", "faces", "extents_mm")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
