"""Build a complete G1-D with Aurora Lite grippers from pinned supplier CAD.

No hardware SDK, DDS, robot, or camera connection is used. The output is a
CAD-derived model, not a manufacturer-issued URDF. See model_contract.json for
the proposed wrist mount and the uncalibrated mass/inertia/contact parameters.
"""

import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import yaml

ROOT = Path(__file__).resolve().parents[3]
CAD_SHA256 = "658e5e2a175d455655a2990ceaf0e69fad48df6c1bba04d2eabdf7a6b4909c59"
G1D_SHA256 = "300634984610d0b280d154a00faf304d9744c712de792f619b9a1c92d76b18e4"
OPENING_M = 0.085
NOMINAL_MASS_KG = 0.615
# Source top-level occurrence numbers, pinned by the source hash and checked by
# their product names. The source assembly is fully open. These two groups are
# the rack carriers, linear carriages and the screws attached to each carriage.
CARRIERS = {"positive": [4, 6, 23, 24, 25, 26, 31, 32], "negative": [3, 5, 19, 20, 21, 22]}
SENSORS = {"positive": 33, "negative": 55}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def numbers(values):
    return " ".join(f"{float(v):.12g}" for v in values)


def save_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def read_cad(path):
    from OCP.collections import Sequence_TDF_Label
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDocStd import TDocStd_Document
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise ValueError("Cannot read supplier CAD")
    document = TDocStd_Document(TCollection_ExtendedString("BinXCAF"))
    if not reader.Transfer(document):
        raise ValueError("Cannot transfer supplier CAD assembly")
    tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    roots = Sequence_TDF_Label()
    tool.GetFreeShapes(roots)
    if roots.Length() != 1:
        raise ValueError("Expected one source CAD assembly")
    children = Sequence_TDF_Label()
    tool.GetComponents_s(roots.Value(1), children)
    if children.Length() != 56:
        raise ValueError("Supplier CAD component topology changed")
    return document, tool, children


def definition(tool, label):
    from OCP.TDF import TDF_Label

    referred = TDF_Label()
    return referred if tool.GetReferredShape_s(label, referred) else label


def label_name(label):
    from OCP.TDataStd import TDataStd_Name

    attribute = TDataStd_Name()
    return attribute.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attribute) else ""


def descendants(tool, label, parent_location=None, path=""):
    from OCP.collections import Sequence_TDF_Label
    from OCP.TopLoc import TopLoc_Location

    parent_location = parent_location or TopLoc_Location()
    ref = definition(tool, label)
    shape = tool.GetShape_s(label).Moved(parent_location)
    location = parent_location.Multiplied(tool.GetLocation_s(label))
    children = Sequence_TDF_Label()
    tool.GetComponents_s(ref, children)
    if children.Length():
        for i in range(1, children.Length() + 1):
            yield from descendants(tool, children.Value(i), location, f"{path}/{i}")
    else:
        yield {"path": path, "name": label_name(ref), "shape": shape}


def compound(shapes):
    from OCP.BRep import BRep_Builder
    from OCP.TopoDS import TopoDS_Compound

    builder, result = BRep_Builder(), TopoDS_Compound()
    builder.MakeCompound(result)
    for shape in shapes:
        builder.Add(result, shape)
    return result


def bounds(shape):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib

    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    a, b = box.CornerMin(), box.CornerMax()
    return np.array([[a.X(), a.Y(), a.Z()], [b.X(), b.Y(), b.Z()]])


def volume_properties(shape):
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    center, matrix = props.CentreOfMass(), props.MatrixOfInertia()
    return (
        props.Mass(),
        np.array([center.X(), center.Y(), center.Z()]),
        np.array([[matrix.Value(i + 1, j + 1) for j in range(3)] for i in range(3)]),
    )


def rack_slices(shape):
    """Exact CAD intersections with 4 mm axial slabs, before convex cooking.

    Each rack includes a tall end bracket. One convex hull bridges from that
    bracket to the far rack end, filling the opposite sensor's empty space.
    Axial partitioning preserves the recess without disabling external contact.
    """
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt

    lo, hi = bounds(shape)
    cuts = np.linspace(lo[2] - 1e-5, hi[2] + 1e-5, int(np.ceil((hi[2] - lo[2]) / 4)) + 1)
    result = []
    for low, high in zip(cuts[:-1], cuts[1:], strict=True):
        box = BRepPrimAPI_MakeBox(
            gp_Pnt(lo[0] - 1, lo[1] - 1, float(low)), hi[0] - lo[0] + 2, hi[1] - lo[1] + 2, float(high - low)
        ).Shape()
        section = BRepAlgoAPI_Common(shape, box).Shape()
        if volume_properties(section)[0] > 1e-5:
            result.append(section)
    return result


def tessellate(shape, shift_mm):
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    BRepMesh_IncrementalMesh(shape, 0.15, False, 0.2, False)
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    vertices, faces = [], []
    cast_face = getattr(TopoDS, "Face_s", None) or TopoDS.Face
    while explorer.More():
        face = cast_face(explorer.Current())
        location = TopLoc_Location()
        mesh = BRep_Tool.Triangulation_s(face, location)
        if mesh is not None:
            offset = len(vertices)
            for i in range(1, mesh.NbNodes() + 1):
                point = mesh.Node(i).Transformed(location.Transformation())
                vertices.append((np.array([point.X(), point.Y(), point.Z()]) + shift_mm) * 0.001)
            for i in range(1, mesh.NbTriangles() + 1):
                triangle = mesh.Triangle(i)
                a, b, c = (triangle.Value(j) for j in (1, 2, 3))
                if face.Orientation() == TopAbs_REVERSED:
                    b, c = c, b
                faces.append([offset + a - 1, offset + b - 1, offset + c - 1])
        explorer.Next()
    result = trimesh.Trimesh(vertices=np.array(vertices), faces=np.array(faces), process=False)
    if not len(result.faces) or not np.isfinite(result.vertices).all():
        raise ValueError("Invalid CAD tessellation")
    return result


def link_with_mesh(name, mesh_path, collision_path, mass, center, inertia, rgba):
    node = ET.Element("link", name=name)
    inertial = ET.SubElement(node, "inertial")
    ET.SubElement(inertial, "origin", xyz=numbers(center), rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=f"{mass:.12g}")
    ET.SubElement(
        inertial,
        "inertia",
        **{
            k: f"{inertia[i, j]:.12g}"
            for k, i, j in [("ixx", 0, 0), ("ixy", 0, 1), ("ixz", 0, 2), ("iyy", 1, 1), ("iyz", 1, 2), ("izz", 2, 2)]
        },
    )
    paths = [("visual", mesh_path)] + [("collision", p) for p in collision_path]
    for tag, path in paths:
        element = ET.SubElement(node, tag)
        ET.SubElement(element, "origin", xyz="0 0 0", rpy="0 0 0")
        ET.SubElement(ET.SubElement(element, "geometry"), "mesh", filename=path)
        if tag == "visual":
            ET.SubElement(ET.SubElement(element, "material", name=name + "_material"), "color", rgba=rgba)
    return node


def fixed_joint(name, parent, child, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    node = ET.Element("joint", name=name, type="fixed")
    ET.SubElement(node, "parent", link=parent)
    ET.SubElement(node, "child", link=child)
    ET.SubElement(node, "origin", xyz=numbers(xyz), rpy=numbers(rpy))
    return node


def frame_link(name):
    node = ET.Element("link", name=name)
    inertial = ET.SubElement(node, "inertial")
    ET.SubElement(inertial, "mass", value="1e-6")
    ET.SubElement(inertial, "inertia", ixx="1e-12", iyy="1e-12", izz="1e-12", ixy="0", ixz="0", iyz="0")
    return node


def save_urdf(root, path):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def build(out, repo=ROOT):
    out = out.resolve()
    if out.exists():
        raise FileExistsError(f"Refusing to overwrite {out}")
    cad = repo / "real2sim/grippers/aurora_lite/reference/aurora_lite_gripper.step"
    source = repo / "real2sim/robot/g1d_v4/g1_d_full_dex1_v4.urdf"
    if digest(cad) != CAD_SHA256 or digest(source) != G1D_SHA256:
        raise ValueError("Source hash changed; review component grouping before rebuilding")
    document, tool, children = read_cad(cad)  # retain document while its shapes are used
    if label_name(definition(tool, children.Value(7))) != "底座":
        raise ValueError("Mounting-base source occurrence changed")
    for side, index in SENSORS.items():
        if not label_name(definition(tool, children.Value(index))).startswith("楔形传感器-"):
            raise ValueError(f"Sensor source occurrence changed: {side}")
    source_mount_z = bounds(tool.GetShape_s(children.Value(7)))[0, 2]
    shift = np.array([0.0, 0.0, -source_mount_z])
    groups, membership = {}, {}
    moving = {i for values in CARRIERS.values() for i in values} | set(SENSORS.values())
    body_ids = sorted(set(range(1, children.Length() + 1)) - moving)
    groups["base"] = compound([tool.GetShape_s(children.Value(i)) for i in body_ids])
    membership["base"] = body_ids
    gel_bounds = []
    for side, ids in CARRIERS.items():
        groups[side + "_carrier"] = compound([tool.GetShape_s(children.Value(i)) for i in ids])
        membership[side + "_carrier"] = ids
        parts = list(descendants(tool, children.Value(SENSORS[side]), path=str(SENSORS[side])))
        gel = [p for p in parts if "SILICON" in p["name"].upper()]
        shell = [p for p in parts if "SILICON" not in p["name"].upper()]
        if len(gel) != 1:
            raise ValueError("Expected exactly one CAD silicone component per jaw")
        groups[side + "_gel"] = gel[0]["shape"]
        groups[side + "_sensor"] = compound([p["shape"] for p in shell])
        membership[side + "_gel"] = [{"path": p["path"], "name": p["name"]} for p in gel]
        membership[side + "_sensor"] = [{"path": p["path"], "name": p["name"]} for p in shell]
        gel_bounds.append(bounds(gel[0]["shape"]))
    gap = gel_bounds[0][0, 0] - gel_bounds[1][1, 0]
    if not np.isclose(gap * 0.001, OPENING_M, atol=2e-6):
        raise ValueError("Source CAD opening does not match the documented 85 mm stroke")
    tcp_z = float(np.mean([b[:, 2].mean() for b in gel_bounds]) + shift[2]) * 0.001
    properties = {name: volume_properties(shape) for name, shape in groups.items()}
    volumes = {name: p[0] for name, p in properties.items()}
    if min(volumes.values()) <= 0:
        raise ValueError("Non-positive CAD solid volume")
    out.mkdir(parents=True)
    mesh_dir = out / "meshes/aurora"
    mesh_dir.mkdir(parents=True)
    gripper = ET.Element("robot", name="aurora_lite_ws_cad_derived")
    group_reports = {}
    for name, shape in groups.items():
        local_shift = shift.copy()
        if name.startswith("positive_"):
            local_shift[0] -= OPENING_M * 500
        elif name.startswith("negative_"):
            local_shift[0] += OPENING_M * 500
        mesh = tessellate(shape, local_shift)
        visual = f"meshes/aurora/{name}_visual.stl"
        mesh.export(out / visual)
        # A hull over a carrier's rack + slider + screws fills the empty space
        # up to the opposite WS shell and blocks full closure. Keep separate
        # supplier-occurrence hulls so that empty space stays empty.
        shapes = (
            [tool.GetShape_s(children.Value(i)) for i in CARRIERS[name.split("_")[0]]]
            if name.endswith("_carrier")
            else [shape]
        )
        if name.endswith("_carrier"):
            shapes = rack_slices(shapes[0]) + shapes[1:]
        collisions, hull_vertices = [], []
        for index, part in enumerate(shapes):
            collision = f"meshes/aurora/{name}_collision_{index}.stl"
            hull = tessellate(part, local_shift).convex_hull
            hull.export(out / collision)
            collisions.append(collision)
            hull_vertices.append(len(hull.vertices))
        volume, center, tensor = properties[name]
        mass = NOMINAL_MASS_KG * volume / sum(volumes.values())
        inertia = tensor * (mass / volume) * 1e-6
        if not np.isfinite(inertia).all() or np.linalg.eigvalsh(inertia).min() <= 0:
            raise ValueError("Non-positive CAD-derived inertia")
        rgba = "0.65 0.67 0.70 1" if name == "base" else "0.10 0.11 0.12 1"
        if name.endswith("_gel"):
            rgba = "0.22 0.32 0.38 1"
        gripper.append(
            link_with_mesh(
                "aurora_" + name + "_link", visual, collisions, mass, (center + local_shift) * 0.001, inertia, rgba
            )
        )
        group_reports[name] = {
            "source_occurrences": membership[name],
            "volume_mm3": volume,
            "mass_kg_estimated": mass,
            "inertia_kg_m2_estimated": inertia.tolist(),
            "source_bbox_mm": bounds(shape).tolist(),
            "closed_mesh_bbox_m": mesh.bounds.tolist(),
            "visual_triangles": len(mesh.faces),
            "collision_hull_vertices": hull_vertices,
        }
    for side, axis in (("positive", "1 0 0"), ("negative", "-1 0 0")):
        joint = ET.Element("joint", name=f"aurora_{side}_jaw_joint", type="prismatic")
        ET.SubElement(joint, "parent", link="aurora_base_link")
        ET.SubElement(joint, "child", link=f"aurora_{side}_carrier_link")
        ET.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")
        ET.SubElement(joint, "axis", xyz=axis)
        ET.SubElement(joint, "limit", lower="0", upper="0.0425", effort="40", velocity="0.07")
        if side == "negative":
            ET.SubElement(joint, "mimic", joint="aurora_positive_jaw_joint", multiplier="1", offset="0")
        gripper.append(joint)
        gripper.append(
            fixed_joint(f"aurora_{side}_sensor_fixed", f"aurora_{side}_carrier_link", f"aurora_{side}_sensor_link")
        )
        gripper.append(fixed_joint(f"aurora_{side}_gel_fixed", f"aurora_{side}_sensor_link", f"aurora_{side}_gel_link"))
        pad = gel_bounds[0 if side == "positive" else 1]
        center = pad.mean(axis=0) + shift
        center[0] = (pad[0, 0] - 42.5) if side == "positive" else (pad[1, 0] + 42.5)
        frame = f"aurora_{side}_contact_frame"
        gripper.append(frame_link(frame))
        gripper.append(
            fixed_joint(
                frame + "_fixed",
                f"aurora_{side}_gel_link",
                frame,
                xyz=center * 0.001,
                rpy=(-np.pi / 2, 0, np.pi / 2 if side == "positive" else -np.pi / 2),
            )
        )
    save_urdf(gripper, out / "aurora_lite_ws.urdf")
    full = ET.parse(source).getroot()
    full.set("name", "g1d_aurora_lite_ws")
    for element in list(full):
        if "dex1" in (element.get("name") or ""):
            full.remove(element)
    for joint in full.findall("joint"):
        if joint.get("name").endswith("Wheel_Joint"):
            joint.set("type", "fixed")
    for mesh in full.iter("mesh"):
        original = source.parent / mesh.get("filename")
        relative = Path("meshes/g1d") / original.name
        target = out / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.exists():
            shutil.copyfile(original, target)
        mesh.set("filename", str(relative))
    for side in ("left", "right"):
        mount = side + "_aurora_mount_link"
        full.append(frame_link(mount))
        full.append(
            fixed_joint(side + "_aurora_wrist_mount_joint", side + "_wrist_yaw_link", mount, xyz=(0.0415, 0, 0))
        )
        for element in gripper:
            element = copy.deepcopy(element)
            if element.get("name"):
                element.set("name", side + "_" + element.get("name"))
            for child in element.iter():
                for attr in ("link", "joint"):
                    if child.get(attr):
                        child.set(attr, side + "_" + child.get(attr))
            full.append(element)
        full.append(
            fixed_joint(
                side + "_aurora_cad_mount_joint", mount, side + "_aurora_base_link", rpy=(np.pi / 2, 0, np.pi / 2)
            )
        )
        tcp = side + "_aurora_tcp_link"
        full.append(frame_link(tcp))
        full.append(fixed_joint(side + "_aurora_tcp_fixed", mount, tcp, xyz=(tcp_z, 0, 0)))
    save_urdf(full, out / "g1d_aurora_lite_ws.urdf")
    source_contract = json.loads((source.parent / "physics_contract.json").read_text())
    names = {link.get("name") for link in full.findall("link")}
    filtered = {a: [b for b in bs if b in names] for a, bs in source_contract["filtered_pairs"].items() if a in names}
    pairs = []
    for side in ("left", "right"):
        pair = [side + "_aurora_positive_carrier_link", side + "_aurora_negative_carrier_link"]
        pairs.append(pair)
    save_json(
        out / "physics_contract.json",
        {
            "filtered_pairs": filtered,
            "additional_internal_slider_carrier_pairs": pairs,
            "external_gel_contacts_enabled": True,
            "robot_self_collision_enabled": True,
            "collision_model": (
                "carrier rack partitioned by 4 mm axial CAD slabs, other carrier occurrences use separate hulls; "
                "other groups use one convex hull; "
                "internal rack carrier pair filtered; no tactile deformation model"
            ),
        },
    )
    for side in ("left", "right"):
        config = yaml.safe_load((repo / f"scripts/g1d/{side}_robot_config.yml").read_text())
        config.update(
            urdf_path="./g1d_aurora_lite_ws.urdf",
            ee_link=f"{side}_aurora_mount_link",
            gripper_joints_name=[f"{side}_aurora_positive_jaw_joint", f"{side}_aurora_negative_jaw_joint"],
            gripper_move={
                "base": f"{side}_aurora_positive_jaw_joint",
                "sign": 1.0,
                "mimic": [f"{side}_aurora_negative_jaw_joint", 1.0, 0.0],
            },
            gripper_bias=tcp_z,
            gripper_scale=[0.0, 0.0425],
            camera=[c for c in config.get("camera", []) if c["link"] == "head_link"],
        )
        (out / f"{side}_robot_config.yml").write_text(yaml.safe_dump(config, sort_keys=False))
    contract = {
        "model": "g1d_aurora_lite_ws",
        "urdf_origin": "generated from official STEP CAD, not a manufacturer-issued URDF",
        "sources": {"g1d_urdf_sha256": G1D_SHA256, "aurora_step_sha256": CAD_SHA256},
        "cad_base_plane_z_mm": float(source_mount_z),
        "cad_opening_m": float(gap * 0.001),
        "max_opening_m": OPENING_M,
        "q_jaw_m": "opening_m / 2; q=0 closed, q=0.0425 open; opposite axes, equal travel",
        "normalized_action": "0 closed, 1 open; two arm joint vectors (7 each) and one scalar per gripper",
        "nominal_single_gripper_mass_kg": NOMINAL_MASS_KG,
        "mass_inertia_basis": (
            "615 g product specification distributed by exact CAD solid volumes with uniform "
            "effective density; estimates, not per-part measurements"
        ),
        "virtual_frame_mass_kg_each": 1e-6,
        "single_jaw_effort_limit_N": 40,
        "opening_speed_limit_m_s": 0.14,
        "jaw_speed_limit_m_s": 0.07,
        "speed_basis": (
            "conservative interpretation of the 140 mm/s product specification as total opening speed; not measured"
        ),
        "tcp_distance_from_mount_m": tcp_z,
        "tcp_basis": "midpoint of CAD silicone-pad axial bounds; not measured tool calibration",
        "wrist_mount": {
            "xyz_m": [0.0415, 0.0, 0.0],
            "native_cad_rpy_rad": [float(np.pi / 2), 0.0, float(np.pi / 2)],
            "status": (
                "proposed geometry placement; Dex1 wrist interface origin reused; Aurora-to-G1-D "
                "flange/adapter not mechanically verified"
            ),
        },
        "tactile": {
            "sensors": 4,
            "geometry": "two native WS sensor assemblies per gripper retained from official Aurora CAD",
            "contact_frames": "geometric inner-face centres, +Z normal into the jaw gap; not calibrated optical frames",
            "response": (
                "rigid contact only; no claim of real optical images, force-map accuracy, soft-gel "
                "deformation or SDK compatibility validation"
            ),
            "user_step": (
                "separate supplied WS revision retained and inspected as reference; not silently "
                "substituted for the matched factory assembly"
            ),
        },
        "cameras": "original G1-D head stereo only; no Dex1 wrist RGB extrinsics copied onto Aurora",
        "calibration": {
            "camera": "pending for this complete body",
            "joint_zero": "pending",
            "gripper": "pending",
            "dynamics": "pending",
            "wrist_mount": "pending",
        },
        "cad_groups": group_reports,
    }
    save_json(out / "model_contract.json", contract)
    save_json(out / "sha256.json", {str(p.relative_to(out)): digest(p) for p in sorted(out.rglob("*")) if p.is_file()})
    print(
        json.dumps(
            {
                "out": str(out),
                "links": len(full.findall("link")),
                "joints": len(full.findall("joint")),
                "opening_m": gap * 0.001,
                "tcp_m": tcp_z,
                "gripper_mass_kg": sum(NOMINAL_MASS_KG * v / sum(volumes.values()) for v in volumes.values()),
            }
        )
    )
    return contract


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--repo", type=Path, default=ROOT)
    args = parser.parse_args()
    build(args.out, args.repo.resolve())


if __name__ == "__main__":
    main()
