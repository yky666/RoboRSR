"""CPU-only URDF forward kinematics for model audits and geometry previews."""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh


def vector(text, default=(0, 0, 0)):
    return np.array([float(x) for x in text.split()]) if text else np.array(default, dtype=float)


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


def origin_transform(element):
    result = np.eye(4)
    if element is None:
        return result
    roll, pitch, yaw = vector(element.get("rpy"))
    result[:3, :3] = rotation((0, 0, 1), yaw) @ rotation((0, 1, 0), pitch) @ rotation((1, 0, 0), roll)
    result[:3, 3] = vector(element.get("xyz"))
    return result


def fk(root, supplied=None):
    supplied = supplied or {}
    joints = {j.get("name"): j for j in root.findall("joint")}
    values = {}

    def value(name, active=()):
        if name in active:
            raise ValueError("Mimic cycle")
        if name in values:
            return values[name]
        mimic = joints[name].find("mimic")
        if mimic is not None:
            result = value(mimic.get("joint"), (*active, name)) * float(mimic.get("multiplier", "1")) + float(
                mimic.get("offset", "0")
            )
        else:
            result = supplied.get(name, 0.0)
        values[name] = result
        return result

    children = {j.find("child").get("link") for j in joints.values()}
    roots = {link.get("name") for link in root.findall("link")} - children
    if len(roots) != 1:
        raise ValueError("URDF must have a single root")
    transforms = {roots.pop(): np.eye(4)}
    pending = list(joints.values())
    while pending:
        progress = False
        for joint in list(pending):
            parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
            if parent not in transforms:
                continue
            if child in transforms:
                raise ValueError("URDF has multiple parents or a cycle")
            motion = np.eye(4)
            kind = joint.get("type")
            if kind != "fixed":
                axis = vector(joint.find("axis").get("xyz"), (1, 0, 0))
                q = value(joint.get("name"))
                if kind == "prismatic":
                    motion[:3, 3] = axis * q
                elif kind in ("revolute", "continuous"):
                    motion[:3, :3] = rotation(axis, q)
                else:
                    raise ValueError(f"Unsupported joint type: {kind}")
            transforms[child] = transforms[parent] @ origin_transform(joint.find("origin")) @ motion
            pending.remove(joint)
            progress = True
        if not progress:
            raise ValueError("URDF has a disconnected joint tree")
    return transforms


def visuals(path, q=None, simplify_faces=None):
    path = Path(path)
    root = ET.parse(path).getroot()
    transforms = fk(root, q)
    result = []
    for link in root.findall("link"):
        for visual in link.findall("visual"):
            geometry = visual.find("geometry/mesh")
            if geometry is None:
                raise ValueError("This model audit expects mesh geometry")
            mesh = trimesh.load(path.parent / geometry.get("filename"), force="mesh", process=False)
            if simplify_faces and len(mesh.faces) > simplify_faces:
                # STL repeats each triangle's vertices. Weld those duplicates
                # before preview decimation so whole disconnected facets are
                # not discarded. The frozen source STL remains untouched.
                mesh.merge_vertices()
                mesh = mesh.simplify_quadric_decimation(face_count=simplify_faces)
            mesh.apply_scale(vector(geometry.get("scale"), (1, 1, 1)))
            mesh.apply_transform(transforms[link.get("name")] @ origin_transform(visual.find("origin")))
            color = visual.find("material/color")
            rgba = (
                vector(color.get("rgba"), (0.65, 0.65, 0.68, 1))
                if color is not None
                else np.array([0.65, 0.65, 0.68, 1])
            )
            mesh.visual.face_colors = np.tile(np.rint(rgba * 255).astype(np.uint8), (len(mesh.faces), 1))
            result.append((link.get("name"), mesh, rgba))
    return result
