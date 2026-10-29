"""Visual-only USD writeback, with explicit world-to-local conversion.

Call after the Newton worker step, before rendering. The generated mesh has no
PhysX body/collision schema: simulating this surface in both engines is invalid.
"""

import numpy as np
from pxr import UsdGeom, UsdPhysics, Vt


class UsdMeshSink:
    def __init__(self, stage, root="/World/NewtonVisuals"):
        self.stage, self.root = stage, root
        self.topology = {}
        self.stamp = None
        UsdGeom.Xform.Define(stage, root)
        if not np.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1):
            raise ValueError("Render stage must use meters")
        if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
            raise ValueError("Render stage must use Z up")

    def update(self, state):
        stamp = (state["epoch"], state["frame"])
        if self.stamp is not None and stamp < self.stamp:
            raise ValueError("Stale worker state")
        # Validate the complete frame before changing any mesh.
        pending = []
        for name, item in state["objects"].items():
            if not name.isidentifier():
                raise ValueError("Mesh name must be an identifier")
            points = np.asarray(item["points_m"], dtype=np.float32)
            faces = np.asarray(item["faces"], dtype=np.int32)
            if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
                raise ValueError("Invalid nodes")
            if faces.ndim != 2 or faces.shape[1] != 3 or faces.min() < 0 or faces.max() >= len(points):
                raise ValueError("Invalid triangles")
            path = self.root + "/" + name
            prim = self.stage.GetPrimAtPath(path)
            parent = prim if prim else self.stage.GetPrimAtPath(self.root)
            while parent and parent.IsValid():
                if parent.HasAPI(UsdPhysics.RigidBodyAPI) or parent.HasAPI(UsdPhysics.CollisionAPI):
                    raise ValueError("Newton visual cannot be under a PhysX simulated prim")
                parent = parent.GetParent()
            topology = (len(points), faces.tobytes())
            if name in self.topology and topology != self.topology[name]:
                raise ValueError("Topology changed; reload sink")
            pending.append((name, path, points, faces, topology))
        for name, path, points, faces, topology in pending:
            mesh = UsdGeom.Mesh.Define(self.stage, path)
            inv = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(mesh.GetPrim()).GetInverse())
            local = (np.c_[points, np.ones(len(points))] @ inv)[:, :3].astype(np.float32)
            mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(local))
            mesh.GetFaceVertexCountsAttr().Set([3] * len(faces))
            mesh.GetFaceVertexIndicesAttr().Set(faces.ravel().tolist())
            mesh.GetSubdivisionSchemeAttr().Set("none")
            mesh.GetNormalsAttr().Clear()
            mesh.GetExtentAttr().Set(Vt.Vec3fArray.FromNumpy(np.array([local.min(0), local.max(0)])))
            self.topology[name] = topology
        self.stamp = stamp
