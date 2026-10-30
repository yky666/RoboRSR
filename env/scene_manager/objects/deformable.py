"""Opt-in PhysX FEM wrapper with explicit nodal restoration between episodes."""

from isaaclab.assets import DeformableObject as NativeDeformable, DeformableObjectCfg
import numpy as np
from pxr import Gf, UsdGeom
from scipy.spatial.transform import Rotation
import torch

from env.scene_manager.objects.dynamic import DynamicObject


class DeformableObject(DynamicObject):
    """Keep generic Dynamic assets unchanged; selected by an explicit FEM backend."""

    def __init__(self, **kwargs):
        self._visual_bindings = []
        self._visual_callback_id = None
        self._visual_update_count = 0
        self._logical_world = None
        super().__init__(**kwargs)
        self.fem = NativeDeformable(DeformableObjectCfg(prim_path=self._prim_path, spawn=None))
        self._rest_local = None
        self._free_flags = None
        self._rest_scale = np.asarray(self.scale, dtype=float).copy()

    def initialize(self):
        super().initialize()
        descriptors = self.physics_config.get("fem_visual_meshes", [])
        if not descriptors or self._visual_bindings:
            return
        import hashlib

        from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager

        from env.scene_manager.objects.tetra_visuals import bind_tetrahedra

        self._capture_rest()
        nodes = self.fem.data.default_nodal_state_w[0, :, :3].detach().cpu().numpy()
        tets = self.fem.root_physx_view.get_sim_element_indices()[0].detach().cpu().numpy()
        for descriptor in descriptors:
            relative = descriptor["prim"]
            if relative.startswith("/") or ".." in relative.split("/"):
                raise ValueError("FEM visual meshes must remain below the asset root")
            mesh = UsdGeom.Mesh.Get(self.stage, self._prim_path + "/" + relative)
            points = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float32)
            face_indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int32)
            digest = hashlib.sha256(points.astype("<f4").tobytes() + face_indices.astype("<i4").tobytes()).hexdigest()
            if digest != descriptor["mesh_sha256"]:
                raise ValueError("Visual mesh changed since packaging")
            matrix = UsdGeom.XformCache().GetLocalToWorldTransform(mesh.GetPrim())
            world = np.array([matrix.Transform(Gf.Vec3d(*map(float, point))) for point in points])
            indices, weights, diagnostics = bind_tetrahedra(world, nodes, tets)
            UsdGeom.Xformable(mesh).SetResetXformStack(True)
            self._visual_bindings.append(dict(mesh=mesh, indices=indices, weights=weights, diagnostics=diagnostics))
        self._visual_callback_id = SimulationManager.register_callback(
            self._sync_visual_meshes, IsaacEvents.POST_PHYSICS_STEP, name=self._prim_path + "_tetra_visuals"
        )
        self._sync_visual_meshes()

    def _sync_visual_meshes(self, _dt=None):
        if not self._visual_bindings:
            return
        if not self.stage.GetPrimAtPath(self._prim_path).IsValid():
            self.close_visual_meshes()
            return
        from pxr import Vt

        from env.scene_manager.objects.tetra_visuals import apply_embedding

        nodes = self.fem.root_physx_view.get_sim_nodal_positions()[0].detach().cpu().numpy()
        for binding in self._visual_bindings:
            points = apply_embedding(nodes, binding["indices"], binding["weights"])
            binding["mesh"].GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
        self._visual_update_count += 1

    def close_visual_meshes(self):
        if self._visual_callback_id is not None:
            from isaacsim.core.simulation_manager import SimulationManager

            SimulationManager.deregister_callback(self._visual_callback_id)
            self._visual_callback_id = None
        self._visual_bindings.clear()

    def __del__(self):
        try:
            self.close_visual_meshes()
        except Exception:
            pass

    def get_world_pose(self):
        if self._logical_world is None:
            return super().get_world_pose()
        return tuple(self._backend_utils.convert(v, self._device) for v in self._logical_world)

    def _parent_matrix(self):
        parent = self.stage.GetPrimAtPath(self._prim_path).GetParent()
        return UsdGeom.XformCache().GetLocalToWorldTransform(parent)

    @staticmethod
    def _matrix(pos, ori):
        matrix = Gf.Matrix4d().SetRotate(Gf.Quatd(float(ori[0]), Gf.Vec3d(*map(float, ori[1:]))))
        matrix.SetTranslateOnly(Gf.Vec3d(*map(float, pos)))
        return matrix

    @staticmethod
    def _pose(matrix):
        quat = matrix.ExtractRotationQuat()
        return np.asarray(matrix.ExtractTranslation()), np.asarray([quat.GetReal(), *quat.GetImaginary()])

    def get_local_pose(self):
        if self._logical_world is None:
            return super().get_local_pose()
        matrix = self._matrix(*self._logical_world) * self._parent_matrix().GetInverse()
        return tuple(self._backend_utils.convert(v, self._device) for v in self._pose(matrix))

    def set_local_pose(self, translation=None, orientation=None):
        if not hasattr(self, "fem") or not self.fem.is_initialized:
            return super().set_local_pose(translation=translation, orientation=orientation)
        self._capture_rest()
        self._validate_scale()
        old_pos, old_ori = self.get_local_pose()
        matrix = self._matrix(
            translation if translation is not None else old_pos, orientation if orientation is not None else old_ori
        )
        self._logical_world = self._pose(matrix * self._parent_matrix())
        self.restore_nodes()

    def set_world_pose(self, position=None, orientation=None):
        if not hasattr(self, "fem") or not self.fem.is_initialized:
            return super().set_world_pose(position=position, orientation=orientation)
        self._capture_rest()
        self._validate_scale()
        old_pos, old_ori = self.get_world_pose()
        matrix = self._matrix(
            position if position is not None else old_pos, orientation if orientation is not None else old_ori
        )
        self._logical_world = self._pose(matrix)
        self.restore_nodes()

    def set_local_scale(self, scale):
        if hasattr(self, "fem") and self.fem.is_initialized:
            value = scale.detach().cpu().numpy() if isinstance(scale, torch.Tensor) else scale
            if not np.allclose(value, self._rest_scale, atol=1e-8, rtol=0):
                raise ValueError("Live FEM scale changes require recooking; reload the asset instead")
            return
        return super().set_local_scale(scale)

    def _validate_scale(self):
        if not np.allclose(self.scale, self._rest_scale, atol=1e-8, rtol=0):
            raise ValueError("Live FEM scale changes require recooking; reload the asset instead")

    def _frame(self):
        pos, ori = self.get_world_pose()
        if isinstance(pos, torch.Tensor):
            pos = pos.detach().cpu().numpy()
        if isinstance(ori, torch.Tensor):
            ori = ori.detach().cpu().numpy()
        pos, ori = np.asarray(pos), np.asarray(ori)
        rotation = Rotation.from_quat(ori[[1, 2, 3, 0]]).as_matrix()
        return pos, ori, rotation

    def _capture_rest(self):
        if not self.fem.is_initialized:
            raise RuntimeError("FEM state requires a running/reset simulation before nodal reset")
        if self._rest_local is not None:
            return
        pos, _, rotation = self._frame()
        nodes = self.fem.data.default_nodal_state_w[..., :3].detach().cpu().numpy()
        scale = self._rest_scale
        if np.any(scale <= 0):
            raise ValueError("FEM scale must be positive")
        self._rest_local = ((nodes - pos) @ rotation) / scale
        self._free_flags = self.fem.data.nodal_kinematic_target[..., 3:].clone()

    def restore_nodes(self):
        """Restore undeformed positions, zero velocity and original free/fixed flags."""
        self._capture_rest()
        self._validate_scale()
        pos, _, rotation = self._frame()
        points = (self._rest_local * np.asarray(self.scale)) @ rotation.T + pos
        state = torch.zeros_like(self.fem.data.default_nodal_state_w)
        state[..., :3] = torch.as_tensor(points, device=state.device, dtype=state.dtype)
        targets = torch.cat((state[..., :3], self._free_flags), dim=-1)
        self.fem.write_nodal_state_to_sim(state)
        self.fem.write_nodal_kinematic_target_to_sim(targets)
        self.fem.update(0.0)
        self._sync_visual_meshes()

    def apply_saved_pose(self):
        # PhysX skinning bakes the original USD frame. Move world-space nodes,
        # keeping the live USD root stable to avoid applying placement twice.
        self._capture_rest()
        self._validate_scale()
        self.set_local_pose(translation=self.default_pos, orientation=self.default_ori)

    def relocate_offscreen(self):
        self._capture_rest()
        # At 100km float32 cannot resolve this asset's millimetre FEM edges.
        self.set_local_pose(translation=np.array([100.0, 100.0, 100.0]), orientation=self.default_ori)

    def sample_mesh_vertices(self):
        if not self.fem.is_initialized:
            return super().sample_mesh_vertices()
        points = self.fem.root_physx_view.get_sim_nodal_positions()[0].detach().cpu().numpy()
        pos, ori, rotation = self._frame()
        return points, (points - pos) @ rotation, pos, ori
