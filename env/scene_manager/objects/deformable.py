import random

from isaaclab.assets import DeformableObject as FemAsset, DeformableObjectCfg
import isaaclab.sim as sim_utils
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.stage import get_current_stage
import numpy as np
from omegaconf import DictConfig
from pxr import PhysxSchema, Usd, UsdShade
import torch

from env.scene_manager.layout_manager import LayoutManager


class DeformableObject:
    """PhysX FEM volume deformable (isaaclab.assets.DeformableObject) from an asset USD that carries its own
    deformable-body schema and material. Placed, reset and parked offscreen through the nodal state; the pose used by
    the layout and the experts is the rigid fit of the nodes to the rest shape. Needs the GPU physics pipeline."""

    physics_type = "deformable"
    _FAR_CENTER = (100.0, 100.0, 100.0)  # FEM nodes need float32 precision: not as far as the rigid 100 km
    _FAR_JITTER = 1.0

    def __init__(
        self,
        prim_path: str,
        usd_path: str,
        inst_config: DictConfig,
        env_origin: torch.Tensor,
        default_pos: tuple,
        default_ori: tuple,
        scale: tuple,
        scene=None,
    ):
        if not usd_path:
            raise ValueError("DeformableObject requires a valid usd_path.")
        self._device = str(SimulationManager.get_physics_sim_device())
        if not self._device.startswith("cuda"):
            raise RuntimeError(
                f"Deformable object '{prim_path}' needs the GPU physics pipeline (PhysX FEM soft bodies are GPU-only); "
                f"the simulation device is '{self._device}'."
            )
        self.prim_path = prim_path
        self.usd_prim_path = prim_path
        prim_path_parts = prim_path.split("/")
        self.category_name = prim_path_parts[-2]
        self.instance_name = prim_path_parts[-1]
        self.model_name, self.model_id = LayoutManager.decode_inst_name(self.instance_name)
        self.instance_config = inst_config
        self.physics_config = self.instance_config.get("physics", {})
        self.env_origin = env_origin
        if isinstance(self.env_origin, torch.Tensor):
            self.env_origin = self.env_origin.detach().cpu().numpy().tolist()
        self.usd_path = usd_path
        self.default_pos = default_pos
        self.default_ori = default_ori
        self.scale = tuple(float(s) for s in scale)
        self._cooked_scale = self.scale
        self.fem = FemAsset(
            DeformableObjectCfg(
                prim_path=prim_path,
                spawn=sim_utils.UsdFileCfg(usd_path=usd_path, scale=self.scale),
                init_state=DeformableObjectCfg.InitialStateCfg(
                    pos=tuple(float(v) for v in default_pos), rot=tuple(float(v) for v in default_ori)
                ),
            )
        )
        self._set_friction(self.physics_config.get("dynamic_friction"))
        self._set_body_attributes(self.physics_config.get("deformable_body", {}))
        if scene is not None:
            scene.deformable_objects[prim_path] = self.fem  # written and updated every physics step by the scene
        self._rest_local = None

    def _set_friction(self, friction):
        """Override the asset's deformable material friction before the simulation starts (layout `physics`)."""
        if friction is None:
            return
        stage = get_current_stage()
        for prim in Usd.PrimRange(stage.GetPrimAtPath(self.prim_path)):
            if prim.HasAPI(PhysxSchema.PhysxDeformableBodyAPI):
                for target in UsdShade.MaterialBindingAPI(prim).GetDirectBindingRel("physics").GetTargets():
                    material = PhysxSchema.PhysxDeformableBodyMaterialAPI(stage.GetPrimAtPath(target))
                    if material:
                        material.CreateDynamicFrictionAttr().Set(float(friction))

    def _set_body_attributes(self, attributes):
        """Override PhysxDeformableBodyAPI attributes (e.g. {"sleepThreshold": 0.0}) before the simulation starts;
        attributes that change the cooked meshes (resolution, simplification) are not meant to be set here."""
        stage = get_current_stage()
        for prim in Usd.PrimRange(stage.GetPrimAtPath(self.prim_path)):
            if prim.HasAPI(PhysxSchema.PhysxDeformableBodyAPI):
                for name, value in attributes.items():
                    attr = prim.GetAttribute(f"physxDeformable:{name}")
                    if not attr:
                        raise ValueError(f"'{self.prim_path}' has no deformable attribute physxDeformable:{name}")
                    attr.Set(type(attr.Get())(value) if attr.Get() is not None else value)

    def initialize(self):
        """IsaacLab initializes the asset when the simulation plays; nothing to do here."""

    @property
    def view(self):
        return self.fem.root_physx_view

    def _origin(self):
        return torch.tensor(self.env_origin, dtype=torch.float32, device=self._device)

    def rest_local(self) -> torch.Tensor:
        """Rest nodal positions in the asset frame (from IsaacLab's default nodal state at the spawn pose)."""
        if self._rest_local is None:
            nodes = self.fem.data.default_nodal_state_w[0, :, :3]
            pos = self._origin() + torch.tensor(self.fem.cfg.init_state.pos, device=self._device)
            rot = _quat_to_matrix(torch.tensor(self.fem.cfg.init_state.rot, device=self._device))
            self._rest_local = (nodes - pos) @ rot
        return self._rest_local

    def nodal_positions(self, is_relative: bool = True) -> torch.Tensor:
        nodes = self.view.get_sim_nodal_positions()[0]
        return nodes - self._origin() if is_relative else nodes

    def _write_pose(self, translation, orientation):
        # No kinematic targets: in the G1-D env, writing them (even all flags 1.0, "free") left the body frozen.
        if not self.fem.is_initialized:
            return
        rot = _quat_to_matrix(torch.tensor(orientation, dtype=torch.float32, device=self._device))
        pos = self._origin() + torch.tensor(translation, dtype=torch.float32, device=self._device)
        nodes = self.rest_local() @ rot.T + pos
        state = torch.cat([nodes, torch.zeros_like(nodes)], dim=-1).unsqueeze(0)
        self.fem.write_nodal_state_to_sim(state)

    def _check_scale(self):
        if tuple(float(s) for s in self.scale) != self._cooked_scale:
            raise ValueError(
                f"Deformable '{self.prim_path}' cannot change scale at runtime ({self._cooked_scale} -> {self.scale}):"
                " the scale is baked into the cooked FEM meshes; spawn a new object instead."
            )

    def apply_saved_pose(self):
        """Rest shape at the saved eval-layout pose, zero nodal velocities."""
        self._check_scale()
        self._write_pose(self.default_pos, self.default_ori)

    def relocate_offscreen(self):
        """Rest shape parked away from the scene; it falls freely there until the next apply_saved_pose."""
        self._check_scale()
        far = [c + random.uniform(-self._FAR_JITTER, self._FAR_JITTER) for c in self._FAR_CENTER]
        self._write_pose(far, self.default_ori)

    def _get_object_transform(self, device=None):
        """Pose (env frame) of the rigid transform that best maps the rest nodes onto the current nodes (Kabsch)."""
        rest = self.rest_local()
        nodes = self.nodal_positions()
        rest_c, nodes_c = rest.mean(0), nodes.mean(0)
        u, _, vt = torch.linalg.svd((rest - rest_c).T @ (nodes - nodes_c))
        d = torch.sign(torch.linalg.det(vt.T @ u.T))
        rot = vt.T @ torch.diag(torch.tensor([1.0, 1.0, float(d)], device=rest.device)) @ u.T
        pos = nodes_c - rot @ rest_c
        device = device or "cpu"  # callers combine poses with numpy, as the CPU-pipeline rigid poses allow
        return pos.to(device), _matrix_to_quat(rot).to(device), device

    def get_bbox(self, is_relative: bool = True):
        pos, ori, _ = self._get_object_transform()
        rest = self.rest_local()
        bbox = torch.cat([rest.min(0).values, rest.max(0).values]).cpu().numpy()
        pos = pos.cpu().numpy()
        if not is_relative:
            pos = pos + np.asarray(self.env_origin, dtype=np.float32)
        return pos, ori.cpu().numpy(), bbox


def _quat_to_matrix(q: torch.Tensor) -> torch.Tensor:
    w, x, y, z = q.tolist()
    return torch.tensor(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=torch.float32,
        device=q.device,
    )


def _matrix_to_quat(m: torch.Tensor) -> torch.Tensor:
    from scipy.spatial.transform import Rotation

    x, y, z, w = Rotation.from_matrix(m.detach().cpu().numpy()).as_quat()
    return torch.tensor([w, x, y, z], dtype=torch.float32)
