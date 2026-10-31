"""Genie cloth ingestion plus an explicit Newton unified VBD lifecycle.

The upstream mixin is loaded unmodified and hash checked. This is a portable
subset, not the complete Genie ROS engine. All numerical values are candidates.
"""

import hashlib
import importlib.metadata
import importlib.util
import logging
from pathlib import Path
from types import SimpleNamespace

import newton
import numpy as np
from pxr import Usd, UsdGeom
import warp as wp

GENIE_COMMIT = "30d2bc123d4eda624770dc0daa528541257de580"
CLOTH_PATH = "source/geniesim_ros/src/ros_ws/src/genie_sim_engine/scripts/engine/newton/cloth.py"
CLOTH_SHA256 = "232ae712759ff70e35d6881eeb16d7811d2bee3a2f3d92e1c037b7284e1c141e"


def load_upstream(root):
    source = Path(root) / CLOTH_PATH
    if hashlib.sha256(source.read_bytes()).hexdigest() != CLOTH_SHA256:
        raise ValueError("Genie cloth source differs from the pinned baseline")
    spec = importlib.util.spec_from_file_location("roborsr_genie_cloth", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module._ClothMixin


def volume(points, tets):
    q = points[tets]
    return np.linalg.det(np.stack([q[:, 1] - q[:, 0], q[:, 2] - q[:, 0], q[:, 3] - q[:, 0]], -1)) / 6


class Runtime:
    def __init__(self, config, genie_root, device="cpu"):
        for package, version in [("newton", "1.7.0"), ("warp-lang", "1.17.0")]:
            if importlib.metadata.version(package) != version:
                raise ValueError(f"Expected {package}=={version}")
        self.config = config
        self.dt = float(config.get("frame_dt", 1 / 60))
        self.substeps = int(config.get("substeps", 10))
        if not np.isfinite(self.dt) or self.dt <= 0 or self.substeps < 1:
            raise ValueError("Positive finite timestep and substeps required")
        wp.init()
        self.device = device
        self.builder = b = newton.ModelBuilder(gravity=(0, 0, -9.81))
        b.add_ground_plane()
        newton.solvers.SolverVBD.register_custom_attributes(b)
        self.objects = {}
        self.pins = []
        self.rest_volumes = {}
        mixin = None
        for item in config["objects"]:
            name = item["name"]
            if name in self.objects:
                raise ValueError("Duplicate object name")
            mass = float(item["mass_kg"])
            if not np.isfinite(mass) or mass <= 0:
                raise ValueError("Positive finite mass required")
            start, tri_start, tet_start = b.particle_count, len(b.tri_indices), len(b.tet_indices)
            if item["kind"] == "cloth":
                if mixin is None:
                    mixin = load_upstream(genie_root)
                stage = Usd.Stage.Open(item["usd"])
                if stage is None or not np.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1):
                    raise ValueError("Cloth must have meter units")
                if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
                    raise ValueError("Cloth must have Z up")
                mesh = UsdGeom.Mesh(stage.GetPrimAtPath(item["prim_path"]))
                if not mesh:
                    raise ValueError("Explicit mesh prim required")
                transform = UsdGeom.XformCache().GetLocalToWorldTransform(mesh.GetPrim())
                if not np.allclose(np.asarray(transform), np.eye(4), atol=1e-9):
                    raise ValueError("Bake cloth transforms into meter-scale vertices before ingestion")
                points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
                counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
                faces = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
                if not np.all(counts == 3) or not np.isfinite(points).all():
                    raise ValueError("Finite triangle mesh required")
                if faces.min() < 0 or faces.max() >= len(points):
                    raise ValueError("Invalid mesh indices")
                q = points[faces]
                areas = np.linalg.norm(np.cross(q[:, 1] - q[:, 0], q[:, 2] - q[:, 0]), axis=1) / 2
                if np.any(areas <= 1e-12):
                    raise ValueError("Degenerate cloth triangles")
                params = {
                    "tri_ke": 1500,
                    "tri_ka": 1500,
                    "tri_kd": 0.01,
                    "edge_ke": 1e-5,
                    "edge_kd": 1e-5,
                    "particle_radius": 0.0005,
                    **item.get("material", {}),
                }
                params["density"] = mass / areas.sum()
                host = SimpleNamespace(
                    _stage=stage,
                    _logger=logging.getLogger("genie_cloth"),
                    _DENSITY=params["density"],
                    _TRI_KE=params["tri_ke"],
                    _TRI_KA=params["tri_ka"],
                    _TRI_KD=params["tri_kd"],
                    _EDGE_KE=params["edge_ke"],
                    _EDGE_KD=params["edge_kd"],
                    _PARTICLE_RADIUS=params["particle_radius"],
                )
                pose = item.get("pose_xyzw", [0, 0, 0, 0, 0, 0, 1])
                mixin._inject_cloth(
                    host, b, {"name": name, "usd_prim_path": item["prim_path"], "pose": pose, "params": params}
                )
                # The upstream mixin exposes only its last particle span. Keep per-object spans here.
                assert host._cloth_particle_start == start and b.particle_count > start
                pins = np.asarray(item.get("pins", []), dtype=int)
                if np.any(pins < 0) or np.any(pins >= b.particle_count - start):
                    raise ValueError("Pin index out of bounds")
                self.pins.extend((start + pins).tolist())
            elif item["kind"] == "soft_grid":
                size = np.asarray(item["size_m"], dtype=float)
                cells = int(item.get("cells", 4))
                if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0) or cells < 1:
                    raise ValueError("Positive soft-grid size and resolution required")
                e, nu = float(item.get("young_pa", 20000)), float(item.get("poisson", 0.3))
                if not np.isfinite([e, nu]).all() or e <= 0 or not -1 < nu < 0.5:
                    raise ValueError("Invalid elastic parameters")
                center = np.asarray(item.get("center_m", [0, 0, 0.2]))
                b.add_soft_grid(
                    pos=wp.vec3(*(center - size / 2)),
                    rot=wp.quat_identity(),
                    vel=wp.vec3(0),
                    dim_x=cells,
                    dim_y=cells,
                    dim_z=cells,
                    cell_x=size[0] / cells,
                    cell_y=size[1] / cells,
                    cell_z=size[2] / cells,
                    density=mass / np.prod(size),
                    k_mu=e / (2 * (1 + nu)),
                    k_lambda=e * nu / ((1 + nu) * (1 - 2 * nu)),
                    k_damp=5,
                    particle_radius=0.0005,
                )
                tets = np.asarray(b.tet_indices[tet_start:], dtype=int) - start
                points = np.asarray(b.particle_q[start:])
                volumes = volume(points, tets)
                lumped = np.zeros(len(points))
                for k in range(4):
                    np.add.at(lumped, tets[:, k], abs(volumes) / 4)
                lumped *= mass / lumped.sum()
                b.particle_mass[start:] = lumped.tolist()
                self.rest_volumes[name] = (tets, volumes)
            else:
                raise ValueError("Supported kinds: cloth, soft_grid")
            end = b.particle_count
            faces = np.asarray(b.tri_indices[tri_start:], dtype=int).reshape(-1, 3) - start
            actual = float(np.sum(b.particle_mass[start:end]))
            if not np.isclose(actual, mass, rtol=1e-5):
                raise ValueError("Mesh mass does not match registration")
            self.objects[name] = dict(
                start=start,
                end=end,
                faces=faces.tolist(),
                mass_kg=actual,
                asset_id=item.get("asset_id"),
                kind=item["kind"],
            )
        self.bodies = {}
        self.kinematic = set()
        for item in config.get("bodies", []):
            name = item["name"]
            if name in self.bodies:
                raise ValueError("Duplicate rigid-body name")
            kinematic = bool(item.get("kinematic", False))
            body = b.add_body(
                xform=wp.transform(item["pose_xyzw"][:3], item["pose_xyzw"][3:]), is_kinematic=kinematic, label=name
            )
            cfg = newton.ModelBuilder.ShapeConfig(density=float(item.get("density", 1000)))
            cfg.has_particle_collision = bool(item.get("particle_contact", True))
            if item["shape"] == "sphere":
                b.add_shape_sphere(body, radius=item["radius_m"], cfg=cfg)
            elif item["shape"] == "box":
                sx, sy, sz = item["size_m"]
                b.add_shape_box(body, hx=sx / 2, hy=sy / 2, hz=sz / 2, cfg=cfg)
            else:
                raise ValueError("Unsupported body shape")
            self.bodies[name] = body
            if kinematic:
                self.kinematic.add(name)
        b.color(include_bending=True)
        self.model = b.finalize(device=device)
        self.model.soft_contact_ke = 100000
        self.model.soft_contact_kd = 10
        self.model.soft_contact_mu = 0.5
        flags = self.model.particle_flags.numpy()
        flags[self.pins] &= np.bitwise_not(np.asarray(int(newton.ParticleFlags.ACTIVE), dtype=flags.dtype))
        self.model.particle_flags.assign(flags)
        self.epoch = -1
        self.reset()

    def reset(self):
        self.state, self.next_state = self.model.state(), self.model.state()
        self.control = self.model.control()
        self.pipeline = newton.CollisionPipeline(self.model, soft_contact_gap=0.0005, deterministic=True)
        self.contacts = self.pipeline.contacts()
        self.solver = newton.solvers.SolverVBD(
            self.model,
            iterations=int(self.config.get("iterations", 12)),
            integrate_with_external_rigid_solver=False,
            rigid_compliant_alm=False,
            particle_enable_self_contact=True,
            particle_self_contact_margin=0.001,
            particle_self_contact_gap=0.0005,
            particle_topological_contact_filter_threshold=2,
            particle_vertex_contact_buffer_size=32,
            particle_edge_contact_buffer_size=32,
            rigid_body_particle_contact_buffer_size=4096,
            deterministic=wp.DeterministicMode.RUN_TO_RUN,
        )
        from real2sim_deformables.diagnostics import track_overflows

        self.detector_overflow = track_overflows(self.solver.trimesh_collision_detector)
        self.frame, self.epoch = 0, self.epoch + 1
        self.overflow = False
        return self.snapshot()

    def set_kinematic(self, name, pose_xyzw, velocity):
        if name not in self.kinematic:
            raise ValueError("Only declared kinematic proxies accept poses")
        pose, vel = np.asarray(pose_xyzw, dtype=float), np.asarray(velocity, dtype=float)
        if pose.shape != (7,) or vel.shape != (6,) or not np.isfinite(np.r_[pose, vel]).all():
            raise ValueError("Finite pose[7] and velocity[6] required")
        if not np.isclose(np.linalg.norm(pose[3:]), 1, atol=1e-6):
            raise ValueError("Non-unit quaternion")
        for state in (self.state, self.next_state):
            q, qd = state.body_q.numpy(), state.body_qd.numpy()
            q[self.bodies[name]], qd[self.bodies[name]] = pose, vel
            state.body_q.assign(q)
            state.body_qd.assign(qd)

    def step(self, frames=1):
        if type(frames) is not int or not 1 <= frames <= 600:
            raise ValueError("frames must be 1..600")
        for _ in range(frames):
            for _ in range(self.substeps):
                self.state.clear_forces()
                self.next_state.clear_forces()
                self.pipeline.collide(self.state, self.contacts)
                self.solver.step(self.state, self.next_state, self.control, self.contacts, self.dt / self.substeps)
                self.state, self.next_state = self.next_state, self.state
                self.overflow |= bool(np.any(self.detector_overflow.numpy()[:4]))
                self.overflow |= int(self.contacts.soft_contact_count.numpy()[0]) > self.contacts.soft_contact_max
                self.overflow |= int(self.contacts.rigid_contact_count.numpy()[0]) > self.contacts.rigid_contact_max
            self.frame += 1
        return self.snapshot()

    def snapshot(self):
        q, qd = self.state.particle_q.numpy(), self.state.particle_qd.numpy()
        if not np.isfinite(q).all() or not np.isfinite(qd).all():
            raise FloatingPointError("Nonfinite node state")
        objects = {}
        for name, spec in self.objects.items():
            points = q[spec["start"] : spec["end"]]
            item = {**spec, "points_m": points.tolist(), "velocities_m_s": qd[spec["start"] : spec["end"]].tolist()}
            if name in self.rest_volumes:
                tets, rest = self.rest_volumes[name]
                item["minimum_J"] = float(np.min(volume(points, tets) / rest))
            objects[name] = item
        return dict(
            schema="roborsr.genie_deformables.v1",
            epoch=self.epoch,
            frame=self.frame,
            time_s=self.frame * self.dt,
            frame_dt_s=self.dt,
            objects=objects,
            bodies={n: self.state.body_q.numpy()[i].tolist() for n, i in self.bodies.items()},
            contact_overflow=self.overflow,
            material_calibrated=False,
            genie_commit=GENIE_COMMIT,
            physics="Newton unified VBD; external robot proxies are one-way",
            device=str(self.model.device),
        )
