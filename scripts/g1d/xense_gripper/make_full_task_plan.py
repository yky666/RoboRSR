import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as E

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

p = argparse.ArgumentParser(description="Build a bounded nominal pick/place IK plan; validate collisions in simulation")
p.add_argument("--out", type=Path, required=True)
p.add_argument("--x-offset", type=float, default=0.0)
a = p.parse_args()
if abs(a.x_offset) > 0.02:
    raise ValueError("Layout offset outside reviewed range")
R = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(R))
from scripts.g1d.xense_gripper.urdf_tools import fk  # noqa: E402

root = E.parse(R / "real2sim/robot/g1d_aurora_lite_ws/g1d_aurora_lite_ws.urdf").getroot()
res = {}
for side, sy in [("left", 1), ("right", -1)]:
    names = [
        side + "_" + x + "_joint"
        for x in ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"]
    ]
    limits = [root.find(f"joint[@name='{n}']/limit") for n in names]
    lo = np.array([float(x.get("lower")) for x in limits])
    hi = np.array([float(x.get("upper")) for x in limits])
    rot = Rotation.from_euler("y", 0.3).as_matrix()
    q = np.array([0, sy * 0.2, 0, -0.5, 0, 1.4, 0])
    data = {}
    for name, p in [
        ("source", [0.35, sy * 0.23, 0.72]),
        ("pregrasp", [0.35, sy * 0.23, 0.82]),
        ("lift", [0.35, sy * 0.23, 0.84]),
        ("transfer", [0.43, sy * 0.25, 0.84]),
        ("place", [0.43, sy * 0.25, 0.72]),
        ("retreat", [0.43, sy * 0.25, 0.86]),
    ]:
        p = np.asarray(p, dtype=float)
        p[0] += a.x_offset

        def cost(x):
            t = fk(root, dict(zip(names, x)))[side + "_aurora_tcp_link"]
            return np.r_[(t[:3, 3] - p) * 10, Rotation.from_matrix(rot.T @ t[:3, :3]).as_rotvec()]

        sol = least_squares(
            cost,
            np.clip(q, lo + 1e-5, hi - 1e-5),
            bounds=(lo + 1e-6, hi - 1e-6),
            max_nfev=300,
            xtol=1e-11,
            gtol=1e-11,
            ftol=1e-11,
        )
        for attempt in range(12):
            if np.linalg.norm(sol.fun) < 1e-5:
                break
            alt = least_squares(
                cost, np.random.default_rng(102 + attempt).uniform(lo, hi), bounds=(lo + 1e-6, hi - 1e-6), max_nfev=200
            )
            if np.linalg.norm(alt.fun) < np.linalg.norm(sol.fun):
                sol = alt
        q = sol.x
        err = np.linalg.norm(cost(q))
        print(side, name, err, np.round(q, 3))
        assert err < 0.01
        data[name] = {"q": q.tolist(), "tcp_m": p.tolist(), "fit_error": err}
    trajectories = {}
    q = np.array(data["pregrasp"]["q"])
    previous = "pregrasp"
    for phase, goal_name, count in [
        ("approach", "source", 15),
        ("lift", "lift", 20),
        ("transport", "transfer", 20),
        ("lower", "place", 20),
        ("retreat", "retreat", 15),
    ]:
        start_pos = np.array(data[previous]["tcp_m"])
        goal_pos = np.array(data[goal_name]["tcp_m"])
        sequence = []
        for index in range(count):
            blend = (index + 1) / count
            blend = blend * blend * (3 - 2 * blend)
            p = start_pos + (goal_pos - start_pos) * blend
            prior = q.copy()

            def smooth_cost(value):
                return np.r_[cost(value), (value - prior) * 1e-4]

            fitted = least_squares(smooth_cost, q, bounds=(lo + 1e-6, hi - 1e-6), max_nfev=100)
            q = fitted.x
            if np.linalg.norm(cost(q)) > 1e-4:
                raise ValueError(f"Unreachable Cartesian waypoint: {side}/{phase}/{index}")
            sequence.append(q.tolist())
        trajectories[phase] = sequence
        previous = goal_name
    res[side] = {"joint_names": names, "waypoints": data, "trajectories": trajectories}
a.out.write_text(json.dumps(res, indent=2))
