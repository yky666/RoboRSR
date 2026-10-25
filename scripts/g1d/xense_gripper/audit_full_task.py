"""Read-only recomputation of free-body pick/place acceptance from saved states."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def audit(directory):
    scene = json.loads((directory / "scene_schema.json").read_text())
    frames = [json.loads(line) for line in (directory / "frames.jsonl").read_text().splitlines()]
    complete = json.loads((directory / "recording_complete.json").read_text())
    checks = dict(
        complete=complete["frames"] == len(frames),
        gravity=scene["gravity_m_s2"] == [0, 0, -9.81],
        no_attachment=not scene["objects_attached"],
        no_reset=scene["object_resets_after_initial_layout"] == 0,
        no_fake_force=not scene["artificial_holding_forces"],
        all_phases=all(
            any(f["phase"] == p for f in frames)
            for p in ["initial", "approach", "close", "lift", "transport", "lower", "release", "retreat", "final"]
        ),
    )
    sides = {}
    for side in ["left", "right"]:
        pos = np.array([f["objects"][side]["pose_wxyz"][:3] for f in frames])
        quat = np.array([f["objects"][side]["pose_wxyz"][3:] for f in frames])
        if not np.allclose(np.linalg.norm(quat, axis=1), 1, atol=1e-3):
            raise ValueError("Invalid object orientation")
        w, x, y, z = quat.T
        rotation_z = np.c_[2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]
        half_height = np.abs(rotation_z) @ (np.array(scene["objects"][side]["size_m"]) / 2)
        vel = np.array([f["objects"][side]["velocity_w"][:3] for f in frames])
        final = [i for i, f in enumerate(frames) if f["phase"] == "final"][-10:]
        lifted = [i for i, f in enumerate(frames) if f["phase"] in ("lift_hold", "transport", "transport_hold")]
        goal = np.array(scene["target_centres_m"][side])
        gap = np.array(
            [
                np.linalg.norm(
                    np.array(f["pad_world"][side + "_positive"])[:3, 3]
                    - np.array(f["pad_world"][side + "_negative"])[:3, 3]
                )
                for f in frames
            ]
        )
        c = dict(
            initial_on_table=abs(pos[0, 2] - 0.03 - scene["table_top_m"]) < 0.003,
            actual_lift=max(pos[lifted, 2]) - pos[0, 2] > 0.06,
            horizontal_transport=np.linalg.norm(pos[final, :2].mean(0) - pos[0, :2]) > 0.06,
            placed_in_goal=max(np.linalg.norm(pos[final, :2] - goal[:2], axis=1)) < 0.025,
            resting_on_table=max(abs(pos[final, 2] - half_height[final] - scene["table_top_m"])) < 0.003,
            upright_final=min(rotation_z[final, 2]) > np.cos(np.deg2rad(10)),
            released=max(np.linalg.norm(vel[final], axis=1)) < 0.03 and min(gap[final]) > 0.08,
        )
        c = {k: bool(v) for k, v in c.items()}
        sides[side] = dict(
            checks=c,
            passed=all(c.values()),
            max_lift_m=float(max(pos[:, 2]) - pos[0, 2]),
            final_position_m=pos[final].mean(0).tolist(),
            target_m=goal.tolist(),
            final_upright_within_10deg=bool(min(rotation_z[final, 2]) > np.cos(np.deg2rad(10))),
            placement_orientation_requirement="strict data quality: final original +Z within 10 degrees of world +Z",
            peak_contact_n={
                sign: max(
                    float(np.linalg.norm(f["native_contact_normal_force_w_n"][side + "_" + sign])) for f in frames
                )
                for sign in ["positive", "negative"]
            },
        )
    result = dict(
        schema="g1d_free_pick_place.v1",
        passed=all(checks.values()) and all(s["passed"] for s in sides.values()),
        protocol_checks=checks,
        sides=sides,
        frames=len(frames),
        duration_s=frames[-1]["simulation_time_s"] - frames[0]["simulation_time_s"],
        source_frames_sha256=hashlib.sha256((directory / "frames.jsonl").read_bytes()).hexdigest(),
        calibrated_hardware=False,
        scope="one dual-arm task episode with candidate parameters; not a general grasp success rate",
    )
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("recording", type=Path)
    p.add_argument("--out", type=Path, required=True)
    a = p.parse_args()
    r = audit(a.recording)
    a.out.write_text(json.dumps(r, indent=2) + "\n")
    print(json.dumps(r))
