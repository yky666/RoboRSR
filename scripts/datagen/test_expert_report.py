"""expert_report.py on synthetic episodes (no Isaac): python -m pytest scripts/datagen/test_expert_report.py"""

import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.datagen import expert_report as R  # noqa: E402

h5py = pytest.importorskip("h5py")


def write_episode(run, layout_id, quality, joint_jump=0.0, fling=False):
    run.mkdir(parents=True, exist_ok=True)
    stem = run / f"episode_{layout_id:06d}"
    q = dict(layout_seed=1000, layout_id=layout_id, fps=25, frames=50, **quality)
    stem.with_suffix(".json").write_text(json.dumps(q))
    t = np.linspace(0, 1, 50, dtype=np.float32)[:, None]
    with h5py.File(stem.with_suffix(".hdf5"), "w") as f:
        for side in ("left", "right"):
            joints = np.repeat(0.5 * t, 6, axis=1)
            joints[25:] += joint_jump
            f[f"state/{side}_arm_joint_states"] = joints
            f[f"state/{side}_ee_poses"] = np.hstack([0.3 * t, 0 * t, 1 + 0 * t, 1 + 0 * t, 0 * t, 0 * t, 0 * t])
        bowls = np.zeros((50, 3, 7), np.float32)
        bowls[:, :, 2], bowls[:, :, 3] = 0.79, 1.0
        if fling:
            bowls[30:, 1, :2] = 0.5
        f["privileged/bowl_poses"] = bowls


def test_wilson():
    assert R.wilson(0, 0) == [0.0, 0.0]
    low, high = R.wilson(7, 12)
    assert low < 7 / 12 < high and 0 <= low and high <= 1


def test_outcome():
    assert R.outcome(dict(accepted=True)) == "accepted"
    assert R.outcome(dict(accepted=False, reason="bowl out of reach")) == "rejected: bowl out of reach"
    failure = "place bowl2: right IK fails at [0.079, -0.081, 1.133]"
    assert R.outcome(dict(accepted=False, failure=failure)) == "expert: place bowl: right IK fails at"
    checks = [["is_stacked{}", False], ["all_robot_back_to_origin{}", True]]
    assert R.outcome(dict(accepted=False, final_checks=checks)) == "task checks: is_stacked{}"
    assert R.outcome(dict(accepted=False)) == "task checks: held too briefly"


def test_report(tmp_path, capsys):
    run = tmp_path / "stack_bowls"
    write_episode(run, 0, dict(accepted=True))
    write_episode(run, 1, dict(accepted=False, failure="pick bowl1: left IK jumps 0.52 rad at [0.1, 0.2, 1.0]"))
    write_episode(run, 2, dict(accepted=False, final_checks=[["is_stacked{}", False]]), joint_jump=0.4, fling=True)
    out = tmp_path / "report"
    R.main([str(run), "--out", str(out), "--sheets", "0"])
    report = json.loads((out / "report.json").read_text())
    s = report["summary"]
    assert (s["episodes"], s["accepted"], s["accepted_frames"]) == (3, 1, [50, 50, 50])
    jump = "expert: pick bowl: left IK jumps rad at"
    assert s["outcomes"] == {"accepted": 1, jump: 1, "task checks: is_stacked{}": 1}
    assert s["flags"] == {"left joint step": 1, "right joint step": 1, "bowl flung": 1}
    assert report["episodes"][0]["motion"]["left"]["link6_step_m"] < R.LINK6_STEP_M

    R.main([str(run), "--out", str(tmp_path / "again"), "--sheets", "0", "--baseline", str(out / "report.json")])
    assert "(baseline 1/3 = 33%)" in capsys.readouterr().out
