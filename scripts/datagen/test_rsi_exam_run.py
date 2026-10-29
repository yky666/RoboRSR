import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

import rsi_exam_run


def test_assign_balances_lanes_and_keeps_tasks_whole():
    jobs = [{"task": "fold_clothes", "episodes": 6}] * 4 + [{"task": "stack_bowls", "episodes": 2}] * 6
    jobs += [{"task": "push_T", "episodes": 10}]
    plan = rsi_exam_run.assign(jobs, [(0, 3), (1, 4)], env1={"fold_clothes"})
    assert sorted(t for ts in plan.values() for t in ts) == ["fold_clothes", "push_T", "stack_bowls"]
    assert plan[(0, 3)] == ["fold_clothes"]  # 4 starts + 24 single-env batches outweigh the rest


def test_gate_needs_exit_zero_and_every_planned_episode(tmp_path):
    for name, n in (("a", 3), ("b", 2)):
        (tmp_path / name).mkdir()
        details = {str(i): {"layout_id": i + 1, "score": 1.0} for i in range(n)}
        (tmp_path / name / "_result.json").write_text(json.dumps({"eval_time": n, "details": details}))
    results = [
        {"job": "a", "rc": 0, "dir": str(tmp_path / "a"), "planned": 3},
        {"job": "b", "rc": 0, "dir": str(tmp_path / "b"), "planned": 3},
        {"job": "c", "rc": -9, "dir": "", "planned": 1},
        {"job": "d", "rc": 0, "dir": str(tmp_path / "b"), "planned": 3},
    ]
    log = "[PhysX] broken envs=[{0}] -> abandon seeds=[{0}]; refill from queue=[]\n"
    (tmp_path / "d.log").write_text(log.format(0) + log.format(1))
    assert rsi_exam_run.gate(results, tmp_path) == ["b", "c"]
    assert results[3]["unstable"] == [0]  # 1 was abandoned, then scored after a restart
