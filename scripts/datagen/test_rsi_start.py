import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

import rsi_start


def _result(root, run_id, task, scores):
    d = root / "RoboDojo" / task / "OpenWAM" / "arx_x5" / "0_ckpt_name=x,action_type=ee" / f"{run_id}_{task}"
    d.mkdir(parents=True)
    details = {str(i): {"layout_id": i, "success": s == 1, "score": s} for i, s in enumerate(scores)}
    (d / "_result.json").write_text(json.dumps({"eval_time": len(scores), "details": details}))


def test_start_eval_against_leaderboard(tmp_path):
    _result(tmp_path, "r0", "stack_bowls", [1.0] * 9 + [0.0])
    _result(tmp_path, "r0", "push_T", [0.0] * 10)
    _result(tmp_path, "r0", "fold_clothes", [0.0] * 10)
    _result(tmp_path, "other", "push_T", [1.0] * 10)
    scores = rsi_start.collect(tmp_path, "r0")
    assert sorted(scores) == ["fold_clothes", "push_T", "stack_bowls"] and scores["push_T"] == [0.0] * 10

    lb = json.loads(rsi_start.LEADERBOARD.read_text())["tasks"]
    report = rsi_start.compare(scores, lb, episodes=10, margin=5.0, max_fail=0)
    assert report["tasks"]["stack_bowls"]["pass"]  # 90 vs 89.6
    assert report["tasks"]["push_T"]["pass"]  # 0 vs 0, zero-width interval
    assert report["failed"] == ["fold_clothes"] and not report["passed"]  # 0 vs 88.27
