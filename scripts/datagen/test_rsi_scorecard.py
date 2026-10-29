import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

import rsi_scorecard


def _lay(task, seed, i, axis, level, capability=False):
    return {"task": task, "seed": seed, "layout_id": i, "axis": axis, "level": level, "capability": capability}


def _result(root, run_id, task, seed, scores):
    d = (
        root
        / "RoboDojo"
        / task
        / "OpenWAM"
        / "arx_x5"
        / f"{seed}_ckpt_name=x,action_type=ee"
        / f"{run_id}_{task}_arx_x5_{seed}"
    )
    d.mkdir(parents=True)
    details = {str(i): {"layout_id": i, "success": s == 1, "score": s} for i, s in enumerate(scores)}
    (d / "_result.json").write_text(json.dumps({"eval_time": len(scores), "details": details}))


def test_retention_levels_and_banding(tmp_path):
    layouts = [_lay("a", 5100, i, None, "L0") for i in range(4)] + [_lay("b", 5100, i, None, "L0") for i in range(4)]
    layouts += [_lay("a", 5121, i, "clutter", "L1") for i in range(30)]
    layouts += [_lay("b", 5121, i, "clutter", "L1") for i in range(30)]
    layouts += [_lay("p", 5000, i, None, "L0", capability=True) for i in range(10)]
    layouts += [_lay("a", 5131, 0, "surfaces", "L1")]  # no result: listed as missing
    _result(tmp_path, "ex", "a", 5100, [0.8] * 4)
    _result(tmp_path, "ex", "b", 5100, [0.05] * 4)  # L0 mean 5 < 10: capability gap
    _result(tmp_path, "ex", "a", 5121, [0.8] * 30)  # rho 1.0
    _result(tmp_path, "ex", "b", 5121, [0.05] * 30)
    _result(tmp_path, "ex", "p", 5000, [0.0] * 10)

    eps, missing = rsi_scorecard.episodes({"layouts": layouts}, tmp_path, "ex")
    assert missing == [("a", 5131, 0)] and len(eps) == 78
    levels = {"levels": {"place": {"clutter": "L1", "surfaces": "L1"}, "pack": {"clutter": "L1"}}}
    j = rsi_scorecard.judge(eps, levels)
    assert j["capability_gaps"] == ["b"]
    (row,) = j["subjects"]
    assert (row["axis"], row["n"], row["rho"]) == ("clutter", 30, 1.0)
    assert j["moves"] == {"clutter/L1": "L2"}
    assert j["levels"]["pack"]["clutter"] == "L2" and j["levels"]["place"]["surfaces"] == "L1"

    card = rsi_scorecard.scorecard(j, 1, "m")
    assert card["subjects"] == [{"axis": "clutter", "level": 1, "n": 30, "score": "80-85", "retention": "1.0-1.1"}]
    assert {c["task"] for c in card["capability"]} == {"b", "p"}
    assert rsi_scorecard.judge(eps, levels) == j  # deterministic
