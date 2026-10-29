from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

import rsi_ledger


def test_rows_regression_and_report():
    sealed0 = {"score": 30.0, "ci95": [25.0, 35.0], "n": 120}
    r0 = rsi_ledger.row(0, "M_0", sealed=sealed0)
    assert "regressed" not in r0
    scores = {
        "episodes": 270,
        "subjects": [{"axis": "clutter", "level": "L1", "n": 30, "score": 40.0, "rho": 1.1, "rho_ci95": [0.6, 1.7]}],
        "moves": {"clutter/L1": "L2"},
        "levels": {"place": {"clutter": "L2"}},
        "capability": {"push_T": {"n": 10, "score": 0.0}},
        "capability_gaps": [],
    }
    r1 = rsi_ledger.row(1, "M_1", scores, {"score": 20.0, "ci95": [15.0, 24.0], "n": 120}, prev_rows=[r0])
    assert r1["regressed"] and r1["best_before"] == "M_0"
    r2 = rsi_ledger.row(2, "M_2", sealed={"score": 33.0, "ci95": [28.0, 38.0], "n": 120}, prev_rows=[r0, r1])
    assert not r2["regressed"]

    start = {"tasks": {"stack_bowls": {"score": 91.5}, "push_T": {"score": 0.0}}}
    final = {"tasks": {"stack_bowls": {"score": 95.0}, "push_T": {"score": 10.0}}}
    lb = {"tasks": {"stack_bowls": {"score": 89.6}, "push_T": {"score": 0.0}}}
    text = rsi_ledger.report([r0, r1, r2], start, lb, final)
    assert "| 1 | M_1 | 20.0 | [15.0, 24.0] | 是 |" in text
    assert "| clutter | L1 | 30 |" in text and "起始 45.75，终评 52.50" in text
