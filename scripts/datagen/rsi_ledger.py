"""The RSI run's ledger (rsi/AGENTS.md 5.1 referee/ledger.jsonl): one row per round, and the run report.

    python scripts/datagen/rsi_ledger.py append --ledger referee/ledger.jsonl --round 1 --model M_1 \
        --scores referee/rounds/r1/scores.json --sealed referee/sealed/m1_scores.json \
        --audit referee/rounds/r1/audit.json --training referee/rounds/r1/training.json
    python scripts/datagen/rsi_ledger.py report --ledger referee/ledger.jsonl --start start.json \
        --leaderboard rsi/baseline/openwam_alpha_leaderboard.json [--final final.json] --out report.md

A row keeps what the rounds are compared on: the sealed-probe score (the only number comparable across rounds, since
exams change every round), the exam's subjects and level moves, the capability set, the data accepted into training
and the training cost. Every input is optional except the round and model, so round 0 (M_0: start eval and sealed
probe only) has a row too. A model is "regressed" when its sealed-probe score is below the best earlier one by more
than the two intervals allow (G4).
"""

import argparse
import json
from pathlib import Path
import time


def load(path):
    return json.loads(Path(path).read_text()) if path else None


def row(rnd, model, scores=None, sealed=None, audit=None, training=None, prev_rows=()):
    r = {"round": rnd, "model": model, "time": time.strftime("%Y-%m-%dT%H:%M:%S")}
    if sealed:
        r["sealed"] = {k: sealed[k] for k in ("score", "ci95", "n", "by_kind") if k in sealed}
    if scores:
        r["exam"] = {
            "episodes": scores["episodes"],
            "subjects": [
                {k: s[k] for k in ("axis", "level", "n", "score", "rho", "rho_ci95")} for s in scores["subjects"]
            ],
            "moves": scores["moves"],
            "levels": scores["levels"],
            "capability": scores["capability"],
            "capability_gaps": scores["capability_gaps"],
        }
    if audit:
        r["data"] = {
            k: audit[k] for k in ("accepted", "episodes", "batches", "rejected_batches", "by_task") if k in audit
        }
    if training:
        r["training"] = training
    best = [p for p in prev_rows if "sealed" in p]
    if "sealed" in r and best:
        top = max(best, key=lambda p: p["sealed"]["score"])
        r["regressed"] = r["sealed"]["ci95"][1] < top["sealed"]["ci95"][0]
        r["best_before"] = top["model"]
    return r


def read_ledger(path):
    p = Path(path)
    return [json.loads(line) for line in p.read_text().splitlines() if line.strip()] if p.exists() else []


def fmt_ci(ci, digits=1):
    return f"[{ci[0]:.{digits}f}, {ci[1]:.{digits}f}]"


def cells(*xs):
    return "| " + " | ".join(str(x) for x in xs) + " |"


def report(rows, start=None, leaderboard=None, final=None):
    out = ["# RSI 运行报告", ""]
    out += [
        "## 封存探针（每轮同一组布局，唯一能跨轮比较的成绩）",
        "",
        "| 轮 | 模型 | Score | 95% 区间 | 退步 |",
        "| :-- | :-- | --: | :-- | :-- |",
    ]
    for r in rows:
        if "sealed" in r:
            s = r["sealed"]
            out.append(
                cells(
                    r["round"], r["model"], f"{s['score']:.1f}", fmt_ci(s["ci95"]), "是" if r.get("regressed") else ""
                )
            )
    out += ["", "## 每轮考试（考卷每轮都变，只看本轮的保留率和级别变化）", ""]
    for r in rows:
        if "exam" not in r:
            continue
        e = r["exam"]
        out += [
            f"### 第 {r['round']} 轮（{r['model']}，{e['episodes']} 回合）",
            "",
            "| 维度 | 级别 | n | Score | ρ | ρ 区间 |",
            "| :-- | :-- | --: | --: | --: | :-- |",
        ]
        for s in e["subjects"]:
            rho = f"{s['rho']:.2f}"
            out.append(cells(s["axis"], s["level"], s["n"], f"{s['score']:.1f}", rho, fmt_ci(s["rho_ci95"], 2)))
        cap = ", ".join(f"{t} {c['score']:.1f}（n={c['n']}）" for t, c in e["capability"].items())
        out += ["", f"- 级别变化：{e['moves'] or '无'}", f"- 能力缺口：{cap or '无'}", ""]
    out += ["## 每轮数据", "", "| 轮 | 通过审计 | 回合数 | 作废批次 | 训练 |", "| :-- | :-- | --: | :-- | :-- |"]
    for r in rows:
        if "data" in r or "training" in r:
            d, t = r.get("data", {}), r.get("training", {})
            train = ", ".join(f"{k}={v}" for k, v in t.items() if k in ("steps", "hours", "init", "episodes_total"))
            out.append(
                cells(r["round"], d.get("accepted", ""), d.get("episodes", ""), d.get("rejected_batches", ""), train)
            )
    if start or final:
        out += ["", "## 原 RoboDojo 评测（官方评测集 0，20 个任务 × 布局 0–9）", ""]
        tasks = sorted((start or final)["tasks"])
        head = "| 任务 | 榜单 OpenWAM-α | 起始 M_0 |" + (" 终评 |" if final else "")
        out += [head, "| :-- | --: | --: |" + (" --: |" if final else "")]
        for t in tasks:
            lb = leaderboard["tasks"][t]["score"] if leaderboard else float("nan")
            s0 = start["tasks"][t]["score"] if start else float("nan")
            line = f"| {t} | {lb:.1f} | {s0:.1f} |"
            if final:
                line += f" {final['tasks'][t]['score']:.1f} |"
            out.append(line)
        means = {
            name: sum(d["tasks"][t]["score"] for t in tasks) / len(tasks)
            for name, d in (("起始", start), ("终评", final))
            if d
        }
        out += ["", "- 平均：" + "，".join(f"{k} {v:.2f}" for k, v in means.items())]
    return "\n".join(out) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    a1 = sub.add_parser("append")
    a1.add_argument("--ledger", required=True)
    a1.add_argument("--round", type=int, required=True)
    a1.add_argument("--model", required=True)
    for k in ("scores", "sealed", "audit", "training"):
        a1.add_argument(f"--{k}")
    a2 = sub.add_parser("report")
    a2.add_argument("--ledger", required=True)
    for k in ("start", "leaderboard", "final"):
        a2.add_argument(f"--{k}")
    a2.add_argument("--out", required=True)
    a = ap.parse_args(argv)

    if a.cmd == "append":
        rows = read_ledger(a.ledger)
        if any(r["round"] == a.round and r["model"] == a.model for r in rows):
            raise SystemExit(f"round {a.round} / {a.model} is already in {a.ledger}")
        r = row(a.round, a.model, load(a.scores), load(a.sealed), load(a.audit), load(a.training), rows)
        with open(a.ledger, "a") as f:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
        print(json.dumps(r, ensure_ascii=False)[:400])
    else:
        text = report(read_ledger(a.ledger), load(a.start), load(a.leaderboard), load(a.final))
        Path(a.out).write_text(text)
        print(f"wrote {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
