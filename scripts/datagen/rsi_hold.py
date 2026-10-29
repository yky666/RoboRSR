"""In-process policy for layout checks (--policy_module scripts.datagen.rsi_hold): holds the arms for STEPS actions,
then ends the episode as a fail. A layout the engine finds unstable at start-up, or that a held robot knocks over, is
missing from the run's _result.json details; one that already succeeds is recorded as a success."""

STEPS = 40


def _hold(obs):
    return {k: v for k, v in obs["state"].items() if not k.endswith("_ee_pose") and k != "world_ee_state"}


def eval_one_episode_batch(TASK_ENV, model_client):
    for _ in range(STEPS):
        obs = TASK_ENV.get_obs_batch()
        live = [o["env_idx"] for o in obs if not TASK_ENV.end_flag[o["env_idx"]]]
        if not live:
            break
        TASK_ENV.take_action_batch([_hold(o) for o in obs if o["env_idx"] in live], env_idx_list=live)
    for i in range(TASK_ENV.num_envs):
        if not TASK_ENV.end_flag[i]:
            TASK_ENV.success[i] = False
            TASK_ENV.end_flag[i] = True


def eval_one_episode(TASK_ENV, model_client):
    eval_one_episode_batch(TASK_ENV, model_client)
