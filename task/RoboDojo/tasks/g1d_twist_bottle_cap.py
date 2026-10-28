from env.environment.task_env import TaskEnv
from env.reward_manager.reward_manager import RewardManager


class TwistBottleCapG1dCommon:
    """Unscrew the bottle cap: cap_twist (metadata passive.functional.cap) past OPEN of its two-turn range."""

    OPEN = 0.9

    def __init__(self, config, app, **kwargs):
        super().__init__(config, app, **kwargs)
        self.reward_manager = RewardManager(self.num_envs)
        self.step_lim = 2400  # 80 s: up to sixteen 60-degree strokes with regrasps

    def _post_setup_scene(self, sim):
        super()._post_setup_scene(sim)
        self.reward_manager.initialize(self)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed, options=options)
        self.reward_manager.reset()

    def run_reward(self):
        rm = self.reward_manager
        rm.check([rm.is_joint_position_above_ratio(label="bottle", percentage=self.OPEN, tag="cap")])

    def gen_instruction(self, env_idx):
        return ["Unscrew the cap of the water bottle."]


class g1d_twist_bottle_cap(TwistBottleCapG1dCommon, TaskEnv):
    """G1-D unscrews the cap of the fixed Nongfu Spring bottle."""
