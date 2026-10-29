from env.environment.task_env import TaskEnv
from env.reward_manager.reward_manager import RewardManager


class TwistBottleCapG1dCommon:
    """Unscrew the bottle cap: cap_twist (metadata passive.functional.cap) past OPEN of its two-turn range, with the
    bottle still standing on the table (held, not lifted)."""

    OPEN = 0.9
    LIFT_TOL, TILT_TOL = 0.02, 20  # m, deg: also rejects a bottle flung by a physics blow-up

    def __init__(self, config, app, **kwargs):
        super().__init__(config, app, **kwargs)
        self.reward_manager = RewardManager(self.num_envs)
        self.step_lim = 9000  # 300 s: up to twenty 45-degree strokes at ~150 deg/s, each regripped from above

    def _post_setup_scene(self, sim):
        super()._post_setup_scene(sim)
        self.reward_manager.initialize(self)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed, options=options)
        self.reward_manager.reset()

    def run_reward(self):
        rm = self.reward_manager
        rm.check(
            [
                rm.is_joint_position_above_ratio(label="bottle", percentage=self.OPEN, tag="cap"),
                rm.is_not_lift("bottle", z_threshold=self.LIFT_TOL),
                rm.is_axis_up("bottle", axis=[0, 0, 1], threshold=self.TILT_TOL),
            ]
        )

    def gen_instruction(self, env_idx):
        return ["Unscrew the cap of the water bottle."]


class g1d_twist_bottle_cap(TwistBottleCapG1dCommon, TaskEnv):
    """G1-D unscrews the cap of the fixed Nongfu Spring bottle."""
