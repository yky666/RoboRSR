from env.environment.task_env import TaskEnv
from task.RoboDojo.tasks.g1d_deliver_cup_to_lounge import DeliverCupToLoungeG1dCommon

POURED = 0.2  # at least a fifth of the ~300 ml leaves the jug ...
IN_CUP = 0.9  # ... and 90% of what left it is still in the cup on the lounge table


class PourWaterDeliverToLoungeG1dCommon(DeliverCupToLoungeG1dCommon):
    """G1-D mobile manipulation in the counter scene (real_scene2): one hand pours water from the fridge jug into the
    cup the other hand holds up at the counter (as g1d_pour_water_into_cup), the jug goes back down, then the cup is
    driven to the lounge's round table and set down there. Layouts: scripts/g1d/build_asset.py (pour_deliver_layout)."""

    def __init__(self, config, app, **kwargs):
        super().__init__(config, app, **kwargs)
        self.step_lim = 2100  # pour ~400 actions, set the jug down ~250, deliver ~1000, plus slack

    def pour_check(self):
        return self.reward_manager.is_A_fluid_in_B(
            "water",
            "cup",
            percentage_threshold=IN_CUP,
            label_C="bottle",
            B_buffer=0.0,
            C_buffer=0.0,
            B_z_threshold=0.001,
            C_residual_threshold=1 - POURED,
            # splashes break into many 3 mm droplets: count them as spilled (the bean checks drop small clusters, and
            # with water these alone pass their 2% cap and fail the pour)
            ignore_scattered=False,
        )

    def run_reward(self):
        self.reward_manager.check([*self.delivered_checks(), self.pour_check()])

    def gen_instruction(self, env_idx):
        return ["Pour some water from the jug into the cup at the counter, then take the cup to the round table."]


class g1d_pour_water_deliver_to_lounge(PourWaterDeliverToLoungeG1dCommon, TaskEnv):
    """Mobile G1-D (g1d_counter_mobile): pour from the jug at the counter, carry the cup to the lounge table."""
