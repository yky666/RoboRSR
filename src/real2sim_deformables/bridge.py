"""Opt-in lifecycle for a RoboRSR host stage and separate Newton process.

The caller advances existing robot physics, supplies kinematic proxy poses, calls
step with the matching time interval, then renders. No reaction is written back
to a PhysX robot. This facade does not change the production scene factory.
"""

import numpy as np

from real2sim_deformables.client import Client
from real2sim_deformables.usd_sink import UsdMeshSink


class GenieDeformableBridge:
    def __init__(self, stage, python, config, genie_root, log, device="cpu", root="/World/NewtonVisuals"):
        self.sink = UsdMeshSink(stage, root)
        self.client = Client(python, config, genie_root, log, device)
        try:
            self.state = self.client.request("state")
            self.sink.update(self.state)
        except BaseException:
            self.client.close(force=True)
            raise

    def step(self, dt, proxies=None):
        if not np.isclose(dt, self.state["frame_dt_s"], atol=1e-12, rtol=0):
            raise ValueError("Host timestep differs from Newton frame interval")
        for name, command in (proxies or {}).items():
            self.client.request("kinematic", name=name, **command)
        self.state = self.client.request("step")
        self.sink.update(self.state)
        return self.state

    def reset(self):
        self.state = self.client.request("reset")
        self.sink.update(self.state)
        return self.state

    def close(self):
        self.client.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
