"""Host-side process bridge; needs neither Warp nor Newton in Isaac's Python."""

import json
import os
from pathlib import Path
import selectors
import subprocess


class Client:
    def __init__(self, python, config, genie_root, log, device="cpu", timeout=120):
        self.timeout, self.sequence = timeout, 0
        self.log = Path(log).open("w")
        env = dict(os.environ)
        env["PYTHONPATH"] = str(Path(__file__).resolve().parents[1])
        self.process = subprocess.Popen(
            [
                str(python),
                "-m",
                "real2sim_deformables.worker",
                "--config",
                str(config),
                "--genie-root",
                str(genie_root),
                "--device",
                device,
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=self.log,
            text=True,
            bufsize=1,
            env=env,
        )
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.process.stdout, selectors.EVENT_READ)

    def request(self, op, **kwargs):
        if self.process.poll() is not None:
            raise RuntimeError("Newton worker exited; inspect worker log")
        self.sequence += 1
        self.process.stdin.write(json.dumps(dict(id=self.sequence, op=op, **kwargs), allow_nan=False) + "\n")
        self.process.stdin.flush()
        if not self.selector.select(self.timeout):
            self.close(force=True)
            raise TimeoutError("Newton worker timed out and was stopped")
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError("Newton worker closed its reply stream; inspect worker log")
        response = json.loads(line)
        if response.get("id") != self.sequence:
            raise RuntimeError("Out-of-order worker response")
        if "error" in response:
            raise RuntimeError(response["error_type"] + ": " + response["error"])
        return response["result"]

    def close(self, force=False):
        if self.process.poll() is None:
            if not force:
                try:
                    self.request("close")
                except (RuntimeError, BrokenPipeError):
                    pass
            if self.process.poll() is None:
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
        self.selector.close()
        self.log.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
