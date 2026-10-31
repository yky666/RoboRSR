"""JSON-lines worker. stdout is reserved for replies; physics logs go to stderr."""

import argparse
import contextlib
import json
import sys
import traceback


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", required=True)
    p.add_argument("--genie-root", required=True)
    p.add_argument("--device", default="cpu")
    args = p.parse_args()
    wire = sys.stdout
    with contextlib.redirect_stdout(sys.stderr):
        from real2sim_deformables.runtime import Runtime

        with open(args.config) as f:
            config = json.load(f)
        runtime = Runtime(config, args.genie_root, args.device)
    for line in sys.stdin:
        request = {}
        try:
            request = json.loads(line)
            with contextlib.redirect_stdout(sys.stderr):
                op = request["op"]
                if op == "state":
                    result = runtime.snapshot()
                elif op == "step":
                    result = runtime.step(request.get("frames", 1))
                elif op == "reset":
                    result = runtime.reset()
                elif op == "kinematic":
                    runtime.set_kinematic(request["name"], request["pose_xyzw"], request["velocity"])
                    result = runtime.snapshot()
                elif op == "close":
                    result = {"closed": True}
                else:
                    raise ValueError("Unknown operation")
            response = {"id": request["id"], "result": result}
        except Exception as exc:
            traceback.print_exc(file=sys.stderr)
            response = {"id": request.get("id"), "error": str(exc), "error_type": type(exc).__name__}
        wire.write(json.dumps(response, allow_nan=False) + "\n")
        wire.flush()
        if request.get("op") == "close":
            break


if __name__ == "__main__":
    main()
