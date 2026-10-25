"""One-way G1-WS SimpleSensor + FemSensor worker for actual simulation records.

Use the vendor CPython 3.10 runtime on PYTHONPATH in an isolated environment.
No createSensor, enumeration, device runtime, real camera or DDS API is called.
Vendor files remain external and are checked against the frozen source hashes.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from env.robot_manager.aurora_contract import CHANNELS  # noqa: E402
from env.robot_manager.tactile_geometry import depth_from_scene, fem_world_matrix, object_matrix  # noqa: E402

EXPECTED = {
    "g1-ws_table.npz": "2cd8f3b989f70d56965cdf70325f0ab2285986af8c72229253661f61a3948b8b",
    "g1-ws.npz": "3fd053c7046999bce2bbdfc21f9d0fbe09071e3a3d6246c4f682097c469b5b51",
}


def input_frames(directory, follow=False, idle_timeout=120):
    """Consume only complete JSONL lines; a completion receipt ends a live stream."""
    path = directory / "frames.jsonl"
    if not follow:
        yield from (json.loads(line) for line in path.read_text().splitlines())
        return
    position, last_progress, expected_id = 0, time.monotonic(), 0
    while True:
        if path.exists():
            with path.open() as stream:
                stream.seek(position)
                line = stream.readline()
                if line.endswith("\n"):
                    frame = json.loads(line)
                    if frame["frame_id"] != expected_id:
                        raise ValueError("Non-contiguous producer frame ID")
                    position = stream.tell()
                    expected_id += 1
                    last_progress = time.monotonic()
                    yield frame
                    continue
        done = directory / "recording_complete.json"
        if done.exists():
            if json.loads(done.read_text())["frames"] != expected_id:
                raise ValueError("Incomplete producer stream")
            return
        if time.monotonic() - last_progress > idle_timeout:
            raise TimeoutError("Physics producer stopped without completion receipt")
        time.sleep(0.02)


def host_array(value):
    return np.asarray(value.get() if hasattr(value, "get") else value)


def sdk_matrix(Matrix4x4, value):
    # Vendor Matrix4x4 uses the same row/column transform layout in its constructor;
    # verify round-trip instead of silently transposing an untested pose.
    result = Matrix4x4(np.asarray(value, dtype=np.float32))
    returned = np.asarray(result)
    if returned.shape == (16,):
        returned = returned.reshape(4, 4, order="F")  # Qt/QMatrix4x4 column-major data
    if returned.shape != (4, 4) or not np.allclose(returned, value, atol=1e-6):
        raise ValueError("Vendor Matrix4x4 layout mismatch")
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--recording", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--fem-iterations", type=int, default=10)
    p.add_argument("--follow", action="store_true", help="Follow a running simulator's append-only JSONL")
    args = p.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    (args.out / "packets").mkdir()
    from PySide6.QtWidgets import QApplication
    from xensim import PROJ_DIR
    from xensim.core import FemSensor, Matrix4x4, SimpleSensor

    app = QApplication.instance() or QApplication([])
    files = {name: PROJ_DIR / "assets/fem" / name for name in EXPECTED}
    for name, path in files.items():
        if hashlib.sha256(path.read_bytes()).hexdigest() != EXPECTED[name]:
            raise ValueError(f"Vendor resource hash mismatch: {name}")
    scene = json.loads((args.recording / "scene_schema.json").read_text())
    frames = input_frames(args.recording, args.follow)
    contract = dict(
        schema="g1d_four_pad_tactile.v1",
        channels=CHANNELS,
        images="RGB uint8 [channel,height,width,3]",
        depth="mm; negative intrusion; [4,175,100]",
        image_row0="negative contact-frame Y; optical hardware alignment unverified",
        active_area_mm=[17.3, 29.14],
        cad_pad_rescaled=False,
        force="FEM node array, native vendor coordinates/units; not measured force calibration",
        physics_force="PhysX contact normal and friction vectors in world coordinates, N",
        physics_owner="Isaac/PhysX only; FEM force is NOT applied to objects",
        fem_iterations_per_observation=args.fem_iterations,
        no_contact_pose_policy=(
            "hold relative pose on release; positive clearance frees contacts; no distant-object shear drive"
        ),
        fem_depth="actual ray surface distance: mm, negative intrusion, positive separation; miss=+5mm",
        vendor_resources=EXPECTED,
        external_vendor_resources=True,
        hardware_connected=False,
        inputs_from_actual_recorded_poses=True,
        real_sensor_calibration_validated=False,
        output_mode="live ordered stream" if args.follow else "offline frame-associated observations",
    )
    (args.out / "tactile_contract.json").write_text(json.dumps(contract, indent=2) + "\n")
    simple = {ch: SimpleSensor(calibrate_file=files["g1-ws_table.npz"], visible=False) for ch in CHANNELS}
    fem = {
        ch: FemSensor(
            calibrate_file=files["g1-ws_table.npz"], fem_file=files["g1-ws.npz"], depth_size=(100, 175), visible=False
        )
        for ch in CHANNELS
    }
    rows, baselines, last_relative = [], {}, {}
    for frame in frames:
        started = time.monotonic()
        payload = dict(
            frame_id=np.array(frame["frame_id"], dtype=np.int64),
            simulation_time_s=np.array(frame["simulation_time_s"], dtype=np.float64),
        )
        arrays = {
            k: []
            for k in (
                "depth_mm",
                "fem_surface_clearance_mm",
                "simple_rgb",
                "fem_rgb",
                "fem_node_force",
                "fem_marker",
                "native_normal_force_w_n",
                "native_friction_force_w_n",
            )
        }
        channel_rows = {}
        for ch in CHANNELS:
            pad = frame["pad_world"][ch]
            depth = depth_from_scene(pad, frame["objects"], scene["objects"])
            clearance = depth_from_scene(pad, frame["objects"], scene["objects"], include_clearance=True)
            if not np.isfinite(depth).all() or depth.min() < -1.0:
                raise ValueError(f"Frame {frame['frame_id']}/{ch}: outside tested 1 mm indentation domain")
            # Select the closest recorded fixture in the pad frame for FEM's
            # relative tangential-pose tracking. All depth still comes from rays.
            tw = fem_world_matrix(pad)
            nearest = min(
                frame["objects"],
                key=lambda n: np.linalg.norm(np.asarray(frame["objects"][n]["pose_wxyz"][:3]) - tw[:3, 3]),
                default=None,
            )
            obj = object_matrix(frame["objects"][nearest]["pose_wxyz"]) if nearest else tw.copy()
            if np.any(depth < 0):
                last_relative[ch] = np.linalg.inv(tw) @ obj
            else:
                obj = tw @ last_relative.get(ch, np.eye(4))
            simple[ch].step(depth, smooth_norm=2, smooth_mesh=2)
            fem[ch].step(sdk_matrix(Matrix4x4, obj), sdk_matrix(Matrix4x4, tw), clearance, nstep=args.fem_iterations)
            simple[ch].update()
            fem[ch].update()
            app.processEvents()
            image = host_array(fem[ch].get_image())
            force, marker = host_array(fem[ch].get_force()), host_array(fem[ch].get_marker())
            if ch not in baselines:
                baselines[ch] = marker.copy()
            for k, value in dict(
                depth_mm=depth,
                fem_surface_clearance_mm=clearance,
                simple_rgb=host_array(simple[ch].get_image()),
                fem_rgb=image,
                fem_node_force=force,
                fem_marker=marker,
                native_normal_force_w_n=np.array(frame["native_contact_normal_force_w_n"][ch]),
                native_friction_force_w_n=np.array(frame["native_friction_force_w_n"][ch]),
            ).items():
                if not np.isfinite(value).all():
                    raise ValueError(f"Non-finite output {k} at {frame['frame_id']}/{ch}")
                arrays[k].append(value)
            channel_rows[ch] = dict(
                min_depth_mm=float(depth.min()),
                contact_pixels=int(np.count_nonzero(depth < 0)),
                fem_node_force_array_norm=float(np.linalg.norm(force)),
                fem_node_force_vector_sum=force.sum(axis=0).tolist(),
                marker_displacement_norm=float(np.linalg.norm(marker - baselines[ch])),
                nearest_fixture=nearest,
            )
        payload.update({k: np.stack(v) for k, v in arrays.items()})
        destination = args.out / "packets" / f"{frame['frame_id']:05d}.npz"
        temporary = destination.with_suffix(".tmp")
        with temporary.open("wb") as stream:
            np.savez_compressed(stream, **payload)
        os.replace(temporary, destination)
        row = dict(
            frame_id=frame["frame_id"],
            simulation_time_s=frame["simulation_time_s"],
            phase=frame["phase"],
            channels=channel_rows,
            processing_seconds=time.monotonic() - started,
        )
        rows.append(row)
        with (args.out / "metrics.jsonl").open("a") as stream:
            stream.write(json.dumps(row) + "\n")
        if frame["frame_id"] % 10 == 0:
            print(
                json.dumps(dict(frame=frame["frame_id"], phase=frame["phase"], seconds=row["processing_seconds"])),
                flush=True,
            )
    report = dict(
        status="completed",
        frames=len(rows),
        channels=list(CHANNELS),
        vendor_runtime_executed=True,
        inputs_from_actual_recorded_poses=True,
        all_outputs_finite=True,
        force_feedback_to_physics=False,
        hardware_connected=False,
        calibration_validated=False,
        max_depth_mm={ch: -min(r["channels"][ch]["min_depth_mm"] for r in rows) for ch in CHANNELS},
        source_frames_sha256=hashlib.sha256((args.recording / "frames.jsonl").read_bytes()).hexdigest(),
    )
    (args.out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
