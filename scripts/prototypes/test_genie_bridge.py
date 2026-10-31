"""Failure guards for an optional external-physics visual bridge."""

from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import pytest

pytest.importorskip("pxr")
pytest.importorskip("newton")
from pxr import Usd, UsdGeom, UsdPhysics

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from real2sim_deformables.runtime import CLOTH_PATH, load_upstream
from real2sim_deformables.usd_sink import UsdMeshSink


class BridgeGuards(unittest.TestCase):
    def stage(self):
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(stage, 1)
        UsdGeom.SetStageUpAxis(stage, "Z")
        return stage

    def frame(self):
        return {
            "epoch": 0,
            "frame": 0,
            "objects": {"cloth": {"points_m": [[0, 0, 0], [1, 0, 0], [0, 1, 0]], "faces": [[0, 1, 2]]}},
        }

    def test_modified_upstream_refused(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / CLOTH_PATH
            p.parent.mkdir(parents=True)
            p.write_text('raise Exception("must not execute")')
            with self.assertRaisesRegex(ValueError, "pinned"):
                load_upstream(d)

    def test_units_and_axes_are_explicit(self):
        stage = self.stage()
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)
        with self.assertRaisesRegex(ValueError, "meters"):
            UsdMeshSink(stage)
        UsdGeom.SetStageMetersPerUnit(stage, 1)
        UsdGeom.SetStageUpAxis(stage, "Y")
        with self.assertRaisesRegex(ValueError, "Z up"):
            UsdMeshSink(stage)

    def test_double_physics_rejected(self):
        stage = self.stage()
        parent = UsdGeom.Xform.Define(stage, "/World")
        UsdPhysics.RigidBodyAPI.Apply(parent.GetPrim())
        sink = UsdMeshSink(stage)
        with self.assertRaisesRegex(ValueError, "PhysX"):
            sink.update(self.frame())
        self.assertFalse(stage.GetPrimAtPath("/World/NewtonVisuals/cloth"))

    def test_malformed_frame_does_not_partially_write(self):
        stage = self.stage()
        sink = UsdMeshSink(stage)
        state = self.frame()
        state["objects"]["second"] = {"points_m": [[float("nan"), 0, 0]], "faces": [[0, 0, 0]]}
        with self.assertRaises(ValueError):
            sink.update(state)
        self.assertFalse(stage.GetPrimAtPath("/World/NewtonVisuals/cloth"))

    def test_topology_and_stale_rejected(self):
        stage = self.stage()
        sink = UsdMeshSink(stage)
        state = self.frame()
        state["frame"] = 3
        sink.update(state)
        with self.assertRaisesRegex(ValueError, "Stale"):
            sink.update(self.frame())
        state["objects"]["cloth"]["faces"] = [[2, 1, 0]]
        with self.assertRaisesRegex(ValueError, "Topology"):
            sink.update(state)
        np.testing.assert_array_equal(
            UsdGeom.Mesh(stage.GetPrimAtPath("/World/NewtonVisuals/cloth")).GetFaceVertexIndicesAttr().Get(), [0, 1, 2]
        )


if __name__ == "__main__":
    unittest.main()
