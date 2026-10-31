"""Selection and mass resolution at the external asset boundary (no Isaac)."""

import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location("geniesim", Path(__file__).with_name("geniesim.py"))
G = importlib.util.module_from_spec(spec)
spec.loader.exec_module(G)


def test_tabletop_requires_metric_finite_dimensions_and_rigid_annotation():
    assert G.tabletop({"unit": "m", "size": [0.15, 0.075, 0.15]}, {}) is None
    for parameters in (
        {"unit": "cm", "size": [15, 7.5, 15]},
        {"unit": "m", "size": [float("nan"), 0.1, 0.1]},
        {"unit": "m", "size": [0.1, 0.1]},
        {"unit": "m", "size": [0.8, 0.2, 0.2]},
    ):
        assert G.tabletop(parameters, {}) is not None
    assert G.tabletop({"unit": "m", "size": [0.1, 0.1, 0.1]}, {"has_joint": True}) == "articulated"


def test_authored_mass_wins_and_is_not_silently_capped():
    physics, evidence = G.choose_mass([0.85792065], {"mass": 0.01})
    assert physics == {"mass": 0.85792065, "mass_cap": 0.85792065}
    assert evidence["json_mass_kg"] == 0.01 and not evidence["measured"]
    assert G.choose_mass([], {"mass": 0.05})[0]["mass"] == 0.05
    for masses, parameters in [([0.1, 0.2], {}), ([], {}), ([], {"mass": float("nan")}), ([], {"mass": -1})]:
        with pytest.raises(ValueError):
            G.choose_mass(masses, parameters)


@pytest.mark.parametrize("axis", ["Y", "Z"])
def test_conversion_normalizes_below_rigid_root_and_preserves_mass(tmp_path, axis):
    pytest.importorskip("pxr")
    import json

    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    folder = tmp_path / "source/objects/genie/test"
    folder.mkdir(parents=True)
    (folder / "object_parameters.json").write_text(json.dumps({"mass": 0.01, "upAxis": ["z"]}))
    stage = Usd.Stage.CreateNew(str(folder / "Aligned.usda"))
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    UsdGeom.SetStageUpAxis(stage, axis)
    root = UsdGeom.Xform.Define(stage, "/Object")
    stage.SetDefaultPrim(root.GetPrim())
    root.AddTranslateOp().Set(Gf.Vec3d(100, 50, 10))
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(0.75)
    mesh = UsdGeom.Cube.Define(stage, "/Object/Cube")
    mesh.CreateSizeAttr(10)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    material = UsdPhysics.MaterialAPI.Apply(root.GetPrim())
    material.CreateStaticFrictionAttr(0.5)
    material.CreateDynamicFrictionAttr(0.3)
    stage.GetRootLayer().Save()
    result = G.convert_one(
        tmp_path / "source", {"src": "objects/genie/test", "id": "Rigid/geniesim_test/00000"}, tmp_path / "out", "test"
    )
    output = Usd.Stage.Open(str(Path(result["dir"]) / "object.usdz"))
    assert not UsdGeom.Xformable(output.GetDefaultPrim()).GetOrderedXformOps()
    assert result["mass_kg"] == 0.75
    assert UsdGeom.GetStageUpAxis(output) == UsdGeom.Tokens.z
    assert result["size_m"] == pytest.approx([0.1, 0.1, 0.1])
    assert G.missing_dependencies(Path(result["dir"]) / "object.usdz") == []


def test_placement_metadata_has_box_vertices_and_no_unverified_contact_frame():
    geometry, active = G.placement_metadata([0.1, 0.2, 0.3])
    assert len(geometry["oriented_bbox"]["vertices"]) == 8
    frame = active["place"]["up"]
    assert frame["projection_circle"]["center"] == [0, 0, -0.15, 1, 0, 0, 0]
    assert "contact_circle" not in frame


def test_selection_never_executes_asset_python(tmp_path, monkeypatch):
    import json

    folder = tmp_path / "objects/genie/unsafe"
    folder.mkdir(parents=True)
    (folder / "object_parameters.json").write_text(json.dumps({"unit": "m", "size": [0.1, 0.1, 0.1]}))
    marker = tmp_path / "executed"
    (folder / "item.py").write_text(f"__import__('pathlib').Path({str(marker)!r}).touch()")
    monkeypatch.setattr(G.subprocess, "check_output", lambda *args, **kwargs: "test-revision")
    result = G.select(tmp_path)
    assert not result["chosen"] and len(result["dropped"]) == 1
    assert not marker.exists()
