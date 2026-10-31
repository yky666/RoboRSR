#!/usr/bin/env bash
# CPU checks for every commit, also run by CI (.github/workflows/checks.yml): bash scripts/check.sh [BASE_REF]
# Lints only the Python files changed since BASE_REF (default origin/main; older files predate the formatter), then
# runs the unit tests and the task inventory check. Needs ruff==0.11.7 (as .pre-commit-config.yaml), pytest, numpy,
# scipy, h5py, opencv-python-headless, pyyaml, shapely, rtree, transforms3d and tqdm; the XPolicyLab submodule for the
# trajectory format test.
# GPU checks (scripted demos, collection, replay) run on a server; see scripts/datagen/SKILL.md.
set -euo pipefail
cd "$(dirname "$0")/.."
base="${1:-origin/main}"
git rev-parse -q --verify "$base^{commit}" >/dev/null || base=origin/main
files=$(git diff --name-only --diff-filter=ACMR "$base" -- '*.py' | grep -vE '^(third_party|XPolicyLab|real2sim)/' || true)
if [ -n "$files" ]; then
    ruff check $files
    ruff format --check $files
fi
git diff --check "$base"
python -m pytest -q scripts/datagen scripts/scenes scripts/objects/test_manitwin.py scripts/objects/test_gso.py scripts/objects/test_geniesim.py scripts/objects/test_object_names.py scripts/objects/test_deformable.py scripts/prototypes/test_genie_bridge.py scripts/g1d/test_office_beans.py scripts/g1d/test_g1d_body.py scripts/g1d/test_randomize.py \
    src/real2sim_platform/test_server.py src/real2sim_platform/test_deformables.py scripts/g1d/test_aurora_integration.py scripts/g1d/xense_gripper
python scripts/internal/task_inventory.py --format json --check >/dev/null
if command -v node >/dev/null; then node src/real2sim_platform/test_dataset_catalog.cjs >/dev/null; fi  # page-side dataset catalog
echo "checks passed"
