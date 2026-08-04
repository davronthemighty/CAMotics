#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

for exe in \
  camsim \
  camsim-path \
  camsim-region-plan \
  camsim-boundary-plan \
  camsim-render-regions \
  camsim-stitch-stock \
  camsim-reduce-export
do
  if [ ! -x "./$exe" ]; then
    echo "$exe executable not found." >&2
    echo "Build sparse split tools first, for example:" >&2
    echo "  scons -j10 $exe with_gui=0 with_tpl=0" >&2
    exit 2
  fi
done

tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

expect_failure() {
  local label="$1"
  shift
  if "$@" >/dev/null 2>&1; then
    echo "$label was unexpectedly accepted" >&2
    exit 1
  fi
}

input="${1:-examples/heart/heart.camotics}"

./camsim-path --threads 4 --resolution 1 \
  "$input" "$tmpdir/toolpath.json"
./camsim-region-plan \
  --xy-bins 8 \
  --target-region-cells 2000 \
  "$tmpdir/toolpath.json" "$tmpdir/region-plan.json"
if ./camsim-region-plan \
  --xy-bins 4097 \
  "$tmpdir/toolpath.json" "$tmpdir/oversized-region-plan.json" \
  >"$tmpdir/oversized-region-plan.log" 2>&1
then
  echo "oversized sparse region plan was unexpectedly accepted" >&2
  exit 1
fi
grep -q "planning memory limit" "$tmpdir/oversized-region-plan.log"
./camsim-boundary-plan \
  "$tmpdir/region-plan.json" \
  "$tmpdir/ownership-boundary.json"
./camsim-render-regions --threads 4 \
  "$tmpdir/toolpath.json" \
  "$tmpdir/region-plan.json" \
  "$tmpdir/region-surface.json"
./camsim-stitch-stock \
  --ownership-boundary "$tmpdir/ownership-boundary.json" \
  "$tmpdir/region-plan.json" \
  "$tmpdir/region-surface.json" \
  "$tmpdir/stitched-surface.json"
./camsim-reduce-export \
  "$tmpdir/stitched-surface.json" \
  "$tmpdir/output.stl"

python3 - "$tmpdir/toolpath.json" "$tmpdir/region-plan.json" \
  "$tmpdir/region-surface.json" "$tmpdir/stitched-surface.json" \
  "$tmpdir" <<'PY'
import copy
import json
import pathlib
import sys

toolpath_file, region_plan_file, region_surface_file, stitched_file, \
    output_dir = sys.argv[1:]
output_dir = pathlib.Path(output_dir)

def load(filename):
    with open(filename, encoding="utf-8") as f:
        return json.load(f)

def write(name, artifact):
    with open(output_dir / name, "w", encoding="utf-8") as f:
        json.dump(artifact, f)

toolpath = load(toolpath_file)
mutated = copy.deepcopy(toolpath)
mutated["artifact-kind"] = "wrong sparse artifact kind"
write("wrong-kind.json", mutated)

mutated = copy.deepcopy(toolpath)
mutated["artifact-version"] = "999"
write("wrong-version.json", mutated)

mutated = copy.deepcopy(toolpath)
mutated["simulation"]["resolution"] += 0.5
write("wrong-resolution.json", mutated)

mutated = copy.deepcopy(toolpath)
mutated["simulation"]["workpiece"]["min"][0] += 1
write("wrong-bounds.json", mutated)

mutated = copy.deepcopy(toolpath)
current_mode = mutated["simulation"]["render-mode"]
mutated["simulation"]["render-mode"] = (
    "CMS_MODE" if current_mode != "CMS_MODE" else "MCUBES_MODE"
)
write("wrong-render-mode.json", mutated)

mutated = copy.deepcopy(toolpath)
mutated["contract"]["toolpath-hash"] = "mutated-toolpath-hash"
write("wrong-toolpath-hash.json", mutated)

region_plan = load(region_plan_file)
mutated = copy.deepcopy(region_plan)
mutated["contract"]["input-hash"] = "mutated-input-hash"
write("wrong-input-hash.json", mutated)

mutated = copy.deepcopy(region_plan)
mutated["contract"]["region-plan-hash"] = "mutated-region-plan-hash"
write("wrong-region-plan-hash.json", mutated)

region_surface = load(region_surface_file)
mutated = copy.deepcopy(region_surface)
mutated["contract"]["region-plan-hash"] = "mutated-surface-plan-hash"
write("wrong-surface-plan-hash.json", mutated)

stitched = load(stitched_file)
eligibility = stitched["reduction-eligibility"]

mutated = copy.deepcopy(stitched)
del mutated["reduction-eligibility"]
write("missing-reduction-eligibility.json", mutated)

mutated = copy.deepcopy(stitched)
mutated["reduction-eligibility"]["triangle-origins"].pop()
write("truncated-triangle-origins.json", mutated)

mutated = copy.deepcopy(stitched)
origins = mutated["reduction-eligibility"]["triangle-origins"]
first = next(i for i, origin in enumerate(origins) if origin == 0)
second = next(i for i, origin in enumerate(origins) if origin != origins[first])
origins[first], origins[second] = origins[second], origins[first]
write("reordered-triangle-origins.json", mutated)

mutated = copy.deepcopy(stitched)
origins = mutated["reduction-eligibility"]["triangle-origins"]
analytic = next(i for i, origin in enumerate(origins) if origin == 2)
origins[analytic] = 0
write("relabelled-analytic-origin.json", mutated)

mutated = copy.deepcopy(stitched)
mutated["reduction-eligibility"]["triangle-count"] += 1
write("wrong-eligibility-triangle-count.json", mutated)

mutated = copy.deepcopy(stitched)
mutated["reduction-eligibility"]["locked-seam-vertices"][0]["x"] += 1
write("mutated-locked-seam-vertex.json", mutated)

mutated = copy.deepcopy(stitched)
mutated["reduction-eligibility"]["locked-seam-edges"][0]["a"]["x"] += 1
write("mutated-locked-seam-edge.json", mutated)

mutated = copy.deepcopy(stitched)
mutated["reduction-eligibility"]["binding-hash"] = "mutated-binding-hash"
write("wrong-eligibility-binding-hash.json", mutated)

mutated = copy.deepcopy(stitched)
vertices = mutated["simulation"]["surface"]["vertices"]
vertices[:9], vertices[9:18] = vertices[9:18], vertices[:9]
write("reordered-stitched-surface.json", mutated)
PY

for artifact in wrong-kind wrong-version wrong-resolution wrong-bounds \
  wrong-render-mode wrong-toolpath-hash
do
  expect_failure "$artifact toolpath artifact" \
    ./camsim-region-plan "$tmpdir/$artifact.json" \
    "$tmpdir/$artifact-region-plan.json"
done

expect_failure "mutated input hash" \
  ./camsim-render-regions "$tmpdir/toolpath.json" \
  "$tmpdir/wrong-input-hash.json" "$tmpdir/wrong-input-surface.json"
expect_failure "mutated region plan hash" \
  ./camsim-boundary-plan "$tmpdir/wrong-region-plan-hash.json" \
  "$tmpdir/wrong-region-plan-boundary.json"
expect_failure "mutated region surface lineage" \
  ./camsim-stitch-stock "$tmpdir/region-plan.json" \
  "$tmpdir/wrong-surface-plan-hash.json" \
  "$tmpdir/wrong-surface-stitched.json"

for artifact in missing-reduction-eligibility truncated-triangle-origins \
  reordered-triangle-origins relabelled-analytic-origin \
  wrong-eligibility-triangle-count mutated-locked-seam-vertex \
  mutated-locked-seam-edge wrong-eligibility-binding-hash \
  reordered-stitched-surface
do
  expect_failure "$artifact stitched eligibility" \
    ./camsim-reduce-export --safe-reduce \
    "$tmpdir/$artifact.json" "$tmpdir/$artifact.stl"
done

declare -A split_safe_reduce_args
split_safe_reduce_args[apply]='--safe-reduce'
split_safe_reduce_args[report]='--safe-reduce-report'
split_safe_reduce_args[provenance]='--safe-reduce --safe-reduce-provenance-neighbors'
split_safe_reduce_args[trusted]='--safe-reduce --safe-reduce-trust-provenance-neighbors'
split_safe_reduce_args[hole]='--safe-reduce --safe-reduce-hole-aware'
split_safe_reduce_args[cosimplify]='--safe-reduce --safe-reduce-boundary-cosimplify'

for mode in apply report provenance trusted hole cosimplify; do
  read -r -a extra <<<"${split_safe_reduce_args[$mode]}"
  ./camsim-reduce-export "${extra[@]}" \
    --profile "$tmpdir/split-safe-$mode.json" \
    "$tmpdir/stitched-surface.json" "$tmpdir/split-safe-$mode.stl"
done

python3 - "$tmpdir" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
zero = (
    "safe_reduce_analyzed_analytic_triangles",
    "safe_reduce_analytic_adjacency_insertions",
    "safe_reduce_analytic_component_memberships",
    "safe_reduce_analytic_replacement_triangles",
    "safe_reduce_unknown_locked_triangles",
)
for mode in ("apply", "report", "provenance", "trusted", "hole",
             "cosimplify"):
    metrics = json.load(open(root / f"split-safe-{mode}.json",
                             encoding="utf-8"))["metrics"]
    if metrics["safe_reduce_origin_metadata_valid"] != 1:
        raise SystemExit(mode + ": split eligibility metadata was not valid")
    if any(metrics[key] != 0 for key in zero):
        raise SystemExit(mode + ": locked triangles entered split analysis")
    for key in ("safe_reduce_analytic_identity_preserved",
                "safe_reduce_locked_seams_preserved",
                "safe_reduce_whole_surface_validation_accepted"):
        if metrics[key] != 1:
            raise SystemExit(f"{mode}: split validation failed: {key}")
    if mode == "report":
        if metrics["safe_reduce_output_triangles"] != metrics[
                "safe_reduce_input_triangles"]:
            raise SystemExit("split report mode changed the surface")
    elif metrics["safe_reduce_applied_output_triangles"] >= metrics[
            "safe_reduce_applied_source_triangles"]:
        raise SystemExit(mode + ": split eligible MC was not reduced")
PY

python3 scripts/perf/compare_stl_distance.py \
  --max-samples 500 --cell-size 2 --max-shells 20 \
  --hard-max-error 0.01 --p99-error 0.01 \
  "$tmpdir/output.stl" "$tmpdir/split-safe-apply.stl"

grep -q '"artifact-kind": "CAMotics sparse toolpath JSON"' \
  "$tmpdir/toolpath.json"
grep -q '"artifact-kind": "CAMotics sparse region plan JSON"' \
  "$tmpdir/region-plan.json"
grep -q '"artifact-kind": "CAMotics sparse ownership boundary JSON"' \
  "$tmpdir/ownership-boundary.json"
grep -q '"artifact-kind": "CAMotics sparse region surface artifact"' \
  "$tmpdir/region-surface.json"
grep -q '"artifact-kind": "CAMotics sparse stitched surface artifact"' \
  "$tmpdir/stitched-surface.json"
grep -q '"planner": "adaptive-xy-depth-halo"' "$tmpdir/region-plan.json"
grep -q '"ownership": "mc-active-analytic-untouched"' \
  "$tmpdir/region-plan.json"
grep -q '"active-region-list":' "$tmpdir/region-plan.json"
grep -q '"render-region-list":' "$tmpdir/region-plan.json"
grep -q '"analytic-region-list":' "$tmpdir/region-plan.json"
grep -q '"render-cells-est":' "$tmpdir/region-plan.json"
grep -q '"target-region-cells": 2000' "$tmpdir/region-plan.json"
grep -Eq '"adaptive-leaf-count": [1-9]' "$tmpdir/region-plan.json"
grep -Eq '"adaptive-active-leaf-count": [1-9]' "$tmpdir/region-plan.json"
grep -Eq '"adaptive-split-count": [1-9]' "$tmpdir/region-plan.json"
grep -q '"adaptive-ownership-split-count":' "$tmpdir/region-plan.json"
grep -q '"adaptive-depth-split-count":' "$tmpdir/region-plan.json"
grep -q '"adaptive-density-split-count":' "$tmpdir/region-plan.json"
grep -q '"adaptive-target-split-count":' "$tmpdir/region-plan.json"
grep -Eq '"adaptive-max-leaf-cells": [1-9]' "$tmpdir/region-plan.json"
grep -q '"adaptive-target-exceeded-leaves":' "$tmpdir/region-plan.json"
grep -q '"planner": "grid-top-ownership-boundary-v1"' \
  "$tmpdir/ownership-boundary.json"
grep -q '"loop-count":' "$tmpdir/ownership-boundary.json"
grep -q '"boundary-edges":' "$tmpdir/ownership-boundary.json"
grep -q '"loops":' "$tmpdir/ownership-boundary.json"
grep -q '"renderer": "planned-active-region-render"' \
  "$tmpdir/region-surface.json"
grep -Eq '"ownership-active-regions": [1-9]' "$tmpdir/region-surface.json"
grep -Eq '"cells-visited": [1-9]' "$tmpdir/region-surface.json"
grep -q '"cells-culled":' "$tmpdir/region-surface.json"
grep -Eq '"cells-contoured": [1-9]' "$tmpdir/region-surface.json"
grep -Eq '"vertex-samples": [1-9]' "$tmpdir/region-surface.json"
grep -Eq '"depth-calls": [1-9]' "$tmpdir/region-surface.json"
grep -Eq '"toolsweep-depth-calls": [1-9]' "$tmpdir/region-surface.json"
grep -Eq '"edge-checks": [1-9]' "$tmpdir/region-surface.json"
grep -Eq '"edge-intersections": [1-9]' "$tmpdir/region-surface.json"
grep -q '"stitcher": "analytic-stock-patches-v1"' \
  "$tmpdir/stitched-surface.json"
grep -q '"ownership-boundary-supplied": 1' "$tmpdir/stitched-surface.json"
grep -q '"ownership-boundary-planner": "grid-top-ownership-boundary-v1"' \
  "$tmpdir/stitched-surface.json"
grep -q '"ownership-boundary-loops":' "$tmpdir/stitched-surface.json"
grep -q '"analytic-top-source": "ownership-boundary-loops"' \
  "$tmpdir/stitched-surface.json"
grep -Eq '"analytic-triangles": [1-9]' "$tmpdir/stitched-surface.json"
grep -q '"analytic-top-patches":' "$tmpdir/stitched-surface.json"
grep -q '"analytic-top-boundary-vertices":' \
  "$tmpdir/stitched-surface.json"
grep -q '"analytic-top-triangles":' "$tmpdir/stitched-surface.json"
grep -q '"analytic-top-area-checks":' "$tmpdir/stitched-surface.json"
grep -q '"analytic-top-area-failures": 0' "$tmpdir/stitched-surface.json"
grep -q '"analytic-top-ownership-rejected":' "$tmpdir/stitched-surface.json"
grep -q '"analytic-top-boundary-checks":' "$tmpdir/stitched-surface.json"
grep -q '"analytic-top-boundary-failures": 0' "$tmpdir/stitched-surface.json"
grep -q '"analytic-top-boundary-mismatched-edges": 0' \
  "$tmpdir/stitched-surface.json"
grep -q '"analytic-top-boundary-invalid-incidence-edges": 0' \
  "$tmpdir/stitched-surface.json"
grep -q '"analytic-closure-area-checks":' "$tmpdir/stitched-surface.json"
grep -q '"analytic-closure-area-failures": 0' \
  "$tmpdir/stitched-surface.json"
grep -q '"analytic-closure-area-expected":' \
  "$tmpdir/stitched-surface.json"
grep -q '"analytic-closure-area-triangles":' \
  "$tmpdir/stitched-surface.json"
grep -q '"analytic-closure-area-max-error":' \
  "$tmpdir/stitched-surface.json"
grep -q '"analytic-closure-boundary-checks":' \
  "$tmpdir/stitched-surface.json"
grep -q '"analytic-closure-boundary-failures": 0' \
  "$tmpdir/stitched-surface.json"
grep -q '"analytic-closure-boundary-mismatched-edges": 0' \
  "$tmpdir/stitched-surface.json"
grep -q '"analytic-closure-boundary-invalid-incidence-edges": 0' \
  "$tmpdir/stitched-surface.json"
grep -q '"analytic-bottom-patches":' "$tmpdir/stitched-surface.json"
grep -q '"analytic-bottom-boundary-vertices":' \
  "$tmpdir/stitched-surface.json"
grep -q '"analytic-bottom-area-checks":' "$tmpdir/stitched-surface.json"
grep -q '"analytic-bottom-area-failures": 0' \
  "$tmpdir/stitched-surface.json"
grep -q '"analytic-bottom-area-expected":' "$tmpdir/stitched-surface.json"
grep -q '"analytic-bottom-area-triangles":' "$tmpdir/stitched-surface.json"
grep -q '"analytic-bottom-area-max-error":' "$tmpdir/stitched-surface.json"
grep -Eq '"analytic-side-patches": [1-9]' "$tmpdir/stitched-surface.json"
grep -Eq '"analytic-side-area-checks": [1-9]' "$tmpdir/stitched-surface.json"
grep -q '"analytic-side-area-failures": 0' "$tmpdir/stitched-surface.json"
grep -q '"analytic-side-area-expected":' "$tmpdir/stitched-surface.json"
grep -q '"analytic-side-area-triangles":' "$tmpdir/stitched-surface.json"
grep -q '"analytic-side-area-max-error":' "$tmpdir/stitched-surface.json"
grep -q '"analytic-transition-area-checks":' "$tmpdir/stitched-surface.json"
grep -q '"analytic-transition-area-failures": 0' \
  "$tmpdir/stitched-surface.json"
grep -q '"analytic-transition-area-expected":' \
  "$tmpdir/stitched-surface.json"
grep -q '"analytic-transition-area-triangles":' \
  "$tmpdir/stitched-surface.json"
grep -q '"analytic-transition-area-max-error":' \
  "$tmpdir/stitched-surface.json"
grep -q '"topology-tolerance":' "$tmpdir/stitched-surface.json"
grep -q '"topology-triangles":' "$tmpdir/stitched-surface.json"
grep -q '"boundary-edges":' "$tmpdir/stitched-surface.json"
grep -q '"nonmanifold-edges":' "$tmpdir/stitched-surface.json"
grep -q '"misoriented-edges":' "$tmpdir/stitched-surface.json"
grep -q '"degenerate-triangles":' "$tmpdir/stitched-surface.json"
grep -q '"duplicate-triangles": 0' "$tmpdir/stitched-surface.json"
grep -q '"boundary-loops":' "$tmpdir/stitched-surface.json"
grep -q '"boundary-nonplanar-loops":' "$tmpdir/stitched-surface.json"
grep -q '"boundary-render-boundary-loops":' "$tmpdir/stitched-surface.json"
grep -q '"boundary-loop-details":' "$tmpdir/stitched-surface.json"
grep -Eq '"accepted": (true|false)' "$tmpdir/stitched-surface.json"

python3 - "$tmpdir/region-plan.json" "$tmpdir/region-surface.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as f:
    plan = json.load(f)["region-plan"]
with open(sys.argv[2], encoding="utf-8") as f:
    surface = json.load(f)["region-surface"]

bins = plan["xy-bins"]
leaf_coverage = [0] * (bins * bins)
leaves = plan["active-region-list"] + plan["analytic-region-list"]
for leaf in leaves:
    for y in range(leaf["tile-y"], leaf["tile-y"] + leaf["tile-height"]):
        for x in range(leaf["tile-x"], leaf["tile-x"] + leaf["tile-width"]):
            leaf_coverage[y * bins + x] += 1
if any(count != 1 for count in leaf_coverage):
    raise SystemExit("adaptive leaves do not partition the ownership lattice")
if plan["adaptive-leaf-count"] != len(leaves):
    raise SystemExit("adaptive leaf metric does not match emitted leaves")
if plan["adaptive-active-leaf-count"] != len(plan["active-region-list"]):
    raise SystemExit("adaptive active-leaf metric does not match regions")
if plan["adaptive-target-split-count"] <= 0:
    raise SystemExit("adaptive planner did not split for the cell target")
target = plan["target-region-cells"]
for leaf in plan["active-region-list"]:
    if leaf["estimated-cells"] <= target:
        continue
    if leaf["tile-width"] != 1 or leaf["tile-height"] != 1:
        raise SystemExit("splittable adaptive leaf exceeds the cell target")

if surface["cells-visited"] > plan["full-grid-cells-est"]:
    raise SystemExit("split sparse renderer visited more than the full grid")
if surface["cells-culled"] + surface["cells-contoured"] != surface[
    "cells-visited"
]:
    raise SystemExit("split sparse cell outcomes do not sum to visited cells")
if surface["toolsweep-depth-calls"] > surface["depth-calls"]:
    raise SystemExit("ToolSweep depth calls exceed all depth calls")
PY

if [ ! -s "$tmpdir/output.stl" ]; then
  echo "sparse pipeline did not write a non-empty STL" >&2
  exit 1
fi

cat > "$tmpdir/top-hole.ngc" <<'NC'
G21
T1 M6
G0 Z2
G0 X-6 Y-6
G1 Z-1 F100
G1 X6 Y-6
G0 Z2
G0 X-6 Y-3
G1 Z-1 F100
G1 X6 Y-3
G0 Z2
G0 X-6 Y0
G1 Z-1 F100
G1 X6 Y0
G0 Z2
G0 X-6 Y3
G1 Z-1 F100
G1 X6 Y3
G0 Z2
G0 X-6 Y6
G1 Z-1 F100
G1 X6 Y6
G0 Z2
M2
NC

cat > "$tmpdir/top-hole.camotics" <<JSON
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 1,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 10,
      "diameter": 1,
      "description": ""
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {
      "min": [-20, -20, -5],
      "max": [20, 20, 0]
    }
  },
  "files": ["top-hole.ngc"]
}
JSON

./camsim-path --threads 4 --resolution 1 \
  "$tmpdir/top-hole.camotics" "$tmpdir/top-hole-toolpath.json"
./camsim-region-plan \
  --xy-bins 8 \
  --halo-cells 1 \
  "$tmpdir/top-hole-toolpath.json" "$tmpdir/top-hole-region-plan.json"
./camsim-boundary-plan \
  "$tmpdir/top-hole-region-plan.json" \
  "$tmpdir/top-hole-ownership-boundary.json"
./camsim-render-regions --threads 4 \
  "$tmpdir/top-hole-toolpath.json" \
  "$tmpdir/top-hole-region-plan.json" \
  "$tmpdir/top-hole-region-surface.json"
./camsim-stitch-stock \
  --ownership-boundary "$tmpdir/top-hole-ownership-boundary.json" \
  "$tmpdir/top-hole-region-plan.json" \
  "$tmpdir/top-hole-region-surface.json" \
  "$tmpdir/top-hole-stitched-surface.json"

grep -q '"role": "analytic-top-hole"' \
  "$tmpdir/top-hole-ownership-boundary.json"
grep -q '"analytic-top-source": "ownership-boundary-loops"' \
  "$tmpdir/top-hole-stitched-surface.json"
grep -q '"analytic-top-unsupported-hole-loops": 0' \
  "$tmpdir/top-hole-stitched-surface.json"
grep -Eq '"analytic-top-triangles": [1-9]' \
  "$tmpdir/top-hole-stitched-surface.json"
grep -q '"analytic-bottom-source": "full-stock-bottom"' \
  "$tmpdir/top-hole-stitched-surface.json"
grep -q '"analytic-bottom-patches": 1' \
  "$tmpdir/top-hole-stitched-surface.json"
grep -Eq '"analytic-bottom-area-checks": [1-9]' \
  "$tmpdir/top-hole-stitched-surface.json"
grep -q '"analytic-bottom-area-failures": 0' \
  "$tmpdir/top-hole-stitched-surface.json"
grep -q '"boundary-edges": 0' "$tmpdir/top-hole-stitched-surface.json"
grep -q '"nonmanifold-edges": 0' "$tmpdir/top-hole-stitched-surface.json"
grep -q '"misoriented-edges": 0' "$tmpdir/top-hole-stitched-surface.json"
grep -q '"degenerate-triangles": 0' "$tmpdir/top-hole-stitched-surface.json"
grep -q '"duplicate-triangles": 0' "$tmpdir/top-hole-stitched-surface.json"
grep -q '"accepted": true' "$tmpdir/top-hole-stitched-surface.json"

python3 - "$tmpdir/top-hole-ownership-boundary.json" \
  "$tmpdir/unsupported-ownership-boundary.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as f:
    artifact = json.load(f)
artifact["ownership-boundary"]["ambiguous-vertices"] = 1
with open(sys.argv[2], "w", encoding="utf-8") as f:
    json.dump(artifact, f)
PY

if ./camsim-stitch-stock \
  --ownership-boundary "$tmpdir/unsupported-ownership-boundary.json" \
  "$tmpdir/top-hole-region-plan.json" \
  "$tmpdir/top-hole-region-surface.json" \
  "$tmpdir/unsupported-stitched-surface.json" 2>/dev/null
then
  echo "mutated ownership boundary hash was accepted" >&2
  exit 1
fi

cat > "$tmpdir/through-bottom.ngc" <<'NC'
G21
T1 M6
G0 Z2
G0 X-6 Y0
G1 Z-5 F100
G1 X6 Y0
G0 Z2
M2
NC

cat > "$tmpdir/through-bottom.camotics" <<JSON
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 1,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 10,
      "diameter": 1,
      "description": ""
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {
      "min": [-20, -20, -5],
      "max": [20, 20, 0]
    }
  },
  "files": ["through-bottom.ngc"]
}
JSON

./camsim-path --threads 4 --resolution 1 \
  "$tmpdir/through-bottom.camotics" "$tmpdir/through-bottom-toolpath.json"
./camsim-region-plan \
  --xy-bins 8 \
  --halo-cells 1 \
  "$tmpdir/through-bottom-toolpath.json" \
  "$tmpdir/through-bottom-region-plan.json"
./camsim-boundary-plan \
  "$tmpdir/through-bottom-region-plan.json" \
  "$tmpdir/through-bottom-ownership-boundary.json"
./camsim-render-regions --threads 4 \
  "$tmpdir/through-bottom-toolpath.json" \
  "$tmpdir/through-bottom-region-plan.json" \
  "$tmpdir/through-bottom-region-surface.json"
./camsim-stitch-stock \
  --ownership-boundary "$tmpdir/through-bottom-ownership-boundary.json" \
  "$tmpdir/through-bottom-region-plan.json" \
  "$tmpdir/through-bottom-region-surface.json" \
  "$tmpdir/through-bottom-stitched-surface.json"

grep -q '"analytic-bottom-source": "region-tile-fans"' \
  "$tmpdir/through-bottom-stitched-surface.json"
grep -Eq '"analytic-bottom-area-checks": [1-9]' \
  "$tmpdir/through-bottom-stitched-surface.json"
grep -q '"analytic-bottom-area-failures": 0' \
  "$tmpdir/through-bottom-stitched-surface.json"

cat > "$tmpdir/island-ring.ngc" <<'NC'
G21
T1 M6
G0 Z2
G0 X-12 Y-12
G1 Z-1 F100
G1 X12 Y-12
G0 Z2
G0 X12 Y-12
G1 Z-1 F100
G1 X12 Y12
G0 Z2
G0 X12 Y12
G1 Z-1 F100
G1 X-12 Y12
G0 Z2
G0 X-12 Y12
G1 Z-1 F100
G1 X-12 Y-12
G0 Z2
M2
NC

cat > "$tmpdir/island-ring.camotics" <<JSON
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 1,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 10,
      "diameter": 2,
      "description": ""
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {
      "min": [-20, -20, -5],
      "max": [20, 20, 0]
    }
  },
  "files": ["island-ring.ngc"]
}
JSON

./camsim-path --threads 4 --resolution 1 \
  "$tmpdir/island-ring.camotics" "$tmpdir/island-ring-toolpath.json"
./camsim-region-plan \
  --xy-bins 16 \
  --halo-cells 1 \
  "$tmpdir/island-ring-toolpath.json" \
  "$tmpdir/island-ring-region-plan.json"
./camsim-boundary-plan \
  "$tmpdir/island-ring-region-plan.json" \
  "$tmpdir/island-ring-ownership-boundary.json"
./camsim-render-regions --threads 4 \
  "$tmpdir/island-ring-toolpath.json" \
  "$tmpdir/island-ring-region-plan.json" \
  "$tmpdir/island-ring-region-surface.json"
./camsim-stitch-stock \
  --ownership-boundary "$tmpdir/island-ring-ownership-boundary.json" \
  "$tmpdir/island-ring-region-plan.json" \
  "$tmpdir/island-ring-region-surface.json" \
  "$tmpdir/island-ring-stitched-surface.json"

python3 - "$tmpdir/island-ring-region-plan.json" \
  "$tmpdir/island-ring-ownership-boundary.json" \
  "$tmpdir/island-ring-stitched-surface.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as f:
    region_plan = json.load(f)["region-plan"]
with open(sys.argv[2], encoding="utf-8") as f:
    boundary = json.load(f)["ownership-boundary"]
with open(sys.argv[3], encoding="utf-8") as f:
    stitch = json.load(f)["stitch-stock"]

active_tiles = sum(
    region["tile-width"] * region["tile-height"]
    for region in region_plan["active-region-list"]
)
render_tiles = sum(
    region["tile-width"] * region["tile-height"]
    for region in region_plan["render-region-list"]
)
interior_loops = [
    loop for loop in boundary["loops"]
    if loop["touches-stock-border"] == 0
]

if active_tiles <= 0:
    raise SystemExit("island ring planner produced no active tiles")
if render_tiles <= active_tiles:
    raise SystemExit("island ring render grouping did not cover the pocket "
                     "ring interior")
if len(boundary["loops"]) < 2 or not interior_loops:
    raise SystemExit("island ring boundary did not expose an interior "
                     "untouched island loop")
if boundary["open-loops"] != 0 or boundary["ambiguous-vertices"] != 0:
    raise SystemExit("island ring boundary loops were not cleanly traced")
if stitch["analytic-top-source"] != "ownership-boundary-loops":
    raise SystemExit("island ring stitch did not use ownership loops")
if stitch["analytic-top-unsupported-hole-loops"] != 0:
    raise SystemExit("island ring stitch reported unsupported top loops")
if stitch["analytic-top-area-checks"] <= 0:
    raise SystemExit("island ring stitch did not area-check top patches")
if stitch["analytic-top-area-failures"] != 0:
    raise SystemExit("island ring top patch area validation failed")
if stitch["analytic-top-boundary-checks"] <= 0:
    raise SystemExit("island ring did not validate top patch boundaries")
if stitch["analytic-top-boundary-failures"] != 0:
    raise SystemExit("island ring top boundary validation failed")
if stitch["analytic-top-boundary-mismatched-edges"] != 0:
    raise SystemExit("island ring top triangulation changed boundary edges")
if stitch["analytic-top-boundary-invalid-incidence-edges"] != 0:
    raise SystemExit("island ring top triangulation has invalid incidence")
if (stitch["analytic-top-boundary-expected-edges"] !=
        stitch["analytic-top-boundary-emitted-edges"]):
    raise SystemExit("island ring top boundary edge counts differ")
if not stitch["accepted"]:
    raise SystemExit("island ring sparse stitch was not accepted")
for key in [
    "boundary-edges",
    "nonmanifold-edges",
    "misoriented-edges",
    "degenerate-triangles",
    "duplicate-triangles",
]:
    if stitch[key] != 0:
        raise SystemExit("island ring sparse stitch has nonzero " + key)
PY

./camsim \
  --threads 4 \
  --resolution 1 \
  "$tmpdir/top-hole.camotics" \
  "$tmpdir/top-hole-baseline.stl"

./camsim \
  --sparse-toolpath \
  --sparse-toolpath-xy-bins 8 \
  --sparse-toolpath-halo-cells 1 \
  --sparse-toolpath-target-region-cells 200 \
  --threads 4 \
  --resolution 1 \
  --profile "$tmpdir/top-hole-integrated-profile.json" \
  "$tmpdir/top-hole.camotics" \
  "$tmpdir/top-hole-integrated.stl"

if [ ! -s "$tmpdir/top-hole-integrated.stl" ]; then
  echo "integrated sparse camsim did not write a non-empty STL" >&2
  exit 1
fi

python3 scripts/perf/compare_stl_distance.py \
  --max-samples 300 \
  --cell-size 2 \
  --max-shells 20 \
  --hard-max-error 1.1 \
  --p99-error 1.1 \
  "$tmpdir/top-hole-baseline.stl" \
  "$tmpdir/top-hole-integrated.stl"

python3 - "$tmpdir/top-hole-integrated-profile.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as f:
    profile = json.load(f)

metrics = profile.get("metrics", {})
required = [
    "sparse_toolpath_integrated_enabled",
    "sparse_region_plan_active_regions",
    "sparse_region_plan_render_regions",
    "sparse_region_plan_render_cells_est",
    "sparse_region_plan_target_region_cells",
    "sparse_region_plan_adaptive_leaf_count",
    "sparse_region_plan_adaptive_active_leaf_count",
    "sparse_region_plan_adaptive_split_count",
    "sparse_region_plan_adaptive_ownership_split_count",
    "sparse_region_plan_adaptive_depth_split_count",
    "sparse_region_plan_adaptive_density_split_count",
    "sparse_region_plan_adaptive_target_split_count",
    "sparse_region_plan_adaptive_max_leaf_cells",
    "sparse_region_plan_adaptive_target_exceeded_leaves",
    "sparse_boundary_plan_loops",
    "sparse_region_surface_triangles",
    "sparse_region_surface_plan_render_regions",
    "sparse_region_surface_cells_visited",
    "sparse_region_surface_cells_culled",
    "sparse_region_surface_cells_contoured",
    "sparse_region_surface_vertex_samples",
    "sparse_region_surface_depth_calls",
    "sparse_region_surface_toolsweep_depth_calls",
    "sparse_region_surface_edge_checks",
    "sparse_region_surface_edge_intersections",
    "sparse_stitch_ownership_boundary_supplied",
    "sparse_stitch_analytic_top_ownership_boundary_used",
    "sparse_stitch_analytic_top_unsupported_hole_loops",
    "sparse_stitch_analytic_top_ownership_rejected",
    "sparse_stitch_analytic_top_area_checks",
    "sparse_stitch_analytic_top_area_failures",
    "sparse_stitch_analytic_top_boundary_checks",
    "sparse_stitch_analytic_top_boundary_failures",
    "sparse_stitch_analytic_top_boundary_expected_edges",
    "sparse_stitch_analytic_top_boundary_emitted_edges",
    "sparse_stitch_analytic_top_boundary_mismatched_edges",
    "sparse_stitch_analytic_top_boundary_invalid_incidence_edges",
    "sparse_stitch_analytic_closure_patches",
    "sparse_stitch_analytic_closure_area_checks",
    "sparse_stitch_analytic_closure_area_failures",
    "sparse_stitch_analytic_closure_area_expected_scaled_1e6",
    "sparse_stitch_analytic_closure_area_triangles_scaled_1e6",
    "sparse_stitch_analytic_closure_area_max_error_scaled_1e6",
    "sparse_stitch_analytic_closure_boundary_checks",
    "sparse_stitch_analytic_closure_boundary_failures",
    "sparse_stitch_analytic_closure_boundary_expected_edges",
    "sparse_stitch_analytic_closure_boundary_emitted_edges",
    "sparse_stitch_analytic_closure_boundary_mismatched_edges",
    "sparse_stitch_analytic_closure_boundary_invalid_incidence_edges",
    "sparse_stitch_analytic_bottom_full_stock_used",
    "sparse_stitch_analytic_bottom_area_checks",
    "sparse_stitch_analytic_bottom_area_failures",
    "sparse_stitch_analytic_bottom_area_expected_scaled_1e6",
    "sparse_stitch_analytic_bottom_area_triangles_scaled_1e6",
    "sparse_stitch_analytic_bottom_area_max_error_scaled_1e6",
    "sparse_stitch_analytic_side_area_checks",
    "sparse_stitch_analytic_side_area_failures",
    "sparse_stitch_analytic_side_area_expected_scaled_1e6",
    "sparse_stitch_analytic_side_area_triangles_scaled_1e6",
    "sparse_stitch_analytic_side_area_max_error_scaled_1e6",
    "sparse_stitch_boundary_loops",
    "sparse_stitch_boundary_nonplanar_loops",
    "sparse_stitch_boundary_render_boundary_loops",
    "sparse_stitch_boundary_nonplanar_render_boundary_loops",
    "sparse_stitch_duplicate_triangles",
    "sparse_stitch_topology_accepted",
    "sparse_stitch_analytic_triangles",
    "sparse_stitch_output_triangles",
    "sparse_stitch_reduction_origin_metadata_valid",
    "sparse_stitch_reduction_mc_reducible_triangles",
    "sparse_stitch_reduction_mc_seam_locked_triangles",
    "sparse_stitch_reduction_analytic_locked_triangles",
    "sparse_stitch_reduction_unknown_locked_triangles",
    "sparse_stitch_reduction_locked_seam_vertices",
    "sparse_stitch_reduction_locked_seam_edges",
    "sparse_toolpath_integrated_topology_fallback",
    "sparse_toolpath_integrated_geometry_fallback",
]
missing = [name for name in required if name not in metrics]
if missing:
    raise SystemExit("integrated sparse profile missing metrics: " +
                     ", ".join(missing))

if metrics["sparse_toolpath_integrated_enabled"] != 1:
    raise SystemExit("integrated sparse mode did not enable")
if metrics["sparse_region_plan_active_regions"] < 1:
    raise SystemExit("integrated sparse did not plan active regions")
if metrics["sparse_region_plan_render_regions"] < 1:
    raise SystemExit("integrated sparse did not plan render regions")
if metrics["sparse_region_plan_render_cells_est"] < 1:
    raise SystemExit("integrated sparse did not estimate render cells")
if metrics["sparse_region_plan_target_region_cells"] != 200:
    raise SystemExit("integrated sparse did not preserve the cell target")
if metrics["sparse_region_plan_adaptive_leaf_count"] <= 0:
    raise SystemExit("integrated sparse produced no adaptive leaves")
if metrics["sparse_region_plan_adaptive_active_leaf_count"] <= 0:
    raise SystemExit("integrated sparse produced no active adaptive leaves")
if metrics["sparse_region_plan_adaptive_split_count"] <= 0:
    raise SystemExit("integrated sparse did not adaptively split ownership")
if metrics["sparse_boundary_plan_loops"] < 1:
    raise SystemExit("integrated sparse did not build boundary loops")
if metrics["sparse_region_surface_triangles"] < 1:
    raise SystemExit("integrated sparse rendered no MC triangles")
if metrics["sparse_region_surface_plan_render_regions"] < 1:
    raise SystemExit("integrated sparse surface did not use render regions")
if metrics["sparse_region_surface_cells_visited"] <= 0:
    raise SystemExit("integrated sparse recorded no visited cells")
if metrics["sparse_region_surface_cells_contoured"] <= 0:
    raise SystemExit("integrated sparse recorded no contoured cells")
if metrics["sparse_region_surface_vertex_samples"] <= 0:
    raise SystemExit("integrated sparse recorded no vertex samples")
if metrics["sparse_region_surface_depth_calls"] <= 0:
    raise SystemExit("integrated sparse recorded no depth calls")
if metrics["sparse_region_surface_toolsweep_depth_calls"] <= 0:
    raise SystemExit("integrated sparse recorded no ToolSweep depth calls")
if (metrics["sparse_region_surface_cells_culled"] +
        metrics["sparse_region_surface_cells_contoured"] !=
        metrics["sparse_region_surface_cells_visited"]):
    raise SystemExit("integrated sparse cell outcomes do not sum to visits")
if metrics["sparse_region_surface_cells_visited"] > metrics[
    "sparse_region_plan_full_cells_est"
]:
    raise SystemExit("integrated sparse visited more than the full grid")
if metrics["sparse_stitch_ownership_boundary_supplied"] != 1:
    raise SystemExit("integrated sparse did not supply ownership boundary")
if metrics["sparse_stitch_analytic_top_ownership_boundary_used"] != 1:
    raise SystemExit("integrated sparse did not use top boundary loops")
if metrics["sparse_stitch_analytic_top_unsupported_hole_loops"] != 0:
    raise SystemExit("integrated sparse reported unsupported top holes")
if metrics["sparse_stitch_analytic_top_ownership_rejected"] != 0:
    raise SystemExit("integrated sparse rejected supported top ownership")
if metrics["sparse_stitch_analytic_top_area_checks"] <= 0:
    raise SystemExit("integrated sparse did not area-check top patches")
if metrics["sparse_stitch_analytic_top_area_failures"] != 0:
    raise SystemExit("integrated sparse top patch area validation failed")
if metrics["sparse_stitch_analytic_top_boundary_checks"] <= 0:
    raise SystemExit("integrated sparse did not validate top boundaries")
if metrics["sparse_stitch_analytic_top_boundary_failures"] != 0:
    raise SystemExit("integrated sparse top boundary validation failed")
if metrics["sparse_stitch_analytic_top_boundary_mismatched_edges"] != 0:
    raise SystemExit("integrated sparse top triangulation changed boundaries")
if metrics[
    "sparse_stitch_analytic_top_boundary_invalid_incidence_edges"
] != 0:
    raise SystemExit("integrated sparse top triangulation has invalid incidence")
if metrics["sparse_stitch_analytic_closure_area_failures"] != 0:
    raise SystemExit("integrated sparse closure patch area validation failed")
if (metrics["sparse_stitch_analytic_closure_patches"] > 0 and
        metrics["sparse_stitch_analytic_closure_area_checks"] <= 0):
    raise SystemExit("integrated sparse emitted unchecked closure patches")
if metrics["sparse_stitch_analytic_bottom_full_stock_used"] != 1:
    raise SystemExit("integrated sparse did not use full-stock bottom patch")
if metrics["sparse_stitch_analytic_bottom_area_checks"] <= 0:
    raise SystemExit("integrated sparse did not area-check bottom patches")
if metrics["sparse_stitch_analytic_bottom_area_failures"] != 0:
    raise SystemExit("integrated sparse bottom patch area validation failed")
if metrics["sparse_stitch_analytic_side_area_checks"] <= 0:
    raise SystemExit("integrated sparse did not area-check side patches")
if metrics["sparse_stitch_analytic_side_area_failures"] != 0:
    raise SystemExit("integrated sparse side patch area validation failed")
if metrics["sparse_stitch_boundary_loops"] != 0:
    raise SystemExit("integrated sparse accepted fixture has boundary loops")
if metrics["sparse_stitch_boundary_nonplanar_loops"] != 0:
    raise SystemExit("integrated sparse accepted fixture has nonplanar loops")
if metrics["sparse_stitch_boundary_render_boundary_loops"] != 0:
    raise SystemExit("integrated sparse accepted fixture has render-boundary "
                     "loops")
if metrics["sparse_stitch_boundary_nonplanar_render_boundary_loops"] != 0:
    raise SystemExit("integrated sparse accepted fixture has nonplanar "
                     "render-boundary loops")
if metrics["sparse_stitch_topology_accepted"] != 1:
    raise SystemExit("integrated sparse topology was not accepted")
if metrics["sparse_stitch_duplicate_triangles"] != 0:
    raise SystemExit("integrated sparse has duplicate triangles")
if metrics["sparse_stitch_reduction_origin_metadata_valid"] != 1:
    raise SystemExit("integrated sparse reduction metadata is invalid")
if metrics["sparse_stitch_reduction_mc_reducible_triangles"] <= 0:
    raise SystemExit("integrated sparse classified no reducible MC triangles")
if metrics["sparse_stitch_reduction_mc_seam_locked_triangles"] <= 0:
    raise SystemExit("integrated sparse classified no seam-locked MC triangles")
if (metrics["sparse_stitch_reduction_analytic_locked_triangles"] !=
        metrics["sparse_stitch_analytic_triangles"]):
    raise SystemExit("integrated sparse analytic origin count changed")
if metrics["sparse_stitch_reduction_unknown_locked_triangles"] != 0:
    raise SystemExit("integrated sparse has unknown triangle origins")
if metrics["sparse_stitch_reduction_locked_seam_vertices"] <= 0:
    raise SystemExit("integrated sparse recorded no locked seam vertices")
if metrics["sparse_stitch_reduction_locked_seam_edges"] <= 0:
    raise SystemExit("integrated sparse recorded no locked seam edges")
if (metrics["sparse_stitch_reduction_mc_reducible_triangles"] +
        metrics["sparse_stitch_reduction_mc_seam_locked_triangles"] +
        metrics["sparse_stitch_reduction_analytic_locked_triangles"] !=
        metrics["sparse_stitch_output_triangles"]):
    raise SystemExit("integrated sparse origin classes do not cover output")
if metrics["sparse_toolpath_integrated_topology_fallback"] != 0:
    raise SystemExit("integrated sparse unexpectedly fell back to full render")
if metrics["sparse_toolpath_integrated_geometry_fallback"] != 0:
    raise SystemExit("integrated sparse unexpectedly failed geometry validation")
PY

cat > "$tmpdir/x-cut.ngc" <<'NC'
G21
T1 M6
G0 Z2
G0 X-40 Y-40
G1 Z-1 F100
G1 X40 Y40
G0 Z2
G0 X-40 Y40
G1 Z-1 F100
G1 X40 Y-40
G0 Z2
M2
NC

cat > "$tmpdir/x-cut.camotics" <<JSON
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 1,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 10,
      "diameter": 1,
      "description": ""
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {
      "min": [-50, -50, -5],
      "max": [50, 50, 0]
    }
  },
  "files": ["x-cut.ngc"]
}
JSON

./camsim-path --threads 4 --resolution 1 \
  "$tmpdir/x-cut.camotics" "$tmpdir/x-cut-toolpath.json"
./camsim-region-plan \
  --xy-bins 32 \
  --halo-cells 1 \
  "$tmpdir/x-cut-toolpath.json" "$tmpdir/x-cut-region-plan.json"
./camsim-boundary-plan \
  "$tmpdir/x-cut-region-plan.json" \
  "$tmpdir/x-cut-ownership-boundary.json"

python3 - "$tmpdir/x-cut-region-plan.json" \
  "$tmpdir/x-cut-ownership-boundary.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as f:
    region_plan = json.load(f)["region-plan"]
with open(sys.argv[2], encoding="utf-8") as f:
    boundary = json.load(f)["ownership-boundary"]

active_tiles = sum(
    region["tile-width"] * region["tile-height"]
    for region in region_plan["active-region-list"]
)
render_tiles = sum(
    region["tile-width"] * region["tile-height"]
    for region in region_plan["render-region-list"]
)
render_regions = len(region_plan["render-region-list"])
active_regions = len(region_plan["active-region-list"])
total_tiles = region_plan["xy-bins"] * region_plan["xy-bins"]
filtered_refs = region_plan["toolpath-filtered-tile-refs"]
bbox_refs = region_plan["bbox-tile-refs"]

if active_tiles <= 0:
    raise SystemExit("x-cut planner produced no active tiles")
if render_tiles < active_tiles:
    raise SystemExit("x-cut render grouping lost active ownership tiles")
if render_tiles >= total_tiles:
    raise SystemExit("x-cut render grouping covered every tile")
if render_regions >= active_regions:
    raise SystemExit("x-cut render grouping did not reduce render chunks")
if region_plan["render-cells-est"] < region_plan["active-cells-est"]:
    raise SystemExit("x-cut render cell estimate is smaller than active "
                     "ownership estimate")
if active_tiles >= 260:
    raise SystemExit("x-cut planner did not keep the diagonal ownership sparse")
if active_tiles >= total_tiles:
    raise SystemExit("x-cut planner activated every tile")
if bbox_refs <= 0 or filtered_refs <= 0:
    raise SystemExit("x-cut planner did not report capsule-filtered tile refs")
if bbox_refs <= filtered_refs:
    raise SystemExit("x-cut planner filtered all bbox tile refs")
if region_plan["active-cells-est"] >= region_plan["full-grid-cells-est"]:
    raise SystemExit("x-cut planner did not reduce active cells")
if region_plan["skipped-cells-est"] <= 0:
    raise SystemExit("x-cut planner did not report skipped cells")
if boundary["loop-count"] < 1:
    raise SystemExit("x-cut boundary planner emitted no loops")
if boundary["open-loops"] != 0:
    raise SystemExit("x-cut boundary planner emitted open loops")
if boundary["ambiguous-vertices"] != 0:
    raise SystemExit("x-cut boundary planner emitted ambiguous vertices")
PY

./camsim-render-regions --threads 4 \
  "$tmpdir/x-cut-toolpath.json" \
  "$tmpdir/x-cut-region-plan.json" \
  "$tmpdir/x-cut-region-surface.json"
./camsim-stitch-stock \
  --ownership-boundary "$tmpdir/x-cut-ownership-boundary.json" \
  "$tmpdir/x-cut-region-plan.json" \
  "$tmpdir/x-cut-region-surface.json" \
  "$tmpdir/x-cut-stitched-surface.json"

python3 - "$tmpdir/x-cut-stitched-surface.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as f:
    stitch = json.load(f)["stitch-stock"]

required = [
    "boundary-loops",
    "boundary-open-chains",
    "boundary-planar-loops",
    "boundary-nonplanar-loops",
    "boundary-supported-closure-loops",
    "boundary-render-boundary-loops",
    "boundary-nonplanar-render-boundary-loops",
    "boundary-loop-details",
    "analytic-top-area-checks",
    "analytic-top-area-failures",
    "analytic-top-boundary-checks",
    "analytic-top-boundary-failures",
    "analytic-top-boundary-mismatched-edges",
    "analytic-top-boundary-invalid-incidence-edges",
    "analytic-top-ownership-rejected",
    "analytic-closure-patches",
    "analytic-closure-area-checks",
    "analytic-closure-area-failures",
    "analytic-closure-area-expected",
    "analytic-closure-area-triangles",
    "analytic-closure-area-max-error",
    "analytic-closure-boundary-checks",
    "analytic-closure-boundary-failures",
    "analytic-closure-boundary-mismatched-edges",
    "analytic-closure-boundary-invalid-incidence-edges",
    "analytic-bottom-area-checks",
    "analytic-bottom-area-failures",
    "analytic-bottom-area-expected",
    "analytic-bottom-area-triangles",
    "analytic-bottom-area-max-error",
    "analytic-side-area-checks",
    "analytic-side-area-failures",
    "analytic-side-area-expected",
    "analytic-side-area-triangles",
    "analytic-side-area-max-error",
    "analytic-render-grid-snap-used",
    "analytic-render-grid-snap-candidate-boundary-edges",
    "boundary-edges",
    "duplicate-triangles",
    "accepted",
]
missing = [name for name in required if name not in stitch]
if missing:
    raise SystemExit("x-cut stitch artifact missing boundary diagnostics: " +
                     ", ".join(missing))

if stitch["accepted"]:
    if stitch["boundary-edges"] != 0 or stitch["boundary-loops"] != 0:
        raise SystemExit("accepted x-cut stitch still reports open boundaries")
    if stitch["duplicate-triangles"] != 0:
        raise SystemExit("accepted x-cut stitch has duplicate triangles")
    if stitch["analytic-top-area-checks"] <= 0:
        raise SystemExit("accepted x-cut did not area-check top patches")
    if stitch["analytic-top-area-failures"] != 0:
        raise SystemExit("accepted x-cut top patch area validation failed")
    if stitch["analytic-top-boundary-checks"] <= 0:
        raise SystemExit("accepted x-cut did not validate top boundaries")
    if stitch["analytic-top-boundary-failures"] != 0:
        raise SystemExit("accepted x-cut top boundary validation failed")
    if stitch["analytic-top-boundary-mismatched-edges"] != 0:
        raise SystemExit("accepted x-cut top triangulation changed boundaries")
    if stitch["analytic-top-boundary-invalid-incidence-edges"] != 0:
        raise SystemExit("accepted x-cut top triangulation invalid incidence")
    if stitch["analytic-top-ownership-rejected"] != 0:
        raise SystemExit("accepted x-cut rejected top ownership")
    if stitch["analytic-closure-area-failures"] != 0:
        raise SystemExit("accepted x-cut closure patch area validation failed")
    if (stitch["analytic-closure-patches"] > 0 and
            stitch["analytic-closure-area-checks"] <= 0):
        raise SystemExit("accepted x-cut emitted unchecked closure patches")
    if (stitch["analytic-closure-patches"] > 0 and
            stitch["analytic-closure-boundary-checks"] <= 0):
        raise SystemExit("accepted x-cut emitted unchecked closure boundaries")
    if stitch["analytic-closure-boundary-failures"] != 0:
        raise SystemExit("accepted x-cut closure boundary validation failed")
    if stitch["analytic-bottom-area-checks"] <= 0:
        raise SystemExit("accepted x-cut did not area-check bottom patches")
    if stitch["analytic-bottom-area-failures"] != 0:
        raise SystemExit("accepted x-cut bottom patch area validation failed")
    if stitch["analytic-side-area-checks"] <= 0:
        raise SystemExit("accepted x-cut did not area-check side patches")
    if stitch["analytic-side-area-failures"] != 0:
        raise SystemExit("accepted x-cut side patch area validation failed")
else:
    if stitch["boundary-edges"] <= 0 or stitch["boundary-loops"] <= 0:
        raise SystemExit("rejected x-cut stitch did not report open boundaries")
    if stitch["boundary-nonplanar-loops"] <= 0:
        raise SystemExit("rejected x-cut stitch did not classify nonplanar "
                         "MC boundary loops")
    if stitch["boundary-render-boundary-loops"] <= 0:
        raise SystemExit("rejected x-cut stitch did not classify render "
                         "boundary loops")
    if stitch["boundary-nonplanar-render-boundary-loops"] <= 0:
        raise SystemExit("rejected x-cut stitch did not classify nonplanar "
                         "render boundary loops")
    details = stitch["boundary-loop-details"]
    if len(details) != stitch["boundary-loops"]:
        raise SystemExit("x-cut boundary detail count does not match loops")
    if not any(
        not detail["planar"] and detail["touches-render-boundary"]
        for detail in details
    ):
        raise SystemExit("x-cut boundary details did not identify nonplanar "
                         "render-boundary loops")
    for detail in details:
        bounds = detail.get("bounds", {})
        if "min" not in bounds or "max" not in bounds:
            raise SystemExit("x-cut boundary detail missing bounds")
    raise SystemExit("x-cut sparse stitch is still rejected")
PY

cat > "$tmpdir/mixed-depth.ngc" <<'NC'
G21
T1 M6
G0 Z2
G0 X-16 Y0
G1 Z-1 F100
G1 X0 Y0
G0 Z2
G0 X0 Y0
G1 Z-6 F100
G1 X16 Y0
G0 Z2
M2
NC

cat > "$tmpdir/mixed-depth.camotics" <<JSON
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 1,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 10,
      "diameter": 1,
      "description": ""
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {
      "min": [-20, -10, -10],
      "max": [20, 10, 0]
    }
  },
  "files": ["mixed-depth.ngc"]
}
JSON

./camsim \
  --threads 4 \
  --resolution 1 \
  "$tmpdir/mixed-depth.camotics" \
  "$tmpdir/mixed-depth-baseline.stl"

./camsim \
  --sparse-toolpath \
  --sparse-toolpath-xy-bins 8 \
  --sparse-toolpath-halo-cells 1 \
  --threads 4 \
  --resolution 1 \
  --profile "$tmpdir/mixed-depth-integrated-profile.json" \
  "$tmpdir/mixed-depth.camotics" \
  "$tmpdir/mixed-depth-integrated.stl"

python3 scripts/perf/compare_stl_distance.py \
  --max-samples 500 \
  --cell-size 2 \
  --max-shells 30 \
  --hard-max-error 1.1 \
  --p99-error 1.1 \
  "$tmpdir/mixed-depth-baseline.stl" \
  "$tmpdir/mixed-depth-integrated.stl"

python3 - "$tmpdir/mixed-depth-integrated-profile.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as f:
    metrics = json.load(f).get("metrics", {})

required = [
    "sparse_region_plan_full_cells_est",
    "sparse_region_plan_active_cells_est",
    "sparse_region_plan_skipped_cells_est",
    "sparse_region_plan_active_depth_groups",
    "sparse_stitch_analytic_transition_patches",
    "sparse_stitch_analytic_transition_area_checks",
    "sparse_stitch_analytic_transition_area_failures",
    "sparse_stitch_analytic_transition_area_expected_scaled_1e6",
    "sparse_stitch_analytic_transition_area_triangles_scaled_1e6",
    "sparse_stitch_analytic_transition_area_max_error_scaled_1e6",
    "sparse_stitch_analytic_closure_rejected",
    "sparse_stitch_analytic_closure_candidate_boundary_edges",
    "sparse_stitch_analytic_closure_candidate_nonmanifold_edges",
    "sparse_stitch_analytic_transition_replacement_candidate_boundary_edges",
    "sparse_stitch_analytic_transition_replacement_candidate_nonmanifold_edges",
    "sparse_stitch_analytic_render_grid_snap_used",
    "sparse_stitch_analytic_render_grid_snap_candidate_boundary_edges",
    "sparse_stitch_analytic_top_area_checks",
    "sparse_stitch_analytic_top_area_failures",
    "sparse_stitch_analytic_top_ownership_rejected",
    "sparse_stitch_analytic_closure_patches",
    "sparse_stitch_analytic_closure_area_checks",
    "sparse_stitch_analytic_closure_area_failures",
    "sparse_stitch_analytic_closure_area_expected_scaled_1e6",
    "sparse_stitch_analytic_closure_area_triangles_scaled_1e6",
    "sparse_stitch_analytic_closure_area_max_error_scaled_1e6",
    "sparse_stitch_analytic_closure_boundary_checks",
    "sparse_stitch_analytic_closure_boundary_failures",
    "sparse_stitch_analytic_closure_boundary_expected_edges",
    "sparse_stitch_analytic_closure_boundary_emitted_edges",
    "sparse_stitch_analytic_closure_boundary_mismatched_edges",
    "sparse_stitch_analytic_closure_boundary_invalid_incidence_edges",
    "sparse_stitch_analytic_bottom_full_stock_used",
    "sparse_stitch_analytic_bottom_area_checks",
    "sparse_stitch_analytic_bottom_area_failures",
    "sparse_stitch_analytic_bottom_area_expected_scaled_1e6",
    "sparse_stitch_analytic_bottom_area_triangles_scaled_1e6",
    "sparse_stitch_analytic_bottom_area_max_error_scaled_1e6",
    "sparse_stitch_boundary_edges",
    "sparse_stitch_duplicate_triangles",
    "sparse_stitch_topology_accepted",
    "sparse_stitch_analytic_triangles",
    "sparse_stitch_output_triangles",
    "sparse_stitch_reduction_origin_metadata_valid",
    "sparse_stitch_reduction_mc_reducible_triangles",
    "sparse_stitch_reduction_mc_seam_locked_triangles",
    "sparse_stitch_reduction_analytic_locked_triangles",
    "sparse_stitch_reduction_unknown_locked_triangles",
    "sparse_stitch_reduction_locked_seam_vertices",
    "sparse_stitch_reduction_locked_seam_edges",
    "sparse_toolpath_integrated_topology_fallback",
    "sparse_toolpath_integrated_geometry_fallback",
]
missing = [name for name in required if name not in metrics]
if missing:
    raise SystemExit("mixed-depth sparse profile missing metrics: " +
                     ", ".join(missing))

if metrics["sparse_region_plan_active_depth_groups"] < 2:
    raise SystemExit("mixed-depth sparse did not split active depths")
if metrics["sparse_region_plan_active_cells_est"] >= metrics[
    "sparse_region_plan_full_cells_est"
]:
    raise SystemExit("mixed-depth sparse did not reduce planned cells")
if metrics["sparse_region_plan_skipped_cells_est"] <= 0:
    raise SystemExit("mixed-depth sparse did not report skipped cells")
if metrics["sparse_stitch_analytic_top_area_checks"] <= 0:
    raise SystemExit("mixed-depth sparse did not area-check top patches")
if metrics["sparse_stitch_analytic_top_area_failures"] != 0:
    raise SystemExit("mixed-depth sparse top patch area validation failed")
if metrics["sparse_stitch_analytic_top_ownership_rejected"] != 0:
    raise SystemExit("mixed-depth sparse rejected top ownership")
if metrics["sparse_stitch_analytic_closure_patches"] <= 0:
    raise SystemExit("mixed-depth sparse emitted no closure patches")
if metrics["sparse_stitch_analytic_closure_area_checks"] <= 0:
    raise SystemExit("mixed-depth sparse did not area-check closure patches")
if metrics["sparse_stitch_analytic_closure_area_failures"] != 0:
    raise SystemExit("mixed-depth sparse closure patch area validation failed")
if metrics["sparse_stitch_analytic_closure_boundary_checks"] <= 0:
    raise SystemExit("mixed-depth sparse did not validate closure boundaries")
if metrics["sparse_stitch_analytic_closure_boundary_failures"] != 0:
    raise SystemExit("mixed-depth sparse closure boundary validation failed")
if metrics["sparse_stitch_analytic_closure_boundary_mismatched_edges"] != 0:
    raise SystemExit("mixed-depth sparse closure triangulation changed edges")
if metrics[
    "sparse_stitch_analytic_closure_boundary_invalid_incidence_edges"
] != 0:
    raise SystemExit("mixed-depth sparse closure invalid edge incidence")
if metrics["sparse_stitch_analytic_bottom_area_checks"] <= 0:
    raise SystemExit("mixed-depth sparse did not area-check bottom patches")
if metrics["sparse_stitch_analytic_bottom_area_failures"] != 0:
    raise SystemExit("mixed-depth sparse bottom patch area validation failed")
if metrics["sparse_stitch_analytic_transition_area_failures"] != 0:
    raise SystemExit("mixed-depth sparse transition area validation failed")
if metrics["sparse_stitch_duplicate_triangles"] != 0:
    raise SystemExit("mixed-depth sparse has duplicate triangles")
if metrics["sparse_toolpath_integrated_geometry_fallback"] != 0:
    raise SystemExit("mixed-depth sparse unexpectedly failed geometry "
                     "validation")

accepted = metrics["sparse_stitch_topology_accepted"] == 1
fallback = metrics["sparse_toolpath_integrated_topology_fallback"] == 1
if accepted == fallback:
    raise SystemExit(
        "mixed-depth sparse must either accept topology or fall back, not both"
    )

if accepted:
    if metrics["sparse_stitch_reduction_origin_metadata_valid"] != 1:
        raise SystemExit("mixed-depth sparse reduction metadata is invalid")
    if metrics["sparse_stitch_reduction_mc_reducible_triangles"] <= 0:
        raise SystemExit("mixed-depth sparse classified no reducible MC")
    if metrics["sparse_stitch_reduction_mc_seam_locked_triangles"] <= 0:
        raise SystemExit("mixed-depth sparse classified no seam-locked MC")
    if (metrics["sparse_stitch_reduction_analytic_locked_triangles"] !=
            metrics["sparse_stitch_analytic_triangles"]):
        raise SystemExit("mixed-depth sparse analytic origin count changed")
    if metrics["sparse_stitch_reduction_unknown_locked_triangles"] != 0:
        raise SystemExit("mixed-depth sparse has unknown origins")
    if metrics["sparse_stitch_reduction_locked_seam_vertices"] <= 0:
        raise SystemExit("mixed-depth sparse recorded no seam vertices")
    if metrics["sparse_stitch_reduction_locked_seam_edges"] <= 0:
        raise SystemExit("mixed-depth sparse recorded no seam edges")

if not accepted:
    final_boundary = metrics["sparse_stitch_boundary_edges"]
    closure_boundary = metrics[
        "sparse_stitch_analytic_closure_candidate_boundary_edges"
    ]
    replacement_boundary = metrics[
        "sparse_stitch_analytic_transition_replacement_candidate_boundary_edges"
    ]

    if final_boundary <= 0:
        raise SystemExit("mixed-depth sparse fallback should still expose "
                         "unclosed stock-side boundaries")
    if closure_boundary <= 0 and replacement_boundary <= 0:
        raise SystemExit("mixed-depth sparse did not evaluate seam closure")
    if replacement_boundary <= 0:
        raise SystemExit("mixed-depth sparse did not evaluate transition "
                         "replacement")
    if final_boundary > replacement_boundary:
        raise SystemExit("mixed-depth sparse did not keep the improved "
                         "transition replacement topology")
    if metrics[
        "sparse_stitch_analytic_transition_replacement_candidate_nonmanifold_edges"
    ] != 0:
        raise SystemExit("mixed-depth sparse transition replacement introduced "
                         "nonmanifold edges")
PY

python3 - "$tmpdir" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
fixtures = {
    "multiple-islands": [
        (-20, -15), (20, -15), (20, 15), (-20, 15),
        (-20, -15), (0, -15), (0, 15),
    ],
    "border-touch": [
        (-30, -8), (-14, -8), (-14, 8), (-30, 8), (-30, -8),
    ],
    "touching-holes": [
        (-20, -12), (0, -12), (0, 12), (-20, 12), (-20, -12),
        (20, -12), (20, 12), (0, 12), (0, -12),
    ],
    "nested-rings": [
        (-20, -16), (20, -16), (20, 16), (-20, 16), (-20, -16),
        (-8, -8), (8, -8), (8, 8), (-8, 8), (-8, -8),
    ],
    "narrow-channel": [
        (-20, -0.5), (20, -0.5), (20, 0.5), (-20, 0.5),
    ],
}

for name, points in fixtures.items():
    gcode = [
        "G21", "T1 M6", "G0 Z2",
        f"G0 X{points[0][0]} Y{points[0][1]}",
        "G1 Z-1 F100",
    ]
    gcode.extend(f"G1 X{x} Y{y}" for x, y in points[1:])
    gcode.extend(["G0 Z2", "M2"])
    (root / f"{name}.ngc").write_text(
        "\n".join(gcode) + "\n", encoding="utf-8"
    )
    project = {
        "units": "metric",
        "resolution-mode": "manual",
        "resolution": 1,
        "tools": {
            "1": {
                "units": "metric",
                "shape": "cylindrical",
                "length": 20,
                "diameter": 2,
                "description": "",
            }
        },
        "workpiece": {
            "automatic": False,
            "margin": 0,
            "bounds": {
                "min": [-30, -25, -5],
                "max": [30, 25, 0],
            },
        },
        "files": [f"{name}.ngc"],
    }
    (root / f"{name}.camotics").write_text(
        json.dumps(project), encoding="utf-8"
    )
PY

for name in multiple-islands border-touch; do
  ./camsim-path --threads 4 --resolution 1 \
    "$tmpdir/$name.camotics" "$tmpdir/$name-toolpath.json"
  ./camsim-region-plan \
    --xy-bins 24 \
    --halo-cells 1 \
    "$tmpdir/$name-toolpath.json" "$tmpdir/$name-region-plan.json"
  ./camsim-boundary-plan \
    "$tmpdir/$name-region-plan.json" \
    "$tmpdir/$name-ownership-boundary.json"

  ./camsim \
    --threads 4 \
    --resolution 1 \
    "$tmpdir/$name.camotics" "$tmpdir/$name-baseline.stl"
  ./camsim \
    --sparse-toolpath \
    --sparse-toolpath-xy-bins 24 \
    --sparse-toolpath-halo-cells 1 \
    --threads 4 \
    --resolution 1 \
    --profile "$tmpdir/$name-profile.json" \
    "$tmpdir/$name.camotics" "$tmpdir/$name-sparse.stl"

  python3 scripts/perf/compare_stl_distance.py \
    --max-samples 200 \
    --cell-size 2 \
    --max-shells 20 \
    --hard-max-error 0.01 \
    --p99-error 0.01 \
    "$tmpdir/$name-baseline.stl" "$tmpdir/$name-sparse.stl"
done

for name in touching-holes nested-rings narrow-channel; do
  ./camsim-path --threads 4 --resolution 1 \
    "$tmpdir/$name.camotics" "$tmpdir/$name-toolpath.json"
  ./camsim-region-plan \
    --xy-bins 24 \
    --halo-cells 1 \
    "$tmpdir/$name-toolpath.json" "$tmpdir/$name-region-plan.json"
  ./camsim-boundary-plan \
    "$tmpdir/$name-region-plan.json" \
    "$tmpdir/$name-ownership-boundary.json"

  ./camsim \
    --threads 4 \
    --resolution 1 \
    "$tmpdir/$name.camotics" "$tmpdir/$name-baseline.stl"
  ./camsim \
    --sparse-toolpath \
    --sparse-toolpath-xy-bins 24 \
    --sparse-toolpath-halo-cells 1 \
    --threads 4 \
    --resolution 1 \
    --profile "$tmpdir/$name-profile.json" \
    "$tmpdir/$name.camotics" "$tmpdir/$name-sparse.stl"

  python3 scripts/perf/compare_stl_distance.py \
    --max-samples 200 \
    --cell-size 2 \
    --max-shells 20 \
    --hard-max-error 1.1 \
    --p99-error 1.1 \
    "$tmpdir/$name-baseline.stl" "$tmpdir/$name-sparse.stl"
done

python3 - "$tmpdir" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])


def load(name, suffix, key):
    with (root / f"{name}-{suffix}.json").open(encoding="utf-8") as f:
        return json.load(f)[key]


def enclosed_inactive_components(plan):
    bins = plan["xy-bins"]
    active = [[False] * bins for _ in range(bins)]
    for region in plan["active-region-list"]:
        for y in range(region["tile-y"],
                       region["tile-y"] + region["tile-height"]):
            for x in range(region["tile-x"],
                           region["tile-x"] + region["tile-width"]):
                active[y][x] = True

    seen = set()
    enclosed = 0
    for start_y in range(bins):
        for start_x in range(bins):
            if active[start_y][start_x] or (start_x, start_y) in seen:
                continue
            pending = [(start_x, start_y)]
            seen.add((start_x, start_y))
            touches_border = False
            while pending:
                x, y = pending.pop()
                touches_border |= x in (0, bins - 1) or y in (0, bins - 1)
                for nx, ny in ((x - 1, y), (x + 1, y),
                               (x, y - 1), (x, y + 1)):
                    if not (0 <= nx < bins and 0 <= ny < bins):
                        continue
                    if active[ny][nx] or (nx, ny) in seen:
                        continue
                    seen.add((nx, ny))
                    pending.append((nx, ny))
            if not touches_border:
                enclosed += 1
    return enclosed


multiple_plan = load("multiple-islands", "region-plan", "region-plan")
multiple_boundary = load(
    "multiple-islands", "ownership-boundary", "ownership-boundary"
)
if enclosed_inactive_components(multiple_plan) < 2:
    raise SystemExit("multiple-islands ownership did not preserve multiple "
                     "islands")
if len(multiple_plan["render-region-list"]) != 1:
    raise SystemExit("multiple-islands render grouping was not connected")
if (multiple_boundary["open-loops"] != 0 or
        multiple_boundary["ambiguous-vertices"] != 0):
    raise SystemExit("multiple-islands render boundary was not clean")

border_plan = load("border-touch", "region-plan", "region-plan")
border_boundary = load(
    "border-touch", "ownership-boundary", "ownership-boundary"
)
bins = border_plan["xy-bins"]
if not any(
    region["tile-x"] == 0 or
    region["tile-x"] + region["tile-width"] == bins or
    region["tile-y"] == 0 or
    region["tile-y"] + region["tile-height"] == bins
    for region in border_plan["active-region-list"]
):
    raise SystemExit("border-touch ownership did not reach the stock border")
if not any(loop["touches-stock-border"] for loop in border_boundary["loops"]):
    raise SystemExit("border-touch boundary did not report the stock border")
if (border_boundary["open-loops"] != 0 or
        border_boundary["ambiguous-vertices"] != 0):
    raise SystemExit("border-touch render boundary was not clean")

shape_components = {}
for name in ("touching-holes", "nested-rings", "narrow-channel"):
    plan = load(name, "region-plan", "region-plan")
    boundary = load(name, "ownership-boundary", "ownership-boundary")
    shape_components[name] = enclosed_inactive_components(plan)
    if not plan["active-region-list"]:
        raise SystemExit(name + " planner produced no active ownership")
    if boundary["open-loops"] != 0 or boundary["ambiguous-vertices"] != 0:
        raise SystemExit(name + " render boundary was not cleanly traced")

if shape_components["touching-holes"] < 1:
    raise SystemExit("touching-holes ownership preserved no enclosed region")
if shape_components["nested-rings"] < 1:
    raise SystemExit("nested-rings ownership preserved no enclosed region")
if shape_components["narrow-channel"] != 0:
    raise SystemExit("sub-cell narrow channel was not conservatively merged")

for name in ("multiple-islands", "border-touch"):
    with (root / f"{name}-profile.json").open(encoding="utf-8") as f:
        metrics = json.load(f).get("metrics", {})
    required = [
        "sparse_toolpath_integrated_enabled",
        "sparse_toolpath_integrated_full_coverage_fallback",
        "sparse_toolpath_integrated_topology_fallback",
        "sparse_toolpath_integrated_geometry_fallback",
        "sparse_stitch_topology_accepted",
    ]
    missing = [key for key in required if key not in metrics]
    if missing:
        raise SystemExit(name + " profile missing: " + ", ".join(missing))
    if metrics["sparse_toolpath_integrated_enabled"] != 1:
        raise SystemExit(name + " did not enable sparse mode")
    if metrics["sparse_toolpath_integrated_full_coverage_fallback"] != 0:
        raise SystemExit(name + " used the wrong planning fallback")
    if metrics["sparse_toolpath_integrated_topology_fallback"] != 1:
        raise SystemExit(name + " did not fail closed on rejected topology")
    if metrics["sparse_stitch_topology_accepted"] != 0:
        raise SystemExit(name + " unexpectedly accepted sparse topology")

for name in ("touching-holes", "nested-rings", "narrow-channel"):
    with (root / f"{name}-profile.json").open(encoding="utf-8") as f:
        metrics = json.load(f).get("metrics", {})
    required = [
        "sparse_toolpath_integrated_enabled",
        "sparse_toolpath_integrated_full_coverage_fallback",
        "sparse_toolpath_integrated_topology_fallback",
        "sparse_toolpath_integrated_geometry_fallback",
        "sparse_stitch_topology_accepted",
        "sparse_stitch_boundary_edges",
        "sparse_stitch_nonmanifold_edges",
        "sparse_stitch_misoriented_edges",
        "sparse_stitch_duplicate_triangles",
    ]
    missing = [key for key in required if key not in metrics]
    if missing:
        raise SystemExit(name + " profile missing: " + ", ".join(missing))
    if metrics["sparse_toolpath_integrated_enabled"] != 1:
        raise SystemExit(name + " did not enable sparse mode")
    if metrics["sparse_toolpath_integrated_full_coverage_fallback"] != 0:
        raise SystemExit(name + " unexpectedly used full-coverage fallback")
    topology_accepted = metrics["sparse_stitch_topology_accepted"] == 1
    topology_fallback = metrics[
        "sparse_toolpath_integrated_topology_fallback"
    ] == 1
    if topology_accepted == topology_fallback:
        raise SystemExit(name + " did not report accepted topology or its "
                         "explicit fallback consistently")
    if topology_accepted:
        for key in (
            "sparse_stitch_boundary_edges",
            "sparse_stitch_nonmanifold_edges",
            "sparse_stitch_misoriented_edges",
            "sparse_stitch_duplicate_triangles",
        ):
            if metrics[key] != 0:
                raise SystemExit(name + " accepted with nonzero " + key)
PY

cat > "$tmpdir/full-coverage.ngc" <<'NC'
G21
T1 M6
G0 Z2
G0 X-10 Y-7.5
G1 Z-5 F100
G1 X10 Y-7.5
G0 Z2
G0 X-10 Y-2.5
G1 Z-5 F100
G1 X10 Y-2.5
G0 Z2
G0 X-10 Y2.5
G1 Z-5 F100
G1 X10 Y2.5
G0 Z2
G0 X-10 Y7.5
G1 Z-5 F100
G1 X10 Y7.5
G0 Z2
M2
NC

cat > "$tmpdir/full-coverage.camotics" <<JSON
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 1,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 10,
      "diameter": 1,
      "description": ""
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {
      "min": [-10, -10, -4],
      "max": [10, 10, 0]
    }
  },
  "files": ["full-coverage.ngc"]
}
JSON

./camsim \
  --threads 4 \
  --resolution 1 \
  "$tmpdir/full-coverage.camotics" \
  "$tmpdir/full-coverage-baseline.stl"

./camsim \
  --sparse-toolpath \
  --sparse-toolpath-xy-bins 4 \
  --sparse-toolpath-halo-cells 1 \
  --threads 4 \
  --resolution 1 \
  --profile "$tmpdir/full-coverage-profile.json" \
  "$tmpdir/full-coverage.camotics" \
  "$tmpdir/full-coverage-sparse.stl"

python3 scripts/perf/compare_stl_distance.py \
  --max-samples 200 \
  --cell-size 2 \
  --max-shells 20 \
  --hard-max-error 0.01 \
  --p99-error 0.01 \
  "$tmpdir/full-coverage-baseline.stl" \
  "$tmpdir/full-coverage-sparse.stl"

python3 - "$tmpdir/full-coverage-profile.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as f:
    metrics = json.load(f).get("metrics", {})

required = [
    "sparse_toolpath_integrated_enabled",
    "sparse_toolpath_integrated_full_coverage_fallback",
    "sparse_toolpath_integrated_topology_fallback",
    "sparse_region_plan_full_cells_est",
    "sparse_region_plan_active_cells_est",
    "sparse_region_plan_render_cells_est",
    "sparse_region_plan_target_region_cells",
    "sparse_region_plan_adaptive_leaf_count",
    "sparse_region_plan_adaptive_active_leaf_count",
    "sparse_region_plan_adaptive_split_count",
    "sparse_region_plan_adaptive_max_leaf_cells",
    "sparse_region_plan_adaptive_target_exceeded_leaves",
]
missing = [name for name in required if name not in metrics]
if missing:
    raise SystemExit("full-coverage sparse profile missing metrics: " +
                     ", ".join(missing))

if metrics["sparse_toolpath_integrated_enabled"] != 1:
    raise SystemExit("full-coverage sparse mode did not enable")
if metrics["sparse_toolpath_integrated_full_coverage_fallback"] != 1:
    raise SystemExit("full-coverage sparse did not use planning fallback")
if metrics["sparse_toolpath_integrated_topology_fallback"] != 0:
    raise SystemExit("full-coverage sparse used topology fallback instead "
                     "of planning fallback")
if metrics["sparse_region_plan_active_cells_est"] <= 0:
    raise SystemExit("full-coverage planner did not report active cells")
if metrics["sparse_region_plan_render_cells_est"] <= 0:
    raise SystemExit("full-coverage planner did not report render cells")
if metrics["sparse_region_plan_adaptive_leaf_count"] <= 0:
    raise SystemExit("full-coverage planner emitted no adaptive leaves")
if metrics["sparse_region_plan_adaptive_active_leaf_count"] <= 0:
    raise SystemExit("full-coverage planner emitted no active leaves")
if metrics["sparse_region_plan_adaptive_target_exceeded_leaves"] != 0:
    raise SystemExit("full-coverage adaptive leaves exceeded the cell target")
if metrics["sparse_region_plan_adaptive_max_leaf_cells"] > metrics[
    "sparse_region_plan_target_region_cells"
]:
    raise SystemExit("full-coverage adaptive max leaf exceeded target")
PY

echo "sparse toolpath split and integrated pipeline smoke passed"
