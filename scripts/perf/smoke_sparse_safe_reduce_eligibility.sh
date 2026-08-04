#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

if [ ! -x ./camsim ]; then
  echo "camsim executable not found; build it in WSL first" >&2
  exit 2
fi

tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

cat > "$tmpdir/accepted.ngc" <<'NC'
G21
T1 M6
G0 Z2
G0 X-6 Y-6
G1 Z-1 F100
G1 X6 Y-6
G0 Z2
G0 X-6 Y-2
G1 Z-1 F100
G1 X6 Y-2
G0 Z2
G0 X-6 Y2
G1 Z-1 F100
G1 X6 Y2
G0 Z2
G0 X-6 Y6
G1 Z-1 F100
G1 X6 Y6
G0 Z2
M2
NC

cat > "$tmpdir/accepted.camotics" <<JSON
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 0.5,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 20,
      "diameter": 2,
      "description": ""
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {
      "min": [-20, -20, -10],
      "max": [20, 20, 0]
    }
  },
  "files": ["accepted.ngc"]
}
JSON

cat > "$tmpdir/x-cut.ngc" <<'NC'
G21
T1 M6
G0 Z2
G0 X-16 Y-16
G1 Z-1 F100
G1 X16 Y16
G0 Z2
G0 X-16 Y16
G1 Z-1 F100
G1 X16 Y-16
G0 Z2
M2
NC

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

python3 - "$tmpdir" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
fixtures = {
    "x-cut": ([-20, -20, -5], [20, 20, 0], 1),
    "island-ring": ([-20, -20, -5], [20, 20, 0], 2),
    "mixed-depth": ([-20, -10, -10], [20, 10, 0], 1),
}
for name, (minimum, maximum, diameter) in fixtures.items():
    project = {
        "units": "metric",
        "resolution-mode": "manual",
        "resolution": 0.5,
        "tools": {"1": {"units": "metric", "shape": "cylindrical",
                         "length": 20, "diameter": diameter,
                         "description": ""}},
        "workpiece": {"automatic": False, "margin": 0,
                      "bounds": {"min": minimum, "max": maximum}},
        "files": [f"{name}.ngc"],
    }
    (root / f"{name}.camotics").write_text(json.dumps(project),
                                             encoding="utf-8")
PY

common=(
  --binary
  --sparse-toolpath
  --sparse-toolpath-xy-bins 16
  --sparse-toolpath-halo-cells 1
  --threads 4
  --resolution 0.5
)

./camsim "${common[@]}" \
  --profile "$tmpdir/unreduced.json" \
  "$tmpdir/accepted.camotics" "$tmpdir/unreduced.stl"

for fixture in x-cut island-ring mixed-depth; do
  ./camsim "${common[@]}" \
    --profile "$tmpdir/$fixture-unreduced.json" \
    "$tmpdir/$fixture.camotics" "$tmpdir/$fixture-unreduced.stl"
  ./camsim "${common[@]}" --safe-reduce \
    --profile "$tmpdir/$fixture-apply.json" \
    "$tmpdir/$fixture.camotics" "$tmpdir/$fixture-apply.stl"
  python3 scripts/perf/compare_stl_distance.py \
    --max-samples 500 --cell-size 1 --max-shells 20 \
    --hard-max-error 0.6 --p99-error 0.6 \
    "$tmpdir/$fixture-unreduced.stl" "$tmpdir/$fixture-apply.stl"
done

declare -A mode_args
mode_args[apply]='--safe-reduce'
mode_args[report]='--safe-reduce-report'
mode_args[provenance]='--safe-reduce --safe-reduce-provenance-neighbors'
mode_args[trusted]='--safe-reduce --safe-reduce-provenance-neighbors --safe-reduce-trust-provenance-neighbors'
mode_args[hole]='--safe-reduce --safe-reduce-hole-aware'
mode_args[cosimplify]='--safe-reduce --safe-reduce-boundary-cosimplify'

for mode in apply report provenance trusted hole cosimplify; do
  read -r -a extra <<<"${mode_args[$mode]}"
  ./camsim "${common[@]}" "${extra[@]}" \
    --profile "$tmpdir/$mode.json" \
    "$tmpdir/accepted.camotics" "$tmpdir/$mode.stl"

  python3 scripts/perf/compare_stl_distance.py \
    --max-samples 500 \
    --cell-size 1 \
    --max-shells 20 \
    --hard-max-error 0.6 \
    --p99-error 0.6 \
    "$tmpdir/unreduced.stl" "$tmpdir/$mode.stl"
done

python3 - "$tmpdir" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
modes = ("apply", "report", "provenance", "trusted", "hole", "cosimplify")
zero_metrics = (
    "safe_reduce_analyzed_analytic_triangles",
    "safe_reduce_analytic_adjacency_insertions",
    "safe_reduce_analytic_component_memberships",
    "safe_reduce_analytic_replacement_triangles",
    "safe_reduce_unknown_locked_triangles",
)
required = (
    "safe_reduce_sparse_eligibility_requested",
    "safe_reduce_origin_metadata_valid",
    "safe_reduce_sparse_metadata_fallback",
    "safe_reduce_mc_reducible_triangles",
    "safe_reduce_mc_seam_locked_triangles",
    "safe_reduce_analytic_locked_triangles",
    "safe_reduce_locked_seam_vertices",
    "safe_reduce_locked_seam_edges",
    "safe_reduce_analytic_identity_preserved",
    "safe_reduce_locked_seams_preserved",
    "safe_reduce_whole_surface_validation_checked",
    "safe_reduce_whole_surface_validation_accepted",
    "safe_reduce_input_triangles",
    "safe_reduce_output_triangles",
    "safe_reduce_applied_source_triangles",
    "safe_reduce_applied_output_triangles",
    "safe_reduce_validation_rolled_back",
    "safe_reduce_output_boundary_edges",
    "safe_reduce_output_nonmanifold_edges",
    "safe_reduce_output_misoriented_edges",
    "safe_reduce_output_duplicate_triangles",
    "surface_triangles",
) + zero_metrics

with (root / "unreduced.json").open(encoding="utf-8") as f:
    unreduced = json.load(f).get("metrics", {})

for mode in modes:
    with (root / f"{mode}.json").open(encoding="utf-8") as f:
        metrics = json.load(f).get("metrics", {})
    missing = [key for key in required if key not in metrics]
    if missing:
        raise SystemExit(mode + " missing metrics: " + ", ".join(missing))
    for key in zero_metrics:
        if metrics[key] != 0:
            raise SystemExit(f"{mode}: {key} must be zero")
    for key in (
        "safe_reduce_sparse_eligibility_requested",
        "safe_reduce_origin_metadata_valid",
        "safe_reduce_analytic_identity_preserved",
        "safe_reduce_locked_seams_preserved",
        "safe_reduce_whole_surface_validation_checked",
        "safe_reduce_whole_surface_validation_accepted",
    ):
        if metrics[key] != 1:
            raise SystemExit(f"{mode}: {key} must be one")
    if metrics["safe_reduce_sparse_metadata_fallback"] != 0:
        raise SystemExit(mode + ": valid metadata unexpectedly fell back")
    if metrics["safe_reduce_mc_reducible_triangles"] <= 0:
        raise SystemExit(mode + ": no reducible MC triangles")
    if metrics["safe_reduce_mc_seam_locked_triangles"] <= 0:
        raise SystemExit(mode + ": no seam-locked MC triangles")
    if metrics["safe_reduce_analytic_locked_triangles"] != unreduced[
        "sparse_stitch_analytic_triangles"
    ]:
        raise SystemExit(mode + ": analytic triangle identity count changed")
    if metrics["safe_reduce_locked_seam_vertices"] <= 0:
        raise SystemExit(mode + ": no locked seam vertices")
    if metrics["safe_reduce_locked_seam_edges"] <= 0:
        raise SystemExit(mode + ": no locked seam edges")
    if (metrics["safe_reduce_mc_reducible_triangles"] +
            metrics["safe_reduce_mc_seam_locked_triangles"] +
            metrics["safe_reduce_analytic_locked_triangles"] !=
            metrics["safe_reduce_input_triangles"]):
        raise SystemExit(mode + ": origin counts do not cover input")
    if metrics["safe_reduce_output_triangles"] != metrics["surface_triangles"]:
        raise SystemExit(mode + ": reducer output does not match final surface")
    if metrics["safe_reduce_validation_rolled_back"] != 0:
        raise SystemExit(mode + ": filtered reduction rolled back")
    for key in (
        "safe_reduce_output_boundary_edges",
        "safe_reduce_output_nonmanifold_edges",
        "safe_reduce_output_misoriented_edges",
        "safe_reduce_output_duplicate_triangles",
    ):
        if metrics[key] != 0:
            raise SystemExit(f"{mode}: final whole surface has nonzero {key}")

    if mode == "report":
        if metrics["safe_reduce_output_triangles"] != metrics[
            "safe_reduce_input_triangles"
        ]:
            raise SystemExit("report mode changed the surface")
    elif not (metrics["safe_reduce_applied_output_triangles"] <
              metrics["safe_reduce_applied_source_triangles"]):
        raise SystemExit(mode + ": eligible MC reduction did not apply")

    if mode in ("provenance", "trusted") and metrics[
        "safe_reduce_provenance_neighbors_requested"
    ] != 1:
        raise SystemExit(mode + ": provenance option was not exercised")
    if mode == "trusted" and metrics[
        "safe_reduce_trust_provenance_neighbors_requested"
    ] != 1:
        raise SystemExit("trusted provenance option was not exercised")
    if mode == "hole" and metrics[
        "safe_reduce_hole_aware_apply_requested"
    ] != 1:
        raise SystemExit("hole-aware option was not exercised")
    if mode == "cosimplify" and metrics[
        "safe_reduce_boundary_cosimplify_apply_requested"
    ] != 1:
        raise SystemExit("boundary co-simplify option was not exercised")

if (root / "apply.stl").stat().st_size >= (root / "unreduced.stl").stat().st_size:
    raise SystemExit("filtered sparse reduction did not reduce STL bytes")

for fixture in ("x-cut", "island-ring", "mixed-depth"):
    with (root / f"{fixture}-apply.json").open(encoding="utf-8") as f:
        metrics = json.load(f).get("metrics", {})
    for key in required:
        if key not in metrics:
            raise SystemExit(f"{fixture}: missing metric {key}")
    for key in zero_metrics:
        if metrics[key] != 0:
            raise SystemExit(f"{fixture}: {key} must be zero")
    for key in (
        "safe_reduce_origin_metadata_valid",
        "safe_reduce_analytic_identity_preserved",
        "safe_reduce_locked_seams_preserved",
        "safe_reduce_whole_surface_validation_checked",
        "safe_reduce_whole_surface_validation_accepted",
    ):
        if metrics[key] != 1:
            raise SystemExit(f"{fixture}: {key} must be one")
    if metrics["safe_reduce_sparse_metadata_fallback"] != 0:
        raise SystemExit(fixture + ": valid metadata fell back")
    if metrics["safe_reduce_applied_output_triangles"] >= metrics[
            "safe_reduce_applied_source_triangles"]:
        raise SystemExit(fixture + ": eligible MC reduction did not apply")
    for key in (
        "safe_reduce_output_boundary_edges",
        "safe_reduce_output_nonmanifold_edges",
        "safe_reduce_output_misoriented_edges",
        "safe_reduce_output_duplicate_triangles",
    ):
        if metrics[key] != 0:
            raise SystemExit(f"{fixture}: final topology failed: {key}")
PY

echo "sparse safe-reduce eligibility smoke passed"
