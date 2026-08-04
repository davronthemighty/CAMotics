#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

for exe in camsim-path camsim-region-plan camsim-boundary-plan \
  camsim-render-regions camsim-stitch-stock camsim-reduce-export; do
  [ -x "./$exe" ] || { echo "$exe executable not found" >&2; exit 2; }
done

tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

cat > "$tmpdir/fixture.ngc" <<'NC'
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

cat > "$tmpdir/fixture.camotics" <<JSON
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 0.5,
  "tools": {"1": {"units": "metric", "shape": "cylindrical",
    "length": 20, "diameter": 2, "description": ""}},
  "workpiece": {"automatic": false, "margin": 0,
    "bounds": {"min": [-20, -20, -10], "max": [20, 20, 0]}},
  "files": ["fixture.ngc"]
}
JSON

./camsim-path --threads 4 --resolution 0.5 \
  "$tmpdir/fixture.camotics" "$tmpdir/toolpath.json"
./camsim-region-plan --xy-bins 16 --halo-cells 1 \
  "$tmpdir/toolpath.json" "$tmpdir/region-plan.json"
./camsim-boundary-plan "$tmpdir/region-plan.json" "$tmpdir/boundary.json"
./camsim-render-regions --threads 4 "$tmpdir/toolpath.json" \
  "$tmpdir/region-plan.json" "$tmpdir/region-surface.json"
./camsim-stitch-stock --ownership-boundary "$tmpdir/boundary.json" \
  "$tmpdir/region-plan.json" "$tmpdir/region-surface.json" \
  "$tmpdir/stitched.json"

run_mode() {
  local mode="$1"
  shift
  : > "$tmpdir/$mode-times.txt"
  for iteration in $(seq 1 31); do
    /usr/bin/time -f '%U %e %M' -o "$tmpdir/time.txt" \
      ./camsim-reduce-export --safe-reduce "$@" \
      --profile "$tmpdir/$mode-$iteration.json" \
      "$tmpdir/stitched.json" "$tmpdir/$mode-$iteration.stl"
    cat "$tmpdir/time.txt" >> "$tmpdir/$mode-times.txt"
  done
}

run_mode filtered
run_mode whole --safe-reduce-whole-surface-reference

python3 - "$tmpdir" <<'PY'
import json
import pathlib
import statistics
import sys

root = pathlib.Path(sys.argv[1])

def load_mode(name):
    rows = [tuple(map(float, line.split())) for line in
            (root / f"{name}-times.txt").read_text().splitlines()]
    profile = json.load(open(root / f"{name}-31.json", encoding="utf-8"))
    metrics = profile["metrics"]
    return {
        "user_cpu_seconds_median": statistics.median(row[0] for row in rows),
        "wall_seconds_median": statistics.median(row[1] for row in rows),
        "peak_rss_kib_median": int(statistics.median(row[2] for row in rows)),
        "peak_rss_kib_mean": round(statistics.mean(row[2] for row in rows), 1),
        "user_cpu_seconds_mean": round(
            statistics.mean(row[0] for row in rows), 4),
        "analysis_triangle_records": metrics[
            "safe_reduce_analysis_triangle_records"],
        "analysis_adjacency_slots": metrics[
            "safe_reduce_analysis_adjacency_slots"],
        "analysis_edge_records": metrics["safe_reduce_analysis_edge_records"],
        "component_records": metrics["safe_reduce_components"],
        "candidate_triangles": metrics[
            "safe_reduce_estimated_triangles_after"],
        "candidate_reduction": metrics[
            "safe_reduce_estimated_triangle_reduction"],
        "output_triangles": metrics["safe_reduce_output_triangles"],
        "applied_source_triangles": metrics[
            "safe_reduce_applied_source_triangles"],
        "applied_output_triangles": metrics[
            "safe_reduce_applied_output_triangles"],
        "binary_stl_bytes": (root / f"{name}-31.stl").stat().st_size,
    }

filtered = load_mode("filtered")
whole = load_mode("whole")
if filtered["analysis_triangle_records"] >= whole["analysis_triangle_records"]:
    raise SystemExit("filtered analysis did not admit fewer triangles")
if filtered["analysis_adjacency_slots"] >= whole["analysis_adjacency_slots"]:
    raise SystemExit("filtered analysis did not allocate fewer adjacency slots")
if not (filtered["user_cpu_seconds_mean"] < whole["user_cpu_seconds_mean"] or
        filtered["peak_rss_kib_mean"] < whole["peak_rss_kib_mean"]):
    raise SystemExit("filtered analysis improved neither CPU nor peak RSS")
result = {"filtered": filtered, "whole_surface_reference": whole}
print(json.dumps(result, indent=2, sort_keys=True))
PY

echo "sparse safe-reduce eligibility benchmark passed"
