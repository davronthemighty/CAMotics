#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

camsim="${1:-./camsim}"
tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

write_project() {
  local name="$1"
  local resolution="$2"
  local min_bounds="$3"
  local max_bounds="$4"

  cat > "$tmpdir/$name.camotics" <<JSON
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": $resolution,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 12,
      "diameter": 1
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {
      "min": [$min_bounds],
      "max": [$max_bounds]
    }
  },
  "files": ["$name.nc"]
}
JSON
}

write_flat_gcode() {
  local name="$1"
  cat > "$tmpdir/$name.nc" <<'NC'
G21
F120
M3 S1000
M6 T1
G0 Z2
NC
}

write_detail_gcode() {
  local name="$1"
  cat > "$tmpdir/$name.nc" <<'NC'
G21
F120
M3 S1000
M6 T1
G0 X-1 Y-1 Z2
G1 Z-0.8
G1 X41 Y41 Z-0.8
G0 Z2
G0 X41 Y-1 Z2
G1 Z-0.5
G1 X-1 Y41 Z-0.5
G0 Z2
G0 X20 Y-2 Z2
G1 Z-0.7
G1 X20 Y42 Z-0.7
G0 Z2
G0 X-2 Y20 Z2
G1 Z-0.7
G1 X42 Y20 Z-0.7
G0 Z2
NC
}

write_wide_gcode() {
  local name="$1"
  cat > "$tmpdir/$name.nc" <<'NC'
G21
F120
M3 S1000
M6 T1
G0 X0 Y0 Z3
G1 Z-1
G1 X120 Y80 Z-1
G0 Z3
G0 X120 Y0 Z3
G1 Z-0.6
G1 X0 Y80 Z-0.6
G0 Z3
NC
}

write_prime_gcode() {
  local name="$1"
  cat > "$tmpdir/$name.nc" <<'NC'
G21
F90
M3 S1000
M6 T1
G0 X-1.1 Y-0.9 Z1.4
G1 Z-0.45
G1 X18.7 Y22.9 Z-0.45
G0 Z1.4
G0 X18.3 Y-0.7 Z1.4
G1 Z-0.9
G1 X-1.4 Y20.8 Z-0.9
G0 Z1.4
G0 X8.15 Y-1.2 Z1.4
G1 Z-0.65
G1 X8.15 Y24.2 Z-0.65
G0 Z1.4
NC
}

write_thin_z_gcode() {
  local name="$1"
  cat > "$tmpdir/$name.nc" <<'NC'
G21
F80
M3 S1000
M6 T1
G0 X-1 Y-1 Z0.6
G1 Z-0.18
G1 X19 Y19 Z-0.18
G0 Z0.6
G0 X19 Y-1 Z0.6
G1 Z-0.28
G1 X-1 Y19 Z-0.28
G0 Z0.6
NC
}

write_boundary_plane_gcode() {
  local name="$1"
  cat > "$tmpdir/$name.nc" <<'NC'
G21
F100
M3 S1000
M6 T1
G0 X12 Y-2 Z1
G1 Z-0.55
G1 X12 Y30 Z-0.55
G0 Z1
G0 X-2 Y12 Z1
G1 Z-0.55
G1 X30 Y12 Z-0.55
G0 Z1
G0 X-2 Y-2 Z1
G1 Z-0.75
G1 X30 Y30 Z-0.75
G0 Z1
NC
}

make_cases() {
  write_flat_gcode flat_stock
  write_project flat_stock 1.0 "-4, -4, -4" "44, 44, 2"

  write_detail_gcode seam_detail
  write_project seam_detail 1.0 "-4, -4, -4" "44, 44, 2"

  write_wide_gcode wide_detail
  write_project wide_detail 2.0 "-8, -8, -5" "128, 88, 3"

  write_prime_gcode prime_odd_detail
  write_project prime_odd_detail 0.7 "-3.5, -4.2, -2.8" "18.2, 23.6, 1.4"

  write_thin_z_gcode thin_z_detail
  write_project thin_z_detail 0.25 "-2, -2, -0.75" "20, 20, 0.25"

  write_boundary_plane_gcode boundary_plane_detail
  write_project boundary_plane_detail 0.5 "-4, -4, -3" "28, 28, 1"
}

run_case() {
  local name="$1"
  local dir="$tmpdir/run-$name"
  mkdir -p "$dir"

  local threads_list=(1 2 3 4 5 10 20)

  for threads in "${threads_list[@]}"; do
    "$camsim" --threads "$threads" --profile "$dir/t$threads.json" \
      "$tmpdir/$name.camotics" "$dir/t$threads.stl"
  done

  for threads in "${threads_list[@]}"; do
    if [ "$threads" = 1 ]; then continue; fi
    python3 scripts/perf/compare_stl_geometry.py \
      "$dir/t1.stl" "$dir/t$threads.stl"
  done

  python3 - "$name" "${threads_list[@]}" -- "$dir"/t*.json <<'PY'
import json
import sys

name = sys.argv[1]
sep = sys.argv.index("--")
threads = sys.argv[2:sep]
profiles = []
for path in sys.argv[sep + 1:]:
    with open(path, encoding="utf-8") as f:
        profiles.append(json.load(f))

if len(profiles) != len(threads):
    raise SystemExit(
        f"{name}: expected {len(threads)} profiles, got {len(profiles)}"
    )

triangles = [p.get("metrics", {}).get("surface_triangles") for p in profiles]
if len(set(triangles)) != 1:
    raise SystemExit(f"{name}: triangle counts differ: {triangles}")

for p in profiles:
    metrics = p.get("metrics", {})
    required = [
        "render_tree_cells",
        "render_partition_cells",
        "render_partition_extra_cells",
        "render_partition_missing_cells",
        "render_partition_clipped_cells",
        "render_partition_full_tree_coverage",
        "render_actual_jobs",
        "render_job_cells_min",
        "render_job_cells_max",
    ]
    missing_metrics = [metric for metric in required if metric not in metrics]
    if missing_metrics:
        raise SystemExit(
            f"{name}: missing partition metrics: {', '.join(missing_metrics)}"
        )

    tree_cells = metrics["render_tree_cells"]
    partition_cells = metrics["render_partition_cells"]
    extra = metrics.get("render_partition_extra_cells")
    if extra != 0:
        raise SystemExit(f"{name}: partition extra cells not zero: {extra}")

    missing = metrics["render_partition_missing_cells"]
    expected_missing = max(0, tree_cells - partition_cells)
    if missing != expected_missing:
        raise SystemExit(
            f"{name}: missing-cell metric {missing} != {expected_missing}"
        )

    clipped = metrics["render_partition_clipped_cells"]
    full_coverage = metrics["render_partition_full_tree_coverage"]
    if full_coverage and missing:
        raise SystemExit(f"{name}: full-coverage partition missed {missing}")
    if full_coverage and clipped:
        raise SystemExit(f"{name}: full-coverage partition clipped {clipped}")
    if not full_coverage and clipped != missing:
        raise SystemExit(
            f"{name}: clipped cells {clipped} != missing cells {missing}"
        )

    if metrics["render_job_cells_min"] > metrics["render_job_cells_max"]:
        raise SystemExit(f"{name}: job cell min exceeds max")

    if not metrics["render_actual_jobs"]:
        raise SystemExit(f"{name}: missing render_actual_jobs")

print(f"{name}: partition cross-thread exact smoke passed")
PY
}

make_cases

run_case flat_stock
run_case seam_detail
run_case wide_detail
run_case prime_odd_detail
run_case thin_z_detail
run_case boundary_plane_detail

echo "Partition cross-thread smoke passed"
