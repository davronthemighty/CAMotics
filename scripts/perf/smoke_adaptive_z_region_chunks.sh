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
      "length": 4,
      "diameter": 0.5
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

write_localized_deep_cut() {
  local name="$1"
  cat > "$tmpdir/$name.nc" <<'NC'
G21
F120
M3 S1000
M6 T1
G0 X-1 Y-1 Z1
G1 Z-0.25
G1 X1 Y-1 Z-0.25
G1 X1 Y1 Z-0.25
G1 X-1 Y1 Z-0.25
G1 X-1 Y-1 Z-0.25
G0 Z1
G0 X7 Y7 Z1
G1 Z-1.7
G1 X8 Y7 Z-1.7
G1 X8 Y8 Z-1.7
G1 X7 Y8 Z-1.7
G1 X7 Y7 Z-1.7
G0 Z1
NC
  write_project "$name" 0.25 "-10, -10, -4" "10, 10, 0"
}

write_localized_shallow() {
  local name="$1"
  cat > "$tmpdir/$name.nc" <<'NC'
G21
F120
M3 S1000
M6 T1
G0 X-1 Y-1 Z1
G1 Z-0.25
G1 X1 Y-1 Z-0.25
G1 X1 Y1 Z-0.25
G1 X-1 Y1 Z-0.25
G1 X-1 Y-1 Z-0.25
G0 Z1
NC
  write_project "$name" 0.25 "-10, -10, -4" "10, 10, 0"
}

write_wide_shallow() {
  local name="$1"
  cat > "$tmpdir/$name.nc" <<'NC'
G21
F120
M3 S1000
M6 T1
G0 X-8 Y-8 Z1
G1 Z-0.25
G1 X8 Y-8 Z-0.25
G1 X8 Y8 Z-0.25
G1 X-8 Y8 Z-0.25
G1 X-8 Y-8 Z-0.25
G0 Z1
G0 X-8 Y0 Z1
G1 Z-0.25
G1 X8 Y0 Z-0.25
G0 Z1
G0 X0 Y-8 Z1
G1 Z-0.25
G1 X0 Y8 Z-0.25
G0 Z1
NC
  write_project "$name" 0.25 "-10, -10, -4" "10, 10, 0"
}

write_boundary_crossing() {
  local name="$1"
  cat > "$tmpdir/$name.nc" <<'NC'
G21
F120
M3 S1000
M6 T1
G0 X-9 Y-9 Z1
G1 Z-0.5
G1 X9 Y9 Z-0.5
G0 Z1
G0 X-9 Y9 Z1
G1 Z-0.5
G1 X9 Y-9 Z-0.5
G0 Z1
G0 X0 Y-9 Z1
G1 Z-1.0
G1 X0 Y9 Z-1.0
G0 Z1
NC
  write_project "$name" 0.3 "-10, -10, -3" "10, 10, 0"
}

write_deep_step_boundary() {
  local name="$1"
  cat > "$tmpdir/$name.nc" <<'NC'
G21
F120
M3 S1000
M6 T1
G0 X-2 Y-7 Z1
G1 Z-0.25
G1 X-2 Y7 Z-0.25
G0 Z1
G0 X2 Y-7 Z1
G1 Z-1.75
G1 X2 Y7 Z-1.75
G0 Z1
NC
  write_project "$name" 0.25 "-10, -10, -4" "10, 10, 0"
}

write_corner_edge_cut() {
  local name="$1"
  cat > "$tmpdir/$name.nc" <<'NC'
G21
F120
M3 S1000
M6 T1
G0 X-10 Y-10 Z1
G1 Z-0.75
G1 X-7 Y-10 Z-0.75
G1 X-7 Y-7 Z-0.75
G1 X-10 Y-7 Z-0.75
G1 X-10 Y-10 Z-0.75
G0 Z1
NC
  write_project "$name" 0.25 "-10, -10, -4" "10, 10, 0"
}

write_full_depth_local() {
  local name="$1"
  cat > "$tmpdir/$name.nc" <<'NC'
G21
F120
M3 S1000
M6 T1
G0 X7 Y-1 Z1
G1 Z-3.8
G1 X8 Y-1 Z-3.8
G1 X8 Y1 Z-3.8
G1 X7 Y1 Z-3.8
G1 X7 Y-1 Z-3.8
G0 Z1
NC
  write_project "$name" 0.25 "-10, -10, -4" "10, 10, 0"
}

write_scaled_z_coin() {
  local name="$1"
  cat > "$tmpdir/$name.nc" <<'NC'
G21
F120
M3 S1000
M6 T1
G0 X-8 Y0 Z1
G1 Z-0.25
G2 X8 Y0 I8 J0 Z-0.25
G2 X-8 Y0 I-8 J0 Z-0.25
G0 Z1
G0 X5 Y0 Z1
G1 Z-1.5
G1 X6 Y0 Z-1.5
G0 Z1
NC
  write_project "$name" 0.25 "-12, -12, -4" "12, 12, 0"
}

profile_metric() {
  local json="$1"
  local metric="$2"
  python3 - "$json" "$metric" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as f:
    metrics = json.load(f).get("metrics", {})
print(metrics.get(sys.argv[2], ""))
PY
}

run_metric_exact_case() {
  local name="$1"
  local dir="$tmpdir/run-$name-metric"
  mkdir -p "$dir"

  "$camsim" --threads 1 --profile "$dir/base.json" \
    "$tmpdir/$name.camotics" "$dir/base.stl"
  "$camsim" --threads 1 --profile "$dir/region-metric.json" \
    --adaptive-z-slabs --adaptive-z-region-bins 64 \
    --adaptive-z-initial-depth 0.5 --adaptive-z-slab-height 0.5 \
    --adaptive-z-margin 0.25 \
    "$tmpdir/$name.camotics" "$dir/region-metric.stl"
  "$camsim" --threads 1 --profile "$dir/region-metric-xy64.json" \
    --adaptive-z-slabs --adaptive-z-region-bins 64 \
    --toolsweep-xy-bins 64 --adaptive-z-initial-depth 0.5 \
    --adaptive-z-slab-height 0.5 --adaptive-z-margin 0.25 \
    "$tmpdir/$name.camotics" "$dir/region-metric-xy64.stl"

  python3 scripts/perf/compare_stl_geometry.py \
    "$dir/base.stl" "$dir/region-metric.stl"
  python3 scripts/perf/compare_stl_geometry.py \
    "$dir/base.stl" "$dir/region-metric-xy64.stl"

  python3 - "$name" "$dir/region-metric.json" \
    "$dir/region-metric-xy64.json" <<'PY'
import json
import sys

name, path, xy_path = sys.argv[1:4]
with open(path, encoding="utf-8") as f:
    metrics = json.load(f).get("metrics", {})
with open(xy_path, encoding="utf-8") as f:
    xy_metrics = json.load(f).get("metrics", {})

required = [
    "adaptive_z_region_bins",
    "adaptive_z_region_count",
    "adaptive_z_region_touched_regions",
    "adaptive_z_region_active_cells_est",
    "adaptive_z_region_saved_cells_vs_full",
]
for metric in required:
    if metric not in metrics:
        raise SystemExit(f"{name}: missing regional metric {metric}")

if metrics["adaptive_z_region_bins"] != 64:
    raise SystemExit(f"{name}: expected 64 regional bins")
if metrics["adaptive_z_region_render_enabled"] != 0:
    raise SystemExit(f"{name}: metric-only mode enabled regional render")
if not metrics["adaptive_z_region_touched_regions"]:
    raise SystemExit(f"{name}: no regions were touched")
if xy_metrics.get("toolsweep_xy_bin_count") != 64:
    raise SystemExit(f"{name}: regional+XY64 run did not enable XY bins")
if xy_metrics.get("adaptive_z_region_active_cells_est") != metrics.get(
    "adaptive_z_region_active_cells_est"
):
    raise SystemExit(f"{name}: XY64 changed regional active-cell estimate")
if (
    name == "localized_deep_cut"
    and metrics.get("adaptive_z_region_active_cells_est", 0)
    >= metrics.get("adaptive_z_active_grid_cells_est", 0)
):
    raise SystemExit(f"{name}: regional active cells did not beat global")
PY
}

run_render_case() {
  local name="$1"
  local hard_max_error="$2"
  local p99_error="$3"
  local dir="$tmpdir/run-$name-render"
  mkdir -p "$dir"

  "$camsim" --threads 1 --profile "$dir/base.json" \
    "$tmpdir/$name.camotics" "$dir/base.stl"
  "$camsim" --threads 1 --profile "$dir/global.json" \
    --adaptive-z-slabs --adaptive-z-render \
    --adaptive-z-initial-depth 0.5 --adaptive-z-slab-height 0.5 \
    --adaptive-z-margin 0.25 \
    "$tmpdir/$name.camotics" "$dir/global.stl"
  "$camsim" --threads 1 --profile "$dir/region.json" \
    --adaptive-z-slabs --adaptive-z-region-bins 8 \
    --adaptive-z-region-render --adaptive-z-initial-depth 0.5 \
    --adaptive-z-slab-height 0.5 --adaptive-z-margin 0.25 \
    "$tmpdir/$name.camotics" "$dir/region.stl"

  python3 scripts/perf/compare_stl_distance.py \
    "$dir/base.stl" "$dir/region.stl" \
    --hard-max-error "$hard_max_error" --p99-error "$p99_error" \
    --max-samples 20000

  python3 - "$name" "$dir/base.json" "$dir/global.json" \
    "$dir/region.json" <<'PY'
import json
import sys

name, base_path, global_path, region_path = sys.argv[1:5]
with open(base_path, encoding="utf-8") as f:
    base = json.load(f).get("metrics", {})
with open(global_path, encoding="utf-8") as f:
    global_metrics = json.load(f).get("metrics", {})
with open(region_path, encoding="utf-8") as f:
    region = json.load(f).get("metrics", {})

if region.get("adaptive_z_region_render_enabled") != 1:
    raise SystemExit(f"{name}: regional render was not enabled")
if region.get("adaptive_z_region_reconstructed_triangles", 0) <= 2:
    raise SystemExit(f"{name}: missing regional stock reconstruction")
if region.get("adaptive_z_region_rendered_regions", 0) < 1:
    raise SystemExit(f"{name}: no regional render work was reported")
if region.get("surface_triangles", 0) >= base.get("surface_triangles", 0):
    raise SystemExit(f"{name}: regional render did not reduce triangles")
if region.get("adaptive_z_region_filter_seam_margin_microunits", 0) <= 0:
    raise SystemExit(f"{name}: regional render did not report seam margin")
if region.get("adaptive_z_region_halo_passthrough_enabled") != 1:
    raise SystemExit(f"{name}: regional render did not preserve halo triangles")
if (
    name in {"localized_deep_cut", "deep_step_boundary", "full_depth_local"}
    and region.get("adaptive_z_region_active_cells_est", 0)
    >= global_metrics.get("adaptive_z_active_grid_cells_est", 0)
):
    raise SystemExit(f"{name}: regional active cells did not beat global")
PY
}

run_boundary_threads() {
  local name="$1"
  local dir="$tmpdir/run-$name-threads"
  mkdir -p "$dir"

  "$camsim" --threads 1 "$tmpdir/$name.camotics" "$dir/base.stl"

  for threads in 1 10 20; do
    "$camsim" --threads "$threads" --profile "$dir/region-$threads.json" \
      --adaptive-z-slabs --adaptive-z-region-bins 8 \
      --adaptive-z-initial-depth 0.5 --adaptive-z-slab-height 0.5 \
      --adaptive-z-margin 0.25 \
      "$tmpdir/$name.camotics" "$dir/region-$threads.stl"
    if [ "$threads" = 1 ]; then
      python3 scripts/perf/compare_stl_geometry.py \
        "$dir/base.stl" "$dir/region-$threads.stl"
    else
      python3 scripts/perf/compare_stl_distance.py \
        "$dir/base.stl" "$dir/region-$threads.stl" \
        --hard-max-error 0.45 --p99-error 0.32 --max-samples 20000
    fi
  done
}

write_localized_deep_cut localized_deep_cut
write_localized_shallow localized_shallow
write_wide_shallow wide_shallow
write_boundary_crossing boundary_crossing
write_deep_step_boundary deep_step_boundary
write_corner_edge_cut corner_edge_cut
write_full_depth_local full_depth_local
write_scaled_z_coin scaled_z_coin

run_metric_exact_case localized_deep_cut
run_render_case localized_shallow 0.38 0.27
run_render_case localized_deep_cut 0.38 0.27
run_render_case deep_step_boundary 0.38 0.27
run_render_case corner_edge_cut 0.38 0.27
run_render_case full_depth_local 0.38 0.27
run_render_case scaled_z_coin 0.38 0.27
run_metric_exact_case wide_shallow
run_boundary_threads boundary_crossing

echo "adaptive Z regional chunk smoke passed"
