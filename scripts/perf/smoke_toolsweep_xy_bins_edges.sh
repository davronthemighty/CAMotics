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
      "length": 10,
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

write_empty_gcode() {
  local name="$1"
  cat > "$tmpdir/$name.nc" <<'NC'
G21
F100
M3 S1000
M6 T1
NC
}

write_diagonal_gcode() {
  local name="$1"
  local end_xy="$2"
  cat > "$tmpdir/$name.nc" <<NC
G21
F100
M3 S1000
M6 T1
G0 X0 Y0 Z2
G1 Z-1
G1 X$end_xy Y$end_xy Z-1
G0 Z2
NC
}

make_cases() {
  write_empty_gcode empty_path
  write_project empty_path 1.0 "-2, -2, -3" "12, 12, 1"

  write_diagonal_gcode long_diagonal 120
  write_project long_diagonal 2.0 "-4, -4, -3" "124, 124, 1"

  write_diagonal_gcode tiny_envelope 2
  write_project tiny_envelope 0.1 "-0.2, -0.2, -1.5" "2.2, 2.2, 0.5"

  write_diagonal_gcode large_envelope 200
  write_project large_envelope 2.0 "-20, -20, -3" "220, 220, 1"

  write_diagonal_gcode partial_time 80
  write_project partial_time 2.0 "-4, -4, -3" "84, 84, 1"
}

run_case() {
  local name="$1"
  shift
  local args=("$@")
  local dir="$tmpdir/run-$name"
  mkdir -p "$dir"

  "$camsim" --threads 1 "${args[@]}" --profile "$dir/default.json" \
    "$tmpdir/$name.camotics" "$dir/default.stl"
  "$camsim" --threads 1 "${args[@]}" --toolsweep-xy-bins 64 \
    --profile "$dir/xy.json" "$tmpdir/$name.camotics" "$dir/xy.stl"
  "$camsim" --threads 1 "${args[@]}" --toolsweep-xyz-bins 16 \
    --profile "$dir/xyz.json" "$tmpdir/$name.camotics" "$dir/xyz.stl"

  python3 scripts/perf/compare_stl_geometry.py \
    "$dir/default.stl" "$dir/xy.stl"
  python3 scripts/perf/compare_stl_geometry.py \
    "$dir/default.stl" "$dir/xyz.stl"

  python3 - "$name" "$dir/default.json" "$dir/xy.json" "$dir/xyz.json" <<'PY'
import json
import sys

name, default_path, xy_path, xyz_path = sys.argv[1:5]
with open(default_path, encoding="utf-8") as f:
    default = json.load(f)
with open(xy_path, encoding="utf-8") as f:
    xy = json.load(f)
with open(xyz_path, encoding="utf-8") as f:
    xyz = json.load(f)

default_metrics = default.get("metrics", {})
xy_metrics = xy.get("metrics", {})
xyz_metrics = xyz.get("metrics", {})
default_counters = default.get("counters", {})
xy_counters = xy.get("counters", {})
xyz_counters = xyz.get("counters", {})

if default_metrics.get("surface_triangles") != xy_metrics.get("surface_triangles"):
    raise SystemExit(f"{name}: triangle count changed")
if default_metrics.get("surface_triangles") != xyz_metrics.get("surface_triangles"):
    raise SystemExit(f"{name}: XYZ triangle count changed")

if default_counters.get("toolsweep_collision_candidates") != xy_counters.get(
    "toolsweep_collision_candidates"
):
    raise SystemExit(f"{name}: collision candidate count changed")
if default_counters.get("toolsweep_collision_candidates") != xyz_counters.get(
    "toolsweep_collision_candidates"
):
    raise SystemExit(f"{name}: XYZ collision candidate count changed")

if xy_counters.get("aabb_node_visits") != 0:
    raise SystemExit(f"{name}: XY-bin path still visited AABB nodes")
if xyz_counters.get("aabb_node_visits") != 0:
    raise SystemExit(f"{name}: XYZ-bin path still visited AABB nodes")

if name == "empty_path":
    if xy_metrics.get("toolsweep_xy_bin_refs", 0):
        raise SystemExit(f"{name}: empty path created XY-bin refs")
else:
    if xy_metrics.get("toolsweep_xy_bin_count") != 64:
        raise SystemExit(f"{name}: missing XY-bin count")
    if not xy_metrics.get("toolsweep_xy_bin_refs"):
        raise SystemExit(f"{name}: missing XY-bin refs")

    for metric in [
        "toolsweep_xy_bin_entries",
        "toolsweep_xy_bins_used",
        "toolsweep_xy_bins_empty",
        "toolsweep_xy_bin_max_refs_per_bin",
        "toolsweep_xy_bin_refs_per_used_bin_x1000",
        "toolsweep_xy_bin_hot_bins_4x_avg",
        "toolsweep_xy_bin_hot_threshold",
        "toolsweep_xy_bin_max_box_x_span",
        "toolsweep_xy_bin_max_box_y_span",
        "toolsweep_xy_bin_max_box_refs",
        "toolsweep_xy_bin_refs_per_entry_x1000",
    ]:
        if metric not in xy_metrics:
            raise SystemExit(f"{name}: missing XY-bin metric {metric}")

    if xy_metrics["toolsweep_xy_bin_max_box_x_span"] > 64:
        raise SystemExit(f"{name}: max X span exceeds bin count")
    if xy_metrics["toolsweep_xy_bin_max_box_y_span"] > 64:
        raise SystemExit(f"{name}: max Y span exceeds bin count")
    if xy_metrics["toolsweep_xy_bin_max_box_refs"] > 64 * 64:
        raise SystemExit(f"{name}: max box refs exceed full bin grid")

    if xyz_metrics.get("toolsweep_xyz_bin_count") != 16:
        raise SystemExit(f"{name}: missing XYZ-bin count")
    if not xyz_metrics.get("toolsweep_xyz_bin_refs"):
        raise SystemExit(f"{name}: missing XYZ-bin refs")

    for metric in [
        "toolsweep_xyz_bin_entries",
        "toolsweep_xyz_bins_used",
        "toolsweep_xyz_bins_empty",
        "toolsweep_xyz_bin_max_refs_per_bin",
        "toolsweep_xyz_bin_refs_per_used_bin_x1000",
        "toolsweep_xyz_bin_hot_bins_4x_avg",
        "toolsweep_xyz_bin_hot_threshold",
        "toolsweep_xyz_bin_max_box_x_span",
        "toolsweep_xyz_bin_max_box_y_span",
        "toolsweep_xyz_bin_max_box_z_span",
        "toolsweep_xyz_bin_max_box_refs",
        "toolsweep_xyz_bin_refs_per_entry_x1000",
    ]:
        if metric not in xyz_metrics:
            raise SystemExit(f"{name}: missing XYZ-bin metric {metric}")

    if xyz_metrics["toolsweep_xyz_bin_max_box_x_span"] > 16:
        raise SystemExit(f"{name}: XYZ max X span exceeds bin count")
    if xyz_metrics["toolsweep_xyz_bin_max_box_y_span"] > 16:
        raise SystemExit(f"{name}: XYZ max Y span exceeds bin count")
    if xyz_metrics["toolsweep_xyz_bin_max_box_z_span"] > 16:
        raise SystemExit(f"{name}: XYZ max Z span exceeds bin count")
    if xyz_metrics["toolsweep_xyz_bin_max_box_refs"] > 16 * 16 * 16:
        raise SystemExit(f"{name}: XYZ max box refs exceed full bin grid")

for counter in [
    "toolsweep_xy_bin_queries",
    "toolsweep_xy_bin_out_of_bounds",
    "toolsweep_xy_bin_refs_scanned",
    "toolsweep_xy_bin_bbox_hits",
]:
    if counter not in xy_counters:
        raise SystemExit(f"{name}: missing XY-bin counter {counter}")

if xy_counters["toolsweep_xy_bin_queries"] != xy_counters[
    "toolsweep_depth_calls"
]:
    raise SystemExit(f"{name}: XY-bin query count changed")

if xy_counters["toolsweep_xy_bin_bbox_hits"] != xy_counters[
    "toolsweep_collision_candidates"
]:
    raise SystemExit(f"{name}: XY-bin bbox hits changed")

if xy_counters["toolsweep_xy_bin_bbox_hits"] > xy_counters[
    "toolsweep_xy_bin_refs_scanned"
]:
    raise SystemExit(f"{name}: XY-bin hits exceed scanned refs")

for counter in [
    "toolsweep_xyz_bin_queries",
    "toolsweep_xyz_bin_out_of_bounds",
    "toolsweep_xyz_bin_refs_scanned",
    "toolsweep_xyz_bin_bbox_hits",
]:
    if counter not in xyz_counters:
        raise SystemExit(f"{name}: missing XYZ-bin counter {counter}")

if xyz_counters["toolsweep_xyz_bin_queries"] != xyz_counters[
    "toolsweep_depth_calls"
]:
    raise SystemExit(f"{name}: XYZ-bin query count changed")

if xyz_counters["toolsweep_xyz_bin_bbox_hits"] != xyz_counters[
    "toolsweep_collision_candidates"
]:
    raise SystemExit(f"{name}: XYZ-bin bbox hits changed")

if xyz_counters["toolsweep_xyz_bin_bbox_hits"] > xyz_counters[
    "toolsweep_xyz_bin_refs_scanned"
]:
    raise SystemExit(f"{name}: XYZ-bin hits exceed scanned refs")

print(f"{name}: XY/XYZ-bin exact smoke passed")
PY
}

make_cases

run_case empty_path
run_case long_diagonal
run_case tiny_envelope
run_case large_envelope
run_case partial_time --time 1

echo "ToolSweep XY/XYZ-bin edge smoke passed"
