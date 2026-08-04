#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

camsim="${1:-./camsim}"
tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

"$camsim" --threads 1 examples/cat/cat.camotics "$tmpdir/cat-a.stl"
"$camsim" --threads 1 --profile "$tmpdir/cat-profile.json" \
  examples/cat/cat.camotics "$tmpdir/cat-b.stl"
"$camsim" --threads 2 examples/cat/cat.camotics "$tmpdir/cat-t2.stl"
"$camsim" --threads 1 --toolsweep-xy-bins 64 \
  --profile "$tmpdir/cat-xy-profile.json" \
  examples/cat/cat.camotics "$tmpdir/cat-xy.stl"
"$camsim" --threads 1 --toolsweep-xyz-bins 16 \
  --profile "$tmpdir/cat-xyz-profile.json" \
  examples/cat/cat.camotics "$tmpdir/cat-xyz.stl"

python3 scripts/perf/compare_stl_geometry.py \
  "$tmpdir/cat-a.stl" "$tmpdir/cat-b.stl"
python3 scripts/perf/compare_stl_geometry.py \
  "$tmpdir/cat-a.stl" "$tmpdir/cat-t2.stl"
python3 scripts/perf/compare_stl_geometry.py \
  "$tmpdir/cat-a.stl" "$tmpdir/cat-xy.stl"
python3 scripts/perf/compare_stl_geometry.py \
  "$tmpdir/cat-a.stl" "$tmpdir/cat-xyz.stl"

python3 - "$tmpdir/cat-profile.json" "$tmpdir/cat-xy-profile.json" \
  "$tmpdir/cat-xyz-profile.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as f:
    profile = json.load(f)

counters = profile.get("counters", {})
depth_calls = counters.get("toolsweep_depth_calls")
if depth_calls is None:
    raise SystemExit("missing toolsweep_depth_calls")

buckets = [
    "toolsweep_candidate_calls_0",
    "toolsweep_candidate_calls_1",
    "toolsweep_candidate_calls_2_9",
    "toolsweep_candidate_calls_10_99",
    "toolsweep_candidate_calls_100_999",
    "toolsweep_candidate_calls_1000_plus",
]
missing = [name for name in buckets if name not in counters]
if missing:
    raise SystemExit("missing ToolSweep candidate buckets: " + ", ".join(missing))

bucket_sum = sum(counters[name] for name in buckets)
if bucket_sum != depth_calls:
    raise SystemExit(
        f"candidate bucket sum {bucket_sum} != depth calls {depth_calls}"
    )

triangles = profile.get("metrics", {}).get("surface_triangles")
if triangles != 191664:
    raise SystemExit(f"unexpected cat triangle count: {triangles}")

provenance_triangles = (
    counters.get("contour_grid_edge_triangles", 0)
    + counters.get("contour_cell_center_triangles", 0)
)
if provenance_triangles != triangles:
    raise SystemExit(
        f"contour provenance counters {provenance_triangles} "
        f"!= surface triangles {triangles}"
    )

for name in [
    "surface_provenance_expected_triangles",
    "surface_provenance_triangles",
    "surface_provenance_complete_triangles",
    "surface_provenance_unknown_triangles",
    "surface_provenance_grid_edge_triangles",
    "surface_provenance_cell_center_triangles",
    "surface_provenance_raw_boundary_edges",
    "surface_provenance_raw_nonmanifold_edges",
    "surface_provenance_raw_unique_edges",
    "surface_provenance_raw_max_edge_incidence",
    "surface_provenance_raw_edges_incidence_1",
    "surface_provenance_raw_edges_incidence_2",
    "surface_provenance_raw_edges_incidence_3",
    "surface_provenance_raw_edges_incidence_4",
    "surface_provenance_raw_edges_incidence_5_plus",
    "surface_provenance_raw_twin_edge_slots",
    "surface_provenance_raw_boundary_edge_slots",
    "surface_provenance_raw_nonmanifold_edge_slots",
    "surface_provenance_raw_grid_grid_unique_edges",
    "surface_provenance_raw_grid_grid_max_edge_incidence",
    "surface_provenance_raw_grid_grid_twin_edge_slots",
    "surface_provenance_raw_grid_grid_boundary_edge_slots",
    "surface_provenance_raw_grid_grid_nonmanifold_edge_slots",
    "surface_provenance_raw_grid_grid_welded_spread_edges",
    "surface_provenance_raw_grid_grid_welded_spread_edge_slots",
    "surface_provenance_raw_grid_grid_welded_spread_max_alternate_slots",
    "surface_provenance_raw_center_involved_unique_edges",
    "surface_provenance_raw_center_involved_max_edge_incidence",
    "surface_provenance_raw_center_involved_twin_edge_slots",
    "surface_provenance_raw_center_involved_boundary_edge_slots",
    "surface_provenance_raw_center_involved_nonmanifold_edge_slots",
    "surface_provenance_raw_center_involved_welded_spread_edges",
    "surface_provenance_raw_center_involved_welded_spread_edge_slots",
    "surface_provenance_raw_center_involved_welded_spread_max_alternate_slots",
    "surface_provenance_raw_grid_vertex_unique_keys",
    "surface_provenance_raw_grid_vertex_welded_spread_keys",
    "surface_provenance_raw_grid_vertex_welded_spread_observations",
    "surface_provenance_raw_grid_vertex_welded_spread_max_alternate_observations",
    "surface_provenance_raw_center_vertex_unique_keys",
    "surface_provenance_raw_center_vertex_welded_spread_keys",
    "surface_provenance_raw_center_vertex_welded_spread_observations",
    "surface_provenance_raw_center_vertex_welded_spread_max_alternate_observations",
    "surface_provenance_boundary_edges",
    "surface_provenance_nonmanifold_edges",
    "surface_provenance_welded_unique_edges",
    "surface_provenance_welded_max_edge_incidence",
    "surface_provenance_welded_edges_incidence_1",
    "surface_provenance_welded_edges_incidence_2",
    "surface_provenance_welded_edges_incidence_3",
    "surface_provenance_welded_edges_incidence_4",
    "surface_provenance_welded_edges_incidence_5_plus",
    "surface_provenance_welded_twin_edge_slots",
    "surface_provenance_welded_boundary_edge_slots",
    "surface_provenance_welded_nonmanifold_edge_slots",
    "surface_provenance_watertight",
]:
    if name not in profile.get("metrics", {}):
        raise SystemExit(f"missing contour provenance metric: {name}")

if profile["metrics"]["surface_provenance_expected_triangles"] != triangles:
    raise SystemExit("contour provenance expected triangle count mismatch")
if profile["metrics"]["surface_provenance_triangles"] != triangles:
    raise SystemExit("contour provenance record count mismatch")
if profile["metrics"]["surface_provenance_complete_triangles"] != triangles:
    raise SystemExit("contour provenance is incomplete")
if profile["metrics"]["surface_provenance_unknown_triangles"] != 0:
    raise SystemExit("contour provenance reported unknown triangles")
if profile["metrics"]["surface_provenance_grid_edge_triangles"] != triangles:
    raise SystemExit("cat contour provenance should be all grid-edge triangles")
if profile["metrics"]["surface_provenance_cell_center_triangles"] != 0:
    raise SystemExit("cat contour provenance should have no cell-center triangles")
if profile["metrics"]["surface_provenance_raw_grid_grid_twin_edge_slots"] != profile["metrics"]["surface_provenance_raw_twin_edge_slots"]:
    raise SystemExit("cat raw grid-grid twin slots should equal total raw twin slots")
if profile["metrics"]["surface_provenance_raw_grid_grid_boundary_edge_slots"] != profile["metrics"]["surface_provenance_raw_boundary_edge_slots"]:
    raise SystemExit("cat raw grid-grid boundary slots should equal total raw boundary slots")
if profile["metrics"]["surface_provenance_raw_grid_grid_nonmanifold_edge_slots"] != profile["metrics"]["surface_provenance_raw_nonmanifold_edge_slots"]:
    raise SystemExit("cat raw grid-grid nonmanifold slots should equal total raw nonmanifold slots")
if profile["metrics"]["surface_provenance_raw_center_involved_twin_edge_slots"] != 0:
    raise SystemExit("cat raw center-involved twin slots should be zero")
if profile["metrics"]["surface_provenance_raw_center_involved_boundary_edge_slots"] != 0:
    raise SystemExit("cat raw center-involved boundary slots should be zero")
if profile["metrics"]["surface_provenance_raw_center_involved_nonmanifold_edge_slots"] != 0:
    raise SystemExit("cat raw center-involved nonmanifold slots should be zero")
if profile["metrics"]["surface_provenance_raw_center_involved_welded_spread_edges"] != 0:
    raise SystemExit("cat raw center-involved spread edges should be zero")
if profile["metrics"]["surface_provenance_raw_center_involved_welded_spread_edge_slots"] != 0:
    raise SystemExit("cat raw center-involved spread slots should be zero")
if profile["metrics"]["surface_provenance_raw_center_involved_welded_spread_max_alternate_slots"] != 0:
    raise SystemExit("cat raw center-involved max spread should be zero")
if profile["metrics"]["surface_provenance_raw_center_vertex_unique_keys"] != 0:
    raise SystemExit("cat raw center vertex keys should be zero")
if profile["metrics"]["surface_provenance_raw_center_vertex_welded_spread_keys"] != 0:
    raise SystemExit("cat raw center vertex spread keys should be zero")
if profile["metrics"]["surface_provenance_raw_center_vertex_welded_spread_observations"] != 0:
    raise SystemExit("cat raw center vertex spread observations should be zero")
if profile["metrics"]["surface_provenance_raw_center_vertex_welded_spread_max_alternate_observations"] != 0:
    raise SystemExit("cat raw center vertex max spread should be zero")
if profile["metrics"]["surface_provenance_boundary_edges"] != 0:
    raise SystemExit("contour provenance reported boundary edges")
if profile["metrics"]["surface_provenance_nonmanifold_edges"] != 0:
    raise SystemExit("contour provenance reported nonmanifold edges")
surface_edge_slots = triangles * 3
if profile["metrics"]["surface_provenance_welded_unique_edges"] * 2 != surface_edge_slots:
    raise SystemExit("contour provenance welded unique edge count mismatch")
if profile["metrics"]["surface_provenance_welded_max_edge_incidence"] != 2:
    raise SystemExit("contour provenance welded max edge incidence mismatch")
if profile["metrics"]["surface_provenance_welded_edges_incidence_1"] != 0:
    raise SystemExit("contour provenance welded incidence-1 edges mismatch")
if profile["metrics"]["surface_provenance_welded_edges_incidence_2"] != profile["metrics"]["surface_provenance_welded_unique_edges"]:
    raise SystemExit("contour provenance welded incidence-2 edges mismatch")
if profile["metrics"]["surface_provenance_welded_edges_incidence_3"] != 0:
    raise SystemExit("contour provenance welded incidence-3 edges mismatch")
if profile["metrics"]["surface_provenance_welded_edges_incidence_4"] != 0:
    raise SystemExit("contour provenance welded incidence-4 edges mismatch")
if profile["metrics"]["surface_provenance_welded_edges_incidence_5_plus"] != 0:
    raise SystemExit("contour provenance welded incidence-5+ edges mismatch")
if profile["metrics"]["surface_provenance_welded_twin_edge_slots"] != surface_edge_slots:
    raise SystemExit("contour provenance welded twin slots mismatch")
if profile["metrics"]["surface_provenance_welded_boundary_edge_slots"] != 0:
    raise SystemExit("contour provenance reported welded boundary edge slots")
if profile["metrics"]["surface_provenance_welded_nonmanifold_edge_slots"] != 0:
    raise SystemExit("contour provenance reported welded nonmanifold edge slots")
if profile["metrics"]["surface_provenance_watertight"] != 1:
    raise SystemExit("contour provenance should report watertight topology")

with open(sys.argv[2], encoding="utf-8") as f:
    xy_profile = json.load(f)
with open(sys.argv[3], encoding="utf-8") as f:
    xyz_profile = json.load(f)

xy_metrics = xy_profile.get("metrics", {})
xy_counters = xy_profile.get("counters", {})
xyz_metrics = xyz_profile.get("metrics", {})
xyz_counters = xyz_profile.get("counters", {})
if xy_metrics.get("surface_triangles") != 191664:
    raise SystemExit(
        f"unexpected XY-bin cat triangle count: "
        f"{xy_metrics.get('surface_triangles')}"
    )

if xy_metrics.get("toolsweep_xy_bin_count") != 64:
    raise SystemExit("missing or unexpected toolsweep_xy_bin_count")

if not xy_metrics.get("toolsweep_xy_bin_refs"):
    raise SystemExit("missing toolsweep_xy_bin_refs")

for name in [
    "toolsweep_xy_bins_empty",
    "toolsweep_xy_bin_entries",
    "toolsweep_xy_bin_max_refs_per_bin",
    "toolsweep_xy_bin_refs_per_used_bin_x1000",
    "toolsweep_xy_bin_hot_bins_4x_avg",
    "toolsweep_xy_bin_hot_threshold",
    "toolsweep_xy_bin_max_box_x_span",
    "toolsweep_xy_bin_max_box_y_span",
    "toolsweep_xy_bin_max_box_refs",
    "toolsweep_xy_bin_refs_per_entry_x1000",
]:
    if name not in xy_metrics:
        raise SystemExit(f"missing XY-bin metric: {name}")

for name in [
    "toolsweep_xy_bin_queries",
    "toolsweep_xy_bin_out_of_bounds",
    "toolsweep_xy_bin_refs_scanned",
    "toolsweep_xy_bin_bbox_hits",
]:
    if name not in xy_counters:
        raise SystemExit(f"missing XY-bin counter: {name}")

if xy_counters["toolsweep_xy_bin_queries"] != xy_counters["toolsweep_depth_calls"]:
    raise SystemExit("XY-bin query count differs from ToolSweep depth calls")

if xy_counters["toolsweep_xy_bin_bbox_hits"] != xy_counters[
    "toolsweep_collision_candidates"
]:
    raise SystemExit("XY-bin bbox hits differ from collision candidates")

if xy_counters["toolsweep_xy_bin_bbox_hits"] > xy_counters[
    "toolsweep_xy_bin_refs_scanned"
]:
    raise SystemExit("XY-bin bbox hits exceed scanned refs")

if xyz_metrics.get("surface_triangles") != 191664:
    raise SystemExit(
        f"unexpected XYZ-bin cat triangle count: "
        f"{xyz_metrics.get('surface_triangles')}"
    )

if xyz_metrics.get("toolsweep_xyz_bin_count") != 16:
    raise SystemExit("missing or unexpected toolsweep_xyz_bin_count")

if not xyz_metrics.get("toolsweep_xyz_bin_refs"):
    raise SystemExit("missing toolsweep_xyz_bin_refs")

for name in [
    "toolsweep_xyz_bins_empty",
    "toolsweep_xyz_bin_entries",
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
    if name not in xyz_metrics:
        raise SystemExit(f"missing XYZ-bin metric: {name}")

for name in [
    "toolsweep_xyz_bin_queries",
    "toolsweep_xyz_bin_out_of_bounds",
    "toolsweep_xyz_bin_refs_scanned",
    "toolsweep_xyz_bin_bbox_hits",
]:
    if name not in xyz_counters:
        raise SystemExit(f"missing XYZ-bin counter: {name}")

if xyz_counters["toolsweep_xyz_bin_queries"] != xyz_counters[
    "toolsweep_depth_calls"
]:
    raise SystemExit("XYZ-bin query count differs from ToolSweep depth calls")

if xyz_counters["toolsweep_xyz_bin_bbox_hits"] != xyz_counters[
    "toolsweep_collision_candidates"
]:
    raise SystemExit("XYZ-bin bbox hits differ from collision candidates")

if xyz_counters["toolsweep_xyz_bin_bbox_hits"] > xyz_counters[
    "toolsweep_xyz_bin_refs_scanned"
]:
    raise SystemExit("XYZ-bin bbox hits exceed scanned refs")

print("camsim regression smoke passed")
PY
