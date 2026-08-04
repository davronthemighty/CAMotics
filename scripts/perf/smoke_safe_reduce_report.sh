#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

camsim="${1:-./camsim}"

tmp="${TMPDIR:-/tmp}/camotics-safe-reduce-report-$$"
mkdir -p "$tmp"
trap 'rm -rf "$tmp"' EXIT

"$camsim" \
  --threads 2 \
  --resolution 1 \
  --safe-reduce-report \
  --no-export \
  --profile "$tmp/profile.json" \
  examples/cat/cat.camotics \
  >"$tmp/camsim.log" 2>&1

grep -q "Safe reduction report:" "$tmp/camsim.log"

"$camsim" \
  --threads 2 \
  --resolution 1 \
  --safe-reduce-report \
  --profile "$tmp/default-no-export-profile.json" \
  examples/cat/cat.camotics \
  >"$tmp/default-no-export-camsim.log" 2>&1

grep -q "Safe reduction report:" "$tmp/default-no-export-camsim.log"

python3 - "$tmp/profile.json" "$tmp/default-no-export-profile.json" <<'PY'
import json
import sys

explicit = json.load(open(sys.argv[1])).get("metrics", {})
implicit = json.load(open(sys.argv[2])).get("metrics", {})

for key in (
    "safe_reduce_input_triangles",
    "safe_reduce_output_triangles",
    "safe_reduce_estimated_triangles_after",
    "safe_reduce_estimated_triangle_reduction",
    "safe_reduce_hole_aware_apply_requested",
    "safe_reduce_validation_candidate_checked",
):
    if implicit.get(key) != explicit.get(key):
        raise SystemExit(f"implicit safe-reduce-report no-export changed {key}")

if implicit.get("safe_reduce_validation_candidate_checked") != 0:
    raise SystemExit("implicit safe-reduce-report unexpectedly applied validation")
PY

python3 - "$tmp/profile.json" <<'PY'
import json
import sys

profile = json.load(open(sys.argv[1]))
metrics = profile.get("metrics", {})

required = [
    "safe_reduce_coord_tolerance_scaled_1e6",
    "safe_reduce_plane_distance_tolerance_scaled_1e6",
    "safe_reduce_pairwise_normal_angle_millidegrees",
    "safe_reduce_input_triangles",
    "safe_reduce_output_triangles",
    "safe_reduce_input_binary_stl_bytes",
    "safe_reduce_output_binary_stl_bytes",
    "safe_reduce_output_binary_stl_bytes_saved",
    "safe_reduce_estimated_binary_stl_bytes_after",
    "safe_reduce_estimated_binary_stl_bytes_saved",
    "safe_reduce_components",
    "safe_reduce_input_boundary_edges",
    "safe_reduce_input_nonmanifold_edges",
    "safe_reduce_input_misoriented_edges",
    "safe_reduce_input_degenerate_triangles",
    "safe_reduce_input_watertight",
    "safe_reduce_source_expected_floats",
    "safe_reduce_source_vertex_floats",
    "safe_reduce_source_normal_floats",
    "safe_reduce_source_vertex_count_mismatch",
    "safe_reduce_source_normal_count_mismatch",
    "safe_reduce_output_boundary_edges",
    "safe_reduce_output_nonmanifold_edges",
    "safe_reduce_output_misoriented_edges",
    "safe_reduce_output_degenerate_triangles",
    "safe_reduce_output_watertight",
    "safe_reduce_estimated_triangles_after",
    "safe_reduce_estimated_triangle_reduction",
    "safe_reduce_component_decision_fingerprint",
    "safe_reduce_decision_bearing_components",
    "safe_reduce_decision_bearing_triangles",
    "safe_reduce_single_triangle_components",
    "safe_reduce_components_lt8_triangles",
    "safe_reduce_components_lt64_triangles",
    "safe_reduce_max_component_triangles",
    "safe_reduce_component_neighbor_slots",
    "safe_reduce_component_neighbor_candidates",
    "safe_reduce_component_plane_fit_tests",
    "safe_reduce_component_plane_fit_accepted",
    "safe_reduce_component_plane_vertex_checks",
    "safe_reduce_boundary_edge_scans",
    "safe_reduce_component_boundary_edges",
    "safe_reduce_boundary_info_checks",
    "safe_reduce_phase1_replacement_checks",
    "safe_reduce_hole_aware_replacement_checks",
    "safe_reduce_estimated_replacement_checks",
    "safe_reduce_feasible_replacement_checks",
    "safe_reduce_writable_replacement_checks",
    "safe_reduce_unwritable_replacement_checks",
    "safe_reduce_phase1_writable_replacement_checks",
    "safe_reduce_hole_aware_writable_replacement_checks",
    "safe_reduce_phase1_unwritable_replacement_checks",
    "safe_reduce_hole_aware_unwritable_replacement_checks",
    "safe_reduce_replacement_edge_incidence_checks",
    "safe_reduce_replacement_edge_incidence_rejected",
    "safe_reduce_phase1_replacement_edge_incidence_rejected",
    "safe_reduce_hole_aware_replacement_edge_incidence_rejected",
    "safe_reduce_phase1_components",
    "safe_reduce_phase1_estimated_reduction",
    "safe_reduce_hole_aware_components",
    "safe_reduce_rejected_boundary_components",
    "safe_reduce_rejected_no_savings_components",
    "safe_reduce_rejected_triangulation_components",
    "safe_reduce_rejected_triangulation_triangles",
    "safe_reduce_contour_provenance_available",
    "safe_reduce_contour_provenance_triangles",
    "safe_reduce_contour_provenance_complete_triangles",
    "safe_reduce_contour_provenance_unknown_triangles",
    "safe_reduce_contour_provenance_raw_boundary_edges",
    "safe_reduce_contour_provenance_raw_nonmanifold_edges",
    "safe_reduce_contour_provenance_raw_misoriented_edges",
    "safe_reduce_contour_provenance_raw_unique_edges",
    "safe_reduce_contour_provenance_raw_max_edge_incidence",
    "safe_reduce_contour_provenance_raw_edges_incidence_1",
    "safe_reduce_contour_provenance_raw_edges_incidence_2",
    "safe_reduce_contour_provenance_raw_edges_incidence_3",
    "safe_reduce_contour_provenance_raw_edges_incidence_4",
    "safe_reduce_contour_provenance_raw_edges_incidence_5_plus",
    "safe_reduce_contour_provenance_raw_twin_edge_slots",
    "safe_reduce_contour_provenance_raw_boundary_edge_slots",
    "safe_reduce_contour_provenance_raw_nonmanifold_edge_slots",
    "safe_reduce_contour_provenance_raw_grid_grid_unique_edges",
    "safe_reduce_contour_provenance_raw_grid_grid_max_edge_incidence",
    "safe_reduce_contour_provenance_raw_grid_grid_twin_edge_slots",
    "safe_reduce_contour_provenance_raw_grid_grid_boundary_edge_slots",
    "safe_reduce_contour_provenance_raw_grid_grid_nonmanifold_edge_slots",
    "safe_reduce_contour_provenance_raw_grid_grid_welded_spread_edges",
    "safe_reduce_contour_provenance_raw_grid_grid_welded_spread_edge_slots",
    "safe_reduce_contour_provenance_raw_grid_grid_welded_spread_max_alternate_slots",
    "safe_reduce_contour_provenance_raw_center_involved_unique_edges",
    "safe_reduce_contour_provenance_raw_center_involved_max_edge_incidence",
    "safe_reduce_contour_provenance_raw_center_involved_twin_edge_slots",
    "safe_reduce_contour_provenance_raw_center_involved_boundary_edge_slots",
    "safe_reduce_contour_provenance_raw_center_involved_nonmanifold_edge_slots",
    "safe_reduce_contour_provenance_raw_center_involved_welded_spread_edges",
    "safe_reduce_contour_provenance_raw_center_involved_welded_spread_edge_slots",
    "safe_reduce_contour_provenance_raw_center_involved_welded_spread_max_alternate_slots",
    "safe_reduce_contour_provenance_raw_grid_vertex_unique_keys",
    "safe_reduce_contour_provenance_raw_grid_vertex_welded_spread_keys",
    "safe_reduce_contour_provenance_raw_grid_vertex_welded_spread_observations",
    "safe_reduce_contour_provenance_raw_grid_vertex_welded_spread_max_alternate_observations",
    "safe_reduce_contour_provenance_raw_center_vertex_unique_keys",
    "safe_reduce_contour_provenance_raw_center_vertex_welded_spread_keys",
    "safe_reduce_contour_provenance_raw_center_vertex_welded_spread_observations",
    "safe_reduce_contour_provenance_raw_center_vertex_welded_spread_max_alternate_observations",
    "safe_reduce_contour_provenance_boundary_edges",
    "safe_reduce_contour_provenance_nonmanifold_edges",
    "safe_reduce_contour_provenance_misoriented_edges",
    "safe_reduce_contour_provenance_welded_unique_edges",
    "safe_reduce_contour_provenance_welded_max_edge_incidence",
    "safe_reduce_contour_provenance_welded_edges_incidence_1",
    "safe_reduce_contour_provenance_welded_edges_incidence_2",
    "safe_reduce_contour_provenance_welded_edges_incidence_3",
    "safe_reduce_contour_provenance_welded_edges_incidence_4",
    "safe_reduce_contour_provenance_welded_edges_incidence_5_plus",
    "safe_reduce_contour_provenance_welded_twin_edge_slots",
    "safe_reduce_contour_provenance_welded_boundary_edge_slots",
    "safe_reduce_contour_provenance_welded_nonmanifold_edge_slots",
    "safe_reduce_contour_provenance_watertight",
    "safe_reduce_contour_provenance_matches_input",
    "safe_reduce_contour_provenance_neighbors_available",
    "safe_reduce_contour_provenance_neighbors_cached",
    "safe_reduce_contour_provenance_neighbors_raw",
    "safe_reduce_contour_provenance_neighbor_slots",
    "safe_reduce_contour_provenance_neighbor_mismatches",
    "safe_reduce_contour_provenance_neighbor_parity_audited",
    "safe_reduce_contour_provenance_neighbor_parity",
    "safe_reduce_contour_provenance_component_report_available",
    "safe_reduce_contour_provenance_components",
    "safe_reduce_contour_provenance_component_decision_fingerprint",
    "safe_reduce_contour_provenance_decision_bearing_components",
    "safe_reduce_contour_provenance_decision_bearing_triangles",
    "safe_reduce_contour_provenance_estimated_triangles_after",
    "safe_reduce_contour_provenance_estimated_triangle_reduction",
    "safe_reduce_contour_provenance_phase1_components",
    "safe_reduce_contour_provenance_hole_aware_components",
    "safe_reduce_contour_provenance_estimated_replacement_checks",
    "safe_reduce_contour_provenance_feasible_replacement_checks",
    "safe_reduce_contour_provenance_writable_replacement_checks",
    "safe_reduce_contour_provenance_unwritable_replacement_checks",
    "safe_reduce_contour_provenance_phase1_writable_replacement_checks",
    "safe_reduce_contour_provenance_hole_aware_writable_replacement_checks",
    "safe_reduce_contour_provenance_phase1_unwritable_replacement_checks",
    "safe_reduce_contour_provenance_hole_aware_unwritable_replacement_checks",
    "safe_reduce_contour_provenance_replacement_edge_incidence_checks",
    "safe_reduce_contour_provenance_replacement_edge_incidence_rejected",
    "safe_reduce_contour_provenance_phase1_replacement_edge_incidence_rejected",
    "safe_reduce_contour_provenance_hole_aware_replacement_edge_incidence_rejected",
    "safe_reduce_contour_provenance_rejected_boundary_components",
    "safe_reduce_contour_provenance_rejected_no_savings_components",
    "safe_reduce_contour_provenance_rejected_triangulation_components",
    "safe_reduce_contour_provenance_component_metric_mismatches",
    "safe_reduce_contour_provenance_component_parity",
    "safe_reduce_hole_aware_apply_requested",
    "safe_reduce_trust_provenance_neighbors_requested",
    "safe_reduce_trusted_provenance_neighbors_eligible",
    "safe_reduce_trusted_provenance_rejected_no_triangle_surface",
    "safe_reduce_trusted_provenance_rejected_no_provenance",
    "safe_reduce_trusted_provenance_rejected_no_cached_neighbors",
    "safe_reduce_trusted_provenance_rejected_triangle_mismatch",
    "safe_reduce_trusted_provenance_rejected_incomplete",
    "safe_reduce_trusted_provenance_rejected_unknown",
    "safe_reduce_trusted_provenance_rejected_non_watertight",
    "safe_reduce_trusted_provenance_rejected_orientation",
    "safe_reduce_trusted_provenance_rejected_raw_topology",
    "safe_reduce_trusted_provenance_rejected_raw_welded_spread",
    "safe_reduce_trusted_provenance_rejected_neighbor_size",
    "safe_reduce_trusted_provenance_rejected_neighbor_open_slot",
    "safe_reduce_trusted_provenance_rejected_neighbor_range",
    "safe_reduce_trusted_provenance_rejected_neighbor_self",
    "safe_reduce_trusted_provenance_rejected_neighbor_duplicate",
    "safe_reduce_trusted_provenance_rejected_neighbor_asymmetry",
    "safe_reduce_trusted_provenance_rejected_neighbor_edge_mismatch",
    "safe_reduce_trusted_provenance_neighbor_slots_checked",
    "safe_reduce_trusted_provenance_neighbor_edge_slots_checked",
    "safe_reduce_trusted_provenance_neighbor_edge_mismatches",
    "safe_reduce_trusted_provenance_neighbors_used",
    "safe_reduce_default_adjacency_skipped",
    "safe_reduce_boundary_cosimplify_apply_requested",
    "safe_reduce_boundary_cosimplify_candidate_rolled_back",
    "safe_reduce_boundary_cosimplify_fallback_used",
    "safe_reduce_boundary_cosimplify_candidate_components",
    "safe_reduce_boundary_cosimplify_source_triangles",
    "safe_reduce_boundary_cosimplify_boundary_vertices",
    "safe_reduce_boundary_cosimplify_simplified_boundary_vertices",
    "safe_reduce_boundary_cosimplify_estimated_triangles_after",
    "safe_reduce_boundary_cosimplify_estimated_triangles_after_simplified",
    "safe_reduce_boundary_cosimplify_estimated_extra_reduction",
    "safe_reduce_boundary_cosimplify_max_component_extra_reduction",
    "safe_reduce_boundary_cosimplify_rejected_candidate_boundary_edges",
    "safe_reduce_boundary_cosimplify_rejected_candidate_nonmanifold_edges",
    "safe_reduce_boundary_cosimplify_rejected_candidate_misoriented_edges",
    "safe_reduce_boundary_cosimplify_rejected_candidate_degenerate_triangles",
    "safe_reduce_boundary_cosimplify_rejected_candidate_expected_triangles",
    "safe_reduce_boundary_cosimplify_rejected_candidate_actual_triangles",
    "safe_reduce_boundary_cosimplify_rejected_candidate_topology_worse",
    "safe_reduce_boundary_cosimplify_rejected_candidate_degenerate_worse",
    "safe_reduce_boundary_cosimplify_rejected_candidate_orientation_worse",
    "safe_reduce_boundary_cosimplify_rejected_candidate_vertex_count_mismatch",
    "safe_reduce_boundary_cosimplify_rejected_candidate_normal_count_mismatch",
    "safe_reduce_boundary_cosimplify_rejected_candidate_triangle_count_mismatch",
    "safe_reduce_boundary_cosimplify_contract_vertices_considered",
    "safe_reduce_boundary_cosimplify_contract_vertices_accepted",
    "safe_reduce_boundary_cosimplify_contract_rejected_single_sided",
    "safe_reduce_boundary_cosimplify_contract_rejected_ambiguous",
    "safe_reduce_boundary_cosimplify_contract_rejected_non_collinear",
    "safe_reduce_boundary_cosimplify_contract_rejected_ineligible",
    "safe_reduce_boundary_cosimplify_contract_rejected_ownership",
    "safe_reduce_boundary_cosimplify_contract_interface_edges",
    "safe_reduce_boundary_cosimplify_contract_chain_interfaces",
    "safe_reduce_boundary_cosimplify_contract_chains",
    "safe_reduce_boundary_cosimplify_contract_chain_vertices",
    "safe_reduce_boundary_cosimplify_contract_chain_interior_vertices",
    "safe_reduce_boundary_cosimplify_contract_chain_vertices_accepted",
    "safe_reduce_boundary_cosimplify_contract_rejected_missing_owner",
    "safe_reduce_boundary_cosimplify_contract_rejected_ambiguous_owner",
    "safe_reduce_boundary_cosimplify_contract_rejected_chain_ineligible",
    "safe_reduce_boundary_cosimplify_contract_rejected_unsafe_endpoint",
    "safe_reduce_boundary_cosimplify_contract_rejected_chain_non_collinear",
    "safe_reduce_boundary_cosimplify_contract_components_considered",
    "safe_reduce_boundary_cosimplify_contract_components_affected",
    "safe_reduce_boundary_cosimplify_contract_replacement_checks",
    "safe_reduce_boundary_cosimplify_contract_triangulation_rejected",
    "safe_reduce_boundary_cosimplify_contract_edge_incidence_rejected",
    "safe_reduce_boundary_cosimplify_contract_no_savings_rejected",
    "safe_reduce_boundary_cosimplify_contract_global_rejected",
    "safe_reduce_boundary_cosimplify_contract_applied_components",
    "safe_reduce_boundary_cosimplify_contract_applied_source_triangles",
    "safe_reduce_boundary_cosimplify_contract_applied_output_triangles",
    "safe_reduce_validation_topology_worse",
    "safe_reduce_validation_degenerate_worse",
    "safe_reduce_validation_orientation_worse",
    "safe_reduce_validation_vertex_count_mismatch",
    "safe_reduce_validation_normal_count_mismatch",
    "safe_reduce_validation_triangle_count_mismatch",
    "safe_reduce_validation_rolled_back",
    "safe_reduce_validation_expected_output_triangles",
    "safe_reduce_validation_candidate_triangles",
    "safe_reduce_validation_candidate_checked",
    "safe_reduce_validation_candidate_boundary_edges",
    "safe_reduce_validation_candidate_nonmanifold_edges",
    "safe_reduce_validation_candidate_misoriented_edges",
    "safe_reduce_validation_candidate_degenerate_triangles",
    "safe_reduce_validation_candidate_watertight",
]

side_names = ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max", "cut")
side_fields = (
    "input_triangles",
    "component_triangles",
    "zero_normal_triangles",
    "degenerate_triangles",
    "unaccounted_triangles",
    "components",
    "single_triangle_components",
    "single_triangle_triangles",
    "estimated_triangles_after",
    "estimated_triangle_reduction",
    "output_triangles",
    "phase1_components",
    "phase1_source_triangles",
    "phase1_estimated_output_triangles",
    "phase1_estimated_reduction",
    "hole_aware_components",
    "hole_aware_source_triangles",
    "hole_aware_estimated_output_triangles",
    "hole_aware_estimated_reduction",
    "rejected_boundary_components",
    "rejected_boundary_triangles",
    "rejected_no_savings_components",
    "rejected_no_savings_triangles",
    "rejected_triangulation_components",
    "rejected_triangulation_triangles",
    "applied_components",
    "applied_source_triangles",
    "applied_output_triangles",
    "validation_rollback_components",
    "validation_rollback_source_triangles",
    "validation_rollback_candidate_output_triangles",
    "boundary_cosimplify_candidate_components",
    "boundary_cosimplify_source_triangles",
    "boundary_cosimplify_boundary_vertices",
    "boundary_cosimplify_simplified_boundary_vertices",
    "boundary_cosimplify_estimated_triangles_after",
    "boundary_cosimplify_estimated_triangles_after_simplified",
    "boundary_cosimplify_estimated_extra_reduction",
)
for side_name in side_names:
    for field in side_fields:
        required.append(f"safe_reduce_side_{side_name}_{field}")

missing = [name for name in required if name not in metrics]
if missing:
    raise SystemExit(f"missing safe reduce metrics: {missing}")

if metrics["safe_reduce_input_triangles"] <= 0:
    raise SystemExit("safe reduce report saw no triangles")
if metrics["safe_reduce_coord_tolerance_scaled_1e6"] != 100:
    raise SystemExit("safe reduce report coord tolerance default changed")
if metrics["safe_reduce_plane_distance_tolerance_scaled_1e6"] != 100:
    raise SystemExit("safe reduce report plane tolerance default changed")
if metrics["safe_reduce_pairwise_normal_angle_millidegrees"] != 250:
    raise SystemExit("safe reduce report normal angle default changed")
input_bytes = 84 + 50 * metrics["safe_reduce_input_triangles"]
output_bytes = 84 + 50 * metrics["safe_reduce_output_triangles"]
estimated_bytes = 84 + 50 * metrics["safe_reduce_estimated_triangles_after"]
if metrics["safe_reduce_input_binary_stl_bytes"] != input_bytes:
    raise SystemExit("safe reduce input byte estimate mismatch")
if metrics["safe_reduce_output_binary_stl_bytes"] != output_bytes:
    raise SystemExit("safe reduce output byte estimate mismatch")
if metrics["safe_reduce_estimated_binary_stl_bytes_after"] != estimated_bytes:
    raise SystemExit("safe reduce estimated byte output mismatch")
if metrics["safe_reduce_output_binary_stl_bytes_saved"] != max(0, input_bytes - output_bytes):
    raise SystemExit("safe reduce output byte savings mismatch")
if metrics["safe_reduce_estimated_binary_stl_bytes_saved"] != max(0, input_bytes - estimated_bytes):
    raise SystemExit("safe reduce estimated byte savings mismatch")
if metrics["safe_reduce_estimated_binary_stl_bytes_saved"] <= 0:
    raise SystemExit("safe reduce report estimated no binary STL byte savings")
if metrics["safe_reduce_components"] <= 0:
    raise SystemExit("safe reduce report saw no components")
side_input = sum(metrics[f"safe_reduce_side_{name}_input_triangles"] for name in side_names)
side_components = sum(metrics[f"safe_reduce_side_{name}_components"] for name in side_names)
side_component_triangles = sum(metrics[f"safe_reduce_side_{name}_component_triangles"] for name in side_names)
side_zero_normals = sum(metrics[f"safe_reduce_side_{name}_zero_normal_triangles"] for name in side_names)
side_unaccounted = sum(metrics[f"safe_reduce_side_{name}_unaccounted_triangles"] for name in side_names)
side_output = sum(metrics[f"safe_reduce_side_{name}_output_triangles"] for name in side_names)
side_estimated_after = sum(metrics[f"safe_reduce_side_{name}_estimated_triangles_after"] for name in side_names)
side_estimated_reduction = sum(metrics[f"safe_reduce_side_{name}_estimated_triangle_reduction"] for name in side_names)
side_single_components = sum(metrics[f"safe_reduce_side_{name}_single_triangle_components"] for name in side_names)
side_single_triangles = sum(metrics[f"safe_reduce_side_{name}_single_triangle_triangles"] for name in side_names)
side_phase1_components = sum(metrics[f"safe_reduce_side_{name}_phase1_components"] for name in side_names)
side_phase1_source = sum(metrics[f"safe_reduce_side_{name}_phase1_source_triangles"] for name in side_names)
side_phase1_reduction = sum(metrics[f"safe_reduce_side_{name}_phase1_estimated_reduction"] for name in side_names)
side_hole_components = sum(metrics[f"safe_reduce_side_{name}_hole_aware_components"] for name in side_names)
side_hole_source = sum(metrics[f"safe_reduce_side_{name}_hole_aware_source_triangles"] for name in side_names)
side_hole_reduction = sum(metrics[f"safe_reduce_side_{name}_hole_aware_estimated_reduction"] for name in side_names)
side_rejected_boundary_components = sum(metrics[f"safe_reduce_side_{name}_rejected_boundary_components"] for name in side_names)
side_rejected_boundary_triangles = sum(metrics[f"safe_reduce_side_{name}_rejected_boundary_triangles"] for name in side_names)
side_rejected_no_savings_components = sum(metrics[f"safe_reduce_side_{name}_rejected_no_savings_components"] for name in side_names)
side_rejected_no_savings_triangles = sum(metrics[f"safe_reduce_side_{name}_rejected_no_savings_triangles"] for name in side_names)
side_rejected_triangulation_components = sum(metrics[f"safe_reduce_side_{name}_rejected_triangulation_components"] for name in side_names)
side_rejected_triangulation_triangles = sum(metrics[f"safe_reduce_side_{name}_rejected_triangulation_triangles"] for name in side_names)
side_cosimplify_components = sum(metrics[f"safe_reduce_side_{name}_boundary_cosimplify_candidate_components"] for name in side_names)
side_cosimplify_source = sum(metrics[f"safe_reduce_side_{name}_boundary_cosimplify_source_triangles"] for name in side_names)
side_cosimplify_boundary_vertices = sum(metrics[f"safe_reduce_side_{name}_boundary_cosimplify_boundary_vertices"] for name in side_names)
side_cosimplify_simplified_vertices = sum(metrics[f"safe_reduce_side_{name}_boundary_cosimplify_simplified_boundary_vertices"] for name in side_names)
side_cosimplify_after = sum(metrics[f"safe_reduce_side_{name}_boundary_cosimplify_estimated_triangles_after"] for name in side_names)
side_cosimplify_after_simplified = sum(metrics[f"safe_reduce_side_{name}_boundary_cosimplify_estimated_triangles_after_simplified"] for name in side_names)
side_cosimplify_extra_reduction = sum(metrics[f"safe_reduce_side_{name}_boundary_cosimplify_estimated_extra_reduction"] for name in side_names)
if side_input != metrics["safe_reduce_input_triangles"]:
    raise SystemExit("safe reduce side input triangle total mismatch")
if side_components != metrics["safe_reduce_components"]:
    raise SystemExit("safe reduce side component total mismatch")
if side_component_triangles + side_zero_normals + side_unaccounted != side_input:
    raise SystemExit("safe reduce side input/component ledger mismatch")
if side_unaccounted != 0:
    raise SystemExit("safe reduce side ledger left triangles unaccounted")
if side_output != metrics["safe_reduce_output_triangles"]:
    raise SystemExit("safe reduce side output triangle total mismatch")
if side_estimated_after != metrics["safe_reduce_estimated_triangles_after"]:
    raise SystemExit("safe reduce side estimated output total mismatch")
if side_estimated_reduction != metrics["safe_reduce_estimated_triangle_reduction"]:
    raise SystemExit("safe reduce side estimated reduction total mismatch")
if side_single_components != metrics["safe_reduce_single_triangle_components"]:
    raise SystemExit("safe reduce side single-triangle component mismatch")
if side_single_triangles != metrics["safe_reduce_single_triangle_components"]:
    raise SystemExit("safe reduce side single-triangle count mismatch")
if side_phase1_components != metrics["safe_reduce_phase1_components"]:
    raise SystemExit("safe reduce side phase-one component mismatch")
if side_phase1_source != metrics["safe_reduce_phase1_source_triangles"]:
    raise SystemExit("safe reduce side phase-one source mismatch")
if side_phase1_reduction != metrics["safe_reduce_phase1_estimated_reduction"]:
    raise SystemExit("safe reduce side phase-one reduction mismatch")
if side_hole_components != metrics["safe_reduce_hole_aware_components"]:
    raise SystemExit("safe reduce side hole-aware component mismatch")
if side_hole_source != metrics["safe_reduce_hole_aware_source_triangles"]:
    raise SystemExit("safe reduce side hole-aware source mismatch")
if side_hole_reduction != metrics["safe_reduce_hole_aware_estimated_reduction"]:
    raise SystemExit("safe reduce side hole-aware reduction mismatch")
if side_rejected_boundary_components != metrics["safe_reduce_rejected_boundary_components"]:
    raise SystemExit("safe reduce side rejected-boundary component mismatch")
if side_rejected_boundary_triangles != metrics["safe_reduce_rejected_boundary_triangles"]:
    raise SystemExit("safe reduce side rejected-boundary triangle mismatch")
if side_rejected_no_savings_components + side_single_components != metrics["safe_reduce_rejected_no_savings_components"]:
    raise SystemExit("safe reduce side no-savings component mismatch")
if side_rejected_no_savings_triangles + side_single_triangles != metrics["safe_reduce_rejected_no_savings_triangles"]:
    raise SystemExit("safe reduce side no-savings triangle mismatch")
if side_rejected_triangulation_components != metrics["safe_reduce_rejected_triangulation_components"]:
    raise SystemExit("safe reduce side rejected-triangulation component mismatch")
if side_rejected_triangulation_triangles != metrics["safe_reduce_rejected_triangulation_triangles"]:
    raise SystemExit("safe reduce side rejected-triangulation triangle mismatch")
if side_cosimplify_components != metrics["safe_reduce_boundary_cosimplify_candidate_components"]:
    raise SystemExit("safe reduce side boundary co-simplify component mismatch")
if side_cosimplify_source != metrics["safe_reduce_boundary_cosimplify_source_triangles"]:
    raise SystemExit("safe reduce side boundary co-simplify source mismatch")
if side_cosimplify_boundary_vertices != metrics["safe_reduce_boundary_cosimplify_boundary_vertices"]:
    raise SystemExit("safe reduce side boundary co-simplify boundary-vertex mismatch")
if side_cosimplify_simplified_vertices != metrics["safe_reduce_boundary_cosimplify_simplified_boundary_vertices"]:
    raise SystemExit("safe reduce side boundary co-simplify simplified-vertex mismatch")
if side_cosimplify_after != metrics["safe_reduce_boundary_cosimplify_estimated_triangles_after"]:
    raise SystemExit("safe reduce side boundary co-simplify estimated-after mismatch")
if side_cosimplify_after_simplified != metrics["safe_reduce_boundary_cosimplify_estimated_triangles_after_simplified"]:
    raise SystemExit("safe reduce side boundary co-simplify simplified-after mismatch")
if side_cosimplify_extra_reduction != metrics["safe_reduce_boundary_cosimplify_estimated_extra_reduction"]:
    raise SystemExit("safe reduce side boundary co-simplify extra-reduction mismatch")
if side_cosimplify_simplified_vertices > side_cosimplify_boundary_vertices:
    raise SystemExit("safe reduce boundary co-simplify grew boundary vertices")
if side_cosimplify_after_simplified > side_cosimplify_after:
    raise SystemExit("safe reduce boundary co-simplify grew estimated triangles")
if metrics["safe_reduce_input_watertight"] != 1:
    raise SystemExit("cat reference surface should be watertight")
if metrics["safe_reduce_source_expected_floats"] != metrics["safe_reduce_input_triangles"] * 9:
    raise SystemExit("safe reduce report source expected-float count mismatch")
if metrics["safe_reduce_source_vertex_floats"] != metrics["safe_reduce_source_expected_floats"]:
    raise SystemExit("safe reduce report source vertex float count mismatch")
if metrics["safe_reduce_source_normal_floats"] != metrics["safe_reduce_source_expected_floats"]:
    raise SystemExit("safe reduce report source normal float count mismatch")
if metrics["safe_reduce_source_vertex_count_mismatch"] != 0:
    raise SystemExit("safe reduce report unexpectedly flagged source vertices")
if metrics["safe_reduce_source_normal_count_mismatch"] != 0:
    raise SystemExit("safe reduce report unexpectedly flagged source normals")
if metrics["safe_reduce_output_watertight"] != 1:
    raise SystemExit("safe reduce report output should match input watertightness")
if metrics["safe_reduce_input_boundary_edges"] != 0:
    raise SystemExit("cat reference surface should have no boundary edges")
if metrics["safe_reduce_input_nonmanifold_edges"] != 0:
    raise SystemExit("cat reference surface should have no nonmanifold edges")
if metrics["safe_reduce_input_misoriented_edges"] != 0:
    raise SystemExit("cat reference surface should have no misoriented edges")
if metrics["safe_reduce_input_degenerate_triangles"] != 0:
    raise SystemExit("cat reference surface should have no degenerate triangles")
if metrics["safe_reduce_output_boundary_edges"] != 0:
    raise SystemExit("safe reduce report output should have no boundary edges")
if metrics["safe_reduce_output_nonmanifold_edges"] != 0:
    raise SystemExit("safe reduce report output should have no nonmanifold edges")
if metrics["safe_reduce_output_misoriented_edges"] != 0:
    raise SystemExit("safe reduce report output should have no misoriented edges")
if metrics["safe_reduce_output_degenerate_triangles"] != 0:
    raise SystemExit("safe reduce report output should have no degenerates")
if metrics["safe_reduce_phase1_components"] <= 0:
    raise SystemExit("safe reduce report found no phase-one candidates")
if metrics["safe_reduce_phase1_estimated_reduction"] <= 0:
    raise SystemExit("safe reduce report estimated no phase-one reduction")
if metrics["safe_reduce_estimated_triangles_after"] >= metrics["safe_reduce_input_triangles"]:
    raise SystemExit("safe reduce report did not estimate a smaller mesh")
if metrics["safe_reduce_decision_bearing_components"] <= 0:
    raise SystemExit("safe reduce recorded no decision-bearing components")
if metrics["safe_reduce_decision_bearing_triangles"] <= 0:
    raise SystemExit("safe reduce recorded no decision-bearing triangles")
if metrics["safe_reduce_component_decision_fingerprint"] == 0:
    raise SystemExit("safe reduce decision fingerprint was not recorded")
if metrics["safe_reduce_boundary_info_checks"] != metrics["safe_reduce_components"]:
    raise SystemExit("safe reduce boundary checks no longer match component count")
if metrics["safe_reduce_component_neighbor_slots"] != metrics["safe_reduce_input_triangles"] * 3:
    raise SystemExit("safe reduce component neighbor slot accounting changed")
if metrics["safe_reduce_boundary_edge_scans"] != metrics["safe_reduce_input_triangles"] * 3:
    raise SystemExit("safe reduce boundary edge scan accounting changed")
if metrics["safe_reduce_component_plane_fit_tests"] < metrics["safe_reduce_component_plane_fit_accepted"]:
    raise SystemExit("safe reduce accepted more plane fits than it tested")
if metrics["safe_reduce_component_plane_vertex_checks"] <= 0:
    raise SystemExit("safe reduce recorded no plane vertex checks")
if metrics["safe_reduce_component_plane_vertex_checks"] > metrics["safe_reduce_component_plane_fit_tests"] * 3:
    raise SystemExit("safe reduce plane vertex check accounting exceeded full-triangle checks")
if metrics["safe_reduce_components_lt8_triangles"] < metrics["safe_reduce_single_triangle_components"]:
    raise SystemExit("safe reduce small component bucket ordering changed")
if metrics["safe_reduce_components_lt64_triangles"] < metrics["safe_reduce_components_lt8_triangles"]:
    raise SystemExit("safe reduce small component bucket ordering changed")
if metrics["safe_reduce_max_component_triangles"] <= 0:
    raise SystemExit("safe reduce max component size was not recorded")
if metrics["safe_reduce_phase1_replacement_checks"] < metrics["safe_reduce_phase1_components"]:
    raise SystemExit("safe reduce phase-one replacement checks are inconsistent")
if metrics["safe_reduce_estimated_replacement_checks"] < metrics["safe_reduce_feasible_replacement_checks"]:
    raise SystemExit("safe reduce estimated fewer replacements than were feasible")
if metrics["safe_reduce_feasible_replacement_checks"] < metrics["safe_reduce_phase1_components"]:
    raise SystemExit("safe reduce feasible replacement checks are inconsistent")
if metrics["safe_reduce_writable_replacement_checks"] != metrics["safe_reduce_feasible_replacement_checks"]:
    raise SystemExit("safe reduce writable/feasible replacement checks diverged")
if (
    metrics["safe_reduce_writable_replacement_checks"]
    + metrics["safe_reduce_unwritable_replacement_checks"]
    != metrics["safe_reduce_estimated_replacement_checks"]
):
    raise SystemExit("safe reduce writable/unwritable accounting changed")
if metrics["safe_reduce_phase1_writable_replacement_checks"] < metrics["safe_reduce_phase1_components"]:
    raise SystemExit("safe reduce phase-one writable replacement checks are inconsistent")
if (
    metrics["safe_reduce_phase1_writable_replacement_checks"]
    + metrics["safe_reduce_phase1_unwritable_replacement_checks"]
    + metrics["safe_reduce_hole_aware_writable_replacement_checks"]
    + metrics["safe_reduce_hole_aware_unwritable_replacement_checks"]
    != metrics["safe_reduce_estimated_replacement_checks"]
):
    raise SystemExit("safe reduce replacement bucket accounting changed")
if (
    metrics["safe_reduce_phase1_replacement_edge_incidence_rejected"]
    + metrics["safe_reduce_hole_aware_replacement_edge_incidence_rejected"]
    != metrics["safe_reduce_replacement_edge_incidence_rejected"]
):
    raise SystemExit("safe reduce local edge-incidence rejection buckets changed")
if metrics["safe_reduce_replacement_edge_incidence_checks"] < metrics["safe_reduce_writable_replacement_checks"]:
    raise SystemExit("safe reduce writable replacements exceeded local edge-incidence checks")
if metrics["safe_reduce_contour_provenance_available"] != 1:
    raise SystemExit("safe reduce report did not see contour provenance")
if metrics["safe_reduce_contour_provenance_triangles"] != metrics["safe_reduce_input_triangles"]:
    raise SystemExit("safe reduce provenance triangle count mismatch")
if metrics["safe_reduce_contour_provenance_complete_triangles"] != metrics["safe_reduce_input_triangles"]:
    raise SystemExit("safe reduce provenance is incomplete")
if metrics["safe_reduce_contour_provenance_unknown_triangles"] != 0:
    raise SystemExit("safe reduce provenance has unknown triangles")
edge_slots = metrics["safe_reduce_input_triangles"] * 3
for prefix in (
    "safe_reduce_contour_provenance_raw",
    "safe_reduce_contour_provenance_welded",
):
    unique = metrics[f"{prefix}_unique_edges"]
    histogram = (
        metrics[f"{prefix}_edges_incidence_1"]
        + metrics[f"{prefix}_edges_incidence_2"]
        + metrics[f"{prefix}_edges_incidence_3"]
        + metrics[f"{prefix}_edges_incidence_4"]
        + metrics[f"{prefix}_edges_incidence_5_plus"]
    )
    if unique != histogram:
        raise SystemExit(f"{prefix} edge incidence histogram mismatch")
    twin = metrics[f"{prefix}_twin_edge_slots"]
    boundary = metrics[f"{prefix}_boundary_edge_slots"]
    nonmanifold = metrics[f"{prefix}_nonmanifold_edge_slots"]
    if twin + boundary + nonmanifold != edge_slots:
        raise SystemExit(f"{prefix} edge slot accounting mismatch")
for suffix in (
    "twin_edge_slots",
    "boundary_edge_slots",
    "nonmanifold_edge_slots",
):
    class_sum = (
        metrics[f"safe_reduce_contour_provenance_raw_grid_grid_{suffix}"]
        + metrics[f"safe_reduce_contour_provenance_raw_center_involved_{suffix}"]
    )
    if class_sum != metrics[f"safe_reduce_contour_provenance_raw_{suffix}"]:
        raise SystemExit(f"safe reduce raw provenance {suffix} class sum mismatch")
if metrics["safe_reduce_contour_provenance_boundary_edges"] != metrics["safe_reduce_input_boundary_edges"]:
    raise SystemExit("safe reduce provenance boundary edges mismatch input")
if metrics["safe_reduce_contour_provenance_nonmanifold_edges"] != metrics["safe_reduce_input_nonmanifold_edges"]:
    raise SystemExit("safe reduce provenance nonmanifold edges mismatch input")
if metrics["safe_reduce_contour_provenance_misoriented_edges"] != metrics["safe_reduce_input_misoriented_edges"]:
    raise SystemExit("safe reduce provenance misoriented edges mismatch input")
if metrics["safe_reduce_contour_provenance_watertight"] != metrics["safe_reduce_input_watertight"]:
    raise SystemExit("safe reduce provenance watertightness mismatch input")
if metrics["safe_reduce_contour_provenance_matches_input"] != 1:
    raise SystemExit("safe reduce provenance did not match input topology")
if metrics["safe_reduce_contour_provenance_neighbors_available"] != 1:
    raise SystemExit("safe reduce provenance neighbors unavailable")
if metrics["safe_reduce_contour_provenance_neighbors_cached"] != 0:
    raise SystemExit("safe reduce default report should rebuild provenance neighbors")
if metrics["safe_reduce_contour_provenance_neighbors_raw"] not in (0, 1):
    raise SystemExit("safe reduce raw-neighbor metric must be boolean")
if metrics["safe_reduce_contour_provenance_neighbors_raw"]:
    if metrics["safe_reduce_contour_provenance_raw_boundary_edges"] != 0:
        raise SystemExit("safe reduce raw neighbors had boundary edges")
    if metrics["safe_reduce_contour_provenance_raw_nonmanifold_edges"] != 0:
        raise SystemExit("safe reduce raw neighbors had nonmanifold edges")
    if metrics["safe_reduce_contour_provenance_raw_misoriented_edges"] != 0:
        raise SystemExit("safe reduce raw neighbors had misoriented edges")
    if metrics["safe_reduce_contour_provenance_misoriented_edges"] != 0:
        raise SystemExit("safe reduce welded provenance had misoriented edges")
    if metrics["safe_reduce_contour_provenance_raw_unique_edges"] != metrics["safe_reduce_contour_provenance_welded_unique_edges"]:
        raise SystemExit("safe reduce raw/welded unique edge mismatch")
    if metrics["safe_reduce_contour_provenance_raw_grid_vertex_welded_spread_keys"] != 0:
        raise SystemExit("safe reduce raw grid vertex spread")
    if metrics["safe_reduce_contour_provenance_raw_center_vertex_welded_spread_keys"] != 0:
        raise SystemExit("safe reduce raw center vertex spread")
if metrics["safe_reduce_contour_provenance_neighbor_slots"] != metrics["safe_reduce_input_triangles"] * 3:
    raise SystemExit("safe reduce provenance neighbor slot count mismatch")
if metrics["safe_reduce_contour_provenance_neighbor_mismatches"] != 0:
    raise SystemExit("safe reduce provenance neighbor mismatch")
if metrics["safe_reduce_contour_provenance_neighbor_parity_audited"] != 1:
    raise SystemExit("safe reduce provenance neighbor parity was not audited")
if metrics["safe_reduce_contour_provenance_neighbor_parity"] != 1:
    raise SystemExit("safe reduce provenance neighbor parity failed")
if metrics["safe_reduce_contour_provenance_component_report_available"] != 1:
    raise SystemExit("safe reduce provenance component report unavailable")
if metrics["safe_reduce_contour_provenance_components"] != metrics["safe_reduce_components"]:
    raise SystemExit("safe reduce provenance component count mismatch")
if metrics["safe_reduce_contour_provenance_component_decision_fingerprint"] != metrics["safe_reduce_component_decision_fingerprint"]:
    raise SystemExit("safe reduce provenance component fingerprint mismatch")
if metrics["safe_reduce_contour_provenance_estimated_triangles_after"] != metrics["safe_reduce_estimated_triangles_after"]:
    raise SystemExit("safe reduce provenance estimated output mismatch")
if metrics["safe_reduce_contour_provenance_estimated_triangle_reduction"] != metrics["safe_reduce_estimated_triangle_reduction"]:
    raise SystemExit("safe reduce provenance estimated reduction mismatch")
if metrics["safe_reduce_contour_provenance_phase1_components"] != metrics["safe_reduce_phase1_components"]:
    raise SystemExit("safe reduce provenance phase-one component mismatch")
if metrics["safe_reduce_contour_provenance_hole_aware_components"] != metrics["safe_reduce_hole_aware_components"]:
    raise SystemExit("safe reduce provenance hole-aware component mismatch")
if metrics["safe_reduce_contour_provenance_estimated_replacement_checks"] != metrics["safe_reduce_estimated_replacement_checks"]:
    raise SystemExit("safe reduce provenance estimated replacement mismatch")
if metrics["safe_reduce_contour_provenance_feasible_replacement_checks"] != metrics["safe_reduce_feasible_replacement_checks"]:
    raise SystemExit("safe reduce provenance feasible replacement mismatch")
if metrics["safe_reduce_contour_provenance_writable_replacement_checks"] != metrics["safe_reduce_writable_replacement_checks"]:
    raise SystemExit("safe reduce provenance writable replacement mismatch")
if metrics["safe_reduce_contour_provenance_unwritable_replacement_checks"] != metrics["safe_reduce_unwritable_replacement_checks"]:
    raise SystemExit("safe reduce provenance unwritable replacement mismatch")
if metrics["safe_reduce_contour_provenance_phase1_writable_replacement_checks"] != metrics["safe_reduce_phase1_writable_replacement_checks"]:
    raise SystemExit("safe reduce provenance phase-one writable mismatch")
if metrics["safe_reduce_contour_provenance_hole_aware_writable_replacement_checks"] != metrics["safe_reduce_hole_aware_writable_replacement_checks"]:
    raise SystemExit("safe reduce provenance hole-aware writable mismatch")
if metrics["safe_reduce_contour_provenance_phase1_unwritable_replacement_checks"] != metrics["safe_reduce_phase1_unwritable_replacement_checks"]:
    raise SystemExit("safe reduce provenance phase-one unwritable mismatch")
if metrics["safe_reduce_contour_provenance_hole_aware_unwritable_replacement_checks"] != metrics["safe_reduce_hole_aware_unwritable_replacement_checks"]:
    raise SystemExit("safe reduce provenance hole-aware unwritable mismatch")
if metrics["safe_reduce_contour_provenance_replacement_edge_incidence_checks"] != metrics["safe_reduce_replacement_edge_incidence_checks"]:
    raise SystemExit("safe reduce provenance local incidence check mismatch")
if metrics["safe_reduce_contour_provenance_replacement_edge_incidence_rejected"] != metrics["safe_reduce_replacement_edge_incidence_rejected"]:
    raise SystemExit("safe reduce provenance local incidence rejection mismatch")
if metrics["safe_reduce_contour_provenance_phase1_replacement_edge_incidence_rejected"] != metrics["safe_reduce_phase1_replacement_edge_incidence_rejected"]:
    raise SystemExit("safe reduce provenance phase-one local incidence mismatch")
if metrics["safe_reduce_contour_provenance_hole_aware_replacement_edge_incidence_rejected"] != metrics["safe_reduce_hole_aware_replacement_edge_incidence_rejected"]:
    raise SystemExit("safe reduce provenance hole-aware local incidence mismatch")
if metrics["safe_reduce_contour_provenance_rejected_boundary_components"] != metrics["safe_reduce_rejected_boundary_components"]:
    raise SystemExit("safe reduce provenance rejected-boundary component mismatch")
if metrics["safe_reduce_contour_provenance_rejected_no_savings_components"] != metrics["safe_reduce_rejected_no_savings_components"]:
    raise SystemExit("safe reduce provenance rejected-no-savings component mismatch")
if metrics["safe_reduce_contour_provenance_rejected_triangulation_components"] != metrics["safe_reduce_rejected_triangulation_components"]:
    raise SystemExit("safe reduce provenance rejected-triangulation component mismatch")
if metrics["safe_reduce_contour_provenance_component_metric_mismatches"] != 0:
    raise SystemExit("safe reduce provenance component metric mismatch")
if metrics["safe_reduce_contour_provenance_component_parity"] != 1:
    raise SystemExit("safe reduce provenance component parity failed")
if metrics["safe_reduce_hole_aware_apply_requested"] != 0:
    raise SystemExit("safe reduce report unexpectedly requested hole-aware apply")
if metrics["safe_reduce_boundary_cosimplify_apply_requested"] != 0:
    raise SystemExit("safe reduce report unexpectedly requested boundary co-simplify apply")
if metrics["safe_reduce_boundary_cosimplify_candidate_rolled_back"] != 0:
    raise SystemExit("safe reduce report unexpectedly rolled back boundary co-simplify")
if metrics["safe_reduce_boundary_cosimplify_fallback_used"] != 0:
    raise SystemExit("safe reduce report unexpectedly used boundary co-simplify fallback")
if metrics["safe_reduce_trust_provenance_neighbors_requested"] != 0:
    raise SystemExit("safe reduce report unexpectedly requested trusted provenance")
if metrics["safe_reduce_trusted_provenance_neighbors_eligible"] != 0:
    raise SystemExit("safe reduce report unexpectedly found trusted provenance eligible")
for key in (
    "safe_reduce_trusted_provenance_rejected_no_triangle_surface",
    "safe_reduce_trusted_provenance_rejected_no_provenance",
    "safe_reduce_trusted_provenance_rejected_no_cached_neighbors",
    "safe_reduce_trusted_provenance_rejected_triangle_mismatch",
    "safe_reduce_trusted_provenance_rejected_incomplete",
    "safe_reduce_trusted_provenance_rejected_unknown",
    "safe_reduce_trusted_provenance_rejected_non_watertight",
    "safe_reduce_trusted_provenance_rejected_orientation",
    "safe_reduce_trusted_provenance_rejected_raw_topology",
    "safe_reduce_trusted_provenance_rejected_raw_welded_spread",
    "safe_reduce_trusted_provenance_rejected_neighbor_size",
    "safe_reduce_trusted_provenance_rejected_neighbor_open_slot",
    "safe_reduce_trusted_provenance_rejected_neighbor_range",
    "safe_reduce_trusted_provenance_rejected_neighbor_self",
    "safe_reduce_trusted_provenance_rejected_neighbor_duplicate",
    "safe_reduce_trusted_provenance_rejected_neighbor_asymmetry",
    "safe_reduce_trusted_provenance_rejected_neighbor_edge_mismatch",
):
    if metrics[key] != 0:
        raise SystemExit(f"safe reduce report unexpectedly rejected trusted provenance via {key}")
if metrics["safe_reduce_trusted_provenance_neighbors_used"] != 0:
    raise SystemExit("safe reduce report unexpectedly used trusted provenance")
if metrics["safe_reduce_trusted_provenance_neighbor_slots_checked"] != 0:
    raise SystemExit("safe reduce report unexpectedly checked trusted neighbor slots")
if metrics["safe_reduce_trusted_provenance_neighbor_edge_slots_checked"] != 0:
    raise SystemExit("safe reduce report unexpectedly checked trusted neighbor edge slots")
if metrics["safe_reduce_trusted_provenance_neighbor_edge_mismatches"] != 0:
    raise SystemExit("safe reduce report unexpectedly found trusted neighbor edge mismatches")
if metrics["safe_reduce_default_adjacency_skipped"] != 0:
    raise SystemExit("safe reduce report unexpectedly skipped default adjacency")
if metrics["safe_reduce_validation_topology_worse"] != 0:
    raise SystemExit("safe reduce report unexpectedly worsened topology")
if metrics["safe_reduce_validation_degenerate_worse"] != 0:
    raise SystemExit("safe reduce report unexpectedly worsened degenerates")
if metrics["safe_reduce_validation_orientation_worse"] != 0:
    raise SystemExit("safe reduce report unexpectedly worsened orientation")
if metrics["safe_reduce_validation_vertex_count_mismatch"] != 0:
    raise SystemExit("safe reduce report unexpectedly mismatched candidate vertex count")
if metrics["safe_reduce_validation_normal_count_mismatch"] != 0:
    raise SystemExit("safe reduce report unexpectedly mismatched candidate normal count")
if metrics["safe_reduce_validation_triangle_count_mismatch"] != 0:
    raise SystemExit("safe reduce report unexpectedly mismatched candidate triangle count")
if metrics["safe_reduce_validation_rolled_back"] != 0:
    raise SystemExit("safe reduce report unexpectedly rolled back")
if metrics["safe_reduce_validation_expected_output_triangles"] != 0:
    raise SystemExit("safe reduce report unexpectedly planned candidate output triangles")
if metrics["safe_reduce_validation_candidate_triangles"] != 0:
    raise SystemExit("safe reduce report unexpectedly built candidate triangles")
if metrics["safe_reduce_validation_candidate_checked"] != 0:
    raise SystemExit("safe reduce report unexpectedly checked a candidate")
if metrics["safe_reduce_validation_candidate_boundary_edges"] != 0:
    raise SystemExit("safe reduce report recorded candidate boundary edges")
if metrics["safe_reduce_validation_candidate_nonmanifold_edges"] != 0:
    raise SystemExit("safe reduce report recorded candidate nonmanifold edges")
if metrics["safe_reduce_validation_candidate_misoriented_edges"] != 0:
    raise SystemExit("safe reduce report recorded candidate misoriented edges")
if metrics["safe_reduce_validation_candidate_degenerate_triangles"] != 0:
    raise SystemExit("safe reduce report recorded candidate degenerates")
if metrics["safe_reduce_validation_candidate_watertight"] != 0:
    raise SystemExit("safe reduce report recorded candidate watertightness")
PY

"$camsim" \
  --threads 2 \
  --resolution 1 \
  --safe-reduce \
  --no-export \
  --profile "$tmp/reduce-profile.json" \
  examples/cat/cat.camotics \
  >"$tmp/reduce-camsim.log" 2>&1

grep -q "Safe reduction:" "$tmp/reduce-camsim.log"

python3 - "$tmp/reduce-profile.json" <<'PY'
import json
import sys

profile = json.load(open(sys.argv[1]))
metrics = profile.get("metrics", {})

required = [
    "safe_reduce_coord_tolerance_scaled_1e6",
    "safe_reduce_plane_distance_tolerance_scaled_1e6",
    "safe_reduce_pairwise_normal_angle_millidegrees",
    "safe_reduce_input_triangles",
    "safe_reduce_output_triangles",
    "safe_reduce_input_boundary_edges",
    "safe_reduce_input_misoriented_edges",
    "safe_reduce_input_degenerate_triangles",
    "safe_reduce_input_watertight",
    "safe_reduce_source_expected_floats",
    "safe_reduce_source_vertex_floats",
    "safe_reduce_source_normal_floats",
    "safe_reduce_source_vertex_count_mismatch",
    "safe_reduce_source_normal_count_mismatch",
    "safe_reduce_output_boundary_edges",
    "safe_reduce_output_misoriented_edges",
    "safe_reduce_output_degenerate_triangles",
    "safe_reduce_output_nonmanifold_edges",
    "safe_reduce_output_watertight",
    "safe_reduce_applied_components",
    "safe_reduce_applied_source_triangles",
    "safe_reduce_applied_output_triangles",
    "safe_reduce_replacement_edge_incidence_checks",
    "safe_reduce_replacement_edge_incidence_rejected",
    "safe_reduce_phase1_replacement_edge_incidence_rejected",
    "safe_reduce_hole_aware_replacement_edge_incidence_rejected",
    "safe_reduce_rejected_triangulation_components",
    "safe_reduce_rejected_triangulation_triangles",
    "safe_reduce_contour_provenance_available",
    "safe_reduce_contour_provenance_triangles",
    "safe_reduce_contour_provenance_complete_triangles",
    "safe_reduce_contour_provenance_unknown_triangles",
    "safe_reduce_contour_provenance_boundary_edges",
    "safe_reduce_contour_provenance_nonmanifold_edges",
    "safe_reduce_contour_provenance_misoriented_edges",
    "safe_reduce_contour_provenance_watertight",
    "safe_reduce_contour_provenance_matches_input",
    "safe_reduce_contour_provenance_neighbors_available",
    "safe_reduce_contour_provenance_neighbors_cached",
    "safe_reduce_contour_provenance_neighbors_raw",
    "safe_reduce_contour_provenance_neighbor_slots",
    "safe_reduce_contour_provenance_neighbor_mismatches",
    "safe_reduce_contour_provenance_neighbor_parity_audited",
    "safe_reduce_contour_provenance_neighbor_parity",
    "safe_reduce_provenance_neighbors_requested",
    "safe_reduce_using_provenance_neighbors",
    "safe_reduce_trust_provenance_neighbors_requested",
    "safe_reduce_trusted_provenance_neighbors_eligible",
    "safe_reduce_trusted_provenance_rejected_no_triangle_surface",
    "safe_reduce_trusted_provenance_rejected_no_provenance",
    "safe_reduce_trusted_provenance_rejected_no_cached_neighbors",
    "safe_reduce_trusted_provenance_rejected_triangle_mismatch",
    "safe_reduce_trusted_provenance_rejected_incomplete",
    "safe_reduce_trusted_provenance_rejected_unknown",
    "safe_reduce_trusted_provenance_rejected_non_watertight",
    "safe_reduce_trusted_provenance_rejected_orientation",
    "safe_reduce_trusted_provenance_rejected_raw_topology",
    "safe_reduce_trusted_provenance_rejected_raw_welded_spread",
    "safe_reduce_trusted_provenance_rejected_neighbor_size",
    "safe_reduce_trusted_provenance_rejected_neighbor_open_slot",
    "safe_reduce_trusted_provenance_rejected_neighbor_range",
    "safe_reduce_trusted_provenance_rejected_neighbor_self",
    "safe_reduce_trusted_provenance_rejected_neighbor_duplicate",
    "safe_reduce_trusted_provenance_rejected_neighbor_asymmetry",
    "safe_reduce_trusted_provenance_rejected_neighbor_edge_mismatch",
    "safe_reduce_trusted_provenance_neighbor_slots_checked",
    "safe_reduce_trusted_provenance_neighbor_edge_slots_checked",
    "safe_reduce_trusted_provenance_neighbor_edge_mismatches",
    "safe_reduce_trusted_provenance_neighbors_used",
    "safe_reduce_hole_aware_apply_requested",
    "safe_reduce_hole_aware_applied_components",
    "safe_reduce_hole_aware_applied_source_triangles",
    "safe_reduce_hole_aware_applied_output_triangles",
    "safe_reduce_validation_topology_worse",
    "safe_reduce_validation_degenerate_worse",
    "safe_reduce_validation_orientation_worse",
    "safe_reduce_validation_vertex_count_mismatch",
    "safe_reduce_validation_normal_count_mismatch",
    "safe_reduce_validation_triangle_count_mismatch",
    "safe_reduce_validation_rolled_back",
    "safe_reduce_validation_expected_output_triangles",
    "safe_reduce_validation_candidate_triangles",
    "safe_reduce_validation_candidate_checked",
    "safe_reduce_validation_candidate_boundary_edges",
    "safe_reduce_validation_candidate_nonmanifold_edges",
    "safe_reduce_validation_candidate_misoriented_edges",
    "safe_reduce_validation_candidate_degenerate_triangles",
    "safe_reduce_validation_candidate_watertight",
]

missing = [name for name in required if name not in metrics]
if missing:
    raise SystemExit(f"missing safe reduce apply metrics: {missing}")

if metrics["safe_reduce_input_watertight"] != 1:
    raise SystemExit("cat reference surface should be watertight before safe reduce")
if metrics["safe_reduce_source_expected_floats"] != metrics["safe_reduce_input_triangles"] * 9:
    raise SystemExit("safe reduce apply source expected-float count mismatch")
if metrics["safe_reduce_source_vertex_floats"] != metrics["safe_reduce_source_expected_floats"]:
    raise SystemExit("safe reduce apply source vertex float count mismatch")
if metrics["safe_reduce_source_normal_floats"] != metrics["safe_reduce_source_expected_floats"]:
    raise SystemExit("safe reduce apply source normal float count mismatch")
if metrics["safe_reduce_source_vertex_count_mismatch"] != 0:
    raise SystemExit("safe reduce apply unexpectedly flagged source vertices")
if metrics["safe_reduce_source_normal_count_mismatch"] != 0:
    raise SystemExit("safe reduce apply unexpectedly flagged source normals")
if metrics["safe_reduce_coord_tolerance_scaled_1e6"] != 100:
    raise SystemExit("safe reduce apply coord tolerance default changed")
if metrics["safe_reduce_plane_distance_tolerance_scaled_1e6"] != 100:
    raise SystemExit("safe reduce apply plane tolerance default changed")
if metrics["safe_reduce_pairwise_normal_angle_millidegrees"] != 250:
    raise SystemExit("safe reduce apply normal angle default changed")
if metrics["safe_reduce_input_boundary_edges"] != 0:
    raise SystemExit("cat reference surface should have no boundary edges before safe reduce")
if metrics["safe_reduce_input_misoriented_edges"] != 0:
    raise SystemExit("cat reference surface should have no misoriented edges before safe reduce")
if metrics["safe_reduce_input_degenerate_triangles"] != 0:
    raise SystemExit("cat reference surface should have no degenerates before safe reduce")
if metrics["safe_reduce_output_watertight"] != 1:
    raise SystemExit("safe reduce output should be watertight")
if metrics["safe_reduce_output_boundary_edges"] != 0:
    raise SystemExit("safe reduce output should have no boundary edges")
if metrics["safe_reduce_output_nonmanifold_edges"] != 0:
    raise SystemExit("safe reduce output should have no nonmanifold edges")
if metrics["safe_reduce_output_misoriented_edges"] != 0:
    raise SystemExit("safe reduce output should have no misoriented edges")
if metrics["safe_reduce_output_degenerate_triangles"] != 0:
    raise SystemExit("safe reduce output should have no degenerate triangles")
if metrics["safe_reduce_applied_components"] <= 0:
    raise SystemExit("safe reduce did not apply any component on cat")
if metrics["safe_reduce_applied_output_triangles"] <= 0:
    raise SystemExit("safe reduce applied no output triangles")
if metrics["safe_reduce_applied_source_triangles"] <= metrics["safe_reduce_applied_output_triangles"]:
    raise SystemExit("safe reduce did not reduce applied component triangles")
if metrics["safe_reduce_output_triangles"] >= metrics["safe_reduce_input_triangles"]:
    raise SystemExit("safe reduce did not lower final triangle count")
if (
    metrics["safe_reduce_phase1_replacement_edge_incidence_rejected"]
    + metrics["safe_reduce_hole_aware_replacement_edge_incidence_rejected"]
    != metrics["safe_reduce_replacement_edge_incidence_rejected"]
):
    raise SystemExit("safe reduce apply local incidence rejection buckets changed")
if metrics["safe_reduce_replacement_edge_incidence_checks"] < metrics["safe_reduce_applied_components"]:
    raise SystemExit("safe reduce apply used more components than local edge-incidence checks")
if metrics["safe_reduce_contour_provenance_available"] != 1:
    raise SystemExit("safe reduce apply did not see contour provenance")
if metrics["safe_reduce_contour_provenance_triangles"] != metrics["safe_reduce_input_triangles"]:
    raise SystemExit("safe reduce apply provenance triangle count mismatch")
if metrics["safe_reduce_contour_provenance_complete_triangles"] != metrics["safe_reduce_input_triangles"]:
    raise SystemExit("safe reduce apply provenance is incomplete")
if metrics["safe_reduce_contour_provenance_unknown_triangles"] != 0:
    raise SystemExit("safe reduce apply provenance has unknown triangles")
if metrics["safe_reduce_contour_provenance_boundary_edges"] != metrics["safe_reduce_input_boundary_edges"]:
    raise SystemExit("safe reduce apply provenance boundary edges mismatch input")
if metrics["safe_reduce_contour_provenance_nonmanifold_edges"] != metrics["safe_reduce_input_nonmanifold_edges"]:
    raise SystemExit("safe reduce apply provenance nonmanifold edges mismatch input")
if metrics["safe_reduce_contour_provenance_misoriented_edges"] != metrics["safe_reduce_input_misoriented_edges"]:
    raise SystemExit("safe reduce apply provenance misoriented edges mismatch input")
if metrics["safe_reduce_contour_provenance_watertight"] != metrics["safe_reduce_input_watertight"]:
    raise SystemExit("safe reduce apply provenance watertightness mismatch input")
if metrics["safe_reduce_contour_provenance_matches_input"] != 1:
    raise SystemExit("safe reduce apply provenance did not match input topology")
if metrics["safe_reduce_provenance_neighbors_requested"] != 0:
    raise SystemExit("safe reduce apply unexpectedly requested provenance neighbors")
if metrics["safe_reduce_using_provenance_neighbors"] != 0:
    raise SystemExit("safe reduce apply unexpectedly used provenance neighbors")
if metrics["safe_reduce_trust_provenance_neighbors_requested"] != 0:
    raise SystemExit("safe reduce apply unexpectedly requested trusted provenance")
if metrics["safe_reduce_trusted_provenance_neighbors_eligible"] != 0:
    raise SystemExit("safe reduce apply unexpectedly found trusted provenance eligible")
for key in (
    "safe_reduce_trusted_provenance_rejected_no_triangle_surface",
    "safe_reduce_trusted_provenance_rejected_no_provenance",
    "safe_reduce_trusted_provenance_rejected_no_cached_neighbors",
    "safe_reduce_trusted_provenance_rejected_triangle_mismatch",
    "safe_reduce_trusted_provenance_rejected_incomplete",
    "safe_reduce_trusted_provenance_rejected_unknown",
    "safe_reduce_trusted_provenance_rejected_non_watertight",
    "safe_reduce_trusted_provenance_rejected_orientation",
    "safe_reduce_trusted_provenance_rejected_raw_topology",
    "safe_reduce_trusted_provenance_rejected_raw_welded_spread",
    "safe_reduce_trusted_provenance_rejected_neighbor_size",
    "safe_reduce_trusted_provenance_rejected_neighbor_open_slot",
    "safe_reduce_trusted_provenance_rejected_neighbor_range",
    "safe_reduce_trusted_provenance_rejected_neighbor_self",
    "safe_reduce_trusted_provenance_rejected_neighbor_duplicate",
    "safe_reduce_trusted_provenance_rejected_neighbor_asymmetry",
    "safe_reduce_trusted_provenance_rejected_neighbor_edge_mismatch",
):
    if metrics[key] != 0:
        raise SystemExit(f"safe reduce apply unexpectedly rejected trusted provenance via {key}")
if metrics["safe_reduce_trusted_provenance_neighbors_used"] != 0:
    raise SystemExit("safe reduce apply unexpectedly used trusted provenance")
if metrics["safe_reduce_trusted_provenance_neighbor_slots_checked"] != 0:
    raise SystemExit("safe reduce apply unexpectedly checked trusted neighbor slots")
if metrics["safe_reduce_trusted_provenance_neighbor_edge_slots_checked"] != 0:
    raise SystemExit("safe reduce apply unexpectedly checked trusted neighbor edge slots")
if metrics["safe_reduce_trusted_provenance_neighbor_edge_mismatches"] != 0:
    raise SystemExit("safe reduce apply unexpectedly found trusted neighbor edge mismatches")
if metrics["safe_reduce_hole_aware_apply_requested"] != 0:
    raise SystemExit("safe reduce apply unexpectedly requested hole-aware apply")
if metrics["safe_reduce_boundary_cosimplify_apply_requested"] != 0:
    raise SystemExit("safe reduce apply unexpectedly requested boundary co-simplify apply")
if metrics["safe_reduce_boundary_cosimplify_candidate_rolled_back"] != 0:
    raise SystemExit("safe reduce apply unexpectedly rolled back boundary co-simplify")
if metrics["safe_reduce_boundary_cosimplify_fallback_used"] != 0:
    raise SystemExit("safe reduce apply unexpectedly used boundary co-simplify fallback")
if metrics["safe_reduce_hole_aware_applied_components"] != 0:
    raise SystemExit("safe reduce default unexpectedly applied hole-aware components")
if metrics["safe_reduce_hole_aware_applied_source_triangles"] != 0:
    raise SystemExit("safe reduce default hole-aware source triangles changed")
if metrics["safe_reduce_hole_aware_applied_output_triangles"] != 0:
    raise SystemExit("safe reduce default hole-aware output triangles changed")
if metrics["safe_reduce_validation_topology_worse"] != 0:
    raise SystemExit("safe reduce apply unexpectedly worsened topology")
if metrics["safe_reduce_validation_degenerate_worse"] != 0:
    raise SystemExit("safe reduce apply unexpectedly worsened degenerates")
if metrics["safe_reduce_validation_orientation_worse"] != 0:
    raise SystemExit("safe reduce apply unexpectedly worsened orientation")
if metrics["safe_reduce_validation_vertex_count_mismatch"] != 0:
    raise SystemExit("safe reduce apply candidate vertex count mismatch")
if metrics["safe_reduce_validation_normal_count_mismatch"] != 0:
    raise SystemExit("safe reduce apply candidate normal count mismatch")
if metrics["safe_reduce_validation_triangle_count_mismatch"] != 0:
    raise SystemExit("safe reduce apply candidate triangle count mismatch")
if metrics["safe_reduce_validation_rolled_back"] != 0:
    raise SystemExit("safe reduce apply unexpectedly rolled back")
if metrics["safe_reduce_validation_expected_output_triangles"] != metrics["safe_reduce_output_triangles"]:
    raise SystemExit("safe reduce apply expected output triangle count mismatch")
if metrics["safe_reduce_validation_candidate_triangles"] != metrics["safe_reduce_output_triangles"]:
    raise SystemExit("safe reduce apply candidate triangle count mismatch")
if metrics["safe_reduce_validation_candidate_triangles"] != metrics["safe_reduce_validation_expected_output_triangles"]:
    raise SystemExit("safe reduce apply candidate/expected triangle count mismatch")
if metrics["safe_reduce_validation_candidate_checked"] != 1:
    raise SystemExit("safe reduce apply did not validate a candidate")
if metrics["safe_reduce_validation_candidate_boundary_edges"] != metrics["safe_reduce_output_boundary_edges"]:
    raise SystemExit("safe reduce apply candidate boundary count mismatch")
if metrics["safe_reduce_validation_candidate_nonmanifold_edges"] != metrics["safe_reduce_output_nonmanifold_edges"]:
    raise SystemExit("safe reduce apply candidate nonmanifold count mismatch")
if metrics["safe_reduce_validation_candidate_misoriented_edges"] != metrics["safe_reduce_output_misoriented_edges"]:
    raise SystemExit("safe reduce apply candidate misoriented edge count mismatch")
if metrics["safe_reduce_validation_candidate_degenerate_triangles"] != metrics["safe_reduce_output_degenerate_triangles"]:
    raise SystemExit("safe reduce apply candidate degenerate count mismatch")
if metrics["safe_reduce_validation_candidate_watertight"] != metrics["safe_reduce_output_watertight"]:
    raise SystemExit("safe reduce apply candidate watertightness mismatch")
if metrics["safe_reduce_contour_provenance_neighbors_available"] != 0:
    raise SystemExit("safe reduce apply built provenance neighbors by default")
if metrics["safe_reduce_contour_provenance_neighbors_cached"] != 0:
    raise SystemExit("safe reduce apply should not use cached provenance neighbors by default")
if metrics["safe_reduce_contour_provenance_neighbors_raw"] != 0:
    raise SystemExit("safe reduce apply default provenance neighbors raw should be zero")
if metrics["safe_reduce_contour_provenance_neighbor_slots"] != 0:
    raise SystemExit("safe reduce apply default provenance neighbor slots should be zero")
if metrics["safe_reduce_contour_provenance_neighbor_mismatches"] != 0:
    raise SystemExit("safe reduce apply provenance neighbor mismatch")
if metrics["safe_reduce_contour_provenance_neighbor_parity"] != 0:
    raise SystemExit("safe reduce apply default provenance neighbor parity should be zero")
PY

"$camsim" \
  --threads 2 \
  --resolution 1 \
  --safe-reduce \
  --safe-reduce-boundary-cosimplify \
  --no-export \
  --profile "$tmp/boundary-cosimplify-profile.json" \
  examples/cat/cat.camotics \
  >"$tmp/boundary-cosimplify-camsim.log" 2>&1

grep -q "Safe reduction:" "$tmp/boundary-cosimplify-camsim.log"

python3 - "$tmp/reduce-profile.json" "$tmp/boundary-cosimplify-profile.json" <<'PY'
import json
import sys

default = json.load(open(sys.argv[1])).get("metrics", {})
cosimplify = json.load(open(sys.argv[2])).get("metrics", {})

required = [
    "safe_reduce_boundary_cosimplify_apply_requested",
    "safe_reduce_boundary_cosimplify_candidate_rolled_back",
    "safe_reduce_boundary_cosimplify_fallback_used",
    "safe_reduce_boundary_cosimplify_candidate_components",
    "safe_reduce_boundary_cosimplify_estimated_extra_reduction",
    "safe_reduce_boundary_cosimplify_contract_vertices_considered",
    "safe_reduce_boundary_cosimplify_contract_vertices_accepted",
    "safe_reduce_boundary_cosimplify_contract_rejected_single_sided",
    "safe_reduce_boundary_cosimplify_contract_rejected_ambiguous",
    "safe_reduce_boundary_cosimplify_contract_rejected_non_collinear",
    "safe_reduce_boundary_cosimplify_contract_rejected_ineligible",
    "safe_reduce_boundary_cosimplify_contract_rejected_ownership",
    "safe_reduce_boundary_cosimplify_contract_components_considered",
    "safe_reduce_boundary_cosimplify_contract_components_affected",
    "safe_reduce_boundary_cosimplify_contract_replacement_checks",
    "safe_reduce_boundary_cosimplify_contract_triangulation_rejected",
    "safe_reduce_boundary_cosimplify_contract_edge_incidence_rejected",
    "safe_reduce_boundary_cosimplify_contract_no_savings_rejected",
    "safe_reduce_boundary_cosimplify_contract_global_rejected",
    "safe_reduce_boundary_cosimplify_contract_applied_components",
    "safe_reduce_boundary_cosimplify_contract_applied_source_triangles",
    "safe_reduce_boundary_cosimplify_contract_applied_output_triangles",
    "safe_reduce_input_triangles",
    "safe_reduce_output_triangles",
    "safe_reduce_applied_components",
    "safe_reduce_applied_source_triangles",
    "safe_reduce_applied_output_triangles",
    "safe_reduce_validation_rolled_back",
    "safe_reduce_output_watertight",
    "safe_reduce_output_boundary_edges",
    "safe_reduce_output_nonmanifold_edges",
    "safe_reduce_output_misoriented_edges",
    "safe_reduce_output_degenerate_triangles",
]
missing = [name for name in required if name not in cosimplify]
if missing:
    raise SystemExit(f"missing boundary co-simplify fallback metrics: {missing}")

if cosimplify["safe_reduce_boundary_cosimplify_apply_requested"] != 1:
    raise SystemExit("boundary co-simplify request was not recorded")
if cosimplify["safe_reduce_boundary_cosimplify_candidate_components"] <= 0:
    raise SystemExit("boundary co-simplify saw no candidates")
if cosimplify["safe_reduce_boundary_cosimplify_estimated_extra_reduction"] <= 0:
    raise SystemExit("boundary co-simplify saw no estimated extra reduction")
if cosimplify["safe_reduce_boundary_cosimplify_contract_vertices_considered"] <= 0:
    raise SystemExit("boundary co-simplify contract saw no boundary vertices")
if cosimplify["safe_reduce_boundary_cosimplify_candidate_rolled_back"] != 0:
    raise SystemExit("boundary co-simplify unexpectedly rolled back on cat")
if cosimplify["safe_reduce_boundary_cosimplify_fallback_used"] != 0:
    raise SystemExit("boundary co-simplify unexpectedly used fallback on cat")
if cosimplify["safe_reduce_validation_rolled_back"] != 0:
    raise SystemExit("boundary co-simplify left final validation rolled back")
if cosimplify["safe_reduce_input_triangles"] != default["safe_reduce_input_triangles"]:
    raise SystemExit("boundary co-simplify changed input triangle count")
if cosimplify["safe_reduce_output_triangles"] > default["safe_reduce_output_triangles"]:
    raise SystemExit("boundary co-simplify increased output triangle count")
if cosimplify["safe_reduce_output_watertight"] != 1:
    raise SystemExit("boundary co-simplify output was not watertight")
for key in (
    "safe_reduce_output_boundary_edges",
    "safe_reduce_output_nonmanifold_edges",
    "safe_reduce_output_misoriented_edges",
    "safe_reduce_output_degenerate_triangles",
):
    if cosimplify[key] != 0:
        raise SystemExit(f"boundary co-simplify produced bad topology metric {key}")
PY

cat > "$tmp/box.nc" <<'NC'
G21
M6 T1
G0 X0 Y0 Z5
NC

cat > "$tmp/box.camotics" <<'JSON'
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 0.5,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 4,
      "diameter": 1
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {
      "min": [-5, -5, -2],
      "max": [5, 5, 0]
    }
  },
  "files": ["box.nc"]
}
JSON

"$camsim" \
  --threads 1 \
  --safe-reduce \
  --no-export \
  --profile "$tmp/box-reduce-profile.json" \
  "$tmp/box.camotics" \
  >"$tmp/box-reduce-camsim.log" 2>&1

"$camsim" \
  --threads 1 \
  --safe-reduce \
  --safe-reduce-boundary-cosimplify \
  --no-export \
  --profile "$tmp/box-boundary-cosimplify-profile.json" \
  "$tmp/box.camotics" \
  >"$tmp/box-boundary-cosimplify-camsim.log" 2>&1

python3 - "$tmp/box-reduce-profile.json" \
  "$tmp/box-boundary-cosimplify-profile.json" <<'PY'
import json
import sys

default = json.load(open(sys.argv[1])).get("metrics", {})
cosimplify = json.load(open(sys.argv[2])).get("metrics", {})

required = [
    "safe_reduce_input_triangles",
    "safe_reduce_output_triangles",
    "safe_reduce_boundary_cosimplify_contract_interface_edges",
    "safe_reduce_boundary_cosimplify_contract_chain_interfaces",
    "safe_reduce_boundary_cosimplify_contract_chains",
    "safe_reduce_boundary_cosimplify_contract_chain_vertices",
    "safe_reduce_boundary_cosimplify_contract_chain_interior_vertices",
    "safe_reduce_boundary_cosimplify_contract_chain_vertices_accepted",
    "safe_reduce_boundary_cosimplify_contract_applied_components",
    "safe_reduce_boundary_cosimplify_contract_applied_source_triangles",
    "safe_reduce_boundary_cosimplify_contract_applied_output_triangles",
    "safe_reduce_side_x_min_applied_output_triangles",
    "safe_reduce_side_x_max_applied_output_triangles",
    "safe_reduce_side_y_min_applied_output_triangles",
    "safe_reduce_side_y_max_applied_output_triangles",
    "safe_reduce_side_z_min_applied_source_triangles",
    "safe_reduce_side_z_min_applied_output_triangles",
    "safe_reduce_boundary_cosimplify_candidate_rolled_back",
    "safe_reduce_boundary_cosimplify_fallback_used",
    "safe_reduce_validation_rolled_back",
    "safe_reduce_output_watertight",
    "safe_reduce_output_boundary_edges",
    "safe_reduce_output_nonmanifold_edges",
    "safe_reduce_output_misoriented_edges",
    "safe_reduce_output_degenerate_triangles",
]
missing = [name for name in required if name not in cosimplify]
if missing:
    raise SystemExit(f"missing generated-box boundary co-simplify metrics: {missing}")

if cosimplify["safe_reduce_input_triangles"] != default["safe_reduce_input_triangles"]:
    raise SystemExit("generated-box co-simplify changed input triangle count")
if cosimplify["safe_reduce_output_triangles"] >= default["safe_reduce_output_triangles"]:
    raise SystemExit("generated-box co-simplify did not reduce output triangles")
if cosimplify["safe_reduce_boundary_cosimplify_contract_interface_edges"] <= 0:
    raise SystemExit("generated-box co-simplify found no contract interfaces")
if cosimplify["safe_reduce_boundary_cosimplify_contract_chain_interfaces"] <= 0:
    raise SystemExit("generated-box co-simplify found no chain interfaces")
if cosimplify["safe_reduce_boundary_cosimplify_contract_chain_vertices_accepted"] <= 0:
    raise SystemExit("generated-box co-simplify accepted no chain vertices")
if cosimplify["safe_reduce_boundary_cosimplify_contract_applied_components"] <= 0:
    raise SystemExit("generated-box co-simplify applied no contract components")
if cosimplify["safe_reduce_boundary_cosimplify_contract_applied_source_triangles"] <= cosimplify["safe_reduce_boundary_cosimplify_contract_applied_output_triangles"]:
    raise SystemExit("generated-box co-simplify contract had no source/output savings")
if cosimplify["safe_reduce_side_z_min_applied_output_triangles"] != 2:
    raise SystemExit("generated-box bottom did not collapse to 2 triangles")
for side in ("x_min", "x_max", "y_min", "y_max"):
    key = f"safe_reduce_side_{side}_applied_output_triangles"
    if cosimplify[key] != 2:
        raise SystemExit(f"generated-box side {side} did not collapse to 2 triangles")
if cosimplify["safe_reduce_boundary_cosimplify_candidate_rolled_back"] != 0:
    raise SystemExit("generated-box co-simplify candidate rolled back")
if cosimplify["safe_reduce_boundary_cosimplify_fallback_used"] != 0:
    raise SystemExit("generated-box co-simplify used fallback")
if cosimplify["safe_reduce_validation_rolled_back"] != 0:
    raise SystemExit("generated-box co-simplify final validation rolled back")
if cosimplify["safe_reduce_output_watertight"] != 1:
    raise SystemExit("generated-box co-simplify output was not watertight")
for key in (
    "safe_reduce_output_boundary_edges",
    "safe_reduce_output_nonmanifold_edges",
    "safe_reduce_output_misoriented_edges",
    "safe_reduce_output_degenerate_triangles",
):
    if cosimplify[key] != 0:
        raise SystemExit(f"generated-box co-simplify produced bad topology metric {key}")
PY

"$camsim" \
  --threads 2 \
  --resolution 1 \
  --safe-reduce \
  --safe-reduce-hole-aware \
  --no-export \
  --profile "$tmp/hole-aware-profile.json" \
  examples/cat/cat.camotics \
  >"$tmp/hole-aware-camsim.log" 2>&1

python3 - "$tmp/reduce-profile.json" "$tmp/hole-aware-profile.json" <<'PY'
import json
import sys

default = json.load(open(sys.argv[1])).get("metrics", {})
hole = json.load(open(sys.argv[2])).get("metrics", {})

required = [
    "safe_reduce_hole_aware_apply_requested",
    "safe_reduce_input_triangles",
    "safe_reduce_source_expected_floats",
    "safe_reduce_source_vertex_floats",
    "safe_reduce_source_normal_floats",
    "safe_reduce_source_vertex_count_mismatch",
    "safe_reduce_source_normal_count_mismatch",
    "safe_reduce_output_triangles",
    "safe_reduce_applied_components",
    "safe_reduce_hole_aware_components",
    "safe_reduce_estimated_replacement_checks",
    "safe_reduce_feasible_replacement_checks",
    "safe_reduce_writable_replacement_checks",
    "safe_reduce_unwritable_replacement_checks",
    "safe_reduce_hole_aware_writable_replacement_checks",
    "safe_reduce_phase1_unwritable_replacement_checks",
    "safe_reduce_hole_aware_unwritable_replacement_checks",
    "safe_reduce_replacement_edge_incidence_checks",
    "safe_reduce_replacement_edge_incidence_rejected",
    "safe_reduce_phase1_replacement_edge_incidence_rejected",
    "safe_reduce_hole_aware_replacement_edge_incidence_rejected",
    "safe_reduce_hole_aware_applied_components",
    "safe_reduce_hole_aware_applied_source_triangles",
    "safe_reduce_hole_aware_applied_output_triangles",
    "safe_reduce_validation_topology_worse",
    "safe_reduce_validation_degenerate_worse",
    "safe_reduce_validation_orientation_worse",
    "safe_reduce_validation_vertex_count_mismatch",
    "safe_reduce_validation_normal_count_mismatch",
    "safe_reduce_validation_triangle_count_mismatch",
    "safe_reduce_validation_rolled_back",
    "safe_reduce_validation_expected_output_triangles",
    "safe_reduce_validation_candidate_triangles",
    "safe_reduce_validation_candidate_checked",
    "safe_reduce_validation_candidate_boundary_edges",
    "safe_reduce_validation_candidate_nonmanifold_edges",
    "safe_reduce_validation_candidate_misoriented_edges",
    "safe_reduce_validation_candidate_degenerate_triangles",
    "safe_reduce_validation_candidate_watertight",
    "safe_reduce_output_watertight",
    "safe_reduce_output_boundary_edges",
    "safe_reduce_output_misoriented_edges",
    "safe_reduce_output_degenerate_triangles",
    "safe_reduce_output_nonmanifold_edges",
]
missing = [name for name in required if name not in hole]
if missing:
    raise SystemExit(f"missing hole-aware safe reduce metrics: {missing}")

if hole["safe_reduce_hole_aware_apply_requested"] != 1:
    raise SystemExit("safe reduce hole-aware flag was not recorded")
if hole["safe_reduce_input_triangles"] != default["safe_reduce_input_triangles"]:
    raise SystemExit("safe reduce hole-aware input triangle count changed")
if hole["safe_reduce_source_expected_floats"] != hole["safe_reduce_input_triangles"] * 9:
    raise SystemExit("safe reduce hole-aware source expected-float count mismatch")
if hole["safe_reduce_source_vertex_floats"] != hole["safe_reduce_source_expected_floats"]:
    raise SystemExit("safe reduce hole-aware source vertex float count mismatch")
if hole["safe_reduce_source_normal_floats"] != hole["safe_reduce_source_expected_floats"]:
    raise SystemExit("safe reduce hole-aware source normal float count mismatch")
if hole["safe_reduce_source_vertex_count_mismatch"] != 0:
    raise SystemExit("safe reduce hole-aware unexpectedly flagged source vertices")
if hole["safe_reduce_source_normal_count_mismatch"] != 0:
    raise SystemExit("safe reduce hole-aware unexpectedly flagged source normals")
if hole["safe_reduce_output_triangles"] >= default["safe_reduce_output_triangles"]:
    raise SystemExit("safe reduce hole-aware did not lower output triangle count")
if hole["safe_reduce_applied_components"] <= default["safe_reduce_applied_components"]:
    raise SystemExit("safe reduce hole-aware did not apply extra components")
if hole["safe_reduce_hole_aware_components"] <= 0:
    raise SystemExit("safe reduce hole-aware saw no multi-loop candidates")
if hole["safe_reduce_estimated_replacement_checks"] < hole["safe_reduce_feasible_replacement_checks"]:
    raise SystemExit("safe reduce hole-aware estimated fewer replacements than were feasible")
if hole["safe_reduce_writable_replacement_checks"] != hole["safe_reduce_feasible_replacement_checks"]:
    raise SystemExit("safe reduce hole-aware writable/feasible replacement checks diverged")
if (
    hole["safe_reduce_writable_replacement_checks"]
    + hole["safe_reduce_unwritable_replacement_checks"]
    != hole["safe_reduce_estimated_replacement_checks"]
):
    raise SystemExit("safe reduce hole-aware writable/unwritable accounting changed")
if (
    hole["safe_reduce_phase1_unwritable_replacement_checks"]
    + hole["safe_reduce_hole_aware_unwritable_replacement_checks"]
    != hole["safe_reduce_unwritable_replacement_checks"]
):
    raise SystemExit("safe reduce hole-aware unwritable bucket accounting changed")
if (
    hole["safe_reduce_phase1_replacement_edge_incidence_rejected"]
    + hole["safe_reduce_hole_aware_replacement_edge_incidence_rejected"]
    != hole["safe_reduce_replacement_edge_incidence_rejected"]
):
    raise SystemExit("safe reduce hole-aware local incidence rejection buckets changed")
if hole["safe_reduce_replacement_edge_incidence_checks"] < hole["safe_reduce_writable_replacement_checks"]:
    raise SystemExit("safe reduce hole-aware writable replacements exceeded local edge-incidence checks")
if hole["safe_reduce_hole_aware_writable_replacement_checks"] <= 0:
    raise SystemExit("safe reduce hole-aware produced no writable multi-loop replacements")
if hole["safe_reduce_hole_aware_applied_components"] <= 0:
    raise SystemExit("safe reduce hole-aware applied no multi-loop components")
if hole["safe_reduce_hole_aware_writable_replacement_checks"] < hole["safe_reduce_hole_aware_applied_components"]:
    raise SystemExit("safe reduce hole-aware applied more components than were writable")
if hole["safe_reduce_hole_aware_applied_source_triangles"] <= hole["safe_reduce_hole_aware_applied_output_triangles"]:
    raise SystemExit("safe reduce hole-aware did not reduce applied multi-loop triangles")
if hole["safe_reduce_output_watertight"] != 1:
    raise SystemExit("safe reduce hole-aware output should be watertight")
if hole["safe_reduce_output_boundary_edges"] != 0:
    raise SystemExit("safe reduce hole-aware output should have no boundary edges")
if hole["safe_reduce_output_nonmanifold_edges"] != 0:
    raise SystemExit("safe reduce hole-aware output should have no nonmanifold edges")
if hole["safe_reduce_output_misoriented_edges"] != 0:
    raise SystemExit("safe reduce hole-aware output should have no misoriented edges")
if hole["safe_reduce_output_degenerate_triangles"] != 0:
    raise SystemExit("safe reduce hole-aware output should have no degenerate triangles")
if hole["safe_reduce_validation_topology_worse"] != 0:
    raise SystemExit("safe reduce hole-aware unexpectedly worsened topology")
if hole["safe_reduce_validation_degenerate_worse"] != 0:
    raise SystemExit("safe reduce hole-aware unexpectedly worsened degenerates")
if hole["safe_reduce_validation_orientation_worse"] != 0:
    raise SystemExit("safe reduce hole-aware unexpectedly worsened orientation")
if hole["safe_reduce_validation_vertex_count_mismatch"] != 0:
    raise SystemExit("safe reduce hole-aware candidate vertex count mismatch")
if hole["safe_reduce_validation_normal_count_mismatch"] != 0:
    raise SystemExit("safe reduce hole-aware candidate normal count mismatch")
if hole["safe_reduce_validation_triangle_count_mismatch"] != 0:
    raise SystemExit("safe reduce hole-aware candidate triangle count mismatch")
if hole["safe_reduce_validation_rolled_back"] != 0:
    raise SystemExit("safe reduce hole-aware unexpectedly rolled back")
if hole["safe_reduce_validation_expected_output_triangles"] != hole["safe_reduce_output_triangles"]:
    raise SystemExit("safe reduce hole-aware expected output triangle count mismatch")
if hole["safe_reduce_validation_candidate_triangles"] != hole["safe_reduce_output_triangles"]:
    raise SystemExit("safe reduce hole-aware candidate triangle count mismatch")
if hole["safe_reduce_validation_candidate_triangles"] != hole["safe_reduce_validation_expected_output_triangles"]:
    raise SystemExit("safe reduce hole-aware candidate/expected triangle count mismatch")
if hole["safe_reduce_validation_candidate_checked"] != 1:
    raise SystemExit("safe reduce hole-aware did not validate a candidate")
if hole["safe_reduce_validation_candidate_boundary_edges"] != hole["safe_reduce_output_boundary_edges"]:
    raise SystemExit("safe reduce hole-aware candidate boundary count mismatch")
if hole["safe_reduce_validation_candidate_nonmanifold_edges"] != hole["safe_reduce_output_nonmanifold_edges"]:
    raise SystemExit("safe reduce hole-aware candidate nonmanifold count mismatch")
if hole["safe_reduce_validation_candidate_misoriented_edges"] != hole["safe_reduce_output_misoriented_edges"]:
    raise SystemExit("safe reduce hole-aware candidate misoriented edge count mismatch")
if hole["safe_reduce_validation_candidate_degenerate_triangles"] != hole["safe_reduce_output_degenerate_triangles"]:
    raise SystemExit("safe reduce hole-aware candidate degenerate count mismatch")
if hole["safe_reduce_validation_candidate_watertight"] != hole["safe_reduce_output_watertight"]:
    raise SystemExit("safe reduce hole-aware candidate watertightness mismatch")
PY

"$camsim" \
  --threads 2 \
  --resolution 1 \
  --safe-reduce-report \
  --safe-reduce-hole-aware \
  --no-export \
  --profile "$tmp/hole-aware-report-profile.json" \
  examples/cat/cat.camotics \
  >"$tmp/hole-aware-report-camsim.log" 2>&1

python3 - "$tmp/reduce-profile.json" "$tmp/hole-aware-report-profile.json" <<'PY'
import json
import sys

default = json.load(open(sys.argv[1])).get("metrics", {})
report = json.load(open(sys.argv[2])).get("metrics", {})

required = [
    "safe_reduce_hole_aware_apply_requested",
    "safe_reduce_input_triangles",
    "safe_reduce_output_triangles",
    "safe_reduce_hole_aware_components",
    "safe_reduce_hole_aware_writable_replacement_checks",
    "safe_reduce_hole_aware_applied_components",
    "safe_reduce_applied_components",
    "safe_reduce_validation_candidate_checked",
]
missing = [name for name in required if name not in report]
if missing:
    raise SystemExit(f"missing hole-aware report metrics: {missing}")

if report["safe_reduce_hole_aware_apply_requested"] != 1:
    raise SystemExit("safe reduce hole-aware report did not record requested mode")
if report["safe_reduce_input_triangles"] != default["safe_reduce_input_triangles"]:
    raise SystemExit("safe reduce hole-aware report input triangle count changed")
if report["safe_reduce_output_triangles"] != report["safe_reduce_input_triangles"]:
    raise SystemExit("safe reduce hole-aware report changed output triangle count")
if report["safe_reduce_hole_aware_components"] <= 0:
    raise SystemExit("safe reduce hole-aware report saw no multi-loop candidates")
if report["safe_reduce_hole_aware_writable_replacement_checks"] <= 0:
    raise SystemExit("safe reduce hole-aware report saw no writable multi-loop candidates")
if report["safe_reduce_applied_components"] != 0:
    raise SystemExit("safe reduce hole-aware report applied components")
if report["safe_reduce_hole_aware_applied_components"] != 0:
    raise SystemExit("safe reduce hole-aware report applied multi-loop components")
if report["safe_reduce_validation_candidate_checked"] != 0:
    raise SystemExit("safe reduce hole-aware report unexpectedly ran apply validation")
PY

"$camsim" \
  --threads 2 \
  --resolution 1 \
  --safe-reduce-report \
  --safe-reduce-plane-tolerance 0.0002 \
  --safe-reduce-normal-angle 0.5 \
  --no-export \
  --profile "$tmp/custom-report-profile.json" \
  examples/cat/cat.camotics \
  >"$tmp/custom-report-camsim.log" 2>&1

python3 - "$tmp/custom-report-profile.json" <<'PY'
import json
import sys

metrics = json.load(open(sys.argv[1])).get("metrics", {})

if metrics.get("safe_reduce_plane_distance_tolerance_scaled_1e6") != 200:
    raise SystemExit("safe reduce custom plane tolerance was not applied")
if metrics.get("safe_reduce_pairwise_normal_angle_millidegrees") != 500:
    raise SystemExit("safe reduce custom normal angle was not applied")
if metrics.get("safe_reduce_input_triangles", 0) <= 0:
    raise SystemExit("safe reduce custom report saw no triangles")
PY

if "$camsim" \
  --threads 1 \
  --resolution 1 \
  --reduce \
  --safe-reduce \
  --no-export \
  examples/cat/cat.camotics \
  >"$tmp/misused-legacy-reduce.log" 2>&1; then
  raise_error="safe reduce worked with legacy reduce enabled"
  echo "$raise_error" >&2
  exit 1
fi

if "$camsim" \
  --threads 1 \
  --resolution 1 \
  --safe-reduce-hole-aware \
  --no-export \
  examples/cat/cat.camotics \
  >"$tmp/misused-hole-aware.log" 2>&1; then
  raise_error="safe reduce hole-aware flag worked without safe-reduce mode"
  echo "$raise_error" >&2
  exit 1
fi

if ! grep -q -- "--safe-reduce or --safe-reduce-report" "$tmp/misused-hole-aware.log"; then
  echo "safe reduce hole-aware misuse message did not mention report mode" >&2
  exit 1
fi

if "$camsim" \
  --threads 1 \
  --resolution 1 \
  --safe-reduce-plane-tolerance 0.0002 \
  --no-export \
  examples/cat/cat.camotics \
  >"$tmp/misused-tolerance.log" 2>&1; then
  raise_error="safe reduce tolerance flag worked without safe-reduce mode"
  echo "$raise_error" >&2
  exit 1
fi

if "$camsim" \
  --threads 1 \
  --resolution 1 \
  --safe-reduce-provenance-neighbors \
  --no-export \
  examples/cat/cat.camotics \
  >"$tmp/misused-provenance-neighbors.log" 2>&1; then
  raise_error="safe reduce provenance neighbors worked without safe-reduce mode"
  echo "$raise_error" >&2
  exit 1
fi

if "$camsim" \
  --threads 1 \
  --resolution 1 \
  --safe-reduce \
  --safe-reduce-trust-provenance-neighbors \
  --no-export \
  examples/cat/cat.camotics \
  >"$tmp/misused-trusted-provenance.log" 2>&1; then
  raise_error="safe reduce trusted provenance worked without provenance-neighbor mode"
  echo "$raise_error" >&2
  exit 1
fi

if "$camsim" \
  --threads 1 \
  --resolution 1 \
  --safe-reduce-report \
  --safe-reduce-plane-tolerance 0 \
  --no-export \
  examples/cat/cat.camotics \
  >"$tmp/bad-plane-tolerance-zero.log" 2>&1; then
  raise_error="safe reduce plane tolerance accepted zero"
  echo "$raise_error" >&2
  exit 1
fi

if "$camsim" \
  --threads 1 \
  --resolution 1 \
  --safe-reduce-report \
  --safe-reduce-plane-tolerance -0.0001 \
  --no-export \
  examples/cat/cat.camotics \
  >"$tmp/bad-plane-tolerance-negative.log" 2>&1; then
  raise_error="safe reduce plane tolerance accepted a negative value"
  echo "$raise_error" >&2
  exit 1
fi

if "$camsim" \
  --threads 1 \
  --resolution 1 \
  --safe-reduce-report \
  --safe-reduce-normal-angle 0 \
  --no-export \
  examples/cat/cat.camotics \
  >"$tmp/bad-normal-angle-zero.log" 2>&1; then
  raise_error="safe reduce normal angle accepted zero"
  echo "$raise_error" >&2
  exit 1
fi

if "$camsim" \
  --threads 1 \
  --resolution 1 \
  --safe-reduce-report \
  --safe-reduce-normal-angle -0.25 \
  --no-export \
  examples/cat/cat.camotics \
  >"$tmp/bad-normal-angle-negative.log" 2>&1; then
  raise_error="safe reduce normal angle accepted a negative value"
  echo "$raise_error" >&2
  exit 1
fi

if "$camsim" \
  --threads 1 \
  --resolution 1 \
  --safe-reduce-report \
  --safe-reduce-normal-angle 6 \
  --no-export \
  examples/cat/cat.camotics \
  >"$tmp/bad-normal-angle.log" 2>&1; then
  raise_error="safe reduce normal angle cap was not enforced"
  echo "$raise_error" >&2
  exit 1
fi

echo "safe reduce report/apply smoke passed"
