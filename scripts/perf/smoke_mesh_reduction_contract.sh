#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

tmp="${TMPDIR:-/tmp}/camotics-mesh-reduction-contract-$$"
mkdir -p "$tmp"
trap 'rm -rf "$tmp"' EXIT

python3 scripts/perf/mesh_reduction_contract_report.py \
  --self-test \
  --output-json "$tmp/self-test.json" \
  >"$tmp/self-test.log"

python3 scripts/perf/compare_stl_cut_centroid_plane.py \
  --self-test \
  >"$tmp/centroid-plane-self-test.log"

python3 - "$tmp/self-test.json" <<'PY'
import json
import sys

data = json.load(open(sys.argv[1]))

required = [
    "simple_plane",
    "concave_plane",
    "hole_aware",
    "hole_aware_two_hole",
    "vertical_replacement",
    "sloped_replacement",
    "bad_boundary",
    "hole_aware_failure",
    "hole_aware_same_direction_failure",
    "hole_aware_nested_failure",
    "hole_aware_straddling_failure",
    "hole_aware_intersecting_holes_failure",
    "hole_aware_touching_outer_failure",
    "hole_aware_touching_holes_failure",
    "edge_incidence_only",
    "validation_gate",
    "boundary_direction",
    "trusted_neighbor_edge",
    "trusted_provenance_gate",
]
missing = [name for name in required if name not in data]
if missing:
    raise SystemExit(f"missing self-test sections: {missing}")

simple = data["simple_plane"]["top_components"][0]
if simple["classification"] != "cxx_phase1_simple_planar_candidate":
    raise SystemExit("simple plane candidate classification changed")
if simple["boundary_loops"] != 1 or simple["bad_boundary_vertices"] != 0:
    raise SystemExit("simple plane boundary contract changed")
if not simple["replacement_checked"] or not simple["replacement_feasible"]:
    raise SystemExit("simple plane replacement feasibility changed")
if not simple["replacement_boundary_matches"]:
    raise SystemExit("simple plane replacement boundary mismatch")
if not simple["replacement_area_matches"]:
    raise SystemExit("simple plane replacement area mismatch")
if not simple["replacement_edge_incidence_checked"]:
    raise SystemExit("simple plane replacement edge-incidence check disappeared")
if not simple["replacement_edge_incidence_valid"]:
    raise SystemExit("simple plane replacement edge-incidence became invalid")
if simple["replacement_triangles_after"] != simple["estimated_triangles_after"]:
    raise SystemExit("simple plane replacement count changed")
if not simple["decision_bearing"] or not simple["component_decision_fingerprint"]:
    raise SystemExit("simple plane decision fingerprint missing")
simple_totals = data["simple_plane"]["component_totals"]
if simple_totals["decision_bearing_components"] != 1:
    raise SystemExit("simple plane decision-bearing component count changed")
if not simple_totals["component_decision_fingerprint"]:
    raise SystemExit("simple plane aggregate decision fingerprint missing")

concave = data["concave_plane"]["top_components"][0]
if concave["classification"] != "cxx_phase1_simple_planar_candidate":
    raise SystemExit("concave plane candidate classification changed")
if concave["boundary_loops"] != 1 or concave["bad_boundary_vertices"] != 0:
    raise SystemExit("concave plane boundary contract changed")
if not concave["replacement_checked"] or not concave["replacement_feasible"]:
    raise SystemExit("concave plane replacement feasibility changed")
if not concave["replacement_boundary_matches"]:
    raise SystemExit("concave plane replacement boundary mismatch")
if not concave["replacement_area_matches"]:
    raise SystemExit("concave plane replacement area mismatch")
if concave["replacement_triangles_after"] != concave["estimated_triangles_after"]:
    raise SystemExit("concave plane replacement count changed")

hole = data["hole_aware"]["top_components"][0]
if hole["classification"] != "candidate_requires_hole_aware_triangulation":
    raise SystemExit("hole-aware candidate classification changed")
if hole["boundary_loops"] <= 1:
    raise SystemExit("hole-aware fixture no longer has multiple loops")
if hole["replacement_checked"] or hole["replacement_feasible"]:
    raise SystemExit("hole-aware fixture unexpectedly checked as phase-one replacement")
if not hole["hole_aware_checked"] or not hole["hole_aware_feasible"]:
    raise SystemExit("hole-aware boundary validation failed")
if hole["hole_aware_ordered_loops"] != hole["boundary_loops"]:
    raise SystemExit("hole-aware ordered loop count changed")
if hole["hole_aware_outer_loops"] != 1 or hole["hole_aware_hole_loops"] != 1:
    raise SystemExit("hole-aware outer/hole loop counts changed")
if hole["hole_aware_estimated_triangles_after"] != hole["estimated_triangles_after"]:
    raise SystemExit("hole-aware replacement estimate changed")
if not hole["hole_aware_replacement_checked"]:
    raise SystemExit("hole-aware replacement prototype did not run")
if not hole["hole_aware_replacement_feasible"]:
    raise SystemExit("hole-aware replacement prototype failed")
if not hole["hole_aware_replacement_boundary_matches"]:
    raise SystemExit("hole-aware replacement boundary mismatch")
if not hole["hole_aware_replacement_area_matches"]:
    raise SystemExit("hole-aware replacement area mismatch")
if not hole["hole_aware_replacement_edge_incidence_checked"]:
    raise SystemExit("hole-aware replacement edge-incidence check disappeared")
if not hole["hole_aware_replacement_edge_incidence_valid"]:
    raise SystemExit("hole-aware replacement edge-incidence became invalid")
if hole["hole_aware_replacement_triangles_after"] != hole["estimated_triangles_after"]:
    raise SystemExit("hole-aware replacement prototype count changed")
if not hole["decision_bearing"] or not hole["component_decision_fingerprint"]:
    raise SystemExit("hole-aware decision fingerprint missing")
hole_totals = data["hole_aware"]["component_totals"]
if hole_totals["decision_bearing_components"] != 1:
    raise SystemExit("hole-aware decision-bearing component count changed")
if not hole_totals["component_decision_fingerprint"]:
    raise SystemExit("hole-aware aggregate decision fingerprint missing")

two_hole = data["hole_aware_two_hole"]
if two_hole["classification"] != "candidate_requires_hole_aware_triangulation":
    raise SystemExit("two-hole candidate classification changed")
if two_hole["boundary"]["boundary_loops"] != 3:
    raise SystemExit("two-hole boundary loop count changed")
if two_hole["hole_aware"]["hole_aware_outer_loops"] != 1:
    raise SystemExit("two-hole outer loop count changed")
if two_hole["hole_aware"]["hole_aware_hole_loops"] != 2:
    raise SystemExit("two-hole hole loop count changed")
if two_hole["hole_aware"]["hole_aware_estimated_triangles_after"] != 14:
    raise SystemExit("two-hole estimated replacement count changed")
two_hole_repl = two_hole["hole_aware_replacement"]
if not two_hole_repl["hole_aware_replacement_feasible"]:
    raise SystemExit("two-hole hole-aware replacement became infeasible")
if two_hole_repl["hole_aware_replacement_triangles_after"] != 14:
    raise SystemExit("two-hole replacement count changed")
if not two_hole_repl["hole_aware_replacement_edge_incidence_valid"]:
    raise SystemExit("two-hole replacement edge incidence became invalid")

vertical = data["vertical_replacement"]
if vertical["x_phase1_triangles_after"] != 2:
    raise SystemExit("vertical x-plane phase-one replacement count changed")
if vertical["y_phase1_triangles_after"] != 2:
    raise SystemExit("vertical y-plane phase-one replacement count changed")
if vertical["x_hole_aware_loops"] != 2:
    raise SystemExit("vertical x-plane hole-aware loop count changed")
if vertical["x_hole_aware_triangles_after"] != 8:
    raise SystemExit("vertical x-plane hole-aware replacement count changed")
if not vertical["x_phase1_boundary_matches"]:
    raise SystemExit("vertical x-plane phase-one boundary mismatch")
if not vertical["y_phase1_boundary_matches"]:
    raise SystemExit("vertical y-plane phase-one boundary mismatch")
if not vertical["x_hole_aware_boundary_matches"]:
    raise SystemExit("vertical x-plane hole-aware boundary mismatch")

sloped = data["sloped_replacement"]
if sloped["phase1_triangles_after"] != 2:
    raise SystemExit("sloped phase-one replacement count changed")
if not sloped["phase1_boundary_matches"]:
    raise SystemExit("sloped phase-one boundary mismatch")
if sloped["hole_aware_loops"] != 2:
    raise SystemExit("sloped hole-aware loop count changed")
if sloped["hole_aware_triangles_after"] != 8:
    raise SystemExit("sloped hole-aware replacement count changed")
if not sloped["hole_aware_boundary_matches"]:
    raise SystemExit("sloped hole-aware boundary mismatch")

bad = data["bad_boundary"]
if bad["classification"] != "reject_boundary_not_closed_loops":
    raise SystemExit("bad boundary rejection classification changed")

hole_bad = data["hole_aware_failure"]
if hole_bad["classification"] != "reject_hole_aware_boundary_validation_failed":
    raise SystemExit("hole-aware failure classification changed")
if hole_bad["hole_aware"]["hole_aware_feasible"]:
    raise SystemExit("hole-aware failure unexpectedly became feasible")
if hole_bad["hole_aware"]["hole_aware_failure"] != "hole_outside_outer_loop":
    raise SystemExit("hole-aware failure reason changed")
repl = hole_bad["hole_aware_replacement"]
if not repl["hole_aware_replacement_checked"]:
    raise SystemExit("hole-aware failure replacement prototype did not run")
if repl["hole_aware_replacement_feasible"]:
    raise SystemExit("hole-aware failure replacement prototype unexpectedly became feasible")
if repl["hole_aware_replacement_failure"] != "hole_outside_outer_loop":
    raise SystemExit("hole-aware failure replacement reason changed")

same_direction_bad = data["hole_aware_same_direction_failure"]
if same_direction_bad["classification"] != "reject_hole_aware_replacement_failed":
    raise SystemExit("same-direction hole-aware failure classification changed")
if same_direction_bad["boundary"]["boundary_loops"] != 2:
    raise SystemExit("same-direction hole-aware failure loop count changed")
if not same_direction_bad["hole_aware"]["hole_aware_feasible"]:
    raise SystemExit("same-direction hole-aware boundary validation stopped being feasible")
same_direction_repl = same_direction_bad["hole_aware_replacement"]
if not same_direction_repl["hole_aware_replacement_checked"]:
    raise SystemExit("same-direction hole-aware replacement prototype did not run")
if same_direction_repl["hole_aware_replacement_feasible"]:
    raise SystemExit("same-direction hole-aware replacement unexpectedly became feasible")
if same_direction_repl["hole_aware_replacement_failure"] != "boundary_direction_mismatch":
    raise SystemExit("same-direction hole-aware replacement reason changed")

nested_bad = data["hole_aware_nested_failure"]
if nested_bad["classification"] != "reject_hole_aware_boundary_validation_failed":
    raise SystemExit("nested hole-aware failure classification changed")
if nested_bad["boundary"]["boundary_loops"] != 3:
    raise SystemExit("nested hole-aware failure loop count changed")
if nested_bad["hole_aware"]["hole_aware_feasible"]:
    raise SystemExit("nested hole-aware failure unexpectedly became feasible")
if nested_bad["hole_aware"]["hole_aware_failure"] != "hole_inside_hole":
    raise SystemExit("nested hole-aware failure reason changed")
nested_repl = nested_bad["hole_aware_replacement"]
if not nested_repl["hole_aware_replacement_checked"]:
    raise SystemExit("nested hole-aware replacement prototype did not run")
if nested_repl["hole_aware_replacement_feasible"]:
    raise SystemExit("nested hole-aware replacement unexpectedly became feasible")
if nested_repl["hole_aware_replacement_failure"] != "hole_inside_hole":
    raise SystemExit("nested hole-aware replacement reason changed")

straddling_bad = data["hole_aware_straddling_failure"]
if straddling_bad["classification"] != "reject_hole_aware_boundary_validation_failed":
    raise SystemExit("straddling hole-aware failure classification changed")
if straddling_bad["boundary"]["boundary_loops"] != 2:
    raise SystemExit("straddling hole-aware failure loop count changed")
if straddling_bad["hole_aware"]["hole_aware_feasible"]:
    raise SystemExit("straddling hole-aware failure unexpectedly became feasible")
if straddling_bad["hole_aware"]["hole_aware_failure"] != "hole_outside_outer_loop":
    raise SystemExit("straddling hole-aware failure reason changed")
straddling_repl = straddling_bad["hole_aware_replacement"]
if not straddling_repl["hole_aware_replacement_checked"]:
    raise SystemExit("straddling hole-aware replacement prototype did not run")
if straddling_repl["hole_aware_replacement_feasible"]:
    raise SystemExit("straddling hole-aware replacement unexpectedly became feasible")
if straddling_repl["hole_aware_replacement_failure"] != "hole_outside_outer_loop":
    raise SystemExit("straddling hole-aware replacement reason changed")

intersecting_bad = data["hole_aware_intersecting_holes_failure"]
if intersecting_bad["classification"] != "reject_hole_aware_boundary_validation_failed":
    raise SystemExit("intersecting hole-aware failure classification changed")
if intersecting_bad["boundary"]["boundary_loops"] != 3:
    raise SystemExit("intersecting hole-aware failure loop count changed")
if intersecting_bad["hole_aware"]["hole_aware_feasible"]:
    raise SystemExit("intersecting hole-aware failure unexpectedly became feasible")
if intersecting_bad["hole_aware"]["hole_aware_failure"] != "hole_intersects_hole":
    raise SystemExit("intersecting hole-aware failure reason changed")
intersecting_repl = intersecting_bad["hole_aware_replacement"]
if not intersecting_repl["hole_aware_replacement_checked"]:
    raise SystemExit("intersecting hole-aware replacement prototype did not run")
if intersecting_repl["hole_aware_replacement_feasible"]:
    raise SystemExit("intersecting hole-aware replacement unexpectedly became feasible")
if intersecting_repl["hole_aware_replacement_failure"] != "hole_intersects_hole":
    raise SystemExit("intersecting hole-aware replacement reason changed")

touching_outer_bad = data["hole_aware_touching_outer_failure"]
if touching_outer_bad["classification"] != "reject_hole_aware_boundary_validation_failed":
    raise SystemExit("touching-outer hole-aware failure classification changed")
if touching_outer_bad["boundary"]["boundary_loops"] != 2:
    raise SystemExit("touching-outer hole-aware failure loop count changed")
if touching_outer_bad["hole_aware"]["hole_aware_feasible"]:
    raise SystemExit("touching-outer hole-aware failure unexpectedly became feasible")
if touching_outer_bad["hole_aware"]["hole_aware_failure"] != "hole_outside_outer_loop":
    raise SystemExit("touching-outer hole-aware failure reason changed")
touching_outer_repl = touching_outer_bad["hole_aware_replacement"]
if not touching_outer_repl["hole_aware_replacement_checked"]:
    raise SystemExit("touching-outer hole-aware replacement prototype did not run")
if touching_outer_repl["hole_aware_replacement_feasible"]:
    raise SystemExit("touching-outer hole-aware replacement unexpectedly became feasible")
if touching_outer_repl["hole_aware_replacement_failure"] != "hole_outside_outer_loop":
    raise SystemExit("touching-outer hole-aware replacement reason changed")

touching_holes_bad = data["hole_aware_touching_holes_failure"]
if touching_holes_bad["classification"] != "reject_boundary_not_closed_loops":
    raise SystemExit("touching-holes failure classification changed")
if touching_holes_bad["boundary"]["bad_boundary_vertices"] <= 0:
    raise SystemExit("touching-holes failure lost bad boundary vertices")
if touching_holes_bad["boundary"]["duplicate_boundary_edges"] <= 0:
    raise SystemExit("touching-holes failure lost duplicate boundary edges")
if touching_holes_bad["hole_aware"]["hole_aware_feasible"]:
    raise SystemExit("touching-holes failure unexpectedly became feasible")
if touching_holes_bad["hole_aware"]["hole_aware_failure"] != "loop_order_failed":
    raise SystemExit("touching-holes failure reason changed")
touching_holes_repl = touching_holes_bad["hole_aware_replacement"]
if not touching_holes_repl["hole_aware_replacement_checked"]:
    raise SystemExit("touching-holes replacement prototype did not run")
if touching_holes_repl["hole_aware_replacement_feasible"]:
    raise SystemExit("touching-holes replacement unexpectedly became feasible")
if touching_holes_repl["hole_aware_replacement_failure"] != "loop_order_failed":
    raise SystemExit("touching-holes replacement reason changed")

edge = data["edge_incidence_only"]["strict_edge_incidence"]
if edge["boundary_edges"] <= 0 or edge["nonmanifold_edges"] != 0:
    raise SystemExit("edge-incidence-only fixture contract changed")
if edge["misoriented_edges"] != 0:
    raise SystemExit("edge-incidence-only fixture orientation changed")
if edge["watertight_edge_count"]:
    raise SystemExit("edge-incidence-only fixture unexpectedly watertight")

gate = data["validation_gate"]
if gate["open_preserved"]["validation_rolled_back"]:
    raise SystemExit("validation gate rolled back unchanged open topology")
if gate["open_preserved"]["validation_triangle_count_mismatch"]:
    raise SystemExit("validation gate reported triangle count mismatch on unchanged open topology")
if gate["open_preserved"]["validation_vertex_count_mismatch"]:
    raise SystemExit("validation gate reported vertex count mismatch on unchanged open topology")
if gate["open_preserved"]["validation_normal_count_mismatch"]:
    raise SystemExit("validation gate reported normal count mismatch on unchanged open topology")
if gate["open_improved"]["validation_rolled_back"]:
    raise SystemExit("validation gate rolled back improved open topology")
if not gate["watertight_to_open"]["validation_rolled_back"]:
    raise SystemExit("validation gate accepted watertight-to-open topology")
if gate["watertight_to_open"]["output_boundary_edges"] != 0:
    raise SystemExit("validation rollback did not restore watertight boundary count")
if not gate["open_to_more_open"]["validation_rolled_back"]:
    raise SystemExit("validation gate accepted more-open topology")
if gate["open_to_more_open"]["output_boundary_edges"] != 8:
    raise SystemExit("validation rollback did not restore open boundary count")
if not gate["nonmanifold_worse"]["validation_rolled_back"]:
    raise SystemExit("validation gate accepted worse nonmanifold topology")
if gate["nonmanifold_worse"]["output_nonmanifold_edges"] != 0:
    raise SystemExit("validation rollback did not restore nonmanifold count")
if gate["degenerate_preserved"]["validation_rolled_back"]:
    raise SystemExit("validation gate rolled back unchanged degenerate count")
if gate["degenerate_improved"]["validation_rolled_back"]:
    raise SystemExit("validation gate rolled back improved degenerate count")
if not gate["degenerate_worse"]["validation_rolled_back"]:
    raise SystemExit("validation gate accepted worse degenerate count")
if not gate["degenerate_worse"]["validation_degenerate_worse"]:
    raise SystemExit("validation gate did not record degenerate worsening")
if gate["degenerate_worse"]["output_degenerate_triangles"] != 0:
    raise SystemExit("validation rollback did not restore degenerate count")
if not gate["orientation_worse"]["validation_rolled_back"]:
    raise SystemExit("validation gate accepted worse orientation")
if not gate["orientation_worse"]["validation_orientation_worse"]:
    raise SystemExit("validation gate did not record orientation worsening")
if gate["orientation_worse"]["output_misoriented_edges"] != 0:
    raise SystemExit("validation rollback did not restore orientation count")
if not gate["vertex_count_mismatch"]["validation_rolled_back"]:
    raise SystemExit("validation gate accepted candidate vertex count mismatch")
if not gate["vertex_count_mismatch"]["validation_vertex_count_mismatch"]:
    raise SystemExit("validation gate did not record vertex count mismatch")
if not gate["vertex_count_mismatch"]["output_watertight"]:
    raise SystemExit("validation vertex-count rollback lost watertight topology")
if not gate["normal_count_mismatch"]["validation_rolled_back"]:
    raise SystemExit("validation gate accepted candidate normal count mismatch")
if not gate["normal_count_mismatch"]["validation_normal_count_mismatch"]:
    raise SystemExit("validation gate did not record normal count mismatch")
if not gate["normal_count_mismatch"]["output_watertight"]:
    raise SystemExit("validation normal-count rollback lost watertight topology")
if not gate["triangle_count_mismatch"]["validation_rolled_back"]:
    raise SystemExit("validation gate accepted candidate triangle count mismatch")
if not gate["triangle_count_mismatch"]["validation_triangle_count_mismatch"]:
    raise SystemExit("validation gate did not record triangle count mismatch")
if gate["triangle_count_mismatch"]["expected_output_triangles"] != 10:
    raise SystemExit("validation gate expected triangle count changed")
if gate["triangle_count_mismatch"]["candidate_triangles"] != 9:
    raise SystemExit("validation gate candidate triangle count changed")
if not gate["triangle_count_mismatch"]["output_watertight"]:
    raise SystemExit("validation rollback lost watertight topology")

source = data["source_buffer_preflight"]
if not source["valid"]["source_buffers_valid"]:
    raise SystemExit("source preflight rejected valid buffers")
if source["valid"]["source_expected_floats"] != 18:
    raise SystemExit("source preflight expected-float count changed")
if source["valid"]["source_vertex_count_mismatch"]:
    raise SystemExit("source preflight reported vertex mismatch on valid buffers")
if source["valid"]["source_normal_count_mismatch"]:
    raise SystemExit("source preflight reported normal mismatch on valid buffers")
if source["vertex_short"]["source_buffers_valid"]:
    raise SystemExit("source preflight accepted short vertex buffer")
if not source["vertex_short"]["source_vertex_count_mismatch"]:
    raise SystemExit("source preflight did not record short vertex buffer")
if source["vertex_short"]["source_normal_count_mismatch"]:
    raise SystemExit("source preflight reported wrong normal mismatch")
if source["normal_short"]["source_buffers_valid"]:
    raise SystemExit("source preflight accepted short normal buffer")
if source["normal_short"]["source_vertex_count_mismatch"]:
    raise SystemExit("source preflight reported wrong vertex mismatch")
if not source["normal_short"]["source_normal_count_mismatch"]:
    raise SystemExit("source preflight did not record short normal buffer")
if source["both_wrong"]["source_buffers_valid"]:
    raise SystemExit("source preflight accepted malformed source buffers")
if not source["both_wrong"]["source_vertex_count_mismatch"]:
    raise SystemExit("source preflight did not record malformed vertex buffer")
if not source["both_wrong"]["source_normal_count_mismatch"]:
    raise SystemExit("source preflight did not record malformed normal buffer")

boundary_direction = data["boundary_direction"]
if not boundary_direction["canonical_boundary_matches_flipped"]:
    raise SystemExit("boundary-direction self-test lost canonical boundary match")
if not boundary_direction["directed_boundary_detects_flipped"]:
    raise SystemExit("boundary-direction self-test did not detect flipped winding")
if not boundary_direction["orientation_repaired"]:
    raise SystemExit("boundary-direction self-test did not repair winding")
if not boundary_direction["canonical_boundary_matches_internal_misorientation"]:
    raise SystemExit("boundary-direction self-test lost internal mismatch setup")
if not boundary_direction["local_edge_incidence_rejects_internal_misorientation"]:
    raise SystemExit("local replacement edge-incidence check did not reject internal misorientation")

trusted_edge = data["trusted_neighbor_edge"]
if trusted_edge["correct_edge_mismatches"] != 0:
    raise SystemExit("trusted-neighbor correct edge table reported mismatches")
if trusted_edge["wrong_edge_mismatches"] <= 0:
    raise SystemExit("trusted-neighbor wrong edge table was not rejected")
if trusted_edge["same_orientation_edge_mismatches"] != 0:
    raise SystemExit("trusted-neighbor same-orientation table changed edge identity")
if trusted_edge["same_orientation_mismatches"] <= 0:
    raise SystemExit("trusted-neighbor same-orientation table was not rejected")
if trusted_edge["correct_cached_edge_mismatches"] != trusted_edge["correct_edge_mismatches"]:
    raise SystemExit("trusted-neighbor cached correct result changed")
if trusted_edge["wrong_cached_edge_mismatches"] != trusted_edge["wrong_edge_mismatches"]:
    raise SystemExit("trusted-neighbor cached wrong-edge result changed")
if trusted_edge["same_orientation_cached_mismatches"] != trusted_edge["same_orientation_mismatches"]:
    raise SystemExit("trusted-neighbor cached same-orientation result changed")
if trusted_edge["correct_reciprocal_slots"] != [0, 2]:
    raise SystemExit("trusted-neighbor correct reciprocal slots changed")
if trusted_edge["wrong_reciprocal_slots"] != [1, 2]:
    raise SystemExit("trusted-neighbor wrong reciprocal slots changed")
if trusted_edge["same_orientation_reciprocal_slots"] != [0, 2]:
    raise SystemExit("trusted-neighbor same-orientation reciprocal slots changed")
if not trusted_edge["cached_reciprocal_slots_match_scan"]:
    raise SystemExit("trusted-neighbor cached slot contract diverged from scan")
if not trusted_edge["reciprocal_triangle_links_are_not_enough"]:
    raise SystemExit("trusted-neighbor edge contract lost its safety note")
for key in (
    "wrong_size_rejected",
    "range_rejected",
    "self_rejected",
    "duplicate_rejected",
    "asymmetry_rejected",
):
    if not trusted_edge[key]:
        raise SystemExit(f"trusted-neighbor malformed-sidecar check failed: {key}")

trusted_gate = data["trusted_provenance_gate"]
if not trusted_gate["clean_eligible"]:
    raise SystemExit("trusted provenance clean sidecar was not eligible")
if not trusted_gate["raw_orientation_rejected"]:
    raise SystemExit("trusted provenance accepted raw misoriented sidecar")
if not trusted_gate["raw_orientation_rejected_raw_topology"]:
    raise SystemExit("trusted provenance raw topology gate ignored orientation")
if not trusted_gate["welded_orientation_rejected"]:
    raise SystemExit("trusted provenance accepted welded misoriented sidecar")
if not trusted_gate["edge_mismatch_rejected"]:
    raise SystemExit("trusted provenance accepted neighbor edge mismatch")
if not trusted_gate["neighbor_orientation_rejected"]:
    raise SystemExit("trusted provenance accepted neighbor orientation mismatch")
PY

echo "mesh reduction contract smoke passed"
