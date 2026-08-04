#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

camsim="${1:-./camsim}"

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

compare_profiles() {
  local name=$1
  local default_profile=$2
  local provenance_profile=$3

  python3 - "$name" "$default_profile" "$provenance_profile" <<'PY'
import json
import sys

name = sys.argv[1]
default = json.load(open(sys.argv[2])).get("metrics", {})
provenance = json.load(open(sys.argv[3])).get("metrics", {})

for key in (
    "safe_reduce_input_triangles",
    "safe_reduce_source_expected_floats",
    "safe_reduce_source_vertex_floats",
    "safe_reduce_source_normal_floats",
    "safe_reduce_source_vertex_count_mismatch",
    "safe_reduce_source_normal_count_mismatch",
    "safe_reduce_output_triangles",
    "safe_reduce_input_binary_stl_bytes",
    "safe_reduce_output_binary_stl_bytes",
    "safe_reduce_output_binary_stl_bytes_saved",
    "safe_reduce_estimated_binary_stl_bytes_after",
    "safe_reduce_estimated_binary_stl_bytes_saved",
    "safe_reduce_components",
    "safe_reduce_estimated_triangles_after",
    "safe_reduce_estimated_triangle_reduction",
    "safe_reduce_phase1_components",
    "safe_reduce_hole_aware_components",
    "safe_reduce_estimated_replacement_checks",
    "safe_reduce_feasible_replacement_checks",
    "safe_reduce_writable_replacement_checks",
    "safe_reduce_unwritable_replacement_checks",
    "safe_reduce_phase1_writable_replacement_checks",
    "safe_reduce_hole_aware_writable_replacement_checks",
    "safe_reduce_phase1_unwritable_replacement_checks",
    "safe_reduce_hole_aware_unwritable_replacement_checks",
    "safe_reduce_rejected_triangulation_components",
    "safe_reduce_rejected_triangulation_triangles",
    "safe_reduce_applied_components",
    "safe_reduce_applied_source_triangles",
    "safe_reduce_applied_output_triangles",
    "safe_reduce_output_boundary_edges",
    "safe_reduce_output_nonmanifold_edges",
    "safe_reduce_output_misoriented_edges",
    "safe_reduce_output_degenerate_triangles",
    "safe_reduce_output_watertight",
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
):
    if default.get(key) != provenance.get(key):
        raise SystemExit(
            f"{name}: provenance-neighbor metric mismatch for {key}: "
            f"{default.get(key)} != {provenance.get(key)}"
        )

if provenance.get("safe_reduce_provenance_neighbors_requested") != 1:
    raise SystemExit(f"{name}: provenance-neighbor run did not record request")
if provenance.get("safe_reduce_trust_provenance_neighbors_requested") != 0:
    raise SystemExit(f"{name}: provenance-neighbor run unexpectedly requested trust")
if provenance.get("safe_reduce_trusted_provenance_neighbors_eligible") != 0:
    raise SystemExit(f"{name}: provenance-neighbor run unexpectedly marked trust eligible")
if provenance.get("safe_reduce_contour_provenance_neighbor_parity") != 1:
    raise SystemExit(f"{name}: provenance-neighbor run lacked neighbor parity")
if provenance.get("safe_reduce_contour_provenance_neighbor_parity_audited") != 1:
    raise SystemExit(f"{name}: provenance-neighbor run did not audit neighbor parity")
if provenance.get("safe_reduce_contour_provenance_neighbors_cached") != 1:
    raise SystemExit(f"{name}: provenance-neighbor run did not use cached neighbors")
if provenance.get("safe_reduce_using_provenance_neighbors") != 1:
    raise SystemExit(f"{name}: provenance-neighbor run did not activate path")
if provenance.get("safe_reduce_output_watertight") != 1:
    raise SystemExit(f"{name}: provenance-neighbor output not watertight")
if provenance.get("safe_reduce_source_expected_floats") != provenance.get(
    "safe_reduce_input_triangles", 0
) * 9:
    raise SystemExit(f"{name}: provenance-neighbor source expected-float count mismatch")
if provenance.get("safe_reduce_source_vertex_floats") != provenance.get(
    "safe_reduce_source_expected_floats"
):
    raise SystemExit(f"{name}: provenance-neighbor source vertex float count mismatch")
if provenance.get("safe_reduce_source_normal_floats") != provenance.get(
    "safe_reduce_source_expected_floats"
):
    raise SystemExit(f"{name}: provenance-neighbor source normal float count mismatch")
if provenance.get("safe_reduce_source_vertex_count_mismatch") != 0:
    raise SystemExit(f"{name}: provenance-neighbor unexpectedly flagged source vertices")
if provenance.get("safe_reduce_source_normal_count_mismatch") != 0:
    raise SystemExit(f"{name}: provenance-neighbor unexpectedly flagged source normals")
if (
    provenance.get("safe_reduce_writable_replacement_checks", 0)
    + provenance.get("safe_reduce_unwritable_replacement_checks", 0)
    != provenance.get("safe_reduce_estimated_replacement_checks", 0)
):
    raise SystemExit(f"{name}: provenance-neighbor writable/unwritable accounting changed")
if (
    provenance.get("safe_reduce_phase1_unwritable_replacement_checks", 0)
    + provenance.get("safe_reduce_hole_aware_unwritable_replacement_checks", 0)
    != provenance.get("safe_reduce_unwritable_replacement_checks", 0)
):
    raise SystemExit(f"{name}: provenance-neighbor unwritable buckets changed")
if provenance.get("safe_reduce_validation_topology_worse") != 0:
    raise SystemExit(f"{name}: provenance-neighbor unexpectedly worsened topology")
if provenance.get("safe_reduce_validation_degenerate_worse") != 0:
    raise SystemExit(f"{name}: provenance-neighbor unexpectedly worsened degenerates")
if provenance.get("safe_reduce_validation_orientation_worse") != 0:
    raise SystemExit(f"{name}: provenance-neighbor unexpectedly worsened orientation")
if provenance.get("safe_reduce_validation_vertex_count_mismatch") != 0:
    raise SystemExit(f"{name}: provenance-neighbor candidate vertex count mismatch")
if provenance.get("safe_reduce_validation_normal_count_mismatch") != 0:
    raise SystemExit(f"{name}: provenance-neighbor candidate normal count mismatch")
if provenance.get("safe_reduce_validation_triangle_count_mismatch") != 0:
    raise SystemExit(f"{name}: provenance-neighbor candidate triangle count mismatch")
if provenance.get("safe_reduce_validation_rolled_back") != 0:
    raise SystemExit(f"{name}: provenance-neighbor unexpectedly rolled back")
if provenance.get("safe_reduce_validation_expected_output_triangles") != provenance.get(
    "safe_reduce_output_triangles"
):
    raise SystemExit(f"{name}: provenance-neighbor expected output triangle count mismatch")
if provenance.get("safe_reduce_validation_candidate_triangles") != provenance.get(
    "safe_reduce_output_triangles"
):
    raise SystemExit(f"{name}: provenance-neighbor candidate triangle count mismatch")
if provenance.get("safe_reduce_validation_candidate_triangles") != provenance.get(
    "safe_reduce_validation_expected_output_triangles"
):
    raise SystemExit(f"{name}: provenance-neighbor candidate/expected triangle count mismatch")
if provenance.get("safe_reduce_output_misoriented_edges") != 0:
    raise SystemExit(f"{name}: provenance-neighbor output has misoriented edges")
if provenance.get("safe_reduce_validation_candidate_misoriented_edges") != provenance.get(
    "safe_reduce_output_misoriented_edges"
):
    raise SystemExit(f"{name}: provenance-neighbor candidate misoriented count mismatch")

print(
    f"{name}: input={provenance['safe_reduce_input_triangles']} "
    f"output={provenance['safe_reduce_output_triangles']} "
    f"applied={provenance['safe_reduce_applied_components']} "
    f"using_provenance="
    f"{provenance['safe_reduce_using_provenance_neighbors']}"
)
PY
}

compare_trusted_profiles() {
  local name=$1
  local default_profile=$2
  local trusted_profile=$3

  python3 - "$name" "$default_profile" "$trusted_profile" <<'PY'
import json
import sys

name = sys.argv[1]
default = json.load(open(sys.argv[2])).get("metrics", {})
trusted = json.load(open(sys.argv[3])).get("metrics", {})

for key in (
    "safe_reduce_input_triangles",
    "safe_reduce_source_expected_floats",
    "safe_reduce_source_vertex_floats",
    "safe_reduce_source_normal_floats",
    "safe_reduce_source_vertex_count_mismatch",
    "safe_reduce_source_normal_count_mismatch",
    "safe_reduce_output_triangles",
    "safe_reduce_input_binary_stl_bytes",
    "safe_reduce_output_binary_stl_bytes",
    "safe_reduce_output_binary_stl_bytes_saved",
    "safe_reduce_estimated_binary_stl_bytes_after",
    "safe_reduce_estimated_binary_stl_bytes_saved",
    "safe_reduce_components",
    "safe_reduce_estimated_triangles_after",
    "safe_reduce_estimated_triangle_reduction",
    "safe_reduce_phase1_components",
    "safe_reduce_hole_aware_components",
    "safe_reduce_estimated_replacement_checks",
    "safe_reduce_feasible_replacement_checks",
    "safe_reduce_writable_replacement_checks",
    "safe_reduce_unwritable_replacement_checks",
    "safe_reduce_phase1_writable_replacement_checks",
    "safe_reduce_hole_aware_writable_replacement_checks",
    "safe_reduce_phase1_unwritable_replacement_checks",
    "safe_reduce_hole_aware_unwritable_replacement_checks",
    "safe_reduce_rejected_triangulation_components",
    "safe_reduce_rejected_triangulation_triangles",
    "safe_reduce_applied_components",
    "safe_reduce_applied_source_triangles",
    "safe_reduce_applied_output_triangles",
    "safe_reduce_output_boundary_edges",
    "safe_reduce_output_nonmanifold_edges",
    "safe_reduce_output_misoriented_edges",
    "safe_reduce_output_degenerate_triangles",
    "safe_reduce_output_watertight",
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
):
    if default.get(key) != trusted.get(key):
        raise SystemExit(
            f"{name}: trusted-provenance metric mismatch for {key}: "
            f"{default.get(key)} != {trusted.get(key)}"
        )

if trusted.get("safe_reduce_provenance_neighbors_requested") != 1:
    raise SystemExit(f"{name}: trusted run did not request provenance neighbors")
if trusted.get("safe_reduce_trust_provenance_neighbors_requested") != 1:
    raise SystemExit(f"{name}: trusted run did not record trust request")
if trusted.get("safe_reduce_trusted_provenance_neighbors_eligible") != 1:
    raise SystemExit(f"{name}: trusted run was not marked eligible")
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
    if trusted.get(key) != 0:
        raise SystemExit(f"{name}: trusted run rejected eligibility via {key}")
if trusted.get("safe_reduce_contour_provenance_neighbors_cached") != 1:
    raise SystemExit(f"{name}: trusted run did not use cached neighbors")
if trusted.get("safe_reduce_using_provenance_neighbors") != 1:
    raise SystemExit(f"{name}: trusted run did not activate provenance path")
if trusted.get("safe_reduce_trusted_provenance_neighbors_used") != 1:
    raise SystemExit(f"{name}: trusted run did not use trusted provenance")
if trusted.get("safe_reduce_default_adjacency_skipped") != 1:
    raise SystemExit(f"{name}: trusted run did not skip default adjacency")
if trusted.get("safe_reduce_trusted_provenance_neighbor_slots_checked", 0) <= 0:
    raise SystemExit(f"{name}: trusted run did not validate neighbor slots")
if trusted.get("safe_reduce_trusted_provenance_neighbor_edge_slots_checked", 0) <= 0:
    raise SystemExit(f"{name}: trusted run did not validate neighbor edge slots")
if trusted.get("safe_reduce_trusted_provenance_neighbor_edge_mismatches", 0) != 0:
    raise SystemExit(f"{name}: trusted run found neighbor edge mismatches")
if trusted.get("safe_reduce_contour_provenance_neighbor_parity_audited") != 0:
    raise SystemExit(f"{name}: trusted run unexpectedly audited skipped adjacency")
if trusted.get("safe_reduce_output_watertight") != 1:
    raise SystemExit(f"{name}: trusted output not watertight")
if trusted.get("safe_reduce_source_expected_floats") != trusted.get(
    "safe_reduce_input_triangles", 0
) * 9:
    raise SystemExit(f"{name}: trusted source expected-float count mismatch")
if trusted.get("safe_reduce_source_vertex_floats") != trusted.get(
    "safe_reduce_source_expected_floats"
):
    raise SystemExit(f"{name}: trusted source vertex float count mismatch")
if trusted.get("safe_reduce_source_normal_floats") != trusted.get(
    "safe_reduce_source_expected_floats"
):
    raise SystemExit(f"{name}: trusted source normal float count mismatch")
if trusted.get("safe_reduce_source_vertex_count_mismatch") != 0:
    raise SystemExit(f"{name}: trusted unexpectedly flagged source vertices")
if trusted.get("safe_reduce_source_normal_count_mismatch") != 0:
    raise SystemExit(f"{name}: trusted unexpectedly flagged source normals")
if (
    trusted.get("safe_reduce_writable_replacement_checks", 0)
    + trusted.get("safe_reduce_unwritable_replacement_checks", 0)
    != trusted.get("safe_reduce_estimated_replacement_checks", 0)
):
    raise SystemExit(f"{name}: trusted writable/unwritable accounting changed")
if (
    trusted.get("safe_reduce_phase1_unwritable_replacement_checks", 0)
    + trusted.get("safe_reduce_hole_aware_unwritable_replacement_checks", 0)
    != trusted.get("safe_reduce_unwritable_replacement_checks", 0)
):
    raise SystemExit(f"{name}: trusted unwritable buckets changed")
if trusted.get("safe_reduce_validation_topology_worse") != 0:
    raise SystemExit(f"{name}: trusted run unexpectedly worsened topology")
if trusted.get("safe_reduce_validation_degenerate_worse") != 0:
    raise SystemExit(f"{name}: trusted run unexpectedly worsened degenerates")
if trusted.get("safe_reduce_validation_orientation_worse") != 0:
    raise SystemExit(f"{name}: trusted run unexpectedly worsened orientation")
if trusted.get("safe_reduce_validation_vertex_count_mismatch") != 0:
    raise SystemExit(f"{name}: trusted run candidate vertex count mismatch")
if trusted.get("safe_reduce_validation_normal_count_mismatch") != 0:
    raise SystemExit(f"{name}: trusted run candidate normal count mismatch")
if trusted.get("safe_reduce_validation_triangle_count_mismatch") != 0:
    raise SystemExit(f"{name}: trusted run candidate triangle count mismatch")
if trusted.get("safe_reduce_validation_rolled_back") != 0:
    raise SystemExit(f"{name}: trusted run unexpectedly rolled back")
if trusted.get("safe_reduce_validation_expected_output_triangles") != trusted.get(
    "safe_reduce_output_triangles"
):
    raise SystemExit(f"{name}: trusted run expected output triangle count mismatch")
if trusted.get("safe_reduce_validation_candidate_triangles") != trusted.get(
    "safe_reduce_output_triangles"
):
    raise SystemExit(f"{name}: trusted run candidate triangle count mismatch")
if trusted.get("safe_reduce_validation_candidate_triangles") != trusted.get(
    "safe_reduce_validation_expected_output_triangles"
):
    raise SystemExit(f"{name}: trusted run candidate/expected triangle count mismatch")
if trusted.get("safe_reduce_output_misoriented_edges") != 0:
    raise SystemExit(f"{name}: trusted output has misoriented edges")
if trusted.get("safe_reduce_validation_candidate_misoriented_edges") != trusted.get(
    "safe_reduce_output_misoriented_edges"
):
    raise SystemExit(f"{name}: trusted candidate misoriented count mismatch")

print(
    f"{name}: trusted input={trusted['safe_reduce_input_triangles']} "
    f"output={trusted['safe_reduce_output_triangles']} "
    f"applied={trusted['safe_reduce_applied_components']} "
    f"skipped_default_adjacency="
    f"{trusted['safe_reduce_default_adjacency_skipped']}"
)
PY
}

"$camsim" \
  --threads 2 \
  --resolution 1 \
  --safe-reduce-report \
  --safe-reduce-provenance-neighbors \
  --no-export \
  --profile "$tmp/report.profile.json" \
  examples/cat/cat.camotics \
  >"$tmp/report.log" 2>&1

python3 - "$tmp/report.profile.json" <<'PY'
import json
import sys

metrics = json.load(open(sys.argv[1])).get("metrics", {})

if metrics.get("safe_reduce_provenance_neighbors_requested") != 1:
    raise SystemExit("provenance-neighbor report did not record request")
if metrics.get("safe_reduce_trust_provenance_neighbors_requested") != 0:
    raise SystemExit("provenance-neighbor report unexpectedly requested trust")
if metrics.get("safe_reduce_trusted_provenance_neighbors_eligible") != 0:
    raise SystemExit("provenance-neighbor report unexpectedly marked trust eligible")
if metrics.get("safe_reduce_contour_provenance_neighbor_parity") != 1:
    raise SystemExit("provenance-neighbor report lacked neighbor parity")
if metrics.get("safe_reduce_contour_provenance_neighbor_parity_audited") != 1:
    raise SystemExit("provenance-neighbor report did not audit neighbor parity")
if metrics.get("safe_reduce_contour_provenance_neighbors_cached") != 1:
    raise SystemExit("provenance-neighbor report did not use cached neighbors")
if metrics.get("safe_reduce_using_provenance_neighbors") != 1:
    raise SystemExit("provenance-neighbor report did not activate path")
PY

"$camsim" \
  --threads 2 \
  --resolution 1 \
  --safe-reduce-report \
  --safe-reduce-provenance-neighbors \
  --no-export \
  examples/cat/cat.camotics \
  >"$tmp/report-no-profile.log" 2>&1

grep -q "provenance_neighbors=yes" "$tmp/report-no-profile.log"
grep -q "using_provenance_neighbors=yes" "$tmp/report-no-profile.log"

"$camsim" \
  --threads 2 \
  --resolution 1 \
  --safe-reduce \
  --safe-reduce-provenance-neighbors \
  --no-export \
  examples/cat/cat.camotics \
  >"$tmp/reduce-no-profile.log" 2>&1

grep -q "provenance_neighbors=yes" "$tmp/reduce-no-profile.log"
grep -q "using_provenance_neighbors=yes" "$tmp/reduce-no-profile.log"

"$camsim" \
  --threads 2 \
  --resolution 1 \
  --safe-reduce \
  --profile "$tmp/default.profile.json" \
  examples/cat/cat.camotics \
  "$tmp/default.stl" \
  >"$tmp/default.log" 2>&1

"$camsim" \
  --threads 2 \
  --resolution 1 \
  --safe-reduce \
  --safe-reduce-provenance-neighbors \
  --profile "$tmp/provenance.profile.json" \
  examples/cat/cat.camotics \
  "$tmp/provenance.stl" \
  >"$tmp/provenance.log" 2>&1

python3 scripts/perf/compare_stl_geometry.py \
  "$tmp/default.stl" \
  "$tmp/provenance.stl" \
  --tolerance 1e-6

compare_profiles cat "$tmp/default.profile.json" "$tmp/provenance.profile.json"

"$camsim" \
  --threads 2 \
  --resolution 1 \
  --safe-reduce \
  --safe-reduce-provenance-neighbors \
  --safe-reduce-trust-provenance-neighbors \
  --profile "$tmp/trusted.profile.json" \
  examples/cat/cat.camotics \
  "$tmp/trusted.stl" \
  >"$tmp/trusted.log" 2>&1

python3 scripts/perf/compare_stl_geometry.py \
  "$tmp/default.stl" \
  "$tmp/trusted.stl" \
  --tolerance 1e-6

compare_trusted_profiles cat "$tmp/default.profile.json" "$tmp/trusted.profile.json"

echo "safe reduce provenance-neighbor smoke passed"
