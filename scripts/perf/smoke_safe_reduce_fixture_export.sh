#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

camsim="${1:-./camsim}"

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

cases="${CAMOTICS_SAFE_REDUCE_EXPORT_CASES:-cat}"
exact_tolerance="${CAMOTICS_SAFE_REDUCE_EXPORT_EXACT_TOLERANCE:-1e-6}"
trusted_reference_requested="${CAMOTICS_SAFE_REDUCE_TRUST_PROVENANCE_NEIGHBORS:-0}"
safe_reduce_reference_flags=()
safe_reduce_extra_flags=()
if [ "${CAMOTICS_SAFE_REDUCE_HOLE_AWARE:-0}" = "1" ]; then
  safe_reduce_reference_flags+=(--safe-reduce-hole-aware)
  safe_reduce_extra_flags+=(--safe-reduce-hole-aware)
fi
if [ "${CAMOTICS_SAFE_REDUCE_PROVENANCE_NEIGHBORS:-0}" = "1" ] ||
   [ "${CAMOTICS_SAFE_REDUCE_TRUST_PROVENANCE_NEIGHBORS:-0}" = "1" ]; then
  safe_reduce_extra_flags+=(--safe-reduce-provenance-neighbors)
fi
if [ "${CAMOTICS_SAFE_REDUCE_TRUST_PROVENANCE_NEIGHBORS:-0}" = "1" ]; then
  safe_reduce_extra_flags+=(--safe-reduce-trust-provenance-neighbors)
fi

compare_trusted_reference() {
  local name=$1
  local reference_profile=$2
  local trusted_profile=$3

  python3 - "$name" "$reference_profile" "$trusted_profile" <<'PY'
import json
import sys

name = sys.argv[1]
reference = json.load(open(sys.argv[2])).get("metrics", {})
trusted = json.load(open(sys.argv[3])).get("metrics", {})

semantic_keys = (
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
    "safe_reduce_replacement_edge_incidence_checks",
    "safe_reduce_replacement_edge_incidence_rejected",
    "safe_reduce_phase1_replacement_edge_incidence_rejected",
    "safe_reduce_hole_aware_replacement_edge_incidence_rejected",
    "safe_reduce_rejected_triangulation_components",
    "safe_reduce_rejected_triangulation_triangles",
    "safe_reduce_applied_components",
    "safe_reduce_applied_source_triangles",
    "safe_reduce_applied_output_triangles",
    "safe_reduce_hole_aware_applied_components",
    "safe_reduce_hole_aware_applied_source_triangles",
    "safe_reduce_hole_aware_applied_output_triangles",
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
)

for key in semantic_keys:
    if reference.get(key) != trusted.get(key):
        raise SystemExit(
            f"{name}: trusted export changed {key}: "
            f"{reference.get(key)} != {trusted.get(key)}"
        )

if trusted.get("safe_reduce_provenance_neighbors_requested") != 1:
    raise SystemExit(f"{name}: trusted export did not request provenance neighbors")
if trusted.get("safe_reduce_trust_provenance_neighbors_requested") != 1:
    raise SystemExit(f"{name}: trusted export did not record trust request")
if trusted.get("safe_reduce_trusted_provenance_neighbors_eligible") != 1:
    raise SystemExit(f"{name}: trusted export was not eligible")
if trusted.get("safe_reduce_using_provenance_neighbors") != 1:
    raise SystemExit(f"{name}: trusted export did not use provenance neighbors")
if trusted.get("safe_reduce_trusted_provenance_neighbors_used") != 1:
    raise SystemExit(f"{name}: trusted export did not activate trusted neighbors")
if trusted.get("safe_reduce_default_adjacency_skipped") != 1:
    raise SystemExit(f"{name}: trusted export did not skip default adjacency")
if trusted.get("safe_reduce_contour_provenance_neighbor_parity_audited") != 0:
    raise SystemExit(f"{name}: trusted export unexpectedly audited default adjacency")
if trusted.get("safe_reduce_trusted_provenance_neighbor_slots_checked", 0) <= 0:
    raise SystemExit(f"{name}: trusted export did not validate neighbor slots")
if trusted.get("safe_reduce_trusted_provenance_neighbor_edge_slots_checked", 0) <= 0:
    raise SystemExit(f"{name}: trusted export did not validate neighbor edge slots")
if trusted.get("safe_reduce_trusted_provenance_neighbor_edge_mismatches", 0) != 0:
    raise SystemExit(f"{name}: trusted export found neighbor edge mismatches")

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
        raise SystemExit(f"{name}: trusted export rejected via {key}")

print(
    f"{name}: trusted export matches ordinary safe-reduce "
    f"triangles={trusted['safe_reduce_output_triangles']} "
    f"applied={trusted['safe_reduce_applied_components']} "
    f"skipped_default_adjacency="
    f"{trusted['safe_reduce_default_adjacency_skipped']}"
)
PY
}

run_export() {
  local name=$1
  local project=$2
  local comparator=$3
  local edge_tolerance=$4
  shift 4

  if [ ! -f "$project" ]; then
    echo "skip $name: missing $project"
    return
  fi

  local dir="$tmp/$name"
  mkdir -p "$dir"

  "$camsim" \
    "$@" \
    --profile "$dir/baseline.profile.json" \
    "$project" \
    "$dir/baseline.stl" \
    >"$dir/baseline.log" 2>&1

  "$camsim" \
    "$@" \
    --safe-reduce \
    "${safe_reduce_extra_flags[@]}" \
    --profile "$dir/safe-reduce.profile.json" \
    "$project" \
    "$dir/safe-reduce.stl" \
    >"$dir/safe-reduce.log" 2>&1

  python3 scripts/perf/mesh_reduction_contract_report.py \
    --edge-incidence-only \
    --coord-tolerance "$edge_tolerance" \
    "$dir/safe-reduce.stl" \
    --output-json "$dir/edge.json" \
    >"$dir/edge-report.log"

  if [ "$comparator" = "distance" ]; then
    python3 scripts/perf/compare_stl_distance.py \
      "$dir/baseline.stl" \
      "$dir/safe-reduce.stl" \
      --cut-surface \
      --cut-z-epsilon 0.025 \
      --cut-xy-epsilon 0.025 \
      --hard-max-error 0.075 \
      --p99-error 0.0525 \
      >"$dir/distance.log"

  elif [ "$comparator" = "heightfield" ]; then
    python3 scripts/perf/compare_stl_cut_heightfield.py \
      "$dir/baseline.stl" \
      "$dir/safe-reduce.stl" \
      --cell-size 0.05 \
      --cut-z-epsilon 0.025 \
      --cut-xy-epsilon 0.025 \
      --point-mode centroid \
      --hard-max-error 0.075 \
      --p99-error 0.0525 \
      --max-missing-cell-ratio 0.001 \
      >"$dir/heightfield.log"

  elif [ "$comparator" = "centroid-plane" ]; then
    python3 scripts/perf/compare_stl_cut_centroid_plane.py \
      "$dir/baseline.stl" \
      "$dir/safe-reduce.stl" \
      --cell-size 0.25 \
      --cut-z-epsilon 0.025 \
      --cut-xy-epsilon 0.025 \
      --hard-max-error 0.075 \
      --p99-error 0.0525 \
      --oriented-normals \
      --max-normal-angle 0.25 \
      --p99-normal-angle 0.25 \
      --max-skipped-normal-ratio 0.01 \
      --max-unmatched-ratio 0.01 \
      --bidirectional \
      >"$dir/centroid-plane.log"

  else
    echo "unknown comparator for $name: $comparator" >&2
    return 2
  fi

  if [ "$trusted_reference_requested" = "1" ]; then
    "$camsim" \
      "$@" \
      --safe-reduce \
      "${safe_reduce_reference_flags[@]}" \
      --profile "$dir/reference-safe-reduce.profile.json" \
      "$project" \
      "$dir/reference-safe-reduce.stl" \
      >"$dir/reference-safe-reduce.log" 2>&1

    python3 scripts/perf/compare_stl_geometry.py \
      "$dir/reference-safe-reduce.stl" \
      "$dir/safe-reduce.stl" \
      --tolerance "$exact_tolerance" \
      >"$dir/trusted-reference-exact.log"

    compare_trusted_reference \
      "$name" \
      "$dir/reference-safe-reduce.profile.json" \
      "$dir/safe-reduce.profile.json"
  fi

  python3 - \
    "$name" \
    "$dir/edge.json" \
    "$dir/safe-reduce.profile.json" \
    "${CAMOTICS_SAFE_REDUCE_HOLE_AWARE:-0}" \
    "${CAMOTICS_SAFE_REDUCE_PROVENANCE_NEIGHBORS:-0}" \
    "${CAMOTICS_SAFE_REDUCE_TRUST_PROVENANCE_NEIGHBORS:-0}" <<'PY'
import json
import sys

name = sys.argv[1]
edge = json.load(open(sys.argv[2]))
profile = json.load(open(sys.argv[3]))
expect_hole_aware = sys.argv[4] == "1"
expect_provenance = sys.argv[5] == "1"
expect_trust = sys.argv[6] == "1"
metrics = profile.get("metrics", {})
incidence = edge.get("strict_edge_incidence") or {}

if incidence.get("boundary_edges") != 0:
    raise SystemExit(
        f"{name}: safe-reduce export has boundary edges "
        f"{incidence.get('boundary_edges')}"
    )
if incidence.get("nonmanifold_edges") != 0:
    raise SystemExit(
        f"{name}: safe-reduce export has nonmanifold edges "
        f"{incidence.get('nonmanifold_edges')}"
    )
if incidence.get("misoriented_edges") != 0:
    raise SystemExit(
        f"{name}: safe-reduce export has misoriented edges "
        f"{incidence.get('misoriented_edges')}"
    )
if metrics.get("safe_reduce_output_watertight") != 1:
    raise SystemExit(f"{name}: profile did not report watertight output")
if metrics.get("safe_reduce_output_boundary_edges") != 0:
    raise SystemExit(f"{name}: profile reported output boundary edges")
if metrics.get("safe_reduce_output_nonmanifold_edges") != 0:
    raise SystemExit(f"{name}: profile reported output nonmanifold edges")
if metrics.get("safe_reduce_output_misoriented_edges") != 0:
    raise SystemExit(f"{name}: profile reported output misoriented edges")
if metrics.get("safe_reduce_output_degenerate_triangles") != 0:
    raise SystemExit(f"{name}: profile reported output degenerate triangles")
if metrics.get("safe_reduce_source_expected_floats") != metrics.get(
    "safe_reduce_input_triangles", 0
) * 9:
    raise SystemExit(f"{name}: source expected-float count mismatch")
if metrics.get("safe_reduce_source_vertex_floats") != metrics.get(
    "safe_reduce_source_expected_floats"
):
    raise SystemExit(f"{name}: source vertex float count mismatch")
if metrics.get("safe_reduce_source_normal_floats") != metrics.get(
    "safe_reduce_source_expected_floats"
):
    raise SystemExit(f"{name}: source normal float count mismatch")
if metrics.get("safe_reduce_source_vertex_count_mismatch") != 0:
    raise SystemExit(f"{name}: unexpectedly flagged source vertices")
if metrics.get("safe_reduce_source_normal_count_mismatch") != 0:
    raise SystemExit(f"{name}: unexpectedly flagged source normals")
if (
    metrics.get("safe_reduce_writable_replacement_checks", 0)
    + metrics.get("safe_reduce_unwritable_replacement_checks", 0)
    != metrics.get("safe_reduce_estimated_replacement_checks", 0)
):
    raise SystemExit(f"{name}: writable/unwritable replacement accounting changed")
if (
    metrics.get("safe_reduce_phase1_unwritable_replacement_checks", 0)
    + metrics.get("safe_reduce_hole_aware_unwritable_replacement_checks", 0)
    != metrics.get("safe_reduce_unwritable_replacement_checks", 0)
):
    raise SystemExit(f"{name}: unwritable replacement buckets changed")
if metrics.get("safe_reduce_validation_topology_worse") != 0:
    raise SystemExit(f"{name}: safe-reduce unexpectedly worsened topology")
if metrics.get("safe_reduce_validation_degenerate_worse") != 0:
    raise SystemExit(f"{name}: safe-reduce unexpectedly worsened degenerates")
if metrics.get("safe_reduce_validation_orientation_worse") != 0:
    raise SystemExit(f"{name}: safe-reduce unexpectedly worsened orientation")
if metrics.get("safe_reduce_validation_vertex_count_mismatch") != 0:
    raise SystemExit(f"{name}: safe-reduce candidate vertex count mismatch")
if metrics.get("safe_reduce_validation_normal_count_mismatch") != 0:
    raise SystemExit(f"{name}: safe-reduce candidate normal count mismatch")
if metrics.get("safe_reduce_validation_triangle_count_mismatch") != 0:
    raise SystemExit(f"{name}: safe-reduce candidate triangle count mismatch")
if metrics.get("safe_reduce_validation_rolled_back") != 0:
    raise SystemExit(f"{name}: safe-reduce unexpectedly rolled back")
if metrics.get("safe_reduce_validation_expected_output_triangles") != metrics.get(
    "safe_reduce_output_triangles"
):
    raise SystemExit(f"{name}: safe-reduce expected output triangle count mismatch")
if metrics.get("safe_reduce_validation_candidate_triangles") != metrics.get(
    "safe_reduce_output_triangles"
):
    raise SystemExit(f"{name}: safe-reduce candidate triangle count mismatch")
if metrics.get("safe_reduce_validation_candidate_triangles") != metrics.get(
    "safe_reduce_validation_expected_output_triangles"
):
    raise SystemExit(f"{name}: safe-reduce candidate/expected triangle count mismatch")
if metrics.get("safe_reduce_validation_candidate_misoriented_edges") != metrics.get(
    "safe_reduce_output_misoriented_edges"
):
    raise SystemExit(f"{name}: safe-reduce candidate misoriented count mismatch")
if metrics.get("safe_reduce_applied_components", 0) <= 0:
    raise SystemExit(f"{name}: safe-reduce applied no components")
if metrics.get("safe_reduce_output_triangles", 0) >= metrics.get(
    "safe_reduce_input_triangles", 0
):
    raise SystemExit(f"{name}: safe-reduce did not lower triangle count")
input_bytes = 84 + 50 * metrics.get("safe_reduce_input_triangles", 0)
output_bytes = 84 + 50 * metrics.get("safe_reduce_output_triangles", 0)
estimated_bytes = 84 + 50 * metrics.get("safe_reduce_estimated_triangles_after", 0)
if metrics.get("safe_reduce_input_binary_stl_bytes") != input_bytes:
    raise SystemExit(f"{name}: safe-reduce input byte estimate mismatch")
if metrics.get("safe_reduce_output_binary_stl_bytes") != output_bytes:
    raise SystemExit(f"{name}: safe-reduce output byte estimate mismatch")
if metrics.get("safe_reduce_estimated_binary_stl_bytes_after") != estimated_bytes:
    raise SystemExit(f"{name}: safe-reduce estimated byte output mismatch")
if metrics.get("safe_reduce_output_binary_stl_bytes_saved") != input_bytes - output_bytes:
    raise SystemExit(f"{name}: safe-reduce output byte savings mismatch")
if metrics.get("safe_reduce_estimated_binary_stl_bytes_saved") != input_bytes - estimated_bytes:
    raise SystemExit(f"{name}: safe-reduce estimated byte savings mismatch")
if metrics.get("safe_reduce_output_binary_stl_bytes_saved", 0) <= 0:
    raise SystemExit(f"{name}: safe-reduce reported no byte savings")
if expect_hole_aware:
    if metrics.get("safe_reduce_hole_aware_apply_requested") != 1:
        raise SystemExit(f"{name}: hole-aware flag was not recorded")
    if metrics.get("safe_reduce_hole_aware_components", 0) <= 0:
        raise SystemExit(f"{name}: no hole-aware candidates were reported")
    if metrics.get("safe_reduce_hole_aware_applied_components", 0) <= 0:
        raise SystemExit(f"{name}: no hole-aware candidates were applied")
else:
    if metrics.get("safe_reduce_hole_aware_apply_requested") != 0:
        raise SystemExit(f"{name}: unexpected hole-aware apply request")
    if metrics.get("safe_reduce_hole_aware_applied_components", 0) != 0:
        raise SystemExit(f"{name}: default safe-reduce applied hole-aware components")

if expect_trust:
    expect_provenance = True

if expect_provenance:
    if metrics.get("safe_reduce_provenance_neighbors_requested") != 1:
        raise SystemExit(f"{name}: provenance-neighbor request not recorded")
    if metrics.get("safe_reduce_using_provenance_neighbors") != 1:
        raise SystemExit(f"{name}: provenance-neighbor path was not used")
    if not expect_trust and metrics.get("safe_reduce_contour_provenance_neighbor_parity_audited") != 1:
        raise SystemExit(f"{name}: provenance-neighbor parity was not audited")
else:
    if metrics.get("safe_reduce_provenance_neighbors_requested") != 0:
        raise SystemExit(f"{name}: unexpected provenance-neighbor request")
    if metrics.get("safe_reduce_using_provenance_neighbors") != 0:
        raise SystemExit(f"{name}: unexpected provenance-neighbor use")
    if metrics.get("safe_reduce_contour_provenance_neighbor_parity_audited") != 0:
        raise SystemExit(f"{name}: unexpected provenance-neighbor parity audit")

if expect_trust:
    if metrics.get("safe_reduce_trust_provenance_neighbors_requested") != 1:
        raise SystemExit(f"{name}: trusted provenance request not recorded")
    if metrics.get("safe_reduce_trusted_provenance_neighbors_used") != 1:
        raise SystemExit(f"{name}: trusted provenance path was not used")
    if metrics.get("safe_reduce_default_adjacency_skipped") != 1:
        raise SystemExit(f"{name}: trusted provenance did not skip adjacency")
    if metrics.get("safe_reduce_trusted_provenance_neighbor_slots_checked", 0) <= 0:
        raise SystemExit(f"{name}: trusted provenance did not validate neighbor slots")
    if metrics.get("safe_reduce_trusted_provenance_neighbor_edge_slots_checked", 0) <= 0:
        raise SystemExit(f"{name}: trusted provenance did not validate neighbor edge slots")
    if metrics.get("safe_reduce_trusted_provenance_neighbor_edge_mismatches", 0) != 0:
        raise SystemExit(f"{name}: trusted provenance found neighbor edge mismatches")
    if metrics.get("safe_reduce_contour_provenance_neighbor_parity_audited") != 0:
        raise SystemExit(f"{name}: trusted provenance unexpectedly audited skipped adjacency")
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
        if metrics.get(key) != 0:
            raise SystemExit(f"{name}: trusted provenance rejected via {key}")
else:
    if metrics.get("safe_reduce_trust_provenance_neighbors_requested") != 0:
        raise SystemExit(f"{name}: unexpected trusted provenance request")
    if metrics.get("safe_reduce_trusted_provenance_neighbors_used") != 0:
        raise SystemExit(f"{name}: unexpected trusted provenance use")
    if metrics.get("safe_reduce_default_adjacency_skipped") != 0:
        raise SystemExit(f"{name}: unexpected default adjacency skip")

print(
    f"{name}: input={metrics['safe_reduce_input_triangles']} "
    f"output={metrics['safe_reduce_output_triangles']} "
    f"applied={metrics['safe_reduce_applied_components']} "
    f"hole_aware_applied={metrics.get('safe_reduce_hole_aware_applied_components', 0)} "
    f"using_provenance={metrics.get('safe_reduce_using_provenance_neighbors', 0)} "
    f"trusted_provenance={metrics.get('safe_reduce_trusted_provenance_neighbors_used', 0)} "
    f"boundary_edges={incidence.get('boundary_edges')} "
    f"nonmanifold_edges={incidence.get('nonmanifold_edges')}"
)
PY
}

run_cat() {
  run_export \
    cat \
    examples/cat/cat.camotics \
    distance \
    0.001 \
    --threads "${CAMOTICS_SAFE_REDUCE_EXPORT_THREADS:-2}" \
    --resolution 1
}

IFS=',' read -ra selected_cases <<< "$cases"
for selected in "${selected_cases[@]}"; do
  case "$selected" in
    cat) run_cat ;;
    "")
      ;;
    *)
      echo "unknown CAMOTICS_SAFE_REDUCE_EXPORT_CASES entry: $selected" >&2
      exit 2
      ;;
  esac
done

echo "safe reduce fixture export validation passed"
