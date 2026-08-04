#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

run_case() {
  local name=$1
  local project=$2
  shift 2

  if [ ! -f "$project" ]; then
    echo "skip $name: missing $project"
    return
  fi

  mkdir -p "$tmp/$name"
  ./camsim \
    "$@" \
    --safe-reduce-report \
    --profile-only \
    --profile "$tmp/$name/profile.json" \
    "$project" \
    >"$tmp/$name/camsim.log" 2>&1

  python3 - "$name" "$tmp/$name/profile.json" <<'PY'
import json
import sys

name = sys.argv[1]
profile = json.load(open(sys.argv[2]))
metrics = profile.get("metrics", {})

semantic_pairs = [
    ("safe_reduce_components",
     "safe_reduce_contour_provenance_components"),
    ("safe_reduce_estimated_triangles_after",
     "safe_reduce_contour_provenance_estimated_triangles_after"),
    ("safe_reduce_estimated_triangle_reduction",
     "safe_reduce_contour_provenance_estimated_triangle_reduction"),
    ("safe_reduce_component_decision_fingerprint",
     "safe_reduce_contour_provenance_component_decision_fingerprint"),
    ("safe_reduce_decision_bearing_components",
     "safe_reduce_contour_provenance_decision_bearing_components"),
    ("safe_reduce_decision_bearing_triangles",
     "safe_reduce_contour_provenance_decision_bearing_triangles"),
    ("safe_reduce_phase1_components",
     "safe_reduce_contour_provenance_phase1_components"),
    ("safe_reduce_hole_aware_components",
     "safe_reduce_contour_provenance_hole_aware_components"),
    ("safe_reduce_estimated_replacement_checks",
     "safe_reduce_contour_provenance_estimated_replacement_checks"),
    ("safe_reduce_feasible_replacement_checks",
     "safe_reduce_contour_provenance_feasible_replacement_checks"),
    ("safe_reduce_writable_replacement_checks",
     "safe_reduce_contour_provenance_writable_replacement_checks"),
    ("safe_reduce_unwritable_replacement_checks",
     "safe_reduce_contour_provenance_unwritable_replacement_checks"),
    ("safe_reduce_phase1_writable_replacement_checks",
     "safe_reduce_contour_provenance_phase1_writable_replacement_checks"),
    ("safe_reduce_hole_aware_writable_replacement_checks",
     "safe_reduce_contour_provenance_hole_aware_writable_replacement_checks"),
    ("safe_reduce_phase1_unwritable_replacement_checks",
     "safe_reduce_contour_provenance_phase1_unwritable_replacement_checks"),
    ("safe_reduce_hole_aware_unwritable_replacement_checks",
     "safe_reduce_contour_provenance_hole_aware_unwritable_replacement_checks"),
    ("safe_reduce_replacement_edge_incidence_checks",
     "safe_reduce_contour_provenance_replacement_edge_incidence_checks"),
    ("safe_reduce_replacement_edge_incidence_rejected",
     "safe_reduce_contour_provenance_replacement_edge_incidence_rejected"),
    ("safe_reduce_phase1_replacement_edge_incidence_rejected",
     "safe_reduce_contour_provenance_phase1_replacement_edge_incidence_rejected"),
    ("safe_reduce_hole_aware_replacement_edge_incidence_rejected",
     "safe_reduce_contour_provenance_hole_aware_replacement_edge_incidence_rejected"),
    ("safe_reduce_rejected_boundary_components",
     "safe_reduce_contour_provenance_rejected_boundary_components"),
    ("safe_reduce_rejected_no_savings_components",
     "safe_reduce_contour_provenance_rejected_no_savings_components"),
    ("safe_reduce_rejected_triangulation_components",
     "safe_reduce_contour_provenance_rejected_triangulation_components"),
]

required = [
    "safe_reduce_input_triangles",
    "safe_reduce_source_expected_floats",
    "safe_reduce_source_vertex_floats",
    "safe_reduce_source_normal_floats",
    "safe_reduce_source_vertex_count_mismatch",
    "safe_reduce_source_normal_count_mismatch",
    "safe_reduce_contour_provenance_component_report_available",
    "safe_reduce_contour_provenance_neighbor_mismatches",
    "safe_reduce_contour_provenance_neighbor_parity",
    "safe_reduce_contour_provenance_component_metric_mismatches",
    "safe_reduce_contour_provenance_component_parity",
]
for left, right in semantic_pairs:
    required.extend([left, right])

missing = [key for key in required if key not in metrics]
if missing:
    raise SystemExit(f"{name}: missing metrics {missing}")

if metrics["safe_reduce_contour_provenance_neighbor_mismatches"] != 0:
    raise SystemExit(f"{name}: provenance neighbor mismatch")
if metrics["safe_reduce_source_expected_floats"] != metrics["safe_reduce_input_triangles"] * 9:
    raise SystemExit(f"{name}: source expected-float count mismatch")
if metrics["safe_reduce_source_vertex_floats"] != metrics["safe_reduce_source_expected_floats"]:
    raise SystemExit(f"{name}: source vertex float count mismatch")
if metrics["safe_reduce_source_normal_floats"] != metrics["safe_reduce_source_expected_floats"]:
    raise SystemExit(f"{name}: source normal float count mismatch")
if metrics["safe_reduce_source_vertex_count_mismatch"] != 0:
    raise SystemExit(f"{name}: source vertex count mismatch")
if metrics["safe_reduce_source_normal_count_mismatch"] != 0:
    raise SystemExit(f"{name}: source normal count mismatch")
if metrics["safe_reduce_contour_provenance_neighbor_parity"] != 1:
    raise SystemExit(f"{name}: provenance neighbor parity failed")
if metrics["safe_reduce_contour_provenance_component_report_available"] != 1:
    raise SystemExit(f"{name}: provenance component report unavailable")
for left, right in semantic_pairs:
    if metrics[right] != metrics[left]:
        raise SystemExit(
            f"{name}: provenance metric mismatch for {left}: "
            f"{metrics[left]} != {metrics[right]}"
        )
if metrics["safe_reduce_contour_provenance_component_metric_mismatches"] != 0:
    raise SystemExit(f"{name}: provenance component metric mismatch")
if metrics["safe_reduce_contour_provenance_component_parity"] != 1:
    raise SystemExit(f"{name}: provenance component parity failed")
if metrics["safe_reduce_decision_bearing_components"] <= 0:
    raise SystemExit(f"{name}: no decision-bearing components")
if metrics["safe_reduce_decision_bearing_triangles"] <= 0:
    raise SystemExit(f"{name}: no decision-bearing triangles")
if (
    metrics["safe_reduce_phase1_replacement_edge_incidence_rejected"]
    + metrics["safe_reduce_hole_aware_replacement_edge_incidence_rejected"]
    != metrics["safe_reduce_replacement_edge_incidence_rejected"]
):
    raise SystemExit(f"{name}: local incidence rejection buckets diverged")
if metrics["safe_reduce_replacement_edge_incidence_checks"] < metrics["safe_reduce_writable_replacement_checks"]:
    raise SystemExit(f"{name}: local incidence checks missed writable replacements")

print(
    f"{name}: triangles={metrics['safe_reduce_input_triangles']} "
    f"components={metrics['safe_reduce_components']} "
    f"phase1={metrics['safe_reduce_phase1_components']} "
    f"hole_aware={metrics['safe_reduce_hole_aware_components']} "
    f"writable={metrics['safe_reduce_writable_replacement_checks']} "
    f"incidence_rejected="
    f"{metrics['safe_reduce_replacement_edge_incidence_rejected']} "
    f"estimated_reduction="
    f"{metrics['safe_reduce_estimated_triangle_reduction']}"
)
PY
}

run_case \
  cat \
  examples/cat/cat.camotics \
  --threads "${CAMOTICS_SAFE_REDUCE_CAT_THREADS:-2}" \
  --resolution 1

echo "safe reduce provenance parity smoke passed"
