#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

camsim="${1:-./camsim}"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

thread_list="${CAMOTICS_SAFE_REDUCE_TRUSTED_THREAD_LIST:-1 2 4 10}"

cat > "$tmp/boundary_plane.nc" <<'NC'
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

cat > "$tmp/boundary_plane.camotics" <<JSON
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 0.5,
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
      "min": [-4, -4, -3],
      "max": [28, 28, 1]
    }
  },
  "files": ["boundary_plane.nc"]
}
JSON

"$camsim" \
  --threads 1 \
  --resolution 1 \
  --safe-reduce \
  --safe-reduce-hole-aware \
  --profile "$tmp/default.profile.json" \
  examples/cat/cat.camotics \
  "$tmp/default.stl" \
  >"$tmp/default.log" 2>&1

for threads in $thread_list; do
  "$camsim" \
    --threads "$threads" \
    --resolution 1 \
    --safe-reduce \
    --safe-reduce-hole-aware \
    --safe-reduce-provenance-neighbors \
    --safe-reduce-trust-provenance-neighbors \
    --profile "$tmp/trusted-t$threads.profile.json" \
    examples/cat/cat.camotics \
    "$tmp/trusted-t$threads.stl" \
    >"$tmp/trusted-t$threads.log" 2>&1

  python3 scripts/perf/compare_stl_geometry.py \
    "$tmp/default.stl" \
    "$tmp/trusted-t$threads.stl" \
    --tolerance 1e-6
done

python3 - "$tmp/default.profile.json" "$thread_list" "$tmp" <<'PY'
import json
import sys
from pathlib import Path

default = json.load(open(sys.argv[1])).get("metrics", {})
threads = sys.argv[2].split()
root = Path(sys.argv[3])

if default.get("safe_reduce_output_watertight") != 1:
    raise SystemExit("default safe-reduce output was not watertight")
if default.get("safe_reduce_output_boundary_edges") != 0:
    raise SystemExit("default safe-reduce output had boundary edges")
if default.get("safe_reduce_output_nonmanifold_edges") != 0:
    raise SystemExit("default safe-reduce output had nonmanifold edges")
if default.get("safe_reduce_output_misoriented_edges") != 0:
    raise SystemExit("default safe-reduce output had misoriented edges")
if default.get("safe_reduce_hole_aware_apply_requested") != 1:
    raise SystemExit("default safe-reduce did not request hole-aware mode")
if default.get("safe_reduce_hole_aware_applied_components", 0) <= 0:
    raise SystemExit("default safe-reduce applied no hole-aware components")

reference = None
default_summary = {
    "input": default.get("safe_reduce_input_triangles"),
    "output": default.get("safe_reduce_output_triangles"),
    "input_bytes": default.get("safe_reduce_input_binary_stl_bytes"),
    "output_bytes": default.get("safe_reduce_output_binary_stl_bytes"),
    "output_bytes_saved": default.get("safe_reduce_output_binary_stl_bytes_saved"),
    "estimated_bytes": default.get("safe_reduce_estimated_binary_stl_bytes_after"),
    "estimated_bytes_saved": default.get("safe_reduce_estimated_binary_stl_bytes_saved"),
    "applied_components": default.get("safe_reduce_applied_components"),
    "applied_source": default.get("safe_reduce_applied_source_triangles"),
    "applied_output": default.get("safe_reduce_applied_output_triangles"),
    "hole_aware_applied": default.get("safe_reduce_hole_aware_applied_components"),
    "estimated_replacements": default.get("safe_reduce_estimated_replacement_checks"),
    "feasible_replacements": default.get("safe_reduce_feasible_replacement_checks"),
    "writable_replacements": default.get("safe_reduce_writable_replacement_checks"),
    "unwritable_replacements": default.get("safe_reduce_unwritable_replacement_checks"),
    "phase1_writable": default.get("safe_reduce_phase1_writable_replacement_checks"),
    "hole_aware_writable": default.get("safe_reduce_hole_aware_writable_replacement_checks"),
    "phase1_unwritable": default.get("safe_reduce_phase1_unwritable_replacement_checks"),
    "hole_aware_unwritable": default.get("safe_reduce_hole_aware_unwritable_replacement_checks"),
    "local_incidence_checks": default.get("safe_reduce_replacement_edge_incidence_checks"),
    "local_incidence_rejected": default.get("safe_reduce_replacement_edge_incidence_rejected"),
    "phase1_incidence_rejected": default.get("safe_reduce_phase1_replacement_edge_incidence_rejected"),
    "hole_aware_incidence_rejected": default.get("safe_reduce_hole_aware_replacement_edge_incidence_rejected"),
}
for thread in threads:
    metrics = json.load(
        open(root / f"trusted-t{thread}.profile.json")
    ).get("metrics", {})

    required = [
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
        "safe_reduce_applied_components",
        "safe_reduce_applied_source_triangles",
        "safe_reduce_applied_output_triangles",
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
        "safe_reduce_hole_aware_apply_requested",
        "safe_reduce_hole_aware_applied_components",
        "safe_reduce_provenance_neighbors_requested",
        "safe_reduce_using_provenance_neighbors",
        "safe_reduce_trust_provenance_neighbors_requested",
        "safe_reduce_trusted_provenance_neighbors_eligible",
        "safe_reduce_trusted_provenance_neighbors_used",
        "safe_reduce_trusted_provenance_neighbor_edge_slots_checked",
        "safe_reduce_trusted_provenance_neighbor_edge_mismatches",
        "safe_reduce_default_adjacency_skipped",
        "safe_reduce_contour_provenance_neighbor_parity_audited",
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
    ]
    missing = [key for key in required if key not in metrics]
    if missing:
        raise SystemExit(f"threads={thread}: missing metrics {missing}")

    if metrics["safe_reduce_output_watertight"] != 1:
        raise SystemExit(f"threads={thread}: output not watertight")
    if metrics["safe_reduce_source_expected_floats"] != metrics["safe_reduce_input_triangles"] * 9:
        raise SystemExit(f"threads={thread}: source expected-float count mismatch")
    if metrics["safe_reduce_source_vertex_floats"] != metrics["safe_reduce_source_expected_floats"]:
        raise SystemExit(f"threads={thread}: source vertex float count mismatch")
    if metrics["safe_reduce_source_normal_floats"] != metrics["safe_reduce_source_expected_floats"]:
        raise SystemExit(f"threads={thread}: source normal float count mismatch")
    if metrics["safe_reduce_source_vertex_count_mismatch"] != 0:
        raise SystemExit(f"threads={thread}: source vertex count mismatch")
    if metrics["safe_reduce_source_normal_count_mismatch"] != 0:
        raise SystemExit(f"threads={thread}: source normal count mismatch")
    if metrics["safe_reduce_validation_topology_worse"] != 0:
        raise SystemExit(f"threads={thread}: trusted run worsened topology")
    if metrics["safe_reduce_validation_degenerate_worse"] != 0:
        raise SystemExit(f"threads={thread}: trusted run worsened degenerates")
    if metrics["safe_reduce_validation_orientation_worse"] != 0:
        raise SystemExit(f"threads={thread}: trusted run worsened orientation")
    if metrics["safe_reduce_validation_vertex_count_mismatch"] != 0:
        raise SystemExit(f"threads={thread}: candidate vertex count mismatch")
    if metrics["safe_reduce_validation_normal_count_mismatch"] != 0:
        raise SystemExit(f"threads={thread}: candidate normal count mismatch")
    if metrics["safe_reduce_validation_triangle_count_mismatch"] != 0:
        raise SystemExit(f"threads={thread}: candidate triangle count mismatch")
    if metrics["safe_reduce_validation_rolled_back"] != 0:
        raise SystemExit(f"threads={thread}: trusted run rolled back")
    if metrics["safe_reduce_validation_expected_output_triangles"] != metrics["safe_reduce_output_triangles"]:
        raise SystemExit(f"threads={thread}: expected output triangle count mismatch")
    if metrics["safe_reduce_validation_candidate_triangles"] != metrics["safe_reduce_output_triangles"]:
        raise SystemExit(f"threads={thread}: candidate triangle count mismatch")
    if metrics["safe_reduce_validation_candidate_triangles"] != metrics["safe_reduce_validation_expected_output_triangles"]:
        raise SystemExit(f"threads={thread}: candidate/expected triangle count mismatch")
    if metrics["safe_reduce_validation_candidate_checked"] != 1:
        raise SystemExit(f"threads={thread}: candidate was not validated")
    if metrics["safe_reduce_validation_candidate_boundary_edges"] != metrics["safe_reduce_output_boundary_edges"]:
        raise SystemExit(f"threads={thread}: candidate boundary count mismatch")
    if metrics["safe_reduce_validation_candidate_nonmanifold_edges"] != metrics["safe_reduce_output_nonmanifold_edges"]:
        raise SystemExit(f"threads={thread}: candidate nonmanifold count mismatch")
    if metrics["safe_reduce_validation_candidate_misoriented_edges"] != metrics["safe_reduce_output_misoriented_edges"]:
        raise SystemExit(f"threads={thread}: candidate misoriented count mismatch")
    if metrics["safe_reduce_validation_candidate_degenerate_triangles"] != metrics["safe_reduce_output_degenerate_triangles"]:
        raise SystemExit(f"threads={thread}: candidate degenerate count mismatch")
    if metrics["safe_reduce_validation_candidate_watertight"] != metrics["safe_reduce_output_watertight"]:
        raise SystemExit(f"threads={thread}: candidate watertightness mismatch")
    if metrics["safe_reduce_output_boundary_edges"] != 0:
        raise SystemExit(f"threads={thread}: output boundary edges")
    if metrics["safe_reduce_output_nonmanifold_edges"] != 0:
        raise SystemExit(f"threads={thread}: output nonmanifold edges")
    if metrics["safe_reduce_output_misoriented_edges"] != 0:
        raise SystemExit(f"threads={thread}: output misoriented edges")
    if metrics["safe_reduce_output_degenerate_triangles"] != 0:
        raise SystemExit(f"threads={thread}: output degenerate triangles")
    if metrics["safe_reduce_hole_aware_apply_requested"] != 1:
        raise SystemExit(f"threads={thread}: hole-aware flag not recorded")
    if metrics["safe_reduce_hole_aware_applied_components"] <= 0:
        raise SystemExit(f"threads={thread}: no hole-aware components applied")
    if metrics["safe_reduce_writable_replacement_checks"] != metrics["safe_reduce_feasible_replacement_checks"]:
        raise SystemExit(f"threads={thread}: writable/feasible replacement checks diverged")
    if (
        metrics["safe_reduce_writable_replacement_checks"]
        + metrics["safe_reduce_unwritable_replacement_checks"]
        != metrics["safe_reduce_estimated_replacement_checks"]
    ):
        raise SystemExit(f"threads={thread}: writable/unwritable replacement accounting changed")
    if (
        metrics["safe_reduce_phase1_unwritable_replacement_checks"]
        + metrics["safe_reduce_hole_aware_unwritable_replacement_checks"]
        != metrics["safe_reduce_unwritable_replacement_checks"]
    ):
        raise SystemExit(f"threads={thread}: unwritable replacement buckets diverged")
    if (
        metrics["safe_reduce_phase1_replacement_edge_incidence_rejected"]
        + metrics["safe_reduce_hole_aware_replacement_edge_incidence_rejected"]
        != metrics["safe_reduce_replacement_edge_incidence_rejected"]
    ):
        raise SystemExit(f"threads={thread}: local incidence rejection buckets diverged")
    if metrics["safe_reduce_replacement_edge_incidence_checks"] < metrics["safe_reduce_writable_replacement_checks"]:
        raise SystemExit(f"threads={thread}: local incidence checks missed writable replacements")
    if metrics["safe_reduce_hole_aware_writable_replacement_checks"] <= 0:
        raise SystemExit(f"threads={thread}: no writable hole-aware replacements")
    if metrics["safe_reduce_hole_aware_writable_replacement_checks"] < metrics["safe_reduce_hole_aware_applied_components"]:
        raise SystemExit(f"threads={thread}: applied more hole-aware components than were writable")
    if metrics["safe_reduce_provenance_neighbors_requested"] != 1:
        raise SystemExit(f"threads={thread}: provenance request not recorded")
    if metrics["safe_reduce_using_provenance_neighbors"] != 1:
        raise SystemExit(f"threads={thread}: provenance path not used")
    if metrics["safe_reduce_trust_provenance_neighbors_requested"] != 1:
        raise SystemExit(f"threads={thread}: trust request not recorded")
    if metrics["safe_reduce_trusted_provenance_neighbors_eligible"] != 1:
        raise SystemExit(f"threads={thread}: trusted path not eligible")
    if metrics["safe_reduce_trusted_provenance_neighbors_used"] != 1:
        raise SystemExit(f"threads={thread}: trusted path not used")
    if metrics["safe_reduce_default_adjacency_skipped"] != 1:
        raise SystemExit(f"threads={thread}: default adjacency not skipped")
    if metrics.get("safe_reduce_trusted_provenance_neighbor_slots_checked", 0) <= 0:
        raise SystemExit(f"threads={thread}: trusted neighbor slots were not checked")
    if metrics.get("safe_reduce_trusted_provenance_neighbor_edge_slots_checked", 0) <= 0:
        raise SystemExit(f"threads={thread}: trusted neighbor edge slots were not checked")
    if metrics.get("safe_reduce_trusted_provenance_neighbor_edge_mismatches", 0) != 0:
        raise SystemExit(f"threads={thread}: trusted neighbor edge mismatches")
    if metrics["safe_reduce_contour_provenance_neighbor_parity_audited"] != 0:
        raise SystemExit(f"threads={thread}: skipped adjacency was audited")

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
            raise SystemExit(f"threads={thread}: rejected trusted path via {key}")

    summary = {
        "input": metrics["safe_reduce_input_triangles"],
        "output": metrics["safe_reduce_output_triangles"],
        "input_bytes": metrics["safe_reduce_input_binary_stl_bytes"],
        "output_bytes": metrics["safe_reduce_output_binary_stl_bytes"],
        "output_bytes_saved": metrics["safe_reduce_output_binary_stl_bytes_saved"],
        "estimated_bytes": metrics["safe_reduce_estimated_binary_stl_bytes_after"],
        "estimated_bytes_saved": metrics["safe_reduce_estimated_binary_stl_bytes_saved"],
        "applied_components": metrics["safe_reduce_applied_components"],
        "applied_source": metrics["safe_reduce_applied_source_triangles"],
        "applied_output": metrics["safe_reduce_applied_output_triangles"],
        "hole_aware_applied": metrics["safe_reduce_hole_aware_applied_components"],
        "estimated_replacements": metrics["safe_reduce_estimated_replacement_checks"],
        "feasible_replacements": metrics["safe_reduce_feasible_replacement_checks"],
        "writable_replacements": metrics["safe_reduce_writable_replacement_checks"],
        "unwritable_replacements": metrics["safe_reduce_unwritable_replacement_checks"],
        "phase1_writable": metrics["safe_reduce_phase1_writable_replacement_checks"],
        "hole_aware_writable": metrics["safe_reduce_hole_aware_writable_replacement_checks"],
        "phase1_unwritable": metrics["safe_reduce_phase1_unwritable_replacement_checks"],
        "hole_aware_unwritable": metrics["safe_reduce_hole_aware_unwritable_replacement_checks"],
        "local_incidence_checks": metrics["safe_reduce_replacement_edge_incidence_checks"],
        "local_incidence_rejected": metrics["safe_reduce_replacement_edge_incidence_rejected"],
        "phase1_incidence_rejected": metrics["safe_reduce_phase1_replacement_edge_incidence_rejected"],
        "hole_aware_incidence_rejected": metrics["safe_reduce_hole_aware_replacement_edge_incidence_rejected"],
    }
    if reference is None:
        reference = summary
        if summary != default_summary:
            raise SystemExit(
                f"threads={thread}: trusted reducer metrics differ from default: "
                f"{summary} != {default_summary}"
            )
    elif summary != reference:
        raise SystemExit(
            f"threads={thread}: trusted reducer metrics differ: "
            f"{summary} != {reference}"
        )

print(
    "cat trusted cross-thread smoke passed: "
    + ", ".join(f"threads={thread}" for thread in threads)
)
PY

"$camsim" \
  --threads 1 \
  --safe-reduce \
  --safe-reduce-hole-aware \
  --profile "$tmp/boundary-default.profile.json" \
  "$tmp/boundary_plane.camotics" \
  "$tmp/boundary-default.stl" \
  >"$tmp/boundary-default.log" 2>&1

for threads in $thread_list; do
  "$camsim" \
    --threads "$threads" \
    --safe-reduce \
    --safe-reduce-hole-aware \
    --safe-reduce-provenance-neighbors \
    --safe-reduce-trust-provenance-neighbors \
    --profile "$tmp/boundary-trusted-t$threads.profile.json" \
    "$tmp/boundary_plane.camotics" \
    "$tmp/boundary-trusted-t$threads.stl" \
    >"$tmp/boundary-trusted-t$threads.log" 2>&1

  python3 scripts/perf/compare_stl_geometry.py \
    "$tmp/boundary-default.stl" \
    "$tmp/boundary-trusted-t$threads.stl" \
    --tolerance 1e-6
done

python3 - "$tmp/boundary-default.profile.json" "$thread_list" "$tmp" <<'PY'
import json
import sys
from pathlib import Path

default = json.load(open(sys.argv[1])).get("metrics", {})
threads = sys.argv[2].split()
root = Path(sys.argv[3])

summary_keys = (
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
    "safe_reduce_applied_components",
    "safe_reduce_applied_source_triangles",
    "safe_reduce_applied_output_triangles",
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

if default.get("safe_reduce_output_watertight") != 1:
    raise SystemExit("boundary-plane default output was not watertight")
if default.get("safe_reduce_output_boundary_edges") != 0:
    raise SystemExit("boundary-plane default output had boundary edges")
if default.get("safe_reduce_output_nonmanifold_edges") != 0:
    raise SystemExit("boundary-plane default output had nonmanifold edges")
if default.get("safe_reduce_output_misoriented_edges") != 0:
    raise SystemExit("boundary-plane default output had misoriented edges")
if default.get("safe_reduce_applied_components", 0) <= 0:
    raise SystemExit("boundary-plane default applied no safe-reduce components")

reference = None
default_summary = {key: default.get(key) for key in summary_keys}

for thread in threads:
    metrics = json.load(
        open(root / f"boundary-trusted-t{thread}.profile.json")
    ).get("metrics", {})

    missing = [key for key in summary_keys if key not in metrics]
    if missing:
        raise SystemExit(f"boundary-plane threads={thread}: missing {missing}")

    summary = {key: metrics.get(key) for key in summary_keys}
    if summary != default_summary:
        raise SystemExit(
            f"boundary-plane threads={thread}: trusted metrics differ from "
            f"default: {summary} != {default_summary}"
        )
    if reference is None:
        reference = summary
    elif summary != reference:
        raise SystemExit(
            f"boundary-plane threads={thread}: trusted metrics changed: "
            f"{summary} != {reference}"
        )

    if metrics["safe_reduce_writable_replacement_checks"] != metrics["safe_reduce_feasible_replacement_checks"]:
        raise SystemExit(
            f"boundary-plane threads={thread}: writable/feasible checks diverged"
        )
    if (
        metrics["safe_reduce_writable_replacement_checks"]
        + metrics["safe_reduce_unwritable_replacement_checks"]
        != metrics["safe_reduce_estimated_replacement_checks"]
    ):
        raise SystemExit(
            f"boundary-plane threads={thread}: writable/unwritable accounting changed"
        )
    if (
        metrics["safe_reduce_phase1_unwritable_replacement_checks"]
        + metrics["safe_reduce_hole_aware_unwritable_replacement_checks"]
        != metrics["safe_reduce_unwritable_replacement_checks"]
    ):
        raise SystemExit(
            f"boundary-plane threads={thread}: unwritable buckets diverged"
        )
    if (
        metrics["safe_reduce_phase1_replacement_edge_incidence_rejected"]
        + metrics["safe_reduce_hole_aware_replacement_edge_incidence_rejected"]
        != metrics["safe_reduce_replacement_edge_incidence_rejected"]
    ):
        raise SystemExit(
            f"boundary-plane threads={thread}: local incidence buckets diverged"
        )
    if metrics["safe_reduce_replacement_edge_incidence_checks"] < metrics["safe_reduce_writable_replacement_checks"]:
        raise SystemExit(
            f"boundary-plane threads={thread}: local incidence checks missed writable replacements"
        )

    if metrics["safe_reduce_provenance_neighbors_requested"] != 1:
        raise SystemExit(f"boundary-plane threads={thread}: provenance not requested")
    if metrics["safe_reduce_using_provenance_neighbors"] != 1:
        raise SystemExit(f"boundary-plane threads={thread}: provenance path unused")
    if metrics["safe_reduce_trust_provenance_neighbors_requested"] != 1:
        raise SystemExit(f"boundary-plane threads={thread}: trust not requested")
    if metrics["safe_reduce_trusted_provenance_neighbors_eligible"] != 1:
        raise SystemExit(f"boundary-plane threads={thread}: trusted path ineligible")
    if metrics["safe_reduce_trusted_provenance_neighbors_used"] != 1:
        raise SystemExit(f"boundary-plane threads={thread}: trusted path unused")
    if metrics["safe_reduce_default_adjacency_skipped"] != 1:
        raise SystemExit(
            f"boundary-plane threads={thread}: default adjacency was not skipped"
        )
    if metrics.get("safe_reduce_trusted_provenance_neighbor_slots_checked", 0) <= 0:
        raise SystemExit(
            f"boundary-plane threads={thread}: trusted neighbor slots unchecked"
        )
    if metrics.get("safe_reduce_trusted_provenance_neighbor_edge_slots_checked", 0) <= 0:
        raise SystemExit(
            f"boundary-plane threads={thread}: trusted edge slots unchecked"
        )
    if metrics.get("safe_reduce_trusted_provenance_neighbor_edge_mismatches", 0) != 0:
        raise SystemExit(
            f"boundary-plane threads={thread}: trusted edge mismatches"
        )

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
            raise SystemExit(
                f"boundary-plane threads={thread}: rejected trusted path via {key}"
            )

print(
    "boundary-plane trusted cross-thread smoke passed: "
    + ", ".join(f"threads={thread}" for thread in threads)
)
PY

echo "safe reduce trusted cross-thread smoke passed"
