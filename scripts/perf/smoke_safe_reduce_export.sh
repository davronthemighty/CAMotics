#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

camsim="${1:-./camsim}"

tmp="${TMPDIR:-/tmp}/camotics-safe-reduce-export-$$"
mkdir -p "$tmp"
trap 'rm -rf "$tmp"' EXIT

"$camsim" \
  --threads 2 \
  --resolution 1 \
  --profile "$tmp/baseline-profile.json" \
  examples/cat/cat.camotics \
  "$tmp/cat-baseline.stl" \
  >"$tmp/baseline.log" 2>&1

"$camsim" \
  --threads 2 \
  --resolution 1 \
  --safe-reduce \
  --profile "$tmp/safe-reduce-profile.json" \
  examples/cat/cat.camotics \
  "$tmp/cat-safe-reduce.stl" \
  >"$tmp/safe-reduce.log" 2>&1

"$camsim" \
  --threads 2 \
  --resolution 1 \
  --safe-reduce \
  --safe-reduce-hole-aware \
  --profile "$tmp/safe-reduce-hole-aware-profile.json" \
  examples/cat/cat.camotics \
  "$tmp/cat-safe-reduce-hole-aware.stl" \
  >"$tmp/safe-reduce-hole-aware.log" 2>&1

python3 scripts/perf/compare_stl_distance.py \
  "$tmp/cat-baseline.stl" \
  "$tmp/cat-safe-reduce.stl" \
  --hard-max-error 1.5 \
  --p99-error 1.05

python3 scripts/perf/compare_stl_distance.py \
  "$tmp/cat-baseline.stl" \
  "$tmp/cat-safe-reduce-hole-aware.stl" \
  --hard-max-error 1.5 \
  --p99-error 1.05

python3 scripts/perf/mesh_reduction_contract_report.py \
  --edge-incidence \
  --coord-tolerance 0.001 \
  "$tmp/cat-safe-reduce.stl" \
  --output-json "$tmp/edge.json" \
  >"$tmp/edge-report.log"

python3 scripts/perf/mesh_reduction_contract_report.py \
  --edge-incidence \
  --coord-tolerance 0.001 \
  "$tmp/cat-safe-reduce-hole-aware.stl" \
  --output-json "$tmp/edge-hole-aware.json" \
  >"$tmp/edge-hole-aware-report.log"

python3 - "$tmp/edge.json" "$tmp/safe-reduce-profile.json" "$tmp/edge-hole-aware.json" "$tmp/safe-reduce-hole-aware-profile.json" <<'PY'
import json
import sys

edge = json.load(open(sys.argv[1]))
profile = json.load(open(sys.argv[2]))
hole_edge = json.load(open(sys.argv[3]))
hole_profile = json.load(open(sys.argv[4]))
metrics = profile.get("metrics", {})
incidence = edge.get("strict_edge_incidence") or {}
hole_metrics = hole_profile.get("metrics", {})
hole_incidence = hole_edge.get("strict_edge_incidence") or {}

if incidence.get("boundary_edges") != 0:
    raise SystemExit(
        f"safe-reduce output has boundary edges: "
        f"{incidence.get('boundary_edges')}"
    )
if incidence.get("nonmanifold_edges") != 0:
    raise SystemExit(
        f"safe-reduce output has nonmanifold edges: "
        f"{incidence.get('nonmanifold_edges')}"
    )
if incidence.get("misoriented_edges") != 0:
    raise SystemExit(
        f"safe-reduce output has misoriented edges: "
        f"{incidence.get('misoriented_edges')}"
    )
if metrics.get("safe_reduce_applied_components", 0) <= 0:
    raise SystemExit("safe-reduce export applied no components")
if metrics.get("safe_reduce_applied_output_triangles", 0) <= 0:
    raise SystemExit("safe-reduce export applied no output triangles")
if metrics.get("safe_reduce_output_watertight") != 1:
    raise SystemExit("safe-reduce profile did not report watertight output")
if metrics.get("safe_reduce_output_boundary_edges") != 0:
    raise SystemExit("safe-reduce profile reported output boundary edges")
if metrics.get("safe_reduce_output_nonmanifold_edges") != 0:
    raise SystemExit("safe-reduce profile reported output nonmanifold edges")
if metrics.get("safe_reduce_output_misoriented_edges") != 0:
    raise SystemExit("safe-reduce profile reported output misoriented edges")
if metrics.get("safe_reduce_output_degenerate_triangles") != 0:
    raise SystemExit("safe-reduce profile reported output degenerate triangles")
if metrics.get("safe_reduce_output_triangles", 0) >= metrics.get(
    "safe_reduce_input_triangles", 0
):
    raise SystemExit("safe-reduce export did not lower triangle count")
input_bytes = 84 + 50 * metrics.get("safe_reduce_input_triangles", 0)
output_bytes = 84 + 50 * metrics.get("safe_reduce_output_triangles", 0)
estimated_bytes = 84 + 50 * metrics.get("safe_reduce_estimated_triangles_after", 0)
if metrics.get("safe_reduce_input_binary_stl_bytes") != input_bytes:
    raise SystemExit("safe-reduce default input byte estimate mismatch")
if metrics.get("safe_reduce_output_binary_stl_bytes") != output_bytes:
    raise SystemExit("safe-reduce default output byte estimate mismatch")
if metrics.get("safe_reduce_estimated_binary_stl_bytes_after") != estimated_bytes:
    raise SystemExit("safe-reduce default estimated byte output mismatch")
if metrics.get("safe_reduce_output_binary_stl_bytes_saved") != input_bytes - output_bytes:
    raise SystemExit("safe-reduce default output byte savings mismatch")
if metrics.get("safe_reduce_estimated_binary_stl_bytes_saved") != input_bytes - estimated_bytes:
    raise SystemExit("safe-reduce default estimated byte savings mismatch")
if metrics.get("safe_reduce_output_binary_stl_bytes_saved", 0) <= 0:
    raise SystemExit("safe-reduce default reported no byte savings")
if metrics.get("safe_reduce_hole_aware_apply_requested") != 0:
    raise SystemExit("safe-reduce default export requested hole-aware apply")
if metrics.get("safe_reduce_hole_aware_applied_components") != 0:
    raise SystemExit("safe-reduce default export applied hole-aware components")
if metrics.get("safe_reduce_source_expected_floats") != metrics.get(
    "safe_reduce_input_triangles", 0
) * 9:
    raise SystemExit("safe-reduce default source expected-float count mismatch")
if metrics.get("safe_reduce_source_vertex_floats") != metrics.get(
    "safe_reduce_source_expected_floats"
):
    raise SystemExit("safe-reduce default source vertex float count mismatch")
if metrics.get("safe_reduce_source_normal_floats") != metrics.get(
    "safe_reduce_source_expected_floats"
):
    raise SystemExit("safe-reduce default source normal float count mismatch")
if metrics.get("safe_reduce_source_vertex_count_mismatch") != 0:
    raise SystemExit("safe-reduce default unexpectedly flagged source vertices")
if metrics.get("safe_reduce_source_normal_count_mismatch") != 0:
    raise SystemExit("safe-reduce default unexpectedly flagged source normals")
if metrics.get("safe_reduce_validation_topology_worse") != 0:
    raise SystemExit("safe-reduce default export unexpectedly worsened topology")
if metrics.get("safe_reduce_validation_degenerate_worse") != 0:
    raise SystemExit("safe-reduce default export unexpectedly worsened degenerates")
if metrics.get("safe_reduce_validation_orientation_worse") != 0:
    raise SystemExit("safe-reduce default export unexpectedly worsened orientation")
if metrics.get("safe_reduce_validation_vertex_count_mismatch") != 0:
    raise SystemExit("safe-reduce default export candidate vertex count mismatch")
if metrics.get("safe_reduce_validation_normal_count_mismatch") != 0:
    raise SystemExit("safe-reduce default export candidate normal count mismatch")
if metrics.get("safe_reduce_validation_triangle_count_mismatch") != 0:
    raise SystemExit("safe-reduce default export candidate triangle count mismatch")
if metrics.get("safe_reduce_validation_rolled_back") != 0:
    raise SystemExit("safe-reduce default export unexpectedly rolled back")
if metrics.get("safe_reduce_validation_expected_output_triangles") != metrics.get(
    "safe_reduce_output_triangles"
):
    raise SystemExit("safe-reduce default expected output triangle count mismatch")
if metrics.get("safe_reduce_validation_candidate_triangles") != metrics.get(
    "safe_reduce_output_triangles"
):
    raise SystemExit("safe-reduce default candidate triangle count mismatch")
if metrics.get("safe_reduce_validation_candidate_triangles") != metrics.get(
    "safe_reduce_validation_expected_output_triangles"
):
    raise SystemExit("safe-reduce default candidate/expected triangle count mismatch")
if metrics.get("safe_reduce_validation_candidate_checked") != 1:
    raise SystemExit("safe-reduce default export did not validate a candidate")
if metrics.get("safe_reduce_validation_candidate_boundary_edges") != metrics.get(
    "safe_reduce_output_boundary_edges"
):
    raise SystemExit("safe-reduce default candidate boundary count mismatch")
if metrics.get("safe_reduce_validation_candidate_nonmanifold_edges") != metrics.get(
    "safe_reduce_output_nonmanifold_edges"
):
    raise SystemExit("safe-reduce default candidate nonmanifold count mismatch")
if metrics.get("safe_reduce_validation_candidate_misoriented_edges") != metrics.get(
    "safe_reduce_output_misoriented_edges"
):
    raise SystemExit("safe-reduce default candidate misoriented count mismatch")
if metrics.get("safe_reduce_validation_candidate_degenerate_triangles") != metrics.get(
    "safe_reduce_output_degenerate_triangles"
):
    raise SystemExit("safe-reduce default candidate degenerate count mismatch")
if metrics.get("safe_reduce_validation_candidate_watertight") != metrics.get(
    "safe_reduce_output_watertight"
):
    raise SystemExit("safe-reduce default candidate watertightness mismatch")

if hole_incidence.get("boundary_edges") != 0:
    raise SystemExit(
        f"safe-reduce hole-aware output has boundary edges: "
        f"{hole_incidence.get('boundary_edges')}"
    )
if hole_incidence.get("nonmanifold_edges") != 0:
    raise SystemExit(
        f"safe-reduce hole-aware output has nonmanifold edges: "
        f"{hole_incidence.get('nonmanifold_edges')}"
    )
if hole_incidence.get("misoriented_edges") != 0:
    raise SystemExit(
        f"safe-reduce hole-aware output has misoriented edges: "
        f"{hole_incidence.get('misoriented_edges')}"
    )
if hole_metrics.get("safe_reduce_hole_aware_apply_requested") != 1:
    raise SystemExit("safe-reduce hole-aware export did not record the flag")
if hole_metrics.get("safe_reduce_hole_aware_applied_components", 0) <= 0:
    raise SystemExit("safe-reduce hole-aware export applied no multi-loop components")
if hole_metrics.get("safe_reduce_hole_aware_applied_source_triangles", 0) <= hole_metrics.get(
    "safe_reduce_hole_aware_applied_output_triangles", 0
):
    raise SystemExit("safe-reduce hole-aware export did not reduce multi-loop triangles")
if hole_metrics.get("safe_reduce_output_watertight") != 1:
    raise SystemExit("safe-reduce hole-aware profile did not report watertight output")
if hole_metrics.get("safe_reduce_output_boundary_edges") != 0:
    raise SystemExit("safe-reduce hole-aware profile reported output boundary edges")
if hole_metrics.get("safe_reduce_output_nonmanifold_edges") != 0:
    raise SystemExit("safe-reduce hole-aware profile reported output nonmanifold edges")
if hole_metrics.get("safe_reduce_output_misoriented_edges") != 0:
    raise SystemExit("safe-reduce hole-aware profile reported output misoriented edges")
if hole_metrics.get("safe_reduce_output_degenerate_triangles") != 0:
    raise SystemExit("safe-reduce hole-aware profile reported output degenerate triangles")
if hole_metrics.get("safe_reduce_output_triangles", 0) >= metrics.get(
    "safe_reduce_output_triangles", 0
):
    raise SystemExit("safe-reduce hole-aware export did not further lower triangle count")
hole_input_bytes = 84 + 50 * hole_metrics.get("safe_reduce_input_triangles", 0)
hole_output_bytes = 84 + 50 * hole_metrics.get("safe_reduce_output_triangles", 0)
hole_estimated_bytes = 84 + 50 * hole_metrics.get(
    "safe_reduce_estimated_triangles_after", 0
)
if hole_metrics.get("safe_reduce_input_binary_stl_bytes") != hole_input_bytes:
    raise SystemExit("safe-reduce hole-aware input byte estimate mismatch")
if hole_metrics.get("safe_reduce_output_binary_stl_bytes") != hole_output_bytes:
    raise SystemExit("safe-reduce hole-aware output byte estimate mismatch")
if hole_metrics.get("safe_reduce_estimated_binary_stl_bytes_after") != hole_estimated_bytes:
    raise SystemExit("safe-reduce hole-aware estimated byte output mismatch")
if hole_metrics.get("safe_reduce_output_binary_stl_bytes_saved") != hole_input_bytes - hole_output_bytes:
    raise SystemExit("safe-reduce hole-aware output byte savings mismatch")
if hole_metrics.get("safe_reduce_estimated_binary_stl_bytes_saved") != hole_input_bytes - hole_estimated_bytes:
    raise SystemExit("safe-reduce hole-aware estimated byte savings mismatch")
if hole_metrics.get("safe_reduce_output_binary_stl_bytes_saved", 0) <= metrics.get(
    "safe_reduce_output_binary_stl_bytes_saved", 0
):
    raise SystemExit("safe-reduce hole-aware byte savings did not improve default")
if hole_metrics.get("safe_reduce_source_expected_floats") != hole_metrics.get(
    "safe_reduce_input_triangles", 0
) * 9:
    raise SystemExit("safe-reduce hole-aware source expected-float count mismatch")
if hole_metrics.get("safe_reduce_source_vertex_floats") != hole_metrics.get(
    "safe_reduce_source_expected_floats"
):
    raise SystemExit("safe-reduce hole-aware source vertex float count mismatch")
if hole_metrics.get("safe_reduce_source_normal_floats") != hole_metrics.get(
    "safe_reduce_source_expected_floats"
):
    raise SystemExit("safe-reduce hole-aware source normal float count mismatch")
if hole_metrics.get("safe_reduce_source_vertex_count_mismatch") != 0:
    raise SystemExit("safe-reduce hole-aware unexpectedly flagged source vertices")
if hole_metrics.get("safe_reduce_source_normal_count_mismatch") != 0:
    raise SystemExit("safe-reduce hole-aware unexpectedly flagged source normals")
if hole_metrics.get("safe_reduce_validation_topology_worse") != 0:
    raise SystemExit("safe-reduce hole-aware export unexpectedly worsened topology")
if hole_metrics.get("safe_reduce_validation_degenerate_worse") != 0:
    raise SystemExit("safe-reduce hole-aware export unexpectedly worsened degenerates")
if hole_metrics.get("safe_reduce_validation_orientation_worse") != 0:
    raise SystemExit("safe-reduce hole-aware export unexpectedly worsened orientation")
if hole_metrics.get("safe_reduce_validation_vertex_count_mismatch") != 0:
    raise SystemExit("safe-reduce hole-aware export candidate vertex count mismatch")
if hole_metrics.get("safe_reduce_validation_normal_count_mismatch") != 0:
    raise SystemExit("safe-reduce hole-aware export candidate normal count mismatch")
if hole_metrics.get("safe_reduce_validation_triangle_count_mismatch") != 0:
    raise SystemExit("safe-reduce hole-aware export candidate triangle count mismatch")
if hole_metrics.get("safe_reduce_validation_rolled_back") != 0:
    raise SystemExit("safe-reduce hole-aware export unexpectedly rolled back")
if hole_metrics.get("safe_reduce_validation_expected_output_triangles") != hole_metrics.get(
    "safe_reduce_output_triangles"
):
    raise SystemExit("safe-reduce hole-aware expected output triangle count mismatch")
if hole_metrics.get("safe_reduce_validation_candidate_triangles") != hole_metrics.get(
    "safe_reduce_output_triangles"
):
    raise SystemExit("safe-reduce hole-aware candidate triangle count mismatch")
if hole_metrics.get("safe_reduce_validation_candidate_triangles") != hole_metrics.get(
    "safe_reduce_validation_expected_output_triangles"
):
    raise SystemExit("safe-reduce hole-aware candidate/expected triangle count mismatch")
if hole_metrics.get("safe_reduce_validation_candidate_checked") != 1:
    raise SystemExit("safe-reduce hole-aware export did not validate a candidate")
if hole_metrics.get("safe_reduce_validation_candidate_boundary_edges") != hole_metrics.get(
    "safe_reduce_output_boundary_edges"
):
    raise SystemExit("safe-reduce hole-aware candidate boundary count mismatch")
if hole_metrics.get("safe_reduce_validation_candidate_nonmanifold_edges") != hole_metrics.get(
    "safe_reduce_output_nonmanifold_edges"
):
    raise SystemExit("safe-reduce hole-aware candidate nonmanifold count mismatch")
if hole_metrics.get("safe_reduce_validation_candidate_misoriented_edges") != hole_metrics.get(
    "safe_reduce_output_misoriented_edges"
):
    raise SystemExit("safe-reduce hole-aware candidate misoriented count mismatch")
if hole_metrics.get("safe_reduce_validation_candidate_degenerate_triangles") != hole_metrics.get(
    "safe_reduce_output_degenerate_triangles"
):
    raise SystemExit("safe-reduce hole-aware candidate degenerate count mismatch")
if hole_metrics.get("safe_reduce_validation_candidate_watertight") != hole_metrics.get(
    "safe_reduce_output_watertight"
):
    raise SystemExit("safe-reduce hole-aware candidate watertightness mismatch")
PY

echo "safe reduce export validation passed"
