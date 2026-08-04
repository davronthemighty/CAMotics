#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

project="${1:?usage: $0 PROJECT RESOLUTION OUTPUT_DIR [XY_BINS]}"
resolution="${2:?usage: $0 PROJECT RESOLUTION OUTPUT_DIR [XY_BINS]}"
output_dir="${3:?usage: $0 PROJECT RESOLUTION OUTPUT_DIR [XY_BINS]}"
xy_bins="${4:-64}"

if [ ! -x ./camsim ] || [ ! -x /usr/bin/time ]; then
  echo "WSL camsim and /usr/bin/time are required" >&2
  exit 2
fi
project="$(realpath "$project")"
mkdir -p "$output_dir"
output_dir="$(cd "$output_dir" && pwd)"

time_format='user_seconds=%U\nsystem_seconds=%S\nelapsed_seconds=%e\nmax_rss_kb=%M'
/usr/bin/time -f "$time_format" -o "$output_dir/baseline.time" \
  ./camsim --binary --threads 4 --resolution "$resolution" \
    --profile "$output_dir/baseline-profile.json" \
    "$project" "$output_dir/baseline.stl"
/usr/bin/time -f "$time_format" -o "$output_dir/sparse.time" \
  ./camsim --binary --sparse-toolpath \
    --sparse-toolpath-xy-bins "$xy_bins" \
    --sparse-toolpath-halo-cells 1 \
    --threads 4 --resolution "$resolution" \
    --profile "$output_dir/sparse-profile.json" \
    "$project" "$output_dir/sparse.stl"

python3 scripts/perf/compare_stl_distance.py \
  --max-samples 1000 \
  --cell-size "$(python3 -c "print(float('$resolution') * 2)")" \
  --max-shells 20 \
  --hard-max-error "$(python3 -c "print(float('$resolution') * 1.2)")" \
  --p99-error "$(python3 -c "print(float('$resolution') * 1.2)")" \
  "$output_dir/baseline.stl" "$output_dir/sparse.stl"

python3 - "$project" "$resolution" "$xy_bins" "$output_dir" <<'PY'
import json
import pathlib
import sys

project, resolution, xy_bins, output_dir = sys.argv[1:]
root = pathlib.Path(output_dir)


def load(name):
    with (root / name).open(encoding="utf-8") as f:
        return json.load(f)


def read_time(name):
    values = {}
    with (root / name).open(encoding="utf-8") as f:
        for line in f:
            key, value = line.strip().split("=", 1)
            values[key] = float(value)
    values["max_rss_kb"] = int(values["max_rss_kb"])
    return values


def jobs(profile):
    keys = (
        "cells_visited", "cells_culled", "cells_contoured", "triangles",
        "vertex_samples", "depth_calls", "toolsweep_depth_calls",
        "edge_checks", "edge_intersections",
    )
    return {
        key: sum(job.get(key, 0) for job in profile.get("render_jobs", []))
        for key in keys
    }


baseline = load("baseline-profile.json")
sparse = load("sparse-profile.json")
base_metrics = baseline.get("metrics", {})
metrics = sparse.get("metrics", {})
required = (
    "sparse_region_plan_full_cells_est",
    "sparse_region_plan_active_cells_est",
    "sparse_region_plan_render_cells_est",
    "sparse_region_plan_max_active_depth_cells",
    "sparse_region_surface_cells_visited",
    "sparse_region_surface_vertex_samples",
    "sparse_region_surface_depth_calls",
    "sparse_region_surface_toolsweep_depth_calls",
    "sparse_region_surface_triangles",
    "sparse_toolpath_integrated_full_coverage_fallback",
    "sparse_toolpath_integrated_topology_fallback",
    "sparse_toolpath_integrated_geometry_fallback",
    "surface_triangles",
    "estimated_binary_stl_bytes",
)
missing = [key for key in required if key not in metrics]
if missing:
    raise SystemExit("sparse profile missing: " + ", ".join(missing))

fallback = {
    "full_coverage": metrics[
        "sparse_toolpath_integrated_full_coverage_fallback"
    ],
    "topology": metrics["sparse_toolpath_integrated_topology_fallback"],
    "geometry": metrics["sparse_toolpath_integrated_geometry_fallback"],
}
if any(value not in (0, 1) for value in fallback.values()):
    raise SystemExit("invalid sparse fallback metric")

report = {
    "schema": "camotics-sparse-project-benchmark-v1",
    "project": project,
    "resolution": float(resolution),
    "xy_bins": int(xy_bins),
    "baseline": {
        "actual": jobs(baseline),
        "surface_triangles": base_metrics.get("surface_triangles", 0),
        "estimated_binary_stl_bytes": base_metrics.get(
            "estimated_binary_stl_bytes", 0
        ),
        "time": read_time("baseline.time"),
        "stl_bytes": (root / "baseline.stl").stat().st_size,
    },
    "sparse": {
        "attempt_actual": {
            "cells_visited": metrics["sparse_region_surface_cells_visited"],
            "vertex_samples": metrics[
                "sparse_region_surface_vertex_samples"
            ],
            "depth_calls": metrics["sparse_region_surface_depth_calls"],
            "toolsweep_depth_calls": metrics[
                "sparse_region_surface_toolsweep_depth_calls"
            ],
            "triangles": metrics["sparse_region_surface_triangles"],
        },
        "process_actual": jobs(sparse),
        "plan": {
            "full_cells": metrics["sparse_region_plan_full_cells_est"],
            "active_cells": metrics["sparse_region_plan_active_cells_est"],
            "render_cells": metrics["sparse_region_plan_render_cells_est"],
            "max_active_depth_cells": metrics[
                "sparse_region_plan_max_active_depth_cells"
            ],
        },
        "fallback": fallback,
        "surface_triangles": metrics["surface_triangles"],
        "estimated_binary_stl_bytes": metrics[
            "estimated_binary_stl_bytes"
        ],
        "time": read_time("sparse.time"),
        "stl_bytes": (root / "sparse.stl").stat().st_size,
    },
}
with (root / "summary.json").open("w", encoding="utf-8") as f:
    json.dump(report, f, indent=2, sort_keys=True)
    f.write("\n")

print(json.dumps({
    "baseline_cells": report["baseline"]["actual"]["cells_visited"],
    "sparse_attempt_cells": report["sparse"]["attempt_actual"][
        "cells_visited"
    ],
    "baseline_depth_calls": report["baseline"]["actual"]["depth_calls"],
    "sparse_attempt_depth_calls": report["sparse"]["attempt_actual"][
        "depth_calls"
    ],
    "baseline_user_seconds": report["baseline"]["time"]["user_seconds"],
    "sparse_user_seconds": report["sparse"]["time"]["user_seconds"],
    "baseline_max_rss_kb": report["baseline"]["time"]["max_rss_kb"],
    "sparse_max_rss_kb": report["sparse"]["time"]["max_rss_kb"],
    "baseline_stl_bytes": report["baseline"]["stl_bytes"],
    "sparse_stl_bytes": report["sparse"]["stl_bytes"],
    "fallback": fallback,
}, sort_keys=True))
PY

echo "sparse project benchmark passed: $output_dir/summary.json"
