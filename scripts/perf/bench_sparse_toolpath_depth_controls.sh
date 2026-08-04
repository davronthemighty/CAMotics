#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

if [ ! -x ./camsim ]; then
  echo "camsim executable not found; build it in WSL first" >&2
  exit 2
fi
if [ ! -x /usr/bin/time ]; then
  echo "/usr/bin/time is required" >&2
  exit 2
fi

output_dir="${1:-$(mktemp -d)}"
mkdir -p "$output_dir"
output_dir="$(cd "$output_dir" && pwd)"

write_fixture() {
  local name="$1"
  local depth="$2"

  cat > "$output_dir/$name.ngc" <<NC
G21
T1 M6
G0 Z2
G0 X-6 Y-6
G1 Z$depth F100
G1 X6 Y-6
G0 Z2
G0 X-6 Y-2
G1 Z$depth F100
G1 X6 Y-2
G0 Z2
G0 X-6 Y2
G1 Z$depth F100
G1 X6 Y2
G0 Z2
G0 X-6 Y6
G1 Z$depth F100
G1 X6 Y6
G0 Z2
M2
NC

  cat > "$output_dir/$name.camotics" <<JSON
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 0.5,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 20,
      "diameter": 2,
      "description": ""
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {
      "min": [-20, -20, -10],
      "max": [20, 20, 0]
    }
  },
  "files": ["$name.ngc"]
}
JSON
}

run_fixture() {
  local name="$1"

  /usr/bin/time \
    -f 'user_seconds=%U\nsystem_seconds=%S\nelapsed_seconds=%e\nmax_rss_kb=%M' \
    -o "$output_dir/$name-baseline.time" \
    ./camsim --binary --threads 4 --resolution 0.5 \
      --profile "$output_dir/$name-baseline-profile.json" \
      "$output_dir/$name.camotics" "$output_dir/$name-baseline.stl"

  /usr/bin/time \
    -f 'user_seconds=%U\nsystem_seconds=%S\nelapsed_seconds=%e\nmax_rss_kb=%M' \
    -o "$output_dir/$name-sparse.time" \
    ./camsim --binary --sparse-toolpath \
      --sparse-toolpath-xy-bins 16 \
      --sparse-toolpath-halo-cells 1 \
      --threads 4 --resolution 0.5 \
      --profile "$output_dir/$name-sparse-profile.json" \
      "$output_dir/$name.camotics" "$output_dir/$name-sparse.stl"

  python3 scripts/perf/compare_stl_distance.py \
    --max-samples 500 \
    --cell-size 1 \
    --max-shells 20 \
    --hard-max-error 0.6 \
    --p99-error 0.6 \
    "$output_dir/$name-baseline.stl" "$output_dir/$name-sparse.stl"
}

write_fixture shallow-thick -1
write_fixture deep-cut -9.5
run_fixture shallow-thick
run_fixture deep-cut

python3 - "$output_dir" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])


def read_time(path):
    values = {}
    with path.open(encoding="utf-8") as f:
        for line in f:
            key, value = line.strip().split("=", 1)
            values[key] = float(value)
    values["max_rss_kb"] = int(values["max_rss_kb"])
    return values


def load_profile(path):
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def sum_jobs(profile):
    keys = (
        "cells_visited",
        "cells_culled",
        "cells_contoured",
        "triangles",
        "vertex_samples",
        "depth_calls",
        "toolsweep_depth_calls",
        "edge_checks",
        "edge_intersections",
    )
    return {
        key: sum(job.get(key, 0) for job in profile.get("render_jobs", []))
        for key in keys
    }


report = {"schema": "camotics-sparse-depth-controls-v1", "cases": {}}
for name in ("shallow-thick", "deep-cut"):
    baseline = load_profile(root / f"{name}-baseline-profile.json")
    sparse = load_profile(root / f"{name}-sparse-profile.json")
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
        raise SystemExit(name + " sparse profile missing: " + ", ".join(missing))

    report["cases"][name] = {
        "baseline": {
            "actual": sum_jobs(baseline),
            "surface_triangles": baseline.get("metrics", {}).get(
                "surface_triangles", 0
            ),
            "time": read_time(root / f"{name}-baseline.time"),
            "stl_bytes": (root / f"{name}-baseline.stl").stat().st_size,
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
            "process_actual": sum_jobs(sparse),
            "plan": {
                "full_cells": metrics["sparse_region_plan_full_cells_est"],
                "active_cells": metrics["sparse_region_plan_active_cells_est"],
                "render_cells": metrics["sparse_region_plan_render_cells_est"],
                "max_active_depth_cells": metrics[
                    "sparse_region_plan_max_active_depth_cells"
                ],
            },
            "fallback": {
                "full_coverage": metrics[
                    "sparse_toolpath_integrated_full_coverage_fallback"
                ],
                "topology": metrics[
                    "sparse_toolpath_integrated_topology_fallback"
                ],
                "geometry": metrics[
                    "sparse_toolpath_integrated_geometry_fallback"
                ],
            },
            "surface_triangles": metrics["surface_triangles"],
            "estimated_binary_stl_bytes": metrics[
                "estimated_binary_stl_bytes"
            ],
            "time": read_time(root / f"{name}-sparse.time"),
            "stl_bytes": (root / f"{name}-sparse.stl").stat().st_size,
        },
    }

shallow = report["cases"]["shallow-thick"]
deep = report["cases"]["deep-cut"]
if shallow["sparse"]["fallback"] != {
    "full_coverage": 0, "topology": 0, "geometry": 0
}:
    raise SystemExit("shallow-thick sparse result unexpectedly fell back")
if shallow["sparse"]["attempt_actual"]["cells_visited"] >= shallow[
    "baseline"
]["actual"]["cells_visited"]:
    raise SystemExit("shallow-thick sparse render did not reduce visited cells")
if shallow["sparse"]["attempt_actual"]["depth_calls"] >= shallow[
    "baseline"
]["actual"]["depth_calls"]:
    raise SystemExit("shallow-thick sparse render did not reduce depth calls")
if shallow["sparse"]["plan"]["max_active_depth_cells"] >= deep[
    "sparse"
]["plan"]["max_active_depth_cells"]:
    raise SystemExit("deep-cut control did not increase the active Z depth")

with (root / "summary.json").open("w", encoding="utf-8") as f:
    json.dump(report, f, indent=2, sort_keys=True)
    f.write("\n")

for name, case in report["cases"].items():
    base = case["baseline"]
    sparse = case["sparse"]
    print(
        name,
        "baseline_cells=", base["actual"]["cells_visited"],
        "sparse_attempt_cells=", sparse["attempt_actual"]["cells_visited"],
        "baseline_depth_calls=", base["actual"]["depth_calls"],
        "sparse_attempt_depth_calls=", sparse["attempt_actual"]["depth_calls"],
        "max_active_depth_cells=", sparse["plan"]["max_active_depth_cells"],
        "baseline_user_s=", base["time"]["user_seconds"],
        "sparse_user_s=", sparse["time"]["user_seconds"],
        "baseline_rss_kb=", base["time"]["max_rss_kb"],
        "sparse_rss_kb=", sparse["time"]["max_rss_kb"],
        "baseline_stl_bytes=", base["stl_bytes"],
        "sparse_stl_bytes=", sparse["stl_bytes"],
        "fallback=", sparse["fallback"],
    )
PY

echo "sparse toolpath depth-control benchmark passed: $output_dir/summary.json"
