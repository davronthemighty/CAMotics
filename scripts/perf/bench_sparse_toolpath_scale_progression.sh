#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

output_dir="${1:-$(mktemp -d)}"
xy_bins="${2:-64}"
mkdir -p "$output_dir"
output_dir="$(cd "$output_dir" && pwd)"

cat > "$output_dir/scale.ngc" <<'NC'
G21
T1 M6
G0 Z2
G0 X-10 Y-10
G1 Z-1 F100
G1 X10 Y-10
G0 Z2
G0 X-10 Y-5
G1 Z-1 F100
G1 X10 Y-5
G0 Z2
G0 X-10 Y0
G1 Z-1 F100
G1 X10 Y0
G0 Z2
G0 X-10 Y5
G1 Z-1 F100
G1 X10 Y5
G0 Z2
G0 X-10 Y10
G1 Z-1 F100
G1 X10 Y10
G0 Z2
M2
NC

cat > "$output_dir/scale.camotics" <<JSON
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
      "min": [-50, -50, -10],
      "max": [50, 50, 0]
    }
  },
  "files": ["scale.ngc"]
}
JSON

scripts/perf/bench_sparse_toolpath_project.sh \
  "$output_dir/scale.camotics" 0.5 "$output_dir/res-0.5" "$xy_bins"
scripts/perf/bench_sparse_toolpath_project.sh \
  "$output_dir/scale.camotics" 0.25 "$output_dir/res-0.25" "$xy_bins"

./camsim-path --threads 4 --resolution 0.025 \
  "$output_dir/scale.camotics" "$output_dir/res-0.025-toolpath.json"
./camsim-region-plan --xy-bins "$xy_bins" --halo-cells 1 \
  --target-region-cells 1000000 \
  "$output_dir/res-0.025-toolpath.json" \
  "$output_dir/res-0.025-region-plan.json"

python3 - "$output_dir" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
with (root / "res-0.5" / "summary.json").open(encoding="utf-8") as f:
    coarse = json.load(f)
with (root / "res-0.25" / "summary.json").open(encoding="utf-8") as f:
    medium = json.load(f)
with (root / "res-0.025-region-plan.json").open(encoding="utf-8") as f:
    fine_plan = json.load(f)["region-plan"]

for label, report in (("0.5", coarse), ("0.25", medium)):
    fallback = report["sparse"]["fallback"]
    if fallback != {"full_coverage": 0, "topology": 0, "geometry": 0}:
        raise SystemExit(label + " scale sparse result unexpectedly fell back")
    if report["sparse"]["attempt_actual"]["cells_visited"] >= report[
        "baseline"
    ]["actual"]["cells_visited"]:
        raise SystemExit(label + " scale sparse cells did not improve")
    if report["sparse"]["attempt_actual"]["depth_calls"] >= report[
        "baseline"
    ]["actual"]["depth_calls"]:
        raise SystemExit(label + " scale sparse depth calls did not improve")

full_cells = fine_plan["full-grid-cells-est"]
active_cells = fine_plan["active-cells-est"]
render_cells = fine_plan["render-cells-est"]
if full_cells < 6_000_000_000:
    raise SystemExit("0.025 mm planning case did not reach 6B full cells")
if not (0 < active_cells <= render_cells < full_cells):
    raise SystemExit("0.025 mm sparse plan did not reduce the full grid")

summary = {
    "schema": "camotics-sparse-scale-progression-v1",
    "actual_runs": {
        "0.5": coarse,
        "0.25": medium,
    },
    "planning_only_0.025": {
        "full_cells": full_cells,
        "active_cells": active_cells,
        "render_cells": render_cells,
        "skipped_cells": fine_plan["skipped-cells-est"],
        "adaptive_leaf_count": fine_plan["adaptive-leaf-count"],
        "adaptive_max_leaf_cells": fine_plan["adaptive-max-leaf-cells"],
        "target_exceeded_leaves": fine_plan[
            "adaptive-target-exceeded-leaves"
        ],
        "full_render_launched": False,
    },
}
with (root / "summary.json").open("w", encoding="utf-8") as f:
    json.dump(summary, f, indent=2, sort_keys=True)
    f.write("\n")

for label, report in (("0.5", coarse), ("0.25", medium)):
    print(
        label,
        "baseline_cells", report["baseline"]["actual"]["cells_visited"],
        "sparse_cells", report["sparse"]["attempt_actual"]["cells_visited"],
        "baseline_user_s", report["baseline"]["time"]["user_seconds"],
        "sparse_user_s", report["sparse"]["time"]["user_seconds"],
        "baseline_rss_kb", report["baseline"]["time"]["max_rss_kb"],
        "sparse_rss_kb", report["sparse"]["time"]["max_rss_kb"],
    )
print("0.025 planning", summary["planning_only_0.025"])
PY

echo "sparse scale progression passed: $output_dir/summary.json"
