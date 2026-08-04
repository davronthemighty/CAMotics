#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

camsim="${1:-./camsim}"
tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

cat > "$tmpdir/reduce.nc" <<'NC'
G21
F120
M3 S1000
M6 T1
G0 X-8 Y-8 Z1
G1 Z-0.5
G1 X8 Y-8 Z-0.5
G1 X8 Y8 Z-0.5
G1 X-8 Y8 Z-0.5
G1 X-8 Y-8 Z-0.5
G0 Z1
G0 X-8 Y0 Z1
G1 Z-0.25
G1 X8 Y0 Z-0.25
G0 Z1
G0 X0 Y-8 Z1
G1 Z-0.25
G1 X0 Y8 Z-0.25
G0 Z1
NC

cat > "$tmpdir/reduce.camotics" <<'JSON'
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 0.25,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 4,
      "diameter": 0.8
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {
      "min": [-10, -10, -1],
      "max": [10, 10, 0]
    }
  },
  "files": ["reduce.nc"]
}
JSON

"$camsim" --threads 1 --profile "$tmpdir/base.json" \
  "$tmpdir/reduce.camotics" "$tmpdir/base.stl"
"$camsim" --threads 1 --profile "$tmpdir/reduced.json" --reduce \
  "$tmpdir/reduce.camotics" "$tmpdir/reduced.stl"

python3 scripts/perf/compare_stl_distance.py \
  "$tmpdir/base.stl" "$tmpdir/reduced.stl" \
  --hard-max-error 0.375 --p99-error 0.2625 --max-samples 20000

python3 - "$tmpdir/base.json" "$tmpdir/reduced.json" \
  "$tmpdir/base.stl" "$tmpdir/reduced.stl" <<'PY'
import json
import os
import sys

base_profile, reduced_profile, base_stl, reduced_stl = sys.argv[1:5]
with open(base_profile, encoding="utf-8") as f:
    base = json.load(f)
with open(reduced_profile, encoding="utf-8") as f:
    reduced = json.load(f)

base_metrics = base.get("metrics", {})
reduced_metrics = reduced.get("metrics", {})
base_counters = base.get("counters", {})
reduced_counters = reduced.get("counters", {})

base_triangles = base_metrics.get("surface_triangles", 0)
reduced_triangles = reduced_metrics.get("surface_triangles", 0)
if not base_triangles or not reduced_triangles:
    raise SystemExit("missing surface triangle metrics")
if reduced_triangles >= base_triangles * 0.25:
    raise SystemExit(
        f"reduction too small: {base_triangles} -> {reduced_triangles}"
    )

base_size = os.path.getsize(base_stl)
reduced_size = os.path.getsize(reduced_stl)
if reduced_size >= base_size * 0.25:
    raise SystemExit(f"STL size reduction too small: {base_size} -> {reduced_size}")

for metric in ("render_tree_cells", "render_partition_cells"):
    if base_metrics.get(metric) != reduced_metrics.get(metric):
        raise SystemExit(f"reduce changed render metric {metric}")

for counter in (
    "tool_sweep_depth_calls",
    "toolsweep_collision_candidates",
    "aabb_node_visits",
):
    if base_counters.get(counter) != reduced_counters.get(counter):
        raise SystemExit(f"reduce changed simulation counter {counter}")

print(
    "reduce export smoke passed: "
    f"triangles {base_triangles}->{reduced_triangles}, "
    f"bytes {base_size}->{reduced_size}"
)
PY
