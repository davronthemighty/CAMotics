#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

camsim="${1:-./camsim}"
tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

write_project() {
  local name="$1"
  local resolution="$2"

  cat > "$tmpdir/$name.camotics" <<JSON
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": $resolution,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 10,
      "diameter": 2
    },
    "5": {
      "units": "metric",
      "shape": "snubnose",
      "length": 20,
      "diameter": 10.917967697244908,
      "snub_diameter": 0.2
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {
      "min": [-6, -6, -0.8],
      "max": [6, 6, 0]
    }
  },
  "files": ["$name.nc"]
}
JSON
}

write_project dense-vbit 0.2
python3 - "$tmpdir/dense-vbit.nc" <<'PY'
import math
import sys

with open(sys.argv[1], "w", encoding="ascii", newline="\n") as out:
    out.write("G21\nG90\nF120\nM3 S12000\nM6 T1\n")
    out.write("G0 X-5 Y-5 Z2\nG1 Z-0.4\nG1 X5 Y-5\nG0 Z2\n")
    out.write("M6 T5\nF80\nG0 X5 Y0 Z2\nG1 Z-0.7\n")
    segments = 4000
    turns = 24
    for i in range(1, segments + 1):
        a = 2 * math.pi * turns * i / segments
        r = 5 * (1 - 0.75 * i / segments)
        out.write(f"G1 X{r * math.cos(a):.6f} Y{r * math.sin(a):.6f} Z-0.7\n")
    out.write("G0 Z2\nM5\nM30\n")
PY

write_project edge-cases 0.1
cat > "$tmpdir/edge-cases.nc" <<'NC'
G21
G90
F100
M3 S12000
M6 T5
G0 X-5 Y-5 Z2
G0 X5 Y-5 Z2
G1 X-5 Y-4 Z-0.7
G1 X5 Y-3 Z0.08
G1 X-5 Y-2 Z-0.81
G1 X5 Y-1 Z-0.7
G1 X-5 Y0 Z-0.7
G0 Z2
M6 T1
G0 X-5 Y2 Z2
G1 Z-0.4
G1 X5 Y2 Z-0.4
G0 Z2
M5
M30
NC

run_pair() {
  local name="$1"
  shift
  local args=("$@")
  local out="$tmpdir/run-$name"
  mkdir -p "$out"

  "$camsim" --threads 2 --toolsweep-xy-bins 64 "${args[@]}" \
    --profile "$out/legacy.json" "$tmpdir/$name.camotics" "$out/legacy.stl"
  "$camsim" --threads 2 --toolsweep-xy-bins 64 --toolsweep-stock-bounds \
    "${args[@]}" --profile "$out/bounded.json" \
    "$tmpdir/$name.camotics" "$out/bounded.stl"

  # Binary STL headers contain the command line, which intentionally differs
  # by --toolsweep-stock-bounds.  Compare the count and triangle payload.
  cmp -i 80 "$out/legacy.stl" "$out/bounded.stl"
  python3 scripts/perf/compare_stl_geometry.py \
    "$out/legacy.stl" "$out/bounded.stl"

  python3 - "$name" "$out/legacy.json" "$out/bounded.json" <<'PY'
import json
import sys

name, legacy_path, bounded_path = sys.argv[1:4]
with open(legacy_path, encoding="utf-8") as src:
    legacy = json.load(src)
with open(bounded_path, encoding="utf-8") as src:
    bounded = json.load(src)

lm = legacy["metrics"]
bm = bounded["metrics"]
lc = legacy["counters"]
bc = bounded["counters"]

if lm.get("toolsweep_query_bounds_enabled") != 0:
    raise SystemExit(f"{name}: legacy run unexpectedly enabled query bounds")
if bm.get("toolsweep_query_bounds_enabled") != 1:
    raise SystemExit(f"{name}: bounded run did not report query bounds")
if lm.get("surface_triangles") != bm.get("surface_triangles"):
    raise SystemExit(f"{name}: triangle count changed")

for metric in [
    "toolsweep_tool_5_unbounded_aabb_boxes",
    "toolsweep_tool_5_query_rejected_moves",
    "toolsweep_tool_5_bbox_xy_area_scaled_1e6",
    "toolsweep_tool_5_unbounded_bbox_xy_area_scaled_1e6",
]:
    if metric not in bm:
        raise SystemExit(f"{name}: missing bounded metric {metric}")

if bm["toolsweep_tool_5_bbox_xy_area_scaled_1e6"] > bm[
    "toolsweep_tool_5_unbounded_bbox_xy_area_scaled_1e6"
]:
    raise SystemExit(f"{name}: bounded V-bit area exceeds legacy area")

if name == "dense-vbit":
    if bm["toolsweep_tool_5_bbox_xy_area_scaled_1e6"] * 10 >= bm[
        "toolsweep_tool_5_unbounded_bbox_xy_area_scaled_1e6"
    ]:
        raise SystemExit(f"{name}: V-bit bbox area did not shrink by 10x")
    if bm["toolsweep_xy_bin_refs"] >= lm["toolsweep_xy_bin_refs"]:
        raise SystemExit(f"{name}: stored XY references did not decrease")
    if bc["toolsweep_xy_bin_refs_scanned"] >= lc[
        "toolsweep_xy_bin_refs_scanned"
    ]:
        raise SystemExit(f"{name}: scanned XY references did not decrease")
    if bc["toolsweep_collision_candidates"] >= lc[
        "toolsweep_collision_candidates"
    ]:
        raise SystemExit(f"{name}: exact candidate count did not decrease")

print(
    f"{name}: stock-bound ToolSweep exact smoke passed; "
    f"refs={lm['toolsweep_xy_bin_refs']}->{bm['toolsweep_xy_bin_refs']}, "
    f"candidates={lc['toolsweep_collision_candidates']}->"
    f"{bc['toolsweep_collision_candidates']}"
)
PY
}

run_pair dense-vbit
run_pair edge-cases
run_pair edge-cases --time 25

echo "ToolSweep stock-bounds smoke passed"
