#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

camsim="${1:-./camsim}"
tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

cat > "$tmpdir/dense-vbit.camotics" <<'JSON'
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 0.2,
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
  "files": ["dense-vbit.nc"]
}
JSON

python3 - "$tmpdir/dense-vbit.nc" <<'PY'
import math
import sys

path = sys.argv[1]
with open(path, "w", encoding="ascii", newline="\n") as out:
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

profile="$tmpdir/dense-vbit.profile.json"
"$camsim" --threads 2 --toolsweep-xy-bins 64 --profile-only \
  --profile "$profile" "$tmpdir/dense-vbit.camotics"

python3 - "$profile" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as src:
    profile = json.load(src)

m = profile["metrics"]

required = [
    "toolsweep_tool_1_moves",
    "toolsweep_tool_1_cutting_moves",
    "toolsweep_tool_1_aabb_boxes",
    "toolsweep_tool_1_unbounded_aabb_boxes",
    "toolsweep_tool_1_bbox_xy_area_scaled_1e6",
    "toolsweep_tool_1_unbounded_bbox_xy_area_scaled_1e6",
    "toolsweep_tool_1_path_length_microunits",
    "toolsweep_tool_1_shape_cylindrical",
    "toolsweep_tool_1_xy_bin_refs",
    "toolsweep_tool_1_xy_bin_max_box_refs",
    "toolsweep_tool_5_moves",
    "toolsweep_tool_5_cutting_moves",
    "toolsweep_tool_5_aabb_boxes",
    "toolsweep_tool_5_unbounded_aabb_boxes",
    "toolsweep_tool_5_bbox_xy_area_scaled_1e6",
    "toolsweep_tool_5_unbounded_bbox_xy_area_scaled_1e6",
    "toolsweep_tool_5_path_length_microunits",
    "toolsweep_tool_5_shape_snubnose",
    "toolsweep_tool_5_radius_microunits",
    "toolsweep_tool_5_length_microunits",
    "toolsweep_tool_5_snub_radius_microunits",
    "toolsweep_tool_5_xy_bin_refs",
    "toolsweep_tool_5_xy_bin_max_box_refs",
]

missing = [name for name in required if name not in m]
if missing:
    raise SystemExit(f"missing per-tool metrics: {missing}")

if m["toolsweep_tool_5_moves"] < 4000:
    raise SystemExit("dense V-bit fixture did not retain its short moves")
if m["toolsweep_tool_5_xy_bin_refs"] <= m["toolsweep_tool_1_xy_bin_refs"]:
    raise SystemExit("dense V-bit fixture did not dominate stored XY references")

tool_boxes = m["toolsweep_tool_1_aabb_boxes"] + m["toolsweep_tool_5_aabb_boxes"]
if tool_boxes != m["toolsweep_aabb_boxes"]:
    raise SystemExit(
        f"per-tool boxes {tool_boxes} do not match global {m['toolsweep_aabb_boxes']}"
    )

tool_refs = m["toolsweep_tool_1_xy_bin_refs"] + m["toolsweep_tool_5_xy_bin_refs"]
if tool_refs != m["toolsweep_xy_bin_refs"]:
    raise SystemExit(
        f"per-tool refs {tool_refs} do not match global {m['toolsweep_xy_bin_refs']}"
    )

print(
    "ToolSweep per-tool metrics smoke passed: "
    f"boxes={tool_boxes}, refs={tool_refs}, "
    f"vbit_moves={m['toolsweep_tool_5_moves']}"
)
PY
