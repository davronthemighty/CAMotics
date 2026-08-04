#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

camsim="${1:-./camsim}"
tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

cat > "$tmpdir/shallow.nc" <<'NC'
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
NC

write_project() {
  local name="$1"
  local min_z="$2"

  cat > "$tmpdir/$name.camotics" <<JSON
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 0.25,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 8,
      "diameter": 0.8
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {
      "min": [-10, -10, $min_z],
      "max": [10, 10, 0]
    }
  },
  "files": ["shallow.nc"]
}
JSON
}

write_project trimmed -1
write_project untrimmed -5

"$camsim" --threads 1 --no-export --surface-stats \
  --profile "$tmpdir/trimmed.json" "$tmpdir/trimmed.camotics"
"$camsim" --threads 1 --no-export --surface-stats \
  --profile "$tmpdir/untrimmed.json" "$tmpdir/untrimmed.camotics"

"$camsim" --perf-warnings-only --profile "$tmpdir/warnings.json" \
  "$tmpdir/untrimmed.camotics" >"$tmpdir/warnings.log" 2>&1
grep -Fq "trimmed stock may reduce memory and output size" \
  "$tmpdir/warnings.log"

python3 - "$tmpdir/trimmed.json" "$tmpdir/untrimmed.json" \
  "$tmpdir/warnings.json" <<'PY'
import json
import sys

trimmed_path, untrimmed_path, warnings_path = sys.argv[1:4]
with open(trimmed_path, encoding="utf-8") as f:
    trimmed = json.load(f)
with open(untrimmed_path, encoding="utf-8") as f:
    untrimmed = json.load(f)
with open(warnings_path, encoding="utf-8") as f:
    warnings = json.load(f)

tm = trimmed.get("metrics", {})
um = untrimmed.get("metrics", {})
tc = trimmed.get("counters", {})
uc = untrimmed.get("counters", {})

def require_reduction(name, trimmed_value, untrimmed_value, max_ratio):
    if not trimmed_value or not untrimmed_value:
        raise SystemExit(f"missing {name}: {trimmed_value}, {untrimmed_value}")
    ratio = trimmed_value / untrimmed_value
    if ratio >= max_ratio:
        raise SystemExit(
            f"{name} did not shrink enough: "
            f"{trimmed_value} / {untrimmed_value} = {ratio:.3f}"
        )

require_reduction(
    "render_tree_cells",
    tm.get("render_tree_cells"),
    um.get("render_tree_cells"),
    0.40,
)
require_reduction(
    "surface_triangles",
    tm.get("surface_triangles"),
    um.get("surface_triangles"),
    0.85,
)
require_reduction(
    "estimated_binary_stl_bytes",
    tm.get("estimated_binary_stl_bytes"),
    um.get("estimated_binary_stl_bytes"),
    0.85,
)

trimmed_depth = tc.get("tool_sweep_depth_calls") or tc.get(
    "toolsweep_depth_calls"
)
untrimmed_depth = uc.get("tool_sweep_depth_calls") or uc.get(
    "toolsweep_depth_calls"
)
require_reduction(
    "toolsweep_depth_calls",
    trimmed_depth,
    untrimmed_depth,
    0.65,
)

warning_metrics = warnings.get("metrics", {})
if "surface_triangles" in warning_metrics:
    raise SystemExit("perf-warnings-only unexpectedly simulated the surface")

print(
    "trimmed stock workflow smoke passed: "
    f"cells {um['render_tree_cells']}->{tm['render_tree_cells']}, "
    f"triangles {um['surface_triangles']}->{tm['surface_triangles']}"
)
PY
