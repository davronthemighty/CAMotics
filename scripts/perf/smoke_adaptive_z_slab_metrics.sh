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
G0 X0 Y0 Z2
G1 Z-0.2
G1 X20 Y0 Z-0.2
G1 X20 Y20 Z-0.2
G1 X0 Y20 Z-0.2
G1 X0 Y0 Z-0.2
G0 Z2
NC

cat > "$tmpdir/shallow.camotics" <<JSON
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 1.0,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 2,
      "diameter": 1
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {
      "min": [-2, -2, -20],
      "max": [22, 22, 2]
    }
  },
  "files": ["shallow.nc"]
}
JSON

"$camsim" --threads 1 --profile "$tmpdir/base.json" \
  "$tmpdir/shallow.camotics" "$tmpdir/base.stl"
"$camsim" --threads 1 --profile "$tmpdir/adaptive.json" \
  --adaptive-z-slabs --adaptive-z-initial-depth 4 \
  --adaptive-z-slab-height 4 --adaptive-z-margin 1 \
  "$tmpdir/shallow.camotics" "$tmpdir/adaptive.stl"

python3 scripts/perf/compare_stl_geometry.py \
  "$tmpdir/base.stl" "$tmpdir/adaptive.stl"

python3 - "$tmpdir/base.json" "$tmpdir/adaptive.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as f:
    base = json.load(f)
with open(sys.argv[2], encoding="utf-8") as f:
    adaptive = json.load(f)

base_metrics = base.get("metrics", {})
adaptive_metrics = adaptive.get("metrics", {})

if "adaptive_z_enabled" in base_metrics:
    raise SystemExit("base profile unexpectedly contains adaptive Z metrics")

required = [
    "adaptive_z_enabled",
    "adaptive_z_stock_height_microunits",
    "adaptive_z_initial_depth_microunits",
    "adaptive_z_slab_height_microunits",
    "adaptive_z_margin_microunits",
    "adaptive_z_swept_depth_microunits",
    "adaptive_z_required_depth_microunits",
    "adaptive_z_active_depth_microunits",
    "adaptive_z_total_slabs",
    "adaptive_z_required_slabs",
    "adaptive_z_requires_expansion",
    "adaptive_z_full_grid_cells_est",
    "adaptive_z_active_grid_cells_est",
    "adaptive_z_estimated_saved_cells",
]
missing = [name for name in required if name not in adaptive_metrics]
if missing:
    raise SystemExit("adaptive profile missing metrics: " + ", ".join(missing))

if adaptive_metrics["adaptive_z_enabled"] != 1:
    raise SystemExit("adaptive_z_enabled was not set")

if adaptive_metrics["surface_triangles"] != base_metrics["surface_triangles"]:
    raise SystemExit("adaptive metrics changed triangle count")

full_cells = adaptive_metrics["adaptive_z_full_grid_cells_est"]
active_cells = adaptive_metrics["adaptive_z_active_grid_cells_est"]
saved_cells = adaptive_metrics["adaptive_z_estimated_saved_cells"]
if active_cells > full_cells:
    raise SystemExit("active grid estimate is larger than full grid estimate")
if saved_cells != full_cells - active_cells:
    raise SystemExit("saved cell estimate does not match full-active")

print("adaptive Z-slab metrics smoke passed")
PY
