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

  cat > "$tmpdir/$name.camotics" <<JSON
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 0.25,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 3,
      "diameter": 0.6
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {
      "min": [-12, -12, -4],
      "max": [12, 12, 0]
    }
  },
  "files": ["$name.nc"]
}
JSON
}

write_coin_like_gcode() {
  local name="$1"
  local z="$2"

  cat > "$tmpdir/$name.nc" <<NC
G21
F120
M3 S1000
M6 T1
G0 X-8 Y-8 Z1
G1 Z$z
G1 X8 Y-8 Z$z
G1 X8 Y8 Z$z
G1 X-8 Y8 Z$z
G1 X-8 Y-8 Z$z
G0 Z1
G0 X-6 Y0 Z1
G1 Z$z
G1 X6 Y0 Z$z
G0 Z1
G0 X0 Y-6 Z1
G1 Z$z
G1 X0 Y6 Z$z
G0 Z1
NC
}

run_case() {
  local name="$1"
  local expect_expansion="$2"
  local dir="$tmpdir/run-$name"
  mkdir -p "$dir"

  /usr/bin/time -v -o "$dir/base.time" \
    "$camsim" --threads 1 --profile "$dir/base.json" \
    "$tmpdir/$name.camotics" "$dir/base.stl"
  /usr/bin/time -v -o "$dir/adaptive.time" \
    "$camsim" --threads 1 --profile "$dir/adaptive.json" \
    --adaptive-z-slabs --adaptive-z-initial-depth 0.8 \
    --adaptive-z-slab-height 0.8 --adaptive-z-margin 0.2 \
    "$tmpdir/$name.camotics" "$dir/adaptive.stl"
  /usr/bin/time -v -o "$dir/render.time" \
    "$camsim" --threads 1 --profile "$dir/render.json" \
    --adaptive-z-slabs --adaptive-z-render --adaptive-z-initial-depth 0.8 \
    --adaptive-z-slab-height 0.8 --adaptive-z-margin 0.2 \
    "$tmpdir/$name.camotics" "$dir/render.stl"

  python3 scripts/perf/compare_stl_geometry.py \
    "$dir/base.stl" "$dir/adaptive.stl"
  python3 scripts/perf/compare_stl_distance.py \
    "$dir/base.stl" "$dir/render.stl" \
    --hard-max-error 0.35 --p99-error 0.26 --max-samples 20000

  python3 - "$name" "$expect_expansion" "$dir/base.json" \
    "$dir/adaptive.json" "$dir/render.json" "$dir/base.time" \
    "$dir/render.time" <<'PY'
import json
import re
import sys

(
    name,
    expect_expansion,
    base_path,
    adaptive_path,
    render_path,
    base_time_path,
    render_time_path,
) = sys.argv[1:8]
expect_expansion = int(expect_expansion)

with open(base_path, encoding="utf-8") as f:
    base = json.load(f)
with open(adaptive_path, encoding="utf-8") as f:
    adaptive = json.load(f)
with open(render_path, encoding="utf-8") as f:
    render = json.load(f)

def read_user_time(path):
    with open(path, encoding="utf-8") as f:
        for line in f:
            if "User time" in line:
                return float(line.rsplit(":", 1)[1])
    raise SystemExit(f"{name}: missing user time in {path}")

def read_max_rss(path):
    with open(path, encoding="utf-8") as f:
        for line in f:
            if "Maximum resident set size" in line:
                return int(line.rsplit(":", 1)[1])
    raise SystemExit(f"{name}: missing max RSS in {path}")

base_metrics = base.get("metrics", {})
metrics = adaptive.get("metrics", {})
render_metrics = render.get("metrics", {})

if metrics.get("surface_triangles") != base_metrics.get("surface_triangles"):
    raise SystemExit(f"{name}: adaptive metric mode changed triangle count")

full = metrics.get("adaptive_z_full_grid_cells_est")
active = metrics.get("adaptive_z_active_grid_cells_est")
saved = metrics.get("adaptive_z_estimated_saved_cells")
expanded = metrics.get("adaptive_z_requires_expansion")

if not full or not active or saved is None:
    raise SystemExit(f"{name}: missing adaptive Z cell estimates")
if active >= full:
    raise SystemExit(f"{name}: deep-stock fixture did not reduce active cells")
if saved != full - active:
    raise SystemExit(f"{name}: saved cells do not equal full-active")
if expanded != expect_expansion:
    raise SystemExit(
        f"{name}: expected expansion {expect_expansion}, got {expanded}"
    )

initial_depth = metrics["adaptive_z_initial_depth_microunits"]
active_depth = metrics["adaptive_z_active_depth_microunits"]
if expect_expansion and active_depth <= initial_depth:
    raise SystemExit(f"{name}: expansion did not increase active depth")

if render_metrics.get("adaptive_z_render_enabled") != 1:
    raise SystemExit(f"{name}: adaptive render did not enable active grid")
if render_metrics.get("adaptive_z_reconstructed_lower_triangles") != 10:
    raise SystemExit(f"{name}: lower stock reconstruction triangle mismatch")
if render_metrics.get("adaptive_z_active_grid_cells_est") != active:
    raise SystemExit(f"{name}: render active cell estimate changed")
if render_metrics.get("surface_triangles", 0) >= base_metrics.get(
    "surface_triangles", 0
):
    raise SystemExit(f"{name}: adaptive render did not reduce triangle count")

base_user = read_user_time(base_time_path)
render_user = read_user_time(render_time_path)
base_rss = read_max_rss(base_time_path)
render_rss = read_max_rss(render_time_path)
time_delta = (render_user - base_user) / base_user * 100
rss_delta = (render_rss - base_rss) / base_rss * 100

if render_user > base_user * 1.05:
    raise SystemExit(
        f"{name}: adaptive render user CPU regressed: "
        f"{base_user:.2f}s -> {render_user:.2f}s"
    )
if render_rss > base_rss * 1.10:
    raise SystemExit(
        f"{name}: adaptive render peak RSS regressed: "
        f"{base_rss}KB -> {render_rss}KB"
    )

print(
    f"{name}: full={full} active={active} saved={saved} "
    f"expansion={expanded} render_triangles="
    f"{render_metrics.get('surface_triangles')} "
    f"base_user_s={base_user:.2f} render_user_s={render_user:.2f} "
    f"delta_pct={time_delta:.1f} base_rss_kb={base_rss} "
    f"render_rss_kb={render_rss} rss_delta_pct={rss_delta:.1f}"
)
PY
}

write_coin_like_gcode shallow_coin -0.35
write_project shallow_coin

write_coin_like_gcode scaled_z_coin -1.7
write_project scaled_z_coin

run_case shallow_coin 0
run_case scaled_z_coin 1

echo "adaptive Z deep-stock stress smoke passed"
