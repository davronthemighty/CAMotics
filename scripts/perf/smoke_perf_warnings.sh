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
  local diameter="$3"
  local min_bounds="$4"
  local max_bounds="$5"

  cat > "$tmpdir/$name.camotics" <<JSON
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": $resolution,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 120,
      "diameter": $diameter
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {
      "min": [$min_bounds],
      "max": [$max_bounds]
    }
  },
  "files": ["$name.nc"]
}
JSON
}

write_cut() {
  local name="$1"
  local z="$2"

  cat > "$tmpdir/$name.nc" <<NC
G21
F120
M3 S1000
M6 T1
G0 X0 Y0 Z1
G1 Z$z
G1 X1 Y0 Z$z
G0 Z1
NC
}

write_dense_tiny_cut() {
  local name="$1"

  cat > "$tmpdir/$name.nc" <<NC
G21
F120
M3 S1000
M6 T1
G0 X0 Y0 Z1
G1 Z-0.2
NC

  for i in $(seq 1 10050); do
    if (( i % 2 )); then
      printf 'G1 X0.1 Y0 Z-0.2\n' >> "$tmpdir/$name.nc"
    else
      printf 'G1 X0 Y0 Z-0.2\n' >> "$tmpdir/$name.nc"
    fi
  done

  cat >> "$tmpdir/$name.nc" <<NC
G0 Z1
NC
}

run_warning_case() {
  local name="$1"
  shift
  local log="$tmpdir/$name.log"
  local profile="$tmpdir/$name.json"

  "$camsim" --perf-warnings-only --profile "$profile" \
    "$tmpdir/$name.camotics" >"$log" 2>&1

  for expected in "$@"; do
    if ! grep -Fq "$expected" "$log"; then
      echo "$name: missing warning substring: $expected" >&2
      cat "$log" >&2
      exit 1
    fi
  done

  python3 - "$name" "$profile" <<'PY'
import json
import sys

name, path = sys.argv[1:3]
with open(path, encoding="utf-8") as f:
    profile = json.load(f)

metrics = profile.get("metrics", {})
if "surface_triangles" in metrics:
    raise SystemExit(f"{name}: perf-warnings-only ran surface simulation")

timers = profile.get("timers", {})
if "surface_compute" in timers:
    raise SystemExit(f"{name}: perf-warnings-only recorded surface_compute")
PY
}

run_advice_case() {
  local name="$1"
  shift
  local log="$tmpdir/$name-advice.log"
  local profile="$tmpdir/$name-advice.json"

  "$camsim" --perf-advice --adaptive-z-region-bins 64 \
    --profile "$profile" "$tmpdir/$name.camotics" >"$log" 2>&1

  for expected in "$@"; do
    if ! grep -Fq "$expected" "$log"; then
      echo "$name advice: missing warning substring: $expected" >&2
      cat "$log" >&2
      exit 1
    fi
  done

  python3 - "$name" "$profile" <<'PY'
import json
import sys

name, path = sys.argv[1:3]
with open(path, encoding="utf-8") as f:
    profile = json.load(f)

metrics = profile.get("metrics", {})
phases = {phase.get("name") for phase in profile.get("phases", [])}

if "surface_triangles" in metrics or "surface_compute" in phases:
    raise SystemExit(f"{name}: perf-advice ran surface simulation")

required = [
    "perf_advice_grid_cells",
    "perf_advice_feed_moves",
    "perf_advice_region_active_cells_est",
    "perf_advice_region_best_bins",
    "perf_advice_region_best_initial_depth_microunits",
    "perf_advice_region_candidate_bins",
    "perf_advice_region_candidate_depths",
    "perf_advice_region_render_cells_est",
    "perf_advice_region_runtime_fit",
]
missing = [key for key in required if key not in metrics]
if missing:
    raise SystemExit(f"{name}: missing perf-advice metrics: {missing}")

if metrics["perf_advice_grid_cells"] <= 0:
    raise SystemExit(f"{name}: missing nonzero perf-advice grid estimate")

if metrics["perf_advice_region_candidate_bins"] <= 1:
    raise SystemExit(f"{name}: perf-advice did not sweep regional bin counts")

if metrics["perf_advice_region_best_bins"] < 2:
    raise SystemExit(f"{name}: invalid best regional bin count")

if metrics["perf_advice_region_best_initial_depth_microunits"] <= 0:
    raise SystemExit(f"{name}: invalid best regional initial depth")
PY
}

write_cut tiny_tool -0.2
write_project tiny_tool 0.25 0.5 "-2, -2, -1" "2, 2, 0"

write_dense_tiny_cut dense_tiny_tool
write_project dense_tiny_tool 0.25 0.5 "-2, -2, -1" "2, 2, 0"

write_cut deep_stock -0.5
write_project deep_stock 0.25 2.0 "-2, -2, -5" "2, 2, 0"

write_cut large_envelope -99
write_project large_envelope 0.5 10.0 "-150, -150, -100" "150, 150, 0"

run_warning_case tiny_tool \
  "Performance warning: resolution" \
  "Tiny-bit detail may be under-resolved" \
  "Performance recommendation: try --toolsweep-xy-bins 64"
run_warning_case deep_stock \
  "Performance warning: stock extends" \
  "trimmed stock may reduce memory and output size" \
  "Performance recommendation: run --adaptive-z-slabs" \
  "Performance recommendation: when full physical stock is not required"
run_warning_case large_envelope \
  "Performance warning: estimated grid is" \
  "Large envelopes at fine resolution" \
  "Performance recommendation: if STL size is the bottleneck"

run_advice_case dense_tiny_tool \
  "Performance advice: grid=" \
  "Performance advice: --toolsweep-xy-bins 64 is a good candidate" \
  "Performance advice: validate --toolsweep-xy-bins against"

run_advice_case large_envelope \
  "Performance advice: grid=" \
  "Performance advice: prefer --binary" \
  "Performance advice: --reduce is worth testing"

python3 - "$tmpdir/tiny_tool.json" "$tmpdir/deep_stock.json" \
  "$tmpdir/large_envelope.json" "$tmpdir/dense_tiny_tool-advice.json" \
  "$tmpdir/large_envelope-advice.json" <<'PY'
import json
import sys

tiny_path, deep_path, large_path, dense_advice_path, large_advice_path = \
    sys.argv[1:6]

def metrics(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f).get("metrics", {})

tiny = metrics(tiny_path)
deep = metrics(deep_path)
large = metrics(large_path)
dense_advice = metrics(dense_advice_path)
large_advice = metrics(large_advice_path)

checks = [
    (tiny.get("perf_recommend_toolsweep_xy_bins") == 64,
     "tiny tool did not recommend ToolSweep XY64"),
    (deep.get("perf_recommend_global_adaptive_z") == 1,
     "deep stock did not recommend adaptive Z measurement"),
    (deep.get("perf_recommend_trimmed_stock") == 1,
     "deep stock did not recommend explicit trimmed-stock workflow"),
    (large.get("perf_recommend_reduce") == 1,
     "large envelope did not recommend reduce for STL-size pressure"),
    (large.get("perf_warning_grid_cells", 0) > 50000000,
     "large envelope grid-cell metric missing or too small"),
    (dense_advice.get("perf_advice_recommend_toolsweep_xy_bins") == 64,
     "dense tiny advice did not recommend ToolSweep XY64"),
    (dense_advice.get("perf_advice_candidate_toolpath_spatial_indexing") == 1,
     "dense tiny advice did not mark spatial indexing as a candidate"),
    (large_advice.get("perf_advice_recommend_binary") == 1,
     "large advice did not recommend binary STL"),
    (large_advice.get("perf_advice_recommend_reduce") == 1,
     "large advice did not recommend reduce"),
    (large_advice.get("perf_advice_region_candidate_depths", 0) > 1,
     "large advice did not sweep regional initial depths"),
]

for ok, message in checks:
    if not ok:
        raise SystemExit(message)
PY

echo "perf warnings smoke passed"
