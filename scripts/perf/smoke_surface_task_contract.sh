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
  local shape="$2"
  cat > "$tmpdir/$name.camotics" <<JSON
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 0.25,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "$shape",
      "length": 8,
      "diameter": 1
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {"min": [-3, -3, -1], "max": [3, 3, 0]}
  },
  "files": ["$name.nc"]
}
JSON

  cat > "$tmpdir/$name.nc" <<'NC'
G21
G90
M6 T1
F120
G0 X-2 Y-2 Z1
G1 Z-0.4
G1 X2 Y-2
G1 X2 Y2
G1 X-2 Y2
G0 Z1
M30
NC
}

write_project eligible cylindrical
write_project unsupported ballnose

full_args=(--threads 2 --toolsweep-xy-bins 64 --toolsweep-stock-bounds)

"$camsim" "${full_args[@]}" --profile "$tmpdir/full.json" \
  "$tmpdir/eligible.camotics" "$tmpdir/full.stl"
"$camsim" --threads 2 --dexel --profile "$tmpdir/auto.json" \
  "$tmpdir/eligible.camotics" "$tmpdir/auto.stl"

"$camsim" "${full_args[@]}" "$tmpdir/unsupported.camotics" \
  "$tmpdir/unsupported-full.stl"
"$camsim" "${full_args[@]}" --dexel --profile "$tmpdir/unsupported.json" \
  "$tmpdir/unsupported.camotics" "$tmpdir/unsupported-auto.stl"
cmp -i 80 "$tmpdir/unsupported-full.stl" "$tmpdir/unsupported-auto.stl"

"$camsim" "${full_args[@]}" --time 0.5 "$tmpdir/eligible.camotics" \
  "$tmpdir/partial-full.stl"
"$camsim" "${full_args[@]}" --dexel --time 0.5 \
  --profile "$tmpdir/partial.json" "$tmpdir/eligible.camotics" \
  "$tmpdir/partial-auto.stl"
cmp -i 80 "$tmpdir/partial-full.stl" "$tmpdir/partial-auto.stl"

for phase in eligibility preparing rasterizing building validating; do
  "$camsim" --threads 2 --dexel --surface-task-cancel-phase "$phase" \
    --profile "$tmpdir/cancel-$phase.json" "$tmpdir/eligible.camotics"
done

python3 - "$tmpdir" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])

def metrics(name):
    with (root / f"{name}.json").open(encoding="utf-8") as src:
        return json.load(src)["metrics"]

full = metrics("full")
auto = metrics("auto")
unsupported = metrics("unsupported")
partial = metrics("partial")

if full.get("simulation_backend_requested_auto_dexel", 0) != 0:
    raise SystemExit("full-MC policy unexpectedly requested Auto Dexel")
if full.get("simulation_backend_dexel_attempted", 0) != 0:
    raise SystemExit("full-MC policy unexpectedly attempted Dexel")
if full.get("simulation_backend_selected_full_mc") != 1:
    raise SystemExit("full-MC policy did not report the selected backend")

if auto.get("simulation_backend_requested_auto_dexel") != 1:
    raise SystemExit("Auto policy was not reported")
if auto.get("simulation_backend_dexel_attempted") != 1:
    raise SystemExit("eligible Auto input did not attempt Dexel")
if auto.get("simulation_backend_selected_dexel") != 1:
    raise SystemExit("eligible Auto input did not select Dexel")
if auto.get("dexel_candidate_accepted") != 1:
    raise SystemExit("eligible Dexel candidate was not accepted")

if unsupported.get("simulation_backend_selected_full_mc") != 1:
    raise SystemExit("unsupported tool did not select full MC")
if unsupported.get("dexel_fallback_unsupported_tool") != 1:
    raise SystemExit("unsupported tool fallback reason was not reported")
if partial.get("simulation_backend_selected_full_mc") != 1:
    raise SystemExit("partial time did not select full MC")
if partial.get("dexel_fallback_partial_time") != 1:
    raise SystemExit("partial-time fallback reason was not reported")

for phase in ("eligibility", "preparing", "rasterizing", "building",
              "validating"):
    cancelled = metrics(f"cancel-{phase}")
    if cancelled.get("surface_task_cancelled") != 1:
        raise SystemExit(f"{phase} phase did not cancel")
    if cancelled.get("surface_task_cancelled_without_surface") != 1:
        raise SystemExit(f"{phase} cancellation published a surface")
    if cancelled.get("surface_task_cancelled_without_fallback") != 1:
        raise SystemExit(f"{phase} cancellation ran fallback")
    if cancelled.get("simulation_backend_selected_full_mc", 0):
        raise SystemExit(f"{phase} cancellation selected full MC")

print("Shared SurfaceTask backend and cancellation contract passed")
PY

# Retained-roof (multi-interval) and empty-column candidate rejection remain
# covered by the dedicated dexel fixture suite.
bash scripts/perf/smoke_dexel_simulation.sh "$camsim"
