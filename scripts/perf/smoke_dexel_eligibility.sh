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
}

write_project eligible cylindrical
cat > "$tmpdir/eligible.nc" <<'NC'
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

write_project unsupported ballnose
cp "$tmpdir/eligible.nc" "$tmpdir/unsupported.nc"

write_project rotary cylindrical
cat > "$tmpdir/rotary.nc" <<'NC'
G21
G90
M6 T1
F120
G0 X-2 Y0 Z1
G1 Z-0.4
G1 X2 A10
G0 Z1
M30
NC

classify() {
  local name="$1"
  shift
  "$camsim" --dexel-eligibility-only --profile "$tmpdir/$name.json" \
    "$@" "$tmpdir/${name%%-*}.camotics"
}

classify eligible
classify eligible-partial --time 0.5
classify unsupported
classify rotary

"$camsim" --threads 2 "$tmpdir/unsupported.camotics" "$tmpdir/legacy.stl"
"$camsim" --threads 2 --dexel --profile "$tmpdir/fallback.json" \
  "$tmpdir/unsupported.camotics" "$tmpdir/fallback.stl"
cmp -i 80 "$tmpdir/legacy.stl" "$tmpdir/fallback.stl"
python3 scripts/perf/compare_stl_geometry.py \
  "$tmpdir/legacy.stl" "$tmpdir/fallback.stl"

python3 - "$tmpdir" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])

def metrics(name):
    with (root / f"{name}.json").open(encoding="utf-8") as src:
        return json.load(src)["metrics"]

eligible = metrics("eligible")
partial = metrics("eligible-partial")
unsupported = metrics("unsupported")
rotary = metrics("rotary")
fallback = metrics("fallback")

if eligible.get("dexel_eligibility_accepted") != 1:
    raise SystemExit("eligible project was rejected")
if partial.get("dexel_rejection_partial_time") != 1:
    raise SystemExit("partial-time project did not report its reason")
if unsupported.get("dexel_rejection_unsupported_tool") != 1:
    raise SystemExit("unsupported tool did not report its reason")
if rotary.get("dexel_rejection_rotary_or_aux_axes") != 1:
    raise SystemExit("rotary project did not report its reason")
if fallback.get("dexel_fallback_unsupported_tool") != 1:
    raise SystemExit("unsupported integrated run did not fail closed")
if fallback.get("dexel_eligibility_accepted") != 0:
    raise SystemExit("fallback run was unexpectedly eligible")

print("Dexel eligibility and full-MC fallback smoke passed")
PY
