#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

camsim="${1:-./camsim}"
tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

cat > "$tmpdir/dexel.camotics" <<'JSON'
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 0.2,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 8,
      "diameter": 1
    },
    "5": {
      "units": "metric",
      "shape": "snubnose",
      "length": 10,
      "diameter": 3,
      "snub_diameter": 0.2
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {"min": [-4, -4, -1], "max": [4, 4, 0]}
  },
  "files": ["dexel.nc"]
}
JSON

cat > "$tmpdir/dexel.nc" <<'NC'
G21
G90
F120
M6 T1
G0 X-3 Y-2 Z1
G1 Z-0.45
G1 X3
G0 Z1
M6 T5
G0 X-2 Y1 Z1
G1 Z-0.65
G1 X2 Y2 Z-0.55
G0 Z1
M30
NC

"$camsim" --threads 2 --profile "$tmpdir/full.json" \
  "$tmpdir/dexel.camotics" "$tmpdir/full.stl"
"$camsim" --threads 2 --dexel --profile "$tmpdir/dexel.json" \
  "$tmpdir/dexel.camotics" "$tmpdir/dexel.stl"
"$camsim" --threads 2 --dexel \
  --dexel-grid-png "$tmpdir/dexel-height.png" \
  --profile "$tmpdir/dexel-height.json" \
  "$tmpdir/dexel.camotics" >"$tmpdir/dexel-height.log" 2>&1
"$camsim" --threads 2 --dexel --dexel-skip-topology-validation \
  --profile "$tmpdir/dexel-trusted.json" \
  "$tmpdir/dexel.camotics" "$tmpdir/dexel-trusted.stl"
cmp -i 80 "$tmpdir/dexel.stl" "$tmpdir/dexel-trusted.stl"
"$camsim" --threads 2 --dexel --safe-reduce \
  --profile "$tmpdir/dexel-safe.json" \
  "$tmpdir/dexel.camotics" "$tmpdir/dexel-safe.stl"

python3 scripts/perf/compare_stl_distance.py \
  "$tmpdir/full.stl" "$tmpdir/dexel.stl" \
  --hard-max-error 0.3 --p99-error 0.21 --max-samples 20000
python3 scripts/perf/compare_stl_distance.py \
  "$tmpdir/dexel.stl" "$tmpdir/dexel-safe.stl" \
  --hard-max-error 0.3 --p99-error 0.21 --max-samples 20000

python3 - "$tmpdir/dexel-height.png" "$tmpdir/dexel-height.json" \
  "$tmpdir/dexel-height.log" <<'PY'
import json
import re
import struct
import sys
import zlib

png_path, profile_path, log_path = sys.argv[1:]
data = open(png_path, "rb").read()
if data[:8] != b"\x89PNG\r\n\x1a\n":
    raise SystemExit("Dexel height map has an invalid PNG signature")

offset = 8
chunks = []
while offset < len(data):
    length = struct.unpack(">I", data[offset:offset + 4])[0]
    kind = data[offset + 4:offset + 8]
    payload = data[offset + 8:offset + 8 + length]
    expected_crc = struct.unpack(">I", data[offset + 8 + length:
                                             offset + 12 + length])[0]
    actual_crc = zlib.crc32(kind)
    actual_crc = zlib.crc32(payload, actual_crc) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        raise SystemExit(f"invalid {kind!r} PNG chunk CRC")
    chunks.append((kind, payload))
    offset += length + 12
    if kind == b"IEND":
        break

ihdr = next(payload for kind, payload in chunks if kind == b"IHDR")
width, height, depth, color, compression, filtering, interlace = \
    struct.unpack(">IIBBBBB", ihdr)
if depth != 8 or color != 0:
    raise SystemExit(
        f"height map is not 8-bit single-channel grayscale: {depth=}, {color=}"
    )
if compression or filtering or interlace:
    raise SystemExit("height map uses unsupported PNG encoding flags")

raw = zlib.decompress(b"".join(
    payload for kind, payload in chunks if kind == b"IDAT"
))
stride = width + 1
if len(raw) != stride * height:
    raise SystemExit("height-map PNG decompressed size is incorrect")
if any(raw[row * stride] != 0 for row in range(height)):
    raise SystemExit("height-map PNG did not use deterministic None filters")
pixels = b"".join(
    raw[row * stride + 1:(row + 1) * stride] for row in range(height)
)
if min(pixels) != 0 or max(pixels) != 255:
    raise SystemExit("height-map extrema are not black and white")

with open(profile_path, encoding="utf-8") as src:
    metrics = json.load(src)["metrics"]
if metrics.get("dexel_height_map_width") != width:
    raise SystemExit("profile PNG width does not match IHDR")
if metrics.get("dexel_height_map_height") != height:
    raise SystemExit("profile PNG height does not match IHDR")
if metrics.get("dexel_height_map_png_bytes") != len(data):
    raise SystemExit("profile PNG byte count does not match the file")

with open(log_path, encoding="utf-8") as src:
    log = src.read()
match = re.search(
    r"Dexel height-map PNG written:.* min_z=([-+0-9.eE]+) "
    r"max_z=([-+0-9.eE]+)",
    log,
)
if not match:
    raise SystemExit("height-map log is missing the exported Z range")
logged_min_z, logged_max_z = map(float, match.groups())
profile_min_z = metrics.get("dexel_height_map_min_z")
profile_max_z = metrics.get("dexel_height_map_max_z")
if profile_min_z is None or profile_max_z is None:
    raise SystemExit("profile is missing the Dexel height-map Z range")
if profile_min_z >= 0:
    raise SystemExit("profile lost the signed Dexel minimum Z")
if abs(profile_min_z - logged_min_z) > 1e-6:
    raise SystemExit("profile minimum Z does not match the height-map log")
if abs(profile_max_z - logged_max_z) > 1e-6:
    raise SystemExit("profile maximum Z does not match the height-map log")

print(
    f"Dexel height-map PNG passed: {width}x{height}, "
    f"grayscale=8-bit, z={profile_min_z:g}..{profile_max_z:g}, "
    f"bytes={len(data)}"
)
PY

if "$camsim" --dexel-grid-png "$tmpdir/invalid.png" \
  "$tmpdir/dexel.camotics" >"$tmpdir/invalid-height.log" 2>&1; then
  echo "--dexel-grid-png unexpectedly succeeded without --dexel" >&2
  exit 1
fi
grep -q -- '--dexel-grid-png requires --dexel' \
  "$tmpdir/invalid-height.log"

python3 - "$tmpdir/dexel.json" "$tmpdir/dexel-trusted.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as src:
    metrics = json.load(src)["metrics"]
with open(sys.argv[2], encoding="utf-8") as src:
    trusted = json.load(src)["metrics"]

if metrics.get("dexel_candidate_accepted") != 1:
    raise SystemExit("dexel fixture candidate was not accepted")
for metric in [
    "dexel_columns_allocated",
    "dexel_footprint_cells_considered",
    "dexel_profile_evaluations",
    "dexel_mesh_triangles",
]:
    if metrics.get(metric, 0) < 1:
        raise SystemExit(f"missing dexel work metric: {metric}")
for metric in [
    "dexel_topology_boundary_edges",
    "dexel_topology_nonmanifold_edges",
    "dexel_topology_misoriented_edges",
    "dexel_topology_degenerate_triangles",
    "dexel_topology_duplicate_triangles",
]:
    if metrics.get(metric) != 0:
        raise SystemExit(f"dexel topology failure: {metric}")
if trusted.get("dexel_topology_validation_skipped") != 1:
    raise SystemExit("trusted production path did not report skipped topology")

print(
    "Dexel simulation smoke passed; "
    f"columns={metrics['dexel_columns_allocated']}, "
    f"triangles={metrics['dexel_mesh_triangles']}"
)
PY

# A short cutter moving below an intact roof must reject dynamically and
# produce the ordinary full-MC output.
cat > "$tmpdir/side-entry.camotics" <<'JSON'
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 0.25,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 0.2,
      "diameter": 0.5
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {"min": [-2, -2, -1], "max": [2, 2, 0]}
  },
  "files": ["side-entry.nc"]
}
JSON
cat > "$tmpdir/side-entry.nc" <<'NC'
G21
G90
M6 T1
F120
G0 X-1.5 Y0 Z-0.7
G1 X1.5
M30
NC

"$camsim" --threads 2 "$tmpdir/side-entry.camotics" \
  "$tmpdir/side-full.stl"
"$camsim" --threads 2 --dexel --profile "$tmpdir/side-dexel.json" \
  "$tmpdir/side-entry.camotics" "$tmpdir/side-dexel.stl"
cmp -i 80 "$tmpdir/side-full.stl" "$tmpdir/side-dexel.stl"

python3 - "$tmpdir/side-dexel.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as src:
    metrics = json.load(src)["metrics"]
if metrics.get("dexel_fallback_multi_interval_update") != 1:
    raise SystemExit("below-roof side entry did not trigger full-MC fallback")
if metrics.get("dexel_multi_interval_violations") != 1:
    raise SystemExit("below-roof violation counter changed")
print("Dexel dynamic multi-interval fallback passed")
PY

# Through-bottom removal is detected but remains fail-closed until the
# continuous mesh builder owns conforming empty-column boundaries.
cat > "$tmpdir/through.nc" <<'NC'
G21
G90
M6 T1
F120
G0 X0 Y0 Z1
G1 Z-1.2
G0 Z1
M30
NC
sed 's/side-entry.nc/through.nc/' "$tmpdir/side-entry.camotics" \
  > "$tmpdir/through.camotics"
"$camsim" --threads 2 "$tmpdir/through.camotics" "$tmpdir/through-full.stl"
"$camsim" --threads 2 --dexel --profile "$tmpdir/through-dexel.json" \
  "$tmpdir/through.camotics" "$tmpdir/through-dexel.stl"
cmp -i 80 "$tmpdir/through-full.stl" "$tmpdir/through-dexel.stl"
python3 - "$tmpdir/through-dexel.json" <<'PY'
import json
import sys
with open(sys.argv[1], encoding="utf-8") as src:
    metrics = json.load(src)["metrics"]
if metrics.get("dexel_fallback_empty_column_unsupported") != 1:
    raise SystemExit("through-bottom case did not fail closed")
print("Dexel through-bottom fallback passed")
PY
