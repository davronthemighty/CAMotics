#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
camsim="${1:-$root/camsim}"
out="${2:-$root/build/benchmarks/40mm-comparison}"
threads="${THREADS:-$(nproc)}"
fixture="$out/fixture"

mkdir -p "$out"
python3 "$root/scripts/benchmarks/generate_40mm_stress_fixture.py" \
  --output-dir "$fixture" | tee "$out/fixture.log"
project="$fixture/camotics-fast-40mm-stress.camotics"

run_case() {
  local name="$1"
  shift
  local dir="$out/$name"
  mkdir -p "$dir"
  /usr/bin/time -o "$dir/time.txt" \
    -f 'wall_s=%e\nuser_s=%U\nsystem_s=%S\ncpu_percent=%P\nmax_rss_kib=%M' \
    "$camsim" --binary --threads "$threads" --surface-stats \
    --profile "$dir/profile.json" "$@" "$project" "$dir/result.stl" \
    >"$dir/stdout.log" 2>"$dir/stderr.log"
  python3 - "$dir/profile.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    resolution = json.load(stream).get("metrics", {}).get(
        "simulation_resolution_microunits"
    )
if resolution != 25000:
    raise SystemExit(
        f"expected 0.025 mm simulation resolution, got {resolution!r} microunits"
    )
PY
  sha256sum "$dir/result.stl" >"$dir/SHA256SUM"
}

run_case full_mc
run_case full_mc_safe_reduce --safe-reduce --safe-reduce-report
run_case dexel --dexel
run_case dexel_safe_reduce --dexel --safe-reduce --safe-reduce-report

python3 "$root/scripts/perf/compare_stl_distance_streaming.py" \
  --hard-max-error 0.15 --p99-error 0.10 --max-triangles 20000 \
  --json "$out/full_mc/result.stl" "$out/dexel/result.stl" \
  >"$out/full-vs-dexel-distance.json"
python3 "$root/scripts/perf/compare_stl_distance_streaming.py" \
  --hard-max-error 0.001 --p99-error 0.0002 --max-triangles 20000 \
  --json "$out/full_mc/result.stl" "$out/full_mc_safe_reduce/result.stl" \
  >"$out/full-reduction-distance.json"
python3 "$root/scripts/perf/compare_stl_distance_streaming.py" \
  --hard-max-error 0.001 --p99-error 0.0002 --max-triangles 20000 \
  --json "$out/dexel/result.stl" "$out/dexel_safe_reduce/result.stl" \
  >"$out/dexel-reduction-distance.json"

python3 "$root/scripts/benchmarks/summarize_40mm_benchmark.py" "$out"

cat "$out/results.tsv"
