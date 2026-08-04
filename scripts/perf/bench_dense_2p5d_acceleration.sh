#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

if [[ $# -lt 4 ]]; then
  echo "usage: $0 <project> <resolution> <out-dir> <threads> [modes]" >&2
  echo "modes: legacy optimized sparse dexel dexel-production dexel-safe" >&2
  exit 2
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

project="$1"
resolution="$2"
out_dir="$3"
threads="$4"
modes="${5:-optimized dexel dexel-production}"
mkdir -p "$out_dir"

run_mode() {
  local name="$1"
  shift
  local dir="$out_dir/$name"
  mkdir -p "$dir"
  /usr/bin/time -v ./camsim \
    --resolution "$resolution" --threads "$threads" --surface-stats \
    --profile "$dir/profile.json" "$@" "$project" "$dir/output.stl" \
    > "$dir/run.log" 2> "$dir/time.log"
}

for mode in $modes; do
  case "$mode" in
    legacy) run_mode legacy ;;
    optimized)
      run_mode optimized --toolsweep-xy-bins 64 --toolsweep-stock-bounds ;;
    sparse)
      run_mode sparse --sparse-toolpath ;;
    dexel)
      run_mode dexel --dexel ;;
    dexel-production)
      run_mode dexel-production --dexel --dexel-skip-topology-validation ;;
    dexel-safe)
      run_mode dexel-safe --dexel --safe-reduce ;;
    *) echo "unknown benchmark mode: $mode" >&2; exit 2 ;;
  esac
done

python3 - "$project" "$out_dir" $modes <<'PY'
import json
import pathlib
import re
import sys

project = sys.argv[1]
root = pathlib.Path(sys.argv[2])
rows = []
for mode in sys.argv[3:]:
    directory = root / mode
    profile_path = directory / "profile.json"
    time_path = directory / "time.log"
    output_path = directory / "output.stl"
    with profile_path.open(encoding="utf-8") as src:
        profile = json.load(src)
    time_text = time_path.read_text(encoding="utf-8", errors="replace")

    def value(pattern, cast=float):
        match = re.search(pattern, time_text)
        return cast(match.group(1)) if match else None

    rows.append({
        "mode": mode,
        "user_seconds": value(r"User time \(seconds\): ([0-9.]+)"),
        "system_seconds": value(r"System time \(seconds\): ([0-9.]+)"),
        "elapsed": value(r"Elapsed .*: (\S+)", str),
        "max_rss_kib": value(r"Maximum resident set size \(kbytes\): (\d+)", int),
        "triangles": profile.get("metrics", {}).get("surface_triangles"),
        "bytes": output_path.stat().st_size,
        "profile": str(profile_path),
        "output": str(output_path),
    })

report = {"project": project, "runs": rows}
(root / "summary.json").write_text(
    json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
for row in rows:
    print(
        f"{row['mode']}: elapsed={row['elapsed']} user={row['user_seconds']} "
        f"rss={row['max_rss_kib']} triangles={row['triangles']} "
        f"bytes={row['bytes']}"
    )
PY
