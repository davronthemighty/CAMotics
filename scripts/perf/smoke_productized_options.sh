#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

camsim="${1:-./camsim}"

if [ ! -x "$camsim" ]; then
  echo "camsim executable not found: $camsim" >&2
  echo "Build it first, for example:" >&2
  echo "  scons platform=posix compiler=gnu cc=gcc cxx=g++ ar=ar ranlib=ranlib with_gui=0 with_tpl=0 strict=0 -j2 camsim" >&2
  exit 2
fi

python3 -m py_compile \
  scripts/perf/compare_stl_geometry.py \
  scripts/perf/compare_stl_distance.py \
  scripts/perf/mesh_reduction_contract_report.py

run_smoke() {
  local script="$1"
  shift
  echo "==> $script"
  bash "$script" "$@"
}

run_smoke scripts/perf/smoke_camsim_regression.sh "$camsim"
run_smoke scripts/perf/smoke_original_headless_tests.sh
run_smoke scripts/perf/smoke_partition_cross_thread.sh "$camsim"
run_smoke scripts/perf/smoke_perf_warnings.sh "$camsim"
run_smoke scripts/perf/smoke_reduce_export.sh "$camsim"
run_smoke scripts/perf/smoke_mesh_reduction_contract.sh
run_smoke scripts/perf/smoke_safe_reduce_contract_cxx.sh "$camsim"
run_smoke scripts/perf/smoke_safe_reduce_report.sh "$camsim"
run_smoke scripts/perf/smoke_safe_reduce_export.sh "$camsim"
echo "==> scripts/perf/smoke_safe_reduce_fixture_export.sh (trusted cat)"
CAMOTICS_SAFE_REDUCE_EXPORT_CASES=cat \
CAMOTICS_SAFE_REDUCE_HOLE_AWARE=1 \
CAMOTICS_SAFE_REDUCE_TRUST_PROVENANCE_NEIGHBORS=1 \
  bash scripts/perf/smoke_safe_reduce_fixture_export.sh "$camsim"
run_smoke scripts/perf/smoke_safe_reduce_provenance_parity.sh
run_smoke scripts/perf/smoke_safe_reduce_provenance_neighbors.sh "$camsim"
run_smoke scripts/perf/smoke_safe_reduce_trusted_cross_thread.sh "$camsim"
run_smoke scripts/perf/smoke_trimmed_stock_workflow.sh "$camsim"
run_smoke scripts/perf/smoke_adaptive_z_slab_metrics.sh "$camsim"
run_smoke scripts/perf/smoke_adaptive_z_deep_stock_stress.sh "$camsim"
run_smoke scripts/perf/smoke_adaptive_z_region_chunks.sh "$camsim"
run_smoke scripts/perf/smoke_toolsweep_xy_bins_edges.sh "$camsim"
run_smoke scripts/perf/smoke_toolsweep_xy_bins_incremental.sh

echo "Productized tiny-bit performance option smoke passed"
