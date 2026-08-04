#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

camsim="${1:-./camsim}"

tmp="${TMPDIR:-/tmp}/camotics-safe-reduce-contract-cxx-$$"
mkdir -p "$tmp"
trap 'rm -rf "$tmp"' EXIT

"$camsim" \
  --safe-reduce-contract-self-test \
  --profile "$tmp/profile.json" \
  >"$tmp/self-test.log" 2>&1

grep -q "Safe-reduce contract self-test passed" "$tmp/self-test.log"

python3 - "$tmp/profile.json" <<'PY'
import json
import sys

profile = json.load(open(sys.argv[1]))
metrics = profile.get("metrics", {})

if metrics.get("safe_reduce_contract_self_test_passed") != 1:
    raise SystemExit("C++ safe-reduce contract self-test pass metric missing")
PY

echo "C++ safe-reduce contract smoke passed"
