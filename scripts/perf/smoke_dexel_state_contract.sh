#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

helper="${1:-./camsim-dexel-state}"
"$helper" --threads 4 examples/slant_test/slant_test.camotics

echo "Persistent Dexel state smoke passed"
