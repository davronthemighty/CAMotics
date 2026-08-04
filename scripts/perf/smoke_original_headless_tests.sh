#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

if [ ! -x ./gcodetool ]; then
  echo "gcodetool executable not found." >&2
  echo "Build it first, for example:" >&2
  echo "  scons platform=posix compiler=gnu cc=gcc cxx=g++ ar=ar ranlib=ranlib with_gui=0 with_tpl=0 strict=0 -j2 gcodetool" >&2
  exit 2
fi

python3 -m py_compile tests/testHarness

python3 tests/testHarness --no-color -C tests run oCodeTests
python3 tests/testHarness --no-color -C tests run varRefTests
python3 tests/testHarness --no-color -C tests run offsetTests

echo "Original headless non-TPL regression smoke passed"
