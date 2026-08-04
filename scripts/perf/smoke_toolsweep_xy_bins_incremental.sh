#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

cbang_home="${CBANG_HOME:-cbang}"
read -r -a python_cflags <<< "$(python3-config --includes)"
read -r -a python_ldflags <<< "$(python3-config --embed --ldflags)"
read -r -a extra_cxxflags <<< "${CAMOTICS_SMOKE_CXXFLAGS:-}"
read -r -a extra_ldflags <<< "${CAMOTICS_SMOKE_LDFLAGS:-}"

tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

cat > "$tmpdir/incremental.cpp" <<'CPP'
#include <camotics/project/Project.h>
#include <camotics/sim/CutSim.h>
#include <camotics/sim/Simulation.h>
#include <camotics/sim/SimulationRun.h>
#include <camotics/Task.h>
#include <camotics/contour/Surface.h>
#include <camotics/render/RenderMode.h>

#include <cbang/SmartPointer.h>
#include <cbang/os/SystemUtilities.h>

#include <cstdlib>
#include <string>

using namespace CAMotics;
using namespace cb;
using namespace std;

int main(int argc, char **argv) {
  if (argc != 7) return 2;

  string projectFile = argv[1];
  string out1 = argv[2];
  string out2 = argv[3];
  double t1 = atof(argv[4]);
  double t2 = atof(argv[5]);
  unsigned xyBins = strtoul(argv[6], 0, 10);

  Project::Project project;
  project.load(projectFile);

  CutSim cutSim;
  SmartPointer<GCode::ToolPath> path = cutSim.computeToolPath(project);
  project.getWorkpiece().update(*path);
  Rectangle3D bounds = project.getWorkpiece().getBounds();

  Simulation sim(path, 0, 0, bounds, project.getResolution(), t1,
                 RenderMode(), 1, xyBins);
  SimulationRun run(sim);
  Task task;

  SmartPointer<Surface> first = run.compute(task);
  first->writeSTL(*SystemUtilities::oopen(out1), true, "incremental", "");

  run.setEndTime(t2);
  SmartPointer<Surface> second = run.compute(task);
  second->writeSTL(*SystemUtilities::oopen(out2), true, "incremental", "");

  return 0;
}
CPP

g++ -o "$tmpdir/incremental" \
  -faligned-new -std=c++17 -fsigned-char -Wno-deprecated-declarations \
  "${extra_cxxflags[@]}" \
  -O2 -DNDEBUG -D_REENTRANT -DHAVE_CBANG -DUSING_CBANG -DCAMOTICS_NO_TPL \
  -I"$cbang_home/src" -I"$cbang_home/include" \
  -I"$cbang_home/src/boost" -Isrc \
  "${python_cflags[@]}" -Ibuild \
  "$tmpdir/incremental.cpp" \
  -L"$cbang_home/lib" \
  -Wl,--start-group \
  build/libCAMoticsPy.a build/libCAMotics.a build/libDXF.a build/libSTL.a \
  build/libGCode.a \
  -lstdc++ -lutil -lm -ldl -lz -lutil -lcbang -lcbang-boost -lssl \
  -lcrypto -lre2 -levent -lexpat -llz4 -lbz2 -lz -lpthread -ldl -lm \
  "${python_ldflags[@]}" build/dxflib/libdxflib.a \
  "${extra_ldflags[@]}" \
  -Wl,--end-group

cat > "$tmpdir/incremental.nc" <<'NC'
G21
F60
M3 S1000
M6 T1
G0 X0 Y0 Z2
G1 Z-1
G1 X20 Y0 Z-1
G1 X40 Y40 Z-1
G1 X80 Y40 Z-1
G0 Z2
NC

cat > "$tmpdir/incremental.camotics" <<'JSON'
{
  "units": "metric",
  "resolution-mode": "manual",
  "resolution": 2.0,
  "tools": {
    "1": {
      "units": "metric",
      "shape": "cylindrical",
      "length": 10,
      "diameter": 1
    }
  },
  "workpiece": {
    "automatic": false,
    "margin": 0,
    "bounds": {
      "min": [-4, -4, -3],
      "max": [84, 44, 1]
    }
  },
  "files": ["incremental.nc"]
}
JSON

"$tmpdir/incremental" "$tmpdir/incremental.camotics" \
  "$tmpdir/default-t1.stl" "$tmpdir/default-t2.stl" 25 70 0
"$tmpdir/incremental" "$tmpdir/incremental.camotics" \
  "$tmpdir/xy-t1.stl" "$tmpdir/xy-t2.stl" 25 70 64

python3 scripts/perf/compare_stl_geometry.py \
  "$tmpdir/default-t1.stl" "$tmpdir/xy-t1.stl"
python3 scripts/perf/compare_stl_geometry.py \
  "$tmpdir/default-t2.stl" "$tmpdir/xy-t2.stl"

echo "ToolSweep XY-bin incremental smoke passed"
