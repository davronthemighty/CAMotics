#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-or-later
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$repo_root"

camotics="${1:-./camotics}"
camsim="${2:-./camsim}"
project="examples/slant_test/slant_test.camotics"
tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

gui_prefix=()
if [[ -z "${DISPLAY:-}" ]]; then
  gui_prefix=(xvfb-run -a)
fi

run_gui() {
  local config="$1"
  local log="$2"
  shift 2
  XDG_CONFIG_HOME="$config" timeout 60s "${gui_prefix[@]}" \
    "$camotics" --threads 4 --auto-close-after-simulation "$@" \
    "$project" >"$log" 2>&1
}

# A fresh profile uses safe Auto so normal GUI playback receives the retained
# Dexel path when eligible.  Auto still uses Full MC for every rejected case.
run_gui "$tmpdir/default-config" "$tmpdir/default.log" \
  --test-view-controls --test-dexel-grid-window
test "$(grep -c 'Simulation backend: dexel' "$tmpdir/default.log")" -eq 2
test "$(grep -c 'Dexel eligibility accepted' "$tmpdir/default.log")" -eq 2
grep -q 'GUI tool table: project_tools=1 path_tools=1 rows=1' \
  "$tmpdir/default.log"
view_controls_re='GUI view controls test: table_rows=1 table_inserted=1 '
view_controls_re+='table_resets=[1-9][0-9]* '
view_controls_re+='table_rect=[1-9][0-9]*x[1-9][0-9]* '
view_controls_re+='dock=pass space=pass pan=pass zoom=pass orbit=pass'
grep -Eq "$view_controls_re" "$tmpdir/default.log"
height_window_re='GUI Dexel height-map window test: '
height_window_re+='width=[1-9][0-9]* height=[1-9][0-9]* '
height_window_re+='action=enabled window=visible controls=pass '
height_window_re+='zoom_in=pass zoom_out=pass actual=pass fit=pass '
height_window_re+='ctrl_wheel=pass result=pass'
grep -Eq "$height_window_re" "$tmpdir/default.log"
height_live_re='GUI Dexel height-map live test: stale_clear=pass '
height_live_re+='refresh=pass tool_filter=pass pixels_changed=pass zoom_state=pass '
height_live_re+='scroll_state=pass result=pass'
grep -Eq "$height_live_re" "$tmpdir/default.log"

# An explicitly persisted Full MC selection remains the reference path.
mkdir -p "$tmpdir/full-config/Cauldron Development"
cat > "$tmpdir/full-config/Cauldron Development/CAMotics.conf" <<'INI'
[Settings]
SimulationBackend=0
INI
run_gui "$tmpdir/full-config" "$tmpdir/full.log" \
  --simulation-output "$tmpdir/full-enabled.stl"
test "$(grep -c 'Simulation backend: full-mc' "$tmpdir/full.log")" -eq 1
test "$(grep -c 'Dexel eligibility accepted' "$tmpdir/full.log")" -eq 0

# The bundled Carvera Air profile must load its schematic geometry through the
# non-persistent launch override and run published-envelope diagnostics.
run_gui "$tmpdir/carvera-config" "$tmpdir/carvera.log" \
  --machine "Carvera Air"
grep -q 'Loaded machine Carvera Air' "$tmpdir/carvera.log"
grep -Eq 'Machine profile diagnostics: name=Carvera Air .* warnings=0' \
  "$tmpdir/carvera.log"

# The Settings dialog persists the same QSettings key.  Seed Auto directly so
# this smoke remains noninteractive, then verify selection and export.
mkdir -p "$tmpdir/auto-config/Cauldron Development"
cat > "$tmpdir/auto-config/Cauldron Development/CAMotics.conf" <<'INI'
[Settings]
SimulationBackend=1
INI

"$camsim" --threads 4 --dexel "$project" "$tmpdir/cli-auto.stl" \
  >"$tmpdir/cli-auto.log" 2>&1
run_gui "$tmpdir/auto-config" "$tmpdir/auto.log" \
  --view-frame tool \
  --simulation-output "$tmpdir/gui-auto.stl"
test "$(grep -c 'Simulation backend: dexel' "$tmpdir/auto.log")" -eq 1
test "$(grep -c 'Simulation complete: Dexel' "$tmpdir/auto.log")" -eq 1
test "$(grep -c 'GUI simulation surface written' "$tmpdir/auto.log")" -eq 1
grep -q 'GUI reference frame: tool' "$tmpdir/auto.log"
cmp -i 80 "$tmpdir/cli-auto.stl" "$tmpdir/gui-auto.stl"
python3 scripts/perf/compare_stl_geometry.py \
  "$tmpdir/cli-auto.stl" "$tmpdir/gui-auto.stl"

# Tool checkboxes must remove only that tool's stock cuts.  Exercise both
# backends, require all fixture moves to be filtered, and confirm that each
# result differs from its corresponding enabled simulation.  Full MC and
# Dexel represent an uncut cuboid differently, so they are not byte peers.
run_gui "$tmpdir/auto-config" "$tmpdir/disabled-auto.log" \
  --disable-tools 1 \
  --validate-dexel-topology \
  --simulation-output "$tmpdir/disabled-auto.stl"
run_gui "$tmpdir/full-config" "$tmpdir/disabled-full.log" \
  --disable-tools 1 \
  --simulation-output "$tmpdir/disabled-full.stl"
test "$(grep -c 'Simulation backend: dexel' \
  "$tmpdir/disabled-auto.log")" -eq 1
test "$(grep -c 'Simulation backend: full-mc' \
  "$tmpdir/disabled-full.log")" -eq 1
grep -q 'GUI simulation tool filter: disabled_tools=1 skipped_moves=17 retained_moves=17' \
  "$tmpdir/disabled-auto.log"
grep -q 'GUI simulation tool filter: disabled_tools=1 skipped_moves=17 retained_moves=17' \
  "$tmpdir/disabled-full.log"
if cmp -s -i 80 "$tmpdir/cli-auto.stl" "$tmpdir/disabled-auto.stl"; then
  echo "Disabled-tool result unexpectedly matches the enabled simulation" >&2
  exit 1
fi
if cmp -s -i 80 "$tmpdir/full-enabled.stl" "$tmpdir/disabled-full.stl"; then
  echo "Disabled-tool Full MC result unexpectedly matches enabled stock" >&2
  exit 1
fi

# Accelerated Play must coalesce intermediate requests, finish the last stock
# update before Auto Close, and leave the exact final surface published.
XDG_CONFIG_HOME="$tmpdir/auto-config" timeout 60s "${gui_prefix[@]}" \
  "$camotics" --threads 4 --auto-play --auto-close --play-speed 65536 \
  --simulation-output "$tmpdir/play-final.stl" "$project" \
  >"$tmpdir/play.log" 2>&1
cmp -i 80 "$tmpdir/cli-auto.stl" "$tmpdir/play-final.stl"
test "$(grep -c 'Simulation backend: full-mc' "$tmpdir/play.log" || true)" \
  -eq 0
test "$(grep -c 'GUI simulation scheduled' "$tmpdir/play.log")" -ge 1
test "$(grep -c 'GUI simulation completed' "$tmpdir/play.log")" -ge 1
test "$(grep -c 'GUI playback placeholder: uncut workpiece target=0' \
  "$tmpdir/play.log")" -eq 1
grep -q 'GUI simulation scheduled: generation=1 target=0' "$tmpdir/play.log"
grep -q 'Toolpath playback window:' "$tmpdir/play.log"
grep -Eq 'playback_indices=[1-9][0-9]*' "$tmpdir/play.log"
grep -q 'Dexel playback display LOD: speed=65536 stride=4' "$tmpdir/play.log"
grep -q 'playback_lag=' "$tmpdir/play.log"

# Long programs must use the exact playback clock, not the integer timeline
# slider.  At F0.1 this fixture's slider quantum is several seconds; a broken
# implementation therefore schedules only time zero during this bounded run.
mkdir -p "$tmpdir/long-play"
cp examples/slant_test/slant_test.camotics "$tmpdir/long-play/"
cp examples/slant_test/slant_test.nc "$tmpdir/long-play/"
sed -i 's/G21 T3 F5/G21 T3 F0.1/' "$tmpdir/long-play/slant_test.nc"
set +e
XDG_CONFIG_HOME="$tmpdir/auto-config" timeout 10s xvfb-run -a \
  "$camotics" --threads 4 --auto-play --play-speed 1 \
  "$tmpdir/long-play/slant_test.camotics" \
  >"$tmpdir/long-play.log" 2>&1
long_play_rc=$?
set -e
test "$long_play_rc" -eq 124
test "$(grep -c 'GUI simulation scheduled' "$tmpdir/long-play.log")" \
  -ge 10
test "$(grep -c 'GUI simulation completed' "$tmpdir/long-play.log")" \
  -ge 8
grep -q 'GUI simulation scheduled: generation=1 target=0' \
  "$tmpdir/long-play.log"
grep -Eq 'GUI simulation scheduled: generation=[2-9][0-9]* target=0\.[0-9]+' \
  "$tmpdir/long-play.log"
grep -q 'Dexel playback display LOD: speed=1 stride=1' \
  "$tmpdir/long-play.log"
grep -Eq 'Dexel boundary display update: tiles=[1-9][0-9]*' \
  "$tmpdir/long-play.log"
grep -Eq 'Dexel boundary display update:.*underlay_cells=[0-9]+' \
  "$tmpdir/long-play.log"

# Medium playback uses the 2-cell display grid without changing simulation
# state.  This bounded run needs only to observe the selected display LOD.
set +e
XDG_CONFIG_HOME="$tmpdir/auto-config" timeout 5s xvfb-run -a \
  "$camotics" --threads 4 --auto-play --play-speed 32 "$project" \
  >"$tmpdir/medium-play.log" 2>&1
medium_play_rc=$?
set -e
test "$medium_play_rc" -eq 124
grep -q 'Dexel playback display LOD: speed=32 stride=2' \
  "$tmpdir/medium-play.log"

# A deterministic burst replaces four obsolete targets with one newest task.
run_gui "$tmpdir/auto-config" "$tmpdir/burst.log" \
  --simulation-seek-burst-ratios 0.1,0.2,0.3,0.4
test "$(grep -c 'GUI simulation burst seek' "$tmpdir/burst.log")" -eq 4
test "$(grep -c 'GUI simulation scheduled' "$tmpdir/burst.log")" -eq 1
test "$(grep -c 'GUI simulation completed' "$tmpdir/burst.log")" -eq 1
test "$(grep -c 'Simulation backend: dexel-state' "$tmpdir/burst.log")" -eq 1
grep -Eq 'coalesced=([3-9]|[1-9][0-9]+)' "$tmpdir/burst.log"

# An accepted complete-time Auto run retains bounded Dexel checkpoints.  Both
# partial seeks must use that state without a partial-time full-MC fallback.
run_gui "$tmpdir/auto-config" "$tmpdir/timeline.log" \
  --simulation-seek-ratios 0.4,0.7
test "$(grep -c 'Simulation backend: dexel$' "$tmpdir/timeline.log")" -eq 1
test "$(grep -c 'Simulation backend: dexel-state' \
  "$tmpdir/timeline.log")" -eq 2
test "$(grep -c 'Dexel fallback to full MC: partial_time' \
  "$tmpdir/timeline.log" || true)" -eq 0
test "$(grep -c 'Simulation backend: full-mc' \
  "$tmpdir/timeline.log" || true)" -eq 0
test "$(grep -c 'Dexel eligibility accepted' "$tmpdir/timeline.log")" -eq 1
test "$(grep -c 'GUI simulation timeline seek' "$tmpdir/timeline.log")" -eq 2
test "$(grep -c 'GUI simulation scheduled' "$tmpdir/timeline.log")" -eq 2
test "$(grep -c 'GUI simulation completed' "$tmpdir/timeline.log")" -eq 2
test "$(grep -c 'Dexel state update:' "$tmpdir/timeline.log")" -eq 2
grep -q 'checkpoints=' "$tmpdir/timeline.log"
grep -q 'checkpoint_bytes=' "$tmpdir/timeline.log"
grep -q 'replayed_moves=' "$tmpdir/timeline.log"
test "$(grep -c 'Render aborted' "$tmpdir/timeline.log")" -eq 0
test "$(grep -c 'Computing surface at' "$tmpdir/timeline.log" || true)" -eq 0

# The toolbar End action pauses playback, seeks through the retained state, and
# publishes the canonical exact final result.
run_gui "$tmpdir/auto-config" "$tmpdir/end.log" \
  --simulation-seek-ratios 0.4 --simulation-go-to-end \
  --simulation-output "$tmpdir/end.stl"
grep -q 'GUI simulation go to end' "$tmpdir/end.log"
cmp -i 80 "$tmpdir/cli-auto.stl" "$tmpdir/end.stl"

# Cold restore, forward checkpoint replay, reverse restore, and repeated seeks
# must converge to byte-identical exact STL payloads at the same time.
run_gui "$tmpdir/auto-config" "$tmpdir/cold40.log" \
  --simulation-seek-ratios 0.4 \
  --simulation-output "$tmpdir/cold40.stl"
run_gui "$tmpdir/auto-config" "$tmpdir/forward70.log" \
  --simulation-seek-ratios 0.4,0.7 \
  --simulation-output "$tmpdir/forward70.stl"
run_gui "$tmpdir/auto-config" "$tmpdir/cold70.log" \
  --simulation-seek-ratios 0.7 \
  --simulation-output "$tmpdir/cold70.stl"
run_gui "$tmpdir/auto-config" "$tmpdir/reverse40.log" \
  --simulation-seek-ratios 0.7,0.4 \
  --simulation-output "$tmpdir/reverse40.stl"
run_gui "$tmpdir/auto-config" "$tmpdir/repeat40.log" \
  --simulation-seek-ratios 0.4,0.4 \
  --simulation-output "$tmpdir/repeat40.stl"
cmp -i 80 "$tmpdir/forward70.stl" "$tmpdir/cold70.stl"
cmp -i 80 "$tmpdir/cold40.stl" "$tmpdir/reverse40.stl"
cmp -i 80 "$tmpdir/cold40.stl" "$tmpdir/repeat40.stl"

echo "GUI simulation acceleration smoke passed"
