# GUI playback

CAMotics Fast adds retained simulation playback for eligible Dexel jobs while
keeping full marching cubes available from the simulation backend setting.

## What is cached

After a complete eligible run, the GUI retains the prepared path, tile index,
current height grid, and a bounded set of checkpoints.  Timeline requests are
immutable and coalesced to the newest target.  One worker mutates a run at a
time, and an obsolete completed generation is discarded rather than displayed
late.

Forward playback updates only the open interval.  Reverse and random seeks
restore an earlier checkpoint and replay its suffix.  Exact partial-move
boundaries use an undo patch.  Playback does not repeatedly materialize the
complete triangle soup; export, Reduce, and wireframe request exact surface
materialization when needed.

During fast motion the renderer uses a bounded stock display LOD.  Pause,
seek, export, Reduce, and wireframe restore full display detail.  LOD changes
only presentation; the retained height state is not downsampled.

## Controls

- Space: play or pause while Simulation or Tool View is active.
- End button: stop playback and publish the exact final simulation result.
- Stock frame / Tool frame: change the camera reference without changing the
  stock state.
- Tool Table checkbox: exclude that tool's cutting moves from the simulated
  stock.  The visible path and timing remain available for comparison.
- View > Docks > Tool Table: restore a hidden tool table.

Mouse controls:

- left drag: orbit;
- middle drag: pan;
- wheel: zoom.

Precision trackpad controls:

- two-finger scroll: pan;
- Shift + two-finger vertical scroll: horizontal pan;
- pinch or Ctrl + two-finger scroll: zoom;
- Alt + two-finger scroll: orbit.

The active stock/tool reference center is used for these operations.

## Status bar

Playback state stays visible while running or paused.  The status area reports
state, selected backend or fallback, percentage, path time, move progress,
playback speed, and lag when available.  Progress is not carried by a transient
toolbar icon, so the layout does not bounce as worker status changes.

## Live Dexel height map

Tools > Dexel Height Map and the playback toolbar Height Map button open a
separate nonmodal window when the current result exposes a Dexel grid.  The
window follows accepted seeks, playback updates, backend changes, project
reloads, and tool filtering.  It clears stale pixels while a replacement state
is pending.

Image controls are:

- Zoom In / Zoom Out (`Ctrl++`, `Ctrl+-`);
- Actual Size (`Ctrl+0`);
- Fit to Window (`F`);
- Ctrl + wheel or trackpad scroll to zoom;
- unmodified scrolling to pan the image.

The window retains only the original-resolution pixmap.  Display zoom does not
allocate a permanent 1600% copy.

## Exact boundaries and startup

Pressing Play before retained state is ready displays uncut stock at time zero
and schedules the first exact state.  It does not advance the cutter over a
blank or stale surface.  Boundary patches are composited over a complete
closed grid underlay so early cuts do not reveal background holes.

## Machine profiles

The bundled Carvera Air profile supplies a schematic machine model, travel
envelope, feed and spindle limits, and reference-frame geometry.  Those values
support visualization and diagnostics.  They do not add controller
interpolation, acceleration, backlash, runout, flex, cutting-force, or
collision physics to the material simulation.
