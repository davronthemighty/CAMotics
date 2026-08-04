# Dexel simulation

The Dexel backend is a 2.5D stock-removal model.  It stores one top height for
each point in a uniform XY grid.  This makes top-down engraving and milling
much cheaper than repeatedly sampling a full XYZ volume, while retaining the
requested XY resolution.

## Cutter surface

Rasterization evaluates the cutter surface at each affected Dexel center.  It
does not paint an entire cutter footprint at the commanded tip depth.

For a move with tool-tip height `z`, lateral distance `r`, tip radius `r0`,
outer radius `R`, and tool length `L`, the supported profiles are:

- cylindrical: the cutting surface is `z` for `r <= R`;
- conical: `z + r L / R` for `r <= R`;
- snubnose: `z` for `r <= r0`, then
  `z + (r - r0) L / (R - r0)` for `r0 < r <= R`.

For a linear move, the implementation minimizes that physical surface along
the segment.  The retained bounded refinement is deterministic and is covered
by sloped-move geometry hashes.  Each grid value is lowered only when a cutter
surface removes stock.

The XY pitch is exactly the simulation resolution.  A `0.025 mm` project has
Dexel samples `0.025 mm` apart in both X and Y; camera angle, flat shading, or
triangle edges can make those samples appear wider on screen.

## Eligibility

Dexel accepts a job only when all of these are true:

- the project, workpiece, resolution, moves, times, and tools are finite and
  representable;
- the workpiece starts as a simple stock volume without an imported initial
  surface;
- rendering uses the marching-cubes-compatible solid mode;
- the requested time is the complete path for a one-shot CLI run;
- moves are rapid or cutting moves with no rotary or auxiliary axes;
- every referenced tool exists and is cylindrical, conical, or snubnose with
  valid dimensions.

Eligibility-only inspection is cheap:

```sh
./camsim --dexel-eligibility-only --profile eligibility.json project.camotics
```

Profile keys use stable names such as
`dexel_rejection_rotary_or_aux_axes` and
`dexel_rejection_unsupported_tool`.

## Runtime fallback

Some failures are known only while rasterizing or building the surface.  The
backend abandons the candidate and reruns full marching cubes when it detects:

- a cut that would require more than one solid interval in an XY column;
- an emptied or through-bottom column that the single-height model cannot
  represent;
- topology or geometry validation failure;
- invalid allocation, range, or surface-size accounting.

The profile records `dexel_fallback=1` and one named
`dexel_fallback_<reason>` counter.  Cancellation is different: it returns a
cancelled task and does not start a fallback or publish partial stock.

## Surface construction

The direct surface contains:

- a triangulated top height grid;
- vertical boundary walls around the workpiece;
- a closed bottom;
- exact boundary patches for discontinuities where the direct grid alone
  would hide early cut detail.

Ownership rules prevent adjacent tiles from emitting the same face.  The CLI
validates boundary and nonmanifold edge counts by default.  The advanced
`--dexel-skip-topology-validation` switch exists for measured production runs
after the same build has passed the retained topology suite.

## Height-map export

```sh
./camsim --dexel --dexel-grid-png height.png \
  --profile run.json project.camotics result.stl
```

The PNG is a true 8-bit grayscale image.  The deepest top height observed in
the grid is 0 (black), the highest is 255 (white), and positive Y is upward.
The profile records width, height, signed minimum and maximum Z, and PNG byte
count.  Quantization is for the image only; the retained simulation grid uses
floating-point heights.

## Playback checkpoints

The GUI may retain the final grid, prepared move/tile data, and a bounded set
of work-balanced checkpoints.  Forward updates resume from the current state;
reverse or random seeks restore the nearest earlier checkpoint and replay its
suffix.  A partially executed move has a transactional undo patch so exact
time boundaries do not require rounding to whole moves.

The default checkpoint budget is 256 MiB.  State construction and every seek
are transactional: cancellation leaves the last published grid unchanged.

## Limitations

A Z-dexel is one height interval per XY location.  It cannot represent an
undercut, horizontal tunnel, separate overhang, or arbitrary 4/5-axis stock.
Those jobs use full marching cubes.  Dexel output is still sampled geometry,
not the exact motion lattice or servo path of a particular CNC controller.
