# Sparse surface extraction

Full marching cubes evaluates an entire workpiece volume even when a small
toolpath affects only a narrow part of it.  Sparse extraction divides the XY
domain into ownership regions, marches the uncertain toolpath-adjacent cells,
and reconstructs known untouched stock analytically.

Enable it explicitly:

```sh
./camsim --sparse-toolpath --threads 16 project.camotics sparse.stl
```

## Planning

The planner builds conservative XY bounds for every relevant tool sweep and
inserts them into a bounded bin index.  Active cells are expanded by a halo so
that marched geometry owns enough untouched context to close its boundary.

The public controls are:

- `--sparse-toolpath-xy-bins N`: planner bins per axis, default 64 and maximum
  256;
- `--sparse-toolpath-halo-cells N`: untouched safety halo, minimum 1;
- `--sparse-toolpath-target-region-cells N`: adaptive target size for active
  leaves; zero disables adaptive depth.

All bound, grid, tile, and cell conversions are checked before allocation.
Non-finite input, overflow, an oversized ownership grid, or an invalid plan
rejects the sparse candidate.

## Ownership and analytic stock

Every boundary face has one owner.  A region may contain:

- marching-cubes triangles from uncertain cells;
- analytic top, bottom, and side planes for known stock;
- transition faces between marched and analytic regions;
- locked seams that must not be changed by later reduction.

Analytic faces retain origin metadata so safe reduction can distinguish
reducible marching components from seams and already-minimal stock planes.

## Stitching

Stitching quantizes only for topology identity; emitted coordinates remain at
their computed positions.  Shared edges are ordered deterministically, winding
is corrected against outward normals, and duplicate or degenerate faces are
rejected.

The completed surface must pass:

- zero unexpected boundary edges;
- zero nonmanifold edges;
- consistent triangle winding;
- no duplicate triangles;
- valid ownership and seam accounting;
- sampled bidirectional distance to the full reference within the configured
  contract.

Failure at a gate discards the sparse result.  The CLI does not quietly write
a partially stitched surface.

## Adaptive depth

Large active areas may be split into smaller leaves until the target cell
count is reached.  Parent/child boundaries use the same ownership and halo
rules as uniform regions.  Adaptive depth changes work partitioning, not the
simulation resolution or requested surface tolerance.

## Compatibility

Sparse extraction cannot currently be combined with Dexel, stock-bounded
ToolSweep mode, or the adaptive-Z rendering options.  Full marching cubes is
the reference used by the validation scripts and remains the operational
fallback when a job is not a good sparse candidate.

The sparse path is experimental.  Use the provided topology and distance
tools for any workflow that treats STL output as a release artifact; see
[BENCHMARKS.md](BENCHMARKS.md).
