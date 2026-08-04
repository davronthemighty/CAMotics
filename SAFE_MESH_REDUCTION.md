# Safe mesh reduction

`--safe-reduce` removes redundant triangles only from planar components that
pass geometric and topological validation.  It is separate from the older
`--reduce` option and cannot be combined with it.

```sh
./camsim --safe-reduce --safe-reduce-report report.json \
  project.camotics reduced.stl
```

Use `--safe-reduce-report` by itself to inspect candidates without changing
the output surface.

## Deviation contract

Triangles are grouped only when their vertices lie within the configured
plane-distance tolerance and their normals meet the pairwise angle limit.
The defaults are:

- plane distance: `0.0001` project units;
- normal angle: `0.25` degrees;
- maximum accepted normal angle: `5` degrees.

The reducer does not use triangle count as permission to remove detail.  A
replacement must stay within the requested plane tolerance at its source
vertices and boundaries.  Non-finite tolerances, coordinates, normals, or
unrepresentable quantization are rejected before mutation.

## Topology invariants

For each proposed component, the reducer preserves:

- boundary loops and their orientation;
- shared-edge incidence;
- holes when hole-aware mode is enabled;
- nondegenerate triangles with consistent winding;
- protected sparse seams and analytic-surface identity;
- whole-surface boundary, nonmanifold, duplicate, and component counts.

The default production path handles simple one-loop planar regions.  Optional
hole-aware and boundary co-simplification paths have additional gates:

```sh
--safe-reduce-hole-aware
--safe-reduce-boundary-cosimplify
```

Boundary co-simplification is experimental because neighboring components must
agree on each removed boundary vertex.

## Rejections and rollback

The report separates reasons instead of counting every unmodified triangle as
one generic failure.  Important classes include:

- component touches an unsafe or unsupported boundary;
- replacement saves no triangles;
- loop ordering or triangulation failed;
- edge incidence or winding failed;
- sparse origin metadata was incomplete;
- whole-surface validation failed after tentative application.

Tentative replacements are assembled away from the published surface.  If the
final validation differs from the input topology contract, the entire
candidate is rolled back.  Worker exceptions are propagated only after all
started workers have joined.

The summary reports input/output triangles, applied components and source
triangles, estimated reduction, watertight input/output status, boundary and
nonmanifold edges, rejection totals, validation rollbacks, and unaccounted
triangles.  Accounting is overflow-checked.

## Provenance neighbors

Contour provenance can avoid rebuilding a triangle adjacency table, but it is
not trusted by default.  The two-stage opt-in is:

```sh
--safe-reduce-provenance-neighbors
--safe-reduce-trust-provenance-neighbors
```

The trusted path is used only when provenance is complete, raw and welded
topology agree, and parity with normal adjacency has already been established.
Otherwise the reducer uses the ordinary adjacency table.

## Limits

Safe reduction is intentionally conservative.  Curved, noisy, tiny,
multi-loop, seam-heavy, or already efficient components may show no savings.
A rejection means that this reducer did not prove the replacement safe; it
does not mean that no possible mesh simplification exists.
