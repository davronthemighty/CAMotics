# Accelerated simulation

CAMotics Fast keeps one conservative rule: an optimization may avoid work, but
it may not silently change the requested cut.  Full marching cubes remains the
reference implementation and the automatic fallback.

## Processing path

The simulator first parses and normalizes the G-code into a timed `ToolPath`.
From there, the available paths are:

1. Full marching cubes samples the stock volume and evaluates cutter sweeps at
   each required location.
2. Dexel simulation rasterizes a supported 3-axis path into one retained top
   height per XY column, then constructs a closed surface.
3. Sparse extraction plans toolpath-adjacent ownership regions, marches only
   uncertain cells, and stitches those regions to analytic untouched stock.
4. Safe reduction may replace validated planar triangle components after any
   surface has been built.

The CLI does not select experimental geometry automatically.  `--dexel` and
`--sparse-toolpath` are explicit requests.  Dexel requests still fall back to
full marching cubes when eligibility or runtime validation fails.  The GUI's
Auto setting performs the same Dexel eligibility check; Full MC remains a
selectable persistent reference setting.

## ToolSweep indexing

Full marching cubes asks which tool moves can affect each sample.  Testing
every move at every point dominated small-tool jobs, so the fork adds spatial
indices around the existing sweep objects.

- XY bins index conservative projected move bounds.
- XYZ bins are an experimental three-dimensional alternative.
- Stock-bounded sweeps clip conservative Z bounds to the sampled workpiece
  slab.
- Empty and boundary bins are handled explicitly; candidate order is stable.
- Every accepted candidate still uses the original cutter-distance function.

The index changes candidate lookup, not cutter geometry.  The cross-thread
contract compares exact output at 1, 2, 4, and 10 threads.

Useful controls:

```sh
./camsim --toolsweep-xy-bins 64 project.camotics result.stl
./camsim --toolsweep-xyz-bins 32 project.camotics result.stl
./camsim --toolsweep-stock-bounds project.camotics result.stl
```

`--perf-advice` reports applicable options without rendering.  `--perf-warnings`
adds warnings about small resolution, large envelopes, and expensive stock
setups to a normal run.

## Parallelism and determinism

Raster tiles, marching partitions, sparse regions, and reducer components use
bounded worker groups.  Worker failures are captured, all started threads are
joined, and the original exception is returned to the caller.  Cancellation
does not publish a partial result.

Determinism is enforced at merge boundaries:

- stable region and tile identifiers;
- explicit ownership of shared faces and seam vertices;
- deterministic index and triangle ordering where output identity is a
  contract;
- topology and sampled-distance comparisons where different valid
  triangulations are allowed.

Thread count defaults to the detected logical CPU count and can be set with
`--threads N`.

## Profiling

Use `--profile FILE.json` to record phases and counters.  A typical comparison
also uses `/usr/bin/time -v` for wall time, CPU time, and peak resident memory:

```sh
/usr/bin/time -v ./camsim --threads 16 --profile full.json \
  project.camotics full.stl
/usr/bin/time -v ./camsim --dexel --threads 16 --profile dexel.json \
  project.camotics dexel.stl
```

Profiles include the selected backend, eligibility and fallback reason,
ToolSweep candidate metrics, surface size, reducer accounting, and Dexel grid
or checkpoint statistics when applicable.

## Feature status

| Path | Suitable work | Main rejection or risk |
| --- | --- | --- |
| Full MC | General supported CAMotics simulation | Highest time and memory cost |
| XY ToolSweep index | Dense paths with localized XY motion | Little benefit on very short paths |
| Dexel | 3-axis, top-down, single-height stock removal | Undercuts, rotary motion, unsupported tools |
| Sparse extraction | Toolpath occupies a small part of a large volume | Ownership or stitching validation failure |
| Safe reduction | Large coplanar mesh regions | No safe savings or topology/deviation failure |

See the backend-specific documents before enabling experimental output in an
automated process.
