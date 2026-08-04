# Benchmarks

Benchmark claims in this repository are tied to a command, source revision,
compiler, hardware description, and output checks.  A faster run is not
accepted when its geometry or topology gate fails.

## Public 40 mm stress fixture

The fixture generator is deterministic and self-contained:

```sh
python3 scripts/benchmarks/generate_40mm_stress_fixture.py \
  --output-dir build/benchmarks/40mm-stress
```

Its default output is a 40 x 40 x 0.8 mm workpiece at 0.025 mm resolution with
about 95,000 rapid/cutting blocks.  Concentric relief, radial cuts, and a dense
boustrophedon hatch exercise conical-flank rasterization, short-move indexing,
crossing cuts, two supported tools, and safe-reduce rejection accounting.

Run all four required cases:

```sh
THREADS=18 scripts/benchmarks/benchmark_40mm.sh \
  ./camsim build/benchmarks/40mm-comparison
```

The harness records:

- full MC, full MC with safe reduction, Dexel, and Dexel with safe reduction;
- wall, user, and system time plus maximum RSS;
- JSON profiles, selected backend, fallbacks, and rejection counters;
- triangle/output size and SHA-256;
- boundary/nonmanifold topology metrics from Dexel validation and the
  safe-reduce input/output reports;
- bidirectional sampled distance for backend and reduction comparisons.

## Release test system

The local v2026.08.0 release gate uses:

| Item | Value |
| --- | --- |
| CPU | AMD Ryzen AI 9 365, 9 cores / 18 WSL logical CPUs |
| Memory available to WSL | 23 GiB |
| OS | WSL2, Linux 6.18 x86-64 |
| Compiler | GCC 15.2.0, C++17, release `-O3` |
| Threads | 18 unless stated otherwise |
| Resolution | 0.025 mm |

The release-candidate run used source checkpoint `e58c4e596543` and the command
above. The generated project and NC SHA-256 values were
`0d61ffbdcbbed3df445c3b3d00153409cc671779a648df04d37b9fc3633f952a`
and `5ef61a6c4e549a3b2f980dc32184e07b3bdc0a2885e391c548eeb7427d6eeb70`.
The harness rejected any profile that did not report exactly 0.025 mm.

| Case | Wall | CPU | Peak RSS | Triangles | Binary STL |
| --- | ---: | ---: | ---: | ---: | ---: |
| Full MC | 204.05 s | 290% | 22.10 GiB | 17,425,816 | 830.9 MiB |
| Full MC + safe reduce | 243.94 s | 291% | 22.10 GiB | 11,302,802 | 539.0 MiB |
| Dexel | 11.87 s | 110% | 1.73 GiB | 5,139,200 | 245.1 MiB |
| Dexel + safe reduce | 21.94 s | 107% | 1.73 GiB | 4,876,224 | 232.5 MiB |

Dexel was 17.2 times faster than full MC without reduction and 11.1 times
faster when both outputs were reduced. Full MC extraction used multiple cores,
but topology accounting and export were substantially single-threaded. Dexel
also spent most of this fixture's wall time in single-threaded surface
construction and output. CPU percentage is the aggregate value reported by
GNU `time`, where 100% is one fully occupied logical CPU.

All four meshes were watertight with zero boundary and nonmanifold edges. The
20,000-triangle bidirectional sampler tested 80,000 points in each direction.
Full MC to Dexel had a 0.02344 mm maximum and 0.01439 mm p99 distance; Dexel to
full MC had a 0.01849 mm maximum and 0.008934 mm p99 distance. Both maxima were
below one 0.025 mm grid interval.

Safe reduction removed 35.1% of full-MC triangles. Its sampled maximum change
was 0.0000308 mm (0.0308 micrometres), with a p99 of 0.000000061 mm. Dexel safe
reduction removed 5.12%; its sampled maximum change was 0.0000695 mm (0.0695
micrometres), with a p99 below 0.000000002 mm.

| Safe-reduce decision | Full MC | Dexel |
| --- | ---: | ---: |
| Applied components | 54,838 | 6,559 |
| Applied source triangles | 6,906,446 | 399,755 |
| Applied replacement triangles | 783,432 | 136,779 |
| Boundary rejection, components / triangles | 584 / 26,244 | 6 / 911 |
| No-savings rejection, components / triangles | 6,344,123 / 8,735,418 | 1,909,533 / 2,990,651 |
| Triangulation rejection, components / triangles | 0 / 0 | 0 / 0 |
| Edge-incidence rejection | 0 | 0 |
| Whole-surface validation rollback | 0 | 0 |

The full-MC cases nearly filled the configured 23 GiB WSL memory limit. The
Dexel cases stayed below 2 GiB. These figures are measurements of this public
fixture, not general speed or memory guarantees.

## Retained compact contracts

The compact fixtures are intended to detect drift, not rank hardware.  Current
protected results include:

| Contract | Result |
| --- | --- |
| Accepted Dexel slant surface | 120,032 triangles |
| Dexel STL payload SHA-256 | `a8bfcd4ba90fccd3d7d430075781f04e44840f854a8063cd2842dac300e8b5f7` |
| Retained-state checkpoints | 13 |
| Retained-state checkpoint bytes | 6,141,096 |
| Retained current-state bytes | 7,745,132 |
| Retained boundary triangles | 89,296 |

The payload hash excludes the 80-byte binary STL header, which contains build
metadata.  Cross-thread contracts also require identical trusted output at 1,
2, 4, and 10 threads.

## Geometry tools

Repository tools are usable independently:

```sh
python3 scripts/perf/compare_stl_geometry.py left.stl right.stl
python3 scripts/perf/compare_stl_distance_streaming.py \
  --hard-max-error 0.05 --p99-error 0.025 left.stl right.stl
python3 scripts/perf/mesh_reduction_contract_report.py \
  --edge-incidence-only --output-json topology.json result.stl
```

Distance is checked in both directions.  Topology reports distinguish boundary
edges, nonmanifold edges, duplicate triangles, components, and reducer
rejections.  A one-direction nearest-surface check is not sufficient.

## Regression thresholds

For an unchanged output contract, compare at least three matched runs and use
the median.  Investigate an unexplained wall-time regression over 5% or a peak
RSS increase over the greater of 2% and 16 MiB.  Runtime noise never overrides
an exact hash, topology, maximum-deviation, or fallback failure.

If a simulation is unexpectedly single-core and projected to take more than
one hour, cancel it, identify why parallel work was not enabled, and retry with
the multicore path before collecting a benchmark.
