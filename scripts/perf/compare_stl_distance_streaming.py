#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Memory-bounded sampled STL distance for multi-gigabyte binary meshes."""

from __future__ import annotations

import argparse
import array
import json
import math
import mmap
import struct
from pathlib import Path

from compare_stl_distance import closest_point_on_triangle, dist2


FACET = struct.Struct("<12fH")


class BinarySTL:
    def __init__(self, path: Path):
        self.path = path
        self.file = path.open("rb")
        self.map = mmap.mmap(self.file.fileno(), 0, access=mmap.ACCESS_READ)
        if len(self.map) < 84:
            raise ValueError(f"STL is too small: {path}")
        self.count = struct.unpack_from("<I", self.map, 80)[0]
        expected = 84 + self.count * FACET.size
        if len(self.map) != expected:
            raise ValueError(f"Streaming comparison requires binary STL: {path}")

    def close(self):
        self.map.close()
        self.file.close()

    def triangle(self, index: int):
        values = FACET.unpack_from(self.map, 84 + index * FACET.size)
        return (values[3:6], values[6:9], values[9:12])

    def bounds(self):
        lo = [math.inf, math.inf, math.inf]
        hi = [-math.inf, -math.inf, -math.inf]
        for index in range(self.count):
            triangle = self.triangle(index)
            for point in triangle:
                for axis in range(3):
                    lo[axis] = min(lo[axis], point[axis])
                    hi[axis] = max(hi[axis], point[axis])
        return tuple(lo), tuple(hi)


def union_bounds(a, b):
    return (
        tuple(min(a[0][axis], b[0][axis]) for axis in range(3)),
        tuple(max(a[1][axis], b[1][axis]) for axis in range(3)),
    )


def bin_coord(value, lo, hi, bins):
    if hi <= lo:
        return 0
    return max(0, min(bins - 1, int((value - lo) * bins / (hi - lo))))


def build_index(stl, bounds, bins, expansion):
    cells = [array.array("I") for _ in range(bins * bins)]
    lo, hi = bounds
    refs = 0
    for index in range(stl.count):
        triangle = stl.triangle(index)
        min_x = min(point[0] for point in triangle) - expansion
        max_x = max(point[0] for point in triangle) + expansion
        min_y = min(point[1] for point in triangle) - expansion
        max_y = max(point[1] for point in triangle) + expansion
        x0 = bin_coord(min_x, lo[0], hi[0], bins)
        x1 = bin_coord(max_x, lo[0], hi[0], bins)
        y0 = bin_coord(min_y, lo[1], hi[1], bins)
        y1 = bin_coord(max_y, lo[1], hi[1], bins)
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                cells[y * bins + x].append(index)
                refs += 1
    return cells, refs


def triangle_points(triangle):
    a, b, c = triangle
    yield a
    yield b
    yield c
    yield tuple((a[i] + b[i] + c[i]) / 3 for i in range(3))


def summarize_direction(source, target, bounds, bins, hard_max, max_triangles,
                        cut_surface, cut_epsilon):
    cells, refs = build_index(target, bounds, bins, hard_max)
    lo, hi = bounds
    sample_triangles = min(source.count, max_triangles)
    indices = [i * source.count // sample_triangles
               for i in range(sample_triangles)]
    distances = []
    missing = 0
    max_distance = -1.0
    max_point = None
    max_source_triangle = None

    for index in indices:
        triangle = source.triangle(index)
        if cut_surface:
            centroid = tuple(sum(point[axis] for point in triangle) / 3
                             for axis in range(3))
            lo, hi = bounds
            if (centroid[2] <= lo[2] + cut_epsilon or
                    centroid[2] >= hi[2] - cut_epsilon or
                    centroid[0] <= lo[0] + cut_epsilon or
                    centroid[0] >= hi[0] - cut_epsilon or
                    centroid[1] <= lo[1] + cut_epsilon or
                    centroid[1] >= hi[1] - cut_epsilon):
                continue
        points = (centroid,) if cut_surface else triangle_points(triangle)
        for point in points:
            x = bin_coord(point[0], lo[0], hi[0], bins)
            y = bin_coord(point[1], lo[1], hi[1], bins)
            candidates = cells[y * bins + x]
            if not candidates:
                distances.append(math.inf)
                missing += 1
                continue
            best = math.inf
            for candidate in candidates:
                closest = closest_point_on_triangle(
                    point, target.triangle(candidate))
                best = min(best, dist2(point, closest))
            distance = math.sqrt(best)
            distances.append(distance)
            if max_distance < distance:
                max_distance = distance
                max_point = point
                max_source_triangle = index

    distances.sort()
    count = len(distances)
    percentile = lambda p: distances[min(count - 1, int(p * count))]
    return {
        "samples": count,
        "target_index_refs": refs,
        "target_nonempty_bins": sum(bool(cell) for cell in cells),
        "missing_candidate_samples": missing,
        "max_point": max_point,
        "max_source_triangle": max_source_triangle,
        "max": distances[-1],
        "p99": percentile(0.99),
        "p95": percentile(0.95),
        "rms": math.sqrt(sum(value * value for value in distances) / count),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--hard-max-error", type=float, required=True)
    parser.add_argument("--p99-error", type=float, required=True)
    parser.add_argument("--max-triangles", type=int, default=10000)
    parser.add_argument("--bins", type=int, default=256)
    parser.add_argument("--cut-surface", action="store_true")
    parser.add_argument("--cut-epsilon", type=float, default=1e-6)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    baseline = BinarySTL(args.baseline)
    candidate = BinarySTL(args.candidate)
    try:
        bounds = union_bounds(baseline.bounds(), candidate.bounds())
        forward = summarize_direction(
            baseline, candidate, bounds, args.bins,
            args.hard_max_error, args.max_triangles,
            args.cut_surface, args.cut_epsilon)
        reverse = summarize_direction(
            candidate, baseline, bounds, args.bins,
            args.hard_max_error, args.max_triangles,
            args.cut_surface, args.cut_epsilon)
    finally:
        baseline.close()
        candidate.close()

    failures = []
    for name, summary in (("baseline_to_candidate", forward),
                          ("candidate_to_baseline", reverse)):
        if summary["missing_candidate_samples"]:
            failures.append(f"{name} has samples without indexed candidates")
        if summary["max"] > args.hard_max_error:
            failures.append(
                f"{name} max {summary['max']} > {args.hard_max_error}")
        if summary["p99"] > args.p99_error:
            failures.append(
                f"{name} p99 {summary['p99']} > {args.p99_error}")

    result = {
        "equal_within_tolerance": not failures,
        "failures": failures,
        "baseline_triangles": baseline.count,
        "candidate_triangles": candidate.count,
        "bounds": bounds,
        "cut_surface": args.cut_surface,
        "cut_epsilon": args.cut_epsilon,
        "baseline_to_candidate": forward,
        "candidate_to_baseline": reverse,
    }
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        for name in ("baseline_to_candidate", "candidate_to_baseline"):
            summary = result[name]
            print(
                f"{name}: samples={summary['samples']} "
                f"max={summary['max']:.9g} p99={summary['p99']:.9g} "
                f"p95={summary['p95']:.9g} rms={summary['rms']:.9g}"
            )
        for failure in failures:
            print(failure)

    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
