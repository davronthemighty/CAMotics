#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Sample bidirectional surface distance between two STL files.

This is a no-library quality gate for intentionally lossy output changes such
as mesh reduction. For exact-preservation changes, use compare_stl_geometry.py
instead.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

from compare_stl_geometry import cross, dot, iter_stl, length, sub


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def dist2(a, b):
    d = sub(a, b)
    return dot(d, d)


def triangle_area(tri):
    a, b, c = tri
    return 0.5 * length(cross(sub(b, a), sub(c, a)))


def triangle_bounds(tri):
    return (
        tuple(min(v[i] for v in tri) for i in range(3)),
        tuple(max(v[i] for v in tri) for i in range(3)),
    )


def triangle_centroid(tri):
    return mul(add(add(tri[0], tri[1]), tri[2]), 1.0 / 3.0)


def bounds_for_path(path):
    mins = [math.inf, math.inf, math.inf]
    maxs = [-math.inf, -math.inf, -math.inf]
    count = 0

    for tri in iter_stl(path):
        count += 1
        lo, hi = triangle_bounds(tri)
        for i in range(3):
            mins[i] = min(mins[i], lo[i])
            maxs[i] = max(maxs[i], hi[i])

    if count == 0:
        raise ValueError(f"No triangles found in {path}")

    return (tuple(mins), tuple(maxs)), count


def in_cut_surface(tri, bounds, z_epsilon, xy_epsilon):
    lo, hi = bounds
    c = triangle_centroid(tri)

    if c[2] >= hi[2] - z_epsilon:
        return False
    if c[2] <= lo[2] + z_epsilon:
        return False
    if c[0] <= lo[0] + xy_epsilon or c[0] >= hi[0] - xy_epsilon:
        return False
    if c[1] <= lo[1] + xy_epsilon or c[1] >= hi[1] - xy_epsilon:
        return False

    return True


def load_triangles(path, cut_surface=False, cut_z_epsilon=1e-9,
                   cut_xy_epsilon=1e-9):
    load_summary = {
        "path": str(path),
        "cut_surface": cut_surface,
        "cut_z_epsilon": cut_z_epsilon,
        "cut_xy_epsilon": cut_xy_epsilon,
    }

    if cut_surface:
        bounds, total = bounds_for_path(path)
        triangles = [
            tri for tri in iter_stl(path)
            if in_cut_surface(tri, bounds, cut_z_epsilon, cut_xy_epsilon)
        ]
        load_summary.update({
            "total_triangles": total,
            "loaded_triangles": len(triangles),
            "bounds_min": bounds[0],
            "bounds_max": bounds[1],
        })
    else:
        triangles = list(iter_stl(path))
        load_summary.update({
            "total_triangles": len(triangles),
            "loaded_triangles": len(triangles),
        })

    if not triangles:
        kind = "cut-surface triangles" if cut_surface else "triangles"
        raise ValueError(f"No {kind} found in {path}")

    return triangles, load_summary


def bounds_for_triangles(triangles):
    mins = [math.inf, math.inf, math.inf]
    maxs = [-math.inf, -math.inf, -math.inf]
    for tri in triangles:
        lo, hi = triangle_bounds(tri)
        for i in range(3):
            mins[i] = min(mins[i], lo[i])
            maxs[i] = max(maxs[i], hi[i])
    return tuple(mins), tuple(maxs)


def default_cell_size(bounds):
    lo, hi = bounds
    diag = length(sub(hi, lo))
    return diag / 96 if diag else 1.0


def cell_coord(p, origin, cell_size):
    return tuple(math.floor((p[i] - origin[i]) / cell_size) for i in range(3))


def build_grid(triangles, cell_size=None):
    bounds = bounds_for_triangles(triangles)
    origin = bounds[0]
    cell_size = cell_size or default_cell_size(bounds)
    grid = defaultdict(list)

    for index, tri in enumerate(triangles):
        lo, hi = triangle_bounds(tri)
        c0 = cell_coord(lo, origin, cell_size)
        c1 = cell_coord(hi, origin, cell_size)
        for x in range(c0[0], c1[0] + 1):
            for y in range(c0[1], c1[1] + 1):
                for z in range(c0[2], c1[2] + 1):
                    grid[(x, y, z)].append(index)

    return {
        "triangles": triangles,
        "bounds": bounds,
        "origin": origin,
        "cell_size": cell_size,
        "grid": grid,
    }


def closest_point_on_triangle(p, tri):
    # Christer Ericson, Real-Time Collision Detection, section 5.1.5.
    a, b, c = tri
    ab = sub(b, a)
    ac = sub(c, a)
    ap = sub(p, a)
    d1 = dot(ab, ap)
    d2 = dot(ac, ap)
    if d1 <= 0 and d2 <= 0:
        return a

    bp = sub(p, b)
    d3 = dot(ab, bp)
    d4 = dot(ac, bp)
    if d3 >= 0 and d4 <= d3:
        return b

    vc = d1 * d4 - d3 * d2
    if vc <= 0 and d1 >= 0 and d3 <= 0:
        v = d1 / (d1 - d3)
        return add(a, mul(ab, v))

    cp = sub(p, c)
    d5 = dot(ab, cp)
    d6 = dot(ac, cp)
    if d6 >= 0 and d5 <= d6:
        return c

    vb = d5 * d2 - d1 * d6
    if vb <= 0 and d2 >= 0 and d6 <= 0:
        w = d2 / (d2 - d6)
        return add(a, mul(ac, w))

    va = d3 * d6 - d5 * d4
    if va <= 0 and (d4 - d3) >= 0 and (d5 - d6) >= 0:
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return add(b, mul(sub(c, b), w))

    denom = 1.0 / (va + vb + vc)
    v = vb * denom
    w = vc * denom
    return add(add(a, mul(ab, v)), mul(ac, w))


def shell_cells(center, radius):
    cx, cy, cz = center
    for x in range(cx - radius, cx + radius + 1):
        for y in range(cy - radius, cy + radius + 1):
            for z in range(cz - radius, cz + radius + 1):
                if max(abs(x - cx), abs(y - cy), abs(z - cz)) == radius:
                    yield (x, y, z)


def nearest_distance(point, target, max_shells):
    grid = target["grid"]
    triangles = target["triangles"]
    center = cell_coord(point, target["origin"], target["cell_size"])
    best = math.inf
    visited = set()

    for radius in range(max_shells + 1):
        found = False
        for cell in shell_cells(center, radius):
            if cell in visited:
                continue
            visited.add(cell)
            for index in grid.get(cell, ()):
                found = True
                closest = closest_point_on_triangle(point, triangles[index])
                best = min(best, dist2(point, closest))

        if found and best <= (radius * target["cell_size"]) ** 2:
            break

    if math.isinf(best):
        for tri in triangles:
            closest = closest_point_on_triangle(point, tri)
            best = min(best, dist2(point, closest))

    return math.sqrt(best)


def sample_points(triangles, max_samples, point_mode):
    points = []
    for tri in triangles:
        if point_mode in ("vertices", "all"):
            points.extend(tri)
        if point_mode in ("centroid", "all"):
            points.append(triangle_centroid(tri))

    if len(points) <= max_samples:
        return points

    stride = len(points) / max_samples
    return [points[min(int(i * stride), len(points) - 1)]
            for i in range(max_samples)]


def percentile(values, pct):
    if not values:
        return 0.0
    index = (len(values) - 1) * pct / 100.0
    lo = math.floor(index)
    hi = math.ceil(index)
    if lo == hi:
        return values[int(index)]
    return values[lo] * (hi - index) + values[hi] * (index - lo)


def summarize_distances(name, source, target, max_samples, max_shells,
                        tolerance, point_mode):
    samples = sample_points(source, max_samples, point_mode)
    distances = sorted(nearest_distance(p, target, max_shells) for p in samples)
    over = sum(1 for d in distances if tolerance is not None and d > tolerance)
    rms = math.sqrt(sum(d * d for d in distances) / len(distances))
    return {
        "name": name,
        "samples": len(samples),
        "max": distances[-1],
        "p99": percentile(distances, 99),
        "p95": percentile(distances, 95),
        "rms": rms,
        "over_tolerance": over,
    }


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--max-samples", type=int, default=20000)
    parser.add_argument("--cell-size", type=float)
    parser.add_argument("--max-shells", type=int, default=12)
    parser.add_argument("--hard-max-error", type=float)
    parser.add_argument("--p99-error", type=float)
    parser.add_argument(
        "--point-mode", choices=("centroid", "vertices", "all"),
        help=(
            "Sample triangle centroids, vertices, or both. Defaults to "
            "centroid for --cut-surface and all points otherwise."
        ),
    )
    parser.add_argument(
        "--cut-surface", action="store_true",
        help=(
            "Compare only likely machined/cut surface triangles: below the "
            "stock top, above the stock bottom, and away from exterior XY "
            "walls. This keeps large flat uncut stock faces from dominating "
            "adaptive/render validation."
        ),
    )
    parser.add_argument(
        "--cut-z-epsilon", type=float, default=1e-9,
        help=(
            "Z margin used by --cut-surface to reject stock top and bottom "
            "triangles. Use a value tied to resolution, for example half a "
            "voxel."
        ),
    )
    parser.add_argument(
        "--cut-xy-epsilon", type=float, default=1e-9,
        help=(
            "XY margin used by --cut-surface to reject exterior side-wall "
            "triangles. Use a value tied to resolution."
        ),
    )
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    baseline, baseline_load = load_triangles(
        args.baseline, args.cut_surface, args.cut_z_epsilon,
        args.cut_xy_epsilon)
    candidate, candidate_load = load_triangles(
        args.candidate, args.cut_surface, args.cut_z_epsilon,
        args.cut_xy_epsilon)
    point_mode = args.point_mode or ("centroid" if args.cut_surface else "all")
    baseline_grid = build_grid(baseline, args.cell_size)
    candidate_grid = build_grid(candidate, args.cell_size)
    tolerance = args.hard_max_error

    forward = summarize_distances(
        "baseline_to_candidate", baseline, candidate_grid, args.max_samples,
        args.max_shells, tolerance, point_mode)
    reverse = summarize_distances(
        "candidate_to_baseline", candidate, baseline_grid, args.max_samples,
        args.max_shells, tolerance, point_mode)

    failures = []
    for summary in (forward, reverse):
        if args.hard_max_error is not None and summary["max"] > args.hard_max_error:
            failures.append(
                f"{summary['name']} max {summary['max']} > {args.hard_max_error}"
            )
        if args.p99_error is not None and summary["p99"] > args.p99_error:
            failures.append(
                f"{summary['name']} p99 {summary['p99']} > {args.p99_error}"
            )

    result = {
        "equal_within_tolerance": not failures,
        "failures": failures,
        "baseline_triangles": len(baseline),
        "candidate_triangles": len(candidate),
        "baseline_load": baseline_load,
        "candidate_load": candidate_load,
        "point_mode": point_mode,
        "cell_size": candidate_grid["cell_size"],
        "forward": forward,
        "reverse": reverse,
    }

    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        for summary in (forward, reverse):
            print(
                f"{summary['name']}: samples={summary['samples']} "
                f"max={summary['max']:.9g} p99={summary['p99']:.9g} "
                f"p95={summary['p95']:.9g} rms={summary['rms']:.9g} "
                f"over_tolerance={summary['over_tolerance']}"
            )
        if failures:
            for failure in failures:
                print(failure)
        else:
            print("STL sampled distance is within tolerance")

    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
