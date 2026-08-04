#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Compare machined STL cut surfaces as a streaming XY heightfield.

This helper is intended for very large 2.5D/relief meshes where full
nearest-triangle bidirectional distance is too memory-heavy. It ignores the
uncut stock top, stock bottom, and exterior stock side walls, then compares
the remaining cut surface by XY cells.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
import sys
from pathlib import Path

from compare_stl_geometry import iter_stl


def centroid(tri):
    return (
        (tri[0][0] + tri[1][0] + tri[2][0]) / 3.0,
        (tri[0][1] + tri[1][1] + tri[2][1]) / 3.0,
        (tri[0][2] + tri[1][2] + tri[2][2]) / 3.0,
    )


def percentile(values, pct):
    if not values:
        return 0.0
    index = (len(values) - 1) * pct / 100.0
    lo = math.floor(index)
    hi = math.ceil(index)
    if lo == hi:
        return values[int(index)]
    return values[lo] * (hi - index) + values[hi] * (index - lo)


def bounds_for_path(path):
    mins = [math.inf, math.inf, math.inf]
    maxs = [-math.inf, -math.inf, -math.inf]
    triangles = 0

    for tri in iter_stl(path):
        triangles += 1
        for v in tri:
            for i in range(3):
                mins[i] = min(mins[i], v[i])
                maxs[i] = max(maxs[i], v[i])

    if triangles == 0:
        raise ValueError(f"No triangles found in {path}")

    return {
        "path": str(path),
        "triangles": triangles,
        "bounds_min": mins,
        "bounds_max": maxs,
    }


def is_cut_point(point, bounds, z_epsilon, xy_epsilon):
    lo = bounds["bounds_min"]
    hi = bounds["bounds_max"]

    if point[2] >= hi[2] - z_epsilon:
        return False
    if point[2] <= lo[2] + z_epsilon:
        return False
    if point[0] <= lo[0] + xy_epsilon or point[0] >= hi[0] - xy_epsilon:
        return False
    if point[1] <= lo[1] + xy_epsilon or point[1] >= hi[1] - xy_epsilon:
        return False

    return True


def iter_points(tri, mode):
    if mode == "centroid":
        yield centroid(tri)
        return

    if mode == "vertices":
        yield from tri
        return

    yield from tri
    yield centroid(tri)


def add_sample(field, key, z):
    entry = field.get(key)
    if entry is None:
        field[key] = [z, z, z, 1]
        return

    entry[0] = min(entry[0], z)
    entry[1] = max(entry[1], z)
    entry[2] += z
    entry[3] += 1


def build_heightfield(path, bounds, origin, cell_size, z_epsilon, xy_epsilon,
                      point_mode):
    field = {}
    cut_points = 0

    for tri in iter_stl(path):
        for point in iter_points(tri, point_mode):
            if not is_cut_point(point, bounds, z_epsilon, xy_epsilon):
                continue

            key = (
                math.floor((point[0] - origin[0]) / cell_size + 1e-9),
                math.floor((point[1] - origin[1]) / cell_size + 1e-9),
            )
            add_sample(field, key, point[2])
            cut_points += 1

    if not field:
        raise ValueError(f"No cut-surface samples found in {path}")

    return field, {
        "cut_points": cut_points,
        "cut_cells": len(field),
    }


def compare_fields(baseline, candidate, origin, cell_size, worst_cells):
    baseline_keys = set(baseline)
    candidate_keys = set(candidate)
    common = baseline_keys & candidate_keys
    baseline_only = baseline_keys - candidate_keys
    candidate_only = candidate_keys - baseline_keys

    min_deltas = []
    max_deltas = []
    mean_deltas = []
    worst = []
    worst_sequence = 0

    def push_worst(value, metric, key, baseline_z, candidate_z):
        nonlocal worst_sequence
        if worst_cells <= 0:
            return
        worst_sequence += 1
        center = (
            origin[0] + (key[0] + 0.5) * cell_size,
            origin[1] + (key[1] + 0.5) * cell_size,
        )
        item = (value, worst_sequence, {
            "metric": metric,
            "cell": key,
            "xy_center": center,
            "baseline_z": baseline_z,
            "candidate_z": candidate_z,
            "abs_delta": value,
        })
        if len(worst) < worst_cells:
            heapq.heappush(worst, item)
        elif value > worst[0][0]:
            heapq.heapreplace(worst, item)

    for key in common:
        b = baseline[key]
        c = candidate[key]
        min_delta = abs(b[0] - c[0])
        max_delta = abs(b[1] - c[1])
        b_mean = b[2] / b[3]
        c_mean = c[2] / c[3]
        mean_delta = abs(b_mean - c_mean)
        min_deltas.append(min_delta)
        max_deltas.append(max_delta)
        mean_deltas.append(mean_delta)
        push_worst(min_delta, "min_z", key, b[0], c[0])
        push_worst(max_delta, "max_z", key, b[1], c[1])
        push_worst(mean_delta, "mean_z", key, b_mean, c_mean)

    min_deltas.sort()
    max_deltas.sort()
    mean_deltas.sort()
    all_deltas = sorted(min_deltas + max_deltas + mean_deltas)

    return {
        "baseline_cells": len(baseline),
        "candidate_cells": len(candidate),
        "common_cells": len(common),
        "baseline_only_cells": len(baseline_only),
        "candidate_only_cells": len(candidate_only),
        "baseline_only_ratio": len(baseline_only) / max(len(baseline), 1),
        "candidate_only_ratio": len(candidate_only) / max(len(candidate), 1),
        "max_z_delta": all_deltas[-1] if all_deltas else 0.0,
        "p99_z_delta": percentile(all_deltas, 99),
        "p95_z_delta": percentile(all_deltas, 95),
        "rms_z_delta": (
            math.sqrt(sum(d * d for d in all_deltas) / len(all_deltas))
            if all_deltas else 0.0
        ),
        "min_z": {
            "max": min_deltas[-1] if min_deltas else 0.0,
            "p99": percentile(min_deltas, 99),
            "p95": percentile(min_deltas, 95),
        },
        "max_z": {
            "max": max_deltas[-1] if max_deltas else 0.0,
            "p99": percentile(max_deltas, 99),
            "p95": percentile(max_deltas, 95),
        },
        "mean_z": {
            "max": mean_deltas[-1] if mean_deltas else 0.0,
            "p99": percentile(mean_deltas, 99),
            "p95": percentile(mean_deltas, 95),
        },
        "worst_cells": [
            item for _, _, item in sorted(worst, key=lambda x: x[0], reverse=True)
        ],
    }


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--cell-size", type=float, required=True)
    parser.add_argument("--cut-z-epsilon", type=float, default=1e-9)
    parser.add_argument("--cut-xy-epsilon", type=float, default=1e-9)
    parser.add_argument(
        "--point-mode", choices=("centroid", "vertices", "all"),
        default="centroid",
    )
    parser.add_argument("--hard-max-error", type=float)
    parser.add_argument("--p99-error", type=float)
    parser.add_argument("--max-missing-cell-ratio", type=float, default=0.01)
    parser.add_argument("--worst-cells", type=int, default=10)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    baseline_bounds = bounds_for_path(args.baseline)
    candidate_bounds = bounds_for_path(args.candidate)
    origin = (
        min(baseline_bounds["bounds_min"][0], candidate_bounds["bounds_min"][0]),
        min(baseline_bounds["bounds_min"][1], candidate_bounds["bounds_min"][1]),
    )

    baseline, baseline_samples = build_heightfield(
        args.baseline, baseline_bounds, origin, args.cell_size,
        args.cut_z_epsilon, args.cut_xy_epsilon, args.point_mode)
    candidate, candidate_samples = build_heightfield(
        args.candidate, candidate_bounds, origin, args.cell_size,
        args.cut_z_epsilon, args.cut_xy_epsilon, args.point_mode)
    comparison = compare_fields(
        baseline, candidate, origin, args.cell_size, args.worst_cells)

    failures = []
    if (args.hard_max_error is not None and
            comparison["max_z_delta"] > args.hard_max_error):
        failures.append(
            f"max_z_delta {comparison['max_z_delta']} > {args.hard_max_error}"
        )
    if args.p99_error is not None and comparison["p99_z_delta"] > args.p99_error:
        failures.append(
            f"p99_z_delta {comparison['p99_z_delta']} > {args.p99_error}"
        )
    for key in ("baseline_only_ratio", "candidate_only_ratio"):
        if comparison[key] > args.max_missing_cell_ratio:
            failures.append(
                f"{key} {comparison[key]} > {args.max_missing_cell_ratio}"
            )

    result = {
        "equal_within_tolerance": not failures,
        "failures": failures,
        "baseline_bounds": baseline_bounds,
        "candidate_bounds": candidate_bounds,
        "baseline_samples": baseline_samples,
        "candidate_samples": candidate_samples,
        "cell_size": args.cell_size,
        "cut_z_epsilon": args.cut_z_epsilon,
        "cut_xy_epsilon": args.cut_xy_epsilon,
        "point_mode": args.point_mode,
        "comparison": comparison,
    }

    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print(
            "cut heightfield: "
            f"common={comparison['common_cells']} "
            f"baseline_only={comparison['baseline_only_cells']} "
            f"candidate_only={comparison['candidate_only_cells']} "
            f"max_z_delta={comparison['max_z_delta']:.9g} "
            f"p99_z_delta={comparison['p99_z_delta']:.9g} "
            f"p95_z_delta={comparison['p95_z_delta']:.9g} "
            f"rms_z_delta={comparison['rms_z_delta']:.9g}"
        )
        if failures:
            for failure in failures:
                print(failure)
        else:
            print("STL cut heightfield is within tolerance")

    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
