#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Compare cut surfaces by projecting triangle centroids onto target planes.

This validator is intended for reduced meshes.  It samples source triangle
centroids, finds a target triangle whose XY projection contains that centroid,
interpolates the target triangle's Z at the same XY point, and reports Z,
normal-distance, and normal-angle error.  It deliberately avoids shared STL
vertices as primary samples because a shared vertex has no unique face normal.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import sys
import tempfile
from collections import defaultdict
from pathlib import Path

from compare_stl_geometry import cross, dot, iter_stl, length, sub


def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def mul(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def centroid(tri):
    return mul(add(add(tri[0], tri[1]), tri[2]), 1.0 / 3.0)


def triangle_bounds(tri):
    return (
        tuple(min(v[i] for v in tri) for i in range(3)),
        tuple(max(v[i] for v in tri) for i in range(3)),
    )


def bounds_for_path(path):
    mins = [math.inf, math.inf, math.inf]
    maxs = [-math.inf, -math.inf, -math.inf]
    triangles = 0

    for tri in iter_stl(path):
        triangles += 1
        lo, hi = triangle_bounds(tri)
        for i in range(3):
            mins[i] = min(mins[i], lo[i])
            maxs[i] = max(maxs[i], hi[i])

    if triangles == 0:
        raise ValueError(f"No triangles found in {path}")

    return {
        "path": str(path),
        "triangles": triangles,
        "bounds_min": tuple(mins),
        "bounds_max": tuple(maxs),
    }


def unit_normal(tri):
    n = cross(sub(tri[1], tri[0]), sub(tri[2], tri[0]))
    l = length(n)
    if l == 0:
        return None
    return (n[0] / l, n[1] / l, n[2] / l)


def write_ascii_stl(path, triangles):
    with path.open("w", encoding="utf-8") as f:
        f.write("solid centroid_plane_self_test\n")
        for tri in triangles:
            normal = unit_normal(tri) or (0.0, 0.0, 0.0)
            f.write(
                f"  facet normal {normal[0]:.17g} "
                f"{normal[1]:.17g} {normal[2]:.17g}\n"
            )
            f.write("    outer loop\n")
            for vertex in tri:
                f.write(
                    f"      vertex {vertex[0]:.17g} "
                    f"{vertex[1]:.17g} {vertex[2]:.17g}\n"
                )
            f.write("    endloop\n")
            f.write("  endfacet\n")
        f.write("endsolid centroid_plane_self_test\n")


def rect_triangles(x0, x1, y0, y1, z_fn):
    p00 = (x0, y0, z_fn(x0, y0))
    p10 = (x1, y0, z_fn(x1, y0))
    p11 = (x1, y1, z_fn(x1, y1))
    p01 = (x0, y1, z_fn(x0, y1))
    return [(p00, p10, p11), (p00, p11, p01)]


def vertical_wall_triangles(x, y0, y1, z0, z1):
    p00 = (x, y0, z0)
    p10 = (x, y1, z0)
    p11 = (x, y1, z1)
    p01 = (x, y0, z1)
    return [(p00, p10, p11), (p00, p11, p01)]


def cut_fixture(cut_rectangles):
    triangles = []
    triangles.extend(rect_triangles(0.0, 10.0, 0.0, 10.0, lambda _x, _y: 1.0))
    triangles.extend(rect_triangles(0.0, 10.0, 0.0, 10.0, lambda _x, _y: -1.0))
    for x0, x1, y0, y1, z_fn in cut_rectangles:
        triangles.extend(rect_triangles(x0, x1, y0, y1, z_fn))
    return triangles


def flip_triangles(triangles):
    return [(tri[0], tri[2], tri[1]) for tri in triangles]


def is_cut_centroid(point, bounds, z_epsilon, xy_epsilon):
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


def cell_coord_xy(point, origin, cell_size):
    return (
        math.floor((point[0] - origin[0]) / cell_size),
        math.floor((point[1] - origin[1]) / cell_size),
    )


def xy_bounds(tri):
    return (
        (min(v[0] for v in tri), min(v[1] for v in tri)),
        (max(v[0] for v in tri), max(v[1] for v in tri)),
    )


def barycentric_xy(point, tri, eps):
    x, y = point[0], point[1]
    x0, y0 = tri[0][0], tri[0][1]
    x1, y1 = tri[1][0], tri[1][1]
    x2, y2 = tri[2][0], tri[2][1]
    denom = ((y1 - y2) * (x0 - x2) +
             (x2 - x1) * (y0 - y2))
    if abs(denom) <= eps:
        return None

    w0 = ((y1 - y2) * (x - x2) +
          (x2 - x1) * (y - y2)) / denom
    w1 = ((y2 - y0) * (x - x2) +
          (x0 - x2) * (y - y2)) / denom
    w2 = 1 - w0 - w1
    if w0 < -eps or w1 < -eps or w2 < -eps:
        return None
    if 1 + eps < w0 or 1 + eps < w1 or 1 + eps < w2:
        return None
    return w0, w1, w2


def interpolate_z(weights, tri):
    return weights[0] * tri[0][2] + weights[1] * tri[1][2] + weights[2] * tri[2][2]


def percentile(values, pct):
    if not values:
        return 0.0
    index = (len(values) - 1) * pct / 100.0
    lo = math.floor(index)
    hi = math.ceil(index)
    if lo == hi:
        return values[int(index)]
    return values[lo] * (hi - index) + values[hi] * (index - lo)


def summarize(values):
    values = sorted(values)
    if not values:
        return {
            "max": 0.0,
            "p99": 0.0,
            "p95": 0.0,
            "rms": 0.0,
        }
    return {
        "max": values[-1],
        "p99": percentile(values, 99),
        "p95": percentile(values, 95),
        "rms": math.sqrt(sum(v * v for v in values) / len(values)),
    }


def ratio(part, total):
    return part / max(total, 1)


def build_target_grid(path, bounds, origin, cell_size, z_epsilon, xy_epsilon,
                      min_abs_normal_z, barycentric_epsilon):
    triangles = []
    grid = defaultdict(list)
    total = 0
    cut = 0
    skipped_normal = 0
    skipped_degenerate = 0
    refs = 0

    for tri in iter_stl(path):
        total += 1
        c = centroid(tri)
        if not is_cut_centroid(c, bounds, z_epsilon, xy_epsilon):
            continue
        cut += 1
        n = unit_normal(tri)
        if n is None or abs(n[2]) < min_abs_normal_z:
            skipped_normal += 1
            continue
        lo, hi = xy_bounds(tri)
        if abs((hi[0] - lo[0]) * (hi[1] - lo[1])) <= barycentric_epsilon:
            skipped_degenerate += 1
            continue

        index = len(triangles)
        triangles.append((tri, n))
        c0 = cell_coord_xy((lo[0], lo[1], 0), origin, cell_size)
        c1 = cell_coord_xy((hi[0], hi[1], 0), origin, cell_size)
        for y in range(c0[1], c1[1] + 1):
            for x in range(c0[0], c1[0] + 1):
                grid[(x, y)].append(index)
                refs += 1

    if not triangles:
        raise ValueError(f"No usable cut-surface target triangles in {path}")

    return {
        "path": str(path),
        "bounds": bounds,
        "origin": origin,
        "cell_size": cell_size,
        "triangles": triangles,
        "grid": grid,
        "summary": {
            "total_triangles": total,
            "cut_triangles": cut,
            "indexed_triangles": len(triangles),
            "grid_cells": len(grid),
            "grid_refs": refs,
            "skipped_normal_or_empty": skipped_normal,
            "skipped_normal_or_empty_ratio": ratio(skipped_normal, cut),
            "skipped_xy_degenerate": skipped_degenerate,
        },
    }


def count_source_samples(path, bounds, z_epsilon, xy_epsilon, min_abs_normal_z):
    cut = 0
    usable = 0
    skipped_normal = 0
    for tri in iter_stl(path):
        c = centroid(tri)
        if not is_cut_centroid(c, bounds, z_epsilon, xy_epsilon):
            continue
        cut += 1
        n = unit_normal(tri)
        if n is None or abs(n[2]) < min_abs_normal_z:
            skipped_normal += 1
            continue
        usable += 1
    return cut, usable, skipped_normal


def find_target_sample(point, source_z, target, search_radius,
                       barycentric_epsilon):
    grid = target["grid"]
    center = cell_coord_xy(point, target["origin"], target["cell_size"])
    best = None
    seen = set()

    for radius in range(search_radius + 1):
        for y in range(center[1] - radius, center[1] + radius + 1):
            for x in range(center[0] - radius, center[0] + radius + 1):
                if radius and max(abs(x - center[0]), abs(y - center[1])) != radius:
                    continue
                for index in grid.get((x, y), ()):
                    if index in seen:
                        continue
                    seen.add(index)
                    tri, normal = target["triangles"][index]
                    weights = barycentric_xy(point, tri, barycentric_epsilon)
                    if weights is None:
                        continue
                    z = interpolate_z(weights, tri)
                    error = abs(source_z - z)
                    if best is None or error < best["abs_z_delta"]:
                        best = {
                            "z": z,
                            "normal": normal,
                            "abs_z_delta": error,
                        }
        if best is not None:
            return best

    return None


def compare_direction(name, source_path, source_bounds, target, args):
    cut, usable, skipped_normal = count_source_samples(
        source_path, source_bounds, args.cut_z_epsilon, args.cut_xy_epsilon,
        args.min_abs_normal_z)
    stride = max(1, math.ceil(usable / args.max_samples)) if args.max_samples else 1

    z_errors = []
    normal_distance_errors = []
    normal_angles = []
    worst = []
    unmatched_samples = []
    unmatched = 0
    sampled = 0
    usable_seen = 0

    def push_worst(entry):
        worst.append(entry)
        worst.sort(key=lambda e: e["abs_z_delta"], reverse=True)
        del worst[args.worst_samples:]

    for tri in iter_stl(source_path):
        point = centroid(tri)
        if not is_cut_centroid(point, source_bounds, args.cut_z_epsilon,
                              args.cut_xy_epsilon):
            continue
        source_normal = unit_normal(tri)
        if source_normal is None or abs(source_normal[2]) < args.min_abs_normal_z:
            continue
        usable_seen += 1
        if (usable_seen - 1) % stride:
            continue
        if args.max_samples and args.max_samples <= sampled:
            break

        sampled += 1
        target_sample = find_target_sample(
            point, point[2], target, args.search_radius,
            args.barycentric_epsilon)
        if target_sample is None:
            unmatched += 1
            if len(unmatched_samples) < args.unmatched_samples:
                unmatched_samples.append({
                    "source_xyz": point,
                    "source_normal": source_normal,
                })
            continue

        dz = target_sample["z"] - point[2]
        abs_z = abs(dz)
        z_errors.append(abs_z)
        normal_distance_errors.append(abs(dz * source_normal[2]))
        dot_value = dot(source_normal, target_sample["normal"])
        if not args.oriented_normals:
            dot_value = abs(dot_value)
        dot_normals = max(-1.0, min(1.0, dot_value))
        angle = math.degrees(math.acos(dot_normals))
        normal_angles.append(angle)
        push_worst({
            "source_xyz": point,
            "source_z": point[2],
            "target_z": target_sample["z"],
            "dz": dz,
            "abs_z_delta": abs_z,
            "normal_distance": abs(dz * source_normal[2]),
            "normal_angle_degrees": angle,
        })

    return {
        "name": name,
        "source_cut_triangles": cut,
        "source_usable_triangles": usable,
        "source_skipped_normal_or_empty": skipped_normal,
        "source_skipped_normal_or_empty_ratio": ratio(skipped_normal, cut),
        "stride": stride,
        "samples": sampled,
        "matched": len(z_errors),
        "unmatched": unmatched,
        "unmatched_ratio": unmatched / max(sampled, 1),
        "unmatched_samples": unmatched_samples,
        "abs_z_delta": summarize(z_errors),
        "normal_distance": summarize(normal_distance_errors),
        "normal_angle_degrees": summarize(normal_angles),
        "worst_samples": worst,
    }


def run_self_test():
    with tempfile.TemporaryDirectory(prefix="camotics-centroid-plane-") as tmp:
        root = Path(tmp)
        baseline = root / "baseline.stl"
        identical = root / "identical.stl"
        shifted = root / "shifted.stl"
        missing = root / "missing.stl"
        tilted = root / "tilted.stl"
        flipped = root / "flipped.stl"
        steep = root / "steep.stl"

        baseline_triangles = cut_fixture([
            (2.0, 8.0, 2.0, 8.0, lambda _x, _y: 0.0),
        ])

        write_ascii_stl(baseline, baseline_triangles)
        write_ascii_stl(
            identical,
            cut_fixture([
                (2.0, 8.0, 2.0, 8.0, lambda _x, _y: 0.0),
            ]),
        )
        write_ascii_stl(
            shifted,
            cut_fixture([
                (2.0, 8.0, 2.0, 8.0, lambda _x, _y: 0.02),
            ]),
        )
        write_ascii_stl(
            missing,
            cut_fixture([
                (2.0, 5.0, 2.0, 8.0, lambda _x, _y: 0.0),
            ]),
        )
        write_ascii_stl(
            tilted,
            cut_fixture([
                (2.0, 8.0, 2.0, 8.0, lambda x, _y: 0.02 * (x - 5.0)),
            ]),
        )
        write_ascii_stl(flipped, flip_triangles(baseline_triangles))
        steep_triangles = baseline_triangles + vertical_wall_triangles(
            5.0, 2.0, 8.0, -0.5, 0.5)
        write_ascii_stl(steep, steep_triangles)

        common = [
            "--cell-size", "1.0",
            "--cut-z-epsilon", "0.1",
            "--cut-xy-epsilon", "0.1",
            "--max-unmatched-ratio", "0.1",
            "--max-samples", "100",
            "--bidirectional",
            "--json",
        ]

        cases = [
            (
                "identical",
                identical,
                ["--hard-max-error", "0.001", "--p99-error", "0.001",
                 "--max-normal-angle", "0.1",
                 "--p99-normal-angle", "0.1"],
                True,
            ),
            (
                "shifted",
                shifted,
                ["--hard-max-error", "0.005"],
                False,
            ),
            (
                "missing",
                missing,
                ["--hard-max-error", "0.001", "--p99-error", "0.001"],
                False,
            ),
            (
                "tilted",
                tilted,
                ["--max-normal-angle", "0.5"],
                False,
            ),
            (
                "flipped_unoriented",
                flipped,
                ["--hard-max-error", "0.001", "--p99-error", "0.001",
                 "--max-normal-angle", "0.1"],
                True,
            ),
            (
                "flipped_oriented",
                flipped,
                ["--hard-max-error", "0.001", "--p99-error", "0.001",
                 "--max-normal-angle", "0.1", "--oriented-normals"],
                False,
            ),
            (
                "steep_without_skip_gate",
                steep,
                ["--hard-max-error", "0.001", "--p99-error", "0.001"],
                True,
            ),
            (
                "steep_with_skip_gate",
                steep,
                ["--hard-max-error", "0.001", "--p99-error", "0.001",
                 "--max-skipped-normal-ratio", "0.1"],
                False,
            ),
        ]

        failures = []
        for name, candidate, extra, expect_success in cases:
            with contextlib.redirect_stdout(io.StringIO()):
                rc = main([str(baseline), str(candidate), *common, *extra])
            if expect_success and rc:
                failures.append(f"{name}: expected success, got failure")
            elif not expect_success and rc == 0:
                failures.append(f"{name}: expected failure, got success")

        if failures:
            for failure in failures:
                print(failure)
            return 1

    print("STL centroid-plane self-test passed")
    return 0


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline", type=Path, nargs="?")
    parser.add_argument("candidate", type=Path, nargs="?")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--cell-size", type=float, default=0.25)
    parser.add_argument("--max-samples", type=int, default=200000)
    parser.add_argument("--search-radius", type=int, default=2)
    parser.add_argument("--cut-z-epsilon", type=float, default=1e-9)
    parser.add_argument("--cut-xy-epsilon", type=float, default=1e-9)
    parser.add_argument("--min-abs-normal-z", type=float, default=0.05)
    parser.add_argument("--barycentric-epsilon", type=float, default=1e-8)
    parser.add_argument("--hard-max-error", type=float)
    parser.add_argument("--p99-error", type=float)
    parser.add_argument("--max-unmatched-ratio", type=float, default=0.01)
    parser.add_argument("--max-skipped-normal-ratio", type=float,
                        help="Fail if skipped near-vertical cut triangles "
                             "exceed this ratio in any compared mesh")
    parser.add_argument("--max-normal-angle", type=float)
    parser.add_argument("--p99-normal-angle", type=float)
    parser.add_argument("--oriented-normals", action="store_true",
                        help="Treat opposite triangle normals as a mismatch")
    parser.add_argument("--bidirectional", action="store_true")
    parser.add_argument("--worst-samples", type=int, default=10)
    parser.add_argument("--unmatched-samples", type=int, default=10)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    if args.self_test:
        return run_self_test()

    if args.baseline is None or args.candidate is None:
        parser.error("baseline and candidate STL paths are required")

    baseline_bounds = bounds_for_path(args.baseline)
    candidate_bounds = bounds_for_path(args.candidate)
    origin = (
        min(baseline_bounds["bounds_min"][0], candidate_bounds["bounds_min"][0]),
        min(baseline_bounds["bounds_min"][1], candidate_bounds["bounds_min"][1]),
    )

    candidate_grid = build_target_grid(
        args.candidate, candidate_bounds, origin, args.cell_size,
        args.cut_z_epsilon, args.cut_xy_epsilon, args.min_abs_normal_z,
        args.barycentric_epsilon)
    directions = [
        compare_direction("baseline_to_candidate", args.baseline,
                          baseline_bounds, candidate_grid, args)
    ]

    baseline_grid_summary = None
    if args.bidirectional:
        baseline_grid = build_target_grid(
            args.baseline, baseline_bounds, origin, args.cell_size,
            args.cut_z_epsilon, args.cut_xy_epsilon, args.min_abs_normal_z,
            args.barycentric_epsilon)
        baseline_grid_summary = baseline_grid["summary"]
        directions.append(
            compare_direction("candidate_to_baseline", args.candidate,
                              candidate_bounds, baseline_grid, args))

    failures = []
    target_summaries = [("candidate_target", candidate_grid["summary"])]
    if baseline_grid_summary is not None:
        target_summaries.append(("baseline_target", baseline_grid_summary))

    if args.max_skipped_normal_ratio is not None:
        for name, summary in target_summaries:
            skipped_ratio = summary.get("skipped_normal_or_empty_ratio", 0.0)
            if skipped_ratio > args.max_skipped_normal_ratio:
                failures.append(
                    f"{name} skipped_normal_ratio {skipped_ratio} "
                    f"> {args.max_skipped_normal_ratio}"
                )

    for direction in directions:
        if (args.max_skipped_normal_ratio is not None and
                direction["source_skipped_normal_or_empty_ratio"] >
                args.max_skipped_normal_ratio):
            failures.append(
                f"{direction['name']} source skipped_normal_ratio "
                f"{direction['source_skipped_normal_or_empty_ratio']} "
                f"> {args.max_skipped_normal_ratio}"
            )
        if (args.hard_max_error is not None and
                direction["abs_z_delta"]["max"] > args.hard_max_error):
            failures.append(
                f"{direction['name']} max {direction['abs_z_delta']['max']} "
                f"> {args.hard_max_error}"
            )
        if (args.p99_error is not None and
                direction["abs_z_delta"]["p99"] > args.p99_error):
            failures.append(
                f"{direction['name']} p99 {direction['abs_z_delta']['p99']} "
                f"> {args.p99_error}"
            )
        if direction["unmatched_ratio"] > args.max_unmatched_ratio:
            failures.append(
                f"{direction['name']} unmatched_ratio "
                f"{direction['unmatched_ratio']} > {args.max_unmatched_ratio}"
            )
        if (args.max_normal_angle is not None and
                direction["normal_angle_degrees"]["max"] > args.max_normal_angle):
            failures.append(
                f"{direction['name']} normal max "
                f"{direction['normal_angle_degrees']['max']} "
                f"> {args.max_normal_angle}"
            )
        if (args.p99_normal_angle is not None and
                direction["normal_angle_degrees"]["p99"] > args.p99_normal_angle):
            failures.append(
                f"{direction['name']} normal p99 "
                f"{direction['normal_angle_degrees']['p99']} "
                f"> {args.p99_normal_angle}"
            )

    result = {
        "equal_within_tolerance": not failures,
        "failures": failures,
        "baseline_bounds": baseline_bounds,
        "candidate_bounds": candidate_bounds,
        "target_grid": candidate_grid["summary"],
        "baseline_grid": baseline_grid_summary,
        "settings": {
            "cell_size": args.cell_size,
            "max_samples": args.max_samples,
            "search_radius": args.search_radius,
            "cut_z_epsilon": args.cut_z_epsilon,
            "cut_xy_epsilon": args.cut_xy_epsilon,
            "min_abs_normal_z": args.min_abs_normal_z,
            "barycentric_epsilon": args.barycentric_epsilon,
            "oriented_normals": args.oriented_normals,
            "max_skipped_normal_ratio": args.max_skipped_normal_ratio,
            "bidirectional": args.bidirectional,
        },
        "directions": directions,
    }

    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        for direction in directions:
            z = direction["abs_z_delta"]
            angle = direction["normal_angle_degrees"]
            print(
                f"{direction['name']}: samples={direction['samples']} "
                f"matched={direction['matched']} unmatched={direction['unmatched']} "
                f"skipped_normal_ratio="
                f"{direction['source_skipped_normal_or_empty_ratio']:.9g} "
                f"max_z={z['max']:.9g} p99_z={z['p99']:.9g} "
                f"p95_z={z['p95']:.9g} rms_z={z['rms']:.9g} "
                f"max_angle={angle['max']:.9g} "
                f"p99_angle={angle['p99']:.9g}"
            )
        if failures:
            for failure in failures:
                print(failure)
        else:
            print("STL centroid plane comparison is within tolerance")

    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
