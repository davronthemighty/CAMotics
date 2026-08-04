#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Summarize STL triangle normal distribution for remeshing decisions."""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import time
from collections import Counter
from pathlib import Path


STL_HEADER_BYTES = 80
STL_COUNT_BYTES = 4
STL_RECORD_BYTES = 50


def read_triangle_count(path: Path) -> int:
    with path.open("rb") as f:
        f.seek(STL_HEADER_BYTES)
        return struct.unpack("<I", f.read(STL_COUNT_BYTES))[0]


def iter_binary_normals(path: Path):
    count = read_triangle_count(path)
    with path.open("rb") as f:
        f.seek(STL_HEADER_BYTES + STL_COUNT_BYTES)
        for i in range(count):
            record = f.read(STL_RECORD_BYTES)
            if len(record) != STL_RECORD_BYTES:
                raise ValueError(f"Unexpected EOF in {path} at triangle {i}")
            vals = struct.unpack("<12fH", record)
            yield vals[0:3]


def angle_from_abs_dot(abs_dot: float) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, abs_dot))))


def summarize(path: Path, progress: int):
    thresholds_degrees = [0.125, 0.25, 0.5, 1.0, 2.0, 5.0]
    axis_names = ["x", "y", "z"]
    axis_counts = {
        axis: {str(th): 0 for th in thresholds_degrees}
        for axis in axis_names
    }
    vertical_counts = {str(th): 0 for th in thresholds_degrees}
    horizontal_counts = {str(th): 0 for th in thresholds_degrees}
    dominant = Counter()
    quadrants = Counter()
    total = 0
    invalid = 0
    start = time.time()

    axis_cos = {th: math.cos(math.radians(th)) for th in thresholds_degrees}
    vertical_sin = {th: math.sin(math.radians(th)) for th in thresholds_degrees}

    for normal in iter_binary_normals(path):
        total += 1
        length = math.sqrt(sum(v * v for v in normal))
        if length == 0:
            invalid += 1
            continue

        n = tuple(v / length for v in normal)
        absn = tuple(abs(v) for v in n)
        axis = max(range(3), key=lambda i: absn[i])
        dominant[axis_names[axis]] += 1

        if axis in (0, 1):
            sign_x = 1 if n[0] >= 0 else -1
            sign_y = 1 if n[1] >= 0 else -1
            quadrants[f"x{sign_x}_y{sign_y}"] += 1

        for th in thresholds_degrees:
            key = str(th)
            for i, name in enumerate(axis_names):
                if absn[i] >= axis_cos[th]:
                    axis_counts[name][key] += 1
            if absn[2] <= vertical_sin[th]:
                vertical_counts[key] += 1
            if absn[2] >= axis_cos[th]:
                horizontal_counts[key] += 1

        if progress and total % progress == 0:
            print(
                f"scanned normals: {total:,} ({time.time() - start:.1f}s)",
                file=sys.stderr,
            )

    return {
        "path": str(path),
        "triangles": total,
        "invalid_normals": invalid,
        "dominant_axis": dict(dominant),
        "dominant_axis_ratio": {
            axis: count / total if total else 0.0
            for axis, count in dominant.items()
        },
        "near_axis_counts": axis_counts,
        "near_axis_ratios": {
            axis: {
                th: count / total if total else 0.0
                for th, count in counts.items()
            }
            for axis, counts in axis_counts.items()
        },
        "near_vertical_xy_plane_counts": vertical_counts,
        "near_vertical_xy_plane_ratios": {
            th: count / total if total else 0.0
            for th, count in vertical_counts.items()
        },
        "near_horizontal_counts": horizontal_counts,
        "near_horizontal_ratios": {
            th: count / total if total else 0.0
            for th, count in horizontal_counts.items()
        },
        "xy_dominant_quadrants": dict(quadrants),
        "elapsed_wall_seconds": time.time() - start,
    }


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize STL normal orientation buckets."
    )
    parser.add_argument("stl", type=Path)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--progress", type=int, default=1_000_000)
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    result = summarize(args.stl, args.progress)
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.output_json:
        args.output_json.write_text(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
