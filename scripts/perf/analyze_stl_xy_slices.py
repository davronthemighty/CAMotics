#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later

"""Measure horizontal-slice direction quantization in binary STL surfaces."""

import argparse
import json
import math
import struct


TRIANGLE = struct.Struct("<12fH")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("stl", nargs="+")
    parser.add_argument("--slice", type=float, action="append", required=True)
    parser.add_argument("--angle-tolerance", type=float, default=0.5)
    parser.add_argument("--dedup", type=float, default=1e-5)
    return parser.parse_args()


def angle_distance(angle, target):
    delta = abs(angle - target) % 180
    return min(delta, 180 - delta)


def endpoint_key(point, scale):
    return tuple(round(value / scale) for value in point)


def segment_key(a, b, scale):
    ka = endpoint_key(a, scale)
    kb = endpoint_key(b, scale)
    return (ka, kb) if ka <= kb else (kb, ka)


def intersect_triangle(vertices, z):
    points = []
    for index in range(3):
        a = vertices[index]
        b = vertices[(index + 1) % 3]
        da = a[2] - z
        db = b[2] - z
        if (da < 0) == (db < 0):
            continue
        denominator = b[2] - a[2]
        if not denominator:
            continue
        ratio = (z - a[2]) / denominator
        if ratio < 0 or 1 < ratio:
            continue
        point = (a[0] + (b[0] - a[0]) * ratio,
                 a[1] + (b[1] - a[1]) * ratio)
        if not any(math.dist(point, prior) < 1e-10 for prior in points):
            points.append(point)
    return points[:2] if len(points) >= 2 else ()


def analyze(path, slices, tolerance, dedup):
    segments = {z: {} for z in slices}
    bounds = [math.inf, math.inf, -math.inf, -math.inf]
    with open(path, "rb") as stream:
        stream.seek(80)
        triangle_count = struct.unpack("<I", stream.read(4))[0]
        for _ in range(triangle_count):
            data = stream.read(TRIANGLE.size)
            if len(data) != TRIANGLE.size:
                raise RuntimeError(f"{path}: truncated binary STL")
            values = TRIANGLE.unpack(data)
            vertices = (values[3:6], values[6:9], values[9:12])
            for vertex in vertices:
                bounds[0] = min(bounds[0], vertex[0])
                bounds[1] = min(bounds[1], vertex[1])
                bounds[2] = max(bounds[2], vertex[0])
                bounds[3] = max(bounds[3], vertex[1])
            min_z = min(vertex[2] for vertex in vertices)
            max_z = max(vertex[2] for vertex in vertices)
            for z in slices:
                if z <= min_z or max_z <= z:
                    continue
                points = intersect_triangle(vertices, z)
                if len(points) != 2:
                    continue
                a, b = points
                if math.dist(a, b) <= dedup:
                    continue
                segments[z][segment_key(a, b, dedup)] = (a, b)

    results = []
    stock_margin = 2 * dedup
    for z in slices:
        total = 0.0
        axis = 0.0
        diagonal = 0.0
        bins = set()
        retained = 0
        for a, b in segments[z].values():
            midpoint = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
            if (midpoint[0] <= bounds[0] + stock_margin or
                bounds[2] - stock_margin <= midpoint[0] or
                midpoint[1] <= bounds[1] + stock_margin or
                bounds[3] - stock_margin <= midpoint[1]):
                continue
            dx = b[0] - a[0]
            dy = b[1] - a[1]
            length = math.hypot(dx, dy)
            if not length:
                continue
            angle = math.degrees(math.atan2(dy, dx)) % 180
            total += length
            retained += 1
            bins.add(round(angle * 10) % 1800)
            if min(angle_distance(angle, 0),
                   angle_distance(angle, 90)) <= tolerance:
                axis += length
            if min(angle_distance(angle, 45),
                   angle_distance(angle, 135)) <= tolerance:
                diagonal += length
        results.append({
            "z": z,
            "segments": retained,
            "length_mm": total,
            "axis_percent": 100 * axis / total if total else 0,
            "diagonal_percent": 100 * diagonal / total if total else 0,
            "combined_percent": 100 * (axis + diagonal) / total
            if total else 0,
            "direction_bins_0p1deg": len(bins),
        })
    return {"path": path, "xy_bounds": bounds, "slices": results}


def main():
    args = parse_args()
    report = [
        analyze(path, args.slice, args.angle_tolerance, args.dedup)
        for path in args.stl
    ]
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
