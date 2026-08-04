#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Compare STL geometry with order-independent quantized triangle hashes.

This is intended for performance experiments that should preserve geometry,
such as thread-count or render-job scheduling changes. It is not a quality
metric for lossy mesh reduction.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
import sys
from pathlib import Path


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def length(v):
    return math.sqrt(dot(v, v))


def iter_binary_stl(path: Path, count: int):
    with path.open("rb") as f:
        f.seek(84)
        for _ in range(count):
            chunk = f.read(50)
            if len(chunk) != 50:
                raise ValueError(f"Unexpected EOF in binary STL: {path}")

            vals = struct.unpack("<12fH", chunk)
            yield (vals[3:6], vals[6:9], vals[9:12])


def iter_ascii_stl(path: Path):
    vertices = []

    with path.open("rt", encoding="utf-8", errors="replace") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 4 and parts[0].lower() == "vertex":
                vertices.append(tuple(float(x) for x in parts[1:4]))
                if len(vertices) == 3:
                    yield tuple(vertices)
                    vertices = []

    if vertices:
        raise ValueError(f"Incomplete ASCII STL facet in {path}")


def is_binary_stl(path: Path):
    size = path.stat().st_size
    if size < 84:
        return False, 0

    with path.open("rb") as f:
        f.seek(80)
        count = struct.unpack("<I", f.read(4))[0]

    return size == 84 + 50 * count, count


def iter_stl(path: Path):
    binary, count = is_binary_stl(path)
    if binary:
        yield from iter_binary_stl(path, count)
    else:
        yield from iter_ascii_stl(path)


def quantize_vertex(v, tolerance):
    return tuple(int(round(x / tolerance)) for x in v)


def triangle_digest(triangle, tolerance):
    q = sorted(quantize_vertex(v, tolerance) for v in triangle)
    h = hashlib.sha256()
    for vertex in q:
        h.update(struct.pack("<3q", *vertex))
    return h.digest()


def summarize(path: Path, tolerance: float, hash_triangles: bool):
    count = 0
    bounds_min = [math.inf, math.inf, math.inf]
    bounds_max = [-math.inf, -math.inf, -math.inf]
    area = 0.0
    signed_volume = 0.0
    digests = []

    for tri in iter_stl(path):
        count += 1

        for v in tri:
            for i in range(3):
                bounds_min[i] = min(bounds_min[i], v[i])
                bounds_max[i] = max(bounds_max[i], v[i])

        a, b, c = tri
        normal = cross(sub(b, a), sub(c, a))
        area += 0.5 * length(normal)
        signed_volume += dot(a, cross(b, c)) / 6.0

        if hash_triangles:
            digests.append(triangle_digest(tri, tolerance))

    geometry_hash = None
    if hash_triangles:
        h = hashlib.sha256()
        for digest in sorted(digests):
            h.update(digest)
        geometry_hash = h.hexdigest()

    if count == 0:
        bounds_min = [0.0, 0.0, 0.0]
        bounds_max = [0.0, 0.0, 0.0]

    return {
        "path": str(path),
        "triangles": count,
        "bounds_min": bounds_min,
        "bounds_max": bounds_max,
        "surface_area": area,
        "signed_volume": signed_volume,
        "geometry_hash": geometry_hash,
    }


def close(a, b, abs_tol, rel_tol):
    return abs(a - b) <= max(abs_tol, rel_tol * max(abs(a), abs(b), 1.0))


def compare(a, b, abs_tol, rel_tol, hash_triangles):
    failures = []

    if a["triangles"] != b["triangles"]:
        failures.append(
            f"triangle count differs: {a['triangles']} != {b['triangles']}"
        )

    for key in ("bounds_min", "bounds_max"):
        for i, axis in enumerate("xyz"):
            if not close(a[key][i], b[key][i], abs_tol, rel_tol):
                failures.append(
                    f"{key}.{axis} differs: {a[key][i]} != {b[key][i]}"
                )

    for key in ("surface_area", "signed_volume"):
        if not close(a[key], b[key], abs_tol, rel_tol):
            failures.append(f"{key} differs: {a[key]} != {b[key]}")

    if hash_triangles and a["geometry_hash"] != b["geometry_hash"]:
        failures.append("quantized geometry hash differs")

    return failures


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("left", type=Path)
    parser.add_argument("right", type=Path)
    parser.add_argument(
        "--tolerance", type=float, default=1e-6,
        help="Coordinate quantization and absolute comparison tolerance.",
    )
    parser.add_argument(
        "--relative-tolerance", type=float, default=1e-9,
        help="Relative tolerance for scalar comparisons.",
    )
    parser.add_argument(
        "--skip-hash", action="store_true",
        help="Skip order-independent triangle hashing for very large meshes.",
    )
    parser.add_argument(
        "--json", action="store_true",
        help="Print full JSON summaries.",
    )
    args = parser.parse_args(argv)

    hash_triangles = not args.skip_hash
    left = summarize(args.left, args.tolerance, hash_triangles)
    right = summarize(args.right, args.tolerance, hash_triangles)
    failures = compare(
        left, right, args.tolerance, args.relative_tolerance, hash_triangles
    )

    result = {"equal": not failures, "failures": failures,
              "left": left, "right": right}

    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    elif failures:
        for failure in failures:
            print(failure)
    else:
        print("STL geometry matches")

    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
