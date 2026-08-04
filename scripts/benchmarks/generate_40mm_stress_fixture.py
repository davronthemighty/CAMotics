#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later

"""Generate the public 40 mm CAMotics stress fixture."""

import argparse
import json
import math
from pathlib import Path


def point(radius, angle):
    return radius * math.cos(angle), radius * math.sin(angle)


def move(x, y, z=None, rapid=False):
    code = "G0" if rapid else "G1"
    parts = [code, f"X{x:.6f}", f"Y{y:.6f}"]
    if z is not None:
        parts.append(f"Z{z:.6f}")
    return " ".join(parts)


def generate_gcode(segments, rings, spokes, hatch_lines, hatch_points):
    lines = [
        "(CAMotics Fast public 40 mm stress fixture)",
        "G21 G90 G94",
        "T1 M6",
        "F900",
        "G0 Z1.000000",
    ]

    # Fine concentric relief.  The depth modulation exercises the conical
    # flank along both shallow and sloped moves.
    for ring in range(rings):
        radius = 2.0 + 16.2 * ring / max(1, rings - 1)
        x, y = point(radius, 0)
        depth = -0.10 - 0.42 * (ring % 7) / 6.0
        lines.extend((move(x, y, 1, True), move(x, y, depth)))
        for step in range(1, segments + 1):
            angle = 2 * math.pi * step / segments
            x, y = point(radius, angle)
            z = depth - 0.035 * math.sin(11 * angle + ring * 0.37)
            lines.append(move(x, y, z))
        lines.append("G0 Z1.000000")

    # Radial cuts cross the circular work and create many short-path bins.
    for spoke in range(spokes):
        angle = 2 * math.pi * spoke / spokes
        x, y = point(1.0, angle)
        lines.extend((move(x, y, 1, True), move(x, y, -0.16)))
        for step in range(1, 161):
            radius = 1.0 + 17.4 * step / 160
            x, y = point(radius, angle + 0.015 * math.sin(step * 0.17))
            z = -0.16 - 0.43 * (0.5 + 0.5 * math.sin(step * 0.31 + spoke))
            lines.append(move(x, y, z))
        lines.append("G0 Z1.000000")

    # A small cylindrical tool writes a clipped boustrophedon hatch.  The
    # point count is deliberately high enough to stress move lookup without
    # depending on external artwork.
    lines.extend(("T2 M6", "F700"))
    for row in range(hatch_lines):
        y = -17.5 + 35.0 * row / max(1, hatch_lines - 1)
        half = math.sqrt(max(0.0, 18.0 * 18.0 - y * y))
        direction = 1 if row % 2 == 0 else -1
        start = -half if direction == 1 else half
        end = half if direction == 1 else -half
        lines.extend((move(start, y, 1, True), move(start, y, -0.12)))
        for column in range(1, hatch_points + 1):
            ratio = column / hatch_points
            x = start + (end - start) * ratio
            z = -0.12 - 0.16 * ((row + column) % 9) / 8.0
            lines.append(move(x, y, z))
        lines.append("G0 Z1.000000")

    lines.extend(("M5", "M30", ""))
    return "\n".join(lines)


def project(gcode_name):
    return {
        "units": "metric",
        "resolution-mode": "manual",
        "resolution": 0.025,
        "tools": {
            "1": {
                "units": "metric",
                "shape": "snubnose",
                "length": 4.0,
                "diameter": 0.40,
                "snub_diameter": 0.10,
                "description": "0.4 mm snubnose V-bit",
            },
            "2": {
                "units": "metric",
                "shape": "cylindrical",
                "length": 4.0,
                "diameter": 0.25,
                "description": "0.25 mm end mill",
            },
        },
        "workpiece": {
            "automatic": False,
            "margin": 0,
            "bounds": {"min": [-20, -20, -0.8], "max": [20, 20, 0]},
        },
        "files": [gcode_name],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", type=Path, default=Path("build/benchmarks/40mm-stress")
    )
    parser.add_argument("--segments", type=int, default=720)
    parser.add_argument("--rings", type=int, default=36)
    parser.add_argument("--spokes", type=int, default=180)
    parser.add_argument("--hatch-lines", type=int, default=200)
    parser.add_argument("--hatch-points", type=int, default=200)
    args = parser.parse_args()

    counts = (args.segments, args.rings, args.spokes, args.hatch_lines,
              args.hatch_points)
    if any(value < 1 for value in counts):
        parser.error("all count options must be positive")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    gcode_path = args.output_dir / "camotics-fast-40mm-stress.nc"
    project_path = args.output_dir / "camotics-fast-40mm-stress.camotics"

    gcode_path.write_text(
        generate_gcode(*counts), encoding="utf-8", newline="\n"
    )
    project_path.write_text(
        json.dumps(project(gcode_path.name), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
        newline="\n",
    )

    move_count = sum(
        line.startswith(("G0 ", "G1 "))
        for line in gcode_path.read_text(encoding="utf-8").splitlines()
    )
    print(project_path)
    print(f"motion_blocks={move_count}")


if __name__ == "__main__":
    main()
