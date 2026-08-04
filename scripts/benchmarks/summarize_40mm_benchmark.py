#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later

"""Summarize the four public 40 mm benchmark cases."""

import argparse
import json
import struct
from pathlib import Path


CASES = ("full_mc", "full_mc_safe_reduce", "dexel", "dexel_safe_reduce")


def first(metrics, *names):
    for name in names:
        if name in metrics:
            return metrics[name]
    return None


def read_triangle_count(path):
    if path.stat().st_size < 84:
        raise ValueError(f"binary STL is too small: {path}")
    with path.open("rb") as stream:
        stream.seek(80)
        return struct.unpack("<I", stream.read(4))[0]


def read_case(root, name):
    case = root / name
    metrics = json.loads((case / "profile.json").read_text(
        encoding="utf-8"
    )).get("metrics", {})
    times = dict(
        line.split("=", 1)
        for line in (case / "time.txt").read_text(encoding="utf-8").splitlines()
    )
    stl = case / "result.stl"
    row = {
        "case": name,
        **times,
        "backend": (
            "dexel" if metrics.get("simulation_backend_selected_dexel")
            else "full-mc"
        ),
        "resolution_mm": metrics["simulation_resolution_microunits"] / 1e6,
        "triangles": read_triangle_count(stl),
        "stl_bytes": stl.stat().st_size,
        "sha256": (case / "SHA256SUM").read_text(
            encoding="utf-8"
        ).split()[0],
        "watertight": first(
            metrics,
            "safe_reduce_output_watertight",
            "surface_provenance_watertight",
        ),
        "boundary_edges": first(
            metrics,
            "safe_reduce_output_boundary_edges",
            "surface_provenance_boundary_edges",
            "dexel_topology_boundary_edges",
        ),
        "nonmanifold_edges": first(
            metrics,
            "safe_reduce_output_nonmanifold_edges",
            "surface_provenance_nonmanifold_edges",
            "dexel_topology_nonmanifold_edges",
        ),
        "applied_components": metrics.get("safe_reduce_applied_components"),
        "rejected_boundary_components": metrics.get(
            "safe_reduce_rejected_boundary_components"
        ),
        "rejected_boundary_triangles": metrics.get(
            "safe_reduce_rejected_boundary_triangles"
        ),
        "rejected_no_savings_components": metrics.get(
            "safe_reduce_rejected_no_savings_components"
        ),
        "rejected_no_savings_triangles": metrics.get(
            "safe_reduce_rejected_no_savings_triangles"
        ),
        "rejected_triangulation_components": metrics.get(
            "safe_reduce_rejected_triangulation_components"
        ),
        "rejected_triangulation_triangles": metrics.get(
            "safe_reduce_rejected_triangulation_triangles"
        ),
        "edge_incidence_rejected": metrics.get(
            "safe_reduce_replacement_edge_incidence_rejected"
        ),
        "validation_rolled_back": metrics.get(
            "safe_reduce_validation_rolled_back"
        ),
    }
    if row["watertight"] is None and row["boundary_edges"] == 0:
        row["watertight"] = int(row["nonmanifold_edges"] == 0)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    args = parser.parse_args()

    rows = [read_case(args.results, name) for name in CASES]
    columns = tuple(rows[0])
    with (args.results / "results.tsv").open(
        "w", encoding="utf-8", newline="\n"
    ) as output:
        output.write("\t".join(columns) + "\n")
        for row in rows:
            output.write("\t".join(
                "" if row[column] is None else str(row[column])
                for column in columns
            ) + "\n")
    (args.results / "results.json").write_text(
        json.dumps(rows, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
        newline="\n",
    )


if __name__ == "__main__":
    main()
