#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Report production-safe mesh reduction candidates for binary STL files.

This is a contract harness, not a production reducer.  It nominates connected
near-planar triangle components and classifies them through the invariants a
production reducer must preserve:

* shared-edge connected component
* bounded by closed boundary loops
* optional hole awareness
* normal and plane-distance tolerances
* estimated triangle savings
* optional global edge-incidence watertightness check

The script intentionally uses only the Python standard library.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import tempfile
import time
from array import array
from collections import Counter, defaultdict, deque
from pathlib import Path


STL_HEADER_BYTES = 80
STL_COUNT_BYTES = 4
STL_RECORD_BYTES = 50
EDGE_COORD_BITS = 21
EDGE_COORD_MASK = (1 << EDGE_COORD_BITS) - 1
EDGE_COORD_OFFSET = 1 << (EDGE_COORD_BITS - 1)
FINGERPRINT_MASK = (1 << 64) - 1
FINGERPRINT_GOLDEN = 0x9E3779B97F4A7C15
NO_NEIGHBOR = -1
INVALID_TRUSTED_RECIPROCAL_SLOT = 255


def read_triangle_count(path: Path) -> int:
    with path.open("rb") as f:
        f.seek(STL_HEADER_BYTES)
        return struct.unpack("<I", f.read(STL_COUNT_BYTES))[0]


def normalize(vector):
    length = math.sqrt(sum(value * value for value in vector))
    if length == 0:
        return None
    return tuple(value / length for value in vector)


def dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def angle_degrees_from_dot(value: float) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, value))))


def pack_coord(value: float, tolerance: float) -> int:
    quantized = int(round(value / tolerance)) + EDGE_COORD_OFFSET
    if quantized < 0 or quantized > EDGE_COORD_MASK:
        raise ValueError(
            f"Coordinate {value} is outside packable range for tolerance {tolerance}"
        )
    return quantized


def pack_vertex(point, tolerance: float) -> int:
    x = pack_coord(point[0], tolerance)
    y = pack_coord(point[1], tolerance)
    z = pack_coord(point[2], tolerance)
    return (x << (EDGE_COORD_BITS * 2)) | (y << EDGE_COORD_BITS) | z


def unpack_vertex(value: int, tolerance: float):
    z = value & EDGE_COORD_MASK
    y = (value >> EDGE_COORD_BITS) & EDGE_COORD_MASK
    x = (value >> (EDGE_COORD_BITS * 2)) & EDGE_COORD_MASK
    return (
        (x - EDGE_COORD_OFFSET) * tolerance,
        (y - EDGE_COORD_OFFSET) * tolerance,
        (z - EDGE_COORD_OFFSET) * tolerance,
    )


def edge_key(a: int, b: int) -> tuple[int, int]:
    return (a, b) if a < b else (b, a)


def mix_fingerprint(fingerprint: int, value: int) -> int:
    value = (value + FINGERPRINT_GOLDEN) & FINGERPRINT_MASK
    value = ((value ^ (value >> 30)) * 0xBF58476D1CE4E5B9) & FINGERPRINT_MASK
    value = ((value ^ (value >> 27)) * 0x94D049BB133111EB) & FINGERPRINT_MASK
    value = (value ^ (value >> 31)) & FINGERPRINT_MASK
    return (
        fingerprint
        ^ (
            value
            + FINGERPRINT_GOLDEN
            + ((fingerprint << 6) & FINGERPRINT_MASK)
            + (fingerprint >> 2)
        )
    ) & FINGERPRINT_MASK


def add_unordered_fingerprint(fingerprint: int, value: int) -> int:
    mixed = mix_fingerprint(0, value)
    rotated = ((mixed << 32) | (mixed >> 32)) & FINGERPRINT_MASK
    return (fingerprint + mixed + rotated + FINGERPRINT_GOLDEN) & FINGERPRINT_MASK


def boundary_edge_fingerprint(edge: tuple[int, int]) -> int:
    a, b = edge_key(edge[0], edge[1])
    fingerprint = mix_fingerprint(0, a)
    return mix_fingerprint(fingerprint, b)


def boundary_edges_fingerprint(edges: list[tuple[int, int]]) -> int:
    fingerprint = 0
    for edge in edges:
        fingerprint = add_unordered_fingerprint(
            fingerprint, boundary_edge_fingerprint(edge)
        )
    return fingerprint


def replacement_triangles_fingerprint(triangles) -> int:
    fingerprint = 0
    for tri in triangles:
        ids = sorted(tri)
        triangle_fingerprint = 0
        for value in ids:
            triangle_fingerprint = mix_fingerprint(triangle_fingerprint, value)
        fingerprint = add_unordered_fingerprint(fingerprint, triangle_fingerprint)
    return fingerprint


def edge_vertices(vertices: array, tri: int, edge_slot: int) -> tuple[int, int]:
    base = tri * 3
    if edge_slot == 0:
        return vertices[base], vertices[base + 1]
    if edge_slot == 1:
        return vertices[base + 1], vertices[base + 2]
    return vertices[base + 2], vertices[base]


def trusted_neighbor_reciprocal_slots(
    neighbors: array, triangle_count: int
) -> tuple[array, dict]:
    slots = array(
        "B", [INVALID_TRUSTED_RECIPROCAL_SLOT]
    ) * len(neighbors)
    validation = {
        "open_slot": False,
        "range": False,
        "self": False,
        "duplicate": False,
        "asymmetry": False,
    }

    if len(neighbors) != triangle_count * 3:
        validation["range"] = True
        return slots, validation

    for tri in range(triangle_count):
        tri_neighbors = []
        for slot in range(3):
            index = tri * 3 + slot
            neighbor = neighbors[index]
            tri_neighbors.append(neighbor)

            if neighbor < 0:
                validation["open_slot"] = True
                continue
            if neighbor >= triangle_count:
                validation["range"] = True
                continue
            if neighbor == tri:
                validation["self"] = True
                continue

            reciprocal_slot = INVALID_TRUSTED_RECIPROCAL_SLOT
            for other_slot in range(3):
                if neighbors[neighbor * 3 + other_slot] == tri:
                    reciprocal_slot = other_slot
                    break

            if reciprocal_slot == INVALID_TRUSTED_RECIPROCAL_SLOT:
                validation["asymmetry"] = True
            else:
                slots[index] = reciprocal_slot

        if tri_neighbors[0] >= 0 and tri_neighbors[0] == tri_neighbors[1]:
            validation["duplicate"] = True
        if tri_neighbors[1] >= 0 and tri_neighbors[1] == tri_neighbors[2]:
            validation["duplicate"] = True
        if tri_neighbors[2] >= 0 and tri_neighbors[2] == tri_neighbors[0]:
            validation["duplicate"] = True

    return slots, validation


def trusted_neighbor_edge_mismatches(vertices: array, neighbors: array) -> int:
    return trusted_neighbor_edge_mismatches_from_slots(vertices, neighbors, None)


def trusted_neighbor_edge_mismatches_from_slots(
    vertices: array, neighbors: array, reciprocal_slots: array | None
) -> int:
    return trusted_neighbor_edge_validation_from_slots(
        vertices, neighbors, reciprocal_slots
    )["edge_mismatches"]


def trusted_neighbor_edge_validation_from_slots(
    vertices: array, neighbors: array, reciprocal_slots: array | None
) -> dict:
    mismatches = 0
    orientation_mismatches = 0
    tri_count = len(vertices) // 3

    for tri in range(tri_count):
        for slot in range(3):
            neighbor = neighbors[tri * 3 + slot]
            if neighbor < 0 or neighbor >= tri_count:
                continue

            directed_edge = edge_vertices(vertices, tri, slot)
            edge = canonical_edge(edge_vertices(vertices, tri, slot))
            if (
                reciprocal_slots is not None
                and len(reciprocal_slots) == len(neighbors)
                and reciprocal_slots[tri * 3 + slot]
                != INVALID_TRUSTED_RECIPROCAL_SLOT
            ):
                other_slot = reciprocal_slots[tri * 3 + slot]
                directed_other_edge = edge_vertices(vertices, neighbor, other_slot)
                other_edge = canonical_edge(directed_other_edge)
                if other_edge != edge:
                    mismatches += 1
                elif directed_edge != (
                    directed_other_edge[1],
                    directed_other_edge[0],
                ):
                    orientation_mismatches += 1

            else:
                matching = False
                opposite_orientation = False
                reciprocal = False
                for other_slot in range(3):
                    if neighbors[neighbor * 3 + other_slot] != tri:
                        continue
                    reciprocal = True
                    directed_other_edge = edge_vertices(vertices, neighbor, other_slot)
                    other_edge = canonical_edge(directed_other_edge)
                    if other_edge == edge:
                        matching = True
                        if directed_edge == (
                            directed_other_edge[1],
                            directed_other_edge[0],
                        ):
                            opposite_orientation = True

                if reciprocal and not matching:
                    mismatches += 1
                elif matching and not opposite_orientation:
                    orientation_mismatches += 1

    return {
        "edge_mismatches": mismatches,
        "orientation_mismatches": orientation_mismatches,
    }


def triangle_normal(normals: array, tri: int):
    base = tri * 3
    return (normals[base], normals[base + 1], normals[base + 2])


def triangle_vertices(vertices: array, tri: int) -> tuple[int, int, int]:
    base = tri * 3
    return vertices[base], vertices[base + 1], vertices[base + 2]


def triangle_vertex_points(vertices: array, tri: int, tolerance: float):
    return tuple(unpack_vertex(value, tolerance) for value in triangle_vertices(vertices, tri))


def iter_binary_records(path: Path, max_triangles: int | None = None):
    count = read_triangle_count(path)
    if max_triangles is not None:
        count = min(count, max_triangles)

    with path.open("rb") as f:
        f.seek(STL_HEADER_BYTES + STL_COUNT_BYTES)
        for tri in range(count):
            record = f.read(STL_RECORD_BYTES)
            if len(record) != STL_RECORD_BYTES:
                raise ValueError(f"Unexpected EOF in {path} at triangle {tri}")
            yield tri, record


def build_mesh(
    path: Path,
    *,
    coord_tolerance: float,
    max_triangles: int | None,
    progress: int,
):
    actual_triangles = read_triangle_count(path)
    tri_count = actual_triangles if max_triangles is None else min(actual_triangles, max_triangles)
    vertices = array("Q")
    normals = array("f")
    neighbors = array("i", [NO_NEIGHBOR]) * (tri_count * 3)
    edge_owner: dict[tuple[int, int], int] = {}
    stats = Counter()
    start = time.time()

    for tri, record in iter_binary_records(path, max_triangles=max_triangles):
        vals = struct.unpack("<12fH", record)
        normal = normalize(vals[0:3])
        if normal is None:
            normal = (0.0, 0.0, 0.0)
            stats["zero_normals"] += 1

        points = (vals[3:6], vals[6:9], vals[9:12])
        packed = tuple(pack_vertex(point, coord_tolerance) for point in points)
        if len(set(packed)) != 3:
            stats["degenerate_triangles"] += 1

        vertices.extend(packed)
        normals.extend(normal)

        for slot, pair in enumerate(((0, 1), (1, 2), (2, 0))):
            key = edge_key(packed[pair[0]], packed[pair[1]])
            owner = edge_owner.pop(key, None)
            if owner is None:
                edge_owner[key] = tri * 3 + slot
            else:
                owner_tri = owner // 3
                owner_slot = owner % 3
                neighbors[tri * 3 + slot] = owner_tri
                neighbors[owner_tri * 3 + owner_slot] = tri
                stats["paired_edges"] += 1

        if progress and (tri + 1) % progress == 0:
            print(
                f"built adjacency: {tri + 1:,}/{tri_count:,} "
                f"open_edges={len(edge_owner):,} "
                f"({time.time() - start:.1f}s)",
                file=sys.stderr,
            )

    stats["actual_file_triangles"] = actual_triangles
    stats["triangles_loaded"] = tri_count
    stats["global_boundary_edges_from_adjacency"] = len(edge_owner)
    stats["watertight_by_adjacency"] = int(len(edge_owner) == 0)
    stats["elapsed_build_seconds"] = time.time() - start
    if max_triangles is not None and max_triangles < actual_triangles:
        stats["truncated_input"] = 1

    return vertices, normals, neighbors, dict(stats)


def strict_edge_incidence(
    path: Path,
    *,
    coord_tolerance: float,
    max_triangles: int | None,
):
    edge_counts: dict[tuple[int, int], list[int]] = defaultdict(lambda: [0, 0])
    degenerate = 0
    start = time.time()

    for _, record in iter_binary_records(path, max_triangles=max_triangles):
        vals = struct.unpack("<12fH", record)
        packed = tuple(pack_vertex(vals[offset : offset + 3], coord_tolerance) for offset in (3, 6, 9))
        if len(set(packed)) != 3:
            degenerate += 1
            continue

        for a, b in ((packed[0], packed[1]), (packed[1], packed[2]), (packed[2], packed[0])):
            key = edge_key(a, b)
            edge_counts[key][0] += 1
            if (a, b) == key:
                edge_counts[key][1] += 1

    boundary = sum(1 for count, _ in edge_counts.values() if count == 1)
    nonmanifold = sum(1 for count, _ in edge_counts.values() if count > 2)
    misoriented = sum(
        1
        for count, forward_count in edge_counts.values()
        if count == 2 and forward_count in (0, 2)
    )
    return {
        "unique_edges": len(edge_counts),
        "boundary_edges": boundary,
        "nonmanifold_edges": nonmanifold,
        "misoriented_edges": misoriented,
        "degenerate_triangles": degenerate,
        "max_edge_use": max((count for count, _ in edge_counts.values()), default=0),
        "watertight_edge_count": boundary == 0 and nonmanifold == 0,
        "elapsed_seconds": time.time() - start,
    }


def validation_topology_worse(input_incidence: dict, candidate_incidence: dict) -> bool:
    return (
        input_incidence["boundary_edges"] < candidate_incidence["boundary_edges"]
        or input_incidence["nonmanifold_edges"]
        < candidate_incidence["nonmanifold_edges"]
    )


def validation_degenerate_worse(input_incidence: dict, candidate_incidence: dict) -> bool:
    return (
        input_incidence.get("degenerate_triangles", 0)
        < candidate_incidence.get("degenerate_triangles", 0)
    )


def validation_orientation_worse(input_incidence: dict, candidate_incidence: dict) -> bool:
    return (
        input_incidence.get("misoriented_edges", 0)
        < candidate_incidence.get("misoriented_edges", 0)
    )


def validation_gate(
    input_incidence: dict,
    candidate_incidence: dict,
    *,
    vertex_count_mismatch: bool = False,
    normal_count_mismatch: bool = False,
    expected_output_triangles: int | None = None,
    candidate_triangles: int | None = None,
) -> dict:
    topology_worse = validation_topology_worse(input_incidence, candidate_incidence)
    degenerate_worse = validation_degenerate_worse(
        input_incidence, candidate_incidence
    )
    orientation_worse = validation_orientation_worse(
        input_incidence, candidate_incidence
    )
    triangle_count_mismatch = (
        expected_output_triangles is not None
        and candidate_triangles is not None
        and expected_output_triangles != candidate_triangles
    )
    rolled_back = (
        topology_worse
        or degenerate_worse
        or orientation_worse
        or vertex_count_mismatch
        or normal_count_mismatch
        or triangle_count_mismatch
    )
    output = input_incidence if rolled_back else candidate_incidence

    return {
        "input_boundary_edges": input_incidence["boundary_edges"],
        "input_nonmanifold_edges": input_incidence["nonmanifold_edges"],
        "input_degenerate_triangles": input_incidence.get(
            "degenerate_triangles", 0
        ),
        "input_misoriented_edges": input_incidence.get("misoriented_edges", 0),
        "candidate_boundary_edges": candidate_incidence["boundary_edges"],
        "candidate_nonmanifold_edges": candidate_incidence["nonmanifold_edges"],
        "candidate_degenerate_triangles": candidate_incidence.get(
            "degenerate_triangles", 0
        ),
        "candidate_misoriented_edges": candidate_incidence.get(
            "misoriented_edges", 0
        ),
        "validation_topology_worse": topology_worse,
        "validation_degenerate_worse": degenerate_worse,
        "validation_orientation_worse": orientation_worse,
        "validation_vertex_count_mismatch": vertex_count_mismatch,
        "validation_normal_count_mismatch": normal_count_mismatch,
        "validation_triangle_count_mismatch": triangle_count_mismatch,
        "validation_rolled_back": rolled_back,
        "expected_output_triangles": expected_output_triangles,
        "candidate_triangles": candidate_triangles,
        "output_boundary_edges": output["boundary_edges"],
        "output_nonmanifold_edges": output["nonmanifold_edges"],
        "output_degenerate_triangles": output.get("degenerate_triangles", 0),
        "output_misoriented_edges": output.get("misoriented_edges", 0),
        "output_watertight": (
            output["boundary_edges"] == 0 and output["nonmanifold_edges"] == 0
        ),
    }


def source_buffer_preflight(
    triangle_count: int, vertex_floats: int, normal_floats: int
) -> dict:
    expected = triangle_count * 9
    vertex_mismatch = vertex_floats != expected
    normal_mismatch = normal_floats != expected
    return {
        "triangles": triangle_count,
        "source_expected_floats": expected,
        "source_vertex_floats": vertex_floats,
        "source_normal_floats": normal_floats,
        "source_vertex_count_mismatch": vertex_mismatch,
        "source_normal_count_mismatch": normal_mismatch,
        "source_buffers_valid": not (vertex_mismatch or normal_mismatch),
    }


def trusted_provenance_gate(
    provenance: dict,
    triangle_count: int,
    *,
    cached_neighbors: bool = True,
    cached_neighbors_raw: bool = True,
    neighbor_slots: int | None = None,
    neighbor_validation: dict | None = None,
    neighbor_edge_mismatches: int = 0,
    neighbor_orientation_mismatches: int = 0,
) -> dict:
    if neighbor_validation is None:
        neighbor_validation = {}

    def metric(name: str, default=0):
        return provenance.get(name, default)

    rejected = {
        "rejected_no_cached_neighbors": not cached_neighbors,
        "rejected_triangle_mismatch": metric("triangles") != triangle_count,
        "rejected_incomplete": metric("complete_triangles") != triangle_count,
        "rejected_unknown": metric("unknown_triangles") != 0,
        "rejected_non_watertight": not bool(metric("watertight", False)),
        "rejected_orientation": (
            metric("raw_misoriented_edges") != 0
            or metric("misoriented_edges") != 0
            or neighbor_orientation_mismatches != 0
        ),
        "rejected_neighbor_size": (
            neighbor_slots is None or neighbor_slots != triangle_count * 3
        ),
        "rejected_neighbor_open_slot": bool(
            neighbor_validation.get("open_slot", False)
        ),
        "rejected_neighbor_range": bool(neighbor_validation.get("range", False)),
        "rejected_neighbor_self": bool(neighbor_validation.get("self", False)),
        "rejected_neighbor_duplicate": bool(
            neighbor_validation.get("duplicate", False)
        ),
        "rejected_neighbor_asymmetry": bool(
            neighbor_validation.get("asymmetry", False)
        ),
        "rejected_neighbor_edge_mismatch": neighbor_edge_mismatches != 0,
    }

    raw_topology_matches_welded = (
        metric("raw_boundary_edges") == 0
        and metric("raw_nonmanifold_edges") == 0
        and metric("raw_misoriented_edges") == 0
        and metric("misoriented_edges") == 0
        and metric("raw_unique_edges") == metric("welded_unique_edges")
        and metric("raw_twin_edge_slots") == metric("welded_twin_edge_slots")
        and metric("raw_boundary_edge_slots")
        == metric("welded_boundary_edge_slots")
        and metric("raw_nonmanifold_edge_slots")
        == metric("welded_nonmanifold_edge_slots")
    )
    raw_keys_map_to_single_welded_point = (
        metric("raw_grid_grid_welded_spread_edges") == 0
        and metric("raw_center_involved_welded_spread_edges") == 0
        and metric("raw_grid_vertex_welded_spread_keys") == 0
        and metric("raw_center_vertex_welded_spread_keys") == 0
    )

    rejected["rejected_raw_topology"] = (
        cached_neighbors_raw and not raw_topology_matches_welded
    )
    rejected["rejected_raw_welded_spread"] = (
        cached_neighbors_raw and not raw_keys_map_to_single_welded_point
    )

    return {
        **rejected,
        "cached_neighbors_raw": cached_neighbors_raw,
        "eligible": not any(rejected.values()),
    }


def component_fits_seed_plane(
    tri: int,
    seed_normal,
    seed_distance: float,
    vertices: array,
    normals: array,
    *,
    coord_tolerance: float,
    normal_tolerance: float,
    plane_distance_tolerance: float,
) -> bool:
    normal = triangle_normal(normals, tri)
    if dot(seed_normal, normal) < normal_tolerance:
        return False

    for point in triangle_vertex_points(vertices, tri, coord_tolerance):
        if abs(dot(seed_normal, point) - seed_distance) > plane_distance_tolerance:
            return False
    return True


def boundary_loop_stats(edges: list[tuple[int, int]]):
    graph: dict[int, list[int]] = defaultdict(list)
    duplicate_edges = 0
    seen_edges: set[tuple[int, int]] = set()

    for a, b in edges:
        key = edge_key(a, b)
        if key in seen_edges:
            duplicate_edges += 1
        seen_edges.add(key)
        graph[a].append(b)
        graph[b].append(a)

    degree_counts = Counter(len(neighbors) for neighbors in graph.values())
    bad_vertices = sum(count for degree, count in degree_counts.items() if degree != 2)
    visited: set[int] = set()
    loops = 0

    for start in graph:
        if start in visited:
            continue
        loops += 1
        queue = deque([start])
        visited.add(start)
        while queue:
            value = queue.popleft()
            for neighbor in graph[value]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)

    return {
        "boundary_vertices": len(graph),
        "boundary_edges": len(edges),
        "boundary_loops": loops,
        "bad_boundary_vertices": bad_vertices,
        "duplicate_boundary_edges": duplicate_edges,
        "boundary_vertex_degree_counts": dict(sorted(degree_counts.items())),
    }


def order_boundary_loop(edges: list[tuple[int, int]]):
    loops = order_boundary_loops(edges)
    if loops is None or len(loops) != 1:
        return None
    return loops[0]


def order_boundary_loops(edges: list[tuple[int, int]]):
    graph: dict[int, list[int]] = defaultdict(list)
    for a, b in edges:
        graph[a].append(b)
        graph[b].append(a)

    if not graph or len(graph) != len(edges):
        return None
    if any(len(neighbors) != 2 for neighbors in graph.values()):
        return None

    loops = []
    unvisited = set(graph)
    while unvisited:
        start = min(unvisited)
        loop = []
        previous = None
        current = start

        while True:
            if current not in unvisited:
                return None

            loop.append(current)
            unvisited.remove(current)
            neighbors = graph[current]
            next_value = neighbors[1] if neighbors[0] == previous else neighbors[0]

            previous = current
            current = next_value

            if current == start:
                break
            if len(graph) < len(loop):
                return None

        if len(loop) < 3:
            return None
        loops.append(loop)

    return loops


def triangulation_estimate(boundary) -> tuple[int, bool]:
    if boundary["boundary_loops"] < 1:
        return 0, False
    if boundary["bad_boundary_vertices"] or boundary["duplicate_boundary_edges"]:
        return 0, False

    # For one outer loop and h holes, triangles = V + 2h - 2.
    # With loops = h + 1, this is V + 2 * loops - 4.
    triangles = boundary["boundary_vertices"] + 2 * boundary["boundary_loops"] - 4
    return max(1, triangles), True


def projection_drop_axis(normal) -> int:
    axes = [abs(normal[0]), abs(normal[1]), abs(normal[2])]
    return max(range(3), key=lambda axis: axes[axis])


def project_point(point, drop_axis: int):
    if drop_axis == 0:
        return (point[1], point[2])
    if drop_axis == 1:
        return (point[0], point[2])
    return (point[0], point[1])


def cross2(a, b, c) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def polygon_area_2d(loop, coord_tolerance: float, drop_axis: int) -> float:
    area = 0.0
    for index, vertex in enumerate(loop):
        a = project_point(unpack_vertex(vertex, coord_tolerance), drop_axis)
        b = project_point(
            unpack_vertex(loop[(index + 1) % len(loop)], coord_tolerance),
            drop_axis,
        )
        area += a[0] * b[1] - b[0] * a[1]
    return area * 0.5


def point_in_triangle_inclusive(p, a, b, c, eps: float) -> bool:
    ab = cross2(a, b, p)
    bc = cross2(b, c, p)
    ca = cross2(c, a, p)
    return -eps <= ab and -eps <= bc and -eps <= ca


def point_in_polygon(point, loop, coord_tolerance: float, drop_axis: int) -> bool:
    x, y = point
    inside = False
    count = len(loop)

    for index, vertex in enumerate(loop):
        a = project_point(unpack_vertex(vertex, coord_tolerance), drop_axis)
        b = project_point(
            unpack_vertex(loop[(index + 1) % count], coord_tolerance),
            drop_axis,
        )

        if (a[1] > y) == (b[1] > y):
            continue

        x_at_y = (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]
        if x < x_at_y:
            inside = not inside

    return inside


def points_close_2d(a, b, eps: float) -> bool:
    return abs(a[0] - b[0]) <= eps and abs(a[1] - b[1]) <= eps


def on_segment_2d(p, a, b, eps: float) -> bool:
    if abs(cross2(a, b, p)) > eps:
        return False
    return (
        min(a[0], b[0]) - eps <= p[0] <= max(a[0], b[0]) + eps
        and min(a[1], b[1]) - eps <= p[1] <= max(a[1], b[1]) + eps
    )


def segments_intersect_2d(a, b, c, d, eps: float) -> bool:
    a1 = cross2(a, b, c)
    a2 = cross2(a, b, d)
    a3 = cross2(c, d, a)
    a4 = cross2(c, d, b)

    if ((a1 > eps and a2 < -eps) or (a1 < -eps and a2 > eps)) and (
        (a3 > eps and a4 < -eps) or (a3 < -eps and a4 > eps)
    ):
        return True

    return (
        on_segment_2d(c, a, b, eps)
        or on_segment_2d(d, a, b, eps)
        or on_segment_2d(a, c, d, eps)
        or on_segment_2d(b, c, d, eps)
    )


def bridge_crosses_boundary(
    a_id,
    b_id,
    loops,
    *,
    coord_tolerance: float,
    drop_axis: int,
    eps: float,
) -> bool:
    a = project_point(unpack_vertex(a_id, coord_tolerance), drop_axis)
    b = project_point(unpack_vertex(b_id, coord_tolerance), drop_axis)
    bridge_endpoints = {a_id, b_id}

    for loop in loops:
        for index, c_id in enumerate(loop):
            d_id = loop[(index + 1) % len(loop)]
            edge_endpoints = {c_id, d_id}
            if bridge_endpoints & edge_endpoints:
                continue

            c = project_point(unpack_vertex(c_id, coord_tolerance), drop_axis)
            d = project_point(unpack_vertex(d_id, coord_tolerance), drop_axis)
            if segments_intersect_2d(a, b, c, d, eps):
                return True

    return False


def loop_projection_scale(loops, coord_tolerance: float, drop_axis: int) -> float:
    scale = 0.0
    for loop in loops:
        for vertex in loop:
            point = project_point(unpack_vertex(vertex, coord_tolerance), drop_axis)
            scale = max(scale, abs(point[0]), abs(point[1]))
    return scale


def loop_vertices_inside_loop(
    inner_loop, outer_loop, coord_tolerance: float, drop_axis: int
) -> bool:
    for vertex in inner_loop:
        point = project_point(unpack_vertex(vertex, coord_tolerance), drop_axis)
        if not point_in_polygon(point, outer_loop, coord_tolerance, drop_axis):
            return False
    return True


def loops_intersect(
    loop_a, loop_b, coord_tolerance: float, drop_axis: int, eps: float
) -> bool:
    for a_index, a0_id in enumerate(loop_a):
        a1_id = loop_a[(a_index + 1) % len(loop_a)]
        a0 = project_point(unpack_vertex(a0_id, coord_tolerance), drop_axis)
        a1 = project_point(unpack_vertex(a1_id, coord_tolerance), drop_axis)

        for b_index, b0_id in enumerate(loop_b):
            b1_id = loop_b[(b_index + 1) % len(loop_b)]
            if {a0_id, a1_id} & {b0_id, b1_id}:
                return True

            b0 = project_point(unpack_vertex(b0_id, coord_tolerance), drop_axis)
            b1 = project_point(unpack_vertex(b1_id, coord_tolerance), drop_axis)
            if segments_intersect_2d(a0, a1, b0, b1, eps):
                return True

    return False


def triangulate_loop(loop, coord_tolerance: float, seed_normal):
    if len(loop) < 3:
        return None

    drop_axis = projection_drop_axis(seed_normal)
    working = list(loop)
    area = polygon_area_2d(working, coord_tolerance, drop_axis)
    if abs(area) < 1e-18:
        return None
    if area < 0:
        working.reverse()

    scale = 0.0
    for vertex in working:
        point = project_point(unpack_vertex(vertex, coord_tolerance), drop_axis)
        scale = max(scale, abs(point[0]), abs(point[1]))

    eps = max(1e-18, scale * scale * 1e-14)
    triangles = []
    guard = 0

    while len(working) > 3:
        clipped = False
        count = len(working)

        for index in range(count):
            prev_index = (index + count - 1) % count
            next_index = (index + 1) % count
            prev = working[prev_index]
            curr = working[index]
            next_value = working[next_index]

            if len({prev, curr, next_value}) != 3:
                continue

            a = project_point(unpack_vertex(prev, coord_tolerance), drop_axis)
            b = project_point(unpack_vertex(curr, coord_tolerance), drop_axis)
            c = project_point(unpack_vertex(next_value, coord_tolerance), drop_axis)
            if cross2(a, b, c) <= eps:
                continue

            blocked = False
            for other_index, other in enumerate(working):
                if other_index in (prev_index, index, next_index):
                    continue
                if other in (prev, curr, next_value):
                    continue
                p = project_point(unpack_vertex(other, coord_tolerance), drop_axis)
                if point_in_triangle_inclusive(p, a, b, c, eps):
                    blocked = True
                    break

            if blocked:
                continue

            triangles.append((prev, curr, next_value))
            del working[index]
            clipped = True
            break

        if not clipped:
            return None
        guard += 1
        if guard > len(loop) * len(loop):
            return None

    if len(set(working)) != 3:
        return None

    triangles.append((working[0], working[1], working[2]))
    if len(triangles) + 2 != len(loop):
        return None
    return triangles


def canonical_edge(edge):
    a, b = edge
    return (a, b) if a <= b else (b, a)


def replacement_boundary_matches(boundary_edges, replacement_triangles) -> bool:
    input_boundary = {canonical_edge(edge) for edge in boundary_edges}
    counts = Counter()
    for tri in replacement_triangles:
        a, b, c = tri
        counts[canonical_edge((a, b))] += 1
        counts[canonical_edge((b, c))] += 1
        counts[canonical_edge((c, a))] += 1

    if any(count > 2 for count in counts.values()):
        return False

    output_boundary = {edge for edge, count in counts.items() if count == 1}
    return output_boundary == input_boundary


def replacement_edge_incidence(boundary_edges, replacement_triangles) -> dict:
    input_boundary = {canonical_edge(edge) for edge in boundary_edges}
    counts = Counter()
    forward_counts = Counter()
    degenerate = 0

    for tri in replacement_triangles:
        if len(set(tri)) != 3:
            degenerate += 1
            continue

        for edge in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
            key = canonical_edge(edge)
            counts[key] += 1
            if edge == key:
                forward_counts[key] += 1

    output_boundary = {edge for edge, count in counts.items() if count == 1}
    nonmanifold = sum(1 for count in counts.values() if count > 2)
    misoriented = sum(
        1
        for edge, count in counts.items()
        if count == 2 and forward_counts[edge] in (0, 2)
    )
    boundary_matches = output_boundary == input_boundary

    return {
        "boundary_matches": boundary_matches,
        "boundary_edges": len(output_boundary),
        "nonmanifold_edges": nonmanifold,
        "misoriented_edges": misoriented,
        "degenerate_triangles": degenerate,
        "valid": (
            boundary_matches
            and nonmanifold == 0
            and misoriented == 0
            and degenerate == 0
        ),
    }


def replacement_boundary_matches_direction(boundary_edges, replacement_triangles) -> bool:
    input_boundary = set(boundary_edges)
    counts = Counter()
    for tri in replacement_triangles:
        a, b, c = tri
        counts[canonical_edge((a, b))] += 1
        counts[canonical_edge((b, c))] += 1
        counts[canonical_edge((c, a))] += 1

    output_boundary = set()
    for tri in replacement_triangles:
        a, b, c = tri
        for edge in ((a, b), (b, c), (c, a)):
            if counts[canonical_edge(edge)] == 1:
                output_boundary.add(edge)

    return output_boundary == input_boundary


def flip_replacement_triangles(replacement_triangles):
    return [(a, c, b) for a, b, c in replacement_triangles]


def orient_replacement_to_boundary(boundary_edges, replacement_triangles):
    if replacement_boundary_matches_direction(boundary_edges, replacement_triangles):
        return replacement_triangles

    flipped = flip_replacement_triangles(replacement_triangles)
    if replacement_boundary_matches_direction(boundary_edges, flipped):
        return flipped

    return None


def triangle_area_2d_ids(tri, coord_tolerance: float, drop_axis: int) -> float:
    a = project_point(unpack_vertex(tri[0], coord_tolerance), drop_axis)
    b = project_point(unpack_vertex(tri[1], coord_tolerance), drop_axis)
    c = project_point(unpack_vertex(tri[2], coord_tolerance), drop_axis)
    return abs(cross2(a, b, c)) * 0.5


def hole_aware_target_area(outer_loop, hole_loops, coord_tolerance, drop_axis):
    return abs(polygon_area_2d(outer_loop, coord_tolerance, drop_axis)) - sum(
        abs(polygon_area_2d(loop, coord_tolerance, drop_axis))
        for loop in hole_loops
    )


def find_visible_bridge(current_loop, hole_loop, all_loops, *,
                        coord_tolerance: float, drop_axis: int):
    scale = 0.0
    for loop in all_loops + [current_loop]:
        for vertex in loop:
            point = project_point(unpack_vertex(vertex, coord_tolerance), drop_axis)
            scale = max(scale, abs(point[0]), abs(point[1]))

    eps = max(1e-12, scale * scale * 1e-12)
    candidates = []

    for hole_index, hole_vertex in enumerate(hole_loop):
        hole_point = project_point(
            unpack_vertex(hole_vertex, coord_tolerance), drop_axis
        )
        for outer_index, outer_vertex in enumerate(current_loop):
            if outer_vertex == hole_vertex:
                continue

            outer_point = project_point(
                unpack_vertex(outer_vertex, coord_tolerance), drop_axis
            )
            if points_close_2d(hole_point, outer_point, eps):
                continue
            if bridge_crosses_boundary(
                hole_vertex,
                outer_vertex,
                all_loops + [current_loop],
                coord_tolerance=coord_tolerance,
                drop_axis=drop_axis,
                eps=eps,
            ):
                continue

            distance2 = (
                (hole_point[0] - outer_point[0]) ** 2
                + (hole_point[1] - outer_point[1]) ** 2
            )
            candidates.append((distance2, outer_index, hole_index))

    if not candidates:
        return None
    _, outer_index, hole_index = min(candidates)
    return outer_index, hole_index


def triangulate_hole_aware_loops(loops, coord_tolerance: float, seed_normal):
    drop_axis = projection_drop_axis(seed_normal)
    loop_infos = []
    for loop in loops:
        area = polygon_area_2d(loop, coord_tolerance, drop_axis)
        if abs(area) < 1e-18:
            return None
        loop_infos.append({"loop": list(loop), "area": area, "abs_area": abs(area)})

    outer_info = max(loop_infos, key=lambda item: item["abs_area"])
    hole_infos = [info for info in loop_infos if info is not outer_info]
    current_loop = list(outer_info["loop"])
    if polygon_area_2d(current_loop, coord_tolerance, drop_axis) < 0:
        current_loop.reverse()

    hole_loops = []
    for info in hole_infos:
        hole = list(info["loop"])
        if polygon_area_2d(hole, coord_tolerance, drop_axis) > 0:
            hole.reverse()
        hole_loops.append(hole)

    for hole in sorted(hole_loops, key=len, reverse=True):
        bridge = find_visible_bridge(
            current_loop,
            hole,
            hole_loops,
            coord_tolerance=coord_tolerance,
            drop_axis=drop_axis,
        )
        if bridge is None:
            return None
        outer_index, hole_index = bridge
        current_loop = (
            current_loop[: outer_index + 1]
            + hole[hole_index:]
            + hole[: hole_index + 1]
            + [current_loop[outer_index]]
            + current_loop[outer_index + 1:]
        )

    triangles = triangulate_loop(current_loop, coord_tolerance, seed_normal)
    if triangles is None:
        return None

    target_area = hole_aware_target_area(
        outer_info["loop"], [info["loop"] for info in hole_infos],
        coord_tolerance, drop_axis
    )
    triangle_area = sum(
        triangle_area_2d_ids(tri, coord_tolerance, drop_axis)
        for tri in triangles
    )
    area_tolerance = max(1e-9, abs(target_area) * 1e-8)
    if abs(triangle_area - target_area) > area_tolerance:
        return None

    return triangles


def hole_aware_boundary_validation(
    boundary_edges: list[tuple[int, int]],
    *,
    coord_tolerance: float,
    seed_normal,
):
    result = {
        "hole_aware_checked": True,
        "hole_aware_feasible": False,
        "hole_aware_failure": "",
        "hole_aware_ordered_loops": 0,
        "hole_aware_outer_loops": 0,
        "hole_aware_hole_loops": 0,
        "hole_aware_estimated_triangles_after": 0,
    }

    loops = order_boundary_loops(boundary_edges)
    if loops is None:
        result["hole_aware_failure"] = "loop_order_failed"
        return result

    drop_axis = projection_drop_axis(seed_normal)
    loop_infos = []
    for loop in loops:
        area = polygon_area_2d(loop, coord_tolerance, drop_axis)
        if abs(area) < 1e-18:
            result["hole_aware_failure"] = "zero_area_loop"
            return result
        loop_infos.append({
            "loop": loop,
            "area": area,
            "abs_area": abs(area),
        })

    outer = max(loop_infos, key=lambda item: item["abs_area"])
    hole_infos = [info for info in loop_infos if info is not outer]
    scale = loop_projection_scale(loops, coord_tolerance, drop_axis)
    eps = max(1e-12, scale * scale * 1e-12)

    for hole in hole_infos:
        if (
            not loop_vertices_inside_loop(
                hole["loop"], outer["loop"], coord_tolerance, drop_axis
            )
            or loops_intersect(
                hole["loop"], outer["loop"], coord_tolerance, drop_axis, eps
            )
        ):
            result["hole_aware_failure"] = "hole_outside_outer_loop"
            return result
        for other in hole_infos:
            if other is hole:
                continue
            if (
                loop_vertices_inside_loop(
                    hole["loop"], other["loop"], coord_tolerance, drop_axis
                )
                or loop_vertices_inside_loop(
                    other["loop"], hole["loop"], coord_tolerance, drop_axis
                )
            ):
                result["hole_aware_failure"] = "hole_inside_hole"
                return result
            if loops_intersect(
                hole["loop"], other["loop"], coord_tolerance, drop_axis, eps
            ):
                result["hole_aware_failure"] = "hole_intersects_hole"
                return result

    result["hole_aware_feasible"] = True
    result["hole_aware_ordered_loops"] = len(loops)
    result["hole_aware_outer_loops"] = 1
    result["hole_aware_hole_loops"] = len(hole_infos)
    result["hole_aware_estimated_triangles_after"] = (
        sum(len(loop) for loop in loops) + 2 * len(hole_infos) - 2
    )
    return result


def hole_aware_replacement_feasibility(
    boundary_edges: list[tuple[int, int]],
    *,
    coord_tolerance: float,
    seed_normal,
):
    result = {
        "hole_aware_replacement_checked": True,
        "hole_aware_replacement_feasible": False,
        "hole_aware_replacement_triangles_after": 0,
        "hole_aware_replacement_boundary_matches": False,
        "hole_aware_replacement_area_matches": False,
        "hole_aware_replacement_edge_incidence_checked": False,
        "hole_aware_replacement_edge_incidence_valid": False,
        "hole_aware_replacement_triangle_count": 0,
        "hole_aware_replacement_triangle_fingerprint": 0,
        "hole_aware_replacement_failure": "",
    }

    validation = hole_aware_boundary_validation(
        boundary_edges,
        coord_tolerance=coord_tolerance,
        seed_normal=seed_normal,
    )
    if not validation["hole_aware_feasible"]:
        result["hole_aware_replacement_failure"] = (
            validation["hole_aware_failure"] or "boundary_validation_failed"
        )
        return result

    loops = order_boundary_loops(boundary_edges)
    if loops is None or len(loops) < 2:
        result["hole_aware_replacement_failure"] = "loop_order_failed"
        return result

    triangles = triangulate_hole_aware_loops(
        loops, coord_tolerance, seed_normal
    )
    if triangles is None:
        result["hole_aware_replacement_failure"] = "triangulation_failed"
        return result

    if not replacement_boundary_matches(boundary_edges, triangles):
        result["hole_aware_replacement_failure"] = "boundary_mismatch"
        return result
    triangles = orient_replacement_to_boundary(boundary_edges, triangles)
    if triangles is None:
        result["hole_aware_replacement_failure"] = "boundary_direction_mismatch"
        return result
    incidence = replacement_edge_incidence(boundary_edges, triangles)
    result["hole_aware_replacement_edge_incidence_checked"] = True
    result["hole_aware_replacement_edge_incidence_valid"] = incidence["valid"]
    if not incidence["valid"]:
        result["hole_aware_replacement_failure"] = "edge_incidence_mismatch"
        return result
    result["hole_aware_replacement_triangle_count"] = len(triangles)
    result["hole_aware_replacement_triangle_fingerprint"] = (
        replacement_triangles_fingerprint(triangles)
    )

    drop_axis = projection_drop_axis(seed_normal)
    loop_infos = [
        {
            "loop": loop,
            "abs_area": abs(polygon_area_2d(loop, coord_tolerance, drop_axis)),
        }
        for loop in loops
    ]
    outer = max(loop_infos, key=lambda item: item["abs_area"])
    hole_loops = [info["loop"] for info in loop_infos if info is not outer]
    target_area = hole_aware_target_area(
        outer["loop"], hole_loops, coord_tolerance, drop_axis
    )
    triangle_area = sum(
        triangle_area_2d_ids(tri, coord_tolerance, drop_axis)
        for tri in triangles
    )
    area_tolerance = max(1e-9, abs(target_area) * 1e-8)
    if abs(triangle_area - target_area) > area_tolerance:
        result["hole_aware_replacement_failure"] = "area_mismatch"
        return result

    result["hole_aware_replacement_feasible"] = True
    result["hole_aware_replacement_triangles_after"] = len(triangles)
    result["hole_aware_replacement_boundary_matches"] = True
    result["hole_aware_replacement_area_matches"] = True
    return result


def replacement_feasibility(
    boundary_edges: list[tuple[int, int]],
    *,
    coord_tolerance: float,
    seed_normal,
):
    result = {
        "replacement_checked": True,
        "replacement_feasible": False,
        "replacement_triangles_after": 0,
        "replacement_boundary_matches": False,
        "replacement_area_matches": False,
        "replacement_edge_incidence_checked": False,
        "replacement_edge_incidence_valid": False,
        "replacement_triangle_count": 0,
        "replacement_triangle_fingerprint": 0,
        "replacement_failure": "",
    }

    loop = order_boundary_loop(boundary_edges)
    if loop is None:
        result["replacement_failure"] = "loop_order_failed"
        return result

    triangles = triangulate_loop(loop, coord_tolerance, seed_normal)
    if triangles is None:
        result["replacement_failure"] = "triangulation_failed"
        return result

    if not replacement_boundary_matches(boundary_edges, triangles):
        result["replacement_failure"] = "boundary_mismatch"
        return result
    triangles = orient_replacement_to_boundary(boundary_edges, triangles)
    if triangles is None:
        result["replacement_failure"] = "boundary_direction_mismatch"
        return result
    incidence = replacement_edge_incidence(boundary_edges, triangles)
    result["replacement_edge_incidence_checked"] = True
    result["replacement_edge_incidence_valid"] = incidence["valid"]
    if not incidence["valid"]:
        result["replacement_failure"] = "edge_incidence_mismatch"
        return result
    result["replacement_triangle_count"] = len(triangles)
    result["replacement_triangle_fingerprint"] = replacement_triangles_fingerprint(
        triangles
    )

    drop_axis = projection_drop_axis(seed_normal)
    target_area = abs(polygon_area_2d(loop, coord_tolerance, drop_axis))
    triangle_area = sum(
        triangle_area_2d_ids(tri, coord_tolerance, drop_axis)
        for tri in triangles
    )
    area_tolerance = max(1e-9, target_area * 1e-8)
    if abs(triangle_area - target_area) > area_tolerance:
        result["replacement_failure"] = "area_mismatch"
        return result

    result["replacement_feasible"] = True
    result["replacement_triangles_after"] = len(triangles)
    result["replacement_boundary_matches"] = True
    result["replacement_area_matches"] = True
    return result


def orientation_bucket(normal) -> str:
    abs_z = abs(normal[2])
    if abs_z >= math.cos(math.radians(5.0)):
        return "near_horizontal"
    if abs_z <= math.sin(math.radians(5.0)):
        return "near_vertical"
    return "sloped"


def classify_candidate(
    component,
    boundary,
    reduction: int,
    source_is_truncated: bool,
    replacement,
    hole_aware,
    hole_aware_replacement=None,
) -> str:
    if hole_aware_replacement is None:
        hole_aware_replacement = {}

    if source_is_truncated:
        return "diagnostic_truncated_input"
    if boundary["bad_boundary_vertices"] or boundary["duplicate_boundary_edges"]:
        return "reject_boundary_not_closed_loops"
    if boundary["boundary_loops"] < 1:
        return "reject_missing_boundary"
    if replacement.get("replacement_checked") and not replacement.get("replacement_feasible"):
        return "reject_replacement_triangulation_failed"
    if boundary["boundary_loops"] > 1:
        if not hole_aware.get("hole_aware_checked") or not hole_aware.get("hole_aware_feasible"):
            return "reject_hole_aware_boundary_validation_failed"
        if (
            hole_aware_replacement.get("hole_aware_replacement_checked")
            and not hole_aware_replacement.get("hole_aware_replacement_feasible")
        ):
            return "reject_hole_aware_replacement_failed"
        if reduction <= 0:
            return "reject_no_triangle_savings"
        return "candidate_requires_hole_aware_triangulation"
    if reduction <= 0:
        return "reject_no_triangle_savings"
    if component["max_plane_distance"] > component["plane_distance_tolerance"]:
        return "reject_plane_distance"
    if component["max_normal_angle_degrees"] > component["effective_seed_normal_angle_degrees"] + 1e-9:
        return "reject_normal_angle"
    return "cxx_phase1_simple_planar_candidate"


def component_metrics(component, seed_normal, seed_distance, vertices, normals, coord_tolerance):
    bounds_min = [math.inf, math.inf, math.inf]
    bounds_max = [-math.inf, -math.inf, -math.inf]
    max_plane_distance = 0.0
    max_normal_angle = 0.0

    for tri in component:
        normal = triangle_normal(normals, tri)
        max_normal_angle = max(max_normal_angle, angle_degrees_from_dot(dot(seed_normal, normal)))
        for point in triangle_vertex_points(vertices, tri, coord_tolerance):
            max_plane_distance = max(max_plane_distance, abs(dot(seed_normal, point) - seed_distance))
            for axis in range(3):
                bounds_min[axis] = min(bounds_min[axis], point[axis])
                bounds_max[axis] = max(bounds_max[axis], point[axis])

    return {
        "bounds_min": bounds_min,
        "bounds_max": bounds_max,
        "max_plane_distance": max_plane_distance,
        "max_normal_angle_degrees": max_normal_angle,
    }


def normalize_replacement_decision(
    boundary_loops: int,
    replacement: dict,
    hole_aware: dict,
    hole_aware_replacement: dict,
) -> dict:
    if boundary_loops <= 1:
        return {
            "checked": bool(replacement.get("replacement_checked", False)),
            "estimate_available": False,
            "feasible": bool(replacement.get("replacement_feasible", False)),
            "edge_incidence_checked": bool(
                replacement.get("replacement_edge_incidence_checked", False)
            ),
            "edge_incidence_ok": bool(
                replacement.get("replacement_edge_incidence_valid", False)
            ),
            "triangles_after": int(replacement.get("replacement_triangles_after", 0)),
            "triangle_count": int(replacement.get("replacement_triangle_count", 0)),
            "triangle_fingerprint": int(
                replacement.get("replacement_triangle_fingerprint", 0)
            ),
        }

    estimate_available = bool(hole_aware.get("hole_aware_feasible", False))
    triangles_after = int(
        hole_aware_replacement.get("hole_aware_replacement_triangles_after", 0)
    )
    if estimate_available and not triangles_after:
        triangles_after = int(hole_aware.get("hole_aware_estimated_triangles_after", 0))

    return {
        "checked": bool(
            hole_aware_replacement.get("hole_aware_replacement_checked", False)
        ),
        "estimate_available": estimate_available,
        "feasible": bool(
            hole_aware_replacement.get("hole_aware_replacement_feasible", False)
        ),
        "edge_incidence_checked": bool(
            hole_aware_replacement.get(
                "hole_aware_replacement_edge_incidence_checked", False
            )
        ),
        "edge_incidence_ok": bool(
            hole_aware_replacement.get(
                "hole_aware_replacement_edge_incidence_valid", False
            )
        ),
        "triangles_after": triangles_after,
        "triangle_count": int(
            hole_aware_replacement.get("hole_aware_replacement_triangle_count", 0)
        ),
        "triangle_fingerprint": int(
            hole_aware_replacement.get(
                "hole_aware_replacement_triangle_fingerprint", 0
            )
        ),
    }


def component_decision_fingerprint(
    *,
    component_triangles: int,
    boundary_edges: list[tuple[int, int]],
    boundary: dict,
    estimated_after: int,
    reduction: int,
    decision: dict,
) -> int:
    fingerprint = 0
    for value in (
        component_triangles,
        boundary_edges_fingerprint(boundary_edges),
        boundary["boundary_vertices"],
        boundary["boundary_edges"],
        boundary["boundary_loops"],
        boundary["bad_boundary_vertices"],
        boundary["duplicate_boundary_edges"],
        estimated_after,
        reduction,
        1 if decision["checked"] else 0,
        1 if decision["estimate_available"] else 0,
        1 if decision["feasible"] else 0,
        1 if decision["edge_incidence_checked"] else 0,
        1 if decision["edge_incidence_ok"] else 0,
        decision["triangles_after"],
        decision["triangle_count"],
        decision["triangle_fingerprint"],
    ):
        fingerprint = mix_fingerprint(fingerprint, int(value))
    return fingerprint


def analyze_components(
    vertices: array,
    normals: array,
    neighbors: array,
    *,
    coord_tolerance: float,
    pairwise_normal_angle_degrees: float,
    plane_distance_tolerance: float,
    min_component_triangles: int,
    max_top_components: int,
    source_is_truncated: bool,
    progress: int,
):
    tri_count = len(normals) // 3
    visited = bytearray(tri_count)
    component_mark = array("i", [NO_NEIGHBOR]) * tri_count
    normal_tolerance = math.cos(math.radians(pairwise_normal_angle_degrees / 2.0))
    effective_seed_angle = pairwise_normal_angle_degrees / 2.0
    totals = Counter()
    top_components = []
    component_id = 0
    visited_count = 0
    start = time.time()

    def keep_top(candidate):
        if candidate["triangles"] < min_component_triangles:
            return
        top_components.append(candidate)
        top_components.sort(
            key=lambda item: (
                item["classification"] != "cxx_phase1_simple_planar_candidate",
                -item["estimated_triangle_reduction"],
                -item["triangles"],
            )
        )
        del top_components[max_top_components:]

    for seed in range(tri_count):
        if visited[seed]:
            continue

        seed_normal = triangle_normal(normals, seed)
        if dot(seed_normal, seed_normal) == 0:
            visited[seed] = 1
            visited_count += 1
            totals["zero_normal_components"] += 1
            continue

        seed_point = triangle_vertex_points(vertices, seed, coord_tolerance)[0]
        seed_distance = dot(seed_normal, seed_point)
        stack = [seed]
        component = []
        visited[seed] = 1
        visited_count += 1
        component_mark[seed] = component_id

        while stack:
            tri = stack.pop()
            component.append(tri)
            for slot in range(3):
                neighbor = neighbors[tri * 3 + slot]
                if neighbor < 0 or visited[neighbor]:
                    continue
                if component_fits_seed_plane(
                    neighbor,
                    seed_normal,
                    seed_distance,
                    vertices,
                    normals,
                    coord_tolerance=coord_tolerance,
                    normal_tolerance=normal_tolerance,
                    plane_distance_tolerance=plane_distance_tolerance,
                ):
                    visited[neighbor] = 1
                    visited_count += 1
                    component_mark[neighbor] = component_id
                    stack.append(neighbor)

        boundary_edges = []
        for tri in component:
            for slot in range(3):
                neighbor = neighbors[tri * 3 + slot]
                if neighbor < 0 or component_mark[neighbor] != component_id:
                    boundary_edges.append(edge_vertices(vertices, tri, slot))

        boundary = boundary_loop_stats(boundary_edges)
        estimated_after, valid_boundary = triangulation_estimate(boundary)
        if not valid_boundary:
            estimated_after = len(component)

        reduction = max(0, len(component) - estimated_after)
        metrics = component_metrics(
            component, seed_normal, seed_distance, vertices, normals, coord_tolerance
        )
        replacement = {
            "replacement_checked": False,
            "replacement_feasible": False,
            "replacement_triangles_after": 0,
            "replacement_edge_incidence_checked": False,
            "replacement_edge_incidence_valid": False,
            "replacement_triangle_count": 0,
            "replacement_triangle_fingerprint": 0,
            "replacement_failure": "not_checked",
        }
        hole_aware = {
            "hole_aware_checked": False,
            "hole_aware_feasible": False,
            "hole_aware_failure": "not_checked",
            "hole_aware_ordered_loops": 0,
            "hole_aware_outer_loops": 0,
            "hole_aware_hole_loops": 0,
            "hole_aware_estimated_triangles_after": 0,
        }
        hole_aware_replacement = {
            "hole_aware_replacement_checked": False,
            "hole_aware_replacement_feasible": False,
            "hole_aware_replacement_triangles_after": 0,
            "hole_aware_replacement_boundary_matches": False,
            "hole_aware_replacement_area_matches": False,
            "hole_aware_replacement_edge_incidence_checked": False,
            "hole_aware_replacement_edge_incidence_valid": False,
            "hole_aware_replacement_triangle_count": 0,
            "hole_aware_replacement_triangle_fingerprint": 0,
            "hole_aware_replacement_failure": "not_checked",
        }
        if (
            valid_boundary
            and boundary["boundary_loops"] == 1
            and reduction > 0
            and not source_is_truncated
        ):
            replacement = replacement_feasibility(
                boundary_edges,
                coord_tolerance=coord_tolerance,
                seed_normal=seed_normal,
            )
            if replacement["replacement_feasible"]:
                estimated_after = replacement["replacement_triangles_after"]
                reduction = max(0, len(component) - estimated_after)
            else:
                estimated_after = len(component)
                reduction = 0
        elif (
            valid_boundary
            and boundary["boundary_loops"] > 1
            and reduction > 0
            and not source_is_truncated
        ):
            hole_aware = hole_aware_boundary_validation(
                boundary_edges,
                coord_tolerance=coord_tolerance,
                seed_normal=seed_normal,
            )
            if hole_aware["hole_aware_feasible"]:
                hole_aware_replacement = hole_aware_replacement_feasibility(
                    boundary_edges,
                    coord_tolerance=coord_tolerance,
                    seed_normal=seed_normal,
                )
                if hole_aware_replacement["hole_aware_replacement_feasible"]:
                    estimated_after = hole_aware_replacement[
                        "hole_aware_replacement_triangles_after"
                    ]
                else:
                    estimated_after = hole_aware["hole_aware_estimated_triangles_after"]
                reduction = max(0, len(component) - estimated_after)
            else:
                estimated_after = len(component)
                reduction = 0

        bucket = orientation_bucket(seed_normal)
        candidate_base = {
            "triangles": len(component),
            "estimated_triangles_after": estimated_after,
            "estimated_triangle_reduction": reduction,
            "seed_normal": seed_normal,
            "seed_plane_distance": seed_distance,
            "orientation_bucket": bucket,
            "plane_distance_tolerance": plane_distance_tolerance,
            "effective_seed_normal_angle_degrees": effective_seed_angle,
            **metrics,
            **boundary,
            **replacement,
            **hole_aware,
            **hole_aware_replacement,
        }
        classification = classify_candidate(
            candidate_base, boundary, reduction, source_is_truncated,
            replacement, hole_aware, hole_aware_replacement
        )
        decision = normalize_replacement_decision(
            boundary["boundary_loops"], replacement, hole_aware,
            hole_aware_replacement
        )
        decision_bearing = bool(
            reduction
            or decision["checked"]
            or decision["estimate_available"]
            or decision["edge_incidence_checked"]
        )
        decision_fingerprint = (
            component_decision_fingerprint(
                component_triangles=len(component),
                boundary_edges=boundary_edges,
                boundary=boundary,
                estimated_after=estimated_after,
                reduction=reduction,
                decision=decision,
            )
            if decision_bearing
            else 0
        )

        totals["components"] += 1
        totals["triangles"] += len(component)
        totals[f"classification_{classification}"] += 1
        totals[f"classification_{classification}_triangles"] += len(component)
        totals[f"classification_{classification}_estimated_reduction"] += reduction
        totals[f"{bucket}_components"] += 1
        totals[f"{bucket}_triangles"] += len(component)
        totals[f"{bucket}_estimated_reduction"] += reduction
        totals["estimated_triangles_after"] += estimated_after
        totals["estimated_triangle_reduction"] += reduction
        if decision_bearing:
            totals["decision_bearing_components"] += 1
            totals["decision_bearing_triangles"] += len(component)
            totals["component_decision_fingerprint"] = (
                add_unordered_fingerprint(
                    totals["component_decision_fingerprint"],
                    decision_fingerprint,
                )
            )

        keep_top(
            {
                "component_id": component_id,
                "classification": classification,
                "decision_bearing": decision_bearing,
                "component_decision_fingerprint": decision_fingerprint,
                **candidate_base,
            }
        )

        component_id += 1
        if progress and component_id % progress == 0:
            print(
                f"components={component_id:,} visited={visited_count:,}/{tri_count:,} "
                f"({time.time() - start:.1f}s)",
                file=sys.stderr,
            )

    totals["elapsed_component_seconds"] = time.time() - start
    totals["component_decision_fingerprint"] += 0
    totals["decision_bearing_components"] += 0
    totals["decision_bearing_triangles"] += 0
    return {
        "thresholds": {
            "coord_tolerance": coord_tolerance,
            "pairwise_normal_angle_degrees": pairwise_normal_angle_degrees,
            "effective_seed_normal_angle_degrees": effective_seed_angle,
            "normal_tolerance": normal_tolerance,
            "plane_distance_tolerance": plane_distance_tolerance,
            "min_component_triangles": min_component_triangles,
        },
        "component_totals": dict(totals),
        "top_components": top_components,
    }


def report_contract_notes():
    return [
        "Nomination is not replacement.  C++ may only replace candidates that pass all gates.",
        "cxx_phase1_simple_planar_candidate has one closed boundary loop and estimated savings.",
        "candidate_requires_hole_aware_triangulation has ordered outer/hole loops; C++ applies it only when --safe-reduce-hole-aware is requested.",
        "hole_aware_replacement_feasible is the Python reference contract for the C++ --safe-reduce-hole-aware writer.",
        "hole-aware replacement_feasible covers one outer loop with one or more hole loops.",
        "hole-aware validation rejects loops outside the outer loop and nested or intersecting hole loops.",
        "hole-aware replacement_feasible requires replacement area to match outer area minus hole area.",
        "hole-aware replacement_feasible rejects same-direction hole loops because writable output must preserve source boundary direction.",
        "phase-one replacement_feasible requires replacement boundary edges to match the source boundary exactly.",
        "phase-one replacement_feasible requires replacement area to match the source loop area.",
        "Trusted neighbor sidecars must prove reciprocal triangle links share the same undirected edge.",
        "Trusted provenance sidecars must prove raw and welded edge orientation before skipping default adjacency.",
        "The production reducer must reject replacement output that increases boundary-edge, nonmanifold-edge, misoriented-edge, or degenerate-triangle counts.",
        "Export validation must still use strict edge incidence and sampled bidirectional distance.",
    ]


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Report production-safe planar mesh reduction candidates."
    )
    parser.add_argument("input_stl", nargs="?", type=Path)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--coord-tolerance", type=float, default=1e-4)
    parser.add_argument("--plane-distance-tolerance", type=float, default=1e-4)
    parser.add_argument("--pairwise-normal-angle-degrees", type=float, default=0.25)
    parser.add_argument("--min-component-triangles", type=int, default=64)
    parser.add_argument("--max-top-components", type=int, default=100)
    parser.add_argument("--max-triangles", type=int)
    parser.add_argument("--edge-incidence", action="store_true")
    parser.add_argument(
        "--edge-incidence-only",
        action="store_true",
        help="Report strict edge incidence without component classification.",
    )
    parser.add_argument("--build-progress", type=int, default=1_000_000)
    parser.add_argument("--component-progress", type=int, default=100_000)
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args(argv)


def run_report(args: argparse.Namespace):
    if args.input_stl is None:
        raise SystemExit("input_stl is required unless --self-test is used")

    start = time.time()
    if args.edge_incidence_only:
        incidence = strict_edge_incidence(
            args.input_stl,
            coord_tolerance=args.coord_tolerance,
            max_triangles=args.max_triangles,
        )
        return {
            "input_stl": str(args.input_stl),
            "strict_edge_incidence": incidence,
            "contract_notes": report_contract_notes(),
            "elapsed_wall_seconds": time.time() - start,
        }

    vertices, normals, neighbors, build_stats = build_mesh(
        args.input_stl,
        coord_tolerance=args.coord_tolerance,
        max_triangles=args.max_triangles,
        progress=args.build_progress,
    )
    analysis = analyze_components(
        vertices,
        normals,
        neighbors,
        coord_tolerance=args.coord_tolerance,
        pairwise_normal_angle_degrees=args.pairwise_normal_angle_degrees,
        plane_distance_tolerance=args.plane_distance_tolerance,
        min_component_triangles=args.min_component_triangles,
        max_top_components=args.max_top_components,
        source_is_truncated=bool(build_stats.get("truncated_input")),
        progress=args.component_progress,
    )
    incidence = None
    if args.edge_incidence:
        incidence = strict_edge_incidence(
            args.input_stl,
            coord_tolerance=args.coord_tolerance,
            max_triangles=args.max_triangles,
        )

    result = {
        "input_stl": str(args.input_stl),
        "build": build_stats,
        "strict_edge_incidence": incidence,
        **analysis,
        "contract_notes": report_contract_notes(),
        "elapsed_wall_seconds": time.time() - start,
    }
    return result


def make_record(points):
    normal = normalize(cross(sub(points[1], points[0]), sub(points[2], points[0])))
    if normal is None:
        normal = (0.0, 0.0, 0.0)
    return struct.pack("<12fH", *(normal + points[0] + points[1] + points[2]), 0)


def write_test_stl(path: Path, triangles):
    with path.open("wb") as f:
        f.write(b"contract report self-test".ljust(STL_HEADER_BYTES, b" "))
        f.write(struct.pack("<I", len(triangles)))
        for tri in triangles:
            f.write(make_record(tri))


def grid_plane_triangles(width: int, height: int):
    triangles = []
    for y in range(height):
        for x in range(width):
            p00 = (float(x), float(y), 0.0)
            p10 = (float(x + 1), float(y), 0.0)
            p11 = (float(x + 1), float(y + 1), 0.0)
            p01 = (float(x), float(y + 1), 0.0)
            triangles.append((p00, p10, p11))
            triangles.append((p00, p11, p01))
    return triangles


def grid_ring_triangles(width: int, height: int, hole_x: int, hole_y: int,
                        hole_width: int, hole_height: int):
    triangles = []
    for y in range(height):
        for x in range(width):
            in_hole = (
                hole_x <= x < hole_x + hole_width and
                hole_y <= y < hole_y + hole_height
            )
            if in_hole:
                continue

            p00 = (float(x), float(y), 0.0)
            p10 = (float(x + 1), float(y), 0.0)
            p11 = (float(x + 1), float(y + 1), 0.0)
            p01 = (float(x), float(y + 1), 0.0)
            triangles.append((p00, p10, p11))
            triangles.append((p00, p11, p01))
    return triangles


def grid_with_missing_cells(width: int, height: int, missing: set[tuple[int, int]]):
    triangles = []
    for y in range(height):
        for x in range(width):
            if (x, y) in missing:
                continue

            p00 = (float(x), float(y), 0.0)
            p10 = (float(x + 1), float(y), 0.0)
            p11 = (float(x + 1), float(y + 1), 0.0)
            p01 = (float(x), float(y + 1), 0.0)
            triangles.append((p00, p10, p11))
            triangles.append((p00, p11, p01))
    return triangles


def make_report_args(path: Path, *, edge_incidence=False,
                     edge_incidence_only=False):
    return argparse.Namespace(
        input_stl=path,
        output_json=None,
        coord_tolerance=1e-4,
        plane_distance_tolerance=1e-4,
        pairwise_normal_angle_degrees=0.25,
        min_component_triangles=1,
        max_top_components=10,
        max_triangles=None,
        edge_incidence=edge_incidence,
        edge_incidence_only=edge_incidence_only,
        build_progress=0,
        component_progress=0,
    )


def run_simple_plane_self_test(tmp: Path) -> dict:
    path = tmp / "grid.stl"
    write_test_stl(path, grid_plane_triangles(2, 2))
    result = run_report(make_report_args(path, edge_incidence=True))

    top = result["top_components"][0]
    assert result["build"]["triangles_loaded"] == 8
    assert top["classification"] == "cxx_phase1_simple_planar_candidate"
    assert top["boundary_loops"] == 1
    assert top["bad_boundary_vertices"] == 0
    assert top["replacement_checked"] is True
    assert top["replacement_feasible"] is True
    assert top["replacement_boundary_matches"] is True
    assert top["replacement_area_matches"] is True
    assert top["replacement_triangles_after"] == top["estimated_triangles_after"]
    assert top["estimated_triangle_reduction"] == 2
    assert result["strict_edge_incidence"]["watertight_edge_count"] is False
    return result


def run_concave_plane_self_test(tmp: Path) -> dict:
    path = tmp / "concave-grid.stl"
    write_test_stl(path, grid_with_missing_cells(3, 3, {(2, 2)}))
    result = run_report(make_report_args(path))

    top = result["top_components"][0]
    assert result["build"]["triangles_loaded"] == 16
    assert top["classification"] == "cxx_phase1_simple_planar_candidate"
    assert top["boundary_loops"] == 1
    assert top["bad_boundary_vertices"] == 0
    assert top["replacement_checked"] is True
    assert top["replacement_feasible"] is True
    assert top["replacement_boundary_matches"] is True
    assert top["replacement_area_matches"] is True
    assert top["replacement_triangles_after"] == top["estimated_triangles_after"]
    assert top["estimated_triangle_reduction"] > 0
    return result


def run_hole_aware_self_test(tmp: Path) -> dict:
    path = tmp / "ring.stl"
    write_test_stl(path, grid_ring_triangles(6, 6, 2, 2, 2, 2))
    result = run_report(make_report_args(path))

    top = result["top_components"][0]
    totals = result["component_totals"]
    assert result["build"]["triangles_loaded"] == 64
    assert top["classification"] == "candidate_requires_hole_aware_triangulation"
    assert top["boundary_loops"] == 2
    assert top["bad_boundary_vertices"] == 0
    assert top["duplicate_boundary_edges"] == 0
    assert top["replacement_checked"] is False
    assert top["replacement_feasible"] is False
    assert top["hole_aware_checked"] is True
    assert top["hole_aware_feasible"] is True
    assert top["hole_aware_ordered_loops"] == 2
    assert top["hole_aware_outer_loops"] == 1
    assert top["hole_aware_hole_loops"] == 1
    assert top["hole_aware_estimated_triangles_after"] == top["estimated_triangles_after"]
    assert top["hole_aware_replacement_checked"] is True
    assert top["hole_aware_replacement_feasible"] is True
    assert top["hole_aware_replacement_boundary_matches"] is True
    assert top["hole_aware_replacement_area_matches"] is True
    assert top["hole_aware_replacement_triangles_after"] == top["estimated_triangles_after"]
    assert top["estimated_triangle_reduction"] > 0
    assert totals["classification_candidate_requires_hole_aware_triangulation"] == 1
    return result


def run_hole_aware_two_hole_self_test() -> dict:
    tolerance = 1e-4
    edges = (
        packed_square_edges(0.0, 0.0, 6.0, tolerance)
        + packed_square_edges(1.0, 1.0, 1.0, tolerance, clockwise=True)
        + packed_square_edges(4.0, 3.5, 1.0, tolerance, clockwise=True)
    )
    boundary = boundary_loop_stats(edges)
    estimated_after, valid_boundary = triangulation_estimate(boundary)
    hole_aware = hole_aware_boundary_validation(
        edges,
        coord_tolerance=tolerance,
        seed_normal=(0.0, 0.0, 1.0),
    )
    hole_aware_replacement = hole_aware_replacement_feasibility(
        edges,
        coord_tolerance=tolerance,
        seed_normal=(0.0, 0.0, 1.0),
    )
    component = {
        "max_plane_distance": 0.0,
        "plane_distance_tolerance": tolerance,
        "max_normal_angle_degrees": 0.0,
        "effective_seed_normal_angle_degrees": 0.125,
    }
    classification = classify_candidate(
        component,
        boundary,
        reduction=20,
        source_is_truncated=False,
        replacement={"replacement_checked": False, "replacement_feasible": False},
        hole_aware=hole_aware,
        hole_aware_replacement=hole_aware_replacement,
    )

    assert valid_boundary is True
    assert estimated_after == 14
    assert boundary["boundary_loops"] == 3
    assert boundary["bad_boundary_vertices"] == 0
    assert boundary["duplicate_boundary_edges"] == 0
    assert hole_aware["hole_aware_checked"] is True
    assert hole_aware["hole_aware_feasible"] is True
    assert hole_aware["hole_aware_outer_loops"] == 1
    assert hole_aware["hole_aware_hole_loops"] == 2
    assert hole_aware["hole_aware_estimated_triangles_after"] == estimated_after
    assert hole_aware_replacement["hole_aware_replacement_checked"] is True
    assert hole_aware_replacement["hole_aware_replacement_feasible"] is True
    assert hole_aware_replacement["hole_aware_replacement_triangles_after"] == 14
    assert hole_aware_replacement["hole_aware_replacement_boundary_matches"] is True
    assert hole_aware_replacement["hole_aware_replacement_area_matches"] is True
    assert (
        hole_aware_replacement["hole_aware_replacement_edge_incidence_checked"]
        is True
    )
    assert (
        hole_aware_replacement["hole_aware_replacement_edge_incidence_valid"]
        is True
    )
    assert classification == "candidate_requires_hole_aware_triangulation"
    return {
        "boundary": boundary,
        "estimated_triangles_after": estimated_after,
        "hole_aware": hole_aware,
        "hole_aware_replacement": hole_aware_replacement,
        "classification": classification,
    }


def run_bad_boundary_self_test() -> dict:
    boundary = boundary_loop_stats([
        (1, 2),
        (2, 3),
        (3, 1),
        (1, 4),
    ])
    estimated_after, valid_boundary = triangulation_estimate(boundary)
    component = {
        "max_plane_distance": 0.0,
        "plane_distance_tolerance": 1e-4,
        "max_normal_angle_degrees": 0.0,
        "effective_seed_normal_angle_degrees": 0.125,
    }
    classification = classify_candidate(
        component, boundary, reduction=8, source_is_truncated=False,
        replacement={"replacement_feasible": False},
        hole_aware={"hole_aware_feasible": False})

    assert valid_boundary is False
    assert estimated_after == 0
    assert boundary["bad_boundary_vertices"] == 2
    assert classification == "reject_boundary_not_closed_loops"
    return {
        "boundary": boundary,
        "classification": classification,
    }


def packed_square_edges(
    x: float, y: float, size: float, tolerance: float, *, clockwise: bool = False
):
    return packed_loop_edges(
        [
            (x, y, 0.0),
            (x + size, y, 0.0),
            (x + size, y + size, 0.0),
            (x, y + size, 0.0),
        ],
        tolerance,
        clockwise=clockwise,
    )


def packed_loop_edges(points, tolerance: float, *, clockwise: bool = False):
    ids = [pack_vertex(point, tolerance) for point in points]
    count = len(ids)

    if clockwise:
        return [
            (ids[(count - index) % count], ids[count - index - 1])
            for index in range(count)
        ]

    return [(ids[index], ids[(index + 1) % count]) for index in range(count)]


def plane_point(origin, u_axis, v_axis, u_value: float, v_value: float):
    return tuple(
        origin[index] + u_axis[index] * u_value + v_axis[index] * v_value
        for index in range(3)
    )


def plane_loop_points(origin, u_axis, v_axis, coords):
    return [
        plane_point(origin, u_axis, v_axis, u_value, v_value)
        for u_value, v_value in coords
    ]


def run_vertical_replacement_self_test() -> dict:
    tolerance = 1e-4
    x_edges = packed_loop_edges(
        [
            (0.0, 0.0, 0.0),
            (0.0, 2.0, 0.0),
            (0.0, 2.0, 2.0),
            (0.0, 0.0, 2.0),
        ],
        tolerance,
    )
    x_replacement = replacement_feasibility(
        x_edges,
        coord_tolerance=tolerance,
        seed_normal=(1.0, 0.0, 0.0),
    )

    y_edges = packed_loop_edges(
        [
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 2.0),
            (2.0, 0.0, 2.0),
            (2.0, 0.0, 0.0),
        ],
        tolerance,
    )
    y_replacement = replacement_feasibility(
        y_edges,
        coord_tolerance=tolerance,
        seed_normal=(0.0, 1.0, 0.0),
    )

    x_ring_edges = packed_loop_edges(
        [
            (0.0, 0.0, 0.0),
            (0.0, 4.0, 0.0),
            (0.0, 4.0, 4.0),
            (0.0, 0.0, 4.0),
        ],
        tolerance,
    ) + packed_loop_edges(
        [
            (0.0, 1.0, 1.0),
            (0.0, 2.0, 1.0),
            (0.0, 2.0, 2.0),
            (0.0, 1.0, 2.0),
        ],
        tolerance,
        clockwise=True,
    )
    x_ring_boundary = boundary_loop_stats(x_ring_edges)
    x_ring_hole_aware = hole_aware_boundary_validation(
        x_ring_edges,
        coord_tolerance=tolerance,
        seed_normal=(1.0, 0.0, 0.0),
    )
    x_ring_replacement = hole_aware_replacement_feasibility(
        x_ring_edges,
        coord_tolerance=tolerance,
        seed_normal=(1.0, 0.0, 0.0),
    )

    assert x_replacement["replacement_checked"] is True
    assert x_replacement["replacement_feasible"] is True
    assert x_replacement["replacement_triangles_after"] == 2
    assert x_replacement["replacement_boundary_matches"] is True
    assert x_replacement["replacement_area_matches"] is True
    assert y_replacement["replacement_checked"] is True
    assert y_replacement["replacement_feasible"] is True
    assert y_replacement["replacement_triangles_after"] == 2
    assert y_replacement["replacement_boundary_matches"] is True
    assert y_replacement["replacement_area_matches"] is True
    assert x_ring_boundary["boundary_loops"] == 2
    assert x_ring_hole_aware["hole_aware_checked"] is True
    assert x_ring_hole_aware["hole_aware_feasible"] is True
    assert x_ring_hole_aware["hole_aware_outer_loops"] == 1
    assert x_ring_hole_aware["hole_aware_hole_loops"] == 1
    assert x_ring_replacement["hole_aware_replacement_checked"] is True
    assert x_ring_replacement["hole_aware_replacement_feasible"] is True
    assert x_ring_replacement["hole_aware_replacement_triangles_after"] == 8
    assert x_ring_replacement["hole_aware_replacement_boundary_matches"] is True
    assert x_ring_replacement["hole_aware_replacement_area_matches"] is True

    return {
        "x_phase1_triangles_after": x_replacement["replacement_triangles_after"],
        "y_phase1_triangles_after": y_replacement["replacement_triangles_after"],
        "x_hole_aware_loops": x_ring_boundary["boundary_loops"],
        "x_hole_aware_triangles_after": x_ring_replacement[
            "hole_aware_replacement_triangles_after"
        ],
        "x_phase1_boundary_matches": x_replacement[
            "replacement_boundary_matches"
        ],
        "y_phase1_boundary_matches": y_replacement[
            "replacement_boundary_matches"
        ],
        "x_hole_aware_boundary_matches": x_ring_replacement[
            "hole_aware_replacement_boundary_matches"
        ],
    }


def run_sloped_replacement_self_test() -> dict:
    tolerance = 1e-4
    origin = (0.0, 0.0, 0.0)
    u_axis = (2.0, 0.0, 1.0)
    v_axis = (0.0, 2.0, 0.5)
    seed_normal = normalize(cross(u_axis, v_axis))
    assert seed_normal is not None

    phase1_edges = packed_loop_edges(
        plane_loop_points(
            origin,
            u_axis,
            v_axis,
            [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)],
        ),
        tolerance,
    )
    phase1_replacement = replacement_feasibility(
        phase1_edges,
        coord_tolerance=tolerance,
        seed_normal=seed_normal,
    )

    ring_edges = packed_loop_edges(
        plane_loop_points(
            origin,
            u_axis,
            v_axis,
            [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)],
        ),
        tolerance,
    ) + packed_loop_edges(
        plane_loop_points(
            origin,
            u_axis,
            v_axis,
            [(0.5, 0.5), (1.0, 0.5), (1.0, 1.0), (0.5, 1.0)],
        ),
        tolerance,
        clockwise=True,
    )
    ring_boundary = boundary_loop_stats(ring_edges)
    ring_hole_aware = hole_aware_boundary_validation(
        ring_edges,
        coord_tolerance=tolerance,
        seed_normal=seed_normal,
    )
    ring_replacement = hole_aware_replacement_feasibility(
        ring_edges,
        coord_tolerance=tolerance,
        seed_normal=seed_normal,
    )

    assert phase1_replacement["replacement_checked"] is True
    assert phase1_replacement["replacement_feasible"] is True
    assert phase1_replacement["replacement_triangles_after"] == 2
    assert phase1_replacement["replacement_boundary_matches"] is True
    assert phase1_replacement["replacement_area_matches"] is True
    assert ring_boundary["boundary_loops"] == 2
    assert ring_hole_aware["hole_aware_checked"] is True
    assert ring_hole_aware["hole_aware_feasible"] is True
    assert ring_hole_aware["hole_aware_outer_loops"] == 1
    assert ring_hole_aware["hole_aware_hole_loops"] == 1
    assert ring_replacement["hole_aware_replacement_checked"] is True
    assert ring_replacement["hole_aware_replacement_feasible"] is True
    assert ring_replacement["hole_aware_replacement_triangles_after"] == 8
    assert ring_replacement["hole_aware_replacement_boundary_matches"] is True
    assert ring_replacement["hole_aware_replacement_area_matches"] is True

    return {
        "phase1_triangles_after": phase1_replacement[
            "replacement_triangles_after"
        ],
        "phase1_boundary_matches": phase1_replacement[
            "replacement_boundary_matches"
        ],
        "hole_aware_loops": ring_boundary["boundary_loops"],
        "hole_aware_triangles_after": ring_replacement[
            "hole_aware_replacement_triangles_after"
        ],
        "hole_aware_boundary_matches": ring_replacement[
            "hole_aware_replacement_boundary_matches"
        ],
    }


def run_hole_aware_failure_self_test() -> dict:
    tolerance = 1e-4
    edges = (
        packed_square_edges(0.0, 0.0, 1.0, tolerance)
        + packed_square_edges(3.0, 0.0, 1.0, tolerance)
    )
    boundary = boundary_loop_stats(edges)
    estimated_after, _ = triangulation_estimate(boundary)
    hole_aware = hole_aware_boundary_validation(
        edges,
        coord_tolerance=tolerance,
        seed_normal=(0.0, 0.0, 1.0),
    )
    hole_aware_replacement = hole_aware_replacement_feasibility(
        edges,
        coord_tolerance=tolerance,
        seed_normal=(0.0, 0.0, 1.0),
    )
    component = {
        "max_plane_distance": 0.0,
        "plane_distance_tolerance": tolerance,
        "max_normal_angle_degrees": 0.0,
        "effective_seed_normal_angle_degrees": 0.125,
    }
    classification = classify_candidate(
        component,
        boundary,
        reduction=10,
        source_is_truncated=False,
        replacement={"replacement_checked": False, "replacement_feasible": False},
        hole_aware=hole_aware,
    )

    assert estimated_after == 8
    assert boundary["boundary_loops"] == 2
    assert hole_aware["hole_aware_checked"] is True
    assert hole_aware["hole_aware_feasible"] is False
    assert hole_aware["hole_aware_failure"] == "hole_outside_outer_loop"
    assert hole_aware_replacement["hole_aware_replacement_checked"] is True
    assert hole_aware_replacement["hole_aware_replacement_feasible"] is False
    assert (
        hole_aware_replacement["hole_aware_replacement_failure"]
        == "hole_outside_outer_loop"
    )
    assert classification == "reject_hole_aware_boundary_validation_failed"
    return {
        "boundary": boundary,
        "estimated_triangles_after": estimated_after,
        "hole_aware": hole_aware,
        "hole_aware_replacement": hole_aware_replacement,
        "classification": classification,
    }


def run_hole_aware_same_direction_failure_self_test() -> dict:
    tolerance = 1e-4
    edges = (
        packed_square_edges(0.0, 0.0, 4.0, tolerance)
        + packed_square_edges(1.0, 1.0, 1.0, tolerance)
    )
    boundary = boundary_loop_stats(edges)
    estimated_after, _ = triangulation_estimate(boundary)
    hole_aware = hole_aware_boundary_validation(
        edges,
        coord_tolerance=tolerance,
        seed_normal=(0.0, 0.0, 1.0),
    )
    hole_aware_replacement = hole_aware_replacement_feasibility(
        edges,
        coord_tolerance=tolerance,
        seed_normal=(0.0, 0.0, 1.0),
    )
    component = {
        "max_plane_distance": 0.0,
        "plane_distance_tolerance": tolerance,
        "max_normal_angle_degrees": 0.0,
        "effective_seed_normal_angle_degrees": 0.125,
    }
    classification = classify_candidate(
        component,
        boundary,
        reduction=10,
        source_is_truncated=False,
        replacement={"replacement_checked": False, "replacement_feasible": False},
        hole_aware=hole_aware,
        hole_aware_replacement=hole_aware_replacement,
    )

    assert estimated_after == 8
    assert boundary["boundary_loops"] == 2
    assert hole_aware["hole_aware_checked"] is True
    assert hole_aware["hole_aware_feasible"] is True
    assert hole_aware["hole_aware_failure"] == ""
    assert hole_aware_replacement["hole_aware_replacement_checked"] is True
    assert hole_aware_replacement["hole_aware_replacement_feasible"] is False
    assert (
        hole_aware_replacement["hole_aware_replacement_failure"]
        == "boundary_direction_mismatch"
    )
    assert classification == "reject_hole_aware_replacement_failed"
    return {
        "boundary": boundary,
        "estimated_triangles_after": estimated_after,
        "hole_aware": hole_aware,
        "hole_aware_replacement": hole_aware_replacement,
        "classification": classification,
    }


def run_hole_aware_nested_failure_self_test() -> dict:
    tolerance = 1e-4
    edges = (
        packed_square_edges(0.0, 0.0, 4.0, tolerance)
        + packed_square_edges(1.0, 1.0, 2.0, tolerance)
        + packed_square_edges(1.5, 1.5, 0.5, tolerance)
    )
    boundary = boundary_loop_stats(edges)
    estimated_after, _ = triangulation_estimate(boundary)
    hole_aware = hole_aware_boundary_validation(
        edges,
        coord_tolerance=tolerance,
        seed_normal=(0.0, 0.0, 1.0),
    )
    hole_aware_replacement = hole_aware_replacement_feasibility(
        edges,
        coord_tolerance=tolerance,
        seed_normal=(0.0, 0.0, 1.0),
    )
    component = {
        "max_plane_distance": 0.0,
        "plane_distance_tolerance": tolerance,
        "max_normal_angle_degrees": 0.0,
        "effective_seed_normal_angle_degrees": 0.125,
    }
    classification = classify_candidate(
        component,
        boundary,
        reduction=10,
        source_is_truncated=False,
        replacement={"replacement_checked": False, "replacement_feasible": False},
        hole_aware=hole_aware,
    )

    assert estimated_after == 14
    assert boundary["boundary_loops"] == 3
    assert hole_aware["hole_aware_checked"] is True
    assert hole_aware["hole_aware_feasible"] is False
    assert hole_aware["hole_aware_failure"] == "hole_inside_hole"
    assert hole_aware_replacement["hole_aware_replacement_checked"] is True
    assert hole_aware_replacement["hole_aware_replacement_feasible"] is False
    assert (
        hole_aware_replacement["hole_aware_replacement_failure"]
        == "hole_inside_hole"
    )
    assert classification == "reject_hole_aware_boundary_validation_failed"
    return {
        "boundary": boundary,
        "estimated_triangles_after": estimated_after,
        "hole_aware": hole_aware,
        "hole_aware_replacement": hole_aware_replacement,
        "classification": classification,
    }


def run_hole_aware_straddling_failure_self_test() -> dict:
    tolerance = 1e-4
    edges = (
        packed_square_edges(0.0, 0.0, 4.0, tolerance)
        + packed_square_edges(3.2, 1.5, 1.0, tolerance)
    )
    boundary = boundary_loop_stats(edges)
    hole_aware = hole_aware_boundary_validation(
        edges,
        coord_tolerance=tolerance,
        seed_normal=(0.0, 0.0, 1.0),
    )
    hole_aware_replacement = hole_aware_replacement_feasibility(
        edges,
        coord_tolerance=tolerance,
        seed_normal=(0.0, 0.0, 1.0),
    )
    component = {
        "max_plane_distance": 0.0,
        "plane_distance_tolerance": tolerance,
        "max_normal_angle_degrees": 0.0,
        "effective_seed_normal_angle_degrees": 0.125,
    }
    classification = classify_candidate(
        component,
        boundary,
        reduction=10,
        source_is_truncated=False,
        replacement={"replacement_checked": False, "replacement_feasible": False},
        hole_aware=hole_aware,
    )

    assert boundary["boundary_loops"] == 2
    assert hole_aware["hole_aware_checked"] is True
    assert hole_aware["hole_aware_feasible"] is False
    assert hole_aware["hole_aware_failure"] == "hole_outside_outer_loop"
    assert hole_aware_replacement["hole_aware_replacement_checked"] is True
    assert hole_aware_replacement["hole_aware_replacement_feasible"] is False
    assert (
        hole_aware_replacement["hole_aware_replacement_failure"]
        == "hole_outside_outer_loop"
    )
    assert classification == "reject_hole_aware_boundary_validation_failed"
    return {
        "boundary": boundary,
        "hole_aware": hole_aware,
        "hole_aware_replacement": hole_aware_replacement,
        "classification": classification,
    }


def run_hole_aware_intersecting_holes_failure_self_test() -> dict:
    tolerance = 1e-4
    edges = (
        packed_square_edges(0.0, 0.0, 5.0, tolerance)
        + packed_square_edges(1.0, 1.0, 1.5, tolerance)
        + packed_square_edges(2.0, 1.5, 1.5, tolerance)
    )
    boundary = boundary_loop_stats(edges)
    hole_aware = hole_aware_boundary_validation(
        edges,
        coord_tolerance=tolerance,
        seed_normal=(0.0, 0.0, 1.0),
    )
    hole_aware_replacement = hole_aware_replacement_feasibility(
        edges,
        coord_tolerance=tolerance,
        seed_normal=(0.0, 0.0, 1.0),
    )
    component = {
        "max_plane_distance": 0.0,
        "plane_distance_tolerance": tolerance,
        "max_normal_angle_degrees": 0.0,
        "effective_seed_normal_angle_degrees": 0.125,
    }
    classification = classify_candidate(
        component,
        boundary,
        reduction=10,
        source_is_truncated=False,
        replacement={"replacement_checked": False, "replacement_feasible": False},
        hole_aware=hole_aware,
    )

    assert boundary["boundary_loops"] == 3
    assert hole_aware["hole_aware_checked"] is True
    assert hole_aware["hole_aware_feasible"] is False
    assert hole_aware["hole_aware_failure"] == "hole_intersects_hole"
    assert hole_aware_replacement["hole_aware_replacement_checked"] is True
    assert hole_aware_replacement["hole_aware_replacement_feasible"] is False
    assert (
        hole_aware_replacement["hole_aware_replacement_failure"]
        == "hole_intersects_hole"
    )
    assert classification == "reject_hole_aware_boundary_validation_failed"
    return {
        "boundary": boundary,
        "hole_aware": hole_aware,
        "hole_aware_replacement": hole_aware_replacement,
        "classification": classification,
    }


def run_hole_aware_touching_outer_failure_self_test() -> dict:
    tolerance = 1e-4
    edges = (
        packed_square_edges(0.0, 0.0, 4.0, tolerance)
        + packed_square_edges(3.0, 1.0, 1.0, tolerance)
    )
    boundary = boundary_loop_stats(edges)
    hole_aware = hole_aware_boundary_validation(
        edges,
        coord_tolerance=tolerance,
        seed_normal=(0.0, 0.0, 1.0),
    )
    hole_aware_replacement = hole_aware_replacement_feasibility(
        edges,
        coord_tolerance=tolerance,
        seed_normal=(0.0, 0.0, 1.0),
    )
    component = {
        "max_plane_distance": 0.0,
        "plane_distance_tolerance": tolerance,
        "max_normal_angle_degrees": 0.0,
        "effective_seed_normal_angle_degrees": 0.125,
    }
    classification = classify_candidate(
        component,
        boundary,
        reduction=10,
        source_is_truncated=False,
        replacement={"replacement_checked": False, "replacement_feasible": False},
        hole_aware=hole_aware,
    )

    assert boundary["boundary_loops"] == 2
    assert hole_aware["hole_aware_checked"] is True
    assert hole_aware["hole_aware_feasible"] is False
    assert hole_aware["hole_aware_failure"] == "hole_outside_outer_loop"
    assert hole_aware_replacement["hole_aware_replacement_checked"] is True
    assert hole_aware_replacement["hole_aware_replacement_feasible"] is False
    assert (
        hole_aware_replacement["hole_aware_replacement_failure"]
        == "hole_outside_outer_loop"
    )
    assert classification == "reject_hole_aware_boundary_validation_failed"
    return {
        "boundary": boundary,
        "hole_aware": hole_aware,
        "hole_aware_replacement": hole_aware_replacement,
        "classification": classification,
    }


def run_hole_aware_touching_holes_failure_self_test() -> dict:
    tolerance = 1e-4
    edges = (
        packed_square_edges(0.0, 0.0, 5.0, tolerance)
        + packed_square_edges(1.0, 1.0, 1.0, tolerance)
        + packed_square_edges(2.0, 1.0, 1.0, tolerance)
    )
    boundary = boundary_loop_stats(edges)
    hole_aware = hole_aware_boundary_validation(
        edges,
        coord_tolerance=tolerance,
        seed_normal=(0.0, 0.0, 1.0),
    )
    hole_aware_replacement = hole_aware_replacement_feasibility(
        edges,
        coord_tolerance=tolerance,
        seed_normal=(0.0, 0.0, 1.0),
    )
    component = {
        "max_plane_distance": 0.0,
        "plane_distance_tolerance": tolerance,
        "max_normal_angle_degrees": 0.0,
        "effective_seed_normal_angle_degrees": 0.125,
    }
    classification = classify_candidate(
        component,
        boundary,
        reduction=10,
        source_is_truncated=False,
        replacement={"replacement_checked": False, "replacement_feasible": False},
        hole_aware=hole_aware,
    )

    assert boundary["bad_boundary_vertices"] > 0
    assert boundary["duplicate_boundary_edges"] > 0
    assert hole_aware["hole_aware_checked"] is True
    assert hole_aware["hole_aware_feasible"] is False
    assert hole_aware["hole_aware_failure"] == "loop_order_failed"
    assert hole_aware_replacement["hole_aware_replacement_checked"] is True
    assert hole_aware_replacement["hole_aware_replacement_feasible"] is False
    assert (
        hole_aware_replacement["hole_aware_replacement_failure"]
        == "loop_order_failed"
    )
    assert classification == "reject_boundary_not_closed_loops"
    return {
        "boundary": boundary,
        "hole_aware": hole_aware,
        "hole_aware_replacement": hole_aware_replacement,
        "classification": classification,
    }


def run_edge_incidence_only_self_test(tmp: Path) -> dict:
    path = tmp / "edge-only-grid.stl"
    write_test_stl(path, grid_plane_triangles(2, 2))
    result = run_report(make_report_args(path, edge_incidence_only=True))

    assert "build" not in result
    assert "top_components" not in result
    assert result["strict_edge_incidence"]["boundary_edges"] == 8
    assert result["strict_edge_incidence"]["nonmanifold_edges"] == 0
    assert result["strict_edge_incidence"]["misoriented_edges"] == 0
    assert result["strict_edge_incidence"]["watertight_edge_count"] is False
    return result


def run_validation_gate_self_test() -> dict:
    def incidence(
        boundary_edges: int,
        nonmanifold_edges: int,
        degenerate_triangles: int = 0,
        misoriented_edges: int = 0,
    ) -> dict:
        return {
            "boundary_edges": boundary_edges,
            "nonmanifold_edges": nonmanifold_edges,
            "degenerate_triangles": degenerate_triangles,
            "misoriented_edges": misoriented_edges,
        }

    cases = {
        "open_preserved": validation_gate(
            incidence(8, 0), incidence(8, 0)
        ),
        "open_improved": validation_gate(
            incidence(8, 0), incidence(4, 0)
        ),
        "watertight_to_open": validation_gate(
            incidence(0, 0), incidence(1, 0)
        ),
        "open_to_more_open": validation_gate(
            incidence(8, 0), incidence(10, 0)
        ),
        "nonmanifold_worse": validation_gate(
            incidence(0, 0), incidence(0, 1)
        ),
        "degenerate_preserved": validation_gate(
            incidence(0, 0, 2), incidence(0, 0, 2)
        ),
        "degenerate_improved": validation_gate(
            incidence(0, 0, 2), incidence(0, 0, 1)
        ),
        "degenerate_worse": validation_gate(
            incidence(0, 0, 0), incidence(0, 0, 1)
        ),
        "orientation_worse": validation_gate(
            incidence(0, 0, 0, 0), incidence(0, 0, 0, 1)
        ),
        "vertex_count_mismatch": validation_gate(
            incidence(0, 0),
            incidence(0, 0),
            vertex_count_mismatch=True,
        ),
        "normal_count_mismatch": validation_gate(
            incidence(0, 0),
            incidence(0, 0),
            normal_count_mismatch=True,
        ),
        "triangle_count_mismatch": validation_gate(
            incidence(0, 0),
            incidence(0, 0),
            expected_output_triangles=10,
            candidate_triangles=9,
        ),
    }

    assert cases["open_preserved"]["validation_rolled_back"] is False
    assert cases["open_preserved"]["validation_triangle_count_mismatch"] is False
    assert cases["open_preserved"]["output_boundary_edges"] == 8
    assert cases["open_improved"]["validation_rolled_back"] is False
    assert cases["open_improved"]["output_boundary_edges"] == 4
    assert cases["watertight_to_open"]["validation_topology_worse"] is True
    assert cases["watertight_to_open"]["validation_rolled_back"] is True
    assert cases["watertight_to_open"]["output_boundary_edges"] == 0
    assert cases["watertight_to_open"]["output_watertight"] is True
    assert cases["open_to_more_open"]["validation_rolled_back"] is True
    assert cases["open_to_more_open"]["output_boundary_edges"] == 8
    assert cases["nonmanifold_worse"]["validation_rolled_back"] is True
    assert cases["nonmanifold_worse"]["output_nonmanifold_edges"] == 0
    assert cases["degenerate_preserved"]["validation_rolled_back"] is False
    assert cases["degenerate_preserved"]["output_degenerate_triangles"] == 2
    assert cases["degenerate_improved"]["validation_rolled_back"] is False
    assert cases["degenerate_improved"]["output_degenerate_triangles"] == 1
    assert cases["degenerate_worse"]["validation_degenerate_worse"] is True
    assert cases["degenerate_worse"]["validation_rolled_back"] is True
    assert cases["degenerate_worse"]["output_degenerate_triangles"] == 0
    assert cases["orientation_worse"]["validation_orientation_worse"] is True
    assert cases["orientation_worse"]["validation_rolled_back"] is True
    assert cases["orientation_worse"]["output_misoriented_edges"] == 0
    assert (
        cases["vertex_count_mismatch"]["validation_vertex_count_mismatch"]
        is True
    )
    assert cases["vertex_count_mismatch"]["validation_rolled_back"] is True
    assert cases["vertex_count_mismatch"]["output_watertight"] is True
    assert (
        cases["normal_count_mismatch"]["validation_normal_count_mismatch"]
        is True
    )
    assert cases["normal_count_mismatch"]["validation_rolled_back"] is True
    assert cases["normal_count_mismatch"]["output_watertight"] is True
    assert (
        cases["triangle_count_mismatch"]["validation_triangle_count_mismatch"]
        is True
    )
    assert cases["triangle_count_mismatch"]["validation_rolled_back"] is True
    assert cases["triangle_count_mismatch"]["expected_output_triangles"] == 10
    assert cases["triangle_count_mismatch"]["candidate_triangles"] == 9
    assert cases["triangle_count_mismatch"]["output_watertight"] is True
    return cases


def run_source_buffer_preflight_self_test() -> dict:
    cases = {
        "valid": source_buffer_preflight(2, 18, 18),
        "vertex_short": source_buffer_preflight(2, 17, 18),
        "normal_short": source_buffer_preflight(2, 18, 17),
        "both_wrong": source_buffer_preflight(2, 19, 17),
    }

    assert cases["valid"]["source_buffers_valid"] is True
    assert cases["valid"]["source_expected_floats"] == 18
    assert cases["valid"]["source_vertex_count_mismatch"] is False
    assert cases["valid"]["source_normal_count_mismatch"] is False
    assert cases["vertex_short"]["source_buffers_valid"] is False
    assert cases["vertex_short"]["source_vertex_count_mismatch"] is True
    assert cases["vertex_short"]["source_normal_count_mismatch"] is False
    assert cases["normal_short"]["source_buffers_valid"] is False
    assert cases["normal_short"]["source_vertex_count_mismatch"] is False
    assert cases["normal_short"]["source_normal_count_mismatch"] is True
    assert cases["both_wrong"]["source_buffers_valid"] is False
    assert cases["both_wrong"]["source_vertex_count_mismatch"] is True
    assert cases["both_wrong"]["source_normal_count_mismatch"] is True
    return cases


def run_boundary_direction_self_test() -> dict:
    tolerance = 1e-4
    edges = packed_square_edges(0.0, 0.0, 1.0, tolerance)
    p00 = edges[0][0]
    p10 = edges[0][1]
    p11 = edges[1][1]
    p01 = edges[2][1]

    matching = [(p00, p10, p11), (p00, p11, p01)]
    flipped = flip_replacement_triangles(matching)

    assert replacement_boundary_matches(edges, flipped)
    assert not replacement_boundary_matches_direction(edges, flipped)

    repaired = orient_replacement_to_boundary(edges, flipped)
    assert repaired == matching
    assert replacement_boundary_matches_direction(edges, repaired)

    wrong_internal = [(p00, p10, p11), (p00, p01, p11)]
    wrong_incidence = replacement_edge_incidence(edges, wrong_internal)
    matching_incidence = replacement_edge_incidence(edges, matching)
    assert replacement_boundary_matches(edges, wrong_internal)
    assert wrong_incidence["boundary_matches"] is True
    assert wrong_incidence["misoriented_edges"] == 1
    assert wrong_incidence["valid"] is False
    assert matching_incidence["valid"] is True

    return {
        "canonical_boundary_matches_flipped": True,
        "directed_boundary_detects_flipped": True,
        "orientation_repaired": True,
        "canonical_boundary_matches_internal_misorientation": True,
        "local_edge_incidence_rejects_internal_misorientation": True,
    }


def run_trusted_neighbor_edge_self_test() -> dict:
    tolerance = 1e-4
    p00 = pack_vertex((0.0, 0.0, 0.0), tolerance)
    p10 = pack_vertex((1.0, 0.0, 0.0), tolerance)
    p11 = pack_vertex((1.0, 1.0, 0.0), tolerance)
    p01 = pack_vertex((0.0, 1.0, 0.0), tolerance)
    vertices = array("Q", [p00, p10, p11, p00, p11, p01])
    same_orientation_vertices = array("Q", [p00, p10, p11, p11, p00, p01])
    correct_neighbors = array("i", [NO_NEIGHBOR, NO_NEIGHBOR, 1,
                                    0, NO_NEIGHBOR, NO_NEIGHBOR])
    wrong_edge_neighbors = array("i", [NO_NEIGHBOR, NO_NEIGHBOR, 1,
                                       NO_NEIGHBOR, 0, NO_NEIGHBOR])
    same_orientation_neighbors = array("i", [NO_NEIGHBOR, NO_NEIGHBOR, 1,
                                             0, NO_NEIGHBOR, NO_NEIGHBOR])
    wrong_size_neighbors = array("i", [NO_NEIGHBOR, NO_NEIGHBOR, 1])
    range_neighbors = array("i", [2, NO_NEIGHBOR, 1,
                                  0, NO_NEIGHBOR, NO_NEIGHBOR])
    self_neighbors = array("i", [0, NO_NEIGHBOR, 1,
                                 0, NO_NEIGHBOR, NO_NEIGHBOR])
    duplicate_neighbors = array("i", [1, 1, NO_NEIGHBOR,
                                      0, NO_NEIGHBOR, NO_NEIGHBOR])
    asymmetric_neighbors = array("i", [NO_NEIGHBOR, NO_NEIGHBOR, 1,
                                       NO_NEIGHBOR, NO_NEIGHBOR, NO_NEIGHBOR])

    correct_slots, correct_validation = trusted_neighbor_reciprocal_slots(
        correct_neighbors, 2
    )
    wrong_slots, wrong_validation = trusted_neighbor_reciprocal_slots(
        wrong_edge_neighbors, 2
    )
    same_orientation_slots, same_orientation_validation = (
        trusted_neighbor_reciprocal_slots(same_orientation_neighbors, 2)
    )
    _, wrong_size_validation = trusted_neighbor_reciprocal_slots(
        wrong_size_neighbors, 2
    )
    _, range_validation = trusted_neighbor_reciprocal_slots(
        range_neighbors, 2
    )
    _, self_validation = trusted_neighbor_reciprocal_slots(
        self_neighbors, 2
    )
    _, duplicate_validation = trusted_neighbor_reciprocal_slots(
        duplicate_neighbors, 2
    )
    _, asymmetric_validation = trusted_neighbor_reciprocal_slots(
        asymmetric_neighbors, 2
    )
    correct_mismatches = trusted_neighbor_edge_mismatches(
        vertices, correct_neighbors
    )
    wrong_edge_mismatches = trusted_neighbor_edge_mismatches(
        vertices, wrong_edge_neighbors
    )
    same_orientation_validation_result = trusted_neighbor_edge_validation_from_slots(
        same_orientation_vertices, same_orientation_neighbors, None
    )
    correct_cached_mismatches = trusted_neighbor_edge_mismatches_from_slots(
        vertices, correct_neighbors, correct_slots
    )
    wrong_cached_mismatches = trusted_neighbor_edge_mismatches_from_slots(
        vertices, wrong_edge_neighbors, wrong_slots
    )
    same_orientation_cached_validation = (
        trusted_neighbor_edge_validation_from_slots(
            same_orientation_vertices,
            same_orientation_neighbors,
            same_orientation_slots,
        )
    )

    assert correct_validation["asymmetry"] is False
    assert wrong_validation["asymmetry"] is False
    assert same_orientation_validation["asymmetry"] is False
    assert wrong_size_validation["range"] is True
    assert wrong_size_validation["open_slot"] is False
    assert range_validation["range"] is True
    assert range_validation["open_slot"] is True
    assert self_validation["self"] is True
    assert self_validation["open_slot"] is True
    assert duplicate_validation["duplicate"] is True
    assert duplicate_validation["open_slot"] is True
    assert asymmetric_validation["asymmetry"] is True
    assert asymmetric_validation["open_slot"] is True
    assert correct_slots[2] == 0
    assert correct_slots[3] == 2
    assert wrong_slots[2] == 1
    assert wrong_slots[4] == 2
    assert correct_mismatches == 0
    assert wrong_edge_mismatches == 2
    assert same_orientation_validation_result["edge_mismatches"] == 0
    assert same_orientation_validation_result["orientation_mismatches"] == 2
    assert correct_cached_mismatches == correct_mismatches
    assert wrong_cached_mismatches == wrong_edge_mismatches
    assert same_orientation_cached_validation == same_orientation_validation_result

    return {
        "correct_edge_mismatches": correct_mismatches,
        "wrong_edge_mismatches": wrong_edge_mismatches,
        "same_orientation_edge_mismatches": same_orientation_validation_result[
            "edge_mismatches"
        ],
        "same_orientation_mismatches": same_orientation_validation_result[
            "orientation_mismatches"
        ],
        "correct_cached_edge_mismatches": correct_cached_mismatches,
        "wrong_cached_edge_mismatches": wrong_cached_mismatches,
        "same_orientation_cached_mismatches": (
            same_orientation_cached_validation["orientation_mismatches"]
        ),
        "correct_reciprocal_slots": [correct_slots[2], correct_slots[3]],
        "wrong_reciprocal_slots": [wrong_slots[2], wrong_slots[4]],
        "same_orientation_reciprocal_slots": [
            same_orientation_slots[2],
            same_orientation_slots[3],
        ],
        "reciprocal_triangle_links_are_not_enough": True,
        "cached_reciprocal_slots_match_scan": (
            correct_cached_mismatches == correct_mismatches
            and wrong_cached_mismatches == wrong_edge_mismatches
            and same_orientation_cached_validation
            == same_orientation_validation_result
        ),
        "wrong_size_rejected": wrong_size_validation["range"],
        "range_rejected": range_validation["range"],
        "self_rejected": self_validation["self"],
        "duplicate_rejected": duplicate_validation["duplicate"],
        "asymmetry_rejected": asymmetric_validation["asymmetry"],
    }


def run_trusted_provenance_gate_self_test() -> dict:
    triangle_count = 4
    clean = {
        "triangles": triangle_count,
        "complete_triangles": triangle_count,
        "unknown_triangles": 0,
        "watertight": True,
        "raw_boundary_edges": 0,
        "raw_nonmanifold_edges": 0,
        "raw_misoriented_edges": 0,
        "misoriented_edges": 0,
        "raw_unique_edges": 6,
        "welded_unique_edges": 6,
        "raw_twin_edge_slots": 12,
        "welded_twin_edge_slots": 12,
        "raw_boundary_edge_slots": 0,
        "welded_boundary_edge_slots": 0,
        "raw_nonmanifold_edge_slots": 0,
        "welded_nonmanifold_edge_slots": 0,
        "raw_grid_grid_welded_spread_edges": 0,
        "raw_center_involved_welded_spread_edges": 0,
        "raw_grid_vertex_welded_spread_keys": 0,
        "raw_center_vertex_welded_spread_keys": 0,
    }

    clean_gate = trusted_provenance_gate(
        clean, triangle_count, neighbor_slots=triangle_count * 3
    )

    raw_orientation = dict(clean)
    raw_orientation["raw_misoriented_edges"] = 1
    raw_orientation_gate = trusted_provenance_gate(
        raw_orientation, triangle_count, neighbor_slots=triangle_count * 3
    )

    welded_orientation = dict(clean)
    welded_orientation["misoriented_edges"] = 1
    welded_orientation_gate = trusted_provenance_gate(
        welded_orientation,
        triangle_count,
        cached_neighbors_raw=False,
        neighbor_slots=triangle_count * 3,
    )

    edge_mismatch_gate = trusted_provenance_gate(
        clean,
        triangle_count,
        neighbor_slots=triangle_count * 3,
        neighbor_edge_mismatches=1,
    )
    neighbor_orientation_gate = trusted_provenance_gate(
        clean,
        triangle_count,
        neighbor_slots=triangle_count * 3,
        neighbor_orientation_mismatches=1,
    )

    assert clean_gate["eligible"] is True
    assert raw_orientation_gate["eligible"] is False
    assert raw_orientation_gate["rejected_orientation"] is True
    assert raw_orientation_gate["rejected_raw_topology"] is True
    assert welded_orientation_gate["eligible"] is False
    assert welded_orientation_gate["rejected_orientation"] is True
    assert welded_orientation_gate["rejected_raw_topology"] is False
    assert edge_mismatch_gate["eligible"] is False
    assert edge_mismatch_gate["rejected_neighbor_edge_mismatch"] is True
    assert neighbor_orientation_gate["eligible"] is False
    assert neighbor_orientation_gate["rejected_orientation"] is True

    return {
        "clean_eligible": clean_gate["eligible"],
        "raw_orientation_rejected": raw_orientation_gate[
            "rejected_orientation"
        ],
        "raw_orientation_rejected_raw_topology": raw_orientation_gate[
            "rejected_raw_topology"
        ],
        "welded_orientation_rejected": welded_orientation_gate[
            "rejected_orientation"
        ],
        "edge_mismatch_rejected": edge_mismatch_gate[
            "rejected_neighbor_edge_mismatch"
        ],
        "neighbor_orientation_rejected": neighbor_orientation_gate[
            "rejected_orientation"
        ],
    }


def run_self_test() -> dict:
    with tempfile.TemporaryDirectory() as tmp_name:
        tmp = Path(tmp_name)
        simple = run_simple_plane_self_test(tmp)
        concave = run_concave_plane_self_test(tmp)
        hole_aware = run_hole_aware_self_test(tmp)
        hole_aware_two_hole = run_hole_aware_two_hole_self_test()
        vertical_replacement = run_vertical_replacement_self_test()
        sloped_replacement = run_sloped_replacement_self_test()
        bad_boundary = run_bad_boundary_self_test()
        hole_aware_failure = run_hole_aware_failure_self_test()
        hole_aware_same_direction_failure = (
            run_hole_aware_same_direction_failure_self_test()
        )
        hole_aware_nested_failure = run_hole_aware_nested_failure_self_test()
        hole_aware_straddling_failure = (
            run_hole_aware_straddling_failure_self_test()
        )
        hole_aware_intersecting_holes_failure = (
            run_hole_aware_intersecting_holes_failure_self_test()
        )
        hole_aware_touching_outer_failure = (
            run_hole_aware_touching_outer_failure_self_test()
        )
        hole_aware_touching_holes_failure = (
            run_hole_aware_touching_holes_failure_self_test()
        )
        edge_incidence_only = run_edge_incidence_only_self_test(tmp)
        validation_gate_result = run_validation_gate_self_test()
        source_buffer_preflight = run_source_buffer_preflight_self_test()
        boundary_direction = run_boundary_direction_self_test()
        trusted_neighbor_edge = run_trusted_neighbor_edge_self_test()
        trusted_provenance_gate_result = run_trusted_provenance_gate_self_test()

    return {
        "simple_plane": simple,
        "concave_plane": concave,
        "hole_aware": hole_aware,
        "hole_aware_two_hole": hole_aware_two_hole,
        "vertical_replacement": vertical_replacement,
        "sloped_replacement": sloped_replacement,
        "bad_boundary": bad_boundary,
        "hole_aware_failure": hole_aware_failure,
        "hole_aware_same_direction_failure": hole_aware_same_direction_failure,
        "hole_aware_nested_failure": hole_aware_nested_failure,
        "hole_aware_straddling_failure": hole_aware_straddling_failure,
        "hole_aware_intersecting_holes_failure": (
            hole_aware_intersecting_holes_failure
        ),
        "hole_aware_touching_outer_failure": hole_aware_touching_outer_failure,
        "hole_aware_touching_holes_failure": hole_aware_touching_holes_failure,
        "edge_incidence_only": edge_incidence_only,
        "validation_gate": validation_gate_result,
        "source_buffer_preflight": source_buffer_preflight,
        "boundary_direction": boundary_direction,
        "trusted_neighbor_edge": trusted_neighbor_edge,
        "trusted_provenance_gate": trusted_provenance_gate_result,
    }


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if args.self_test:
        result = run_self_test()
    else:
        result = run_report(args)

    text = json.dumps(result, indent=2, sort_keys=True)
    if args.output_json:
        args.output_json.write_text(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
