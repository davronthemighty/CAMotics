/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2022 Joseph Coffland
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#include "PlanarReductionInternal.h"

#include <camotics/GeometrySafetyInternal.h>

#include <algorithm>
#include <limits>
#include <set>
#include <stdexcept>
#include <unordered_map>

using namespace std;


namespace CAMotics {
namespace PlanarReductionInternal {
  VertexKey packVertex(const vector<float> &vertices, uint64_t offset,
                       double tolerance) {
    VertexKey key;
    if (!Internal::quantizeCoordinate
        (vertices[offset + 0], tolerance, key.x) ||
        !Internal::quantizeCoordinate
        (vertices[offset + 1], tolerance, key.y) ||
        !Internal::quantizeCoordinate
        (vertices[offset + 2], tolerance, key.z))
      throw invalid_argument("Vertex is outside the quantization range.");
    return key;
  }


  uint32_t getVertexId
    (const VertexKey &key, const Vec3 &point,
     unordered_map<VertexKey, uint32_t, VertexKeyHash> &vertexIds,
     vector<Vec3> &points) {
    auto it = vertexIds.find(key);
    if (it != vertexIds.end()) return it->second;

    if (std::numeric_limits<uint32_t>::max() <= points.size())
      throw overflow_error("Planar reduction vertex index overflow.");
    uint32_t id = (uint32_t)points.size();
    vertexIds[key] = id;
    points.push_back(point);
    return id;
  }


  uint32_t getVertexId
    (const VertexKey &key,
     unordered_map<VertexKey, uint32_t, VertexKeyHash> &vertexIds) {
    auto it = vertexIds.find(key);
    if (it != vertexIds.end()) return it->second;

    if (std::numeric_limits<uint32_t>::max() <= vertexIds.size())
      throw overflow_error("Planar reduction vertex index overflow.");
    uint32_t id = (uint32_t)vertexIds.size();
    vertexIds[key] = id;
    return id;
  }


  EdgeIncidenceReport validateEdgeIncidenceVertices
    (const vector<float> &vertices, const PlanarReductionConfig &config) {
    EdgeIncidenceReport report;
    unordered_map<VertexKey, uint32_t, VertexKeyHash> vertexIds;
    unordered_map<EdgeKey, EdgeCount, EdgeKeyHash> edgeCounts;
    set<array<uint32_t, 3>> triangleKeys;

    uint64_t triangleCount = vertices.size() / 9;
    vertexIds.reserve((size_t)triangleCount);
    edgeCounts.reserve((size_t)triangleCount * 3 / 2);

    for (uint64_t tri = 0; tri < triangleCount; tri++) {
      uint64_t offset = tri * 9;
      array<uint32_t, 3> ids;
      for (unsigned i = 0; i < 3; i++)
        ids[i] = getVertexId
          (packVertex(vertices, offset + (uint64_t)i * 3,
                      config.coordTolerance),
           vertexIds);

      if (ids[0] == ids[1] || ids[1] == ids[2] || ids[2] == ids[0]) {
        report.degenerateTriangles++;
        continue;
      }

      array<uint32_t, 3> triangleKey = ids;
      sort(triangleKey.begin(), triangleKey.end());
      if (!triangleKeys.insert(triangleKey).second)
        report.duplicateTriangles++;

      for (unsigned slot = 0; slot < 3; slot++) {
        auto edge = getTriangleEdge(ids, slot);
        EdgeKey key(edge.first, edge.second);
        EdgeCount &count = edgeCounts[key];
        count.count++;
        if (isForwardEdge(edge, key)) count.forwardCount++;
      }
    }

    for (const auto &entry: edgeCounts) {
      if (entry.second.count == 1) report.boundaryEdges++;
      else if (2 < entry.second.count) report.nonManifoldEdges++;
    }
    report.misorientedEdges = countMisorientedEdges(edgeCounts);

    return report;
  }

  void appendTriangle(vector<float> &vertices, vector<float> &normals,
                      const Vec3 &a, const Vec3 &b, const Vec3 &c,
                      const Vec3 &normal) {
    const Vec3 points[3] = {a, b, c};

    for (unsigned i = 0; i < 3; i++) {
      vertices.push_back((float)points[i].x);
      vertices.push_back((float)points[i].y);
      vertices.push_back((float)points[i].z);
      normals.push_back((float)normal.x);
      normals.push_back((float)normal.y);
      normals.push_back((float)normal.z);
    }
  }


  void appendOriginalTriangle(vector<float> &vertices, vector<float> &normals,
                              const vector<float> &sourceVertices,
                              const vector<float> &sourceNormals,
                              uint64_t tri) {
    uint64_t offset = tri * 9;
    vertices.insert(vertices.end(), sourceVertices.begin() + offset,
                    sourceVertices.begin() + offset + 9);
    normals.insert(normals.end(), sourceNormals.begin() + offset,
                   sourceNormals.begin() + offset + 9);
  }


  void appendReducedTriangles(vector<float> &vertices, vector<float> &normals,
                              const vector<array<uint32_t, 3> > &replacement,
                              const vector<Vec3> &points,
                              const Vec3 &seedNormal) {
    for (const auto &tri: replacement) {
      const Vec3 &a = points[tri[0]];
      const Vec3 &b = points[tri[1]];
      const Vec3 &c = points[tri[2]];
      Vec3 normal = normalize(cross(b - a, c - a));
      if (dot(normal, seedNormal) < 0) normal =
        Vec3{-normal.x, -normal.y, -normal.z};

      appendTriangle(vertices, normals, a, b, c, normal);
    }
  }


  void releaseMeshData(MeshData &mesh) {
    releaseVector(mesh.triangles);
    releaseVector(mesh.normals);
    releaseVector(mesh.points);
    releaseVector(mesh.neighbors);
    releaseMap(mesh.edgeInfo);
  }


  void releaseMeshConnectivity(MeshData &mesh) {
    releaseVector(mesh.triangles);
    releaseVector(mesh.normals);
    releaseVector(mesh.neighbors);
    releaseMap(mesh.edgeInfo);
  }


  TrustedNeighborValidation validateTrustedNeighbors
    (const vector<int32_t> &neighbors, uint64_t triangleCount,
     vector<uint8_t> *reciprocalSlots) {
    TrustedNeighborValidation validation;
    if (triangleCount > (uint64_t)numeric_limits<int32_t>::max()) {
      validation.range = true;
      return validation;
    }

    if (triangleCount > (uint64_t)(numeric_limits<size_t>::max() / 3) ||
        neighbors.size() != (size_t)triangleCount * 3) {
      validation.range = true;
      return validation;
    }

    if (reciprocalSlots)
      reciprocalSlots->assign(neighbors.size(),
                              INVALID_TRUSTED_RECIPROCAL_SLOT);

    for (uint64_t tri = 0; tri < triangleCount; tri++) {
      int32_t triNeighbors[3];
      for (unsigned slot = 0; slot < 3; slot++) {
        size_t index = (size_t)tri * 3 + slot;
        int32_t neighbor = neighbors[(size_t)tri * 3 + slot];
        triNeighbors[slot] = neighbor;

        if (neighbor < 0) {
          validation.openSlot = true;
          continue;
        }

        if ((uint64_t)neighbor >= triangleCount) {
          validation.range = true;
          continue;
        }

        if ((uint64_t)neighbor == tri) {
          validation.self = true;
          continue;
        }

        unsigned reciprocalSlot = 3;
        size_t base = (size_t)neighbor * 3;
        for (unsigned otherSlot = 0; otherSlot < 3; otherSlot++)
          if (neighbors[base + otherSlot] == (int32_t)tri) {
            reciprocalSlot = otherSlot;
            break;
          }

        if (reciprocalSlot == 3) validation.asymmetry = true;
        else if (reciprocalSlots)
          (*reciprocalSlots)[index] = (uint8_t)reciprocalSlot;
      }

      if (0 <= triNeighbors[0] && triNeighbors[0] == triNeighbors[1])
        validation.duplicate = true;
      if (0 <= triNeighbors[1] && triNeighbors[1] == triNeighbors[2])
        validation.duplicate = true;
      if (0 <= triNeighbors[2] && triNeighbors[2] == triNeighbors[0])
        validation.duplicate = true;
    }

    return validation;
  }


  TrustedNeighborValidation validateTrustedNeighborEdges
    (const MeshData &mesh, const vector<int32_t> &neighbors,
     const vector<uint8_t> *reciprocalSlots) {
    TrustedNeighborValidation validation;
    if (neighbors.size() != (size_t)mesh.triangleCount * 3) {
      validation.range = true;
      return validation;
    }

    for (uint64_t tri = 0; tri < mesh.triangleCount; tri++) {
      for (unsigned slot = 0; slot < 3; slot++) {
        int32_t neighbor = neighbors[(size_t)tri * 3 + slot];
        if (neighbor < 0 || (uint64_t)neighbor >= mesh.triangleCount)
          continue;

        auto edgePair = getTriangleEdge(mesh.triangles[(size_t)tri], slot);
        EdgeKey edge(edgePair.first, edgePair.second);
        bool reciprocal = false;
        bool matchingReciprocal = false;
        bool matchingOppositeOrientation = false;
        size_t index = (size_t)tri * 3 + slot;

        if (reciprocalSlots && reciprocalSlots->size() == neighbors.size() &&
            (*reciprocalSlots)[index] < 3) {
          unsigned otherSlot = (*reciprocalSlots)[index];
          auto otherEdge =
            getTriangleEdge(mesh.triangles[(size_t)neighbor], otherSlot);
          if (edge == EdgeKey(otherEdge.first, otherEdge.second)) {
            matchingReciprocal = true;
            matchingOppositeOrientation =
              edgePair.first == otherEdge.second &&
              edgePair.second == otherEdge.first;
          }

          reciprocal = true;

        } else {
          size_t base = (size_t)neighbor * 3;
          for (unsigned otherSlot = 0; otherSlot < 3; otherSlot++) {
            if (neighbors[base + otherSlot] != (int32_t)tri) continue;

            reciprocal = true;
            auto otherEdge =
              getTriangleEdge(mesh.triangles[(size_t)neighbor], otherSlot);
            if (edge == EdgeKey(otherEdge.first, otherEdge.second)) {
              matchingReciprocal = true;
              matchingOppositeOrientation =
                edgePair.first == otherEdge.second &&
                edgePair.second == otherEdge.first;
            }
          }
        }

        if (reciprocal && !matchingReciprocal) {
          validation.edgeMismatch = true;
          validation.edgeMismatches++;
        } else if (matchingReciprocal && !matchingOppositeOrientation) {
          validation.orientation = true;
          validation.orientationMismatches++;
        }
      }
    }

    return validation;
  }



}
}
