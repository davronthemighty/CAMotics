/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2022 Joseph Coffland
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#include "ReductionEligibility.h"

#include <camotics/GeometrySafetyInternal.h>
#include <camotics/SHA256.h>

#include <cbang/Exception.h>
#include <cbang/net/Base64.h>

#include <algorithm>
#include <cmath>
#include <set>

using namespace std;
using namespace CAMotics;


namespace {
  template <typename T>
  void hashValue(SHA256 &sha, const T &value) {
    sha.update(reinterpret_cast<const uint8_t *>(&value), sizeof(value));
  }


  template <typename T>
  bool sortedUnique(const vector<T> &values) {
    for (size_t i = 1; i < values.size(); i++)
      if (!(values[i - 1] < values[i])) return false;
    return true;
  }


  bool quantizeVertex(float x, float y, float z, double tolerance,
                      ReductionLockedVertex &vertex) {
    return Internal::quantizeCoordinate(x, tolerance, vertex.x) &&
      Internal::quantizeCoordinate(y, tolerance, vertex.y) &&
      Internal::quantizeCoordinate(z, tolerance, vertex.z);
  }
}


ReductionLockedVertex CAMotics::quantizeReductionVertex
(float x, float y, float z, double tolerance) {
  ReductionLockedVertex vertex;
  if (!quantizeVertex(x, y, z, tolerance, vertex))
    THROW("Reduction vertex is outside the quantization range.");
  return vertex;
}


uint64_t ReductionEligibility::count(ReductionTriangleOrigin origin) const {
  return std::count(triangleOrigins.begin(), triangleOrigins.end(),
                    (uint8_t)origin);
}


string ReductionEligibility::computeBindingHash
(const vector<float> &vertices) const {
  SHA256 sha;
  static const string prefix = "CAMotics sparse reduction eligibility v1";
  sha.update(prefix);
  hashValue(sha, quantizationTolerance);

  uint64_t size = vertices.size();
  hashValue(sha, size);
  if (!vertices.empty())
    sha.update(reinterpret_cast<const uint8_t *>(vertices.data()),
               vertices.size() * sizeof(float));

  size = triangleOrigins.size();
  hashValue(sha, size);
  if (!triangleOrigins.empty())
    sha.update(triangleOrigins.data(), triangleOrigins.size());

  size = lockedSeamVertices.size();
  hashValue(sha, size);
  for (const auto &vertex: lockedSeamVertices) {
    hashValue(sha, vertex.x);
    hashValue(sha, vertex.y);
    hashValue(sha, vertex.z);
  }

  size = lockedSeamEdges.size();
  hashValue(sha, size);
  for (const auto &edge: lockedSeamEdges) {
    hashValue(sha, edge.a.x);
    hashValue(sha, edge.a.y);
    hashValue(sha, edge.a.z);
    hashValue(sha, edge.b.x);
    hashValue(sha, edge.b.y);
    hashValue(sha, edge.b.z);
  }

  return cb::Base64().encode(sha.finalize());
}


void ReductionEligibility::seal(const vector<float> &vertices) {
  bindingHash = computeBindingHash(vertices);
}


bool ReductionEligibility::validFor(const vector<float> &vertices) const {
  if (!isfinite(quantizationTolerance) || quantizationTolerance <= 0)
    return false;
  if (vertices.size() % 9) return false;
  if (triangleOrigins.size() != vertices.size() / 9) return false;
  if (bindingHash.empty() || bindingHash != computeBindingHash(vertices))
    return false;
  if (!sortedUnique(lockedSeamVertices) || !sortedUnique(lockedSeamEdges))
    return false;

  for (uint8_t origin: triangleOrigins)
    if (REDUCTION_UNKNOWN_LOCKED < origin) return false;
  if (count(REDUCTION_UNKNOWN_LOCKED)) return false;

  set<ReductionLockedVertex> surfaceVertices;
  set<ReductionLockedEdge> surfaceEdges;
  for (size_t offset = 0; offset < vertices.size(); offset += 9) {
    ReductionLockedVertex points[3];
    for (unsigned i = 0; i < 3; i++) {
      size_t vertexOffset = offset + (size_t)i * 3;
      if (!quantizeVertex(vertices[vertexOffset], vertices[vertexOffset + 1],
                          vertices[vertexOffset + 2],
                          quantizationTolerance, points[i]))
        return false;
      surfaceVertices.insert(points[i]);
    }
    for (unsigned i = 0; i < 3; i++)
      surfaceEdges.insert(ReductionLockedEdge(points[i], points[(i + 1) % 3]));
  }

  set<ReductionLockedVertex> lockedVertices
    (lockedSeamVertices.begin(), lockedSeamVertices.end());
  for (const auto &vertex: lockedSeamVertices)
    if (!surfaceVertices.count(vertex)) return false;
  for (const auto &edge: lockedSeamEdges)
    if (!lockedVertices.count(edge.a) || !lockedVertices.count(edge.b) ||
        !surfaceEdges.count(edge)) return false;

  return true;
}
