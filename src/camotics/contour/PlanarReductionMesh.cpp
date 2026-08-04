/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2022 Joseph Coffland
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#include "PlanarReduction.h"
#include "PlanarReductionInternal.h"

#include "ContourProvenance.h"
#include "Surface.h"
#include "TriangleSurface.h"

#include <camotics/GeometrySafetyInternal.h>
#include <camotics/Profile.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <deque>
#include <exception>
#include <queue>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std;
using namespace CAMotics;
using namespace CAMotics::PlanarReductionInternal;


namespace {
  const double PI = 3.141592653589793238462643383279502884;


  uint64_t mixFingerprint(uint64_t fingerprint, uint64_t value) {
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
    value ^= value >> 31;
    return fingerprint ^ (value + 0x9e3779b97f4a7c15ULL +
                          (fingerprint << 6) + (fingerprint >> 2));
  }


  void addUnorderedFingerprint(uint64_t &fingerprint, uint64_t value) {
    uint64_t mixed = mixFingerprint(0, value);
    fingerprint += mixed;
    fingerprint += (mixed << 32) | (mixed >> 32);
    fingerprint += 0x9e3779b97f4a7c15ULL;
  }


  void mixBoundaryEdge(uint64_t &fingerprint,
                       const pair<uint32_t, uint32_t> &edge) {
    EdgeKey key(edge.first, edge.second);
    uint64_t edgeFingerprint = 0;
    edgeFingerprint = mixFingerprint(edgeFingerprint, key.a);
    edgeFingerprint = mixFingerprint(edgeFingerprint, key.b);
    addUnorderedFingerprint(fingerprint, edgeFingerprint);
  }


  struct BoundaryInfo {
    uint64_t vertices = 0;
    uint64_t edges = 0;
    uint64_t loops = 0;
    uint64_t badVertices = 0;
    uint64_t duplicateEdges = 0;
  };


  struct ComponentPairKey {
    size_t a = 0;
    size_t b = 0;

    ComponentPairKey() {}
    ComponentPairKey(size_t a, size_t b) :
      a(std::min(a, b)), b(std::max(a, b)) {}

    bool operator==(const ComponentPairKey &o) const {
      return a == o.a && b == o.b;
    }
  };


  struct ComponentPairKeyHash {
    size_t operator()(const ComponentPairKey &key) const {
      uint64_t a = (uint64_t)key.a;
      uint64_t b = (uint64_t)key.b;
      uint64_t value = a ^ (b + 0x9e3779b97f4a7c15ULL +
                            (a << 6) + (a >> 2));
      value ^= value >> 33;
      value *= 0xff51afd7ed558ccdULL;
      value ^= value >> 33;
      value *= 0xc4ceb9fe1a85ec53ULL;
      value ^= value >> 33;
      return (size_t)value;
    }
  };


  bool shouldRollbackCandidateValidation
    (uint64_t inputBoundaryEdges, uint64_t inputNonManifoldEdges,
     uint64_t inputMisorientedEdges, uint64_t inputDegenerateTriangles,
     bool vertexCountMismatch, bool normalCountMismatch,
     bool triangleCountMismatch, const EdgeIncidenceReport &validation,
     bool &topologyWorse, bool &degenerateWorse, bool &orientationWorse) {
    topologyWorse =
      inputBoundaryEdges < validation.boundaryEdges ||
      inputNonManifoldEdges < validation.nonManifoldEdges;
    degenerateWorse =
      inputDegenerateTriangles < validation.degenerateTriangles;
    orientationWorse =
      inputMisorientedEdges < validation.misorientedEdges;

    return vertexCountMismatch || normalCountMismatch ||
      triangleCountMismatch || orientationWorse ||
      topologyWorse || degenerateWorse;
  }


  struct ComponentSummary {
    bool available = false;
    uint64_t components = 0;
    uint64_t estimatedTrianglesAfter = 0;
    uint64_t estimatedTriangleReduction = 0;
    uint64_t componentDecisionFingerprint = 0;
    uint64_t decisionBearingComponents = 0;
    uint64_t decisionBearingTriangles = 0;

    uint64_t phase1Components = 0;
    uint64_t phase1SourceTriangles = 0;
    uint64_t phase1EstimatedReduction = 0;

    uint64_t holeAwareComponents = 0;
    uint64_t holeAwareSourceTriangles = 0;
    uint64_t holeAwareEstimatedReduction = 0;

    uint64_t estimatedReplacementChecks = 0;
    uint64_t feasibleReplacementChecks = 0;
    uint64_t writableReplacementChecks = 0;
    uint64_t unwritableReplacementChecks = 0;
    uint64_t phase1WritableReplacementChecks = 0;
    uint64_t holeAwareWritableReplacementChecks = 0;
    uint64_t phase1UnwritableReplacementChecks = 0;
    uint64_t holeAwareUnwritableReplacementChecks = 0;
    uint64_t replacementEdgeIncidenceChecks = 0;
    uint64_t replacementEdgeIncidenceRejected = 0;
    uint64_t phase1ReplacementEdgeIncidenceRejected = 0;
    uint64_t holeAwareReplacementEdgeIncidenceRejected = 0;

    uint64_t rejectedBoundaryComponents = 0;
    uint64_t rejectedBoundaryTriangles = 0;

    uint64_t rejectedNoSavingsComponents = 0;
    uint64_t rejectedNoSavingsTriangles = 0;

    uint64_t rejectedTriangulationComponents = 0;
    uint64_t rejectedTriangulationTriangles = 0;
  };


  struct ReplacementComponent {
    uint64_t owner = 0;
    vector<array<uint32_t, 3> > triangles;
    Vec3 normal;
  };


  typedef pair<uint32_t, uint32_t> BoundaryEdge;


  void prepareBoundaryEdges(vector<BoundaryEdge> &boundaryEdges,
                            uint64_t componentSize) {
    const size_t maxRetainedEdges = 1u << 20;

    if (maxRetainedEdges < boundaryEdges.capacity() &&
        componentSize < maxRetainedEdges / 4)
      vector<BoundaryEdge>().swap(boundaryEdges);
    else
      boundaryEdges.clear();

    if (boundaryEdges.capacity() < componentSize)
      boundaryEdges.reserve((size_t)componentSize);
  }


  struct BoundaryCoSimplifyEstimate {
    bool available = false;
    uint64_t loops = 0;
    uint64_t boundaryVertices = 0;
    uint64_t simplifiedBoundaryVertices = 0;
    uint64_t trianglesAfter = 0;
    uint64_t trianglesAfterSimplified = 0;
    uint64_t extraReduction = 0;
  };


  struct VertexContractStats {
    uint64_t useCount = 0;
    uint64_t eligibleUseCount = 0;
    uint64_t removableUseCount = 0;
  };


  struct ComponentRecord {
    uint64_t id = 0;
    vector<uint64_t> triangles;
    vector<BoundaryEdge> boundaryEdges;
    vector<vector<uint32_t> > loops;
    Vec3 normal;
    unsigned sideId = PLANAR_REDUCTION_SIDE_CUT;
    BoundaryInfo boundary;
    BoundaryCoSimplifyEstimate boundaryCoSimplify;
    ReplacementCheck replacement;
    ReplacementCheck contractReplacement;
    uint64_t estimatedAfter = 0;
    bool canApplyReplacement = false;
    bool contractEligible = false;
    bool contractAffected = false;
    bool contractAccepted = false;
  };


  struct PendingReplacement {
    uint64_t id = 0;
    vector<uint64_t> triangles;
    vector<BoundaryEdge> boundaryEdges;
    vector<vector<uint32_t> > loops;
    Vec3 normal;
    unsigned sideId = PLANAR_REDUCTION_SIDE_CUT;
    BoundaryInfo boundary;
    BoundaryCoSimplifyEstimate boundaryCoSimplify;
    uint64_t estimatedAfter = 0;
    uint64_t boundaryFingerprint = 0;
    bool canApplyReplacement = false;
    bool loopsOrdered = false;
    ReplacementCheck replacement;
  };


  void recordComponentDecisionFingerprint
    (uint64_t &fingerprint, uint64_t componentTriangles,
     uint64_t boundaryFingerprint, const BoundaryInfo &boundary,
     uint64_t estimatedAfter, uint64_t reduction,
     const ReplacementCheck &replacement) {
    uint64_t componentFingerprint = 0;
    componentFingerprint =
      mixFingerprint(componentFingerprint, componentTriangles);
    componentFingerprint =
      mixFingerprint(componentFingerprint, boundaryFingerprint);
    componentFingerprint =
      mixFingerprint(componentFingerprint, boundary.vertices);
    componentFingerprint = mixFingerprint(componentFingerprint, boundary.edges);
    componentFingerprint = mixFingerprint(componentFingerprint, boundary.loops);
    componentFingerprint =
      mixFingerprint(componentFingerprint, boundary.badVertices);
    componentFingerprint =
      mixFingerprint(componentFingerprint, boundary.duplicateEdges);
    componentFingerprint = mixFingerprint(componentFingerprint, estimatedAfter);
    componentFingerprint = mixFingerprint(componentFingerprint, reduction);
    componentFingerprint =
      mixFingerprint(componentFingerprint, replacement.checked ? 1 : 0);
    componentFingerprint =
      mixFingerprint(componentFingerprint,
                     replacement.estimateAvailable ? 1 : 0);
    componentFingerprint =
      mixFingerprint(componentFingerprint, replacement.feasible ? 1 : 0);
    componentFingerprint =
      mixFingerprint(componentFingerprint,
                     replacement.edgeIncidenceChecked ? 1 : 0);
    componentFingerprint =
      mixFingerprint(componentFingerprint, replacement.edgeIncidenceOk ? 1 : 0);
    componentFingerprint =
      mixFingerprint(componentFingerprint, replacement.trianglesAfter);
    componentFingerprint =
      mixFingerprint(componentFingerprint,
                     (uint64_t)replacement.triangles.size());

    uint64_t replacementFingerprint = 0;

    for (const auto &tri: replacement.triangles) {
      array<uint32_t, 3> ids = tri;
      sort(ids.begin(), ids.end());
      uint64_t triangleFingerprint = 0;
      triangleFingerprint = mixFingerprint(triangleFingerprint, ids[0]);
      triangleFingerprint = mixFingerprint(triangleFingerprint, ids[1]);
      triangleFingerprint = mixFingerprint(triangleFingerprint, ids[2]);
      addUnorderedFingerprint(replacementFingerprint, triangleFingerprint);
    }

    componentFingerprint =
      mixFingerprint(componentFingerprint, replacementFingerprint);
    addUnorderedFingerprint(fingerprint, componentFingerprint);
  }


  Vec3 getPoint(const vector<float> &vertices, uint64_t offset) {
    return Vec3{
      vertices[offset + 0],
      vertices[offset + 1],
      vertices[offset + 2],
    };
  }


  bool isRealConfig(const PlanarReductionConfig &config) {
    return isfinite(config.coordTolerance) &&
      isfinite(config.planeDistanceTolerance) &&
      isfinite(config.pairwiseNormalAngleDegrees) &&
      0 < config.coordTolerance && 0 < config.planeDistanceTolerance &&
      0 < config.pairwiseNormalAngleDegrees &&
      config.pairwiseNormalAngleDegrees <= 180;
  }


  Vec3 getNormal(const vector<float> &normals, uint64_t offset) {
    if (offset + 2 >= normals.size()) return Vec3();
    return normalize(Vec3{
      normals[offset + 0],
      normals[offset + 1],
      normals[offset + 2],
    });
  }


  bool isDegenerateTriangle(const array<uint32_t, 3> &ids) {
    return ids[0] == ids[1] || ids[1] == ids[2] || ids[2] == ids[0];
  }


  double stockSideTolerance(const MeshData &mesh) {
    if (!mesh.boundsValid) return max(1e-9, mesh.coordTolerance * 2);

    double dx = mesh.boundsMax.x - mesh.boundsMin.x;
    double dy = mesh.boundsMax.y - mesh.boundsMin.y;
    double dz = mesh.boundsMax.z - mesh.boundsMin.z;
    double span = max(dx, max(dy, dz));
    return max(max(1e-9, mesh.coordTolerance * 2), span * 1e-8);
  }


  double sideCoordinate(const Vec3 &point, unsigned side) {
    switch (side) {
    case PLANAR_REDUCTION_SIDE_X_MIN:
    case PLANAR_REDUCTION_SIDE_X_MAX: return point.x;
    case PLANAR_REDUCTION_SIDE_Y_MIN:
    case PLANAR_REDUCTION_SIDE_Y_MAX: return point.y;
    case PLANAR_REDUCTION_SIDE_Z_MIN:
    case PLANAR_REDUCTION_SIDE_Z_MAX: return point.z;
    default: return 0;
    }
  }


  double sideBoundaryCoordinate(const MeshData &mesh, unsigned side) {
    switch (side) {
    case PLANAR_REDUCTION_SIDE_X_MIN: return mesh.boundsMin.x;
    case PLANAR_REDUCTION_SIDE_X_MAX: return mesh.boundsMax.x;
    case PLANAR_REDUCTION_SIDE_Y_MIN: return mesh.boundsMin.y;
    case PLANAR_REDUCTION_SIDE_Y_MAX: return mesh.boundsMax.y;
    case PLANAR_REDUCTION_SIDE_Z_MIN: return mesh.boundsMin.z;
    case PLANAR_REDUCTION_SIDE_Z_MAX: return mesh.boundsMax.z;
    default: return 0;
    }
  }


  bool triangleOnStockSide(const MeshData &mesh, uint64_t tri,
                           unsigned side, double tolerance) {
    const array<uint32_t, 3> &ids = mesh.triangles[(size_t)tri];
    double boundary = sideBoundaryCoordinate(mesh, side);

    for (uint32_t id: ids)
      if (tolerance < fabs(sideCoordinate(mesh.points[id], side) - boundary))
        return false;

    return true;
  }


  bool componentOnStockSide(const MeshData &mesh,
                            const vector<uint64_t> &component,
                            unsigned side, double tolerance) {
    if (component.empty()) return false;

    for (uint64_t tri: component)
      if (!triangleOnStockSide(mesh, tri, side, tolerance)) return false;

    return true;
  }


  unsigned classifyTriangleSide(const MeshData &mesh, uint64_t tri) {
    if (!mesh.boundsValid) return PLANAR_REDUCTION_SIDE_CUT;

    double tolerance = stockSideTolerance(mesh);
    for (unsigned side = 0; side < PLANAR_REDUCTION_SIDE_CUT; side++)
      if (triangleOnStockSide(mesh, tri, side, tolerance)) return side;

    return PLANAR_REDUCTION_SIDE_CUT;
  }


  unsigned classifyComponentSide(const MeshData &mesh,
                                 const vector<uint64_t> &component) {
    if (!mesh.boundsValid) return PLANAR_REDUCTION_SIDE_CUT;

    double tolerance = stockSideTolerance(mesh);
    for (unsigned side = 0; side < PLANAR_REDUCTION_SIDE_CUT; side++)
      if (componentOnStockSide(mesh, component, side, tolerance)) return side;

    return PLANAR_REDUCTION_SIDE_CUT;
  }


  uint64_t countDegenerateTriangles(const MeshData &mesh,
                                    const vector<uint64_t> &component) {
    uint64_t degenerate = 0;
    for (uint64_t tri: component)
      if (isDegenerateTriangle(mesh.triangles[(size_t)tri])) degenerate++;

    return degenerate;
  }


  bool fitsSeedPlane(uint64_t tri, const Vec3 &seedNormal,
                     double seedDistance,
                     const vector<array<uint32_t, 3> > &triangles,
                     const vector<Vec3> &normals,
                     const vector<Vec3> &points,
                     const PlanarReductionConfig &config,
                     double normalTolerance) {
    if (dot(seedNormal, normals[tri]) < normalTolerance) return false;

    for (unsigned i = 0; i < 3; i++) {
      const Vec3 &point = points[triangles[tri][i]];
      if (config.planeDistanceTolerance <
          fabs(dot(seedNormal, point) - seedDistance))
        return false;
    }

    return true;
  }


  bool fitsSeedPlaneAcrossEdge
    (uint64_t tri, const pair<uint32_t, uint32_t> &sharedEdge,
     const Vec3 &seedNormal, double seedDistance,
     const vector<array<uint32_t, 3> > &triangles,
     const vector<Vec3> &normals, const vector<Vec3> &points,
     const PlanarReductionConfig &config, double normalTolerance,
     uint64_t &vertexChecks) {
    if (dot(seedNormal, normals[tri]) < normalTolerance) return false;

    // The shared edge is already in the component, so its vertices were
    // previously proven against this seed plane.
    unsigned checked = 0;
    for (unsigned i = 0; i < 3; i++) {
      uint32_t vertex = triangles[tri][i];
      if (vertex == sharedEdge.first || vertex == sharedEdge.second)
        continue;

      checked++;
      vertexChecks++;
      const Vec3 &point = points[vertex];
      if (config.planeDistanceTolerance <
          fabs(dot(seedNormal, point) - seedDistance))
        return false;
    }

    if (checked == 1) return true;

    vertexChecks += 3;
    return fitsSeedPlane(tri, seedNormal, seedDistance, triangles, normals,
                         points, config, normalTolerance);
  }


  BoundaryInfo getBoundaryInfo
    (const vector<pair<uint32_t, uint32_t> > &edges) {
    unordered_map<uint32_t, vector<uint32_t> > graph;
    unordered_set<EdgeKey, EdgeKeyHash> seenEdges;
    BoundaryInfo info;
    info.edges = edges.size();

    for (const auto &edge: edges) {
      EdgeKey key(edge.first, edge.second);
      if (!seenEdges.insert(key).second) info.duplicateEdges++;

      graph[edge.first].push_back(edge.second);
      graph[edge.second].push_back(edge.first);
    }

    info.vertices = graph.size();

    for (const auto &entry: graph)
      if (entry.second.size() != 2) info.badVertices++;

    unordered_set<uint32_t> visited;
    queue<uint32_t> q;
    for (const auto &entry: graph) {
      if (!visited.insert(entry.first).second) continue;

      info.loops++;
      q.push(entry.first);
      while (!q.empty()) {
        uint32_t value = q.front();
        q.pop();

        const vector<uint32_t> &neighbors = graph[value];
        for (uint32_t neighbor: neighbors)
          if (visited.insert(neighbor).second) q.push(neighbor);
      }
    }

    return info;
  }


  uint64_t estimateTrianglesAfter(const BoundaryInfo &boundary) {
    if (!boundary.loops || boundary.badVertices || boundary.duplicateEdges)
      return 0;

    int64_t count =
      (int64_t)boundary.vertices + (int64_t)boundary.loops * 2 - 4;
    return 0 < count ? (uint64_t)count : 1;
  }


  uint64_t estimateTrianglesAfter(uint64_t vertices, uint64_t loops) {
    if (!vertices || !loops) return 0;

    int64_t count = (int64_t)vertices + (int64_t)loops * 2 - 4;
    return 0 < count ? (uint64_t)count : 1;
  }


  double pointLineDistance(const Vec3 &point, const Vec3 &a,
                           const Vec3 &b) {
    Vec3 ab = b - a;
    double abLength = length(ab);
    if (!abLength) return length(point - a);

    return length(cross(ab, point - a)) / abLength;
  }


  bool isCollinearLoopVertex(const vector<Vec3> &points,
                             const vector<uint32_t> &loop, size_t i,
                             double tolerance) {
    size_t count = loop.size();
    if (count < 4) return false;

    const Vec3 &prev = points[loop[(i + count - 1) % count]];
    const Vec3 &cur = points[loop[i]];
    const Vec3 &next = points[loop[(i + 1) % count]];
    Vec3 a = cur - prev;
    Vec3 b = next - cur;
    double aLength = length(a);
    double bLength = length(b);
    if (!aLength || !bLength) return true;
    if (dot(a, b) < 0) return false;

    double scale = max(aLength, bLength);
    double distanceTolerance = max(tolerance, scale * 1e-8);
    return pointLineDistance(cur, prev, next) <= distanceTolerance;
  }


  bool isCollinearChainVertex(const vector<Vec3> &points, uint32_t previous,
                              uint32_t current, uint32_t next,
                              double tolerance) {
    const Vec3 &prev = points[previous];
    const Vec3 &cur = points[current];
    const Vec3 &nextPoint = points[next];
    Vec3 a = cur - prev;
    Vec3 b = nextPoint - cur;
    double aLength = length(a);
    double bLength = length(b);
    if (!aLength || !bLength) return true;
    if (dot(a, b) < 0) return false;

    double scale = max(aLength, bLength);
    double distanceTolerance = max(tolerance, scale * 1e-8);
    return pointLineDistance(cur, prev, nextPoint) <= distanceTolerance;
  }


  uint64_t countSimplifiedLoopVertices(const vector<Vec3> &points,
                                       const vector<uint32_t> &loop,
                                       double tolerance) {
    if (loop.size() <= 3) return loop.size();

    uint64_t kept = 0;
    for (size_t i = 0; i < loop.size(); i++)
      if (!isCollinearLoopVertex(points, loop, i, tolerance)) kept++;

    return max<uint64_t>(3, kept);
  }


  uint64_t boundaryDirectedKey(uint32_t a, uint32_t b) {
    return ((uint64_t)a << 32) | b;
  }


  void orientLoopToBoundaryDirection
    (vector<uint32_t> &loop,
     const unordered_set<uint64_t> &directedBoundaryEdges) {
    if (loop.size() < 2) return;

    uint64_t forward = 0;
    uint64_t backward = 0;
    for (size_t i = 0; i < loop.size(); i++) {
      uint32_t a = loop[i];
      uint32_t b = loop[(i + 1) % loop.size()];
      if (directedBoundaryEdges.count(boundaryDirectedKey(a, b))) forward++;
      if (directedBoundaryEdges.count(boundaryDirectedKey(b, a))) backward++;
    }

    if (forward < backward) reverse(loop.begin(), loop.end());
  }


  vector<BoundaryEdge> loopsToBoundaryEdges
    (const vector<vector<uint32_t> > &loops) {
    vector<BoundaryEdge> edges;
    for (const auto &loop: loops) {
      if (loop.size() < 3) continue;
      edges.reserve(edges.size() + loop.size());
      for (size_t i = 0; i < loop.size(); i++)
        edges.push_back(BoundaryEdge(loop[i], loop[(i + 1) % loop.size()]));
    }

    return edges;
  }


  BoundaryCoSimplifyEstimate estimateBoundaryCoSimplify
    (const vector<pair<uint32_t, uint32_t> > &boundaryEdges,
     const vector<Vec3> &points, double tolerance) {
    BoundaryCoSimplifyEstimate result;
    vector<vector<uint32_t> > loops;
    if (!orderBoundaryLoops(boundaryEdges, loops)) return result;

    result.available = true;
    result.loops = loops.size();

    for (const auto &loop: loops) {
      result.boundaryVertices += loop.size();
      result.simplifiedBoundaryVertices +=
        countSimplifiedLoopVertices(points, loop, tolerance);
    }

    result.trianglesAfter =
      estimateTrianglesAfter(result.boundaryVertices, result.loops);
    result.trianglesAfterSimplified =
      estimateTrianglesAfter(result.simplifiedBoundaryVertices, result.loops);
    if (result.trianglesAfterSimplified < result.trianglesAfter)
      result.extraReduction =
        result.trianglesAfter - result.trianglesAfterSimplified;

    return result;
  }


  vector<uint32_t> simplifyBoundaryLoopByContract
    (const vector<uint32_t> &loop,
     const unordered_set<uint32_t> &acceptedVertices) {
    vector<uint32_t> simplified;
    if (loop.size() <= 3) return loop;

    simplified.reserve(loop.size());
    for (uint32_t vertex: loop)
      if (!acceptedVertices.count(vertex)) simplified.push_back(vertex);

    if (simplified.size() < 3) return loop;
    return simplified;
  }


  ReplacementCheck checkContractBoundaryCoSimplifyReplacement
    (const ComponentRecord &record, const vector<Vec3> &points,
     const unordered_set<uint32_t> &acceptedVertices) {
    ReplacementCheck result;
    result.checked = true;

    if (record.loops.empty()) return result;

    unordered_set<uint64_t> directedBoundaryEdges;
    directedBoundaryEdges.reserve(record.boundaryEdges.size());
    for (const auto &edge: record.boundaryEdges)
      directedBoundaryEdges.insert
        (boundaryDirectedKey(edge.first, edge.second));

    vector<vector<uint32_t> > simplifiedLoops;
    simplifiedLoops.reserve(record.loops.size());
    uint64_t vertices = 0;
    uint64_t simplifiedVertices = 0;
    for (vector<uint32_t> loop: record.loops) {
      orientLoopToBoundaryDirection(loop, directedBoundaryEdges);
      vertices += loop.size();
      vector<uint32_t> simplified =
        simplifyBoundaryLoopByContract(loop, acceptedVertices);
      simplifiedVertices += simplified.size();
      simplifiedLoops.push_back(std::move(simplified));
    }

    result.estimateAvailable = true;
    result.trianglesAfter =
      estimateTrianglesAfter(simplifiedVertices, simplifiedLoops.size());
    if (vertices <= simplifiedVertices) return result;

    vector<BoundaryEdge> simplifiedBoundaryEdges =
      loopsToBoundaryEdges(simplifiedLoops);
    if (simplifiedBoundaryEdges.empty()) return result;

    vector<array<uint32_t, 3> > triangles;
    if (simplifiedLoops.size() == 1) {
      if (!triangulateLoop(simplifiedLoops[0], points, record.normal,
                           triangles))
        return result;

    } else if (!triangulateHoleAwareLoops(simplifiedLoops, points,
                                          record.normal, triangles))
      return result;

    if (!replacementBoundaryMatches(simplifiedBoundaryEdges, triangles))
      return result;
    if (!orientReplacementToBoundary(simplifiedBoundaryEdges, triangles))
      return result;

    result.edgeIncidenceChecked = true;
    result.edgeIncidenceOk =
      replacementEdgeIncidenceOk(simplifiedBoundaryEdges, triangles);
    if (!result.edgeIncidenceOk) return result;

    result.feasible = true;
    result.trianglesAfter = triangles.size();
    result.triangles.swap(triangles);
    return result;
  }


  bool componentHasContractVertex
    (const ComponentRecord &record,
     const unordered_set<uint32_t> &acceptedVertices) {
    if (acceptedVertices.empty()) return false;

    for (const auto &loop: record.loops)
      for (uint32_t vertex: loop)
        if (acceptedVertices.count(vertex)) return true;

    return false;
  }


  void recordContractEdgeVertexUse
    (const vector<ComponentRecord> &records, const vector<Vec3> &points,
     double tolerance,
     unordered_map<uint32_t, VertexContractStats> &vertexStats) {
    unordered_map<EdgeKey, vector<size_t>, EdgeKeyHash> edgeComponents;
    edgeComponents.reserve(records.size() * 4);

    for (size_t i = 0; i < records.size(); i++)
      for (const BoundaryEdge &edge: records[i].boundaryEdges)
        edgeComponents[EdgeKey(edge.first, edge.second)].push_back(i);

    auto edgeContractEligible = [&](uint32_t a, uint32_t b) -> bool {
      auto it = edgeComponents.find(EdgeKey(a, b));
      if (it == edgeComponents.end() || it->second.size() != 2) return false;

      for (size_t component: it->second)
        if (!records[component].contractEligible) return false;

      return true;
    };

    for (const ComponentRecord &record: records)
      for (const auto &loop: record.loops) {
        if (loop.size() < 3) continue;

        for (size_t i = 0; i < loop.size(); i++) {
          uint32_t previous = loop[(i + loop.size() - 1) % loop.size()];
          uint32_t current = loop[i];
          uint32_t next = loop[(i + 1) % loop.size()];

          VertexContractStats &stats = vertexStats[current];
          stats.useCount++;

          bool eligible =
            record.contractEligible &&
            edgeContractEligible(previous, current) &&
            edgeContractEligible(current, next);
          if (!eligible) continue;

          stats.eligibleUseCount++;
          if (isCollinearLoopVertex(points, loop, i, tolerance))
            stats.removableUseCount++;
        }
      }
  }


  unordered_set<uint32_t> buildAcceptedContractChainVertices
    (const vector<ComponentRecord> &records, const vector<Vec3> &points,
     double tolerance, PlanarReductionReport &report) {
    unordered_set<uint32_t> accepted;
    unordered_map<EdgeKey, vector<size_t>, EdgeKeyHash> edgeComponents;
    edgeComponents.reserve(records.size() * 4);

    for (size_t i = 0; i < records.size(); i++)
      for (const BoundaryEdge &edge: records[i].boundaryEdges)
        edgeComponents[EdgeKey(edge.first, edge.second)].push_back(i);

    unordered_map<ComponentPairKey, vector<EdgeKey>, ComponentPairKeyHash>
      interfaceEdges;

    for (const auto &entry: edgeComponents) {
      const vector<size_t> &components = entry.second;
      if (components.size() == 1) {
        if (records[components[0]].contractEligible)
          report.boundaryCoSimplifyContractRejectedMissingOwner++;
        continue;
      }

      if (components.size() != 2) {
        report.boundaryCoSimplifyContractRejectedAmbiguousOwner++;
        continue;
      }

      size_t a = components[0];
      size_t b = components[1];
      bool eligible =
        records[a].contractEligible && records[b].contractEligible;
      if (!eligible) {
        if (records[a].contractEligible || records[b].contractEligible)
          report.boundaryCoSimplifyContractRejectedChainIneligible++;
        continue;
      }

      report.boundaryCoSimplifyContractInterfaceEdges++;
      interfaceEdges[ComponentPairKey(a, b)].push_back(entry.first);
    }

    for (const auto &entry: interfaceEdges) {
      report.boundaryCoSimplifyContractChainInterfaces++;

      unordered_map<uint32_t, vector<uint32_t> > graph;
      graph.reserve(entry.second.size() * 2);
      for (const EdgeKey &edge: entry.second) {
        graph[edge.a].push_back(edge.b);
        graph[edge.b].push_back(edge.a);
      }

      unordered_set<uint32_t> visited;
      visited.reserve(graph.size());

      for (const auto &seedEntry: graph) {
        uint32_t seed = seedEntry.first;
        if (visited.count(seed)) continue;

        vector<uint32_t> stack;
        vector<uint32_t> component;
        stack.push_back(seed);
        visited.insert(seed);
        while (!stack.empty()) {
          uint32_t vertex = stack.back();
          stack.pop_back();
          component.push_back(vertex);
          for (uint32_t neighbor: graph[vertex])
            if (visited.insert(neighbor).second) stack.push_back(neighbor);
        }

        report.boundaryCoSimplifyContractChains++;
        report.boundaryCoSimplifyContractChainVertices += component.size();
        if (component.size() < 3) continue;

        vector<uint32_t> endpoints;
        bool unsafe = false;
        for (uint32_t vertex: component) {
          size_t degree = graph[vertex].size();
          if (degree == 1) endpoints.push_back(vertex);
          else if (degree != 2) unsafe = true;
        }

        if (unsafe || endpoints.size() != 2) {
          report.boundaryCoSimplifyContractRejectedUnsafeEndpoint++;
          continue;
        }

        vector<uint32_t> ordered;
        ordered.reserve(component.size());
        uint32_t previous = numeric_limits<uint32_t>::max();
        uint32_t current = endpoints[0];
        while (true) {
          ordered.push_back(current);
          if (current == endpoints[1]) break;

          const vector<uint32_t> &neighbors = graph[current];
          uint32_t next = numeric_limits<uint32_t>::max();
          for (uint32_t neighbor: neighbors)
            if (neighbor != previous) {
              next = neighbor;
              break;
            }

          if (next == numeric_limits<uint32_t>::max()) break;
          previous = current;
          current = next;
          if (ordered.size() > component.size()) break;
        }

        if (ordered.size() != component.size() ||
            ordered.back() != endpoints[1]) {
          report.boundaryCoSimplifyContractRejectedUnsafeEndpoint++;
          continue;
        }

        for (size_t i = 1; i + 1 < ordered.size(); i++) {
          report.boundaryCoSimplifyContractChainInteriorVertices++;
          if (!isCollinearChainVertex(points, ordered[i - 1], ordered[i],
                                      ordered[i + 1], tolerance)) {
            report.boundaryCoSimplifyContractRejectedChainNonCollinear++;
            continue;
          }

          if (accepted.insert(ordered[i]).second)
            report.boundaryCoSimplifyContractChainVerticesAccepted++;
        }
      }
    }

    return accepted;
  }


  unordered_set<uint32_t> buildAcceptedContractVertices
    (const unordered_map<uint32_t, VertexContractStats> &vertexStats,
     PlanarReductionReport &report) {
    unordered_set<uint32_t> accepted;
    accepted.reserve(vertexStats.size());

    for (const auto &entry: vertexStats) {
      const VertexContractStats &stats = entry.second;
      report.boundaryCoSimplifyContractVerticesConsidered++;

      if (stats.useCount < 2) {
        report.boundaryCoSimplifyContractRejectedSingleSided++;
        continue;
      }

      if (4 < stats.useCount)
        report.boundaryCoSimplifyContractRejectedAmbiguous++;

      if (!stats.eligibleUseCount) {
        report.boundaryCoSimplifyContractRejectedIneligible++;
        continue;
      }

      if (stats.eligibleUseCount < 2) {
        report.boundaryCoSimplifyContractRejectedSingleSided++;
        continue;
      }

      if (stats.eligibleUseCount != stats.removableUseCount) {
        report.boundaryCoSimplifyContractRejectedNonCollinear++;
        continue;
      }

      if (stats.useCount != stats.eligibleUseCount)
        report.boundaryCoSimplifyContractRejectedOwnership++;

      accepted.insert(entry.first);
      report.boundaryCoSimplifyContractVerticesAccepted++;
    }

    return accepted;
  }


  uint64_t addClassifiedComponent(PlanarReductionReport &report,
                                  uint64_t componentTriangles,
                                  uint64_t estimatedAfter,
                                  const BoundaryInfo &boundary,
                                  const ReplacementCheck &replacement) {
    bool triangulationFailed =
      replacement.checked && !replacement.feasible &&
      !replacement.estimateAvailable;
    if (triangulationFailed)
      estimatedAfter = componentTriangles;

    uint64_t reduction =
      estimatedAfter && estimatedAfter < componentTriangles ?
      componentTriangles - estimatedAfter : 0;

    report.estimatedTrianglesAfter +=
      reduction ? estimatedAfter : componentTriangles;
    report.estimatedTriangleReduction += reduction;

    if (boundary.badVertices || boundary.duplicateEdges || !boundary.loops) {
      report.rejectedBoundaryComponents++;
      report.rejectedBoundaryTriangles += componentTriangles;

    } else if (triangulationFailed) {
      report.rejectedTriangulationComponents++;
      report.rejectedTriangulationTriangles += componentTriangles;

    } else if (!reduction) {
      report.rejectedNoSavingsComponents++;
      report.rejectedNoSavingsTriangles += componentTriangles;

    } else if (1 < boundary.loops) {
      report.holeAwareComponents++;
      report.holeAwareSourceTriangles += componentTriangles;
      report.holeAwareEstimatedReduction += reduction;

    } else {
      report.phase1Components++;
      report.phase1SourceTriangles += componentTriangles;
      report.phase1EstimatedReduction += reduction;
    }

    return reduction;
  }


  uint64_t addClassifiedSideComponent(PlanarReductionSideReport &side,
                                      uint64_t componentTriangles,
                                      uint64_t estimatedAfter,
                                      const BoundaryInfo &boundary,
                                      const ReplacementCheck &replacement) {
    bool triangulationFailed =
      replacement.checked && !replacement.feasible &&
      !replacement.estimateAvailable;
    if (triangulationFailed)
      estimatedAfter = componentTriangles;

    uint64_t reduction =
      estimatedAfter && estimatedAfter < componentTriangles ?
      componentTriangles - estimatedAfter : 0;

    side.estimatedTrianglesAfter +=
      reduction ? estimatedAfter : componentTriangles;
    side.estimatedTriangleReduction += reduction;

    if (componentTriangles == 1) {
      side.singleTriangleComponents++;
      side.singleTriangleTriangles += componentTriangles;

    } else if (boundary.badVertices || boundary.duplicateEdges ||
               !boundary.loops) {
      side.rejectedBoundaryComponents++;
      side.rejectedBoundaryTriangles += componentTriangles;

    } else if (triangulationFailed) {
      side.rejectedTriangulationComponents++;
      side.rejectedTriangulationTriangles += componentTriangles;

    } else if (!reduction) {
      side.rejectedNoSavingsComponents++;
      side.rejectedNoSavingsTriangles += componentTriangles;

    } else if (1 < boundary.loops) {
      side.holeAwareComponents++;
      side.holeAwareSourceTriangles += componentTriangles;
      side.holeAwareEstimatedOutputTriangles += estimatedAfter;
      side.holeAwareEstimatedReduction += reduction;

    } else {
      side.phase1Components++;
      side.phase1SourceTriangles += componentTriangles;
      side.phase1EstimatedOutputTriangles += estimatedAfter;
      side.phase1EstimatedReduction += reduction;
    }

    return reduction;
  }


  void recordZeroNormalSideTriangle(PlanarReductionReport &report,
                                    const MeshData &mesh, uint64_t tri) {
    PlanarReductionSideReport &side =
      report.sides[classifyTriangleSide(mesh, tri)];
    side.inputTriangles++;
    side.zeroNormalTriangles++;
    side.estimatedTrianglesAfter++;
    if (isDegenerateTriangle(mesh.triangles[(size_t)tri]))
      side.degenerateTriangles++;
  }


  PlanarReductionSideReport &recordSideComponent
    (PlanarReductionReport &report, const MeshData &mesh,
     const vector<uint64_t> &component) {
    PlanarReductionSideReport &side =
      report.sides[classifyComponentSide(mesh, component)];
    side.inputTriangles += component.size();
    side.componentTriangles += component.size();
    side.degenerateTriangles += countDegenerateTriangles(mesh, component);
    side.components++;
    return side;
  }


  void recordBoundaryCoSimplifyEstimate
    (PlanarReductionReport &report, PlanarReductionSideReport &side,
     uint64_t componentTriangles,
     const BoundaryCoSimplifyEstimate &estimate) {
    if (!estimate.available || !estimate.extraReduction) return;

    report.boundaryCoSimplifyCandidateComponents++;
    report.boundaryCoSimplifySourceTriangles += componentTriangles;
    report.boundaryCoSimplifyBoundaryVertices += estimate.boundaryVertices;
    report.boundaryCoSimplifySimplifiedBoundaryVertices +=
      estimate.simplifiedBoundaryVertices;
    report.boundaryCoSimplifyEstimatedTrianglesAfter +=
      estimate.trianglesAfter;
    report.boundaryCoSimplifyEstimatedTrianglesAfterSimplified +=
      estimate.trianglesAfterSimplified;
    report.boundaryCoSimplifyEstimatedExtraReduction +=
      estimate.extraReduction;
    report.boundaryCoSimplifyMaxComponentExtraReduction =
      max(report.boundaryCoSimplifyMaxComponentExtraReduction,
          estimate.extraReduction);

    side.boundaryCoSimplifyCandidateComponents++;
    side.boundaryCoSimplifySourceTriangles += componentTriangles;
    side.boundaryCoSimplifyBoundaryVertices += estimate.boundaryVertices;
    side.boundaryCoSimplifySimplifiedBoundaryVertices +=
      estimate.simplifiedBoundaryVertices;
    side.boundaryCoSimplifyEstimatedTrianglesAfter += estimate.trianglesAfter;
    side.boundaryCoSimplifyEstimatedTrianglesAfterSimplified +=
      estimate.trianglesAfterSimplified;
    side.boundaryCoSimplifyEstimatedExtraReduction += estimate.extraReduction;
  }


  void recordSideValidationRollback(PlanarReductionReport &report) {
    for (PlanarReductionSideReport &side: report.sides) {
      side.validationRollbackComponents += side.appliedComponents;
      side.validationRollbackSourceTriangles += side.appliedSourceTriangles;
      side.validationRollbackCandidateOutputTriangles +=
        side.appliedOutputTriangles;
    }
  }


  void finalizeSideReports(PlanarReductionReport &report) {
    for (PlanarReductionSideReport &side: report.sides) {
      uint64_t accounted =
        side.componentTriangles + side.zeroNormalTriangles;
      side.unaccountedTriangles =
        side.inputTriangles < accounted ? 0 : side.inputTriangles - accounted;

      if (report.validationRolledBack) {
        side.outputTriangles = side.inputTriangles;
        continue;
      }

      uint64_t consumed =
        min(side.inputTriangles, side.appliedSourceTriangles);
      side.outputTriangles =
        side.inputTriangles - consumed + side.appliedOutputTriangles;
    }
  }


  void resetAppliedReduction(PlanarReductionReport &report) {
    report.appliedComponents = 0;
    report.appliedSourceTriangles = 0;
    report.appliedOutputTriangles = 0;
    report.holeAwareAppliedComponents = 0;
    report.holeAwareAppliedSourceTriangles = 0;
    report.holeAwareAppliedOutputTriangles = 0;

    for (PlanarReductionSideReport &side: report.sides) {
      side.appliedComponents = 0;
      side.appliedSourceTriangles = 0;
      side.appliedOutputTriangles = 0;
    }
  }


  uint64_t addClassifiedComponent(ComponentSummary &summary,
                                  uint64_t componentTriangles,
                                  uint64_t estimatedAfter,
                                  const BoundaryInfo &boundary,
                                  const ReplacementCheck &replacement) {
    bool triangulationFailed =
      replacement.checked && !replacement.feasible &&
      !replacement.estimateAvailable;
    if (triangulationFailed)
      estimatedAfter = componentTriangles;

    uint64_t reduction =
      estimatedAfter && estimatedAfter < componentTriangles ?
      componentTriangles - estimatedAfter : 0;

    summary.estimatedTrianglesAfter +=
      reduction ? estimatedAfter : componentTriangles;
    summary.estimatedTriangleReduction += reduction;

    if (boundary.badVertices || boundary.duplicateEdges || !boundary.loops) {
      summary.rejectedBoundaryComponents++;
      summary.rejectedBoundaryTriangles += componentTriangles;

    } else if (triangulationFailed) {
      summary.rejectedTriangulationComponents++;
      summary.rejectedTriangulationTriangles += componentTriangles;

    } else if (!reduction) {
      summary.rejectedNoSavingsComponents++;
      summary.rejectedNoSavingsTriangles += componentTriangles;

    } else if (1 < boundary.loops) {
      summary.holeAwareComponents++;
      summary.holeAwareSourceTriangles += componentTriangles;
      summary.holeAwareEstimatedReduction += reduction;

    } else {
      summary.phase1Components++;
      summary.phase1SourceTriangles += componentTriangles;
      summary.phase1EstimatedReduction += reduction;
    }

    return reduction;
  }


  MeshData buildMeshData(const Surface &surface,
                         const PlanarReductionConfig &config,
                         bool buildAdjacency) {
    MeshData mesh;
    unordered_map<VertexKey, uint32_t, VertexKeyHash> vertexIds;

    mesh.coordTolerance = config.coordTolerance;
    mesh.triangleCount = surface.getTriangleCount();
    uint64_t maxTriangles = buildAdjacency ?
      (uint64_t)numeric_limits<int32_t>::max() / 3 :
      (uint64_t)numeric_limits<uint32_t>::max() / 3;
    if (maxTriangles < mesh.triangleCount ||
        (uint64_t)numeric_limits<size_t>::max() / 9 < mesh.triangleCount ||
        (buildAdjacency &&
         (uint64_t)numeric_limits<size_t>::max() / 3 < mesh.triangleCount)) {
      mesh.sourceRangeMismatch = true;
      return mesh;
    }
    mesh.sourceExpectedFloats = mesh.triangleCount * 9;
    mesh.triangles.reserve((size_t)mesh.triangleCount);
    mesh.normals.reserve((size_t)mesh.triangleCount);
    vertexIds.reserve((size_t)mesh.triangleCount);

    if (buildAdjacency) {
      mesh.neighbors.assign((size_t)mesh.triangleCount * 3, -1);
      mesh.edgeInfo.reserve((size_t)mesh.triangleCount * 3 / 2);
    }

    surface.getVertices([&](const vector<float> &vertices,
                            const vector<float> &surfaceNormals) {
        mesh.sourceVertexFloats = vertices.size();
        mesh.sourceNormalFloats = surfaceNormals.size();
        mesh.sourceVertexCountMismatch =
          vertices.size() != mesh.sourceExpectedFloats;
        mesh.sourceNormalCountMismatch =
          surfaceNormals.size() != mesh.sourceExpectedFloats;
        if (mesh.sourceBufferMismatch()) return;

        for (float value: vertices) {
          if (!isfinite(value)) {
            mesh.sourceInvalidCoordinates++;
            continue;
          }
          int64_t quantized = 0;
          if (!Internal::quantizeCoordinate
              (value, config.coordTolerance, quantized))
            mesh.sourceRangeMismatch = true;
        }
        for (float value: surfaceNormals)
          if (!isfinite(value)) mesh.sourceInvalidCoordinates++;
        if (mesh.sourceInvalidCoordinates || mesh.sourceRangeMismatch) return;

        for (uint64_t tri = 0; tri < mesh.triangleCount; tri++) {
          uint64_t offset = tri * 9;
          array<uint32_t, 3> ids;
          for (unsigned i = 0; i < 3; i++) {
            uint64_t vertexOffset = offset + (uint64_t)i * 3;
            ids[i] = getVertexId
              (packVertex(vertices, vertexOffset, config.coordTolerance),
               getPoint(vertices, vertexOffset), vertexIds, mesh.points);
          }

          mesh.triangles.push_back(ids);
          mesh.normals.push_back(getNormal(surfaceNormals, offset));
          if (isDegenerateTriangle(ids)) mesh.degenerateTriangles++;

          if (!buildAdjacency) continue;

          for (unsigned slot = 0; slot < 3; slot++) {
            auto edge = getTriangleEdge(ids, slot);
            EdgeKey key(edge.first, edge.second);
            EdgeInfo &info = mesh.edgeInfo[key];
            int32_t owner = info.owner;
            info.count++;
            if (isForwardEdge(edge, key)) info.forwardCount++;

            if (info.count == 1) info.owner = (int32_t)(tri * 3 + slot);
            else if (info.count == 2 && 0 <= owner) {
              uint64_t ownerTri = owner / 3;
              unsigned ownerSlot = owner % 3;
              mesh.neighbors[tri * 3 + slot] = (int32_t)ownerTri;
              mesh.neighbors[ownerTri * 3 + ownerSlot] = (int32_t)tri;
              info.owner = -2;

            } else if (info.count == 3) mesh.nonManifoldEdges++;
          }
        }
      });

    if (!mesh.points.empty()) {
      mesh.boundsMin = mesh.points[0];
      mesh.boundsMax = mesh.points[0];
      mesh.boundsValid = true;

      for (const Vec3 &point: mesh.points) {
        mesh.boundsMin.x = min(mesh.boundsMin.x, point.x);
        mesh.boundsMin.y = min(mesh.boundsMin.y, point.y);
        mesh.boundsMin.z = min(mesh.boundsMin.z, point.z);
        mesh.boundsMax.x = max(mesh.boundsMax.x, point.x);
        mesh.boundsMax.y = max(mesh.boundsMax.y, point.y);
        mesh.boundsMax.z = max(mesh.boundsMax.z, point.z);
      }
    }

    return mesh;
  }


  void copyMeshSourceMetrics(PlanarReductionReport &report,
                             const MeshData &mesh) {
    report.triangles = mesh.triangleCount;
    report.outputTriangles = mesh.triangleCount;
    report.sourceExpectedFloats = mesh.sourceExpectedFloats;
    report.sourceVertexFloats = mesh.sourceVertexFloats;
    report.sourceNormalFloats = mesh.sourceNormalFloats;
    report.sourceVertexCountMismatch = mesh.sourceVertexCountMismatch;
    report.sourceNormalCountMismatch = mesh.sourceNormalCountMismatch;
    report.sourceInvalidCoordinates = mesh.sourceInvalidCoordinates;
    report.sourceRangeMismatch = mesh.sourceRangeMismatch;
  }


  ComponentSummary classifyComponentSummary
    (const MeshData &mesh, const vector<int32_t> &neighbors,
     const PlanarReductionConfig &config) {
    ComponentSummary summary;
    if (neighbors.size() != (size_t)mesh.triangleCount * 3) return summary;

    summary.available = true;
    vector<uint8_t> visited((size_t)mesh.triangleCount);
    vector<int32_t> componentMark((size_t)mesh.triangleCount, -1);
    vector<uint64_t> stack;
    vector<uint64_t> component;
    vector<BoundaryEdge> boundaryEdges;
    const double normalTolerance =
      cos(config.pairwiseNormalAngleDegrees / 2.0 * PI / 180.0);

    for (uint64_t seed = 0; seed < mesh.triangleCount; seed++) {
      if (visited[seed]) continue;

      const Vec3 seedNormal = mesh.normals[seed];
      if (!length(seedNormal)) {
        visited[seed] = 1;
        continue;
      }

      const Vec3 &seedPoint = mesh.points[mesh.triangles[seed][0]];
      double seedDistance = dot(seedNormal, seedPoint);

      component.clear();
      stack.clear();
      stack.push_back(seed);
      visited[seed] = 1;
      componentMark[seed] = (int32_t)summary.components;

      while (!stack.empty()) {
        uint64_t tri = stack.back();
        stack.pop_back();
        component.push_back(tri);

        for (unsigned slot = 0; slot < 3; slot++) {
          int32_t neighbor = neighbors[tri * 3 + slot];
          if (neighbor < 0) continue;
          if ((uint64_t)neighbor >= mesh.triangleCount) {
            summary.available = false;
            return summary;
          }
          if (visited[neighbor]) continue;

          uint64_t vertexChecks = 0;
          if (fitsSeedPlaneAcrossEdge
              (neighbor, getTriangleEdge(mesh.triangles[tri], slot),
               seedNormal, seedDistance, mesh.triangles, mesh.normals,
               mesh.points, config, normalTolerance, vertexChecks)) {
            visited[neighbor] = 1;
            componentMark[neighbor] = (int32_t)summary.components;
            stack.push_back(neighbor);
          }
        }
      }

      prepareBoundaryEdges(boundaryEdges, component.size());
      uint64_t boundaryFingerprint = 0;

      for (uint64_t tri: component)
        for (unsigned slot = 0; slot < 3; slot++) {
          int32_t neighbor = neighbors[tri * 3 + slot];
          if (neighbor >= 0 && (uint64_t)neighbor >= mesh.triangleCount) {
            summary.available = false;
            return summary;
          }
          if (neighbor < 0 ||
              componentMark[neighbor] != (int32_t)summary.components) {
            BoundaryEdge edge = getTriangleEdge(mesh.triangles[tri], slot);
            boundaryEdges.push_back(edge);
            mixBoundaryEdge(boundaryFingerprint, edge);
          }
        }

      BoundaryInfo boundary = getBoundaryInfo(boundaryEdges);
      uint64_t estimatedAfter = estimateTrianglesAfter(boundary);
      ReplacementCheck replacement;
      if (estimatedAfter && estimatedAfter < component.size() &&
          !boundary.badVertices && !boundary.duplicateEdges) {
        if (boundary.loops == 1)
          replacement =
            checkPhase1Replacement(boundaryEdges, mesh.points, seedNormal);
        else if (1 < boundary.loops && config.applyHoleAware)
          replacement =
            checkHoleAwareReplacement(boundaryEdges, mesh.points, seedNormal);

        if (replacement.edgeIncidenceChecked) {
          summary.replacementEdgeIncidenceChecks++;
          if (!replacement.edgeIncidenceOk) {
            summary.replacementEdgeIncidenceRejected++;
            if (boundary.loops == 1)
              summary.phase1ReplacementEdgeIncidenceRejected++;
            else if (1 < boundary.loops)
              summary.holeAwareReplacementEdgeIncidenceRejected++;
          }
        }

        if (replacement.feasible || replacement.estimateAvailable) {
          summary.estimatedReplacementChecks++;
          estimatedAfter = replacement.trianglesAfter;
        }

        if (replacement.feasible && !replacement.triangles.empty()) {
          summary.feasibleReplacementChecks++;
          summary.writableReplacementChecks++;
          if (boundary.loops == 1)
            summary.phase1WritableReplacementChecks++;
          else if (1 < boundary.loops)
            summary.holeAwareWritableReplacementChecks++;

        } else if (replacement.estimateAvailable) {
          summary.unwritableReplacementChecks++;
          if (boundary.loops == 1)
            summary.phase1UnwritableReplacementChecks++;
          else if (1 < boundary.loops)
            summary.holeAwareUnwritableReplacementChecks++;
        }
      }

      uint64_t reduction =
        addClassifiedComponent(summary, component.size(), estimatedAfter,
                               boundary, replacement);
      bool decisionBearing =
        reduction || replacement.checked || replacement.estimateAvailable ||
        replacement.edgeIncidenceChecked;
      if (decisionBearing) {
        summary.decisionBearingComponents++;
        summary.decisionBearingTriangles += component.size();
        recordComponentDecisionFingerprint
          (summary.componentDecisionFingerprint, component.size(),
           boundaryFingerprint, boundary, estimatedAfter, reduction,
           replacement);
      }

      summary.components++;
    }

    return summary;
  }


  uint64_t countComponentSummaryMismatches
    (const PlanarReductionReport &report, const ComponentSummary &summary) {
    uint64_t mismatches = 0;
    if (!summary.available) return 1;

    if (report.components != summary.components) mismatches++;
    if (report.estimatedTrianglesAfter != summary.estimatedTrianglesAfter)
      mismatches++;
    if (report.estimatedTriangleReduction !=
        summary.estimatedTriangleReduction)
      mismatches++;
    if (report.componentDecisionFingerprint !=
        summary.componentDecisionFingerprint)
      mismatches++;
    if (report.decisionBearingComponents !=
        summary.decisionBearingComponents)
      mismatches++;
    if (report.decisionBearingTriangles !=
        summary.decisionBearingTriangles)
      mismatches++;
    if (report.phase1Components != summary.phase1Components) mismatches++;
    if (report.phase1SourceTriangles != summary.phase1SourceTriangles)
      mismatches++;
    if (report.phase1EstimatedReduction !=
        summary.phase1EstimatedReduction)
      mismatches++;
    if (report.holeAwareComponents != summary.holeAwareComponents)
      mismatches++;
    if (report.holeAwareSourceTriangles != summary.holeAwareSourceTriangles)
      mismatches++;
    if (report.holeAwareEstimatedReduction !=
        summary.holeAwareEstimatedReduction)
      mismatches++;
    if (report.estimatedReplacementChecks !=
        summary.estimatedReplacementChecks)
      mismatches++;
    if (report.feasibleReplacementChecks != summary.feasibleReplacementChecks)
      mismatches++;
    if (report.writableReplacementChecks != summary.writableReplacementChecks)
      mismatches++;
    if (report.unwritableReplacementChecks !=
        summary.unwritableReplacementChecks)
      mismatches++;
    if (report.phase1WritableReplacementChecks !=
        summary.phase1WritableReplacementChecks)
      mismatches++;
    if (report.holeAwareWritableReplacementChecks !=
        summary.holeAwareWritableReplacementChecks)
      mismatches++;
    if (report.phase1UnwritableReplacementChecks !=
        summary.phase1UnwritableReplacementChecks)
      mismatches++;
    if (report.holeAwareUnwritableReplacementChecks !=
        summary.holeAwareUnwritableReplacementChecks)
      mismatches++;
    if (report.replacementEdgeIncidenceChecks !=
        summary.replacementEdgeIncidenceChecks)
      mismatches++;
    if (report.replacementEdgeIncidenceRejected !=
        summary.replacementEdgeIncidenceRejected)
      mismatches++;
    if (report.phase1ReplacementEdgeIncidenceRejected !=
        summary.phase1ReplacementEdgeIncidenceRejected)
      mismatches++;
    if (report.holeAwareReplacementEdgeIncidenceRejected !=
        summary.holeAwareReplacementEdgeIncidenceRejected)
      mismatches++;
    if (report.rejectedBoundaryComponents !=
        summary.rejectedBoundaryComponents)
      mismatches++;
    if (report.rejectedBoundaryTriangles !=
        summary.rejectedBoundaryTriangles)
      mismatches++;
    if (report.rejectedNoSavingsComponents !=
        summary.rejectedNoSavingsComponents)
      mismatches++;
    if (report.rejectedNoSavingsTriangles !=
        summary.rejectedNoSavingsTriangles)
      mismatches++;
    if (report.rejectedTriangulationComponents !=
        summary.rejectedTriangulationComponents)
      mismatches++;
    if (report.rejectedTriangulationTriangles !=
        summary.rejectedTriangulationTriangles)
      mismatches++;

    return mismatches;
  }


  PlanarReductionReport analyzeOrReduceImpl(
    const Surface &surface, const PlanarReductionConfig &config,
    TriangleSurface *triangleSurface) {
    PlanarReductionReport report;
    if (!isRealConfig(config)) return report;

    const TriangleSurface *sourceTriangleSurface =
      triangleSurface ? triangleSurface : dynamic_cast<const TriangleSurface *>(&surface);
    bool reduce = triangleSurface != 0;
    bool trustedProvenanceNeighbors = false;
    bool trustRequested =
      config.trustProvenanceNeighbors && config.useProvenanceNeighbors;
    vector<uint8_t> trustedReciprocalSlots;

    report.coordTolerance = config.coordTolerance;
    report.planeDistanceTolerance = config.planeDistanceTolerance;
    report.pairwiseNormalAngleDegrees = config.pairwiseNormalAngleDegrees;
    report.useProvenanceNeighborsRequested = config.useProvenanceNeighbors;
    report.trustProvenanceNeighborsRequested =
      config.trustProvenanceNeighbors;
    report.applyHoleAwareRequested = config.applyHoleAware;
    report.applyBoundaryCoSimplifyRequested =
      config.applyBoundaryCoSimplify;

    if (trustRequested) {
      if (!sourceTriangleSurface)
        report.trustedProvenanceRejectedNoTriangleSurface = true;
      else if (!sourceTriangleSurface->hasContourProvenance())
        report.trustedProvenanceRejectedNoProvenance = true;
      else if (!sourceTriangleSurface->hasContourProvenanceNeighbors())
        report.trustedProvenanceRejectedNoCachedNeighbors = true;
      else {
        uint64_t triangleCount = surface.getTriangleCount();
        const ContourProvenanceReport &provenance =
          sourceTriangleSurface->getContourProvenanceReport();
        const vector<int32_t> &neighbors =
          sourceTriangleSurface->getContourProvenanceNeighbors();
        bool neighborSizeOk = false;
        bool cachedNeighborsRaw =
          sourceTriangleSurface->hasRawContourProvenanceNeighbors();
        report.contourProvenanceNeighborsRaw = cachedNeighborsRaw;

        if (provenance.triangles != triangleCount)
          report.trustedProvenanceRejectedTriangleMismatch = true;
        if (provenance.completeTriangles != triangleCount)
          report.trustedProvenanceRejectedIncomplete = true;
        if (provenance.unknownTriangles)
          report.trustedProvenanceRejectedUnknown = true;
        if (!provenance.watertight)
          report.trustedProvenanceRejectedNonWatertight = true;
        if (provenance.rawMisorientedEdges || provenance.misorientedEdges)
          report.trustedProvenanceRejectedOrientation = true;
        bool rawTopologyMatchesWelded =
          !provenance.rawBoundaryEdges &&
          !provenance.rawNonManifoldEdges &&
          !provenance.rawMisorientedEdges &&
          !provenance.misorientedEdges &&
          provenance.rawUniqueEdges == provenance.weldedUniqueEdges &&
          provenance.rawTwinEdgeSlots == provenance.weldedTwinEdgeSlots &&
          provenance.rawBoundaryEdgeSlots ==
          provenance.weldedBoundaryEdgeSlots &&
          provenance.rawNonManifoldEdgeSlots ==
          provenance.weldedNonManifoldEdgeSlots;
        bool rawKeysMapToSingleWeldedPoint =
          !provenance.rawGridGridWeldedSpreadEdges &&
          !provenance.rawCenterInvolvedWeldedSpreadEdges &&
          !provenance.rawGridVertexWeldedSpreadKeys &&
          !provenance.rawCenterVertexWeldedSpreadKeys;
        if (cachedNeighborsRaw && !rawTopologyMatchesWelded)
          report.trustedProvenanceRejectedRawTopology = true;
        if (cachedNeighborsRaw && !rawKeysMapToSingleWeldedPoint)
          report.trustedProvenanceRejectedRawWeldedSpread = true;
        if (triangleCount <=
            (uint64_t)(numeric_limits<size_t>::max() / 3))
          neighborSizeOk = neighbors.size() == (size_t)triangleCount * 3;
        if (!neighborSizeOk)
          report.trustedProvenanceRejectedNeighborSize = true;

        if (neighborSizeOk) {
          report.trustedProvenanceNeighborSlotsChecked = neighbors.size();
          TrustedNeighborValidation validation;
          {
            Profile::Scope scope("safe_reduce_validate_trusted_neighbors");
            validation = validateTrustedNeighbors
              (neighbors, triangleCount, &trustedReciprocalSlots);
          }
          report.trustedProvenanceRejectedNeighborOpenSlot =
            validation.openSlot;
          report.trustedProvenanceRejectedNeighborRange = validation.range;
          report.trustedProvenanceRejectedNeighborSelf = validation.self;
          report.trustedProvenanceRejectedNeighborDuplicate =
            validation.duplicate;
          report.trustedProvenanceRejectedNeighborAsymmetry =
            validation.asymmetry;
        }

        trustedProvenanceNeighbors =
          !report.trustedProvenanceRejectedTriangleMismatch &&
          !report.trustedProvenanceRejectedIncomplete &&
          !report.trustedProvenanceRejectedUnknown &&
          !report.trustedProvenanceRejectedNonWatertight &&
          !report.trustedProvenanceRejectedOrientation &&
          !report.trustedProvenanceRejectedRawTopology &&
          !report.trustedProvenanceRejectedRawWeldedSpread &&
          !report.trustedProvenanceRejectedNeighborSize &&
          !report.trustedProvenanceRejectedNeighborOpenSlot &&
          !report.trustedProvenanceRejectedNeighborRange &&
          !report.trustedProvenanceRejectedNeighborSelf &&
          !report.trustedProvenanceRejectedNeighborDuplicate &&
          !report.trustedProvenanceRejectedNeighborAsymmetry;
        report.trustedProvenanceNeighborsEligible =
          trustedProvenanceNeighbors;
      }
    }

    MeshData mesh;
    {
      Profile::Scope scope(trustedProvenanceNeighbors ?
                           "safe_reduce_build_mesh_geometry" :
                           "safe_reduce_build_adjacency");
      mesh = buildMeshData(surface, config, !trustedProvenanceNeighbors);
    }

    copyMeshSourceMetrics(report, mesh);
    if (mesh.sourceBufferMismatch()) {
      releaseMeshData(mesh);
      return report;
    }

    if (trustedProvenanceNeighbors) {
      const vector<int32_t> &neighbors =
        sourceTriangleSurface->getContourProvenanceNeighbors();
      TrustedNeighborValidation validation;
      {
        Profile::Scope scope("safe_reduce_validate_trusted_neighbor_edges");
        validation = validateTrustedNeighborEdges
          (mesh, neighbors, &trustedReciprocalSlots);
      }

      report.trustedProvenanceNeighborEdgeSlotsChecked = neighbors.size();
      report.trustedProvenanceRejectedNeighborEdgeMismatch =
        validation.edgeMismatch;
      report.trustedProvenanceNeighborEdgeMismatches =
        validation.edgeMismatches;
      if (validation.orientation)
        report.trustedProvenanceRejectedOrientation = true;
      if (validation.range)
        report.trustedProvenanceRejectedNeighborRange = true;

      trustedProvenanceNeighbors =
        !report.trustedProvenanceRejectedNeighborRange &&
        !report.trustedProvenanceRejectedNeighborEdgeMismatch &&
        !report.trustedProvenanceRejectedOrientation;
      report.trustedProvenanceNeighborsEligible =
        trustedProvenanceNeighbors;

      if (!trustedProvenanceNeighbors) {
        releaseMeshData(mesh);
        Profile::Scope scope
          ("safe_reduce_rebuild_adjacency_after_provenance_reject");
        mesh = buildMeshData(surface, config, true);
        copyMeshSourceMetrics(report, mesh);
        if (mesh.sourceBufferMismatch()) {
          releaseMeshData(mesh);
          return report;
        }
      }
    }

    report.trustedProvenanceNeighborsUsed = trustedProvenanceNeighbors;
    report.defaultAdjacencySkipped = trustedProvenanceNeighbors;
    copyMeshSourceMetrics(report, mesh);
    report.analysisTriangleRecords = mesh.triangles.size();
    report.analysisAdjacencySlots = mesh.neighbors.size();
    report.analysisEdgeRecords = mesh.edgeInfo.size();

    if (trustedProvenanceNeighbors) {
      const ContourProvenanceReport &provenance =
        sourceTriangleSurface->getContourProvenanceReport();
      report.globalBoundaryEdges = provenance.boundaryEdges;
      report.globalNonManifoldEdges = provenance.nonManifoldEdges;
      report.globalMisorientedEdges = provenance.misorientedEdges;

    } else {
      for (const auto &entry: mesh.edgeInfo)
        if (entry.second.count == 1) report.globalBoundaryEdges++;
        else if (2 < entry.second.count) report.globalNonManifoldEdges++;

      report.globalMisorientedEdges = countMisorientedEdges(mesh.edgeInfo);
    }
    report.globalDegenerateTriangles = mesh.degenerateTriangles;
    report.watertightInput =
      !report.globalBoundaryEdges && !report.globalNonManifoldEdges;
    report.outputBoundaryEdges = report.globalBoundaryEdges;
    report.outputNonManifoldEdges = report.globalNonManifoldEdges;
    report.outputMisorientedEdges = report.globalMisorientedEdges;
    report.outputDegenerateTriangles = report.globalDegenerateTriangles;
    report.watertightOutput = report.watertightInput;

    vector<int32_t> provenanceNeighbors;
    const vector<int32_t> *provenanceNeighborTable = &provenanceNeighbors;
    if (sourceTriangleSurface && sourceTriangleSurface->hasContourProvenance()) {
      const ContourProvenanceReport &provenance =
        sourceTriangleSurface->getContourProvenanceReport();

      report.contourProvenanceAvailable = true;
      report.contourProvenanceTriangles = provenance.triangles;
      report.contourProvenanceCompleteTriangles = provenance.completeTriangles;
      report.contourProvenanceUnknownTriangles = provenance.unknownTriangles;
      report.contourProvenanceRawBoundaryEdges = provenance.rawBoundaryEdges;
      report.contourProvenanceRawNonManifoldEdges =
        provenance.rawNonManifoldEdges;
      report.contourProvenanceRawMisorientedEdges =
        provenance.rawMisorientedEdges;
      report.contourProvenanceRawUniqueEdges =
        provenance.rawUniqueEdges;
      report.contourProvenanceRawMaxEdgeIncidence =
        provenance.rawMaxEdgeIncidence;
      report.contourProvenanceRawEdgesIncidence1 =
        provenance.rawEdgesIncidence1;
      report.contourProvenanceRawEdgesIncidence2 =
        provenance.rawEdgesIncidence2;
      report.contourProvenanceRawEdgesIncidence3 =
        provenance.rawEdgesIncidence3;
      report.contourProvenanceRawEdgesIncidence4 =
        provenance.rawEdgesIncidence4;
      report.contourProvenanceRawEdgesIncidence5Plus =
        provenance.rawEdgesIncidence5Plus;
      report.contourProvenanceRawTwinEdgeSlots =
        provenance.rawTwinEdgeSlots;
      report.contourProvenanceRawBoundaryEdgeSlots =
        provenance.rawBoundaryEdgeSlots;
      report.contourProvenanceRawNonManifoldEdgeSlots =
        provenance.rawNonManifoldEdgeSlots;
      report.contourProvenanceRawGridGridUniqueEdges =
        provenance.rawGridGridUniqueEdges;
      report.contourProvenanceRawGridGridMaxEdgeIncidence =
        provenance.rawGridGridMaxEdgeIncidence;
      report.contourProvenanceRawGridGridTwinEdgeSlots =
        provenance.rawGridGridTwinEdgeSlots;
      report.contourProvenanceRawGridGridBoundaryEdgeSlots =
        provenance.rawGridGridBoundaryEdgeSlots;
      report.contourProvenanceRawGridGridNonManifoldEdgeSlots =
        provenance.rawGridGridNonManifoldEdgeSlots;
      report.contourProvenanceRawGridGridWeldedSpreadEdges =
        provenance.rawGridGridWeldedSpreadEdges;
      report.contourProvenanceRawGridGridWeldedSpreadEdgeSlots =
        provenance.rawGridGridWeldedSpreadEdgeSlots;
      report.contourProvenanceRawGridGridWeldedSpreadMaxAlternateSlots =
        provenance.rawGridGridWeldedSpreadMaxAlternateSlots;
      report.contourProvenanceRawCenterInvolvedUniqueEdges =
        provenance.rawCenterInvolvedUniqueEdges;
      report.contourProvenanceRawCenterInvolvedMaxEdgeIncidence =
        provenance.rawCenterInvolvedMaxEdgeIncidence;
      report.contourProvenanceRawCenterInvolvedTwinEdgeSlots =
        provenance.rawCenterInvolvedTwinEdgeSlots;
      report.contourProvenanceRawCenterInvolvedBoundaryEdgeSlots =
        provenance.rawCenterInvolvedBoundaryEdgeSlots;
      report.contourProvenanceRawCenterInvolvedNonManifoldEdgeSlots =
        provenance.rawCenterInvolvedNonManifoldEdgeSlots;
      report.contourProvenanceRawCenterInvolvedWeldedSpreadEdges =
        provenance.rawCenterInvolvedWeldedSpreadEdges;
      report.contourProvenanceRawCenterInvolvedWeldedSpreadEdgeSlots =
        provenance.rawCenterInvolvedWeldedSpreadEdgeSlots;
      report.contourProvenanceRawCenterInvolvedWeldedSpreadMaxAlternateSlots =
        provenance.rawCenterInvolvedWeldedSpreadMaxAlternateSlots;
      report.contourProvenanceRawGridVertexUniqueKeys =
        provenance.rawGridVertexUniqueKeys;
      report.contourProvenanceRawGridVertexWeldedSpreadKeys =
        provenance.rawGridVertexWeldedSpreadKeys;
      report.contourProvenanceRawGridVertexWeldedSpreadObservations =
        provenance.rawGridVertexWeldedSpreadObservations;
      report.contourProvenanceRawGridVertexWeldedSpreadMaxAlternateObservations =
        provenance.rawGridVertexWeldedSpreadMaxAlternateObservations;
      report.contourProvenanceRawCenterVertexUniqueKeys =
        provenance.rawCenterVertexUniqueKeys;
      report.contourProvenanceRawCenterVertexWeldedSpreadKeys =
        provenance.rawCenterVertexWeldedSpreadKeys;
      report.contourProvenanceRawCenterVertexWeldedSpreadObservations =
        provenance.rawCenterVertexWeldedSpreadObservations;
      report.contourProvenanceRawCenterVertexWeldedSpreadMaxAlternateObservations =
        provenance.rawCenterVertexWeldedSpreadMaxAlternateObservations;
      report.contourProvenanceBoundaryEdges = provenance.boundaryEdges;
      report.contourProvenanceNonManifoldEdges = provenance.nonManifoldEdges;
      report.contourProvenanceMisorientedEdges =
        provenance.misorientedEdges;
      report.contourProvenanceWeldedUniqueEdges =
        provenance.weldedUniqueEdges;
      report.contourProvenanceWeldedMaxEdgeIncidence =
        provenance.weldedMaxEdgeIncidence;
      report.contourProvenanceWeldedEdgesIncidence1 =
        provenance.weldedEdgesIncidence1;
      report.contourProvenanceWeldedEdgesIncidence2 =
        provenance.weldedEdgesIncidence2;
      report.contourProvenanceWeldedEdgesIncidence3 =
        provenance.weldedEdgesIncidence3;
      report.contourProvenanceWeldedEdgesIncidence4 =
        provenance.weldedEdgesIncidence4;
      report.contourProvenanceWeldedEdgesIncidence5Plus =
        provenance.weldedEdgesIncidence5Plus;
      report.contourProvenanceWeldedTwinEdgeSlots =
        provenance.weldedTwinEdgeSlots;
      report.contourProvenanceWeldedBoundaryEdgeSlots =
        provenance.weldedBoundaryEdgeSlots;
      report.contourProvenanceWeldedNonManifoldEdgeSlots =
        provenance.weldedNonManifoldEdgeSlots;
      report.contourProvenanceWatertight = provenance.watertight;
      report.contourProvenanceMatchesInput =
        provenance.triangles == mesh.triangleCount &&
        provenance.boundaryEdges == report.globalBoundaryEdges &&
        provenance.nonManifoldEdges == report.globalNonManifoldEdges &&
        provenance.misorientedEdges == report.globalMisorientedEdges &&
        provenance.watertight == report.watertightInput;

      bool shouldBuildProvenanceNeighbors =
        config.useProvenanceNeighbors || !reduce;

      if (shouldBuildProvenanceNeighbors) {
        bool neighborsAvailable = false;
        if (sourceTriangleSurface->hasContourProvenanceNeighbors()) {
          provenanceNeighborTable =
            &sourceTriangleSurface->getContourProvenanceNeighbors();
          neighborsAvailable = true;
          report.contourProvenanceNeighborsCached = true;
          report.contourProvenanceNeighborsRaw =
            sourceTriangleSurface->hasRawContourProvenanceNeighbors();

        } else {
          Profile::Scope scope("safe_reduce_build_provenance_neighbors");
          bool rawNeighbors = false;
          neighborsAvailable = buildContourProvenanceNeighbors
            (sourceTriangleSurface->getContourProvenance(),
             sourceTriangleSurface->getVertices(), 0, mesh.triangleCount,
             provenanceNeighbors, config.coordTolerance, &rawNeighbors);
          provenanceNeighborTable = &provenanceNeighbors;
          report.contourProvenanceNeighborsRaw = rawNeighbors;
        }

        report.contourProvenanceNeighborsAvailable = neighborsAvailable;
        report.contourProvenanceNeighborSlots =
          (uint64_t)provenanceNeighborTable->size();

        if (neighborsAvailable &&
            provenanceNeighborTable->size() == mesh.neighbors.size()) {
          Profile::Scope scope("safe_reduce_check_provenance_neighbor_parity");
          report.contourProvenanceNeighborParityAudited = true;
          for (size_t i = 0; i < mesh.neighbors.size(); i++)
            if ((*provenanceNeighborTable)[i] != mesh.neighbors[i])
              report.contourProvenanceNeighborMismatches++;

          report.contourProvenanceNeighborParity =
            report.contourProvenanceMatchesInput &&
            !report.contourProvenanceNeighborMismatches;
        } else if (neighborsAvailable && trustedProvenanceNeighbors)
          report.contourProvenanceNeighborParity = false;
      }
    }

    releaseMap(mesh.edgeInfo);

    const vector<int32_t> *classificationNeighbors = &mesh.neighbors;
    if (trustedProvenanceNeighbors ||
        (config.useProvenanceNeighbors &&
         report.contourProvenanceNeighborParity)) {
      classificationNeighbors = provenanceNeighborTable;
      report.usingProvenanceNeighbors = true;
    }

    const vector<float> *sourceVertices = 0;
    const vector<float> *sourceNormals = 0;
    vector<float> outputVertices;
    vector<float> outputNormals;
    vector<uint8_t> replaced;
    vector<ReplacementComponent> replacements;
    unordered_map<uint64_t, uint64_t> replacementOwners;
    vector<ComponentRecord> componentRecords;

    if (reduce) {
      sourceVertices = &triangleSurface->getVertices();
      sourceNormals = &triangleSurface->getNormals();
      replaced.assign((size_t)mesh.triangleCount, 0);
      replacements.reserve(1024);
      replacementOwners.reserve(1024);
      componentRecords.reserve(1024);
    }

    vector<uint8_t> planeFitMasks;
    vector<uint8_t> planeFitVertexChecks;
    const double normalTolerance =
      cos(config.pairwiseNormalAngleDegrees / 2.0 * PI / 180.0);
    unsigned planeFitThreads = min<uint64_t>
      (max(1U, config.threads), mesh.triangleCount);
    if (1 < planeFitThreads && 4096 <= mesh.triangleCount) {
      Profile::Scope scope("safe_reduce_cache_plane_fits");
      planeFitMasks.resize((size_t)mesh.triangleCount);
      planeFitVertexChecks.resize((size_t)mesh.triangleCount * 3);
      atomic<uint64_t> nextTriangle(0);
      vector<thread> workers;
      workers.reserve(planeFitThreads);
      exception_ptr workerError;
      mutex workerErrorMutex;

      auto worker = [&] () {
        try {
          const uint64_t chunkSize = 4096;
          while (true) {
            uint64_t begin = nextTriangle.fetch_add
              (chunkSize, memory_order_relaxed);
            if (mesh.triangleCount <= begin) break;
            uint64_t end = min(mesh.triangleCount, begin + chunkSize);

            for (uint64_t tri = begin; tri < end; tri++) {
              const Vec3 &seedNormal = mesh.normals[tri];
              if (!length(seedNormal)) continue;
              const Vec3 &seedPoint =
                mesh.points[mesh.triangles[(size_t)tri][0]];
              double seedDistance = dot(seedNormal, seedPoint);

              for (unsigned slot = 0; slot < 3; slot++) {
                int32_t neighbor =
                  (*classificationNeighbors)[tri * 3 + slot];
                if (neighbor < 0) continue;
                uint64_t vertexChecks = 0;
                bool fits = fitsSeedPlaneAcrossEdge
                  ((uint64_t)neighbor,
                   getTriangleEdge(mesh.triangles[(size_t)tri], slot),
                   seedNormal, seedDistance, mesh.triangles, mesh.normals,
                   mesh.points, config, normalTolerance, vertexChecks);
                if (fits) planeFitMasks[(size_t)tri] |= 1U << slot;
                planeFitVertexChecks[(size_t)tri * 3 + slot] =
                  (uint8_t)min<uint64_t>(vertexChecks, 255);
              }
            }
          }
        } catch (...) {
          lock_guard<mutex> lock(workerErrorMutex);
          if (!workerError) workerError = current_exception();
        }
      };

      try {
        for (unsigned i = 0; i < planeFitThreads; i++)
          workers.emplace_back(worker);
      } catch (...) {
        for (thread &workerThread: workers) workerThread.join();
        throw;
      }
      for (thread &workerThread: workers) workerThread.join();
      if (workerError) rethrow_exception(workerError);
      report.planeFitCacheSlots = mesh.triangleCount * 3;
      report.planeFitCacheThreads = planeFitThreads;
    }

    {
      Profile::Scope scope("safe_reduce_classify_components");

      vector<uint8_t> visited((size_t)mesh.triangleCount);
      vector<int32_t> componentMark((size_t)mesh.triangleCount, -1);
      vector<uint64_t> stack;
      vector<uint64_t> component;
      vector<BoundaryEdge> boundaryEdges;
      vector<PendingReplacement> pendingReplacements;
      pendingReplacements.reserve(256);
      for (uint64_t seed = 0; seed < mesh.triangleCount; seed++) {
        if (visited[seed]) continue;

        const Vec3 seedNormal = mesh.normals[seed];
        if (!length(seedNormal)) {
          recordZeroNormalSideTriangle(report, mesh, seed);
          visited[seed] = 1;
          continue;
        }

        const Vec3 &seedPoint = mesh.points[mesh.triangles[seed][0]];
        double seedDistance = dot(seedNormal, seedPoint);

        component.clear();
        stack.clear();
        stack.push_back(seed);
        visited[seed] = 1;
        componentMark[seed] = (int32_t)report.components;
        bool cachedSingleton =
          !planeFitMasks.empty() && !planeFitMasks[(size_t)seed];

        while (!stack.empty()) {
          uint64_t tri = stack.back();
          stack.pop_back();
          component.push_back(tri);
          report.componentNeighborSlots += 3;

          for (unsigned slot = 0; slot < 3; slot++) {
            int32_t neighbor = (*classificationNeighbors)[tri * 3 + slot];
            if (neighbor < 0 || visited[neighbor]) continue;
            report.componentNeighborCandidates++;
            report.componentPlaneFitTests++;

            uint64_t vertexChecks = 0;
            bool fits = false;
            if (cachedSingleton && tri == seed) {
              fits = planeFitMasks[(size_t)tri] & (1U << slot);
              vertexChecks =
                planeFitVertexChecks[(size_t)tri * 3 + slot];
            } else
              fits = fitsSeedPlaneAcrossEdge
                (neighbor, getTriangleEdge(mesh.triangles[tri], slot),
                 seedNormal, seedDistance, mesh.triangles, mesh.normals,
                 mesh.points, config, normalTolerance, vertexChecks);
            report.componentPlaneVertexChecks += vertexChecks;
            if (fits) {
              report.componentPlaneFitAccepted++;
              visited[neighbor] = 1;
              componentMark[neighbor] = (int32_t)report.components;
              stack.push_back(neighbor);
            }
          }
        }

        if (component.size() == 1) report.singleTriangleComponents++;
        if (component.size() < 8) report.componentsLt8Triangles++;
        if (component.size() < 64) report.componentsLt64Triangles++;
        report.maxComponentTriangles =
          max<uint64_t>(report.maxComponentTriangles, component.size());
        unsigned sideId = classifyComponentSide(mesh, component);
        PlanarReductionSideReport &side = recordSideComponent
          (report, mesh, component);

        if (component.size() == 1) {
          // A valid single triangle has a three-edge, one-loop boundary and
          // can never yield a smaller triangulation.  Preserve all accounting
          // while avoiding per-component hash tables, loop ordering, and
          // retained heap-backed records.
          BoundaryInfo boundary;
          boundary.vertices = 3;
          boundary.edges = 3;
          boundary.loops = 1;
          ReplacementCheck replacement;
          report.boundaryEdgeScans += 3;
          report.componentBoundaryEdges += 3;
          report.boundaryInfoChecks++;
          addClassifiedComponent(report, 1, 1, boundary, replacement);
          addClassifiedSideComponent(side, 1, 1, boundary, replacement);
          if (reduce) report.componentRecordsSkipped++;
          report.components++;
          continue;
        }

        prepareBoundaryEdges(boundaryEdges, component.size());
        uint64_t boundaryFingerprint = 0;

        for (uint64_t tri: component)
          for (unsigned slot = 0; slot < 3; slot++) {
            report.boundaryEdgeScans++;
            int32_t neighbor = (*classificationNeighbors)[tri * 3 + slot];
            if (neighbor < 0 ||
                componentMark[neighbor] != (int32_t)report.components) {
              BoundaryEdge edge = getTriangleEdge(mesh.triangles[tri], slot);
              boundaryEdges.push_back(edge);
              mixBoundaryEdge(boundaryFingerprint, edge);
              report.componentBoundaryEdges++;
            }
          }

        report.boundaryInfoChecks++;
        BoundaryInfo boundary = getBoundaryInfo(boundaryEdges);
        vector<vector<uint32_t> > loops;
        bool loopsOrdered = false;
        if (reduce && config.applyBoundaryCoSimplify)
          loopsOrdered = orderBoundaryLoops(boundaryEdges, loops);
        uint64_t estimatedAfter = estimateTrianglesAfter(boundary);
        BoundaryCoSimplifyEstimate boundaryCoSimplify;
        if (config.applyBoundaryCoSimplify)
          boundaryCoSimplify = estimateBoundaryCoSimplify
            (boundaryEdges, mesh.points,
             max(config.coordTolerance * 2,
                 config.planeDistanceTolerance));
        recordBoundaryCoSimplifyEstimate
          (report, side, component.size(), boundaryCoSimplify);
        bool replacementCandidate =
          estimatedAfter && estimatedAfter < component.size() &&
          !boundary.badVertices && !boundary.duplicateEdges;
        if (replacementCandidate) {
          PendingReplacement pending;
          pending.id = report.components;
          pending.triangles = component;
          pending.boundaryEdges = boundaryEdges;
          pending.loops = std::move(loops);
          pending.normal = seedNormal;
          pending.sideId = sideId;
          pending.boundary = boundary;
          pending.boundaryCoSimplify = boundaryCoSimplify;
          pending.estimatedAfter = estimatedAfter;
          pending.boundaryFingerprint = boundaryFingerprint;
          pending.canApplyReplacement =
            boundary.loops == 1 || config.applyHoleAware;
          pending.loopsOrdered = loopsOrdered;
          pendingReplacements.push_back(std::move(pending));
          report.components++;
          continue;
        }

        ReplacementCheck replacement;
        bool canApplyReplacement =
          boundary.loops == 1 || config.applyHoleAware;

        uint64_t reduction =
          addClassifiedComponent(report, component.size(), estimatedAfter,
                                 boundary, replacement);
        addClassifiedSideComponent(side, component.size(), estimatedAfter,
                                   boundary, replacement);
        bool decisionBearing =
          reduction || replacement.checked || replacement.estimateAvailable ||
          replacement.edgeIncidenceChecked;
        if (decisionBearing) {
          report.decisionBearingComponents++;
          report.decisionBearingTriangles += component.size();
          recordComponentDecisionFingerprint
            (report.componentDecisionFingerprint, component.size(),
             boundaryFingerprint, boundary, estimatedAfter, reduction,
             replacement);
        }

        bool directReplacementEligible =
          reduction && canApplyReplacement && replacement.feasible &&
          !replacement.triangles.empty();
        bool contractEligible =
          config.applyBoundaryCoSimplify && loopsOrdered &&
          canApplyReplacement && boundaryCoSimplify.extraReduction &&
          !boundary.badVertices && !boundary.duplicateEdges;

        if (reduce && (directReplacementEligible || contractEligible)) {
          ComponentRecord record;
          record.id = report.components;
          record.triangles = component;
          record.boundaryEdges = boundaryEdges;
          record.loops = std::move(loops);
          record.normal = seedNormal;
          record.sideId = sideId;
          record.boundary = boundary;
          record.boundaryCoSimplify = boundaryCoSimplify;
          record.replacement = replacement;
          record.estimatedAfter = estimatedAfter;
          record.canApplyReplacement = canApplyReplacement;
          record.contractEligible = contractEligible;
          componentRecords.push_back(std::move(record));
          report.componentRecordsRetained++;

        } else if (reduce) report.componentRecordsSkipped++;

        report.components++;
      }

      if (!pendingReplacements.empty()) {
        {
          Profile::Scope replacementScope
            ("safe_reduce_check_replacements_parallel");
          unsigned workerCount = min<size_t>
            (max(1U, config.threads), pendingReplacements.size());
          atomic<size_t> nextReplacement(0);
          vector<thread> workers;
          workers.reserve(workerCount);
          exception_ptr workerError;
          mutex workerErrorMutex;

          auto worker = [&] () {
            try {
              while (true) {
                size_t index = nextReplacement.fetch_add
                  (1, memory_order_relaxed);
                if (pendingReplacements.size() <= index) break;
                PendingReplacement &pending = pendingReplacements[index];
                if (pending.boundary.loops == 1)
                  pending.replacement = checkPhase1Replacement
                    (pending.boundaryEdges, mesh.points, pending.normal);
                else if (1 < pending.boundary.loops && config.applyHoleAware)
                  pending.replacement = checkHoleAwareReplacement
                    (pending.boundaryEdges, mesh.points, pending.normal);
              }
            } catch (...) {
              lock_guard<mutex> lock(workerErrorMutex);
              if (!workerError) workerError = current_exception();
            }
          };

          try {
            for (unsigned i = 0; i < workerCount; i++)
              workers.emplace_back(worker);
          } catch (...) {
            for (thread &workerThread: workers) workerThread.join();
            throw;
          }
          for (thread &workerThread: workers) workerThread.join();
          if (workerError) rethrow_exception(workerError);
        }

        // Aggregate in original component order so report fingerprints and
        // output replacement ownership remain deterministic.
        for (PendingReplacement &pending: pendingReplacements) {
          ReplacementCheck &replacement = pending.replacement;
          uint64_t estimatedAfter = pending.estimatedAfter;
          if (pending.boundary.loops == 1)
            report.phase1ReplacementChecks++;
          else if (replacement.checked)
            report.holeAwareReplacementChecks++;

          if (replacement.edgeIncidenceChecked) {
            report.replacementEdgeIncidenceChecks++;
            if (!replacement.edgeIncidenceOk) {
              report.replacementEdgeIncidenceRejected++;
              if (pending.boundary.loops == 1)
                report.phase1ReplacementEdgeIncidenceRejected++;
              else if (1 < pending.boundary.loops)
                report.holeAwareReplacementEdgeIncidenceRejected++;
            }
          }
          if (replacement.complexityRejected)
            report.replacementComplexityRejected++;

          if (replacement.feasible || replacement.estimateAvailable) {
            report.estimatedReplacementChecks++;
            estimatedAfter = replacement.trianglesAfter;
          }

          if (replacement.feasible && !replacement.triangles.empty()) {
            report.feasibleReplacementChecks++;
            report.writableReplacementChecks++;
            if (pending.boundary.loops == 1)
              report.phase1WritableReplacementChecks++;
            else if (1 < pending.boundary.loops)
              report.holeAwareWritableReplacementChecks++;

          } else if (replacement.estimateAvailable) {
            report.unwritableReplacementChecks++;
            if (pending.boundary.loops == 1)
              report.phase1UnwritableReplacementChecks++;
            else if (1 < pending.boundary.loops)
              report.holeAwareUnwritableReplacementChecks++;
          }

          PlanarReductionSideReport &side = report.sides[pending.sideId];
          uint64_t reduction = addClassifiedComponent
            (report, pending.triangles.size(), estimatedAfter,
             pending.boundary, replacement);
          addClassifiedSideComponent
            (side, pending.triangles.size(), estimatedAfter,
             pending.boundary, replacement);
          bool decisionBearing =
            reduction || replacement.checked ||
            replacement.estimateAvailable ||
            replacement.edgeIncidenceChecked;
          if (decisionBearing) {
            report.decisionBearingComponents++;
            report.decisionBearingTriangles += pending.triangles.size();
            recordComponentDecisionFingerprint
              (report.componentDecisionFingerprint,
               pending.triangles.size(), pending.boundaryFingerprint,
               pending.boundary, estimatedAfter, reduction, replacement);
          }

          bool directReplacementEligible =
            reduction && pending.canApplyReplacement &&
            replacement.feasible && !replacement.triangles.empty();
          bool contractEligible =
            config.applyBoundaryCoSimplify && pending.loopsOrdered &&
            pending.canApplyReplacement &&
            pending.boundaryCoSimplify.extraReduction &&
            !pending.boundary.badVertices &&
            !pending.boundary.duplicateEdges;

          if (reduce && (directReplacementEligible || contractEligible)) {
            ComponentRecord record;
            record.id = pending.id;
            record.triangles = std::move(pending.triangles);
            record.boundaryEdges = std::move(pending.boundaryEdges);
            record.loops = std::move(pending.loops);
            record.normal = pending.normal;
            record.sideId = pending.sideId;
            record.boundary = pending.boundary;
            record.boundaryCoSimplify = pending.boundaryCoSimplify;
            record.replacement = std::move(replacement);
            record.estimatedAfter = estimatedAfter;
            record.canApplyReplacement = pending.canApplyReplacement;
            record.contractEligible = contractEligible;
            componentRecords.push_back(std::move(record));
            report.componentRecordsRetained++;

          } else if (reduce) report.componentRecordsSkipped++;
        }
      }
    }
    releaseVector(planeFitMasks);
    releaseVector(planeFitVertexChecks);

    sort(componentRecords.begin(), componentRecords.end(),
         [] (const ComponentRecord &a, const ComponentRecord &b) {
           return a.id < b.id;
         });

    if (reduce) {
      unordered_set<uint32_t> acceptedContractVertices;

      if (config.applyBoundaryCoSimplify) {
        unordered_map<uint32_t, VertexContractStats> vertexStats;
        vertexStats.reserve(mesh.points.size());

        const double contractTolerance =
          max(config.coordTolerance * 2, config.planeDistanceTolerance);

        for (ComponentRecord &record: componentRecords)
          if (record.contractEligible)
            report.boundaryCoSimplifyContractComponentsConsidered++;
        recordContractEdgeVertexUse(componentRecords, mesh.points,
                                    contractTolerance, vertexStats);

        acceptedContractVertices =
          buildAcceptedContractVertices(vertexStats, report);
        unordered_set<uint32_t> acceptedContractChainVertices =
          buildAcceptedContractChainVertices(componentRecords, mesh.points,
                                             contractTolerance, report);
        acceptedContractVertices.insert(acceptedContractChainVertices.begin(),
                                        acceptedContractChainVertices.end());
        report.boundaryCoSimplifyContractVerticesAccepted =
          acceptedContractVertices.size();

        bool contractOk = !acceptedContractVertices.empty();
        uint64_t ordinaryAffectedOutput = 0;
        uint64_t contractAffectedOutput = 0;

        for (ComponentRecord &record: componentRecords) {
          record.contractAffected =
            record.contractEligible &&
            componentHasContractVertex(record, acceptedContractVertices);
          if (!record.contractAffected) continue;

          report.boundaryCoSimplifyContractComponentsAffected++;
          report.boundaryCoSimplifyContractReplacementChecks++;
          record.contractReplacement =
            checkContractBoundaryCoSimplifyReplacement
            (record, mesh.points, acceptedContractVertices);

          if (!record.contractReplacement.feasible ||
              record.contractReplacement.triangles.empty()) {
            contractOk = false;
            if (record.contractReplacement.edgeIncidenceChecked &&
                !record.contractReplacement.edgeIncidenceOk)
              report.boundaryCoSimplifyContractEdgeIncidenceRejected++;
            else
              report.boundaryCoSimplifyContractTriangulationRejected++;
            continue;
          }

          uint64_t ordinaryOutput =
            record.replacement.feasible && !record.replacement.triangles.empty()
            ? record.replacement.triangles.size() : record.triangles.size();
          if (ordinaryOutput < record.contractReplacement.triangles.size()) {
            contractOk = false;
            report.boundaryCoSimplifyContractNoSavingsRejected++;
          }

          ordinaryAffectedOutput += ordinaryOutput;
          contractAffectedOutput +=
            record.contractReplacement.triangles.size();
        }

        if (contractOk &&
            (!report.boundaryCoSimplifyContractComponentsAffected ||
             ordinaryAffectedOutput <= contractAffectedOutput)) {
          contractOk = false;
          report.boundaryCoSimplifyContractNoSavingsRejected++;
        }

        if (!contractOk) {
          if (!acceptedContractVertices.empty() ||
              report.boundaryCoSimplifyContractComponentsAffected)
            report.boundaryCoSimplifyContractGlobalRejected++;

        } else {
          for (ComponentRecord &record: componentRecords)
            if (record.contractAffected) record.contractAccepted = true;
        }
      }

      for (const ComponentRecord &record: componentRecords) {
        const ReplacementCheck *selectedReplacement = &record.replacement;
        bool selectedContract = false;

        if (record.contractAccepted &&
            record.contractReplacement.feasible &&
            !record.contractReplacement.triangles.empty()) {
          uint64_t ordinaryOutput =
            record.replacement.feasible && !record.replacement.triangles.empty()
            ? record.replacement.triangles.size() : record.triangles.size();
          if (record.contractReplacement.triangles.size() <= ordinaryOutput) {
            selectedReplacement = &record.contractReplacement;
            selectedContract = true;
          }
        }

        uint64_t selectedReduction =
          selectedReplacement->triangles.size() < record.triangles.size() ?
          record.triangles.size() - selectedReplacement->triangles.size() : 0;
        if (!selectedReduction || !record.canApplyReplacement ||
            !selectedReplacement->feasible ||
            selectedReplacement->triangles.empty())
          continue;

        uint64_t owner = record.triangles[0];
        uint64_t index = replacements.size();
        uint64_t replacementTriangleCount =
          selectedReplacement->triangles.size();
        vector<array<uint32_t, 3> > selectedTriangles =
          selectedReplacement->triangles;
        replacements.push_back
          (ReplacementComponent{owner, std::move(selectedTriangles),
                                record.normal});
        replacementOwners[owner] = index;

        for (uint64_t tri: record.triangles) replaced[(size_t)tri] = 1;

        report.appliedComponents++;
        report.appliedSourceTriangles += record.triangles.size();
        report.appliedOutputTriangles += replacementTriangleCount;
        PlanarReductionSideReport &side = report.sides[record.sideId];
        side.appliedComponents++;
        side.appliedSourceTriangles += record.triangles.size();
        side.appliedOutputTriangles += replacementTriangleCount;

        if (selectedContract) {
          report.boundaryCoSimplifyContractAppliedComponents++;
          report.boundaryCoSimplifyContractAppliedSourceTriangles +=
            record.triangles.size();
          report.boundaryCoSimplifyContractAppliedOutputTriangles +=
            replacementTriangleCount;

        } else if (1 < record.boundary.loops) {
          report.holeAwareAppliedComponents++;
          report.holeAwareAppliedSourceTriangles += record.triangles.size();
          report.holeAwareAppliedOutputTriangles += replacementTriangleCount;
        }
      }
    }

    if (!reduce && report.contourProvenanceNeighborParity &&
        !report.usingProvenanceNeighbors) {
      ComponentSummary provenanceSummary;
      {
        Profile::Scope scope("safe_reduce_classify_provenance_components");
        provenanceSummary = classifyComponentSummary
          (mesh, *provenanceNeighborTable, config);
      }

      report.contourProvenanceComponentReportAvailable =
        provenanceSummary.available;
      report.contourProvenanceComponents = provenanceSummary.components;
      report.contourProvenanceComponentDecisionFingerprint =
        provenanceSummary.componentDecisionFingerprint;
      report.contourProvenanceDecisionBearingComponents =
        provenanceSummary.decisionBearingComponents;
      report.contourProvenanceDecisionBearingTriangles =
        provenanceSummary.decisionBearingTriangles;
      report.contourProvenanceEstimatedTrianglesAfter =
        provenanceSummary.estimatedTrianglesAfter;
      report.contourProvenanceEstimatedTriangleReduction =
        provenanceSummary.estimatedTriangleReduction;
      report.contourProvenancePhase1Components =
        provenanceSummary.phase1Components;
      report.contourProvenanceHoleAwareComponents =
        provenanceSummary.holeAwareComponents;
      report.contourProvenanceEstimatedReplacementChecks =
        provenanceSummary.estimatedReplacementChecks;
      report.contourProvenanceFeasibleReplacementChecks =
        provenanceSummary.feasibleReplacementChecks;
      report.contourProvenanceWritableReplacementChecks =
        provenanceSummary.writableReplacementChecks;
      report.contourProvenanceUnwritableReplacementChecks =
        provenanceSummary.unwritableReplacementChecks;
      report.contourProvenancePhase1WritableReplacementChecks =
        provenanceSummary.phase1WritableReplacementChecks;
      report.contourProvenanceHoleAwareWritableReplacementChecks =
        provenanceSummary.holeAwareWritableReplacementChecks;
      report.contourProvenancePhase1UnwritableReplacementChecks =
        provenanceSummary.phase1UnwritableReplacementChecks;
      report.contourProvenanceHoleAwareUnwritableReplacementChecks =
        provenanceSummary.holeAwareUnwritableReplacementChecks;
      report.contourProvenanceReplacementEdgeIncidenceChecks =
        provenanceSummary.replacementEdgeIncidenceChecks;
      report.contourProvenanceReplacementEdgeIncidenceRejected =
        provenanceSummary.replacementEdgeIncidenceRejected;
      report.contourProvenancePhase1ReplacementEdgeIncidenceRejected =
        provenanceSummary.phase1ReplacementEdgeIncidenceRejected;
      report.contourProvenanceHoleAwareReplacementEdgeIncidenceRejected =
        provenanceSummary.holeAwareReplacementEdgeIncidenceRejected;
      report.contourProvenanceRejectedBoundaryComponents =
        provenanceSummary.rejectedBoundaryComponents;
      report.contourProvenanceRejectedNoSavingsComponents =
        provenanceSummary.rejectedNoSavingsComponents;
      report.contourProvenanceRejectedTriangulationComponents =
        provenanceSummary.rejectedTriangulationComponents;
      report.contourProvenanceComponentMetricMismatches =
        countComponentSummaryMismatches(report, provenanceSummary);
      report.contourProvenanceComponentParity =
        report.contourProvenanceComponentReportAvailable &&
        !report.contourProvenanceComponentMetricMismatches;
    }
    releaseVector(provenanceNeighbors);

    finalizeSideReports(report);
    if (!reduce || !report.appliedComponents) return report;
    releaseMeshConnectivity(mesh);

    uint64_t expectedOutputTriangles =
      report.triangles - report.appliedSourceTriangles +
      report.appliedOutputTriangles;
    report.validationExpectedOutputTriangles = expectedOutputTriangles;
    {
      Profile::Scope scope("safe_reduce_build_candidate");

      outputVertices.reserve((size_t)expectedOutputTriangles * 9);
      outputNormals.reserve((size_t)expectedOutputTriangles * 9);

      for (uint64_t tri = 0; tri < mesh.triangleCount; tri++) {
        if (!replaced[(size_t)tri]) {
          appendOriginalTriangle(outputVertices, outputNormals, *sourceVertices,
                                 *sourceNormals, tri);
          continue;
        }

        auto it = replacementOwners.find(tri);
        if (it == replacementOwners.end()) continue;

        const ReplacementComponent &replacement = replacements[it->second];
        appendReducedTriangles(outputVertices, outputNormals,
                               replacement.triangles, mesh.points,
                               replacement.normal);
      }

    }
    report.validationVertexCountMismatch = outputVertices.size() % 9 != 0;
    report.validationNormalCountMismatch =
      outputNormals.size() != outputVertices.size();
    report.validationCandidateTriangles = outputVertices.size() / 9;
    report.validationTriangleCountMismatch =
      report.validationCandidateTriangles != expectedOutputTriangles;

    releaseVector(replaced);
    releaseVector(replacements);
    releaseMap(replacementOwners);
    releaseMeshData(mesh);
    sourceVertices = 0;
    sourceNormals = 0;

    EdgeIncidenceReport validation;
    {
      Profile::Scope scope("safe_reduce_validate_candidate_vertices");
      validation = validateEdgeIncidenceVertices(outputVertices, config);
    }

    report.outputBoundaryEdges = validation.boundaryEdges;
    report.outputNonManifoldEdges = validation.nonManifoldEdges;
    report.outputMisorientedEdges = validation.misorientedEdges;
    report.outputDegenerateTriangles = validation.degenerateTriangles;
    report.watertightOutput = validation.watertight();
    report.validationCandidateChecked = true;
    report.validationCandidateBoundaryEdges = validation.boundaryEdges;
    report.validationCandidateNonManifoldEdges = validation.nonManifoldEdges;
    report.validationCandidateMisorientedEdges =
      validation.misorientedEdges;
    report.validationCandidateDegenerateTriangles =
      validation.degenerateTriangles;
    report.validationCandidateWatertight = validation.watertight();
    bool rollback = shouldRollbackCandidateValidation
      (report.globalBoundaryEdges, report.globalNonManifoldEdges,
       report.globalMisorientedEdges, report.globalDegenerateTriangles,
       report.validationVertexCountMismatch,
       report.validationNormalCountMismatch,
       report.validationTriangleCountMismatch, validation,
       report.validationTopologyWorse, report.validationDegenerateWorse,
       report.validationOrientationWorse);

    if (rollback) {
      report.validationRolledBack = true;
      recordSideValidationRollback(report);
      resetAppliedReduction(report);
      report.outputTriangles = report.triangles;
      report.outputBoundaryEdges = report.globalBoundaryEdges;
      report.outputNonManifoldEdges = report.globalNonManifoldEdges;
      report.outputMisorientedEdges = report.globalMisorientedEdges;
      report.outputDegenerateTriangles = report.globalDegenerateTriangles;
      report.watertightOutput = report.watertightInput;
      finalizeSideReports(report);
      return report;
    }

    triangleSurface->replace(std::move(outputVertices), std::move(outputNormals));
    report.outputTriangles = triangleSurface->getTriangleCount();
    finalizeSideReports(report);
    return report;
  }


  void appendSelfTestSquare(vector<Vec3> &points,
                            vector<BoundaryEdge> &edges,
                            double x, double y, double size,
                            bool clockwise) {
    uint32_t base = (uint32_t)points.size();
    points.push_back(Vec3{x, y, 0});
    points.push_back(Vec3{x + size, y, 0});
    points.push_back(Vec3{x + size, y + size, 0});
    points.push_back(Vec3{x, y + size, 0});

    if (clockwise) {
      edges.push_back(BoundaryEdge(base, base + 3));
      edges.push_back(BoundaryEdge(base + 3, base + 2));
      edges.push_back(BoundaryEdge(base + 2, base + 1));
      edges.push_back(BoundaryEdge(base + 1, base));

    } else {
      edges.push_back(BoundaryEdge(base, base + 1));
      edges.push_back(BoundaryEdge(base + 1, base + 2));
      edges.push_back(BoundaryEdge(base + 2, base + 3));
      edges.push_back(BoundaryEdge(base + 3, base));
    }
  }


  void appendSelfTestLoop(vector<Vec3> &points,
                          vector<BoundaryEdge> &edges,
                          const vector<Vec3> &loop,
                          bool clockwise) {
    uint32_t base = (uint32_t)points.size();
    points.insert(points.end(), loop.begin(), loop.end());

    if (clockwise)
      for (size_t i = 0; i < loop.size(); i++)
        edges.push_back(BoundaryEdge
                        (base + (uint32_t)((loop.size() - i) % loop.size()),
                         base + (uint32_t)(loop.size() - i - 1)));
    else
      for (size_t i = 0; i < loop.size(); i++)
        edges.push_back(BoundaryEdge
                        (base + (uint32_t)i,
                         base + (uint32_t)((i + 1) % loop.size())));
  }


  Vec3 selfTestPlanePoint(const Vec3 &origin, const Vec3 &uAxis,
                          const Vec3 &vAxis, double u, double v) {
    return Vec3{
      origin.x + uAxis.x * u + vAxis.x * v,
      origin.y + uAxis.y * u + vAxis.y * v,
      origin.z + uAxis.z * u + vAxis.z * v,
    };
  }


  void appendSelfTestPlaneLoop
    (vector<Vec3> &points, vector<BoundaryEdge> &edges,
     const Vec3 &origin, const Vec3 &uAxis, const Vec3 &vAxis,
     const vector<pair<double, double> > &coords, bool clockwise) {
    vector<Vec3> loop;
    loop.reserve(coords.size());
    for (const auto &coord: coords)
      loop.push_back(selfTestPlanePoint(origin, uAxis, vAxis,
                                        coord.first, coord.second));

    appendSelfTestLoop(points, edges, loop, clockwise);
  }


  bool requireSelfTest(bool condition, const string &message,
                       string &failure) {
    if (condition) return true;

    failure = message;
    return false;
  }


  EdgeIncidenceReport selfTestIncidence
    (uint64_t boundaryEdges, uint64_t nonManifoldEdges,
     uint64_t misorientedEdges, uint64_t degenerateTriangles) {
    EdgeIncidenceReport report;
    report.boundaryEdges = boundaryEdges;
    report.nonManifoldEdges = nonManifoldEdges;
    report.misorientedEdges = misorientedEdges;
    report.degenerateTriangles = degenerateTriangles;
    return report;
  }


  bool requireValidationDecision
    (const string &name,
     uint64_t inputBoundaryEdges, uint64_t inputNonManifoldEdges,
     uint64_t inputMisorientedEdges, uint64_t inputDegenerateTriangles,
     const EdgeIncidenceReport &validation, bool vertexCountMismatch,
     bool normalCountMismatch, bool triangleCountMismatch,
     bool expectedTopologyWorse, bool expectedDegenerateWorse,
     bool expectedOrientationWorse, bool expectedRollback,
     string &failure) {
    bool topologyWorse = false;
    bool degenerateWorse = false;
    bool orientationWorse = false;
    bool rollback = shouldRollbackCandidateValidation
      (inputBoundaryEdges, inputNonManifoldEdges, inputMisorientedEdges,
       inputDegenerateTriangles, vertexCountMismatch, normalCountMismatch,
       triangleCountMismatch, validation,
       topologyWorse, degenerateWorse, orientationWorse);

    if (!requireSelfTest(topologyWorse == expectedTopologyWorse,
                         name + " topology decision changed", failure))
      return false;
    if (!requireSelfTest(degenerateWorse == expectedDegenerateWorse,
                         name + " degenerate decision changed", failure))
      return false;
    if (!requireSelfTest(orientationWorse == expectedOrientationWorse,
                         name + " orientation decision changed", failure))
      return false;
    if (!requireSelfTest(rollback == expectedRollback,
                         name + " rollback decision changed", failure))
      return false;

    return true;
  }


  bool requireSourcePreflightDecision
    (const string &name, size_t vertexFloats, size_t normalFloats,
     bool expectedVertexMismatch, bool expectedNormalMismatch,
     string &failure) {
    PlanarReductionConfig config;
    uint64_t expectedTriangles = vertexFloats / 9;
    uint64_t expectedFloats = expectedTriangles * 9;

    auto checkReport =
      [&](const PlanarReductionReport &report, const string &mode) -> bool {
        string prefix = name + " " + mode;
        if (!requireSelfTest(report.triangles == expectedTriangles,
                             prefix + " triangle count changed", failure))
          return false;
        if (!requireSelfTest(report.outputTriangles == expectedTriangles,
                             prefix + " output triangle count changed",
                             failure))
          return false;
        if (!requireSelfTest(report.sourceExpectedFloats == expectedFloats,
                             prefix + " expected source float count changed",
                             failure))
          return false;
        if (!requireSelfTest(report.sourceVertexFloats == vertexFloats,
                             prefix + " vertex source float count changed",
                             failure))
          return false;
        if (!requireSelfTest(report.sourceNormalFloats == normalFloats,
                             prefix + " normal source float count changed",
                             failure))
          return false;
        if (!requireSelfTest(report.sourceVertexCountMismatch ==
                             expectedVertexMismatch,
                             prefix + " vertex mismatch decision changed",
                             failure))
          return false;
        if (!requireSelfTest(report.sourceNormalCountMismatch ==
                             expectedNormalMismatch,
                             prefix + " normal mismatch decision changed",
                             failure))
          return false;
        if (!requireSelfTest(!report.components,
                             prefix + " unexpectedly classified components",
                             failure))
          return false;
        if (!requireSelfTest(!report.validationCandidateChecked,
                             prefix + " unexpectedly built a candidate",
                             failure))
          return false;
        if (!requireSelfTest(!report.appliedComponents,
                             prefix + " unexpectedly applied reductions",
                             failure))
          return false;

        return true;
      };

    TriangleSurface analyzeSurface;
    vector<float> analyzeVertices(vertexFloats, 0);
    vector<float> analyzeNormals(normalFloats, 0);
    analyzeSurface.replace(std::move(analyzeVertices),
                           std::move(analyzeNormals));
    if (!checkReport(analyzeOrReduceImpl(analyzeSurface, config, 0), "analyze"))
      return false;

    TriangleSurface reduceSurface;
    vector<float> reduceVertices(vertexFloats, 1);
    vector<float> reduceNormals(normalFloats, 2);
    reduceSurface.replace(reduceVertices, reduceNormals);
    if (!checkReport(analyzeOrReduceImpl(reduceSurface, config, &reduceSurface),
                     "reduce"))
      return false;
    if (!requireSelfTest(reduceSurface.getVertices() == reduceVertices,
                         name + " reduce mutated source vertices", failure))
      return false;
    if (!requireSelfTest(reduceSurface.getNormals() == reduceNormals,
                         name + " reduce mutated source normals", failure))
      return false;

    return true;
  }


  bool requireTrustedNeighborFlags
    (const string &name, const TrustedNeighborValidation &validation,
     bool openSlot, bool range, bool self, bool duplicate, bool asymmetry,
     bool orientation, bool edgeMismatch, string &failure) {
    if (!requireSelfTest(validation.openSlot == openSlot,
                         name + " open-slot flag changed", failure))
      return false;
    if (!requireSelfTest(validation.range == range,
                         name + " range flag changed", failure))
      return false;
    if (!requireSelfTest(validation.self == self,
                         name + " self-neighbor flag changed", failure))
      return false;
    if (!requireSelfTest(validation.duplicate == duplicate,
                         name + " duplicate-neighbor flag changed", failure))
      return false;
    if (!requireSelfTest(validation.asymmetry == asymmetry,
                         name + " asymmetry flag changed", failure))
      return false;
    if (!requireSelfTest(validation.orientation == orientation,
                         name + " orientation flag changed", failure))
      return false;
    if (!requireSelfTest(validation.edgeMismatch == edgeMismatch,
                         name + " edge-mismatch flag changed", failure))
      return false;

    return true;
  }


  bool requireNoTrustedNeighborFlags
    (const string &name, const TrustedNeighborValidation &validation,
     string &failure) {
    return requireTrustedNeighborFlags
      (name, validation, false, false, false, false, false, false, false,
       failure);
  }


  MeshData makeTrustedNeighborSelfTestMesh() {
    MeshData mesh;
    mesh.triangleCount = 2;
    mesh.triangles.push_back(array<uint32_t, 3>{0, 1, 2});
    mesh.triangles.push_back(array<uint32_t, 3>{0, 2, 3});
    return mesh;
  }


  MeshData makeTrustedNeighborTetraSelfTestMesh() {
    MeshData mesh;
    mesh.triangleCount = 4;
    mesh.triangles.push_back(array<uint32_t, 3>{0, 1, 2});
    mesh.triangles.push_back(array<uint32_t, 3>{0, 3, 1});
    mesh.triangles.push_back(array<uint32_t, 3>{1, 3, 2});
    mesh.triangles.push_back(array<uint32_t, 3>{2, 3, 0});
    return mesh;
  }


  ContourTriangleProvenance makeSelfTestProvenanceTriangle
    (uint8_t a, uint8_t b, uint8_t c) {
    ContourTriangleProvenance provenance;
    provenance.algorithm = ContourTriangleProvenance::MARCHING_CUBES;
    provenance.cell[0] = 0;
    provenance.cell[1] = 0;
    provenance.cell[2] = 0;
    provenance.vertices[0] = ContourVertexProvenance::gridEdge(a);
    provenance.vertices[1] = ContourVertexProvenance::gridEdge(b);
    provenance.vertices[2] = ContourVertexProvenance::gridEdge(c);
    return provenance;
  }


  void appendSelfTestVertex(vector<float> &vertices, const Vec3 &point) {
    vertices.push_back((float)point.x);
    vertices.push_back((float)point.y);
    vertices.push_back((float)point.z);
  }


  vector<float> makeOpenPatchSelfTestVertices() {
    const Vec3 a{0, 0, 0};
    const Vec3 b{1, 0, 0};
    const Vec3 c{0, 1, 0};
    const Vec3 d{1, 1, 0};
    vector<float> vertices;
    vertices.reserve(18);
    appendSelfTestVertex(vertices, a);
    appendSelfTestVertex(vertices, b);
    appendSelfTestVertex(vertices, c);
    appendSelfTestVertex(vertices, a);
    appendSelfTestVertex(vertices, c);
    appendSelfTestVertex(vertices, d);
    return vertices;
  }


  bool runProvenanceNeighborBuilderContractSelfTest(string &failure) {
    {
      MeshData mesh = makeTrustedNeighborTetraSelfTestMesh();
      vector<ContourTriangleProvenance> provenance{
        makeSelfTestProvenanceTriangle(0, 1, 2),
        makeSelfTestProvenanceTriangle(0, 3, 1),
        makeSelfTestProvenanceTriangle(1, 3, 2),
        makeSelfTestProvenanceTriangle(2, 3, 0),
      };
      vector<float> vertices((size_t)mesh.triangleCount * 9, 0);
      vector<int32_t> neighbors;
      vector<uint8_t> reciprocalSlots;
      bool usedRawProvenance = false;

      if (!requireSelfTest
          (buildContourProvenanceNeighbors
           (provenance, vertices, 0, mesh.triangleCount, neighbors, 1e-4,
            &usedRawProvenance),
           "closed provenance neighbor builder failed", failure))
        return false;
      if (!requireSelfTest(usedRawProvenance,
                           "closed provenance neighbor builder did not use "
                           "raw keys", failure))
        return false;

      TrustedNeighborValidation validation =
        validateTrustedNeighbors(neighbors, mesh.triangleCount,
                                 &reciprocalSlots);
      if (!requireNoTrustedNeighborFlags
          ("closed provenance neighbor builder self-test", validation,
           failure))
        return false;

      TrustedNeighborValidation edgeValidation =
        validateTrustedNeighborEdges(mesh, neighbors, &reciprocalSlots);
      if (!requireNoTrustedNeighborFlags
          ("closed provenance neighbor builder edge self-test",
           edgeValidation, failure))
        return false;
    }

    {
      MeshData mesh = makeTrustedNeighborSelfTestMesh();
      vector<ContourTriangleProvenance> provenance{
        makeSelfTestProvenanceTriangle(0, 1, 2),
        makeSelfTestProvenanceTriangle(0, 2, 3),
      };
      vector<float> vertices = makeOpenPatchSelfTestVertices();
      vector<int32_t> neighbors;
      vector<uint8_t> reciprocalSlots;
      bool usedRawProvenance = true;

      if (!requireSelfTest
          (buildContourProvenanceNeighbors
           (provenance, vertices, 0, mesh.triangleCount, neighbors, 1e-4,
            &usedRawProvenance),
           "open provenance neighbor builder fallback failed", failure))
        return false;
      if (!requireSelfTest(!usedRawProvenance,
                           "open provenance neighbor builder did not fall "
                           "back to welded keys", failure))
        return false;
      if (!requireSelfTest
          (neighbors == vector<int32_t>{-1, -1, 1, 0, -1, -1},
           "open provenance neighbor builder slots changed", failure))
        return false;

      TrustedNeighborValidation validation =
        validateTrustedNeighbors(neighbors, mesh.triangleCount,
                                 &reciprocalSlots);
      if (!requireTrustedNeighborFlags
          ("open provenance neighbor builder self-test", validation,
           true, false, false, false, false, false, false, failure))
        return false;

      TrustedNeighborValidation edgeValidation =
        validateTrustedNeighborEdges(mesh, neighbors, &reciprocalSlots);
      if (!requireNoTrustedNeighborFlags
          ("open provenance neighbor builder edge self-test",
           edgeValidation, failure))
        return false;
    }

    return true;
  }


  bool runTrustedNeighborContractSelfTest(string &failure) {
    MeshData mesh = makeTrustedNeighborSelfTestMesh();
    vector<int32_t> correctNeighbors = {-1, -1, 1, 0, -1, -1};
    vector<int32_t> wrongEdgeNeighbors = {-1, -1, 1, -1, 0, -1};
    vector<uint8_t> correctSlots;
    vector<uint8_t> wrongSlots;

    TrustedNeighborValidation correct =
      validateTrustedNeighbors(correctNeighbors, mesh.triangleCount,
                               &correctSlots);
    if (!requireSelfTest(correct.openSlot,
                         "trusted neighbor self-test did not detect open "
                         "boundary slots", failure))
      return false;
    if (!requireSelfTest(!correct.range && !correct.self &&
                         !correct.duplicate && !correct.asymmetry,
                         "trusted neighbor self-test rejected valid "
                         "reciprocal links", failure))
      return false;
    if (!requireSelfTest(correctSlots.size() == correctNeighbors.size() &&
                         correctSlots[2] == 0 && correctSlots[3] == 2,
                         "trusted neighbor reciprocal slots changed",
                         failure))
      return false;

    TrustedNeighborValidation range =
      validateTrustedNeighbors
      (vector<int32_t>{2, -1, 1, 0, -1, -1}, mesh.triangleCount);
    if (!requireTrustedNeighborFlags("trusted neighbor out-of-range self-test",
                                     range, true, true, false, false, false,
                                     false, false, failure))
      return false;

    TrustedNeighborValidation wrongSize =
      validateTrustedNeighbors
      (vector<int32_t>{-1, -1, 1}, mesh.triangleCount);
    if (!requireTrustedNeighborFlags("trusted neighbor size self-test",
                                     wrongSize, false, true, false, false,
                                     false, false, false, failure))
      return false;

    TrustedNeighborValidation self =
      validateTrustedNeighbors
      (vector<int32_t>{0, -1, 1, 0, -1, -1}, mesh.triangleCount);
    if (!requireTrustedNeighborFlags("trusted neighbor self-link self-test",
                                     self, true, false, true, false, false,
                                     false, false, failure))
      return false;

    TrustedNeighborValidation duplicate =
      validateTrustedNeighbors
      (vector<int32_t>{1, 1, -1, 0, -1, -1}, mesh.triangleCount);
    if (!requireTrustedNeighborFlags("trusted neighbor duplicate self-test",
                                     duplicate, true, false, false, true,
                                     false, false, false, failure))
      return false;

    TrustedNeighborValidation asymmetric =
      validateTrustedNeighbors
      (vector<int32_t>{-1, -1, 1, -1, -1, -1}, mesh.triangleCount);
    if (!requireTrustedNeighborFlags("trusted neighbor asymmetry self-test",
                                     asymmetric, true, false, false, false,
                                     true, false, false, failure))
      return false;

    TrustedNeighborValidation correctEdges =
      validateTrustedNeighborEdges(mesh, correctNeighbors, &correctSlots);
    if (!requireNoTrustedNeighborFlags("trusted neighbor edge self-test",
                                       correctEdges, failure))
      return false;

    TrustedNeighborValidation wrong =
      validateTrustedNeighbors(wrongEdgeNeighbors, mesh.triangleCount,
                               &wrongSlots);
    if (!requireSelfTest(!wrong.asymmetry,
                         "wrong-edge trusted neighbor links stopped being "
                         "reciprocal", failure))
      return false;
    if (!requireSelfTest(wrongSlots.size() == wrongEdgeNeighbors.size() &&
                         wrongSlots[2] == 1 && wrongSlots[4] == 2,
                         "wrong-edge reciprocal slots changed", failure))
      return false;

    TrustedNeighborValidation wrongEdges =
      validateTrustedNeighborEdges(mesh, wrongEdgeNeighbors, &wrongSlots);
    if (!requireSelfTest(wrongEdges.edgeMismatch,
                         "trusted neighbor edge mismatch was not detected",
                         failure))
      return false;
    if (!requireSelfTest(wrongEdges.edgeMismatches == 2,
                         "trusted neighbor edge mismatch count changed",
                         failure))
      return false;

    MeshData sameOrientationMesh;
    sameOrientationMesh.triangleCount = 2;
    sameOrientationMesh.triangles.push_back(array<uint32_t, 3>{0, 1, 2});
    sameOrientationMesh.triangles.push_back(array<uint32_t, 3>{2, 0, 3});
    vector<int32_t> sameOrientationNeighbors = {-1, -1, 1, 0, -1, -1};
    vector<uint8_t> sameOrientationSlots;
    TrustedNeighborValidation sameOrientation =
      validateTrustedNeighbors(sameOrientationNeighbors,
                               sameOrientationMesh.triangleCount,
                               &sameOrientationSlots);
    if (!requireSelfTest(!sameOrientation.asymmetry,
                         "same-orientation trusted neighbor links stopped "
                         "being reciprocal", failure))
      return false;

    TrustedNeighborValidation sameOrientationEdges =
      validateTrustedNeighborEdges(sameOrientationMesh, sameOrientationNeighbors,
                                   &sameOrientationSlots);
    if (!requireSelfTest(sameOrientationEdges.orientation,
                         "trusted neighbor orientation mismatch was not "
                         "detected", failure))
      return false;
    if (!requireSelfTest(sameOrientationEdges.orientationMismatches == 2,
                         "trusted neighbor orientation mismatch count changed",
                         failure))
      return false;

    return true;
  }


  bool runPlanarReductionContractSelfTestImpl(string &failure) {
    const Vec3 normal{0, 0, 1};

    if (!runTrustedNeighborContractSelfTest(failure)) return false;
    if (!runProvenanceNeighborBuilderContractSelfTest(failure)) return false;
    if (!requireSourcePreflightDecision
        ("source vertex mismatch", 17, 9, true, false, failure))
      return false;
    if (!requireSourcePreflightDecision
        ("source normal mismatch", 9, 8, false, true, failure))
      return false;
    if (!requireSourcePreflightDecision
        ("source vertex and normal mismatch", 17, 8, true, true, failure))
      return false;

    {
      vector<Vec3> points;
      vector<BoundaryEdge> edges;
      appendSelfTestSquare(points, edges, 0, 0, 1, false);

      ReplacementCheck replacement =
        checkPhase1Replacement(edges, points, normal);
      if (!requireSelfTest(replacement.checked,
                           "phase-one replacement was not checked", failure))
        return false;
      if (!requireSelfTest(replacement.feasible,
                           "phase-one square replacement was not feasible",
                           failure))
        return false;
      if (!requireSelfTest(replacement.trianglesAfter == 2,
                           "phase-one square replacement triangle count "
                           "changed", failure))
        return false;
    }

    {
      vector<Vec3> points;
      vector<BoundaryEdge> edges;
      appendSelfTestLoop(points, edges,
                         vector<Vec3>{
                           Vec3{0, 0, 0},
                           Vec3{3, 0, 0},
                           Vec3{3, 2, 0},
                           Vec3{2, 2, 0},
                           Vec3{2, 3, 0},
                           Vec3{0, 3, 0},
                         },
                         false);

      ReplacementCheck replacement =
        checkPhase1Replacement(edges, points, normal);
      if (!requireSelfTest(replacement.checked,
                           "concave phase-one replacement was not checked",
                           failure))
        return false;
      if (!requireSelfTest(replacement.feasible,
                           "concave phase-one replacement was not feasible",
                           failure))
        return false;
      if (!requireSelfTest(replacement.trianglesAfter == 4,
                           "concave phase-one replacement triangle count "
                           "changed", failure))
        return false;
      if (!requireSelfTest
          (replacementBoundaryMatchesDirection(edges, replacement.triangles),
           "concave phase-one replacement direction mismatch", failure))
        return false;
    }

    {
      vector<Vec3> points;
      vector<BoundaryEdge> edges;
      appendSelfTestSquare(points, edges, 0, 0, 1, true);

      ReplacementCheck replacement =
        checkPhase1Replacement(edges, points, normal);
      if (!requireSelfTest(replacement.feasible,
                           "clockwise phase-one replacement was not feasible",
                           failure))
        return false;
      if (!requireSelfTest
          (replacementBoundaryMatchesDirection(edges, replacement.triangles),
           "clockwise phase-one replacement direction was not repaired",
           failure))
        return false;

      vector<array<uint32_t, 3> > flipped = replacement.triangles;
      flipReplacementTriangles(flipped);
      if (!requireSelfTest
          (!replacementBoundaryMatchesDirection(edges, flipped),
           "directed boundary check accepted flipped replacement", failure))
        return false;

      vector<array<uint32_t, 3> > wrongInternal{
        array<uint32_t, 3>{0, 1, 2},
        array<uint32_t, 3>{0, 3, 2},
      };
      if (!requireSelfTest(replacementBoundaryMatches(edges, wrongInternal),
                           "internal misorientation setup lost boundary match",
                           failure))
        return false;
      if (!requireSelfTest
          (!replacementEdgeIncidenceOk(edges, wrongInternal),
           "local edge-incidence check accepted internal misorientation",
           failure))
        return false;
      if (!requireSelfTest
          (replacementEdgeIncidenceOk(edges, replacement.triangles),
           "local edge-incidence check rejected valid replacement", failure))
        return false;
    }

    {
      const Vec3 xNormal{1, 0, 0};
      vector<Vec3> points;
      vector<BoundaryEdge> edges;
      appendSelfTestLoop(points, edges,
                         vector<Vec3>{
                           Vec3{0, 0, 0},
                           Vec3{0, 2, 0},
                           Vec3{0, 2, 2},
                           Vec3{0, 0, 2},
                         },
                         false);

      ReplacementCheck replacement =
        checkPhase1Replacement(edges, points, xNormal);
      if (!requireSelfTest(replacement.feasible,
                           "vertical x-plane phase-one replacement was not "
                           "feasible", failure))
        return false;
      if (!requireSelfTest(replacement.trianglesAfter == 2,
                           "vertical x-plane replacement triangle count "
                           "changed", failure))
        return false;
      if (!requireSelfTest
          (replacementBoundaryMatchesDirection(edges, replacement.triangles),
           "vertical x-plane replacement direction mismatch", failure))
        return false;
    }

    {
      const Vec3 yNormal{0, 1, 0};
      vector<Vec3> points;
      vector<BoundaryEdge> edges;
      appendSelfTestLoop(points, edges,
                         vector<Vec3>{
                           Vec3{0, 0, 0},
                           Vec3{0, 0, 2},
                           Vec3{2, 0, 2},
                           Vec3{2, 0, 0},
                         },
                         false);

      ReplacementCheck replacement =
        checkPhase1Replacement(edges, points, yNormal);
      if (!requireSelfTest(replacement.feasible,
                           "vertical y-plane phase-one replacement was not "
                           "feasible", failure))
        return false;
      if (!requireSelfTest(replacement.trianglesAfter == 2,
                           "vertical y-plane replacement triangle count "
                           "changed", failure))
        return false;
      if (!requireSelfTest
          (replacementBoundaryMatchesDirection(edges, replacement.triangles),
           "vertical y-plane replacement direction mismatch", failure))
        return false;
    }

    {
      const Vec3 origin{0, 0, 0};
      const Vec3 uAxis{2, 0, 1};
      const Vec3 vAxis{0, 2, 0.5};
      const Vec3 slopedNormal = normalize(cross(uAxis, vAxis));
      vector<Vec3> points;
      vector<BoundaryEdge> edges;
      appendSelfTestPlaneLoop
        (points, edges, origin, uAxis, vAxis,
         vector<pair<double, double> >{
           pair<double, double>(0, 0),
           pair<double, double>(1, 0),
           pair<double, double>(1, 1),
           pair<double, double>(0, 1),
         },
         false);

      ReplacementCheck replacement =
        checkPhase1Replacement(edges, points, slopedNormal);
      if (!requireSelfTest(replacement.feasible,
                           "sloped phase-one replacement was not feasible",
                           failure))
        return false;
      if (!requireSelfTest(replacement.trianglesAfter == 2,
                           "sloped phase-one replacement triangle count "
                           "changed", failure))
        return false;
      if (!requireSelfTest
          (replacementBoundaryMatchesDirection(edges, replacement.triangles),
           "sloped phase-one replacement direction mismatch", failure))
        return false;
    }

    {
      const Vec3 origin{0, 0, 0};
      const Vec3 uAxis{2, 0, 1};
      const Vec3 vAxis{0, 2, 0.5};
      const Vec3 slopedNormal = normalize(cross(uAxis, vAxis));
      vector<Vec3> points;
      vector<BoundaryEdge> edges;
      appendSelfTestPlaneLoop
        (points, edges, origin, uAxis, vAxis,
         vector<pair<double, double> >{
           pair<double, double>(0, 0),
           pair<double, double>(2, 0),
           pair<double, double>(2, 2),
           pair<double, double>(0, 2),
         },
         false);
      appendSelfTestPlaneLoop
        (points, edges, origin, uAxis, vAxis,
         vector<pair<double, double> >{
           pair<double, double>(0.5, 0.5),
           pair<double, double>(1, 0.5),
           pair<double, double>(1, 1),
           pair<double, double>(0.5, 1),
         },
         true);

      ReplacementCheck replacement =
        checkHoleAwareReplacement(edges, points, slopedNormal);
      if (!requireSelfTest(replacement.feasible,
                           "sloped hole-aware replacement was not feasible",
                           failure))
        return false;
      if (!requireSelfTest(replacement.trianglesAfter == 8,
                           "sloped hole-aware replacement triangle count "
                           "changed", failure))
        return false;
      if (!requireSelfTest
          (replacementBoundaryMatchesDirection(edges, replacement.triangles),
           "sloped hole-aware direction mismatch", failure))
        return false;
    }

    {
      vector<Vec3> points;
      vector<BoundaryEdge> edges;
      appendSelfTestSquare(points, edges, 0, 0, 4, false);
      appendSelfTestSquare(points, edges, 1, 1, 1, true);

      ReplacementCheck replacement =
        checkHoleAwareReplacement(edges, points, normal);
      if (!requireSelfTest(replacement.checked,
                           "hole-aware replacement was not checked", failure))
        return false;
      if (!requireSelfTest(replacement.feasible,
                           "hole-aware square-ring replacement was not "
                           "feasible", failure))
        return false;
      if (!requireSelfTest(replacement.trianglesAfter == 8,
                           "hole-aware square-ring triangle count changed",
                           failure))
        return false;
    }

    {
      vector<Vec3> points;
      vector<BoundaryEdge> edges;
      appendSelfTestSquare(points, edges, 0, 0, 4, false);
      appendSelfTestSquare(points, edges, 1, 1, 1, false);

      ReplacementCheck replacement =
        checkHoleAwareReplacement(edges, points, normal);
      if (!requireSelfTest(replacement.checked,
                           "same-direction hole-aware replacement was not "
                           "checked", failure))
        return false;
      if (!requireSelfTest(replacement.estimateAvailable,
                           "same-direction hole-aware replacement lost its "
                           "estimate", failure))
        return false;
      if (!requireSelfTest(!replacement.feasible &&
                           replacement.triangles.empty(),
                           "same-direction hole-aware replacement "
                           "unexpectedly became writable", failure))
        return false;
      if (!requireSelfTest(replacement.trianglesAfter == 8,
                           "same-direction hole-aware estimate changed",
                           failure))
        return false;
    }

    {
      const Vec3 xNormal{1, 0, 0};
      vector<Vec3> points;
      vector<BoundaryEdge> edges;
      appendSelfTestLoop(points, edges,
                         vector<Vec3>{
                           Vec3{0, 0, 0},
                           Vec3{0, 4, 0},
                           Vec3{0, 4, 4},
                           Vec3{0, 0, 4},
                         },
                         false);
      appendSelfTestLoop(points, edges,
                         vector<Vec3>{
                           Vec3{0, 1, 1},
                           Vec3{0, 2, 1},
                           Vec3{0, 2, 2},
                           Vec3{0, 1, 2},
                         },
                         true);

      ReplacementCheck replacement =
        checkHoleAwareReplacement(edges, points, xNormal);
      if (!requireSelfTest(replacement.feasible,
                           "vertical x-plane hole-aware replacement was not "
                           "feasible", failure))
        return false;
      if (!requireSelfTest(replacement.trianglesAfter == 8,
                           "vertical x-plane hole-aware triangle count "
                           "changed", failure))
        return false;
      if (!requireSelfTest
          (replacementBoundaryMatchesDirection(edges, replacement.triangles),
           "vertical x-plane hole-aware direction mismatch", failure))
        return false;
    }

    {
      vector<Vec3> points;
      vector<BoundaryEdge> edges;
      appendSelfTestSquare(points, edges, 0, 0, 6, false);
      appendSelfTestSquare(points, edges, 1, 1, 1, true);
      appendSelfTestSquare(points, edges, 4, 3.5, 1, true);

      ReplacementCheck replacement =
        checkHoleAwareReplacement(edges, points, normal);
      if (!requireSelfTest(replacement.checked,
                           "two-hole replacement was not checked", failure))
        return false;
      if (!requireSelfTest(replacement.feasible,
                           "two-hole replacement was not feasible", failure))
        return false;
      if (!requireSelfTest(replacement.trianglesAfter == 14,
                           "two-hole replacement triangle count changed",
                           failure))
        return false;
      if (!requireSelfTest
          (replacementBoundaryMatchesDirection(edges, replacement.triangles),
           "two-hole replacement direction mismatch", failure))
        return false;
    }

    {
      vector<Vec3> points;
      vector<BoundaryEdge> edges;
      appendSelfTestSquare(points, edges, 0, 0, 4, false);
      appendSelfTestSquare(points, edges, 3, 1, 1, true);

      ReplacementCheck replacement =
        checkHoleAwareReplacement(edges, points, normal);
      if (!requireSelfTest(replacement.checked,
                           "outer-touching hole replacement was not checked",
                           failure))
        return false;
      if (!requireSelfTest(!replacement.feasible &&
                           !replacement.estimateAvailable,
                           "outer-touching hole replacement unexpectedly "
                           "became eligible", failure))
        return false;
    }

    {
      vector<Vec3> points;
      vector<BoundaryEdge> edges;
      appendSelfTestSquare(points, edges, 0, 0, 5, false);
      appendSelfTestSquare(points, edges, 1, 1, 1, true);
      appendSelfTestSquare(points, edges, 2, 1, 1, true);

      ReplacementCheck replacement =
        checkHoleAwareReplacement(edges, points, normal);
      if (!requireSelfTest(replacement.checked,
                           "touching holes replacement was not checked",
                           failure))
        return false;
      if (!requireSelfTest(!replacement.feasible &&
                           !replacement.estimateAvailable,
                           "touching holes replacement unexpectedly became "
                           "eligible", failure))
        return false;
    }

    {
      vector<Vec3> points;
      vector<BoundaryEdge> edges;
      appendSelfTestSquare(points, edges, 0, 0, 4, false);
      appendSelfTestSquare(points, edges, 3.2, 1.5, 1, true);

      ReplacementCheck replacement =
        checkHoleAwareReplacement(edges, points, normal);
      if (!requireSelfTest(replacement.checked,
                           "straddling hole replacement was not checked",
                           failure))
        return false;
      if (!requireSelfTest(!replacement.feasible &&
                           !replacement.estimateAvailable,
                           "straddling hole replacement unexpectedly became "
                           "eligible", failure))
        return false;
    }

    {
      vector<Vec3> points;
      vector<BoundaryEdge> edges;
      appendSelfTestSquare(points, edges, 0, 0, 4, false);
      appendSelfTestSquare(points, edges, 1, 1, 2, true);
      appendSelfTestSquare(points, edges, 1.5, 1.5, 0.5, true);

      ReplacementCheck replacement =
        checkHoleAwareReplacement(edges, points, normal);
      if (!requireSelfTest(replacement.checked,
                           "nested hole replacement was not checked",
                           failure))
        return false;
      if (!requireSelfTest(!replacement.feasible &&
                           !replacement.estimateAvailable,
                           "nested hole replacement unexpectedly became "
                           "eligible", failure))
        return false;
    }

    {
      vector<Vec3> points;
      vector<BoundaryEdge> edges;
      appendSelfTestSquare(points, edges, 0, 0, 5, false);
      appendSelfTestSquare(points, edges, 1, 1, 1.5, true);
      appendSelfTestSquare(points, edges, 2, 1.5, 1.5, true);

      ReplacementCheck replacement =
        checkHoleAwareReplacement(edges, points, normal);
      if (!requireSelfTest(replacement.checked,
                           "intersecting hole replacement was not checked",
                           failure))
        return false;
      if (!requireSelfTest(!replacement.feasible &&
                           !replacement.estimateAvailable,
                           "intersecting hole replacement unexpectedly became "
                           "eligible", failure))
        return false;
    }

    if (!requireValidationDecision
        ("open preserved", 1, 0, 0, 0, selfTestIncidence(1, 0, 0, 0),
         false, false, false, false, false, false, false, failure))
      return false;
    if (!requireValidationDecision
        ("open improved", 2, 0, 0, 0, selfTestIncidence(1, 0, 0, 0),
         false, false, false, false, false, false, false, failure))
      return false;
    if (!requireValidationDecision
        ("watertight to open", 0, 0, 0, 0, selfTestIncidence(1, 0, 0, 0),
         false, false, false, true, false, false, true, failure))
      return false;
    if (!requireValidationDecision
        ("open to more open", 1, 0, 0, 0, selfTestIncidence(2, 0, 0, 0),
         false, false, false, true, false, false, true, failure))
      return false;
    if (!requireValidationDecision
        ("nonmanifold worse", 0, 0, 0, 0, selfTestIncidence(0, 1, 0, 0),
         false, false, false, true, false, false, true, failure))
      return false;
    if (!requireValidationDecision
        ("degenerate preserved", 0, 0, 0, 1, selfTestIncidence(0, 0, 0, 1),
         false, false, false, false, false, false, false, failure))
      return false;
    if (!requireValidationDecision
        ("degenerate improved", 0, 0, 0, 2, selfTestIncidence(0, 0, 0, 1),
         false, false, false, false, false, false, false, failure))
      return false;
    if (!requireValidationDecision
        ("degenerate worse", 0, 0, 0, 0, selfTestIncidence(0, 0, 0, 1),
         false, false, false, false, true, false, true, failure))
      return false;
    if (!requireValidationDecision
        ("orientation worse", 0, 0, 0, 0, selfTestIncidence(0, 0, 1, 0),
         false, false, false, false, false, true, true, failure))
      return false;
    if (!requireValidationDecision
        ("vertex count mismatch", 0, 0, 0, 0, selfTestIncidence(0, 0, 0, 0),
         true, false, false, false, false, false, true, failure))
      return false;
    if (!requireValidationDecision
        ("normal count mismatch", 0, 0, 0, 0, selfTestIncidence(0, 0, 0, 0),
         false, true, false, false, false, false, true, failure))
      return false;
    if (!requireValidationDecision
        ("triangle mismatch", 0, 0, 0, 0, selfTestIncidence(0, 0, 0, 0),
         false, false, true, false, false, false, true, failure))
      return false;

    PlanarReductionConfig config;
    {
      vector<float> vertices = {
        numeric_limits<float>::quiet_NaN(), 0, 0,
        1, 0, 0,
        0, 1, 0,
      };
      vector<float> normals = {
        0, 0, 1, 0, 0, 1, 0, 0, 1,
      };
      TriangleSurface surface;
      surface.replace(vertices, normals);
      PlanarReductionReport report = analyzeOrReduceImpl(surface, config, 0);
      if (!requireSelfTest(report.sourceInvalidCoordinates == 1 &&
                           report.outputTriangles == 1,
                           "non-finite source was not rejected", failure))
        return false;

      ReductionEligibility eligibility;
      eligibility.quantizationTolerance = config.coordTolerance;
      eligibility.triangleOrigins.assign
        (1, (uint8_t)REDUCTION_MC_REDUCIBLE);
      eligibility.seal(vertices);
      if (!requireSelfTest(!eligibility.validFor(vertices),
                           "non-finite eligibility was accepted", failure))
        return false;
    }

    {
      vector<float> vertices = {
        1e30f, 0, 0,
        1, 0, 0,
        0, 1, 0,
      };
      vector<float> normals = {
        0, 0, 1, 0, 0, 1, 0, 0, 1,
      };
      TriangleSurface surface;
      surface.replace(vertices, normals);
      PlanarReductionReport report = analyzeOrReduceImpl(surface, config, 0);
      if (!requireSelfTest(report.sourceRangeMismatch &&
                           report.outputTriangles == 1,
                           "out-of-range source was not rejected", failure))
        return false;
    }

    {
      TriangleSurface surface;
      cb::Vector3F triangle[3] = {
        cb::Vector3F(0, 0, 0), cb::Vector3F(1, 0, 0),
        cb::Vector3F(0, 1, 0),
      };
      surface.add(triangle);
      PlanarReductionConfig invalidConfig = config;
      invalidConfig.coordTolerance = numeric_limits<double>::infinity();
      PlanarReductionReport report =
        analyzeOrReduceImpl(surface, invalidConfig, 0);
      if (!requireSelfTest(!report.triangles && !report.components,
                           "non-finite reducer config was accepted", failure))
        return false;
    }

    failure.clear();
    return true;
  }


}


PlanarReductionReport CAMotics::PlanarReductionInternal::analyzeOrReduceCore(
  const Surface &surface, const PlanarReductionConfig &config,
  TriangleSurface *target) {
  return analyzeOrReduceImpl(surface, config, target);
}


void CAMotics::PlanarReductionInternal::resetAppliedReductionReport(
  PlanarReductionReport &report) {
  resetAppliedReduction(report);
}


void CAMotics::PlanarReductionInternal::finalizeSideReductionReports(
  PlanarReductionReport &report) {
  finalizeSideReports(report);
}


bool CAMotics::PlanarReductionInternal::runPlanarReductionContractSelfTestCore(
  string &failure) {
  return runPlanarReductionContractSelfTestImpl(failure);
}
