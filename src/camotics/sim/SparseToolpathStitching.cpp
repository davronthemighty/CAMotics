/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#include "SparseToolpathInternal.h"

#include "CutSim.h"
#include "SparseToolpathArtifacts.h"
#include "ToolSweep.h"

#include <camotics/GeometrySafetyInternal.h>
#include <camotics/Grid.h>
#include <camotics/Profile.h>
#include <camotics/Task.h>
#include <camotics/contour/CompositeSurface.h>
#include <camotics/contour/GridTree.h>
#include <camotics/contour/TriangleSurface.h>
#include <camotics/render/Renderer.h>
#include <camotics/sim/CutWorkpiece.h>

#include <gcode/ToolPath.h>

#include <cbang/Exception.h>
#include <cbang/json/Sink.h>
#include <cbang/os/SystemInfo.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>


using namespace std;
using namespace cb;
using namespace CAMotics;
using namespace CAMotics::SparseToolpath;
using namespace CAMotics::SparseToolpath::Internal;


namespace {
  const unsigned MAX_SPARSE_AXIS_SEGMENTS = 16 * 1024 * 1024;
  struct BoundaryGridPoint {
    unsigned x = 0;
    unsigned y = 0;

    BoundaryGridPoint() {}
    BoundaryGridPoint(unsigned x, unsigned y) : x(x), y(y) {}

    bool operator==(const BoundaryGridPoint &o) const {
      return x == o.x && y == o.y;
    }

    bool operator<(const BoundaryGridPoint &o) const {
      if (x != o.x) return x < o.x;
      return y < o.y;
    }
  };


  struct PendingBoundaryEdge {
    BoundaryGridPoint from;
    BoundaryGridPoint to;
    OwnershipBoundaryEdge info;
    bool used = false;
  };


  struct StitchVertexKey {
    int64_t x = 0;
    int64_t y = 0;
    int64_t z = 0;

    StitchVertexKey() {}
    StitchVertexKey(int64_t x, int64_t y, int64_t z) : x(x), y(y), z(z) {}

    bool operator==(const StitchVertexKey &o) const {
      return x == o.x && y == o.y && z == o.z;
    }

    bool operator<(const StitchVertexKey &o) const {
      if (x != o.x) return x < o.x;
      if (y != o.y) return y < o.y;
      return z < o.z;
    }
  };


  struct StitchDirectedEdge {
    StitchVertexKey from;
    StitchVertexKey to;
    Vector3F fromPoint;
    Vector3F toPoint;
    bool used = false;
  };


  struct StitchEdgeUse {
    uint64_t count = 0;
    StitchDirectedEdge edge;
  };


  struct PreparedClosurePatch {
    vector<Vector3F> boundary;
    vector<array<unsigned, 3> > triangles;
  };


  struct PatchAreaStats {
    uint64_t checks = 0;
    uint64_t failures = 0;
    double expectedArea = 0;
    double triangleArea = 0;
    double maxError = 0;

    bool accepted() const {return failures == 0;}

    void add(const PatchAreaStats &other) {
      checks += other.checks;
      failures += other.failures;
      expectedArea += other.expectedArea;
      triangleArea += other.triangleArea;
      maxError = max(maxError, other.maxError);
    }
  };


  struct PatchVertexKey {
    int64_t x = 0;
    int64_t y = 0;
    int64_t z = 0;

    bool operator<(const PatchVertexKey &o) const {
      if (x != o.x) return x < o.x;
      if (y != o.y) return y < o.y;
      return z < o.z;
    }
  };


  typedef pair<PatchVertexKey, PatchVertexKey> PatchEdgeKey;


  struct PatchBoundaryStats {
    uint64_t checks = 0;
    uint64_t failures = 0;
    uint64_t expectedEdges = 0;
    uint64_t emittedEdges = 0;
    uint64_t mismatchedEdges = 0;
    uint64_t invalidIncidenceEdges = 0;

    bool accepted() const {return failures == 0;}

    void add(const PatchBoundaryStats &other) {
      checks += other.checks;
      failures += other.failures;
      expectedEdges += other.expectedEdges;
      emittedEdges += other.emittedEdges;
      mismatchedEdges += other.mismatchedEdges;
      invalidIncidenceEdges += other.invalidIncidenceEdges;
    }
  };


  struct StitchBoundaryLoop {
    vector<StitchVertexKey> vertices;
    vector<Vector3F> points;
  };


  bool sameBoundaryEdgeInfo(const OwnershipBoundaryEdge &a,
                            const OwnershipBoundaryEdge &b) {
    return a.analyticRegionID == b.analyticRegionID &&
      a.adjacentOwnership == b.adjacentOwnership &&
      a.adjacentRegionID == b.adjacentRegionID &&
      a.side == b.side;
  }


  bool collinearGridPoints(const BoundaryGridPoint &a,
                           const BoundaryGridPoint &b,
                           const BoundaryGridPoint &c) {
    int64_t abX = (int64_t)b.x - a.x;
    int64_t abY = (int64_t)b.y - a.y;
    int64_t bcX = (int64_t)c.x - b.x;
    int64_t bcY = (int64_t)c.y - b.y;
    return abX * bcY == abY * bcX;
  }


  void simplifyClosedGridLoop(vector<BoundaryGridPoint> &vertices,
                              vector<OwnershipBoundaryEdge> &edges) {
    bool changed = true;
    while (changed && 3 < vertices.size()) {
      changed = false;
      size_t count = vertices.size();

      for (size_t i = 0; i < count; i++) {
        size_t prev = i ? i - 1 : count - 1;
        size_t next = (i + 1) % count;
        if (!sameBoundaryEdgeInfo(edges[prev], edges[i])) continue;
        if (!collinearGridPoints(vertices[prev], vertices[i],
                                 vertices[next]))
          continue;

        vertices.erase(vertices.begin() + i);
        edges.erase(edges.begin() + i);
        changed = true;
        break;
      }
    }
  }


  double polygonArea2D(const vector<OwnershipBoundaryPoint> &vertices) {
    double area = 0;
    for (size_t i = 0; i < vertices.size(); i++) {
      const OwnershipBoundaryPoint &a = vertices[i];
      const OwnershipBoundaryPoint &b = vertices[(i + 1) % vertices.size()];
      area += a.x * b.y - b.x * a.y;
    }

    return area * 0.5;
  }


  bool samePoint(const Vector3F &a, const Vector3F &b);
  bool sameCoord(double a, double b);
  double snapStockGridCoord(double value, double min, double max,
                            double resolution);
  unsigned segmentCount(double length, double resolution);
  void appendEdgePoints(vector<Vector3F> &points, const Vector3F &start,
                        const Vector3F &end, unsigned segments);
  vector<double> makeStockBreakpoints(double min, double max,
                                      double resolution, unsigned bins);
  void appendBreakpointEdgePoints(vector<Vector3F> &points,
                                  const Vector3F &start,
                                  const Vector3F &end,
                                  const vector<double> &breakpoints,
                                  bool xAxis);
  SmartPointer<TriangleSurface> createPlanarBoundaryClosures
  (const Surface &surface, const RegionPlan &regionPlan, double tolerance,
   double resolution, bool includeStockBorders, uint64_t &closurePatches,
   uint64_t &closureTriangles, uint64_t &unsupportedLoops,
   PatchAreaStats &areaStats, PatchBoundaryStats &boundaryStats);


  double cross2D(const Vector3F &a, const Vector3F &b,
                 const Vector3F &c) {
    double abX = b.x() - a.x();
    double abY = b.y() - a.y();
    double acX = c.x() - a.x();
    double acY = c.y() - a.y();
    return abX * acY - abY * acX;
  }


  double polygonArea2D(const vector<Vector3F> &vertices) {
    double area = 0;
    for (size_t i = 0; i < vertices.size(); i++) {
      const Vector3F &a = vertices[i];
      const Vector3F &b = vertices[(i + 1) % vertices.size()];
      area += a.x() * b.y() - b.x() * a.y();
    }

    return area * 0.5;
  }


  bool pointInTriangle2D(const Vector3F &p, const Vector3F &a,
                         const Vector3F &b, const Vector3F &c,
                         double eps) {
    return -eps <= cross2D(a, b, p) && -eps <= cross2D(b, c, p) &&
      -eps <= cross2D(c, a, p);
  }


  bool samePoint2D(const Vector3F &a, const Vector3F &b, double eps) {
    return fabs(a.x() - b.x()) <= eps && fabs(a.y() - b.y()) <= eps;
  }


  bool pointOnSegment2D(const Vector3F &p, const Vector3F &a,
                        const Vector3F &b, double eps) {
    if (eps < fabs(cross2D(a, b, p))) return false;
    return min(a.x(), b.x()) - eps <= p.x() &&
      p.x() <= max(a.x(), b.x()) + eps &&
      min(a.y(), b.y()) - eps <= p.y() &&
      p.y() <= max(a.y(), b.y()) + eps;
  }


  int orientation2D(const Vector3F &a, const Vector3F &b,
                    const Vector3F &c, double eps) {
    double value = cross2D(a, b, c);
    if (eps < value) return 1;
    if (value < -eps) return -1;
    return 0;
  }


  bool segmentsIntersect2D(const Vector3F &a, const Vector3F &b,
                           const Vector3F &c, const Vector3F &d,
                           double eps) {
    int o1 = orientation2D(a, b, c, eps);
    int o2 = orientation2D(a, b, d, eps);
    int o3 = orientation2D(c, d, a, eps);
    int o4 = orientation2D(c, d, b, eps);

    if (o1 != o2 && o3 != o4) return true;
    if (!o1 && pointOnSegment2D(c, a, b, eps)) return true;
    if (!o2 && pointOnSegment2D(d, a, b, eps)) return true;
    if (!o3 && pointOnSegment2D(a, c, d, eps)) return true;
    if (!o4 && pointOnSegment2D(b, c, d, eps)) return true;
    return false;
  }


  bool bridgeCrossesLoop(const Vector3F &a, const Vector3F &b,
                         const vector<Vector3F> &loop, double eps) {
    for (size_t i = 0; i < loop.size(); i++) {
      const Vector3F &c = loop[i];
      const Vector3F &d = loop[(i + 1) % loop.size()];
      if (samePoint2D(a, c, eps) || samePoint2D(a, d, eps) ||
          samePoint2D(b, c, eps) || samePoint2D(b, d, eps))
        continue;
      if (segmentsIntersect2D(a, b, c, d, eps)) return true;
    }

    return false;
  }


  bool pointInLoop2D(const Vector3F &point,
                     const vector<Vector3F> &loop, double eps) {
    bool inside = false;
    for (size_t i = 0, j = loop.size() - 1; i < loop.size(); j = i++) {
      const Vector3F &a = loop[i];
      const Vector3F &b = loop[j];

      if (pointOnSegment2D(point, a, b, eps)) return false;

      bool crosses = ((a.y() > point.y()) != (b.y() > point.y()));
      if (crosses) {
        double x =
          (b.x() - a.x()) * (point.y() - a.y()) / (b.y() - a.y()) +
          a.x();
        if (point.x() < x) inside = !inside;
      }
    }

    return inside;
  }


  bool loopInsideLoop2D(const vector<Vector3F> &inner,
                        const vector<Vector3F> &outer, double eps) {
    for (const auto &point: inner)
      if (!pointInLoop2D(point, outer, eps)) return false;
    return true;
  }


  bool triangulateTopLoop(vector<Vector3F> &boundary,
                          vector<array<unsigned, 3> > &triangles) {
    triangles.clear();

    if (1 < boundary.size() && samePoint(boundary.front(), boundary.back()))
      boundary.pop_back();
    if (boundary.size() < 3) return false;

    double area = polygonArea2D(boundary);
    if (fabs(area) < 1e-18) return false;
    if (area < 0) reverse(boundary.begin(), boundary.end());

    double xMin = boundary[0].x();
    double xMax = boundary[0].x();
    double yMin = boundary[0].y();
    double yMax = boundary[0].y();
    for (const auto &p: boundary) {
      xMin = min<double>(xMin, p.x());
      xMax = max<double>(xMax, p.x());
      yMin = min<double>(yMin, p.y());
      yMax = max<double>(yMax, p.y());
    }
    double scale = max(xMax - xMin, yMax - yMin);
    double eps = max(1e-12, scale * scale * 1e-12);

    vector<unsigned> indices(boundary.size());
    for (unsigned i = 0; i < indices.size(); i++) indices[i] = i;

    size_t guard = 0;
    size_t maxGuard = boundary.size() * boundary.size();
    while (3 < indices.size() && guard++ < maxGuard) {
      bool clipped = false;
      size_t count = indices.size();

      for (size_t i = 0; i < count; i++) {
        unsigned prev = indices[i ? i - 1 : count - 1];
        unsigned current = indices[i];
        unsigned next = indices[(i + 1) % count];

        if (cross2D(boundary[prev], boundary[current],
                    boundary[next]) <= eps)
          continue;

        bool contains = false;
        for (unsigned candidate: indices) {
          if (candidate == prev || candidate == current ||
              candidate == next)
            continue;
          if (samePoint2D(boundary[candidate], boundary[prev], eps) ||
              samePoint2D(boundary[candidate], boundary[current], eps) ||
              samePoint2D(boundary[candidate], boundary[next], eps))
            continue;

          if (pointInTriangle2D(boundary[candidate], boundary[prev],
                                boundary[current], boundary[next], eps)) {
            contains = true;
            break;
          }
        }
        if (contains) continue;

        triangles.push_back({{prev, current, next}});
        indices.erase(indices.begin() + i);
        clipped = true;
        break;
      }

      if (!clipped) return false;
    }

    if (indices.size() != 3) return false;
    if (cross2D(boundary[indices[0]], boundary[indices[1]],
                boundary[indices[2]]) <= eps)
      return false;

    triangles.push_back({{indices[0], indices[1], indices[2]}});
    return true;
  }


  double triangleArea2D(const Vector3F &a, const Vector3F &b,
                        const Vector3F &c) {
    return fabs(cross2D(a, b, c)) * 0.5;
  }


  double triangleArea2D
  (const vector<Vector3F> &boundary,
   const vector<array<unsigned, 3> > &triangles) {
    double area = 0;
    for (const auto &tri: triangles)
      area += triangleArea2D
        (boundary[tri[0]], boundary[tri[1]], boundary[tri[2]]);

    return area;
  }


  double patchAreaTolerance(double expectedArea, double triangleArea,
                            double resolution) {
    double scale = max(max(expectedArea, triangleArea),
                       resolution * resolution);
    return max(1e-9, scale * 1e-9);
  }


  bool recordPatchAreaStats
  (PatchAreaStats &stats, double expected, double actual,
   double resolution) {
    double error = fabs(expected - actual);
    double tolerance = patchAreaTolerance(expected, actual, resolution);

    stats.checks++;
    stats.expectedArea += expected;
    stats.triangleArea += actual;
    stats.maxError = max(stats.maxError, error);
    if (tolerance < error) stats.failures++;

    return error <= tolerance;
  }


  bool recordPatchAreaStats
  (PatchAreaStats &stats, const vector<Vector3F> &boundary,
   const vector<array<unsigned, 3> > &triangles, double resolution) {
    return recordPatchAreaStats(stats, fabs(polygonArea2D(boundary)),
                                triangleArea2D(boundary, triangles),
                                resolution);
  }


  PatchVertexKey makePatchVertexKey(const Vector3F &point,
                                    double tolerance) {
    PatchVertexKey key;
    if (!CAMotics::Internal::quantizeCoordinate
        (point.x(), tolerance, key.x) ||
        !CAMotics::Internal::quantizeCoordinate
        (point.y(), tolerance, key.y) ||
        !CAMotics::Internal::quantizeCoordinate
        (point.z(), tolerance, key.z))
      THROW("Sparse patch vertex is outside the quantization range.");
    return key;
  }


  PatchEdgeKey makePatchEdgeKey(const PatchVertexKey &a,
                                const PatchVertexKey &b) {
    return b < a ? make_pair(b, a) : make_pair(a, b);
  }


  bool recordPatchBoundaryStats
  (PatchBoundaryStats &stats, const vector<Vector3F> &boundary,
   const vector<array<unsigned, 3> > &triangles, double resolution) {
    double tolerance = max(1e-9, resolution * 1e-6);
    vector<PatchVertexKey> keys;
    keys.reserve(boundary.size());
    for (const auto &point: boundary)
      keys.push_back(makePatchVertexKey(point, tolerance));

    map<PatchEdgeKey, uint64_t> expectedUses;
    for (size_t i = 0; i < keys.size(); i++)
      expectedUses[makePatchEdgeKey(keys[i], keys[(i + 1) % keys.size()])]++;

    map<PatchEdgeKey, uint64_t> emittedUses;
    for (const auto &triangle: triangles) {
      const PatchVertexKey &a = keys[triangle[0]];
      const PatchVertexKey &b = keys[triangle[1]];
      const PatchVertexKey &c = keys[triangle[2]];
      emittedUses[makePatchEdgeKey(a, b)]++;
      emittedUses[makePatchEdgeKey(b, c)]++;
      emittedUses[makePatchEdgeKey(c, a)]++;
    }

    set<PatchEdgeKey> expectedBoundary;
    set<PatchEdgeKey> emittedBoundary;
    uint64_t invalid = 0;
    for (const auto &entry: expectedUses) {
      if (entry.second == 1) expectedBoundary.insert(entry.first);
      else if (entry.second != 2) invalid++;
    }
    for (const auto &entry: emittedUses) {
      if (entry.second == 1) emittedBoundary.insert(entry.first);
      else if (entry.second != 2) invalid++;
    }

    uint64_t mismatched = 0;
    for (const auto &edge: expectedBoundary)
      if (!emittedBoundary.count(edge)) mismatched++;
    for (const auto &edge: emittedBoundary)
      if (!expectedBoundary.count(edge)) mismatched++;

    stats.checks++;
    stats.expectedEdges += expectedBoundary.size();
    stats.emittedEdges += emittedBoundary.size();
    stats.mismatchedEdges += mismatched;
    stats.invalidIncidenceEdges += invalid;
    if (mismatched || invalid) stats.failures++;
    return !mismatched && !invalid;
  }


  uint64_t scaledAreaMetric(double area) {
    if (!isfinite(area) || area <= 0) return 0;
    double scaled = area * 1000000.0;
    double limit = nextafter
      ((double)numeric_limits<int64_t>::max(), 0.0);
    if (!isfinite(scaled) || limit < scaled)
      return (uint64_t)numeric_limits<int64_t>::max();
    return (uint64_t)llround(scaled);
  }


  vector<Vector3F> buildSegmentedTopBoundary
  (const OwnershipBoundaryLoop &loop, const Rectangle3D &stockBounds,
   double resolution, unsigned bins, bool snapToRenderGrid) {
    vector<Vector3F> boundary;
    if (loop.vertices.size() < 3) return boundary;

    Vector3D stockMin = stockBounds.getMin();
    Vector3D stockMax = stockBounds.getMax();
    vector<double> xBreaks =
      makeStockBreakpoints(stockMin.x(), stockMax.x(), resolution, bins);
    vector<double> yBreaks =
      makeStockBreakpoints(stockMin.y(), stockMax.y(), resolution, bins);

    for (size_t i = 0; i < loop.vertices.size(); i++) {
      const OwnershipBoundaryPoint &a = loop.vertices[i];
      const OwnershipBoundaryPoint &b =
        loop.vertices[(i + 1) % loop.vertices.size()];
      double ax = snapToRenderGrid ?
        snapStockGridCoord(a.x, stockMin.x(), stockMax.x(), resolution) :
        a.x;
      double ay = snapToRenderGrid ?
        snapStockGridCoord(a.y, stockMin.y(), stockMax.y(), resolution) :
        a.y;
      double bx = snapToRenderGrid ?
        snapStockGridCoord(b.x, stockMin.x(), stockMax.x(), resolution) :
        b.x;
      double by = snapToRenderGrid ?
        snapStockGridCoord(b.y, stockMin.y(), stockMax.y(), resolution) :
        b.y;
      Vector3F start(ax, ay, a.z);
      Vector3F end(bx, by, b.z);
      double dx = bx - ax;
      double dy = by - ay;

      if ((sameCoord(a.y, stockMin.y()) && sameCoord(b.y, stockMin.y())) ||
          (sameCoord(a.y, stockMax.y()) && sameCoord(b.y, stockMax.y())))
        appendBreakpointEdgePoints(boundary, start, end, xBreaks, true);
      else if ((sameCoord(a.x, stockMin.x()) &&
                sameCoord(b.x, stockMin.x())) ||
               (sameCoord(a.x, stockMax.x()) &&
                sameCoord(b.x, stockMax.x())))
        appendBreakpointEdgePoints(boundary, start, end, yBreaks, false);
      else
        appendEdgePoints
          (boundary, start, end,
           segmentCount(sqrt(dx * dx + dy * dy), resolution));
    }

    if (1 < boundary.size() && samePoint(boundary.front(), boundary.back()))
      boundary.pop_back();

    return boundary;
  }


  double loopScale2D(const vector<vector<Vector3F> > &loops) {
    bool initialized = false;
    double xMin = 0;
    double xMax = 0;
    double yMin = 0;
    double yMax = 0;

    for (const auto &loop: loops)
      for (const auto &point: loop) {
        if (!initialized) {
          xMin = xMax = point.x();
          yMin = yMax = point.y();
          initialized = true;
        } else {
          xMin = min<double>(xMin, point.x());
          xMax = max<double>(xMax, point.x());
          yMin = min<double>(yMin, point.y());
          yMax = max<double>(yMax, point.y());
        }
      }

    return initialized ? max(xMax - xMin, yMax - yMin) : 1;
  }


  bool bridgeCrossesAnyLoop
  (const Vector3F &a, const Vector3F &b,
   const vector<vector<Vector3F> > &loops, double eps) {
    for (const auto &loop: loops)
      if (bridgeCrossesLoop(a, b, loop, eps)) return true;
    return false;
  }


  bool loopsIntersect2D(const vector<Vector3F> &a,
                        const vector<Vector3F> &b, double eps) {
    for (size_t ai = 0; ai < a.size(); ai++) {
      const Vector3F &a0 = a[ai];
      const Vector3F &a1 = a[(ai + 1) % a.size()];
      for (size_t bi = 0; bi < b.size(); bi++) {
        const Vector3F &b0 = b[bi];
        const Vector3F &b1 = b[(bi + 1) % b.size()];
        if (segmentsIntersect2D(a0, a1, b0, b1, eps))
          return true;
      }
    }

    return false;
  }


  bool findVisibleBridge(const vector<Vector3F> &outer,
                         const vector<Vector3F> &hole,
                         const vector<vector<Vector3F> > &blockerLoops,
                         double eps, size_t &outerIndex,
                         size_t &holeIndex) {
    bool found = false;
    double bestDistance = numeric_limits<double>::max();

    for (size_t h = 0; h < hole.size(); h++) {
      const Vector3F &hp = hole[h];
      for (size_t o = 0; o < outer.size(); o++) {
        const Vector3F &op = outer[o];
        if (samePoint2D(hp, op, eps)) continue;
        if (bridgeCrossesAnyLoop(hp, op, blockerLoops, eps)) continue;

        double dx = hp.x() - op.x();
        double dy = hp.y() - op.y();
        double distance = dx * dx + dy * dy;
        if (!found || distance < bestDistance) {
          found = true;
          bestDistance = distance;
          outerIndex = o;
          holeIndex = h;
        }
      }
    }

    return found;
  }


  bool mergeHoleIntoLoop(vector<Vector3F> &outer,
                         const vector<Vector3F> &hole,
                         const vector<vector<Vector3F> > &blockerLoops,
                         double eps) {
    size_t outerIndex = 0;
    size_t holeIndex = 0;
    if (!findVisibleBridge(outer, hole, blockerLoops, eps,
                           outerIndex, holeIndex))
      return false;

    vector<Vector3F> merged;
    merged.reserve(outer.size() + hole.size() + 2);
    merged.insert(merged.end(), outer.begin(), outer.begin() + outerIndex + 1);
    merged.insert(merged.end(), hole.begin() + holeIndex, hole.end());
    merged.insert(merged.end(), hole.begin(), hole.begin() + holeIndex + 1);
    merged.push_back(outer[outerIndex]);
    merged.insert(merged.end(), outer.begin() + outerIndex + 1, outer.end());
    outer.swap(merged);
    return true;
  }


  struct PreparedTopPatch {
    vector<Vector3F> boundary;
    vector<array<unsigned, 3> > triangles;
  };


  struct SegmentedTopLoop {
    vector<Vector3F> boundary;
    double area = 0;
    uint64_t sourceVertices = 0;
    vector<size_t> holes;
  };


  bool prepareOwnershipBoundaryTopPatches
  (const OwnershipBoundaryPlan *ownershipBoundary, double resolution,
   vector<PreparedTopPatch> &patches, uint64_t &unsupportedHoleLoops,
   uint64_t &topBoundaryVertices, uint64_t &topTriangles,
   PatchAreaStats &topAreaStats,
   PatchBoundaryStats &topBoundaryStats,
   bool snapToRenderGrid) {
    patches.clear();
    unsupportedHoleLoops = 0;
    topBoundaryVertices = 0;
    topTriangles = 0;
    topAreaStats = PatchAreaStats();
    topBoundaryStats = PatchBoundaryStats();

    if (!ownershipBoundary) return false;
    if (ownershipBoundary->openLoops || ownershipBoundary->ambiguousVertices)
      return false;
    if (ownershipBoundary->loops.empty())
      return ownershipBoundary->analyticTiles == 0;

    vector<SegmentedTopLoop> outers;
    vector<SegmentedTopLoop> holes;

    for (const auto &loop: ownershipBoundary->loops) {
      if (!loop.closed) return false;

      SegmentedTopLoop segmented;
      segmented.boundary = buildSegmentedTopBoundary
        (loop, ownershipBoundary->stockBounds, resolution,
         ownershipBoundary->xyBins, snapToRenderGrid);
      segmented.sourceVertices = segmented.boundary.size();
      segmented.area = polygonArea2D(segmented.boundary);
      if (fabs(segmented.area) < 1e-18)
        return false;

      bool isHole = loop.signedArea < 0 ||
        loop.role.find("hole") != string::npos;
      if (isHole) {
        if (0 < segmented.area) {
          reverse(segmented.boundary.begin(), segmented.boundary.end());
          segmented.area = -segmented.area;
        }
        holes.push_back(segmented);

      } else {
        if (segmented.area < 0) {
          reverse(segmented.boundary.begin(), segmented.boundary.end());
          segmented.area = -segmented.area;
        }
        outers.push_back(segmented);
      }
    }

    if (outers.empty()) {
      unsupportedHoleLoops += holes.size();
      return false;
    }

    vector<vector<Vector3F> > allLoops;
    allLoops.reserve(outers.size() + holes.size());
    for (const auto &outer: outers) allLoops.push_back(outer.boundary);
    for (const auto &hole: holes) allLoops.push_back(hole.boundary);
    double scale = loopScale2D(allLoops);
    double eps = max(1e-12, scale * scale * 1e-12);

    vector<int> holeOwner(holes.size(), -1);
    for (size_t h = 0; h < holes.size(); h++) {
      double ownerArea = numeric_limits<double>::max();
      for (size_t o = 0; o < outers.size(); o++) {
        if (!loopInsideLoop2D(holes[h].boundary, outers[o].boundary, eps))
          continue;
        if (loopsIntersect2D(holes[h].boundary, outers[o].boundary, eps))
          continue;

        if (fabs(outers[o].area) < ownerArea) {
          ownerArea = fabs(outers[o].area);
          holeOwner[h] = (int)o;
        }
      }

      if (holeOwner[h] < 0) {
        unsupportedHoleLoops++;
        return false;
      }

      outers[(size_t)holeOwner[h]].holes.push_back(h);
    }

    for (size_t a = 0; a < holes.size(); a++)
      for (size_t b = a + 1; b < holes.size(); b++)
        if (loopInsideLoop2D(holes[a].boundary, holes[b].boundary, eps) ||
            loopInsideLoop2D(holes[b].boundary, holes[a].boundary, eps) ||
            loopsIntersect2D(holes[a].boundary, holes[b].boundary, eps)) {
          unsupportedHoleLoops += holes.size();
          return false;
        }

    for (auto &outer: outers) {
      vector<Vector3F> merged = outer.boundary;
      sort(outer.holes.begin(), outer.holes.end(),
           [&] (size_t a, size_t b) {
             return holes[b].boundary.size() < holes[a].boundary.size();
           });

      for (size_t holeIndex: outer.holes) {
        vector<vector<Vector3F> > blockerLoops;
        blockerLoops.reserve(1 + holes.size());
        blockerLoops.push_back(merged);
        for (const auto &hole: holes) blockerLoops.push_back(hole.boundary);

        if (!mergeHoleIntoLoop(merged, holes[holeIndex].boundary,
                               blockerLoops, eps)) {
          unsupportedHoleLoops++;
          return false;
        }
      }

      PreparedTopPatch patch;
      patch.boundary = merged;
      if (!triangulateTopLoop(patch.boundary, patch.triangles))
        return false;
      if (!recordPatchBoundaryStats(topBoundaryStats, patch.boundary,
                                    patch.triangles, resolution))
        return false;
      if (!recordPatchAreaStats(topAreaStats, patch.boundary,
                                patch.triangles, resolution))
        return false;

      topBoundaryVertices += patch.boundary.size();
      topTriangles += patch.triangles.size();
      patches.push_back(patch);
    }

    return true;
  }


  void emitPreparedTopPatches(TriangleSurface &surface,
                              const vector<PreparedTopPatch> &patches) {
    for (const auto &patch: patches)
      for (const auto &tri: patch.triangles) {
        Vector3F t[3] = {
          patch.boundary[tri[0]],
          patch.boundary[tri[1]],
          patch.boundary[tri[2]],
        };
        surface.add(t);
      }
  }


  void addQuad(TriangleSurface &surface,
               const Vector3F &a, const Vector3F &b,
               const Vector3F &c, const Vector3F &d) {
    Vector3F t1[3] = {a, b, c};
    Vector3F t2[3] = {a, c, d};
    surface.add(t1);
    surface.add(t2);
  }


  double triangleArea3D(const Vector3F &a, const Vector3F &b,
                        const Vector3F &c) {
    return (b - a).cross(c - a).length() * 0.5;
  }


  void addCheckedQuad(TriangleSurface &surface,
                      const Vector3F &a, const Vector3F &b,
                      const Vector3F &c, const Vector3F &d,
                      double resolution, PatchAreaStats &areaStats) {
    addQuad(surface, a, b, c, d);
    double expected = (b - a).cross(d - a).length();
    double actual = triangleArea3D(a, b, c) + triangleArea3D(a, c, d);
    recordPatchAreaStats(areaStats, expected, actual, resolution);
  }


  Vector3F lerpPoint(const Vector3F &a, const Vector3F &b,
                     unsigned i, unsigned segments) {
    if (!segments) return a;
    double t = (double)i / segments;
    return a + (b - a) * t;
  }


  void addTransitionWall(TriangleSurface &surface,
                         const Vector3F &lowStart,
                         const Vector3F &lowEnd,
                         const Vector3F &highStart,
                         const Vector3F &highEnd,
                         double resolution, bool flip,
                         uint64_t &transitionPatches,
                         PatchAreaStats &areaStats) {
    double edgeLength = lowStart.distance(lowEnd);
    double zLength = fabs(highStart.z() - lowStart.z());
    unsigned edgeSegments = segmentCount(edgeLength, resolution);
    unsigned zSegments = segmentCount(zLength, resolution);

    for (unsigned e = 0; e < edgeSegments; e++) {
      Vector3F lowA = lerpPoint(lowStart, lowEnd, e, edgeSegments);
      Vector3F lowB = lerpPoint(lowStart, lowEnd, e + 1, edgeSegments);
      Vector3F highA = lerpPoint(highStart, highEnd, e, edgeSegments);
      Vector3F highB = lerpPoint(highStart, highEnd, e + 1, edgeSegments);

      for (unsigned z = 0; z < zSegments; z++) {
        Vector3F a = lerpPoint(lowA, highA, z, zSegments);
        Vector3F b = lerpPoint(lowB, highB, z, zSegments);
        Vector3F c = lerpPoint(lowB, highB, z + 1, zSegments);
        Vector3F d = lerpPoint(lowA, highA, z + 1, zSegments);

        if (flip)
          addCheckedQuad(surface, a, d, c, b, resolution, areaStats);
        else addCheckedQuad(surface, a, b, c, d, resolution, areaStats);
        transitionPatches++;
      }
    }
  }


  bool samePoint(const Vector3F &a, const Vector3F &b) {
    return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
  }


  unsigned segmentCount(double length, double resolution) {
    if (length <= 0 || resolution <= 0) return 1;
    unsigned segments = 0;
    if (!CAMotics::Internal::ceilToUnsigned(length, resolution, segments) ||
        MAX_SPARSE_AXIS_SEGMENTS < segments)
      THROW("Sparse analytic edge exceeds the segment memory limit.");
    return segments;
  }


  void appendEdgePoints(vector<Vector3F> &points, const Vector3F &start,
                        const Vector3F &end, unsigned segments) {
    if (points.empty() || !samePoint(points.back(), start))
      points.push_back(start);

    for (unsigned i = 1; i <= segments; i++) {
      double t = (double)i / segments;
      Vector3F p(start + (end - start) * t);
      if (!samePoint(points.back(), p)) points.push_back(p);
    }
  }


  bool sameCoord(double a, double b) {
    return fabs(a - b) <= 1e-9;
  }


  double snapStockGridCoord(double value, double min, double max,
                            double resolution) {
    if (resolution <= 0) return value;
    if (sameCoord(value, min) || sameCoord(value, max)) return value;

    double snapped = min + round((value - min) / resolution) * resolution;
    if (snapped < min) return min;
    if (max < snapped) return max;
    return snapped;
  }


  void addBreakpoint(vector<double> &points, double value) {
    for (double point: points)
      if (sameCoord(point, value)) return;
    points.push_back(value);
  }


  vector<double> makeStockBreakpoints(double min, double max,
                                      double resolution, unsigned bins) {
    vector<double> points;
    unsigned segments = segmentCount(max - min, resolution);
    for (unsigned i = 0; i <= segments; i++)
      addBreakpoint(points, min + (max - min) * i / segments);

    if (bins)
      for (unsigned i = 0; i <= bins; i++)
        addBreakpoint(points, min + (max - min) * i / bins);

    sort(points.begin(), points.end());
    return points;
  }


  void appendPoint(vector<Vector3F> &points, const Vector3F &point) {
    if (points.empty() || !samePoint(points.back(), point))
      points.push_back(point);
  }


  bool coordBetween(double value, double a, double b) {
    return min(a, b) + 1e-9 < value && value < max(a, b) - 1e-9;
  }


  void appendBreakpointEdgePoints(vector<Vector3F> &points,
                                  const Vector3F &start,
                                  const Vector3F &end,
                                  const vector<double> &breakpoints,
                                  bool xAxis) {
    appendPoint(points, start);

    double c0 = xAxis ? start.x() : start.y();
    double c1 = xAxis ? end.x() : end.y();
    bool forward = c0 <= c1;

    auto appendAt = [&] (double c) {
      double t = (c - c0) / (c1 - c0);
      appendPoint(points, start + (end - start) * t);
    };

    if (forward) {
      for (double c: breakpoints)
        if (coordBetween(c, c0, c1)) appendAt(c);
    } else {
      for (auto it = breakpoints.rbegin(); it != breakpoints.rend(); ++it)
        if (coordBetween(*it, c0, c1)) appendAt(*it);
    }

    appendPoint(points, end);
  }


  vector<Vector3F> buildStockBoundary
  (const Rectangle3D &stockBounds, double resolution, unsigned bins,
   double z) {
    Vector3D min = stockBounds.getMin();
    Vector3D max = stockBounds.getMax();
    vector<double> xBreaks =
      makeStockBreakpoints(min.x(), max.x(), resolution, bins);
    vector<double> yBreaks =
      makeStockBreakpoints(min.y(), max.y(), resolution, bins);
    vector<Vector3F> boundary;

    appendBreakpointEdgePoints
      (boundary, Vector3F(min.x(), min.y(), z),
       Vector3F(max.x(), min.y(), z), xBreaks, true);
    appendBreakpointEdgePoints
      (boundary, Vector3F(max.x(), min.y(), z),
       Vector3F(max.x(), max.y(), z), yBreaks, false);
    appendBreakpointEdgePoints
      (boundary, Vector3F(max.x(), max.y(), z),
       Vector3F(min.x(), max.y(), z), xBreaks, true);
    appendBreakpointEdgePoints
      (boundary, Vector3F(min.x(), max.y(), z),
       Vector3F(min.x(), min.y(), z), yBreaks, false);

    if (1 < boundary.size() && samePoint(boundary.front(), boundary.back()))
      boundary.pop_back();

    return boundary;
  }


  void addTopPatchFan(TriangleSurface &surface,
                      const vector<Vector3F> &boundary) {
    if (boundary.size() < 3) return;

    Vector3F center;
    for (const auto &p: boundary) center += p;
    center /= boundary.size();

    for (unsigned i = 0; i < boundary.size(); i++) {
      const Vector3F &a = boundary[i];
      const Vector3F &b = boundary[(i + 1) % boundary.size()];
      Vector3F t[3] = {center, a, b};
      surface.add(t);
    }
  }


  void addBottomPatchFan(TriangleSurface &surface,
                         const vector<Vector3F> &boundary,
                         double resolution, PatchAreaStats &areaStats) {
    if (boundary.size() < 3) return;

    Vector3F center;
    for (const auto &p: boundary) center += p;
    center /= boundary.size();

    double triangleArea = 0;

    for (unsigned i = 0; i < boundary.size(); i++) {
      const Vector3F &a = boundary[i];
      const Vector3F &b = boundary[(i + 1) % boundary.size()];
      Vector3F t[3] = {center, b, a};
      surface.add(t);
      triangleArea += triangleArea2D(center, b, a);
    }

    recordPatchAreaStats(areaStats, fabs(polygonArea2D(boundary)),
                         triangleArea, resolution);
  }


  void addFullBottomPatch(TriangleSurface &surface,
                          const Rectangle3D &stockBounds,
                          double resolution, unsigned bins,
                          uint64_t &bottomPatches,
                          uint64_t &bottomBoundaryVertices,
                          PatchAreaStats &areaStats) {
    vector<Vector3F> boundary =
      buildStockBoundary(stockBounds, resolution, bins,
                         stockBounds.getMin().z());

    bottomPatches++;
    bottomBoundaryVertices += boundary.size();
    addBottomPatchFan(surface, boundary, resolution, areaStats);
  }


  void addOuterStockSideWalls(TriangleSurface &surface,
                              const Rectangle3D &stockBounds,
                              double resolution, unsigned bins,
                              uint64_t &sidePatches,
                              PatchAreaStats &areaStats) {
    Vector3D min = stockBounds.getMin();
    Vector3D max = stockBounds.getMax();
    double x0 = min.x();
    double y0 = min.y();
    double z0 = min.z();
    double x1 = max.x();
    double y1 = max.y();
    double z1 = max.z();
    vector<double> xBreaks = makeStockBreakpoints(x0, x1, resolution, bins);
    vector<double> yBreaks = makeStockBreakpoints(y0, y1, resolution, bins);

    for (unsigned i = 0; i + 1 < xBreaks.size(); i++) {
      double xa = xBreaks[i];
      double xb = xBreaks[i + 1];

      addCheckedQuad(surface, Vector3F(xa, y0, z0),
                     Vector3F(xb, y0, z0), Vector3F(xb, y0, z1),
                     Vector3F(xa, y0, z1), resolution, areaStats);
      sidePatches++;

      addCheckedQuad(surface, Vector3F(xa, y1, z0),
                     Vector3F(xa, y1, z1), Vector3F(xb, y1, z1),
                     Vector3F(xb, y1, z0), resolution, areaStats);
      sidePatches++;
    }

    for (unsigned i = 0; i + 1 < yBreaks.size(); i++) {
      double ya = yBreaks[i];
      double yb = yBreaks[i + 1];

      addCheckedQuad(surface, Vector3F(x0, ya, z0),
                     Vector3F(x0, ya, z1), Vector3F(x0, yb, z1),
                     Vector3F(x0, yb, z0), resolution, areaStats);
      sidePatches++;

      addCheckedQuad(surface, Vector3F(x1, ya, z0),
                     Vector3F(x1, yb, z0), Vector3F(x1, yb, z1),
                     Vector3F(x1, ya, z1), resolution, areaStats);
      sidePatches++;
    }
  }


  StitchVertexKey makeStitchVertexKey(const Vector3F &point,
                                      double tolerance) {
    StitchVertexKey key;
    if (!CAMotics::Internal::quantizeCoordinate
        (point.x(), tolerance, key.x) ||
        !CAMotics::Internal::quantizeCoordinate
        (point.y(), tolerance, key.y) ||
        !CAMotics::Internal::quantizeCoordinate
        (point.z(), tolerance, key.z))
      THROW("Sparse stitch vertex is outside the quantization range.");
    return key;
  }


  pair<StitchVertexKey, StitchVertexKey>
  orderedStitchEdge(const StitchVertexKey &a, const StitchVertexKey &b) {
    return b < a ? make_pair(b, a) : make_pair(a, b);
  }


  Vector3F readSurfacePoint(const vector<float> &vertices, uint64_t offset) {
    return Vector3F(vertices[offset], vertices[offset + 1],
                    vertices[offset + 2]);
  }


  void addStitchEdgeUse
  (map<pair<StitchVertexKey, StitchVertexKey>, StitchEdgeUse> &edgeUses,
   const StitchVertexKey &from, const StitchVertexKey &to,
   const Vector3F &fromPoint, const Vector3F &toPoint) {
    pair<StitchVertexKey, StitchVertexKey> key =
      orderedStitchEdge(from, to);
    StitchEdgeUse &use = edgeUses[key];
    use.count++;
    if (use.count == 1) {
      use.edge.from = from;
      use.edge.to = to;
      use.edge.fromPoint = fromPoint;
      use.edge.toPoint = toPoint;
    }
  }


  vector<StitchDirectedEdge> extractDirectedBoundaryEdges
  (const Surface &surface, double tolerance) {
    map<pair<StitchVertexKey, StitchVertexKey>, StitchEdgeUse> edgeUses;

    surface.getVertices([&] (const vector<float> &vertices,
                             const vector<float> &) {
      for (uint64_t offset = 0; offset + 8 < vertices.size(); offset += 9) {
        Vector3F points[3] = {
          readSurfacePoint(vertices, offset),
          readSurfacePoint(vertices, offset + 3),
          readSurfacePoint(vertices, offset + 6),
        };
        StitchVertexKey keys[3] = {
          makeStitchVertexKey(points[0], tolerance),
          makeStitchVertexKey(points[1], tolerance),
          makeStitchVertexKey(points[2], tolerance),
        };

        if (keys[0] == keys[1] || keys[1] == keys[2] || keys[2] == keys[0])
          continue;

        addStitchEdgeUse(edgeUses, keys[0], keys[1], points[0], points[1]);
        addStitchEdgeUse(edgeUses, keys[1], keys[2], points[1], points[2]);
        addStitchEdgeUse(edgeUses, keys[2], keys[0], points[2], points[0]);
      }
    });

    vector<StitchDirectedEdge> boundaryEdges;
    for (const auto &entry: edgeUses)
      if (entry.second.count == 1)
        boundaryEdges.push_back(entry.second.edge);

    return boundaryEdges;
  }


  bool traceBoundaryLoops(vector<StitchDirectedEdge> &boundaryEdges,
                          vector<StitchBoundaryLoop> &loops) {
    map<StitchVertexKey, vector<size_t> > adjacent;
    map<StitchVertexKey, Vector3F> points;
    for (size_t i = 0; i < boundaryEdges.size(); i++) {
      adjacent[boundaryEdges[i].from].push_back(i);
      adjacent[boundaryEdges[i].to].push_back(i);
      points[boundaryEdges[i].from] = boundaryEdges[i].fromPoint;
      points[boundaryEdges[i].to] = boundaryEdges[i].toPoint;
    }

    bool allClosed = true;
    for (size_t i = 0; i < boundaryEdges.size(); i++) {
      if (boundaryEdges[i].used) continue;

      StitchBoundaryLoop loop;
      StitchVertexKey start = boundaryEdges[i].from;
      StitchVertexKey current = start;

      for (size_t guard = 0; guard <= boundaryEdges.size(); guard++) {
        loop.vertices.push_back(current);
        loop.points.push_back(points[current]);

        auto found = adjacent.find(current);
        if (found == adjacent.end()) {
          allClosed = false;
          break;
        }

        size_t nextEdge = boundaryEdges.size();
        for (size_t candidate: found->second)
          if (!boundaryEdges[candidate].used) {
            nextEdge = candidate;
            break;
          }

        if (nextEdge == boundaryEdges.size()) {
          allClosed = false;
          break;
        }

        StitchDirectedEdge &edge = boundaryEdges[nextEdge];
        edge.used = true;
        current = edge.from == current ? edge.to : edge.from;
        if (current == start) break;
      }

      if (current == start && 2 < loop.vertices.size())
        loops.push_back(loop);
      else allClosed = false;
    }

    return allClosed;
  }


  int getConstantAxis(const vector<Vector3F> &points, double tolerance,
                      double &coordinate) {
    for (unsigned axis = 0; axis < 3; axis++) {
      double minCoord = points[0][axis];
      double maxCoord = points[0][axis];
      for (const auto &point: points) {
        minCoord = min<double>(minCoord, point[axis]);
        maxCoord = max<double>(maxCoord, point[axis]);
      }

      if (maxCoord - minCoord <= tolerance * 2) {
        coordinate = (minCoord + maxCoord) * 0.5;
        return axis;
      }
    }

    return -1;
  }


  bool coordinateOnTileLine(double coordinate, unsigned axis,
                            const RegionPlan &regionPlan,
                            double tolerance) {
    if (!regionPlan.xyBins) return false;

    Vector3D stockMin = regionPlan.stockBounds.getMin();
    Vector3D dims = regionPlan.stockBounds.getDimensions();
    double minCoord = axis ? stockMin.y() : stockMin.x();
    double step = (axis ? dims.y() : dims.x()) / regionPlan.xyBins;

    for (unsigned i = 1; i < regionPlan.xyBins; i++) {
      double line = minCoord + step * i;
      if (fabs(coordinate - line) <= tolerance * 2) return true;
    }

    return false;
  }


  bool coordinateOnStockBorder(double coordinate, unsigned axis,
                               const RegionPlan &regionPlan,
                               double tolerance) {
    Vector3D stockMin = regionPlan.stockBounds.getMin();
    Vector3D stockMax = regionPlan.stockBounds.getMax();
    double minCoord = axis ? stockMin.y() : stockMin.x();
    double maxCoord = axis ? stockMax.y() : stockMax.x();

    return fabs(coordinate - minCoord) <= tolerance * 2 ||
      fabs(coordinate - maxCoord) <= tolerance * 2;
  }


  bool supportedVerticalClosurePlane
  (const vector<Vector3F> &points, unsigned axis, double coordinate,
   const RegionPlan &regionPlan, double tolerance,
   bool includeStockBorders) {
    if (1 < axis) return false;

    double zMin = points[0].z();
    double zMax = points[0].z();
    for (const auto &point: points) {
      zMin = min<double>(zMin, point.z());
      zMax = max<double>(zMax, point.z());
    }

    if (zMax - zMin <= max(1e-9, tolerance * 0.01)) return false;
    return coordinateOnTileLine(coordinate, axis, regionPlan, tolerance) ||
      (includeStockBorders &&
       coordinateOnStockBorder(coordinate, axis, regionPlan, tolerance));
  }


  Vector3F projectClosurePoint(const Vector3F &point, unsigned axis) {
    if (axis == 0) return Vector3F(point.y(), point.z(), 0);
    if (axis == 1) return Vector3F(point.x(), point.z(), 0);
    return Vector3F(point.x(), point.y(), 0);
  }


  void orientClosureTriangles
  (vector<array<unsigned, 3> > &triangles,
   const vector<StitchVertexKey> &boundaryKeys,
   const vector<StitchDirectedEdge> &boundaryEdges) {
    map<pair<StitchVertexKey, StitchVertexKey>,
        pair<StitchVertexKey, StitchVertexKey> > existingDirections;
    for (const auto &edge: boundaryEdges)
      existingDirections[orderedStitchEdge(edge.from, edge.to)] =
        make_pair(edge.from, edge.to);

    map<pair<StitchVertexKey, StitchVertexKey>, StitchEdgeUse> candidateEdges;
    for (const auto &tri: triangles) {
      const StitchVertexKey &a = boundaryKeys[tri[0]];
      const StitchVertexKey &b = boundaryKeys[tri[1]];
      const StitchVertexKey &c = boundaryKeys[tri[2]];
      addStitchEdgeUse(candidateEdges, a, b, Vector3F(), Vector3F());
      addStitchEdgeUse(candidateEdges, b, c, Vector3F(), Vector3F());
      addStitchEdgeUse(candidateEdges, c, a, Vector3F(), Vector3F());
    }

    uint64_t same = 0;
    uint64_t opposite = 0;
    for (const auto &entry: candidateEdges) {
      if (entry.second.count != 1) continue;

      auto found = existingDirections.find(entry.first);
      if (found == existingDirections.end()) continue;

      const StitchDirectedEdge &edge = entry.second.edge;
      if (edge.from == found->second.first &&
          edge.to == found->second.second)
        same++;
      else if (edge.from == found->second.second &&
               edge.to == found->second.first)
        opposite++;
    }

    if (opposite < same)
      for (auto &tri: triangles)
        swap(tri[1], tri[2]);
  }


  bool prepareClosurePatch
  (const StitchBoundaryLoop &loop,
   const vector<StitchDirectedEdge> &boundaryEdges,
   const RegionPlan &regionPlan, double tolerance,
   double resolution, bool includeStockBorders, PreparedClosurePatch &patch,
   PatchAreaStats &areaStats, PatchBoundaryStats &patchBoundaryStats) {
    patch.boundary.clear();
    patch.triangles.clear();
    if (loop.points.size() < 3) return false;

    vector<StitchVertexKey> boundaryKeys;
    patch.boundary = loop.points;
    boundaryKeys = loop.vertices;

    double coordinate = 0;
    int axis = getConstantAxis(patch.boundary, tolerance, coordinate);
    if (axis < 0 ||
        !supportedVerticalClosurePlane(patch.boundary, (unsigned)axis,
                                       coordinate, regionPlan, tolerance,
                                       includeStockBorders))
      return false;

    vector<Vector3F> projected;
    projected.reserve(patch.boundary.size());
    for (const auto &point: patch.boundary)
      projected.push_back(projectClosurePoint(point, (unsigned)axis));

    double area = polygonArea2D(projected);
    if (fabs(area) < tolerance * tolerance) return false;
    if (area < 0) {
      reverse(projected.begin(), projected.end());
      reverse(patch.boundary.begin(), patch.boundary.end());
      reverse(boundaryKeys.begin(), boundaryKeys.end());
    }

    if (!triangulateTopLoop(projected, patch.triangles)) return false;
    orientClosureTriangles(patch.triangles, boundaryKeys, boundaryEdges);
    if (patch.triangles.empty()) return false;
    if (!recordPatchBoundaryStats(patchBoundaryStats, projected,
                                  patch.triangles, resolution))
      return false;
    return recordPatchAreaStats(areaStats, projected, patch.triangles,
                                resolution);
  }


  void emitClosurePatch(TriangleSurface &surface,
                        const PreparedClosurePatch &patch) {
    for (const auto &tri: patch.triangles) {
      Vector3F triangle[3] = {
        patch.boundary[tri[0]],
        patch.boundary[tri[1]],
        patch.boundary[tri[2]],
      };
      surface.add(triangle);
    }
  }


  SmartPointer<TriangleSurface> createPlanarBoundaryClosures
  (const Surface &surface, const RegionPlan &regionPlan, double tolerance,
   double resolution, bool includeStockBorders, uint64_t &closurePatches,
   uint64_t &closureTriangles, uint64_t &unsupportedLoops,
   PatchAreaStats &areaStats, PatchBoundaryStats &boundaryStats) {
    closurePatches = 0;
    closureTriangles = 0;
    unsupportedLoops = 0;
    areaStats = PatchAreaStats();
    boundaryStats = PatchBoundaryStats();

    vector<StitchDirectedEdge> boundaryEdges =
      extractDirectedBoundaryEdges(surface, tolerance);
    if (boundaryEdges.empty()) return 0;

    vector<StitchBoundaryLoop> loops;
    if (!traceBoundaryLoops(boundaryEdges, loops))
      unsupportedLoops++;

    SmartPointer<TriangleSurface> closures = new TriangleSurface;
    for (const auto &loop: loops) {
      PreparedClosurePatch patch;
      if (!prepareClosurePatch(loop, boundaryEdges, regionPlan, tolerance,
                               resolution, includeStockBorders, patch,
                               areaStats, boundaryStats)) {
        unsupportedLoops++;
        continue;
      }

      emitClosurePatch(*closures, patch);
      closurePatches++;
      closureTriangles += patch.triangles.size();
    }

    return closureTriangles ? closures : 0;
  }


  bool pointOnRenderRegionBoundary(const Vector3F &point,
                                   const RegionPlan &regionPlan,
                                   double tolerance) {
    const vector<RegionPlanRegion> &regions =
      regionPlan.renderRegions.empty() ?
      regionPlan.activeRegions : regionPlan.renderRegions;

    for (const auto &region: regions) {
      Rectangle3D bounds = region.bounds;
      if (bounds == Rectangle3D()) continue;

      Vector3D minPoint = bounds.getMin();
      Vector3D maxPoint = bounds.getMax();
      bool inside = minPoint.x() - tolerance * 2 <= point.x() &&
        point.x() <= maxPoint.x() + tolerance * 2 &&
        minPoint.y() - tolerance * 2 <= point.y() &&
        point.y() <= maxPoint.y() + tolerance * 2 &&
        minPoint.z() - tolerance * 2 <= point.z() &&
        point.z() <= maxPoint.z() + tolerance * 2;
      if (!inside) continue;

      double distance = min<double>
        (min<double>(fabs(point.x() - minPoint.x()),
                     fabs(point.x() - maxPoint.x())),
         min<double>
         (min<double>(fabs(point.y() - minPoint.y()),
                      fabs(point.y() - maxPoint.y())),
          min<double>(fabs(point.z() - minPoint.z()),
                      fabs(point.z() - maxPoint.z()))));
      if (distance <= tolerance * 2) return true;
    }

    return false;
  }


  struct BoundaryLoopDetail {
    uint64_t id = 0;
    uint64_t vertices = 0;
    uint64_t renderBoundaryVertices = 0;
    int constantAxis = -1;
    double coordinate = 0;
    bool planar = false;
    bool horizontal = false;
    bool supportedClosure = false;
    bool stockBorder = false;
    bool tileLine = false;
    bool touchesRenderBoundary = false;
    Rectangle3D bounds;
  };


  struct BoundaryLoopSummary {
    uint64_t loops = 0;
    uint64_t openChains = 0;
    uint64_t planarLoops = 0;
    uint64_t nonPlanarLoops = 0;
    uint64_t horizontalLoops = 0;
    uint64_t supportedClosureLoops = 0;
    uint64_t stockBorderLoops = 0;
    uint64_t tileLineLoops = 0;
    uint64_t renderBoundaryLoops = 0;
    uint64_t nonPlanarRenderBoundaryLoops = 0;
    vector<BoundaryLoopDetail> details;
  };


  void writeBoundaryLoopDetails(JSON::Sink &sink,
                                const BoundaryLoopSummary &summary) {
    sink.insertList("boundary-loop-details");
    for (const auto &detail: summary.details) {
      sink.appendDict(true);
      sink.insert("id", detail.id);
      sink.insert("vertices", detail.vertices);
      sink.insert("render-boundary-vertices",
                  detail.renderBoundaryVertices);
      sink.insert("constant-axis", (int64_t)detail.constantAxis);
      sink.insert("coordinate", detail.coordinate);
      sink.insertBoolean("planar", detail.planar);
      sink.insertBoolean("horizontal", detail.horizontal);
      sink.insertBoolean("supported-closure", detail.supportedClosure);
      sink.insertBoolean("stock-border", detail.stockBorder);
      sink.insertBoolean("tile-line", detail.tileLine);
      sink.insertBoolean("touches-render-boundary",
                         detail.touchesRenderBoundary);
      writeBounds(sink, "bounds", detail.bounds);
      sink.endDict();
    }
    sink.endList();
  }


  BoundaryLoopSummary summarizeBoundaryLoops
  (const Surface &surface, const RegionPlan &regionPlan, double tolerance) {
    BoundaryLoopSummary summary;
    vector<StitchDirectedEdge> boundaryEdges =
      extractDirectedBoundaryEdges(surface, tolerance);
    if (boundaryEdges.empty()) return summary;

    vector<StitchBoundaryLoop> loops;
    if (!traceBoundaryLoops(boundaryEdges, loops)) summary.openChains++;
    summary.loops = loops.size();

    for (const auto &loop: loops) {
      BoundaryLoopDetail detail;
      detail.id = summary.details.size();
      detail.vertices = loop.points.size();
      for (const auto &point: loop.points) {
        detail.bounds.add(Vector3D(point.x(), point.y(), point.z()));
        if (pointOnRenderRegionBoundary(point, regionPlan, tolerance))
          detail.renderBoundaryVertices++;
      }

      detail.touchesRenderBoundary = 0 < detail.renderBoundaryVertices;
      if (detail.touchesRenderBoundary) summary.renderBoundaryLoops++;

      double coordinate = 0;
      int axis = getConstantAxis(loop.points, tolerance, coordinate);
      detail.constantAxis = axis;
      detail.coordinate = coordinate;
      if (axis < 0) {
        summary.nonPlanarLoops++;
        if (detail.touchesRenderBoundary)
          summary.nonPlanarRenderBoundaryLoops++;
        summary.details.push_back(detail);
        continue;
      }

      detail.planar = true;
      summary.planarLoops++;
      if (axis == 2) {
        detail.horizontal = true;
        summary.horizontalLoops++;
        summary.details.push_back(detail);
        continue;
      }

      if (supportedVerticalClosurePlane(loop.points, (unsigned)axis,
                                        coordinate, regionPlan, tolerance,
                                        true)) {
        detail.supportedClosure = true;
        summary.supportedClosureLoops++;
      }
      if (coordinateOnStockBorder(coordinate, (unsigned)axis, regionPlan,
                                  tolerance)) {
        detail.stockBorder = true;
        summary.stockBorderLoops++;
      }
      if (coordinateOnTileLine(coordinate, (unsigned)axis, regionPlan,
                               tolerance)) {
        detail.tileLine = true;
        summary.tileLineLoops++;
      }
      summary.details.push_back(detail);
    }

    return summary;
  }


  SmartPointer<Surface> renderActiveRegions
  (const Simulation &sim, const RegionPlan &regionPlan, unsigned threads,
   uint64_t &surfaceChunks, uint64_t &renderCells, RenderStats &stats) {
    const vector<RegionPlanRegion> &regions = regionPlan.renderRegions.empty() ?
      regionPlan.activeRegions : regionPlan.renderRegions;
    if (regions.empty()) return 0;

    double simTime = min(sim.path->getTime(), sim.time);
    SmartPointer<ToolSweep> sweep =
      new ToolSweep(sim.path, 0, numeric_limits<double>::max(),
                    sim.toolSweepXYBins, sim.toolSweepXYZBins);
    sweep->setEndTime(simTime);

    CutWorkpiece cutWP(sweep, sim.workpiece);
    Task task;
    SmartPointer<CompositeSurface> composite = new CompositeSurface;
    uint64_t renderedTriangles = 0;

    {
      Profile::Scope scope("sparse_region_render_active_regions");
      for (const auto &region: regions) {
        if (task.shouldQuit()) break;

        Rectangle3D sampleBounds = region.bounds.grow(sim.resolution * 0.9);
        SmartPointer<GridTree> tree =
          new GridTree(Grid(sampleBounds, sim.resolution));
        renderCells += estimateGridCells(region.bounds, sim.resolution, false);

        Renderer renderer(task);
        renderer.render(cutWP, *tree, region.bounds, threads, sim.mode);
        stats.add(renderer.getStats());
        if (task.shouldQuit()) break;

        SmartPointer<Surface> surface = new TriangleSurface(*tree);
        uint64_t triangles = surface->getTriangleCount();
        renderedTriangles += triangles;
        if (triangles) {
          composite->add(surface);
          surfaceChunks++;
        }
      }
    }

    Profile::setMetric("sparse_region_render_chunks", surfaceChunks);
    Profile::setMetric("sparse_region_render_cells", renderCells);
    Profile::setMetric("sparse_region_render_triangles", renderedTriangles);

    return composite->consolidate();
  }


  SmartPointer<Surface> renderFullBaselineImpl(const Simulation &sim,
                                           unsigned threads) {
    Simulation renderSim
      (sim.path, sim.planConf, 0, sim.workpiece, sim.resolution, sim.time,
       sim.mode, threads ? threads : sim.threads, sim.toolSweepXYBins,
       sim.toolSweepXYZBins);

    CutSim cutSim;
    Profile::Scope scope("sparse_region_render_baseline_surface_compute");
    return cutSim.computeSurface(renderSim);
  }


  struct RegionRenderResult {
    SmartPointer<Surface> surface;
    uint64_t surfaceChunks = 0;
    uint64_t renderCells = 0;
    RenderStats stats;
    string rendererName;
  };


  struct StitchResult {
    SmartPointer<Surface> surface;
    uint64_t inputMCTriangles = 0;
    uint64_t analyticTriangles = 0;
    uint64_t analyticTopPatches = 0;
    uint64_t analyticTopBoundaryVertices = 0;
    uint64_t analyticTopTriangles = 0;
    uint64_t analyticTopUnsupportedHoleLoops = 0;
    uint64_t analyticTopOwnershipRejected = 0;
    PatchAreaStats analyticTopAreaStats;
    PatchBoundaryStats analyticTopBoundaryStats;
    string analyticTopPatchSource;
    uint64_t analyticBottomPatches = 0;
    uint64_t analyticBottomBoundaryVertices = 0;
    PatchAreaStats analyticBottomAreaStats;
    string analyticBottomPatchSource;
    uint64_t analyticTransitionPatches = 0;
    PatchAreaStats analyticTransitionAreaStats;
    uint64_t analyticSidePatches = 0;
    PatchAreaStats analyticSideAreaStats;
    uint64_t analyticClosurePatches = 0;
    uint64_t analyticClosureTriangles = 0;
    PatchAreaStats analyticClosureAreaStats;
    PatchBoundaryStats analyticClosureBoundaryStats;
    uint64_t analyticClosureUnsupportedLoops = 0;
    uint64_t analyticClosureRejected = 0;
    uint64_t analyticClosureCandidateBoundaryEdges = 0;
    uint64_t analyticClosureCandidateNonManifoldEdges = 0;
    uint64_t analyticClosureCandidateMisorientedEdges = 0;
    uint64_t analyticClosureCandidateDegenerateTriangles = 0;
    uint64_t analyticTransitionReplacementCandidateBoundaryEdges = 0;
    uint64_t analyticTransitionReplacementCandidateNonManifoldEdges = 0;
    uint64_t analyticTransitionReplacementCandidateMisorientedEdges = 0;
    uint64_t analyticTransitionReplacementCandidateDegenerateTriangles = 0;
    uint64_t analyticRenderGridSnapUsed = 1;
    uint64_t analyticRenderGridSnapCandidateBoundaryEdges = 0;
    uint64_t analyticRenderGridSnapCandidateNonManifoldEdges = 0;
    uint64_t analyticRenderGridSnapCandidateMisorientedEdges = 0;
    uint64_t analyticRenderGridSnapCandidateDegenerateTriangles = 0;
    BoundaryLoopSummary boundaryLoopSummary;
    double topologyTolerance = 0;
    SurfaceTopologyReport topology;
    bool reductionEligibilityValid = false;
    uint64_t reductionMCReducibleTriangles = 0;
    uint64_t reductionMCSeamLockedTriangles = 0;
    uint64_t reductionAnalyticLockedTriangles = 0;
    uint64_t reductionUnknownLockedTriangles = 0;
    uint64_t reductionLockedSeamVertices = 0;
    uint64_t reductionLockedSeamEdges = 0;

    bool geometryAccepted() const {
      return analyticTopAreaStats.accepted() &&
        analyticTopBoundaryStats.accepted() &&
        !analyticTopOwnershipRejected &&
        analyticBottomAreaStats.accepted() &&
        analyticTransitionAreaStats.accepted() &&
        analyticSideAreaStats.accepted() &&
        analyticClosureAreaStats.accepted() &&
        analyticClosureBoundaryStats.accepted();
    }

    bool accepted() const {
      return topology.accepted() && geometryAccepted();
    }
  };


  ReductionLockedVertex getReductionVertex
  (const vector<float> &vertices, uint64_t triangle, unsigned corner,
   double tolerance) {
    size_t offset = (size_t)triangle * 9 + (size_t)corner * 3;
    return quantizeReductionVertex
      (vertices[offset], vertices[offset + 1], vertices[offset + 2],
       tolerance);
  }


  bool attachReductionEligibility(StitchResult &result) {
    TriangleSurface *surface =
      dynamic_cast<TriangleSurface *>(result.surface.get());
    if (!surface) return false;
    surface->markSparseAcceptedSurface();

    const vector<float> &vertices = surface->getVertices();
    uint64_t triangleCount = surface->getTriangleCount();
    if (triangleCount != result.inputMCTriangles + result.analyticTriangles)
      return false;

    ReductionEligibility eligibility;
    eligibility.quantizationTolerance = result.topologyTolerance;
    eligibility.triangleOrigins.assign
      ((size_t)triangleCount, (uint8_t)REDUCTION_ANALYTIC_LOCKED);
    fill(eligibility.triangleOrigins.begin(),
         eligibility.triangleOrigins.begin() + result.inputMCTriangles,
         (uint8_t)REDUCTION_MC_REDUCIBLE);

    set<ReductionLockedVertex> analyticVertices;
    set<ReductionLockedEdge> analyticEdges;
    for (uint64_t triangle = result.inputMCTriangles;
         triangle < triangleCount; triangle++) {
      ReductionLockedVertex points[3];
      for (unsigned corner = 0; corner < 3; corner++) {
        points[corner] = getReductionVertex
          (vertices, triangle, corner, eligibility.quantizationTolerance);
        analyticVertices.insert(points[corner]);
      }
      for (unsigned edge = 0; edge < 3; edge++)
        analyticEdges.insert
          (ReductionLockedEdge(points[edge], points[(edge + 1) % 3]));
    }

    set<ReductionLockedVertex> lockedVertices;
    set<ReductionLockedEdge> lockedEdges;
    for (uint64_t triangle = 0; triangle < result.inputMCTriangles;
         triangle++) {
      ReductionLockedVertex points[3];
      bool seamLocked = false;
      for (unsigned corner = 0; corner < 3; corner++) {
        points[corner] = getReductionVertex
          (vertices, triangle, corner, eligibility.quantizationTolerance);
        if (analyticVertices.count(points[corner])) {
          seamLocked = true;
          lockedVertices.insert(points[corner]);
        }
      }

      for (unsigned edge = 0; edge < 3; edge++) {
        ReductionLockedEdge key(points[edge], points[(edge + 1) % 3]);
        if (!analyticEdges.count(key)) continue;
        seamLocked = true;
        lockedEdges.insert(key);
        lockedVertices.insert(key.a);
        lockedVertices.insert(key.b);
      }

      if (seamLocked)
        eligibility.triangleOrigins[(size_t)triangle] =
          REDUCTION_MC_SEAM_LOCKED;
    }

    eligibility.lockedSeamVertices.assign
      (lockedVertices.begin(), lockedVertices.end());
    eligibility.lockedSeamEdges.assign(lockedEdges.begin(), lockedEdges.end());
    eligibility.seal(vertices);
    if (!eligibility.validFor(vertices)) return false;

    result.reductionMCReducibleTriangles =
      eligibility.count(REDUCTION_MC_REDUCIBLE);
    result.reductionMCSeamLockedTriangles =
      eligibility.count(REDUCTION_MC_SEAM_LOCKED);
    result.reductionAnalyticLockedTriangles =
      eligibility.count(REDUCTION_ANALYTIC_LOCKED);
    result.reductionUnknownLockedTriangles =
      eligibility.count(REDUCTION_UNKNOWN_LOCKED);
    result.reductionLockedSeamVertices =
      eligibility.lockedSeamVertices.size();
    result.reductionLockedSeamEdges = eligibility.lockedSeamEdges.size();
    surface->setReductionEligibility(eligibility);
    return true;
  }


  SmartPointer<TriangleSurface>
  createAnalyticStockPatches(const RegionPlan &regionPlan,
                             const OwnershipBoundaryPlan *ownershipBoundary,
                             double resolution,
                             uint64_t &topPatches,
                              uint64_t &topBoundaryVertices,
                              uint64_t &topTriangles,
                              uint64_t &topUnsupportedHoleLoops,
                              uint64_t &topOwnershipRejected,
                              PatchAreaStats &topAreaStats,
                              PatchBoundaryStats &topBoundaryStats,
                              string &topPatchSource,
                             uint64_t &bottomPatches,
                             uint64_t &bottomBoundaryVertices,
                             PatchAreaStats &bottomAreaStats,
                             string &bottomPatchSource,
                             uint64_t &transitionPatches,
                             PatchAreaStats &transitionAreaStats,
                             uint64_t &sidePatches,
                             PatchAreaStats &sideAreaStats,
                             bool emitTransitionWalls,
                             bool snapTopToRenderGrid) {
    SmartPointer<TriangleSurface> surface = new TriangleSurface;
    unsigned bins = regionPlan.xyBins;
    size_t tileCount = sparseTileCount(bins);
    vector<bool> activeTiles(tileCount, false);
    vector<double> activeDepths(tileCount, 0);
    double zBottom = regionPlan.stockBounds.getMin().z();
    double zTop = regionPlan.stockBounds.getMax().z();
    Vector3D stockMin = regionPlan.stockBounds.getMin();
    Vector3D stockMax = regionPlan.stockBounds.getMax();
    Vector3D stockDims = regionPlan.stockBounds.getDimensions();
    double xStep = bins ? stockDims.x() / bins : 0;
    double yStep = bins ? stockDims.y() / bins : 0;
    vector<PreparedTopPatch> ownershipTopPatches;
    bottomAreaStats = PatchAreaStats();
    transitionAreaStats = PatchAreaStats();
    sideAreaStats = PatchAreaStats();
    const vector<RegionPlanRegion> &ownershipRegions =
      getSurfaceOwnershipRegions(regionPlan);
    bool useOwnershipTopPatches =
      prepareOwnershipBoundaryTopPatches
       (ownershipBoundary, resolution, ownershipTopPatches,
        topUnsupportedHoleLoops, topBoundaryVertices, topTriangles,
        topAreaStats, topBoundaryStats,
        snapTopToRenderGrid);
    topOwnershipRejected = ownershipBoundary && !useOwnershipTopPatches ? 1 : 0;
    topPatchSource = useOwnershipTopPatches ?
      "ownership-boundary-loops" : "region-tile-fans";
    bool useFullStockBottom = true;

    if (useOwnershipTopPatches) {
      topPatches = ownershipTopPatches.size();
      emitPreparedTopPatches(*surface, ownershipTopPatches);
    }

    auto isActive = [&] (unsigned x, unsigned y) {
      return x < bins && y < bins && activeTiles[(size_t)y * bins + x];
    };

    for (const auto &region: ownershipRegions)
      for (unsigned y = region.tileY;
           y < tileEnd(region.tileY, region.tileHeight, bins); y++)
        for (unsigned x = region.tileX;
             x < tileEnd(region.tileX, region.tileWidth, bins); x++) {
          activeTiles[(size_t)y * bins + x] = true;
          activeDepths[(size_t)y * bins + x] = region.activeDepth;
        }

    auto tileIndex = [bins] (unsigned x, unsigned y) {
      return (size_t)y * bins + x;
    };
    auto activeDepthAt = [&] (unsigned x, unsigned y) {
      return x < bins && y < bins ? activeDepths[tileIndex(x, y)] : 0.0;
    };
    auto tileXMin = [&] (unsigned x) {
      return stockMin.x() + x * xStep;
    };
    auto tileXMax = [&] (unsigned x) {
      return x + 1 == bins ? stockMax.x() : stockMin.x() + (x + 1) * xStep;
    };
    auto tileYMin = [&] (unsigned y) {
      return stockMin.y() + y * yStep;
    };
    auto tileYMax = [&] (unsigned y) {
      return y + 1 == bins ? stockMax.y() : stockMin.y() + (y + 1) * yStep;
    };

    for (const auto &region: ownershipRegions)
      if (region.bounds.getMin().z() <= zBottom + 1e-9) {
        useFullStockBottom = false;
        break;
      }

    bottomPatchSource = useFullStockBottom ?
      "full-stock-bottom" : "region-tile-fans";
    if (useFullStockBottom)
      addFullBottomPatch(*surface, regionPlan.stockBounds, resolution, bins,
                         bottomPatches, bottomBoundaryVertices,
                         bottomAreaStats);

    if (bins)
      for (unsigned y = 0; y < bins; y++)
        for (unsigned x = 0; x < bins; x++) {
          if (!isActive(x, y)) continue;

          double depth = activeDepthAt(x, y);
          if (x + 1 < bins && isActive(x + 1, y)) {
            double otherDepth = activeDepthAt(x + 1, y);
            if (resolution * 0.5 < fabs(depth - otherDepth)) {
              bool currentIsDeep = otherDepth < depth;
              double zLow = zTop - max(depth, otherDepth);
              double zHigh = zTop - min(depth, otherDepth);
              double edgeX = tileXMax(x);
              double y0 = tileYMin(y);
              double y1 = tileYMax(y);

              if (emitTransitionWalls)
                addTransitionWall
                  (*surface, Vector3F(edgeX, y0, zLow),
                   Vector3F(edgeX, y1, zLow),
                   Vector3F(edgeX, y0, zHigh),
                   Vector3F(edgeX, y1, zHigh),
                   resolution, !currentIsDeep, transitionPatches,
                   transitionAreaStats);
            }
          }

          if (y + 1 < bins && isActive(x, y + 1)) {
            double otherDepth = activeDepthAt(x, y + 1);
            if (resolution * 0.5 < fabs(depth - otherDepth)) {
              bool currentIsDeep = otherDepth < depth;
              double zLow = zTop - max(depth, otherDepth);
              double zHigh = zTop - min(depth, otherDepth);
              double edgeY = tileYMax(y);
              double x0 = tileXMin(x);
              double x1 = tileXMax(x);

              if (emitTransitionWalls)
                addTransitionWall
                  (*surface, Vector3F(x1, edgeY, zLow),
                   Vector3F(x0, edgeY, zLow),
                   Vector3F(x1, edgeY, zHigh),
                   Vector3F(x0, edgeY, zHigh),
                   resolution, !currentIsDeep, transitionPatches,
                   transitionAreaStats);
            }
          }
        }

    if (!bins) {
      addOuterStockSideWalls
        (*surface, regionPlan.stockBounds, resolution, bins, sidePatches,
         sideAreaStats);
      return surface;
    }

    for (unsigned tileY = 0; tileY < bins; tileY++)
      for (unsigned tileX = 0; tileX < bins; tileX++) {
        if (isActive(tileX, tileY)) continue;

        double x0 = tileXMin(tileX);
        double y0 = tileYMin(tileY);
        double x1 = tileXMax(tileX);
        double y1 = tileYMax(tileY);

        bool southActive =
          tileY && isActive(tileX, tileY - 1);
        bool eastActive =
          tileX + 1 < bins && isActive(tileX + 1, tileY);
        bool northActive =
          tileY + 1 < bins && isActive(tileX, tileY + 1);
        bool westActive =
          tileX && isActive(tileX - 1, tileY);
        bool southSegmented = southActive || tileY == 0;
        bool eastSegmented = eastActive || tileX + 1 == bins;
        bool northSegmented = northActive || tileY + 1 == bins;
        bool westSegmented = westActive || tileX == 0;

        Vector3F sw(x0, y0, zTop);
        Vector3F se(x1, y0, zTop);
        Vector3F ne(x1, y1, zTop);
        Vector3F nw(x0, y1, zTop);
        vector<Vector3F> boundary;

        appendEdgePoints
          (boundary, sw, se,
           southSegmented ? segmentCount(x1 - x0, resolution) : 1);
        appendEdgePoints
          (boundary, se, ne,
           eastSegmented ? segmentCount(y1 - y0, resolution) : 1);
        appendEdgePoints
          (boundary, ne, nw,
           northSegmented ? segmentCount(x1 - x0, resolution) : 1);
        appendEdgePoints
          (boundary, nw, sw,
           westSegmented ? segmentCount(y1 - y0, resolution) : 1);

        if (1 < boundary.size() &&
            samePoint(boundary.front(), boundary.back()))
          boundary.pop_back();

        if (!useOwnershipTopPatches) {
          topPatches++;
          topBoundaryVertices += boundary.size();
          addTopPatchFan(*surface, boundary);
        }

        if (!useFullStockBottom) {
          vector<Vector3F> bottomBoundary;
          bottomBoundary.reserve(boundary.size());
          for (const auto &p: boundary)
            bottomBoundary.push_back(Vector3F(p.x(), p.y(), zBottom));

          bottomPatches++;
          bottomBoundaryVertices += bottomBoundary.size();
          addBottomPatchFan(*surface, bottomBoundary, resolution,
                            bottomAreaStats);
        }
      }

    if (!useOwnershipTopPatches) topTriangles = topBoundaryVertices;

    addOuterStockSideWalls
      (*surface, regionPlan.stockBounds, resolution, bins, sidePatches,
       sideAreaStats);

    return surface;
  }


  OwnershipBoundaryPlan createOwnershipBoundaryPlan
  (const RegionPlan &regionPlan) {
    OwnershipBoundaryPlan boundary;
    boundary.planner = "grid-top-ownership-boundary-v1";
    boundary.ownership = regionPlan.ownership;
    boundary.xyBins = regionPlan.xyBins;
    boundary.haloCells = regionPlan.haloCells;
    boundary.halo = regionPlan.halo;
    boundary.stockBounds = regionPlan.stockBounds;

    unsigned bins = regionPlan.xyBins;
    if (!bins || regionPlan.stockBounds == Rectangle3D()) return boundary;

    Vector3D stockMin = regionPlan.stockBounds.getMin();
    Vector3D stockMax = regionPlan.stockBounds.getMax();
    Vector3D dims = regionPlan.stockBounds.getDimensions();
    double xStep = dims.x() / bins;
    double yStep = dims.y() / bins;
    double zTop = stockMax.z();

    size_t tileCount = sparseTileCount(bins);
    vector<char> activeTiles(tileCount, false);
    vector<string> activeRegionIDs(tileCount);
    vector<string> analyticRegionIDs(tileCount);

    auto index = [bins] (unsigned x, unsigned y) {
      return (size_t)y * bins + x;
    };
    auto isActive = [&] (unsigned x, unsigned y) {
      return x < bins && y < bins && activeTiles[index(x, y)];
    };
    auto isAnalytic = [&] (unsigned x, unsigned y) {
      return x < bins && y < bins && !activeTiles[index(x, y)];
    };
    auto gridX = [&] (unsigned x) {
      return x == bins ? stockMax.x() : stockMin.x() + x * xStep;
    };
    auto gridY = [&] (unsigned y) {
      return y == bins ? stockMax.y() : stockMin.y() + y * yStep;
    };

    const vector<RegionPlanRegion> &ownershipRegions =
      getSurfaceOwnershipRegions(regionPlan);
    for (const auto &region: ownershipRegions)
      for (unsigned y = region.tileY;
           y < tileEnd(region.tileY, region.tileHeight, bins); y++)
        for (unsigned x = region.tileX;
             x < tileEnd(region.tileX, region.tileWidth, bins); x++) {
          activeTiles[index(x, y)] = true;
          activeRegionIDs[index(x, y)] = region.id;
        }

    for (const auto &region: regionPlan.analyticRegions)
      for (unsigned y = region.tileY;
           y < tileEnd(region.tileY, region.tileHeight, bins); y++)
        for (unsigned x = region.tileX;
             x < tileEnd(region.tileX, region.tileWidth, bins); x++)
          analyticRegionIDs[index(x, y)] = region.id;

    for (unsigned y = 0; y < bins; y++)
      for (unsigned x = 0; x < bins; x++)
        if (isActive(x, y)) boundary.activeTiles++;
        else boundary.analyticTiles++;

    vector<PendingBoundaryEdge> pendingEdges;
    map<BoundaryGridPoint, vector<size_t> > outgoing;

    auto adjacentOwnership = [&] (unsigned x, unsigned y) {
      if (bins <= x || bins <= y) return string("outside-stock");
      return isActive(x, y) ? string("mc") : string("analytic");
    };
    auto adjacentRegionID = [&] (unsigned x, unsigned y) {
      if (bins <= x || bins <= y) return string();
      return isActive(x, y) ? activeRegionIDs[index(x, y)] :
        analyticRegionIDs[index(x, y)];
    };
    auto addEdge =
      [&] (const BoundaryGridPoint &from, const BoundaryGridPoint &to,
           unsigned analyticX, unsigned analyticY, unsigned adjacentX,
           unsigned adjacentY, const string &side) {
        PendingBoundaryEdge edge;
        edge.from = from;
        edge.to = to;
        edge.info.analyticRegionID =
          analyticRegionIDs[index(analyticX, analyticY)];
        edge.info.adjacentOwnership =
          adjacentOwnership(adjacentX, adjacentY);
        edge.info.adjacentRegionID =
          adjacentRegionID(adjacentX, adjacentY);
        edge.info.side = side;
        pendingEdges.push_back(edge);
        outgoing[from].push_back(pendingEdges.size() - 1);
      };

    for (unsigned y = 0; y < bins; y++)
      for (unsigned x = 0; x < bins; x++) {
        if (!isAnalytic(x, y)) continue;

        if (y == 0 || !isAnalytic(x, y - 1))
          addEdge(BoundaryGridPoint(x, y), BoundaryGridPoint(x + 1, y),
                  x, y, x, y ? y - 1 : bins, "south");
        if (x + 1 == bins || !isAnalytic(x + 1, y))
          addEdge(BoundaryGridPoint(x + 1, y),
                  BoundaryGridPoint(x + 1, y + 1),
                  x, y, x + 1, y, "east");
        if (y + 1 == bins || !isAnalytic(x, y + 1))
          addEdge(BoundaryGridPoint(x + 1, y + 1),
                  BoundaryGridPoint(x, y + 1),
                  x, y, x, y + 1, "north");
        if (x == 0 || !isAnalytic(x - 1, y))
          addEdge(BoundaryGridPoint(x, y + 1), BoundaryGridPoint(x, y),
                  x, y, x ? x - 1 : bins, y, "west");
      }

    boundary.boundaryEdges = pendingEdges.size();

    auto makePoint = [&] (const BoundaryGridPoint &point) {
      OwnershipBoundaryPoint p;
      p.x = gridX(point.x);
      p.y = gridY(point.y);
      p.z = zTop;
      p.gridX = point.x;
      p.gridY = point.y;
      return p;
    };

    for (size_t i = 0; i < pendingEdges.size(); i++) {
      if (pendingEdges[i].used) continue;

      BoundaryGridPoint start = pendingEdges[i].from;
      BoundaryGridPoint current = start;
      vector<BoundaryGridPoint> gridVertices;
      vector<OwnershipBoundaryEdge> loopEdges;
      bool closed = false;
      size_t edgeIndex = i;

      while (edgeIndex < pendingEdges.size()) {
        PendingBoundaryEdge &edge = pendingEdges[edgeIndex];
        if (edge.used) break;

        edge.used = true;
        gridVertices.push_back(edge.from);
        loopEdges.push_back(edge.info);
        current = edge.to;

        if (current == start) {
          closed = true;
          break;
        }

        auto found = outgoing.find(current);
        if (found == outgoing.end()) break;

        size_t nextEdge = pendingEdges.size();
        unsigned unused = 0;
        for (size_t candidate: found->second)
          if (!pendingEdges[candidate].used) {
            if (nextEdge == pendingEdges.size()) nextEdge = candidate;
            unused++;
          }

        if (1 < unused) boundary.ambiguousVertices++;
        if (nextEdge == pendingEdges.size()) break;
        edgeIndex = nextEdge;
      }

      if (!closed) {
        gridVertices.push_back(current);
        boundary.openLoops++;
      }

      uint64_t rawVertices = gridVertices.size();
      if (closed) simplifyClosedGridLoop(gridVertices, loopEdges);

      OwnershipBoundaryLoop loop;
      loop.id = "top-loop-" + to_string(boundary.loops.size());
      loop.plane = "z-max";
      loop.coordinate = zTop;
      loop.rawVertices = rawVertices;
      loop.closed = closed ? 1 : 0;
      loop.edges = loopEdges;

      bool touchesStock = false;
      for (const auto &point: gridVertices) {
        if (!point.x || !point.y || point.x == bins || point.y == bins)
          touchesStock = true;
        loop.vertices.push_back(makePoint(point));
      }

      if (closed && 2 < loop.vertices.size())
        loop.signedArea = polygonArea2D(loop.vertices);

      loop.touchesStockBorder = touchesStock ? 1 : 0;
      loop.role = 0 <= loop.signedArea ?
        "analytic-top-outer" : "analytic-top-hole";
      boundary.rawVertices += rawVertices;
      boundary.contractedVertices += loop.vertices.size();
      boundary.loops.push_back(loop);
    }

    return boundary;
  }


  RegionRenderResult renderRegionSurfaceInternal
  (const Simulation &toolpathSim, const RegionPlan &regionPlan,
   unsigned threads) {
    if (toolpathSim.path.isNull())
      THROW("Toolpath artifact has no path.");
    if (regionPlan.planner.empty())
      THROW("Region plan is empty.");

    unsigned renderThreads = threads ? threads : toolpathSim.threads;
    Simulation renderSim
      (toolpathSim.path, toolpathSim.planConf, 0, toolpathSim.workpiece,
       toolpathSim.resolution, toolpathSim.time, toolpathSim.mode,
       renderThreads, toolpathSim.toolSweepXYBins,
       toolpathSim.toolSweepXYZBins);

    RegionRenderResult result;
    result.rendererName = "planned-active-region-render";
    result.surface =
      renderActiveRegions(renderSim, regionPlan, renderThreads,
                          result.surfaceChunks, result.renderCells,
                          result.stats);

    if (result.surface.isNull() || !result.surface->getTriangleCount()) {
      result.rendererName = "baseline-full-render-fallback";
      result.surface = renderFullBaselineImpl(renderSim, renderThreads);
      result.surfaceChunks = result.surface.isNull() ? 0 : 1;
    }

    Profile::setMetric("sparse_region_surface_triangles",
                       result.surface.isNull() ?
                       0 : result.surface->getTriangleCount());
    Profile::setMetric("sparse_region_surface_plan_active_regions",
                       regionPlan.activeRegions.size());
    Profile::setMetric("sparse_region_surface_plan_render_regions",
                       regionPlan.renderRegions.size());
    Profile::setMetric("sparse_region_surface_plan_analytic_regions",
                       regionPlan.analyticRegions.size());
    Profile::setMetric("sparse_region_surface_chunks",
                       result.surfaceChunks);
    Profile::setMetric("sparse_region_surface_render_cells",
                       result.renderCells);
    Profile::setMetric("sparse_region_surface_cells_visited",
                       result.stats.cellsVisited);
    Profile::setMetric("sparse_region_surface_cells_culled",
                       result.stats.cellsCulled);
    Profile::setMetric("sparse_region_surface_cells_contoured",
                       result.stats.cellsContoured);
    Profile::setMetric("sparse_region_surface_vertex_samples",
                       result.stats.vertexSamples);
    Profile::setMetric("sparse_region_surface_depth_calls",
                       result.stats.depthCalls);
    Profile::setMetric("sparse_region_surface_toolsweep_depth_calls",
                       result.stats.toolsweepDepthCalls);
    Profile::setMetric("sparse_region_surface_edge_checks",
                       result.stats.edgeChecks);
    Profile::setMetric("sparse_region_surface_edge_intersections",
                       result.stats.edgeIntersections);
    Profile::setMetric("sparse_region_surface_fallback_used",
                       result.rendererName == "baseline-full-render-fallback" ?
                       1 : 0);

    return result;
  }


  StitchResult stitchStockSurfaceInternal
  (const RegionPlan &regionPlan,
   const OwnershipBoundaryPlan *ownershipBoundary,
   const Simulation &regionSurfaceSim) {
    if (regionSurfaceSim.surface.isNull())
      THROW("Region surface artifact has no surface.");

    if (ownershipBoundary) {
      if (ownershipBoundary->xyBins != regionPlan.xyBins)
        THROW("Ownership boundary xy-bins do not match region plan.");
      if (ownershipBoundary->haloCells != regionPlan.haloCells)
        THROW("Ownership boundary halo-cells do not match region plan.");
      if (ownershipBoundary->stockBounds != regionPlan.stockBounds)
        THROW("Ownership boundary stock bounds do not match region plan.");
    }

    StitchResult result;
    SmartPointer<TriangleSurface> analyticSurface =
      createAnalyticStockPatches(regionPlan, ownershipBoundary,
                                 regionSurfaceSim.resolution,
                                  result.analyticTopPatches,
                                  result.analyticTopBoundaryVertices,
                                  result.analyticTopTriangles,
                                  result.analyticTopUnsupportedHoleLoops,
                                  result.analyticTopOwnershipRejected,
                                  result.analyticTopAreaStats,
                                  result.analyticTopBoundaryStats,
                                  result.analyticTopPatchSource,
                                 result.analyticBottomPatches,
                                 result.analyticBottomBoundaryVertices,
                                 result.analyticBottomAreaStats,
                                 result.analyticBottomPatchSource,
                                 result.analyticTransitionPatches,
                                 result.analyticTransitionAreaStats,
                                 result.analyticSidePatches,
                                 result.analyticSideAreaStats, true, true);
    SmartPointer<CompositeSurface> composite = new CompositeSurface;
    composite->add(regionSurfaceSim.surface);
    composite->add(analyticSurface);
    result.surface = composite->consolidate();
    result.topologyTolerance =
      max(1e-6, regionSurfaceSim.resolution * 0.005);
    result.topology =
      validateSurfaceTopology(*result.surface, result.topologyTolerance);
    result.inputMCTriangles = regionSurfaceSim.surface->getTriangleCount();
    result.analyticTriangles = analyticSurface->getTriangleCount();

    if (!result.topology.accepted()) {
      uint64_t altTopPatches = 0;
      uint64_t altTopBoundaryVertices = 0;
      uint64_t altTopTriangles = 0;
      uint64_t altTopUnsupportedHoleLoops = 0;
      uint64_t altTopOwnershipRejected = 0;
      PatchAreaStats altTopAreaStats;
      PatchBoundaryStats altTopBoundaryStats;
      string altTopPatchSource;
      uint64_t altBottomPatches = 0;
      uint64_t altBottomBoundaryVertices = 0;
      PatchAreaStats altBottomAreaStats;
      string altBottomPatchSource;
      uint64_t altTransitionPatches = 0;
      PatchAreaStats altTransitionAreaStats;
      uint64_t altSidePatches = 0;
      PatchAreaStats altSideAreaStats;

      SmartPointer<TriangleSurface> altAnalyticSurface =
        createAnalyticStockPatches(regionPlan, ownershipBoundary,
                                   regionSurfaceSim.resolution,
                                   altTopPatches,
                                    altTopBoundaryVertices,
                                    altTopTriangles,
                                    altTopUnsupportedHoleLoops,
                                    altTopOwnershipRejected,
                                    altTopAreaStats,
                                    altTopBoundaryStats,
                                    altTopPatchSource,
                                   altBottomPatches,
                                   altBottomBoundaryVertices,
                                   altBottomAreaStats,
                                   altBottomPatchSource,
                                   altTransitionPatches,
                                   altTransitionAreaStats,
                                   altSidePatches, altSideAreaStats,
                                   true, false);
      SmartPointer<CompositeSurface> altComposite = new CompositeSurface;
      altComposite->add(regionSurfaceSim.surface);
      altComposite->add(altAnalyticSurface);
      SmartPointer<Surface> altSurface = altComposite->consolidate();
      SurfaceTopologyReport altTopology =
        validateSurfaceTopology(*altSurface, result.topologyTolerance);
      result.analyticRenderGridSnapCandidateBoundaryEdges =
        altTopology.boundaryEdges;
      result.analyticRenderGridSnapCandidateNonManifoldEdges =
        altTopology.nonManifoldEdges;
      result.analyticRenderGridSnapCandidateMisorientedEdges =
        altTopology.misorientedEdges;
      result.analyticRenderGridSnapCandidateDegenerateTriangles =
        altTopology.degenerateTriangles;

      bool improved =
        (altTopology.accepted() && !result.topology.accepted()) ||
        (altTopology.boundaryEdges < result.topology.boundaryEdges &&
         altTopology.nonManifoldEdges <= result.topology.nonManifoldEdges &&
         altTopology.misorientedEdges <= result.topology.misorientedEdges &&
         altTopology.degenerateTriangles <=
         result.topology.degenerateTriangles &&
         altTopology.duplicateTriangles <=
         result.topology.duplicateTriangles);

      if (improved) {
        result.surface = altSurface;
        result.topology = altTopology;
        result.analyticTopPatches = altTopPatches;
        result.analyticTopBoundaryVertices = altTopBoundaryVertices;
        result.analyticTopTriangles = altTopTriangles;
        result.analyticTopUnsupportedHoleLoops =
          altTopUnsupportedHoleLoops;
        result.analyticTopOwnershipRejected = altTopOwnershipRejected;
        result.analyticTopAreaStats = altTopAreaStats;
        result.analyticTopBoundaryStats = altTopBoundaryStats;
        result.analyticTopPatchSource = altTopPatchSource;
        result.analyticBottomPatches = altBottomPatches;
        result.analyticBottomBoundaryVertices = altBottomBoundaryVertices;
        result.analyticBottomAreaStats = altBottomAreaStats;
        result.analyticBottomPatchSource = altBottomPatchSource;
        result.analyticTransitionPatches = altTransitionPatches;
        result.analyticTransitionAreaStats = altTransitionAreaStats;
        result.analyticSidePatches = altSidePatches;
        result.analyticSideAreaStats = altSideAreaStats;
        result.analyticTriangles = altAnalyticSurface->getTriangleCount();
        result.analyticRenderGridSnapUsed = 0;
      }
    }

    SmartPointer<TriangleSurface> primaryClosureSurface;
    uint64_t primaryClosurePatches = 0;
    uint64_t primaryClosureTriangles = 0;
    uint64_t primaryClosureUnsupportedLoops = 0;
    PatchAreaStats primaryClosureAreaStats;
    PatchBoundaryStats primaryClosureBoundaryStats;

    if (result.topology.boundaryEdges &&
        !result.topology.nonManifoldEdges &&
        !result.topology.misorientedEdges) {
      primaryClosureSurface =
        createPlanarBoundaryClosures
        (*result.surface, regionPlan, result.topologyTolerance,
         regionSurfaceSim.resolution, false,
         primaryClosurePatches, primaryClosureTriangles,
         primaryClosureUnsupportedLoops, primaryClosureAreaStats,
         primaryClosureBoundaryStats);
      result.analyticClosureAreaStats.add(primaryClosureAreaStats);
      result.analyticClosureBoundaryStats.add(primaryClosureBoundaryStats);
      result.analyticClosureUnsupportedLoops =
        primaryClosureUnsupportedLoops;

      if (!primaryClosureSurface.isNull() &&
          primaryClosureSurface->getTriangleCount()) {
        SmartPointer<CompositeSurface> closureComposite =
          new CompositeSurface;
        closureComposite->add(result.surface);
        closureComposite->add(primaryClosureSurface);
        SmartPointer<Surface> candidateSurface =
          closureComposite->consolidate();
        SurfaceTopologyReport candidateTopology =
          validateSurfaceTopology(*candidateSurface, result.topologyTolerance);
        result.analyticClosureCandidateBoundaryEdges =
          candidateTopology.boundaryEdges;
        result.analyticClosureCandidateNonManifoldEdges =
          candidateTopology.nonManifoldEdges;
        result.analyticClosureCandidateMisorientedEdges =
          candidateTopology.misorientedEdges;
        result.analyticClosureCandidateDegenerateTriangles =
          candidateTopology.degenerateTriangles;

        bool improved = candidateTopology.boundaryEdges <
          result.topology.boundaryEdges &&
          candidateTopology.nonManifoldEdges <=
          result.topology.nonManifoldEdges &&
          candidateTopology.misorientedEdges <=
          result.topology.misorientedEdges &&
          candidateTopology.degenerateTriangles <=
          result.topology.degenerateTriangles &&
          candidateTopology.duplicateTriangles <=
          result.topology.duplicateTriangles;

        if (improved) {
          result.surface = candidateSurface;
          result.topology = candidateTopology;
          result.analyticClosurePatches = primaryClosurePatches;
          result.analyticClosureTriangles = primaryClosureTriangles;
          result.analyticTriangles +=
            primaryClosureSurface->getTriangleCount();

        } else result.analyticClosureRejected = 1;
      }
    }

    if (!result.topology.accepted() && result.analyticTransitionPatches &&
        !primaryClosureSurface.isNull() &&
        primaryClosureSurface->getTriangleCount()) {
      uint64_t altTopPatches = 0;
      uint64_t altTopBoundaryVertices = 0;
      uint64_t altTopTriangles = 0;
      uint64_t altTopUnsupportedHoleLoops = 0;
      uint64_t altTopOwnershipRejected = 0;
      PatchAreaStats altTopAreaStats;
      PatchBoundaryStats altTopBoundaryStats;
      string altTopPatchSource;
      uint64_t altBottomPatches = 0;
      uint64_t altBottomBoundaryVertices = 0;
      PatchAreaStats altBottomAreaStats;
      string altBottomPatchSource;
      uint64_t altTransitionPatches = 0;
      PatchAreaStats altTransitionAreaStats;
      uint64_t altSidePatches = 0;
      PatchAreaStats altSideAreaStats;

      SmartPointer<TriangleSurface> altAnalyticSurface =
        createAnalyticStockPatches(regionPlan, ownershipBoundary,
                                   regionSurfaceSim.resolution,
                                   altTopPatches,
                                    altTopBoundaryVertices,
                                    altTopTriangles,
                                    altTopUnsupportedHoleLoops,
                                    altTopOwnershipRejected,
                                    altTopAreaStats,
                                    altTopBoundaryStats,
                                    altTopPatchSource,
                                   altBottomPatches,
                                   altBottomBoundaryVertices,
                                   altBottomAreaStats,
                                   altBottomPatchSource,
                                   altTransitionPatches,
                                   altTransitionAreaStats,
                                   altSidePatches, altSideAreaStats, false,
                                   result.analyticRenderGridSnapUsed != 0);
      SmartPointer<CompositeSurface> altComposite = new CompositeSurface;
      altComposite->add(regionSurfaceSim.surface);
      altComposite->add(altAnalyticSurface);
      SmartPointer<Surface> altSurface = altComposite->consolidate();
      SurfaceTopologyReport altTopology =
        validateSurfaceTopology(*altSurface, result.topologyTolerance);
      result.analyticTransitionReplacementCandidateBoundaryEdges =
        altTopology.boundaryEdges;
      result.analyticTransitionReplacementCandidateNonManifoldEdges =
        altTopology.nonManifoldEdges;
      result.analyticTransitionReplacementCandidateMisorientedEdges =
        altTopology.misorientedEdges;
      result.analyticTransitionReplacementCandidateDegenerateTriangles =
        altTopology.degenerateTriangles;

      bool improved = altTopology.boundaryEdges <
        result.topology.boundaryEdges &&
        altTopology.nonManifoldEdges <=
        result.topology.nonManifoldEdges &&
        altTopology.misorientedEdges <=
        result.topology.misorientedEdges &&
        altTopology.degenerateTriangles <=
        result.topology.degenerateTriangles &&
        altTopology.duplicateTriangles <= result.topology.duplicateTriangles;

      if (improved) {
        result.surface = altSurface;
        result.topology = altTopology;
        result.analyticTopPatches = altTopPatches;
        result.analyticTopBoundaryVertices = altTopBoundaryVertices;
        result.analyticTopTriangles = altTopTriangles;
        result.analyticTopUnsupportedHoleLoops =
          altTopUnsupportedHoleLoops;
        result.analyticTopOwnershipRejected = altTopOwnershipRejected;
        result.analyticTopAreaStats = altTopAreaStats;
        result.analyticTopBoundaryStats = altTopBoundaryStats;
        result.analyticTopPatchSource = altTopPatchSource;
        result.analyticBottomPatches = altBottomPatches;
        result.analyticBottomBoundaryVertices =
          altBottomBoundaryVertices;
        result.analyticBottomAreaStats = altBottomAreaStats;
        result.analyticBottomPatchSource = altBottomPatchSource;
        result.analyticTransitionPatches = altTransitionPatches;
        result.analyticTransitionAreaStats = altTransitionAreaStats;
        result.analyticSidePatches = altSidePatches;
        result.analyticSideAreaStats = altSideAreaStats;
        result.analyticClosurePatches = 0;
        result.analyticClosureTriangles = 0;
        result.analyticClosureUnsupportedLoops =
          primaryClosureUnsupportedLoops;
        result.analyticClosureRejected = 0;
        result.analyticTriangles = altAnalyticSurface->getTriangleCount();

      } else {
        SmartPointer<CompositeSurface> closureComposite =
          new CompositeSurface;
        closureComposite->add(altSurface);
        closureComposite->add(primaryClosureSurface);
        SmartPointer<Surface> candidateSurface =
          closureComposite->consolidate();
        SurfaceTopologyReport candidateTopology =
          validateSurfaceTopology(*candidateSurface, result.topologyTolerance);
        result.analyticTransitionReplacementCandidateBoundaryEdges =
          candidateTopology.boundaryEdges;
        result.analyticTransitionReplacementCandidateNonManifoldEdges =
          candidateTopology.nonManifoldEdges;
        result.analyticTransitionReplacementCandidateMisorientedEdges =
          candidateTopology.misorientedEdges;
        result.analyticTransitionReplacementCandidateDegenerateTriangles =
          candidateTopology.degenerateTriangles;

        bool closureImproved = candidateTopology.boundaryEdges <
          result.topology.boundaryEdges &&
          candidateTopology.nonManifoldEdges <=
          result.topology.nonManifoldEdges &&
          candidateTopology.misorientedEdges <=
          result.topology.misorientedEdges &&
          candidateTopology.degenerateTriangles <=
          result.topology.degenerateTriangles &&
          candidateTopology.duplicateTriangles <=
          result.topology.duplicateTriangles;

        if (closureImproved) {
          result.surface = candidateSurface;
          result.topology = candidateTopology;
          result.analyticTopPatches = altTopPatches;
          result.analyticTopBoundaryVertices = altTopBoundaryVertices;
          result.analyticTopTriangles = altTopTriangles;
          result.analyticTopUnsupportedHoleLoops =
            altTopUnsupportedHoleLoops;
          result.analyticTopOwnershipRejected = altTopOwnershipRejected;
          result.analyticTopAreaStats = altTopAreaStats;
          result.analyticTopBoundaryStats = altTopBoundaryStats;
          result.analyticTopPatchSource = altTopPatchSource;
          result.analyticBottomPatches = altBottomPatches;
          result.analyticBottomBoundaryVertices =
            altBottomBoundaryVertices;
          result.analyticBottomAreaStats = altBottomAreaStats;
          result.analyticBottomPatchSource = altBottomPatchSource;
          result.analyticTransitionPatches =
            altTransitionPatches + primaryClosurePatches;
          result.analyticTransitionAreaStats = altTransitionAreaStats;
          result.analyticSidePatches = altSidePatches;
          result.analyticSideAreaStats = altSideAreaStats;
          result.analyticClosurePatches = primaryClosurePatches;
          result.analyticClosureTriangles = primaryClosureTriangles;
          result.analyticClosureUnsupportedLoops =
            primaryClosureUnsupportedLoops;
          result.analyticClosureRejected = 0;
          result.analyticClosureCandidateBoundaryEdges =
            candidateTopology.boundaryEdges;
          result.analyticClosureCandidateNonManifoldEdges =
            candidateTopology.nonManifoldEdges;
          result.analyticClosureCandidateMisorientedEdges =
            candidateTopology.misorientedEdges;
          result.analyticClosureCandidateDegenerateTriangles =
            candidateTopology.degenerateTriangles;
          result.analyticTriangles =
            altAnalyticSurface->getTriangleCount() +
            primaryClosureSurface->getTriangleCount();
        }
      }
    }

    if (!result.topology.accepted() && result.topology.boundaryEdges &&
        !result.topology.nonManifoldEdges &&
        !result.topology.misorientedEdges &&
        !result.analyticTransitionPatches) {
      uint64_t stockClosurePatches = 0;
      uint64_t stockClosureTriangles = 0;
      uint64_t stockClosureUnsupportedLoops = 0;
      PatchAreaStats stockClosureAreaStats;
      PatchBoundaryStats stockClosureBoundaryStats;
      SmartPointer<TriangleSurface> stockClosureSurface =
        createPlanarBoundaryClosures
        (*result.surface, regionPlan, result.topologyTolerance,
         regionSurfaceSim.resolution, true,
         stockClosurePatches, stockClosureTriangles,
         stockClosureUnsupportedLoops, stockClosureAreaStats,
         stockClosureBoundaryStats);
      result.analyticClosureAreaStats.add(stockClosureAreaStats);
      result.analyticClosureBoundaryStats.add(stockClosureBoundaryStats);

      if (!stockClosureSurface.isNull() &&
          stockClosureSurface->getTriangleCount()) {
        SmartPointer<CompositeSurface> closureComposite =
          new CompositeSurface;
        closureComposite->add(result.surface);
        closureComposite->add(stockClosureSurface);
        SmartPointer<Surface> candidateSurface =
          closureComposite->consolidate();
        SurfaceTopologyReport candidateTopology =
          validateSurfaceTopology(*candidateSurface, result.topologyTolerance);
        result.analyticClosureCandidateBoundaryEdges =
          candidateTopology.boundaryEdges;
        result.analyticClosureCandidateNonManifoldEdges =
          candidateTopology.nonManifoldEdges;
        result.analyticClosureCandidateMisorientedEdges =
          candidateTopology.misorientedEdges;
        result.analyticClosureCandidateDegenerateTriangles =
          candidateTopology.degenerateTriangles;

        bool improved = candidateTopology.boundaryEdges <
          result.topology.boundaryEdges &&
          candidateTopology.nonManifoldEdges <=
          result.topology.nonManifoldEdges &&
          candidateTopology.misorientedEdges <=
          result.topology.misorientedEdges &&
          candidateTopology.degenerateTriangles <=
          result.topology.degenerateTriangles &&
          candidateTopology.duplicateTriangles <=
          result.topology.duplicateTriangles;

        if (improved) {
          result.surface = candidateSurface;
          result.topology = candidateTopology;
          result.analyticClosurePatches += stockClosurePatches;
          result.analyticClosureTriangles += stockClosureTriangles;
          result.analyticClosureUnsupportedLoops =
            stockClosureUnsupportedLoops;
          result.analyticClosureRejected = 0;
          result.analyticTriangles +=
            stockClosureSurface->getTriangleCount();

        } else {
          result.analyticClosureUnsupportedLoops =
            stockClosureUnsupportedLoops;
          result.analyticClosureRejected = 1;
        }
      } else result.analyticClosureUnsupportedLoops =
        stockClosureUnsupportedLoops;
    }

    result.boundaryLoopSummary =
      summarizeBoundaryLoops(*result.surface, regionPlan,
                             result.topologyTolerance);
    if (result.accepted())
      result.reductionEligibilityValid = attachReductionEligibility(result);

    Profile::setMetric("sparse_stitch_input_mc_triangles",
                       result.inputMCTriangles);
    Profile::setMetric("sparse_stitch_analytic_triangles",
                       result.analyticTriangles);
    Profile::setMetric("sparse_stitch_analytic_top_patches",
                       result.analyticTopPatches);
    Profile::setMetric("sparse_stitch_analytic_top_boundary_vertices",
                       result.analyticTopBoundaryVertices);
    Profile::setMetric("sparse_stitch_analytic_top_triangles",
                       result.analyticTopTriangles);
    Profile::setMetric
      ("sparse_stitch_analytic_top_ownership_boundary_used",
       result.analyticTopPatchSource == "ownership-boundary-loops" ? 1 : 0);
    Profile::setMetric
      ("sparse_stitch_analytic_top_unsupported_hole_loops",
       result.analyticTopUnsupportedHoleLoops);
    Profile::setMetric("sparse_stitch_analytic_top_ownership_rejected",
                       result.analyticTopOwnershipRejected);
    Profile::setMetric("sparse_stitch_analytic_top_area_checks",
                       result.analyticTopAreaStats.checks);
    Profile::setMetric("sparse_stitch_analytic_top_area_failures",
                       result.analyticTopAreaStats.failures);
    Profile::setMetric("sparse_stitch_analytic_top_area_expected_scaled_1e6",
                       scaledAreaMetric
                       (result.analyticTopAreaStats.expectedArea));
    Profile::setMetric("sparse_stitch_analytic_top_area_triangles_scaled_1e6",
                       scaledAreaMetric
                       (result.analyticTopAreaStats.triangleArea));
    Profile::setMetric("sparse_stitch_analytic_top_area_max_error_scaled_1e6",
                       scaledAreaMetric
                       (result.analyticTopAreaStats.maxError));
    Profile::setMetric("sparse_stitch_analytic_top_boundary_checks",
                       result.analyticTopBoundaryStats.checks);
    Profile::setMetric("sparse_stitch_analytic_top_boundary_failures",
                       result.analyticTopBoundaryStats.failures);
    Profile::setMetric("sparse_stitch_analytic_top_boundary_expected_edges",
                       result.analyticTopBoundaryStats.expectedEdges);
    Profile::setMetric("sparse_stitch_analytic_top_boundary_emitted_edges",
                       result.analyticTopBoundaryStats.emittedEdges);
    Profile::setMetric("sparse_stitch_analytic_top_boundary_mismatched_edges",
                       result.analyticTopBoundaryStats.mismatchedEdges);
    Profile::setMetric
      ("sparse_stitch_analytic_top_boundary_invalid_incidence_edges",
       result.analyticTopBoundaryStats.invalidIncidenceEdges);
    Profile::setMetric("sparse_stitch_analytic_bottom_patches",
                       result.analyticBottomPatches);
    Profile::setMetric("sparse_stitch_analytic_bottom_boundary_vertices",
                       result.analyticBottomBoundaryVertices);
    Profile::setMetric("sparse_stitch_analytic_bottom_area_checks",
                       result.analyticBottomAreaStats.checks);
    Profile::setMetric("sparse_stitch_analytic_bottom_area_failures",
                       result.analyticBottomAreaStats.failures);
    Profile::setMetric
      ("sparse_stitch_analytic_bottom_area_expected_scaled_1e6",
       scaledAreaMetric(result.analyticBottomAreaStats.expectedArea));
    Profile::setMetric
      ("sparse_stitch_analytic_bottom_area_triangles_scaled_1e6",
       scaledAreaMetric(result.analyticBottomAreaStats.triangleArea));
    Profile::setMetric
      ("sparse_stitch_analytic_bottom_area_max_error_scaled_1e6",
       scaledAreaMetric(result.analyticBottomAreaStats.maxError));
    Profile::setMetric
      ("sparse_stitch_analytic_bottom_full_stock_used",
       result.analyticBottomPatchSource == "full-stock-bottom" ? 1 : 0);
    Profile::setMetric("sparse_stitch_analytic_transition_patches",
                       result.analyticTransitionPatches);
    Profile::setMetric("sparse_stitch_analytic_transition_area_checks",
                       result.analyticTransitionAreaStats.checks);
    Profile::setMetric("sparse_stitch_analytic_transition_area_failures",
                       result.analyticTransitionAreaStats.failures);
    Profile::setMetric
      ("sparse_stitch_analytic_transition_area_expected_scaled_1e6",
       scaledAreaMetric(result.analyticTransitionAreaStats.expectedArea));
    Profile::setMetric
      ("sparse_stitch_analytic_transition_area_triangles_scaled_1e6",
       scaledAreaMetric(result.analyticTransitionAreaStats.triangleArea));
    Profile::setMetric
      ("sparse_stitch_analytic_transition_area_max_error_scaled_1e6",
       scaledAreaMetric(result.analyticTransitionAreaStats.maxError));
    Profile::setMetric("sparse_stitch_analytic_side_patches",
                       result.analyticSidePatches);
    Profile::setMetric("sparse_stitch_analytic_side_area_checks",
                       result.analyticSideAreaStats.checks);
    Profile::setMetric("sparse_stitch_analytic_side_area_failures",
                       result.analyticSideAreaStats.failures);
    Profile::setMetric
      ("sparse_stitch_analytic_side_area_expected_scaled_1e6",
       scaledAreaMetric(result.analyticSideAreaStats.expectedArea));
    Profile::setMetric
      ("sparse_stitch_analytic_side_area_triangles_scaled_1e6",
       scaledAreaMetric(result.analyticSideAreaStats.triangleArea));
    Profile::setMetric
      ("sparse_stitch_analytic_side_area_max_error_scaled_1e6",
       scaledAreaMetric(result.analyticSideAreaStats.maxError));
    Profile::setMetric("sparse_stitch_analytic_closure_patches",
                       result.analyticClosurePatches);
    Profile::setMetric("sparse_stitch_analytic_closure_triangles",
                       result.analyticClosureTriangles);
    Profile::setMetric("sparse_stitch_analytic_closure_area_checks",
                       result.analyticClosureAreaStats.checks);
    Profile::setMetric("sparse_stitch_analytic_closure_area_failures",
                       result.analyticClosureAreaStats.failures);
    Profile::setMetric
      ("sparse_stitch_analytic_closure_area_expected_scaled_1e6",
       scaledAreaMetric(result.analyticClosureAreaStats.expectedArea));
    Profile::setMetric
      ("sparse_stitch_analytic_closure_area_triangles_scaled_1e6",
       scaledAreaMetric(result.analyticClosureAreaStats.triangleArea));
    Profile::setMetric
      ("sparse_stitch_analytic_closure_area_max_error_scaled_1e6",
       scaledAreaMetric(result.analyticClosureAreaStats.maxError));
    Profile::setMetric("sparse_stitch_analytic_closure_boundary_checks",
                       result.analyticClosureBoundaryStats.checks);
    Profile::setMetric("sparse_stitch_analytic_closure_boundary_failures",
                       result.analyticClosureBoundaryStats.failures);
    Profile::setMetric
      ("sparse_stitch_analytic_closure_boundary_expected_edges",
       result.analyticClosureBoundaryStats.expectedEdges);
    Profile::setMetric
      ("sparse_stitch_analytic_closure_boundary_emitted_edges",
       result.analyticClosureBoundaryStats.emittedEdges);
    Profile::setMetric
      ("sparse_stitch_analytic_closure_boundary_mismatched_edges",
       result.analyticClosureBoundaryStats.mismatchedEdges);
    Profile::setMetric
      ("sparse_stitch_analytic_closure_boundary_invalid_incidence_edges",
       result.analyticClosureBoundaryStats.invalidIncidenceEdges);
    Profile::setMetric
      ("sparse_stitch_analytic_closure_unsupported_loops",
       result.analyticClosureUnsupportedLoops);
    Profile::setMetric("sparse_stitch_analytic_closure_rejected",
                       result.analyticClosureRejected);
    Profile::setMetric
      ("sparse_stitch_analytic_closure_candidate_boundary_edges",
       result.analyticClosureCandidateBoundaryEdges);
    Profile::setMetric
      ("sparse_stitch_analytic_closure_candidate_nonmanifold_edges",
       result.analyticClosureCandidateNonManifoldEdges);
    Profile::setMetric
      ("sparse_stitch_analytic_closure_candidate_misoriented_edges",
       result.analyticClosureCandidateMisorientedEdges);
    Profile::setMetric
      ("sparse_stitch_analytic_closure_candidate_degenerate_triangles",
       result.analyticClosureCandidateDegenerateTriangles);
    Profile::setMetric
      ("sparse_stitch_analytic_transition_replacement_candidate_boundary_edges",
       result.analyticTransitionReplacementCandidateBoundaryEdges);
    Profile::setMetric
      ("sparse_stitch_analytic_transition_replacement_candidate_nonmanifold_edges",
       result.analyticTransitionReplacementCandidateNonManifoldEdges);
    Profile::setMetric
      ("sparse_stitch_analytic_transition_replacement_candidate_misoriented_edges",
       result.analyticTransitionReplacementCandidateMisorientedEdges);
    Profile::setMetric
      ("sparse_stitch_analytic_transition_replacement_candidate_degenerate_triangles",
       result.analyticTransitionReplacementCandidateDegenerateTriangles);
    Profile::setMetric("sparse_stitch_analytic_render_grid_snap_used",
                       result.analyticRenderGridSnapUsed);
    Profile::setMetric
      ("sparse_stitch_analytic_render_grid_snap_candidate_boundary_edges",
       result.analyticRenderGridSnapCandidateBoundaryEdges);
    Profile::setMetric
      ("sparse_stitch_analytic_render_grid_snap_candidate_nonmanifold_edges",
       result.analyticRenderGridSnapCandidateNonManifoldEdges);
    Profile::setMetric
      ("sparse_stitch_analytic_render_grid_snap_candidate_misoriented_edges",
       result.analyticRenderGridSnapCandidateMisorientedEdges);
    Profile::setMetric
      ("sparse_stitch_analytic_render_grid_snap_candidate_degenerate_triangles",
       result.analyticRenderGridSnapCandidateDegenerateTriangles);
    Profile::setMetric("sparse_stitch_boundary_loops",
                       result.boundaryLoopSummary.loops);
    Profile::setMetric("sparse_stitch_boundary_open_chains",
                       result.boundaryLoopSummary.openChains);
    Profile::setMetric("sparse_stitch_boundary_planar_loops",
                       result.boundaryLoopSummary.planarLoops);
    Profile::setMetric("sparse_stitch_boundary_nonplanar_loops",
                       result.boundaryLoopSummary.nonPlanarLoops);
    Profile::setMetric("sparse_stitch_boundary_horizontal_loops",
                       result.boundaryLoopSummary.horizontalLoops);
    Profile::setMetric("sparse_stitch_boundary_supported_closure_loops",
                       result.boundaryLoopSummary.supportedClosureLoops);
    Profile::setMetric("sparse_stitch_boundary_stock_border_loops",
                       result.boundaryLoopSummary.stockBorderLoops);
    Profile::setMetric("sparse_stitch_boundary_tile_line_loops",
                       result.boundaryLoopSummary.tileLineLoops);
    Profile::setMetric("sparse_stitch_boundary_render_boundary_loops",
                       result.boundaryLoopSummary.renderBoundaryLoops);
    Profile::setMetric
      ("sparse_stitch_boundary_nonplanar_render_boundary_loops",
       result.boundaryLoopSummary.nonPlanarRenderBoundaryLoops);
    Profile::setMetric("sparse_stitch_output_triangles",
                       result.surface->getTriangleCount());
    Profile::setMetric("sparse_stitch_plan_active_regions",
                       regionPlan.activeRegions.size());
    Profile::setMetric("sparse_stitch_plan_analytic_regions",
                       regionPlan.analyticRegions.size());
    Profile::setMetric
      ("sparse_stitch_ownership_boundary_supplied",
       ownershipBoundary ? 1 : 0);
    if (ownershipBoundary) {
      Profile::setMetric("sparse_stitch_ownership_boundary_loops",
                         ownershipBoundary->loops.size());
      Profile::setMetric("sparse_stitch_ownership_boundary_edges",
                         ownershipBoundary->boundaryEdges);
      Profile::setMetric("sparse_stitch_ownership_boundary_open_loops",
                         ownershipBoundary->openLoops);
      Profile::setMetric
        ("sparse_stitch_ownership_boundary_ambiguous_vertices",
         ownershipBoundary->ambiguousVertices);
      Profile::setMetric
        ("sparse_stitch_ownership_boundary_contracted_vertices",
         ownershipBoundary->contractedVertices);
    }
    Profile::setMetric("sparse_stitch_boundary_edges",
                       result.topology.boundaryEdges);
    Profile::setMetric("sparse_stitch_nonmanifold_edges",
                       result.topology.nonManifoldEdges);
    Profile::setMetric("sparse_stitch_misoriented_edges",
                       result.topology.misorientedEdges);
    Profile::setMetric("sparse_stitch_degenerate_triangles",
                       result.topology.degenerateTriangles);
    Profile::setMetric("sparse_stitch_duplicate_triangles",
                       result.topology.duplicateTriangles);
    Profile::setMetric("sparse_stitch_topology_accepted",
                       result.topology.accepted() ? 1 : 0);
    Profile::setMetric("sparse_stitch_geometry_accepted",
                       result.geometryAccepted() ? 1 : 0);
    Profile::setMetric("sparse_stitch_accepted",
                       result.accepted() ? 1 : 0);
    Profile::setMetric("sparse_stitch_reduction_origin_metadata_valid",
                       result.reductionEligibilityValid ? 1 : 0);
    Profile::setMetric("sparse_stitch_reduction_mc_reducible_triangles",
                       result.reductionMCReducibleTriangles);
    Profile::setMetric("sparse_stitch_reduction_mc_seam_locked_triangles",
                       result.reductionMCSeamLockedTriangles);
    Profile::setMetric("sparse_stitch_reduction_analytic_locked_triangles",
                       result.reductionAnalyticLockedTriangles);
    Profile::setMetric("sparse_stitch_reduction_unknown_locked_triangles",
                       result.reductionUnknownLockedTriangles);
    Profile::setMetric("sparse_stitch_reduction_locked_seam_vertices",
                       result.reductionLockedSeamVertices);
    Profile::setMetric("sparse_stitch_reduction_locked_seam_edges",
                       result.reductionLockedSeamEdges);

    return result;
  }

}


OwnershipBoundaryPlan SparseToolpath::planOwnershipBoundaries
(const RegionPlan &regionPlan) {
  OwnershipBoundaryPlan boundary = createOwnershipBoundaryPlan(regionPlan);

  Profile::setMetric("sparse_boundary_plan_loops", boundary.loops.size());
  Profile::setMetric("sparse_boundary_plan_active_tiles",
                     boundary.activeTiles);
  Profile::setMetric("sparse_boundary_plan_analytic_tiles",
                     boundary.analyticTiles);
  Profile::setMetric("sparse_boundary_plan_edges", boundary.boundaryEdges);
  Profile::setMetric("sparse_boundary_plan_open_loops", boundary.openLoops);
  Profile::setMetric("sparse_boundary_plan_ambiguous_vertices",
                     boundary.ambiguousVertices);
  Profile::setMetric("sparse_boundary_plan_raw_vertices",
                     boundary.rawVertices);
  Profile::setMetric("sparse_boundary_plan_contracted_vertices",
                     boundary.contractedVertices);

  return boundary;
}


void SparseToolpath::writeOwnershipBoundaryArtifact
(const Simulation &regionPlanSim, const RegionPlan &regionPlan,
 const ArtifactContract &inputContract, ostream &stream) {
  if (inputContract.regionPlanHash != computeRegionPlanHash(regionPlan))
    THROW("Region plan artifact contract does not match region plan data.");
  OwnershipBoundaryPlan boundary = planOwnershipBoundaries(regionPlan);
  ArtifactContract contract = inputContract;
  contract.ownershipBoundaryHash =
    computeOwnershipBoundaryHash(boundary);

  writeSimulationArtifact
    (stream, OWNERSHIP_BOUNDARY_ARTIFACT, regionPlanSim, contract,
     [&] (JSON::Sink &sink) {
      sink.beginInsert("ownership-boundary");
      boundary.write(sink);
    });
}


SmartPointer<Surface> SparseToolpath::renderRegionSurface
(const Simulation &toolpathSim, const RegionPlan &regionPlan,
 unsigned threads) {
  return renderRegionSurfaceInternal(toolpathSim, regionPlan, threads).surface;
}


SmartPointer<Surface> SparseToolpath::renderRegionSurfaceArtifact
(const Simulation &toolpathSim, const RegionPlan &regionPlan,
 const ArtifactContract &inputContract, unsigned threads, ostream &stream) {
  if (inputContract.regionPlanHash != computeRegionPlanHash(regionPlan))
    THROW("Sparse render inputs do not share a compatible contract.");
  RegionRenderResult result =
    renderRegionSurfaceInternal(toolpathSim, regionPlan, threads);
  unsigned renderThreads = threads ? threads : toolpathSim.threads;
  Simulation renderSim
    (toolpathSim.path, toolpathSim.planConf, 0, toolpathSim.workpiece,
     toolpathSim.resolution, toolpathSim.time, toolpathSim.mode,
     renderThreads, toolpathSim.toolSweepXYBins,
     toolpathSim.toolSweepXYZBins);
  renderSim.surface = result.surface;

  writeSimulationArtifact
    (stream, REGION_SURFACE_ARTIFACT, renderSim, inputContract,
     [&] (JSON::Sink &sink) {
      sink.beginInsert("region-surface");
      sink.beginDict();
      sink.insert("renderer", result.rendererName);
      sink.insert("surface-chunks", result.surfaceChunks);
      sink.insert("mc-owned-regions-rendered",
                  (uint64_t)(regionPlan.renderRegions.empty() ?
                             regionPlan.activeRegions.size() :
                             regionPlan.renderRegions.size()));
      sink.insert("ownership-active-regions",
                  (uint64_t)regionPlan.activeRegions.size());
      sink.insert("analytic-regions-carried",
                  (uint64_t)regionPlan.analyticRegions.size());
      sink.insert("planned-active-cells-est", regionPlan.activeCells);
      sink.insert("planned-skipped-cells-est", regionPlan.skippedCells);
      sink.insert("render-cells", result.renderCells);
      sink.insert("cells-visited", result.stats.cellsVisited);
      sink.insert("cells-culled", result.stats.cellsCulled);
      sink.insert("cells-contoured", result.stats.cellsContoured);
      sink.insert("vertex-samples", result.stats.vertexSamples);
      sink.insert("depth-calls", result.stats.depthCalls);
      sink.insert("toolsweep-depth-calls", result.stats.toolsweepDepthCalls);
      sink.insert("edge-checks", result.stats.edgeChecks);
      sink.insert("edge-intersections", result.stats.edgeIntersections);
      sink.insert("triangles", result.surface->getTriangleCount());
      writeBounds(sink, "surface-bounds", result.surface->getBounds());
      sink.endDict();
    });

  return result.surface;
}


void SparseToolpath::writeStitchedSurfaceArtifact
(const RegionPlan &regionPlan, const Simulation &regionSurfaceSim,
 const ArtifactContract &inputContract, ostream &stream) {
  writeStitchedSurfaceArtifact(regionPlan, 0, regionSurfaceSim,
                               inputContract, stream);
}


void SparseToolpath::writeStitchedSurfaceArtifact
(const RegionPlan &regionPlan,
 const OwnershipBoundaryPlan *ownershipBoundary,
 const Simulation &regionSurfaceSim, const ArtifactContract &inputContract,
 ostream &stream) {
  if (inputContract.regionPlanHash != computeRegionPlanHash(regionPlan))
    THROW("Sparse stitch inputs do not share a compatible contract.");
  if (ownershipBoundary &&
      inputContract.ownershipBoundaryHash !=
      computeOwnershipBoundaryHash(*ownershipBoundary))
    THROW("Sparse ownership boundary contract does not match boundary data.");
  StitchResult result =
    stitchStockSurfaceInternal(regionPlan, ownershipBoundary,
                               regionSurfaceSim);
  Simulation stitchedSim = regionSurfaceSim;
  stitchedSim.surface = result.surface;
  ArtifactContract contract = inputContract;
  contract.regionSurfaceHash = inputContract.simulationHash;

  TriangleSurface *triangleSurface =
    dynamic_cast<TriangleSurface *>(result.surface.get());
  if (result.accepted() &&
      (!triangleSurface || !triangleSurface->hasReductionEligibility() ||
       !triangleSurface->getReductionEligibility().validFor
         (triangleSurface->getVertices())))
    THROW("Accepted sparse stitched surface has no valid reduction "
          "eligibility metadata.");
  const ReductionEligibility *eligibility = result.accepted() ?
    &triangleSurface->getReductionEligibility() : 0;

  writeSimulationArtifact
    (stream, STITCHED_SURFACE_ARTIFACT, stitchedSim, contract,
     [&] (JSON::Sink &sink) {
      if (eligibility) {
      sink.beginInsert("reduction-eligibility");
      sink.beginDict();
      sink.insert("metadata-version", "1");
      sink.insert("lineage-region-surface-hash", contract.regionSurfaceHash);
      sink.insert("triangle-count", triangleSurface->getTriangleCount());
      sink.insert("quantization-tolerance",
                  eligibility->quantizationTolerance);
      sink.insert("binding-hash", eligibility->bindingHash);
      sink.insert("mc-reducible-triangles",
                  eligibility->count(REDUCTION_MC_REDUCIBLE));
      sink.insert("mc-seam-locked-triangles",
                  eligibility->count(REDUCTION_MC_SEAM_LOCKED));
      sink.insert("analytic-locked-triangles",
                  eligibility->count(REDUCTION_ANALYTIC_LOCKED));
      sink.insert("unknown-locked-triangles",
                  eligibility->count(REDUCTION_UNKNOWN_LOCKED));
      sink.insertList("triangle-origins");
      for (uint8_t origin: eligibility->triangleOrigins)
        sink.append((uint64_t)origin);
      sink.endList();
      sink.insertList("locked-seam-vertices");
      for (const ReductionLockedVertex &vertex:
           eligibility->lockedSeamVertices) {
        sink.appendDict(true);
        sink.insert("x", vertex.x);
        sink.insert("y", vertex.y);
        sink.insert("z", vertex.z);
        sink.endDict();
      }
      sink.endList();
      sink.insertList("locked-seam-edges");
      for (const ReductionLockedEdge &edge: eligibility->lockedSeamEdges) {
        sink.appendDict(true);
        for (unsigned endpoint = 0; endpoint < 2; endpoint++) {
          const ReductionLockedVertex &vertex = endpoint ? edge.b : edge.a;
          sink.beginInsert(endpoint ? "b" : "a");
          sink.beginDict();
          sink.insert("x", vertex.x);
          sink.insert("y", vertex.y);
          sink.insert("z", vertex.z);
          sink.endDict();
        }
        sink.endDict();
      }
      sink.endList();
      sink.endDict();
      }

      sink.beginInsert("stitch-stock");
      sink.beginDict();
      sink.insert("stitcher", "analytic-stock-patches-v1");
      sink.insert("mc-triangles", result.inputMCTriangles);
      sink.insert("analytic-triangles", result.analyticTriangles);
      sink.insert("analytic-top-source", result.analyticTopPatchSource);
      sink.insert("analytic-top-patches", result.analyticTopPatches);
      sink.insert("analytic-top-boundary-vertices",
                  result.analyticTopBoundaryVertices);
      sink.insert("analytic-top-triangles", result.analyticTopTriangles);
      sink.insert("analytic-top-unsupported-hole-loops",
                  result.analyticTopUnsupportedHoleLoops);
      sink.insert("analytic-top-ownership-rejected",
                  result.analyticTopOwnershipRejected);
      sink.insert("analytic-top-area-checks",
                  result.analyticTopAreaStats.checks);
      sink.insert("analytic-top-area-failures",
                  result.analyticTopAreaStats.failures);
      sink.insert("analytic-top-area-expected",
                  result.analyticTopAreaStats.expectedArea);
      sink.insert("analytic-top-area-triangles",
                  result.analyticTopAreaStats.triangleArea);
      sink.insert("analytic-top-area-max-error",
                  result.analyticTopAreaStats.maxError);
      sink.insert("analytic-top-boundary-checks",
                  result.analyticTopBoundaryStats.checks);
      sink.insert("analytic-top-boundary-failures",
                  result.analyticTopBoundaryStats.failures);
      sink.insert("analytic-top-boundary-expected-edges",
                  result.analyticTopBoundaryStats.expectedEdges);
      sink.insert("analytic-top-boundary-emitted-edges",
                  result.analyticTopBoundaryStats.emittedEdges);
      sink.insert("analytic-top-boundary-mismatched-edges",
                  result.analyticTopBoundaryStats.mismatchedEdges);
      sink.insert("analytic-top-boundary-invalid-incidence-edges",
                  result.analyticTopBoundaryStats.invalidIncidenceEdges);
      sink.insert("analytic-bottom-patches", result.analyticBottomPatches);
      sink.insert("analytic-bottom-source", result.analyticBottomPatchSource);
      sink.insert("analytic-bottom-boundary-vertices",
                  result.analyticBottomBoundaryVertices);
      sink.insert("analytic-bottom-area-checks",
                  result.analyticBottomAreaStats.checks);
      sink.insert("analytic-bottom-area-failures",
                  result.analyticBottomAreaStats.failures);
      sink.insert("analytic-bottom-area-expected",
                  result.analyticBottomAreaStats.expectedArea);
      sink.insert("analytic-bottom-area-triangles",
                  result.analyticBottomAreaStats.triangleArea);
      sink.insert("analytic-bottom-area-max-error",
                  result.analyticBottomAreaStats.maxError);
      sink.insert("analytic-transition-patches",
                  result.analyticTransitionPatches);
      sink.insert("analytic-transition-area-checks",
                  result.analyticTransitionAreaStats.checks);
      sink.insert("analytic-transition-area-failures",
                  result.analyticTransitionAreaStats.failures);
      sink.insert("analytic-transition-area-expected",
                  result.analyticTransitionAreaStats.expectedArea);
      sink.insert("analytic-transition-area-triangles",
                  result.analyticTransitionAreaStats.triangleArea);
      sink.insert("analytic-transition-area-max-error",
                  result.analyticTransitionAreaStats.maxError);
      sink.insert("analytic-side-patches", result.analyticSidePatches);
      sink.insert("analytic-side-area-checks",
                  result.analyticSideAreaStats.checks);
      sink.insert("analytic-side-area-failures",
                  result.analyticSideAreaStats.failures);
      sink.insert("analytic-side-area-expected",
                  result.analyticSideAreaStats.expectedArea);
      sink.insert("analytic-side-area-triangles",
                  result.analyticSideAreaStats.triangleArea);
      sink.insert("analytic-side-area-max-error",
                  result.analyticSideAreaStats.maxError);
      sink.insert("analytic-closure-patches",
                  result.analyticClosurePatches);
      sink.insert("analytic-closure-triangles",
                  result.analyticClosureTriangles);
      sink.insert("analytic-closure-area-checks",
                  result.analyticClosureAreaStats.checks);
      sink.insert("analytic-closure-area-failures",
                  result.analyticClosureAreaStats.failures);
      sink.insert("analytic-closure-area-expected",
                  result.analyticClosureAreaStats.expectedArea);
      sink.insert("analytic-closure-area-triangles",
                  result.analyticClosureAreaStats.triangleArea);
      sink.insert("analytic-closure-area-max-error",
                  result.analyticClosureAreaStats.maxError);
      sink.insert("analytic-closure-boundary-checks",
                  result.analyticClosureBoundaryStats.checks);
      sink.insert("analytic-closure-boundary-failures",
                  result.analyticClosureBoundaryStats.failures);
      sink.insert("analytic-closure-boundary-expected-edges",
                  result.analyticClosureBoundaryStats.expectedEdges);
      sink.insert("analytic-closure-boundary-emitted-edges",
                  result.analyticClosureBoundaryStats.emittedEdges);
      sink.insert("analytic-closure-boundary-mismatched-edges",
                  result.analyticClosureBoundaryStats.mismatchedEdges);
      sink.insert("analytic-closure-boundary-invalid-incidence-edges",
                  result.analyticClosureBoundaryStats.invalidIncidenceEdges);
      sink.insert("analytic-closure-unsupported-loops",
                  result.analyticClosureUnsupportedLoops);
      sink.insert("analytic-closure-rejected",
                  result.analyticClosureRejected);
      sink.insert("analytic-closure-candidate-boundary-edges",
                  result.analyticClosureCandidateBoundaryEdges);
      sink.insert("analytic-closure-candidate-nonmanifold-edges",
                  result.analyticClosureCandidateNonManifoldEdges);
      sink.insert("analytic-closure-candidate-misoriented-edges",
                  result.analyticClosureCandidateMisorientedEdges);
      sink.insert("analytic-closure-candidate-degenerate-triangles",
                  result.analyticClosureCandidateDegenerateTriangles);
      sink.insert
        ("analytic-transition-replacement-candidate-boundary-edges",
         result.analyticTransitionReplacementCandidateBoundaryEdges);
      sink.insert
        ("analytic-transition-replacement-candidate-nonmanifold-edges",
         result.analyticTransitionReplacementCandidateNonManifoldEdges);
      sink.insert
        ("analytic-transition-replacement-candidate-misoriented-edges",
         result.analyticTransitionReplacementCandidateMisorientedEdges);
      sink.insert
        ("analytic-transition-replacement-candidate-degenerate-triangles",
         result.analyticTransitionReplacementCandidateDegenerateTriangles);
      sink.insert("analytic-render-grid-snap-used",
                  result.analyticRenderGridSnapUsed);
      sink.insert
        ("analytic-render-grid-snap-candidate-boundary-edges",
         result.analyticRenderGridSnapCandidateBoundaryEdges);
      sink.insert
        ("analytic-render-grid-snap-candidate-nonmanifold-edges",
         result.analyticRenderGridSnapCandidateNonManifoldEdges);
      sink.insert
        ("analytic-render-grid-snap-candidate-misoriented-edges",
         result.analyticRenderGridSnapCandidateMisorientedEdges);
      sink.insert
        ("analytic-render-grid-snap-candidate-degenerate-triangles",
         result.analyticRenderGridSnapCandidateDegenerateTriangles);
      sink.insert("boundary-loops", result.boundaryLoopSummary.loops);
      sink.insert("boundary-open-chains",
                  result.boundaryLoopSummary.openChains);
      sink.insert("boundary-planar-loops",
                  result.boundaryLoopSummary.planarLoops);
      sink.insert("boundary-nonplanar-loops",
                  result.boundaryLoopSummary.nonPlanarLoops);
      sink.insert("boundary-horizontal-loops",
                  result.boundaryLoopSummary.horizontalLoops);
      sink.insert("boundary-supported-closure-loops",
                  result.boundaryLoopSummary.supportedClosureLoops);
      sink.insert("boundary-stock-border-loops",
                  result.boundaryLoopSummary.stockBorderLoops);
      sink.insert("boundary-tile-line-loops",
                  result.boundaryLoopSummary.tileLineLoops);
      sink.insert("boundary-render-boundary-loops",
                  result.boundaryLoopSummary.renderBoundaryLoops);
      sink.insert("boundary-nonplanar-render-boundary-loops",
                  result.boundaryLoopSummary.nonPlanarRenderBoundaryLoops);
      writeBoundaryLoopDetails(sink, result.boundaryLoopSummary);
      sink.insert("stitched-triangles", result.surface->getTriangleCount());
      sink.insert("planned-active-regions",
                  (uint64_t)regionPlan.activeRegions.size());
      sink.insert("planned-render-regions",
                  (uint64_t)regionPlan.renderRegions.size());
      sink.insert("planned-analytic-regions",
                  (uint64_t)regionPlan.analyticRegions.size());
      sink.insert("ownership-boundary-supplied", ownershipBoundary ? 1 : 0);
      if (ownershipBoundary) {
        sink.insert("ownership-boundary-planner",
                    ownershipBoundary->planner);
        sink.insert("ownership-boundary-loops",
                    (uint64_t)ownershipBoundary->loops.size());
        sink.insert("ownership-boundary-edges",
                    ownershipBoundary->boundaryEdges);
        sink.insert("ownership-boundary-open-loops",
                    ownershipBoundary->openLoops);
        sink.insert("ownership-boundary-ambiguous-vertices",
                    ownershipBoundary->ambiguousVertices);
        sink.insert("ownership-boundary-contracted-vertices",
                    ownershipBoundary->contractedVertices);
      }
      sink.insert("topology-tolerance", result.topologyTolerance);
      result.topology.write(sink, result.accepted());
      sink.endDict();
    });
}


SmartPointer<Surface> SparseToolpath::stitchStockSurface
(const RegionPlan &regionPlan,
 const OwnershipBoundaryPlan *ownershipBoundary,
 const Simulation &regionSurfaceSim) {
  return stitchStockSurfaceInternal(regionPlan, ownershipBoundary,
                                    regionSurfaceSim).surface;
}



SmartPointer<Surface> CAMotics::SparseToolpath::Internal::renderFullBaseline
(const Simulation &sim, unsigned threads) {
  return renderFullBaselineImpl(sim, threads);
}


CAMotics::SparseToolpath::Internal::SparseSurfaceResult
CAMotics::SparseToolpath::Internal::buildSparseCandidate
(const Simulation &sim, const RegionPlan &regionPlan,
 const OwnershipBoundaryPlan &boundaryPlan, unsigned threads) {
  RegionRenderResult renderResult;
  {
    Profile::Scope scope("sparse_region_surface");
    renderResult = renderRegionSurfaceInternal(sim, regionPlan, threads);
  }

  Simulation regionSurfaceSim = sim;
  regionSurfaceSim.surface = renderResult.surface;

  StitchResult stitchResult;
  {
    Profile::Scope scope("sparse_stitch_stock");
    stitchResult =
      stitchStockSurfaceInternal(regionPlan, &boundaryPlan, regionSurfaceSim);
  }

  SparseSurfaceResult result;
  result.surface = stitchResult.surface;
  result.topologyAccepted = stitchResult.topology.accepted();
  result.geometryAccepted = stitchResult.geometryAccepted();
  return result;
}
