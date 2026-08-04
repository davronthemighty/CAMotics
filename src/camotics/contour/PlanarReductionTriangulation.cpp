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

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <unordered_map>
#include <unordered_set>

using namespace std;


namespace CAMotics {
namespace PlanarReductionInternal {
  double cross2(const Vec2 &a, const Vec2 &b, const Vec2 &c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  }


  bool orderBoundaryLoop(const vector<pair<uint32_t, uint32_t> > &edges,
                         vector<uint32_t> &loop) {
    vector<vector<uint32_t> > loops;
    if (!orderBoundaryLoops(edges, loops) || loops.size() != 1) return false;

    loop.swap(loops[0]);
    return true;
  }


  bool orderBoundaryLoops(const vector<pair<uint32_t, uint32_t> > &edges,
                          vector<vector<uint32_t> > &loops) {
    unordered_map<uint32_t, vector<uint32_t> > graph;
    for (const auto &edge: edges) {
      graph[edge.first].push_back(edge.second);
      graph[edge.second].push_back(edge.first);
    }

    if (graph.empty() || graph.size() != edges.size()) return false;
    for (const auto &entry: graph)
      if (entry.second.size() != 2) return false;

    unordered_set<uint32_t> visited;
    loops.clear();

    while (visited.size() < graph.size()) {
      uint32_t start = numeric_limits<uint32_t>::max();
      for (const auto &entry: graph)
        if (!visited.count(entry.first))
          start = std::min(start, entry.first);
      if (start == numeric_limits<uint32_t>::max()) return false;

      vector<uint32_t> loop;
      uint32_t previous = numeric_limits<uint32_t>::max();
      uint32_t current = start;

      while (true) {
        if (current != start && visited.count(current)) return false;
        loop.push_back(current);
        visited.insert(current);

        const vector<uint32_t> &neighbors = graph[current];
        uint32_t next = neighbors[0] == previous ? neighbors[1] : neighbors[0];

        previous = current;
        current = next;

        if (current == start) break;
        if (graph.size() < loop.size()) return false;
      }

      if (loop.size() < 3) return false;
      loops.push_back(std::move(loop));
    }

    return !loops.empty();
  }


  bool pointInPolygon(const Vec2 &point, const vector<uint32_t> &loop,
                      const vector<Vec3> &points, unsigned dropAxis) {
    bool inside = false;
    size_t count = loop.size();

    for (size_t i = 0, j = count - 1; i < count; j = i++) {
      Vec2 a = projectPoint(points[loop[i]], dropAxis);
      Vec2 b = projectPoint(points[loop[j]], dropAxis);

      bool crosses =
        ((a.y > point.y) != (b.y > point.y)) &&
        (point.x < (b.x - a.x) * (point.y - a.y) /
         (b.y - a.y) + a.x);
      if (crosses) inside = !inside;
    }

    return inside;
  }


  unsigned projectionDropAxis(const Vec3 &normal) {
    double x = fabs(normal.x);
    double y = fabs(normal.y);
    double z = fabs(normal.z);

    if (x >= y && x >= z) return 0;
    if (y >= x && y >= z) return 1;
    return 2;
  }


  Vec2 projectPoint(const Vec3 &point, unsigned dropAxis) {
    if (dropAxis == 0) return Vec2{point.y, point.z};
    if (dropAxis == 1) return Vec2{point.x, point.z};
    return Vec2{point.x, point.y};
  }


  double polygonArea2D(const vector<uint32_t> &loop,
                       const vector<Vec3> &points, unsigned dropAxis) {
    double area = 0;
    for (size_t i = 0; i < loop.size(); i++) {
      Vec2 a = projectPoint(points[loop[i]], dropAxis);
      Vec2 b = projectPoint(points[loop[(i + 1) % loop.size()]], dropAxis);
      area += a.x * b.y - b.x * a.y;
    }

    return area * 0.5;
  }


  bool pointInTriangleInclusive(const Vec2 &p, const Vec2 &a, const Vec2 &b,
                                const Vec2 &c, double eps) {
    double ab = cross2(a, b, p);
    double bc = cross2(b, c, p);
    double ca = cross2(c, a, p);
    return -eps <= ab && -eps <= bc && -eps <= ca;
  }


  bool pointsClose2D(const Vec2 &a, const Vec2 &b, double eps) {
    return fabs(a.x - b.x) <= eps && fabs(a.y - b.y) <= eps;
  }


  bool onSegment2D(const Vec2 &p, const Vec2 &a, const Vec2 &b,
                   double eps) {
    if (eps < fabs(cross2(a, b, p))) return false;

    return
      min(a.x, b.x) - eps <= p.x && p.x <= max(a.x, b.x) + eps &&
      min(a.y, b.y) - eps <= p.y && p.y <= max(a.y, b.y) + eps;
  }


  bool segmentsIntersect2D(const Vec2 &a, const Vec2 &b, const Vec2 &c,
                           const Vec2 &d, double eps) {
    double a1 = cross2(a, b, c);
    double a2 = cross2(a, b, d);
    double a3 = cross2(c, d, a);
    double a4 = cross2(c, d, b);

    if (((eps < a1 && a2 < -eps) || (a1 < -eps && eps < a2)) &&
        ((eps < a3 && a4 < -eps) || (a3 < -eps && eps < a4)))
      return true;

    return onSegment2D(c, a, b, eps) || onSegment2D(d, a, b, eps) ||
      onSegment2D(a, c, d, eps) || onSegment2D(b, c, d, eps);
  }


  bool bridgeCrossesBoundary(uint32_t aId, uint32_t bId,
                             const vector<vector<uint32_t> > &loops,
                             const vector<Vec3> &points, unsigned dropAxis,
                             double eps) {
    Vec2 a = projectPoint(points[aId], dropAxis);
    Vec2 b = projectPoint(points[bId], dropAxis);

    for (const auto &loop: loops)
      for (size_t i = 0; i < loop.size(); i++) {
        uint32_t cId = loop[i];
        uint32_t dId = loop[(i + 1) % loop.size()];
        if (aId == cId || aId == dId || bId == cId || bId == dId)
          continue;

        Vec2 c = projectPoint(points[cId], dropAxis);
        Vec2 d = projectPoint(points[dId], dropAxis);
        if (segmentsIntersect2D(a, b, c, d, eps)) return true;
      }

    return false;
  }


  bool triangulateLoop(const vector<uint32_t> &inputLoop,
                       const vector<Vec3> &points,
                       const Vec3 &seedNormal,
                       vector<array<uint32_t, 3> > &out,
                       bool *complexityRejected) {
    if (inputLoop.size() < 3) return false;

    unsigned dropAxis = projectionDropAxis(seedNormal);
    vector<uint32_t> loop = inputLoop;
    double area = polygonArea2D(loop, points, dropAxis);
    if (fabs(area) < 1e-18) return false;
    if (area < 0) reverse(loop.begin(), loop.end());

    out.clear();
    double scale = 0;
    for (uint32_t id: loop) {
      Vec2 p = projectPoint(points[id], dropAxis);
      scale = max(scale, max(fabs(p.x), fabs(p.y)));
    }

    const double eps = max(1e-18, scale * scale * 1e-14);

    // Ear clipping below performs an all-vertex containment scan and a vector
    // erase per triangle.  Stock faces commonly have weakly convex loops with
    // thousands of collinear grid vertices.  Preserve every boundary vertex
    // but clip their convex ears through linked indices in linear time.
    bool weaklyConvex = true;
    unsigned strictlyConvex = 0;
    for (size_t i = 0; i < loop.size(); i++) {
      Vec2 a = projectPoint
        (points[loop[(i + loop.size() - 1) % loop.size()]], dropAxis);
      Vec2 b = projectPoint(points[loop[i]], dropAxis);
      Vec2 c = projectPoint(points[loop[(i + 1) % loop.size()]], dropAxis);
      double turn = cross2(a, b, c);
      if (turn < -eps) {
        weaklyConvex = false;
        break;
      }
      if (eps < turn) strictlyConvex++;
    }

    if (weaklyConvex && 3 <= strictlyConvex) {
      size_t count = loop.size();
      vector<size_t> prev(count);
      vector<size_t> next(count);
      vector<uint8_t> active(count, 1);
      vector<uint8_t> queued(count);
      deque<size_t> ears;
      vector<array<uint32_t, 3> > candidate;
      candidate.reserve(count - 2);

      for (size_t i = 0; i < count; i++) {
        prev[i] = (i + count - 1) % count;
        next[i] = (i + 1) % count;
      }

      auto isStrictEar = [&] (size_t i) {
        if (!active[i]) return false;
        Vec2 a = projectPoint(points[loop[prev[i]]], dropAxis);
        Vec2 b = projectPoint(points[loop[i]], dropAxis);
        Vec2 c = projectPoint(points[loop[next[i]]], dropAxis);
        return eps < cross2(a, b, c);
      };
      auto enqueue = [&] (size_t i) {
        if (!queued[i] && isStrictEar(i)) {
          queued[i] = 1;
          ears.push_back(i);
        }
      };

      for (size_t i = 0; i < count; i++) enqueue(i);

      size_t remaining = count;
      while (3 < remaining && !ears.empty()) {
        size_t i = ears.front();
        ears.pop_front();
        queued[i] = 0;
        if (!isStrictEar(i)) continue;

        size_t p = prev[i];
        size_t n = next[i];
        array<uint32_t, 3> tri = {loop[p], loop[i], loop[n]};
        Vec3 normal = normalize(cross(points[tri[1]] - points[tri[0]],
                                      points[tri[2]] - points[tri[0]]));
        if (!length(normal)) break;
        candidate.push_back(tri);

        active[i] = 0;
        next[p] = n;
        prev[n] = p;
        remaining--;
        enqueue(p);
        enqueue(n);
      }

      if (remaining == 3) {
        size_t a = 0;
        while (a < count && !active[a]) a++;
        if (a < count) {
          size_t b = next[a];
          size_t c = next[b];
          array<uint32_t, 3> tri = {loop[a], loop[b], loop[c]};
          Vec3 normal = normalize(cross(points[tri[1]] - points[tri[0]],
                                        points[tri[2]] - points[tri[0]]));
          if (length(normal)) candidate.push_back(tri);
        }
      }

      if (candidate.size() + 2 == inputLoop.size()) {
        out.swap(candidate);
        return true;
      }
    }

    // General ear clipping is quadratic-to-cubic because it tests every
    // remaining point for every candidate ear.  Keep the original component
    // instead of allowing an optional reduction to become hours-scale.
    const size_t maxGeneralEarClipVertices = 512;
    if (complexityRejected && maxGeneralEarClipVertices < loop.size()) {
      *complexityRejected = true;
      return false;
    }

    unsigned guard = 0;

    while (3 < loop.size()) {
      bool clipped = false;
      size_t n = loop.size();

      for (size_t i = 0; i < n; i++) {
        size_t prevIndex = (i + n - 1) % n;
        size_t nextIndex = (i + 1) % n;
        uint32_t prev = loop[prevIndex];
        uint32_t curr = loop[i];
        uint32_t next = loop[nextIndex];
        if (prev == curr || curr == next || next == prev) continue;

        Vec2 a = projectPoint(points[prev], dropAxis);
        Vec2 b = projectPoint(points[curr], dropAxis);
        Vec2 c = projectPoint(points[next], dropAxis);
        if (cross2(a, b, c) <= eps) continue;

        bool blocked = false;
        for (size_t j = 0; j < n; j++) {
          if (j == prevIndex || j == i || j == nextIndex) continue;
          if (loop[j] == prev || loop[j] == curr || loop[j] == next)
            continue;
          Vec2 p = projectPoint(points[loop[j]], dropAxis);
          if (pointInTriangleInclusive(p, a, b, c, eps)) {
            blocked = true;
            break;
          }
        }

        if (blocked) continue;

        array<uint32_t, 3> tri = {prev, curr, next};
        Vec3 normal = normalize(cross(points[curr] - points[prev],
                                      points[next] - points[prev]));
        if (!length(normal)) return false;

        out.push_back(tri);
        loop.erase(loop.begin() + i);
        clipped = true;
        break;
      }

      if (!clipped) return false;
      if (++guard > inputLoop.size() * inputLoop.size()) return false;
    }

    if (loop[0] == loop[1] || loop[1] == loop[2] || loop[2] == loop[0])
      return false;

    array<uint32_t, 3> tri = {loop[0], loop[1], loop[2]};
    Vec3 normal = normalize(cross(points[tri[1]] - points[tri[0]],
                                  points[tri[2]] - points[tri[0]]));
    if (!length(normal)) return false;
    out.push_back(tri);

    return out.size() + 2 == inputLoop.size();
  }


  double triangleArea2D(const array<uint32_t, 3> &tri,
                        const vector<Vec3> &points, unsigned dropAxis) {
    Vec2 a = projectPoint(points[tri[0]], dropAxis);
    Vec2 b = projectPoint(points[tri[1]], dropAxis);
    Vec2 c = projectPoint(points[tri[2]], dropAxis);
    return fabs(cross2(a, b, c)) * 0.5;
  }


  double holeAwareTargetArea(const vector<uint32_t> &outerLoop,
                             const vector<vector<uint32_t> > &holeLoops,
                             const vector<Vec3> &points, unsigned dropAxis) {
    double area = fabs(polygonArea2D(outerLoop, points, dropAxis));
    for (const auto &loop: holeLoops)
      area -= fabs(polygonArea2D(loop, points, dropAxis));
    return area;
  }


  double loopProjectionScale(const vector<vector<uint32_t> > &loops,
                             const vector<Vec3> &points,
                             unsigned dropAxis) {
    double scale = 0;
    for (const auto &loop: loops)
      for (uint32_t id: loop) {
        Vec2 point = projectPoint(points[id], dropAxis);
        scale = max(scale, max(fabs(point.x), fabs(point.y)));
      }

    return scale;
  }


  bool loopVerticesInsideLoop(const vector<uint32_t> &inner,
                              const vector<uint32_t> &outer,
                              const vector<Vec3> &points,
                              unsigned dropAxis) {
    for (uint32_t id: inner) {
      Vec2 point = projectPoint(points[id], dropAxis);
      if (!pointInPolygon(point, outer, points, dropAxis)) return false;
    }

    return true;
  }


  bool loopsIntersect(const vector<uint32_t> &a,
                      const vector<uint32_t> &b,
                      const vector<Vec3> &points, unsigned dropAxis,
                      double eps) {
    for (size_t i = 0; i < a.size(); i++) {
      uint32_t a0Id = a[i];
      uint32_t a1Id = a[(i + 1) % a.size()];
      Vec2 a0 = projectPoint(points[a0Id], dropAxis);
      Vec2 a1 = projectPoint(points[a1Id], dropAxis);

      for (size_t j = 0; j < b.size(); j++) {
        uint32_t b0Id = b[j];
        uint32_t b1Id = b[(j + 1) % b.size()];
        if (a0Id == b0Id || a0Id == b1Id ||
            a1Id == b0Id || a1Id == b1Id)
          return true;

        Vec2 b0 = projectPoint(points[b0Id], dropAxis);
        Vec2 b1 = projectPoint(points[b1Id], dropAxis);
        if (segmentsIntersect2D(a0, a1, b0, b1, eps)) return true;
      }
    }

    return false;
  }


  bool findVisibleBridge(const vector<uint32_t> &currentLoop,
                         const vector<uint32_t> &holeLoop,
                         const vector<vector<uint32_t> > &allLoops,
                         const vector<Vec3> &points, unsigned dropAxis,
                         size_t &outerIndex, size_t &holeIndex) {
    double scale = 0;
    for (const auto &loop: allLoops)
      for (uint32_t id: loop) {
        Vec2 point = projectPoint(points[id], dropAxis);
        scale = max(scale, max(fabs(point.x), fabs(point.y)));
      }
    for (uint32_t id: currentLoop) {
      Vec2 point = projectPoint(points[id], dropAxis);
      scale = max(scale, max(fabs(point.x), fabs(point.y)));
    }

    double eps = max(1e-12, scale * scale * 1e-12);
    double bestDistance2 = numeric_limits<double>::max();
    bool found = false;

    vector<vector<uint32_t> > boundaryLoops = allLoops;
    boundaryLoops.push_back(currentLoop);

    for (size_t h = 0; h < holeLoop.size(); h++) {
      uint32_t holeVertex = holeLoop[h];
      Vec2 holePoint = projectPoint(points[holeVertex], dropAxis);

      for (size_t o = 0; o < currentLoop.size(); o++) {
        uint32_t outerVertex = currentLoop[o];
        if (outerVertex == holeVertex) continue;

        Vec2 outerPoint = projectPoint(points[outerVertex], dropAxis);
        if (pointsClose2D(holePoint, outerPoint, eps)) continue;
        if (bridgeCrossesBoundary(holeVertex, outerVertex, boundaryLoops,
                                  points, dropAxis, eps))
          continue;

        double dx = holePoint.x - outerPoint.x;
        double dy = holePoint.y - outerPoint.y;
        double distance2 = dx * dx + dy * dy;
        if (distance2 < bestDistance2) {
          bestDistance2 = distance2;
          outerIndex = o;
          holeIndex = h;
          found = true;
        }
      }
    }

    return found;
  }


  bool triangulateHoleAwareLoops
    (const vector<vector<uint32_t> > &loops, const vector<Vec3> &points,
     const Vec3 &seedNormal, vector<array<uint32_t, 3> > &out) {
    if (loops.size() < 2) return false;

    unsigned dropAxis = projectionDropAxis(seedNormal);
    double largestArea = 0;
    size_t outer = loops.size();
    vector<double> areas(loops.size());

    for (size_t i = 0; i < loops.size(); i++) {
      areas[i] = polygonArea2D(loops[i], points, dropAxis);
      double absArea = fabs(areas[i]);
      if (absArea < 1e-18) return false;
      if (largestArea < absArea) {
        largestArea = absArea;
        outer = i;
      }
    }

    if (outer == loops.size()) return false;

    vector<uint32_t> currentLoop = loops[outer];
    if (polygonArea2D(currentLoop, points, dropAxis) < 0)
      reverse(currentLoop.begin(), currentLoop.end());

    vector<vector<uint32_t> > holeLoops;
    holeLoops.reserve(loops.size() - 1);
    for (size_t i = 0; i < loops.size(); i++) {
      if (i == outer) continue;

      vector<uint32_t> hole = loops[i];
      if (0 < polygonArea2D(hole, points, dropAxis))
        reverse(hole.begin(), hole.end());
      holeLoops.push_back(std::move(hole));
    }

    sort(holeLoops.begin(), holeLoops.end(),
         [](const vector<uint32_t> &a, const vector<uint32_t> &b) {
           return b.size() < a.size();
         });

    for (const auto &hole: holeLoops) {
      size_t outerIndex = 0;
      size_t holeIndex = 0;
      if (!findVisibleBridge(currentLoop, hole, holeLoops, points, dropAxis,
                             outerIndex, holeIndex))
        return false;

      vector<uint32_t> merged;
      merged.reserve(currentLoop.size() + hole.size() + 2);
      merged.insert(merged.end(), currentLoop.begin(),
                    currentLoop.begin() + outerIndex + 1);
      merged.insert(merged.end(), hole.begin() + holeIndex, hole.end());
      merged.insert(merged.end(), hole.begin(), hole.begin() + holeIndex + 1);
      merged.push_back(currentLoop[outerIndex]);
      merged.insert(merged.end(), currentLoop.begin() + outerIndex + 1,
                    currentLoop.end());
      currentLoop.swap(merged);
    }

    if (!triangulateLoop(currentLoop, points, seedNormal, out)) return false;

    double targetArea = holeAwareTargetArea(loops[outer], holeLoops,
                                           points, dropAxis);
    double triangleArea = 0;
    for (const auto &tri: out)
      triangleArea += triangleArea2D(tri, points, dropAxis);

    double areaTolerance = max(1e-9, fabs(targetArea) * 1e-8);
    return fabs(triangleArea - targetArea) <= areaTolerance;
  }


  bool replacementBoundaryMatches
    (const vector<pair<uint32_t, uint32_t> > &boundaryEdges,
     const vector<array<uint32_t, 3> > &replacement) {
    unordered_set<EdgeKey, EdgeKeyHash> inputBoundary;
    for (const auto &edge: boundaryEdges)
      inputBoundary.insert(EdgeKey(edge.first, edge.second));

    unordered_map<EdgeKey, uint32_t, EdgeKeyHash> counts;
    for (const auto &tri: replacement) {
      for (unsigned slot = 0; slot < 3; slot++) {
        auto edge = getTriangleEdge(tri, slot);
        EdgeKey key(edge.first, edge.second);
        uint32_t &count = counts[key];
        count++;
        if (2 < count) return false;
      }
    }

    unordered_set<EdgeKey, EdgeKeyHash> outputBoundary;
    for (const auto &entry: counts)
      if (entry.second == 1) outputBoundary.insert(entry.first);

    if (inputBoundary.size() != outputBoundary.size()) return false;
    for (const auto &edge: inputBoundary)
      if (!outputBoundary.count(edge)) return false;

    return true;
  }


  bool replacementEdgeIncidenceOk
    (const vector<pair<uint32_t, uint32_t> > &boundaryEdges,
     const vector<array<uint32_t, 3> > &replacement) {
    unordered_set<EdgeKey, EdgeKeyHash> inputBoundary;
    for (const auto &edge: boundaryEdges)
      inputBoundary.insert(EdgeKey(edge.first, edge.second));

    unordered_map<EdgeKey, EdgeCount, EdgeKeyHash> counts;
    for (const auto &tri: replacement) {
      if (tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0])
        return false;

      for (unsigned slot = 0; slot < 3; slot++) {
        auto edge = getTriangleEdge(tri, slot);
        EdgeKey key(edge.first, edge.second);
        EdgeCount &count = counts[key];
        count.count++;
        if (isForwardEdge(edge, key)) count.forwardCount++;
      }
    }

    unordered_set<EdgeKey, EdgeKeyHash> outputBoundary;
    for (const auto &entry: counts) {
      const EdgeCount &count = entry.second;
      if (2 < count.count) return false;
      if (count.count == 2 &&
          (count.forwardCount == 0 || count.forwardCount == 2))
        return false;
      if (count.count == 1) outputBoundary.insert(entry.first);
    }

    if (inputBoundary.size() != outputBoundary.size()) return false;
    for (const auto &edge: inputBoundary)
      if (!outputBoundary.count(edge)) return false;

    return true;
  }


  uint64_t directedEdgeKey(uint32_t a, uint32_t b) {
    return ((uint64_t)a << 32) | b;
  }


  bool replacementBoundaryMatchesDirection
    (const vector<pair<uint32_t, uint32_t> > &boundaryEdges,
     const vector<array<uint32_t, 3> > &replacement) {
    unordered_set<uint64_t> inputBoundary;
    for (const auto &edge: boundaryEdges)
      inputBoundary.insert(directedEdgeKey(edge.first, edge.second));

    unordered_map<EdgeKey, uint32_t, EdgeKeyHash> counts;
    for (const auto &tri: replacement)
      for (unsigned slot = 0; slot < 3; slot++) {
        auto edge = getTriangleEdge(tri, slot);
        counts[EdgeKey(edge.first, edge.second)]++;
      }

    unordered_set<uint64_t> outputBoundary;
    for (const auto &tri: replacement)
      for (unsigned slot = 0; slot < 3; slot++) {
        auto edge = getTriangleEdge(tri, slot);
        if (counts[EdgeKey(edge.first, edge.second)] == 1)
          outputBoundary.insert(directedEdgeKey(edge.first, edge.second));
      }

    return outputBoundary == inputBoundary;
  }


  void flipReplacementTriangles
    (vector<array<uint32_t, 3> > &replacement) {
    for (auto &tri: replacement) swap(tri[1], tri[2]);
  }


  bool orientReplacementToBoundary
    (const vector<pair<uint32_t, uint32_t> > &boundaryEdges,
     vector<array<uint32_t, 3> > &replacement) {
    if (replacementBoundaryMatchesDirection(boundaryEdges, replacement))
      return true;

    flipReplacementTriangles(replacement);
    if (replacementBoundaryMatchesDirection(boundaryEdges, replacement))
      return true;

    flipReplacementTriangles(replacement);
    return false;
  }


  ReplacementCheck checkPhase1Replacement
    (const vector<pair<uint32_t, uint32_t> > &boundaryEdges,
     const vector<Vec3> &points, const Vec3 &seedNormal) {
    ReplacementCheck result;
    result.checked = true;

    vector<uint32_t> loop;
    if (!orderBoundaryLoop(boundaryEdges, loop)) return result;
    if (!triangulateLoop(loop, points, seedNormal, result.triangles,
                         &result.complexityRejected))
      return result;
    if (!replacementBoundaryMatches(boundaryEdges, result.triangles))
      return result;
    if (!orientReplacementToBoundary(boundaryEdges, result.triangles))
      return result;
    result.edgeIncidenceChecked = true;
    result.edgeIncidenceOk =
      replacementEdgeIncidenceOk(boundaryEdges, result.triangles);
    if (!result.edgeIncidenceOk)
      return result;

    unsigned dropAxis = projectionDropAxis(seedNormal);
    double targetArea = fabs(polygonArea2D(loop, points, dropAxis));
    double triangleArea = 0;
    for (const auto &tri: result.triangles)
      triangleArea += triangleArea2D(tri, points, dropAxis);

    double areaTolerance = max(1e-9, targetArea * 1e-8);
    if (areaTolerance < fabs(triangleArea - targetArea)) return result;

    result.feasible = true;
    result.trianglesAfter = result.triangles.size();
    return result;
  }


  ReplacementCheck checkHoleAwareReplacement
    (const vector<pair<uint32_t, uint32_t> > &boundaryEdges,
     const vector<Vec3> &points, const Vec3 &seedNormal) {
    ReplacementCheck result;
    result.checked = true;

    vector<vector<uint32_t> > loops;
    if (!orderBoundaryLoops(boundaryEdges, loops) || loops.size() < 2)
      return result;

    unsigned dropAxis = projectionDropAxis(seedNormal);
    double largestArea = 0;
    size_t outer = loops.size();
    uint64_t vertices = 0;

    for (size_t i = 0; i < loops.size(); i++) {
      double area = fabs(polygonArea2D(loops[i], points, dropAxis));
      if (area < 1e-18) return result;
      if (largestArea < area) {
        largestArea = area;
        outer = i;
      }
      vertices += loops[i].size();
    }

    if (outer == loops.size()) return result;

    double scale = loopProjectionScale(loops, points, dropAxis);
    double eps = max(1e-12, scale * scale * 1e-12);

    for (size_t i = 0; i < loops.size(); i++) {
      if (i == outer) continue;

      if (!loopVerticesInsideLoop(loops[i], loops[outer], points, dropAxis) ||
          loopsIntersect(loops[i], loops[outer], points, dropAxis, eps))
        return result;

      for (size_t j = 0; j < loops.size(); j++) {
        if (i == j || j == outer) continue;
        if (loopVerticesInsideLoop(loops[i], loops[j], points, dropAxis) ||
            loopVerticesInsideLoop(loops[j], loops[i], points, dropAxis) ||
            loopsIntersect(loops[i], loops[j], points, dropAxis, eps))
          return result;
      }
    }

    result.estimateAvailable = true;
    result.trianglesAfter = vertices + 2 * (loops.size() - 1) - 2;

    vector<array<uint32_t, 3> > triangles;
    if (!triangulateHoleAwareLoops(loops, points, seedNormal, triangles))
      return result;
    if (!replacementBoundaryMatches(boundaryEdges, triangles)) return result;
    if (!orientReplacementToBoundary(boundaryEdges, triangles)) return result;

    result.edgeIncidenceChecked = true;
    result.edgeIncidenceOk =
      replacementEdgeIncidenceOk(boundaryEdges, triangles);
    if (!result.edgeIncidenceOk) return result;

    result.feasible = true;
    result.trianglesAfter = triangles.size();
    result.triangles.swap(triangles);

    return result;
  }



}
}
