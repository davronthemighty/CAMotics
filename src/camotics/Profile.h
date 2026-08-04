/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2022 Joseph Coffland
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#pragma once

#include <cstdint>
#include <string>


namespace CAMotics {
  enum class ProfileCounter {
    CELLS_VISITED,
    CELLS_CULLED,
    CELLS_CONTOURED,
    VERTEX_SAMPLES,
    VERTEX_CULLED,
    DEPTH_CALLS,
    EDGE_CHECKS,
    EDGE_INTERSECTIONS,
    LINEAR_INTERSECT_ITERATIONS,
    CONTOUR_GRID_EDGE_TRIANGLES,
    CONTOUR_CELL_CENTER_TRIANGLES,
    TOOLSWEEP_DEPTH_CALLS,
    TOOLSWEEP_COLLISION_CANDIDATES,
    TOOLSWEEP_SORTED_CANDIDATES,
    TOOLSWEEP_XY_BIN_QUERIES,
    TOOLSWEEP_XY_BIN_OUT_OF_BOUNDS,
    TOOLSWEEP_XY_BIN_REFS_SCANNED,
    TOOLSWEEP_XY_BIN_BBOX_HITS,
    TOOLSWEEP_XYZ_BIN_QUERIES,
    TOOLSWEEP_XYZ_BIN_OUT_OF_BOUNDS,
    TOOLSWEEP_XYZ_BIN_REFS_SCANNED,
    TOOLSWEEP_XYZ_BIN_BBOX_HITS,
    TOOLSWEEP_CANDIDATE_CALLS_0,
    TOOLSWEEP_CANDIDATE_CALLS_1,
    TOOLSWEEP_CANDIDATE_CALLS_2_9,
    TOOLSWEEP_CANDIDATE_CALLS_10_99,
    TOOLSWEEP_CANDIDATE_CALLS_100_999,
    TOOLSWEEP_CANDIDATE_CALLS_1000_PLUS,
    AABB_NODE_VISITS,
    AABB_LEAF_HITS,
    COUNT
  };


  struct ProfileCounters {
    uint64_t values[(unsigned)ProfileCounter::COUNT] = {};

    uint64_t get(ProfileCounter counter) const {
      return values[(unsigned)counter];
    }
  };


  struct ProfileRenderJob {
    std::string thread;
    double seconds = 0;
    uint64_t cellsVisited = 0;
    uint64_t cellsCulled = 0;
    uint64_t cellsContoured = 0;
    uint64_t triangles = 0;
    uint64_t vertexSamples = 0;
    uint64_t depthCalls = 0;
    uint64_t edgeChecks = 0;
    uint64_t edgeIntersections = 0;
    uint64_t linearIntersectIterations = 0;
    uint64_t contourGridEdgeTriangles = 0;
    uint64_t contourCellCenterTriangles = 0;
    uint64_t toolsweepDepthCalls = 0;
    uint64_t toolsweepCollisionCandidates = 0;
    uint64_t toolsweepSortedCandidates = 0;
    uint64_t toolsweepXYBinQueries = 0;
    uint64_t toolsweepXYBinOutOfBounds = 0;
    uint64_t toolsweepXYBinRefsScanned = 0;
    uint64_t toolsweepXYBinBBoxHits = 0;
    uint64_t toolsweepXYZBinQueries = 0;
    uint64_t toolsweepXYZBinOutOfBounds = 0;
    uint64_t toolsweepXYZBinRefsScanned = 0;
    uint64_t toolsweepXYZBinBBoxHits = 0;
    uint64_t aabbNodeVisits = 0;
    uint64_t aabbLeafHits = 0;
  };


  class Profile {
  public:
    class Scope {
      std::string name;
      double start = 0;
      bool active = false;

    public:
      Scope(const std::string &name);
      ~Scope();
    };

    static void start(const std::string &path);
    static bool isEnabled();
    static void setThreadCounterCapture(bool enabled);
    static void count(ProfileCounter counter, uint64_t value = 1);
    static ProfileCounters getThreadCounters();
    static void setMetric(const std::string &name, uint64_t value);
    static void setRealMetric(const std::string &name, double value);
    static void addPhase(const std::string &name, double seconds);
    static void addRenderJob(const ProfileRenderJob &job);
    static void write();
    static const char *counterName(ProfileCounter counter);
  };
}
