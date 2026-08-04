/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2022 Joseph Coffland
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#include "Profile.h"

#include <cbang/time/Timer.h>

#include <atomic>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
#include <vector>

using namespace std;
using namespace cb;
using namespace CAMotics;


namespace {
  struct Phase {
    string name;
    double seconds = 0;
  };

  atomic<bool> enabled(false);
  string outputPath;
  mutex profileMutex;
  vector<Phase> phases;
  vector<ProfileRenderJob> renderJobs;
  map<string, uint64_t> metrics;
  map<string, double> realMetrics;

  uint64_t counters[(unsigned)ProfileCounter::COUNT];
  thread_local uint64_t localCounters[(unsigned)ProfileCounter::COUNT];
  thread_local bool localCountersDirty = false;
  thread_local bool localCounterCapture = false;


  string escapeJSON(const string &s) {
    ostringstream out;

    for (char c: s) {
      switch (c) {
      case '\\': out << "\\\\"; break;
      case '"':  out << "\\\""; break;
      case '\b': out << "\\b";  break;
      case '\f': out << "\\f";  break;
      case '\n': out << "\\n";  break;
      case '\r': out << "\\r";  break;
      case '\t': out << "\\t";  break;
      default:   out << c;      break;
      }
    }

    return out.str();
  }


  void flushLocalCounters() {
    if (!localCountersDirty) return;

    lock_guard<mutex> lock(profileMutex);
    for (unsigned i = 0; i < (unsigned)ProfileCounter::COUNT; i++) {
      counters[i] += localCounters[i];
      localCounters[i] = 0;
    }

    localCountersDirty = false;
  }
}


Profile::Scope::Scope(const string &name) : name(name) {
  active = Profile::isEnabled();
  if (active) start = Timer::now();
}


Profile::Scope::~Scope() {
  if (active) Profile::addPhase(name, Timer::now() - start);
}


void Profile::start(const string &path) {
  lock_guard<mutex> lock(profileMutex);

  outputPath = path;
  phases.clear();
  renderJobs.clear();
  metrics.clear();
  realMetrics.clear();
  for (auto &counter: counters) counter = 0;
  for (auto &counter: localCounters) counter = 0;
  localCountersDirty = false;
  enabled.store(!outputPath.empty(), memory_order_release);
}


bool Profile::isEnabled() {return enabled.load(memory_order_acquire);}


void Profile::setThreadCounterCapture(bool enabled) {
  localCounterCapture = enabled;
}


void Profile::count(ProfileCounter counter, uint64_t value) {
  if (!Profile::isEnabled() && !localCounterCapture) return;
  localCounters[(unsigned)counter] += value;
  localCountersDirty = true;
}


ProfileCounters Profile::getThreadCounters() {
  ProfileCounters snapshot;

  if (!Profile::isEnabled() && !localCounterCapture) return snapshot;

  for (unsigned i = 0; i < (unsigned)ProfileCounter::COUNT; i++)
    snapshot.values[i] = localCounters[i];

  return snapshot;
}


void Profile::setMetric(const string &name, uint64_t value) {
  if (!Profile::isEnabled()) return;

  flushLocalCounters();
  lock_guard<mutex> lock(profileMutex);
  realMetrics.erase(name);
  metrics[name] = value;
}


void Profile::setRealMetric(const string &name, double value) {
  if (!Profile::isEnabled()) return;

  flushLocalCounters();
  lock_guard<mutex> lock(profileMutex);
  metrics.erase(name);
  realMetrics[name] = value;
}


void Profile::addPhase(const string &name, double seconds) {
  if (!Profile::isEnabled()) return;

  flushLocalCounters();
  lock_guard<mutex> lock(profileMutex);
  phases.push_back({name, seconds});
}


void Profile::addRenderJob(const ProfileRenderJob &job) {
  if (!Profile::isEnabled()) return;

  flushLocalCounters();
  lock_guard<mutex> lock(profileMutex);
  renderJobs.push_back(job);
}


const char *Profile::counterName(ProfileCounter counter) {
  switch (counter) {
  case ProfileCounter::CELLS_VISITED: return "cells_visited";
  case ProfileCounter::CELLS_CULLED: return "cells_culled";
  case ProfileCounter::CELLS_CONTOURED: return "cells_contoured";
  case ProfileCounter::VERTEX_SAMPLES: return "vertex_samples";
  case ProfileCounter::VERTEX_CULLED: return "vertex_culled";
  case ProfileCounter::DEPTH_CALLS: return "depth_calls";
  case ProfileCounter::EDGE_CHECKS: return "edge_checks";
  case ProfileCounter::EDGE_INTERSECTIONS: return "edge_intersections";
  case ProfileCounter::LINEAR_INTERSECT_ITERATIONS:
    return "linear_intersect_iterations";
  case ProfileCounter::CONTOUR_GRID_EDGE_TRIANGLES:
    return "contour_grid_edge_triangles";
  case ProfileCounter::CONTOUR_CELL_CENTER_TRIANGLES:
    return "contour_cell_center_triangles";
  case ProfileCounter::TOOLSWEEP_DEPTH_CALLS: return "toolsweep_depth_calls";
  case ProfileCounter::TOOLSWEEP_COLLISION_CANDIDATES:
    return "toolsweep_collision_candidates";
  case ProfileCounter::TOOLSWEEP_SORTED_CANDIDATES:
    return "toolsweep_sorted_candidates";
  case ProfileCounter::TOOLSWEEP_XY_BIN_QUERIES:
    return "toolsweep_xy_bin_queries";
  case ProfileCounter::TOOLSWEEP_XY_BIN_OUT_OF_BOUNDS:
    return "toolsweep_xy_bin_out_of_bounds";
  case ProfileCounter::TOOLSWEEP_XY_BIN_REFS_SCANNED:
    return "toolsweep_xy_bin_refs_scanned";
  case ProfileCounter::TOOLSWEEP_XY_BIN_BBOX_HITS:
    return "toolsweep_xy_bin_bbox_hits";
  case ProfileCounter::TOOLSWEEP_XYZ_BIN_QUERIES:
    return "toolsweep_xyz_bin_queries";
  case ProfileCounter::TOOLSWEEP_XYZ_BIN_OUT_OF_BOUNDS:
    return "toolsweep_xyz_bin_out_of_bounds";
  case ProfileCounter::TOOLSWEEP_XYZ_BIN_REFS_SCANNED:
    return "toolsweep_xyz_bin_refs_scanned";
  case ProfileCounter::TOOLSWEEP_XYZ_BIN_BBOX_HITS:
    return "toolsweep_xyz_bin_bbox_hits";
  case ProfileCounter::TOOLSWEEP_CANDIDATE_CALLS_0:
    return "toolsweep_candidate_calls_0";
  case ProfileCounter::TOOLSWEEP_CANDIDATE_CALLS_1:
    return "toolsweep_candidate_calls_1";
  case ProfileCounter::TOOLSWEEP_CANDIDATE_CALLS_2_9:
    return "toolsweep_candidate_calls_2_9";
  case ProfileCounter::TOOLSWEEP_CANDIDATE_CALLS_10_99:
    return "toolsweep_candidate_calls_10_99";
  case ProfileCounter::TOOLSWEEP_CANDIDATE_CALLS_100_999:
    return "toolsweep_candidate_calls_100_999";
  case ProfileCounter::TOOLSWEEP_CANDIDATE_CALLS_1000_PLUS:
    return "toolsweep_candidate_calls_1000_plus";
  case ProfileCounter::AABB_NODE_VISITS: return "aabb_node_visits";
  case ProfileCounter::AABB_LEAF_HITS: return "aabb_leaf_hits";
  case ProfileCounter::COUNT: break;
  }

  return "unknown";
}


void Profile::write() {
  if (!Profile::isEnabled() || outputPath.empty()) return;

  flushLocalCounters();
  lock_guard<mutex> lock(profileMutex);
  ofstream out(outputPath);

  out << fixed << setprecision(6);
  out << "{\n";

  out << "  \"phases\": [\n";
  for (unsigned i = 0; i < phases.size(); i++) {
    const Phase &phase = phases[i];
    out << "    {\"name\": \"" << escapeJSON(phase.name)
        << "\", \"seconds\": " << phase.seconds << "}";
    if (i + 1 < phases.size()) out << ",";
    out << "\n";
  }
  out << "  ],\n";

  out << "  \"counters\": {\n";
  for (unsigned i = 0; i < (unsigned)ProfileCounter::COUNT; i++) {
    ProfileCounter counter = (ProfileCounter)i;
    out << "    \"" << counterName(counter) << "\": "
        << counters[i];
    if (i + 1 < (unsigned)ProfileCounter::COUNT) out << ",";
    out << "\n";
  }
  out << "  },\n";

  out << "  \"metrics\": {\n";
  size_t metricsRemaining = metrics.size() + realMetrics.size();
  for (auto it = metrics.begin(); it != metrics.end();) {
    out << "    \"" << escapeJSON(it->first) << "\": " << it->second;
    ++it;
    if (--metricsRemaining) out << ",";
    out << "\n";
  }
  for (auto it = realMetrics.begin(); it != realMetrics.end();) {
    out << "    \"" << escapeJSON(it->first) << "\": " << it->second;
    ++it;
    if (--metricsRemaining) out << ",";
    out << "\n";
  }
  out << "  },\n";

  out << "  \"render_jobs\": [\n";
  for (unsigned i = 0; i < renderJobs.size(); i++) {
    const ProfileRenderJob &job = renderJobs[i];
    out << "    {\"thread\": \"" << escapeJSON(job.thread)
        << "\", \"seconds\": " << job.seconds
        << ", \"cells_visited\": " << job.cellsVisited
        << ", \"cells_culled\": " << job.cellsCulled
        << ", \"cells_contoured\": " << job.cellsContoured
        << ", \"triangles\": " << job.triangles
        << ", \"vertex_samples\": " << job.vertexSamples
        << ", \"depth_calls\": " << job.depthCalls
        << ", \"edge_checks\": " << job.edgeChecks
        << ", \"edge_intersections\": " << job.edgeIntersections
        << ", \"linear_intersect_iterations\": "
        << job.linearIntersectIterations
        << ", \"contour_grid_edge_triangles\": "
        << job.contourGridEdgeTriangles
        << ", \"contour_cell_center_triangles\": "
        << job.contourCellCenterTriangles
        << ", \"toolsweep_depth_calls\": " << job.toolsweepDepthCalls
        << ", \"toolsweep_collision_candidates\": "
        << job.toolsweepCollisionCandidates
        << ", \"toolsweep_sorted_candidates\": "
        << job.toolsweepSortedCandidates
        << ", \"toolsweep_xy_bin_queries\": "
        << job.toolsweepXYBinQueries
        << ", \"toolsweep_xy_bin_out_of_bounds\": "
        << job.toolsweepXYBinOutOfBounds
        << ", \"toolsweep_xy_bin_refs_scanned\": "
        << job.toolsweepXYBinRefsScanned
        << ", \"toolsweep_xy_bin_bbox_hits\": "
        << job.toolsweepXYBinBBoxHits
        << ", \"toolsweep_xyz_bin_queries\": "
        << job.toolsweepXYZBinQueries
        << ", \"toolsweep_xyz_bin_out_of_bounds\": "
        << job.toolsweepXYZBinOutOfBounds
        << ", \"toolsweep_xyz_bin_refs_scanned\": "
        << job.toolsweepXYZBinRefsScanned
        << ", \"toolsweep_xyz_bin_bbox_hits\": "
        << job.toolsweepXYZBinBBoxHits
        << ", \"aabb_node_visits\": " << job.aabbNodeVisits
        << ", \"aabb_leaf_hits\": " << job.aabbLeafHits
        << "}";
    if (i + 1 < renderJobs.size()) out << ",";
    out << "\n";
  }
  out << "  ]\n";

  out << "}\n";
}
