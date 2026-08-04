/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#include "DexelSimulation.h"
#include "DexelSimulationInternal.h"

#include <camotics/Profile.h>
#include <camotics/Task.h>
#include <camotics/contour/TriangleSurface.h>
#include <camotics/sim/SparseToolpathArtifacts.h>

#include <gcode/Move.h>
#include <gcode/Tool.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <type_traits>
#include <vector>

using namespace std;
using namespace cb;
using namespace CAMotics;
using namespace CAMotics::Dexel;
using namespace CAMotics::Dexel::Internal;


CandidateResult Dexel::compute(const Simulation &sim, bool validateTopology,
                               Task *task,
                               const EligibilityReport *preclassified,
                               bool retainState, bool retainBoundaryOwners,
                               bool retainGrid) {
  CandidateResult result;
  EligibilityReport eligibility = preclassified ?
    *preclassified : classify(sim, task);
  if (!eligibility.eligible) {
    result.reason = eligibility.reason;
    return result;
  }
  if (task && task->shouldQuit()) {
    result.reason = RejectionReason::CANCELLED;
    return result;
  }

  if (task) task->begin("Preparing dexel simulation");
  unique_ptr<Profile::Scope> rasterScope
    (new Profile::Scope("dexel_rasterize"));
  Vector3D stockMin = sim.workpiece.getMin();
  Vector3D stockMax = sim.workpiece.getMax();
  Vector3D dims = sim.workpiece.getDimensions();
  unsigned nx = 0;
  unsigned ny = 0;
  unsigned nz = 0;
  if (!gridDimension(dims.x(), sim.resolution, nx) ||
      !gridDimension(dims.y(), sim.resolution, ny) ||
      (retainBoundaryOwners &&
       !gridDimension(dims.z(), sim.resolution, nz))) {
    result.reason = RejectionReason::INVALID_RESOLUTION;
    return result;
  }
  double xStep = dims.x() / nx;
  double yStep = dims.y() / ny;
  const uint64_t columns = (uint64_t)nx + 1;
  const uint64_t rows = (uint64_t)ny + 1;
  if (columns > numeric_limits<uint64_t>::max() / rows) {
    result.reason = RejectionReason::INVALID_RESOLUTION;
    return result;
  }
  const uint64_t columnCount = columns * rows;
  const size_t bytesPerColumn = sizeof(float) + sizeof(unsigned char) +
    (retainBoundaryOwners ? sizeof(unsigned) : 0);
  if (columnCount > numeric_limits<size_t>::max() / bytesPerColumn ||
      columnCount > vector<float>().max_size() ||
      columnCount > vector<unsigned char>().max_size() ||
      (retainBoundaryOwners &&
       columnCount > vector<unsigned>().max_size())) {
    result.reason = RejectionReason::INVALID_RESOLUTION;
    return result;
  }

  vector<float> tops(columnCount, (float)stockMax.z());
  shared_ptr<vector<unsigned> > boundaryOwners;
  if (retainBoundaryOwners)
    boundaryOwners = make_shared<vector<unsigned> >
      (columnCount, numeric_limits<unsigned>::max());
  vector<unsigned char> changed(columnCount, 0);
  RasterStats stats;
  stats.columns = columnCount;
  const double epsilon = max(1e-10, sim.resolution * 1e-9);

  const unsigned tileSize = 64;
  uint64_t tileCols64 = ((uint64_t)nx + tileSize) / tileSize;
  uint64_t tileRows64 = ((uint64_t)ny + tileSize) / tileSize;
  uint64_t tileCount = tileCols64 * tileRows64;
  if (numeric_limits<unsigned>::max() < tileCols64 ||
      numeric_limits<unsigned>::max() < tileRows64 ||
      numeric_limits<unsigned>::max() < tileCount ||
      vector<vector<unsigned> >().max_size() < tileCount) {
    result.reason = RejectionReason::INVALID_RESOLUTION;
    return result;
  }
  unsigned tileCols = (unsigned)tileCols64;
  unsigned tileRows = (unsigned)tileRows64;
  vector<PreparedMove> prepared;
  prepared.reserve(sim.path->size());
  vector<vector<unsigned> > tileMoves(tileCount);
  vector<StateCheckpoint> checkpoints;
  stats.tiles = tileCount;

  for (const GCode::Move &move: *sim.path) {
    stats.moves++;
    if (task && !(stats.moves & 4095) &&
        !task->update((double)stats.moves / sim.path->size())) {
      result.reason = RejectionReason::CANCELLED;
      return result;
    }
    int toolNo = move.getTool();
    if (toolNo < 0) continue;
    const GCode::Tool &tool = sim.path->getTools().get(toolNo);
    CutterProfile profile = getProfile(tool);
    const Vector3D &a = move.getStartPt();
    const Vector3D &b = move.getEndPt();
    double radius = stockRadius(profile, a, b, stockMin.z(), stockMax.z());
    if (radius < 0) continue;

    int x0 = clampGridIndex
      ((min(a.x(), b.x()) - radius - stockMin.x()) / xStep, nx);
    int x1 = clampGridIndex
      ((max(a.x(), b.x()) + radius - stockMin.x()) / xStep, nx);
    int y0 = clampGridIndex
      ((min(a.y(), b.y()) - radius - stockMin.y()) / yStep, ny);
    int y1 = clampGridIndex
      ((max(a.y(), b.y()) + radius - stockMin.y()) / yStep, ny);
    if (x1 < x0 || y1 < y0) continue;

    PreparedMove preparedMove;
    preparedMove.move = &move;
    preparedMove.profile = profile;
    preparedMove.radius = radius;
    preparedMove.x0 = x0;
    preparedMove.x1 = x1;
    preparedMove.y0 = y0;
    preparedMove.y1 = y1;
    unsigned moveIndex = prepared.size();
    prepared.push_back(preparedMove);

    unsigned tx0 = x0 / tileSize;
    unsigned tx1 = x1 / tileSize;
    unsigned ty0 = y0 / tileSize;
    unsigned ty1 = y1 / tileSize;
    for (unsigned ty = ty0; ty <= ty1; ty++)
      for (unsigned tx = tx0; tx <= tx1; tx++) {
        tileMoves[(uint64_t)ty * tileCols + tx].push_back(moveIndex);
        stats.tileMoveRefs++;
      }
  }

  if (retainState && 1 < prepared.size()) {
    const uint64_t checkpointBudget = 256ULL * 1024 * 1024;
    const uint64_t stateBytes =
      columnCount * (sizeof(float) +
                     (boundaryOwners ? sizeof(unsigned) : 0));
    unsigned checkpointCount = stateBytes ?
      min<uint64_t>(16, checkpointBudget / stateBytes) : 0;
    checkpointCount = min<uint64_t>(checkpointCount, prepared.size() - 1);
    vector<uint64_t> cumulativeWork(prepared.size() + 1, 0);
    for (unsigned i = 0; i < prepared.size(); i++) {
      const PreparedMove &item = prepared[i];
      uint64_t work = (uint64_t)(item.x1 - item.x0 + 1) *
        (item.y1 - item.y0 + 1);
      cumulativeWork[i + 1] =
        numeric_limits<uint64_t>::max() - cumulativeWork[i] < work ?
        numeric_limits<uint64_t>::max() : cumulativeWork[i] + work;
    }
    const uint64_t totalWork = cumulativeWork.back();
    unsigned previousEnd = 0;
    for (unsigned i = 0; i < checkpointCount; i++) {
      StateCheckpoint checkpoint;
      const uint64_t divisor = checkpointCount + 1;
      const uint64_t targetWork = totalWork / divisor * (i + 1) +
        totalWork % divisor * (i + 1) / divisor;
      unsigned end = lower_bound(cumulativeWork.begin() + previousEnd + 1,
                                 cumulativeWork.end(), targetWork) -
        cumulativeWork.begin();
      unsigned remaining = checkpointCount - i;
      unsigned maxEnd = prepared.size() - remaining;
      end = max(previousEnd + 1, min(maxEnd, end));

      // Prefer a nearby tool or rapid/cutting operation boundary when doing
      // so does not materially unbalance the cumulative raster work.
      unsigned window = max(1U, (unsigned)prepared.size() /
                            (checkpointCount + 1) / 4);
      unsigned searchBegin = max(previousEnd + 1,
                                 end < window ? 1U : end - window);
      unsigned searchEnd = min(maxEnd, end + window);
      uint64_t bestError = totalWork;
      unsigned bestEnd = end;
      for (unsigned candidate = searchBegin;
           candidate <= searchEnd; candidate++) {
        const GCode::Move &before = *prepared[candidate - 1].move;
        const GCode::Move &after = *prepared[candidate].move;
        if (before.getTool() == after.getTool() &&
            before.getType() == after.getType()) continue;
        uint64_t value = cumulativeWork[candidate];
        uint64_t error = value < targetWork ?
          targetWork - value : value - targetWork;
        if (error < bestError) {
          bestError = error;
          bestEnd = candidate;
        }
      }
      checkpoint.preparedEnd = bestEnd;
      checkpoint.time =
        prepared[checkpoint.preparedEnd - 1].move->getEndTime();
      checkpoint.tops.resize(columnCount);
      if (boundaryOwners)
        checkpoint.boundaryOwners.resize(columnCount);
      checkpoints.push_back(move(checkpoint));
      previousEnd = bestEnd;
    }
    Profile::setMetric("dexel_state_checkpoint_budget_bytes",
                       checkpointBudget);
    Profile::setMetric("dexel_state_checkpoint_count",
                       checkpoints.size());
    Profile::setMetric("dexel_state_checkpoint_bytes",
                       checkpoints.size() * stateBytes);
    Profile::setMetric("dexel_state_checkpoint_total_work", totalWork);
  }

  if (task && !task->update(1)) {
    result.reason = RejectionReason::CANCELLED;
    return result;
  }

  if (task) task->begin("Rasterizing dexel simulation");
  unsigned workerCount = max(1U, sim.threads);
  workerCount = min<uint64_t>(workerCount, tileCount);
  vector<RasterStats> workerStats(workerCount);
  atomic<uint64_t> nextTile(0);
  atomic<bool> violation(false);
  atomic<bool> stop(false);
  atomic<unsigned> workersDone(0);
  exception_ptr workerError;
  mutex workerErrorMutex;
  vector<thread> workers;
  workers.reserve(workerCount);

  auto rasterWorkerImpl = [&] (unsigned worker, auto captureTag) {
    constexpr bool capture = decltype(captureTag)::value;
    RasterStats &local = workerStats[worker];
    while (!violation.load(memory_order_relaxed) &&
           !stop.load(memory_order_relaxed)) {
      uint64_t tile = nextTile.fetch_add(1, memory_order_relaxed);
      if (tileCount <= tile) break;
      unsigned tileY = tile / tileCols;
      unsigned tileX = tile % tileCols;
      int tileX0 = tileX * tileSize;
      int tileX1 = min<unsigned>(nx, tileX0 + tileSize - 1);
      int tileY0 = tileY * tileSize;
      int tileY1 = min<unsigned>(ny, tileY0 + tileSize - 1);

      auto captureCheckpoint = [&] (StateCheckpoint &checkpoint) {
        for (int y = tileY0; y <= tileY1; y++) {
          uint64_t row = (uint64_t)y * (nx + 1);
          for (int x = tileX0; x <= tileX1; x++) {
            checkpoint.tops[row + x] = tops[row + x];
            if (boundaryOwners)
              checkpoint.boundaryOwners[row + x] =
                (*boundaryOwners)[row + x];
          }
        }
      };
      unsigned nextCheckpoint = 0;

      for (unsigned preparedIndex: tileMoves[tile]) {
        if (violation.load(memory_order_relaxed) ||
            stop.load(memory_order_relaxed)) break;
        if constexpr (capture)
          while (nextCheckpoint < checkpoints.size() &&
                 checkpoints[nextCheckpoint].preparedEnd <= preparedIndex)
            captureCheckpoint(checkpoints[nextCheckpoint++]);
        const PreparedMove &item = prepared[preparedIndex];
        const GCode::Move &move = *item.move;
        const Vector3D &a = move.getStartPt();
        const Vector3D &b = move.getEndPt();
        int localX0 = max(item.x0, tileX0);
        int localX1 = min(item.x1, tileX1);
        int localY0 = max(item.y0, tileY0);
        int localY1 = min(item.y1, tileY1);

        for (int y = localY0; y <= localY1; y++) {
          double py = stockMin.y() + y * yStep;
          for (int x = localX0; x <= localX1; x++) {
            local.footprintCells++;
            double px = stockMin.x() + x * xStep;
            double lo = 0;
            double hi = 0;
            if (!footprintInterval(a, b, px, py, item.radius, lo, hi))
              continue;
            local.insideCells++;

            uint64_t index = (uint64_t)y * (nx + 1) + x;
            double currentTop = tops[index];
            double lower = minimumLower
              (item.profile, a, b, px, py, lo, hi, local);
            if (currentTop <= lower + epsilon) continue;

            double upper = max(a.z() + (b.z() - a.z()) * lo,
                               a.z() + (b.z() - a.z()) * hi) +
              item.profile.length;
            if (upper + epsilon < currentTop) {
              local.multiIntervalViolations++;
              violation.store(true, memory_order_relaxed);
              break;
            }

            float nextTop = (float)max(stockMin.z(), lower);
            if (nextTop < tops[index]) {
              bool wasSolid = stockMin.z() + epsilon < tops[index];
              tops[index] = nextTop;
              if (boundaryOwners) (*boundaryOwners)[index] = preparedIndex;
              if (!changed[index]) {
                changed[index] = 1;
                local.changedColumns++;
              }
              if (wasSolid && tops[index] <= stockMin.z() + epsilon)
                local.emptiedColumns++;
            }
          }
          if (violation.load(memory_order_relaxed) ||
              stop.load(memory_order_relaxed)) break;
        }
      }
      if constexpr (capture)
        while (!violation.load(memory_order_relaxed) &&
               !stop.load(memory_order_relaxed) &&
               nextCheckpoint < checkpoints.size())
          captureCheckpoint(checkpoints[nextCheckpoint++]);
    }
  };

  auto rasterWorker = [&] (unsigned worker) {
    try {
      if (checkpoints.empty())
        rasterWorkerImpl(worker, false_type());
      else rasterWorkerImpl(worker, true_type());
    } catch (...) {
      lock_guard<mutex> lock(workerErrorMutex);
      if (!workerError) workerError = current_exception();
      stop.store(true, memory_order_relaxed);
    }
    workersDone.fetch_add(1, memory_order_release);
  };

  try {
    for (unsigned worker = 0; worker < workerCount; worker++)
      workers.emplace_back(rasterWorker, worker);
  } catch (...) {
    stop.store(true, memory_order_relaxed);
    for (thread &worker: workers) worker.join();
    throw;
  }
  while (workersDone.load(memory_order_acquire) < workerCount) {
    if (task) {
      double progress = min<uint64_t>(nextTile.load(memory_order_relaxed),
                                      tileCount) / (double)tileCount;
      if (!task->update(progress)) stop.store(true, memory_order_relaxed);
    }
    this_thread::sleep_for(chrono::milliseconds(20));
  }
  for (thread &worker: workers) worker.join();
  if (workerError) rethrow_exception(workerError);

  if (task && task->shouldQuit()) {
    result.reason = RejectionReason::CANCELLED;
    return result;
  }

  for (const RasterStats &local: workerStats) {
    stats.footprintCells += local.footprintCells;
    stats.insideCells += local.insideCells;
    stats.changedColumns += local.changedColumns;
    stats.emptiedColumns += local.emptiedColumns;
    stats.profileEvaluations += local.profileEvaluations;
    stats.multiIntervalViolations += local.multiIntervalViolations;
  }

  if (violation.load(memory_order_relaxed)) {
    recordRasterMetrics(stats);
    result.reason = RejectionReason::MULTI_INTERVAL_UPDATE;
    return result;
  }

  recordRasterMetrics(stats);
  Profile::setMetric("dexel_grid_x", nx);
  Profile::setMetric("dexel_grid_y", ny);
  if (stats.emptiedColumns) {
    result.reason = RejectionReason::EMPTY_COLUMN_UNSUPPORTED;
    return result;
  }
  rasterScope.reset();

  if ((retainState || retainGrid) && !validateTopology) {
    if (!retainState) {
      shared_ptr<vector<float> > gridTops =
        make_shared<vector<float> >(move(tops));
      result.surface = new GridSurface
        (sim, nx, ny, xStep, yStep, gridTops);
      result.accepted = true;
      Profile::setMetric("dexel_mesh_top_triangles", (uint64_t)2 * nx * ny);
      Profile::setMetric("dexel_mesh_bottom_triangles",
                         (uint64_t)2 * (nx + ny));
      Profile::setMetric("dexel_mesh_transition_triangles", 0);
      Profile::setMetric("dexel_mesh_side_triangles",
                         (uint64_t)4 * (nx + ny));
      Profile::setMetric("dexel_mesh_triangles",
                         result.surface->getTriangleCount());
      Profile::setMetric("dexel_topology_validation_skipped", 1);
      if (task) task->update(1);
      return result;
    }

    shared_ptr<State::Impl> stateImpl = make_shared<State::Impl>(sim);
    stateImpl->nx = nx;
    stateImpl->ny = ny;
    stateImpl->tileSize = tileSize;
    stateImpl->tileCols = tileCols;
    stateImpl->tileRows = tileRows;
    stateImpl->xStep = xStep;
    stateImpl->yStep = yStep;
    stateImpl->epsilon = epsilon;
    stateImpl->currentTime = sim.path->getTime();
    stateImpl->tops = make_shared<vector<float> >(move(tops));
    stateImpl->prepared = move(prepared);
    stateImpl->tileMoves = move(tileMoves);
    stateImpl->checkpoints = move(checkpoints);
    stateImpl->boundaryOwners = boundaryOwners;
    result.state = make_shared<State>(stateImpl);
    result.surface = new GridSurface
      (sim, nx, ny, xStep, yStep, stateImpl->tops);
    result.accepted = true;
    Profile::setMetric("dexel_mesh_top_triangles", (uint64_t)2 * nx * ny);
    Profile::setMetric("dexel_mesh_bottom_triangles",
                       (uint64_t)2 * (nx + ny));
    Profile::setMetric("dexel_mesh_transition_triangles", 0);
    Profile::setMetric("dexel_mesh_side_triangles",
                       (uint64_t)4 * (nx + ny));
    Profile::setMetric("dexel_mesh_triangles",
                       result.surface->getTriangleCount());
    Profile::setMetric("dexel_topology_validation_skipped", 1);
    Profile::setMetric("dexel_state_current_bytes",
                       result.state->getCurrentBytes());
    Profile::setMetric("dexel_state_checkpoint_count",
                       result.state->getCheckpointCount());
    Profile::setMetric("dexel_state_checkpoint_bytes",
                       result.state->getCheckpointBytes());
    if (task) task->update(1);
    return result;
  }

  RejectionReason surfaceReason = RejectionReason::NONE;
  SmartPointer<TriangleSurface> surface = buildSurface
    (sim, nx, ny, xStep, yStep, tops, task, surfaceReason);
  if (surface.isNull()) {
    result.reason = surfaceReason;
    return result;
  }
  auto retainAcceptedState = [&] () {
    if (!retainState) {
      if (retainGrid) {
        shared_ptr<vector<float> > gridTops =
          make_shared<vector<float> >(move(tops));
        result.surface = new GridSurface
          (sim, nx, ny, xStep, yStep, gridTops);
      }
      return;
    }
    shared_ptr<State::Impl> stateImpl = make_shared<State::Impl>(sim);
    stateImpl->nx = nx;
    stateImpl->ny = ny;
    stateImpl->tileSize = tileSize;
    stateImpl->tileCols = tileCols;
    stateImpl->tileRows = tileRows;
    stateImpl->xStep = xStep;
    stateImpl->yStep = yStep;
    stateImpl->epsilon = epsilon;
    stateImpl->currentTime = sim.path->getTime();
    stateImpl->tops = make_shared<vector<float> >(move(tops));
    stateImpl->prepared = move(prepared);
    stateImpl->tileMoves = move(tileMoves);
    stateImpl->checkpoints = move(checkpoints);
    stateImpl->boundaryOwners = boundaryOwners;
    result.state = make_shared<State>(stateImpl);
    result.surface = new GridSurface
      (sim, nx, ny, xStep, yStep, stateImpl->tops);
    Profile::setMetric("dexel_state_current_bytes",
                       result.state->getCurrentBytes());
    Profile::setMetric("dexel_state_checkpoint_count",
                       result.state->getCheckpointCount());
    Profile::setMetric("dexel_state_checkpoint_bytes",
                       result.state->getCheckpointBytes());
  };

  if (!validateTopology) {
    Profile::setMetric("dexel_topology_validation_skipped", 1);
    result.accepted = true;
    result.surface = surface;
    retainAcceptedState();
    return result;
  }

  if (task) task->begin("Validating dexel topology");
  Profile::Scope topologyScope("dexel_topology_validation");
  double topologyTolerance = max(1e-6, sim.resolution * 1e-6);
  SparseToolpath::SurfaceTopologyReport topology =
    SparseToolpath::validateSurfaceTopology(*surface, topologyTolerance, task);
  if (task && task->shouldQuit()) {
    result.reason = RejectionReason::CANCELLED;
    return result;
  }
  Profile::setMetric("dexel_topology_boundary_edges", topology.boundaryEdges);
  Profile::setMetric("dexel_topology_nonmanifold_edges",
                     topology.nonManifoldEdges);
  Profile::setMetric("dexel_topology_misoriented_edges",
                     topology.misorientedEdges);
  Profile::setMetric("dexel_topology_degenerate_triangles",
                     topology.degenerateTriangles);
  Profile::setMetric("dexel_topology_duplicate_triangles",
                     topology.duplicateTriangles);
  if (!topology.accepted()) {
    result.reason = RejectionReason::TOPOLOGY_VALIDATION;
    return result;
  }

  result.accepted = true;
  result.surface = surface;
  retainAcceptedState();
  if (task) task->update(1);
  return result;
}
