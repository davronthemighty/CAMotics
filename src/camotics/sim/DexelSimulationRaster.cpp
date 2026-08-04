/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#include "DexelSimulationInternal.h"

#include <camotics/Grid.h>
#include <camotics/Profile.h>
#include <camotics/Task.h>
#include <camotics/contour/FieldFunction.h>
#include <camotics/contour/GridTree.h>
#include <camotics/contour/GridTreeRef.h>
#include <camotics/render/RenderJob.h>
#include <camotics/sim/Sweep.h>
#include <camotics/sim/ToolSweep.h>

#include <gcode/Move.h>
#include <gcode/Tool.h>
#include <gcode/ToolPath.h>
#include <gcode/ToolTable.h>

#include <cbang/log/Logger.h>
#include <cbang/thread/Condition.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>


using namespace std;
using namespace cb;
using namespace CAMotics;
using namespace CAMotics::Dexel;
using namespace CAMotics::Dexel::Internal;


namespace CAMotics { namespace Dexel { namespace Internal {
  CutterProfile getProfile(const GCode::Tool &tool) {
    CutterProfile profile;
    profile.length = tool.getLength();
    profile.topRadius = tool.getRadius();
    switch (tool.getShape()) {
    case GCode::ToolShape::TS_CYLINDRICAL:
      profile.bottomRadius = profile.topRadius;
      break;
    case GCode::ToolShape::TS_CONICAL:
      profile.bottomRadius = 0;
      break;
    case GCode::ToolShape::TS_SNUBNOSE:
      profile.bottomRadius = tool.getSnubDiameter() / 2;
      break;
    default: break;
    }
    return profile;
  }


  bool footprintInterval(const Vector3D &a, const Vector3D &b,
                         double x, double y, double radius,
                         double &lo, double &hi) {
    double qx = a.x() - x;
    double qy = a.y() - y;
    double vx = b.x() - a.x();
    double vy = b.y() - a.y();
    double aa = vx * vx + vy * vy;
    double cc = qx * qx + qy * qy - radius * radius;

    if (!aa) {
      if (0 < cc) return false;
      lo = 0;
      hi = 1;
      return true;
    }

    double bb = 2 * (qx * vx + qy * vy);
    double disc = bb * bb - 4 * aa * cc;
    if (disc < 0) return false;
    double root = sqrt(max(0.0, disc));
    lo = max(0.0, (-bb - root) / (2 * aa));
    hi = min(1.0, (-bb + root) / (2 * aa));
    return lo <= hi;
  }


  double lowerAt(const CutterProfile &profile, const Vector3D &a,
                 const Vector3D &b, double x, double y, double t,
                 RasterStats &stats) {
    stats.profileEvaluations++;
    double px = a.x() + (b.x() - a.x()) * t;
    double py = a.y() + (b.y() - a.y()) * t;
    double pz = a.z() + (b.z() - a.z()) * t;
    double dx = px - x;
    double dy = py - y;
    return pz + profile.surfaceHeight(sqrt(dx * dx + dy * dy));
  }


  double minimumLower(const CutterProfile &profile, const Vector3D &a,
                      const Vector3D &b, double x, double y,
                      double lo, double hi, RasterStats &stats) {
    double dz = b.z() - a.z();
    double vx = b.x() - a.x();
    double vy = b.y() - a.y();
    double vv = vx * vx + vy * vy;
    double nearest = vv ?
      ((x - a.x()) * vx + (y - a.y()) * vy) / vv : lo;
    nearest = max(lo, min(hi, nearest));

    if (fabs(dz) <= 1e-14)
      return lowerAt(profile, a, b, x, y, nearest, stats);

    double result = min(lowerAt(profile, a, b, x, y, lo, stats),
                        lowerAt(profile, a, b, x, y, hi, stats));
    result = min(result,
                 lowerAt(profile, a, b, x, y, nearest, stats));

    // The ramp objective is convex over the cutter footprint.  A short
    // golden-section refinement is the reference path; constant-Z engraving
    // takes the exact nearest-point fast path above.
    double left = lo;
    double right = hi;
    const double ratio = 0.6180339887498948482;
    double t1 = right - (right - left) * ratio;
    double t2 = left + (right - left) * ratio;
    double f1 = lowerAt(profile, a, b, x, y, t1, stats);
    double f2 = lowerAt(profile, a, b, x, y, t2, stats);
    for (unsigned i = 0; i < 12; i++) {
      if (f1 < f2) {
        right = t2;
        t2 = t1;
        f2 = f1;
        t1 = right - (right - left) * ratio;
        f1 = lowerAt(profile, a, b, x, y, t1, stats);
      } else {
        left = t1;
        t1 = t2;
        f1 = f2;
        t2 = left + (right - left) * ratio;
        f2 = lowerAt(profile, a, b, x, y, t2, stats);
      }
    }
    return min(result, min(f1, f2));
  }


  double stockRadius(const CutterProfile &profile, const Vector3D &a,
                     const Vector3D &b, double bottom, double top) {
    double moveMin = min(a.z(), b.z());
    double moveMax = max(a.z(), b.z());
    double rawLo = bottom - moveMax;
    double rawHi = top - moveMin;
    if (rawHi < 0 || profile.length < rawLo) return -1;
    double lo = max(0.0, rawLo);
    double hi = min(profile.length, rawHi);
    return max(profile.radiusAt(lo), profile.radiusAt(hi));
  }


  void recordRasterMetrics(const RasterStats &stats) {
    Profile::setMetric("dexel_columns_allocated", stats.columns);
    Profile::setMetric("dexel_moves_processed", stats.moves);
    Profile::setMetric("dexel_footprint_cells_considered",
                       stats.footprintCells);
    Profile::setMetric("dexel_footprint_cells_inside", stats.insideCells);
    Profile::setMetric("dexel_columns_changed", stats.changedColumns);
    Profile::setMetric("dexel_columns_emptied", stats.emptiedColumns);
    Profile::setMetric("dexel_profile_evaluations",
                       stats.profileEvaluations);
    Profile::setMetric("dexel_multi_interval_violations",
                       stats.multiIntervalViolations);
    Profile::setMetric("dexel_raster_tiles", stats.tiles);
    Profile::setMetric("dexel_tile_move_refs", stats.tileMoveRefs);
  }



}}}


Dexel::State::State(const shared_ptr<Impl> &impl) : impl(impl) {}


double Dexel::State::getTime() const {return impl->currentTime;}


uint64_t Dexel::State::getCurrentBytes() const {
  uint64_t bytes = impl->tops->size() * sizeof(float) +
    (impl->boundaryOwners ?
     impl->boundaryOwners->size() * sizeof(unsigned) : 0);
  bytes += impl->boundaryTiles.size() *
    sizeof(shared_ptr<const BoundaryTile>);
  for (const auto &tile: impl->boundaryTiles)
    if (tile)
      bytes += sizeof(BoundaryTile) + tile->cliffCells.size() +
        (tile->vertices.size() + tile->normals.size()) * sizeof(float);
  return bytes;
}


uint64_t Dexel::State::getCheckpointBytes() const {
  uint64_t bytes = 0;
  for (const StateCheckpoint &checkpoint: impl->checkpoints) {
    bytes += checkpoint.tops.size() * sizeof(float);
    bytes += checkpoint.boundaryOwners.size() * sizeof(unsigned);
  }
  return bytes;
}


uint64_t Dexel::State::getCheckpointCount() const {
  return impl->checkpoints.size();
}


bool Dexel::State::hasBoundaryOwners() const {
  return impl->boundaryOwners != 0;
}


SmartPointer<FieldFunction> Dexel::State::getBoundaryFieldFor
(const shared_ptr<const vector<unsigned> > &owners,
 double currentTime) const {
  if (!owners)
    THROW("Dexel boundary producer ownership was not retained");

  class ProducerBoundaryField : public FieldFunction {
    shared_ptr<const State::Impl> state;
    double currentTime;
    shared_ptr<const vector<unsigned> > owners;
    vector<SmartPointer<Sweep> > sweeps;

  public:
    ProducerBoundaryField
    (const shared_ptr<const State::Impl> &state, double currentTime,
     const shared_ptr<const vector<unsigned> > &owners) :
      state(state), currentTime(currentTime), owners(owners) {
      for (const auto &entry: state->sim.path->getTools()) {
        const GCode::Tool &tool = entry.second;
        unsigned toolNo = tool.getNumber();
        if (sweeps.size() <= toolNo) sweeps.resize(toolNo + 1);
        sweeps[toolNo] = ToolSweep::getSweep(tool);
      }
    }

    double depth(const Vector3D &p) const override {
      const Vector3D stockMin = state->sim.workpiece.getMin();
      int cellX = (int)floor((p.x() - stockMin.x()) / state->xStep);
      int cellY = (int)floor((p.y() - stockMin.y()) / state->yStep);
      cellX = max(0, min((int)state->nx - 1, cellX));
      cellY = max(0, min((int)state->ny - 1, cellY));
      unsigned candidates[25];
      unsigned candidateCount = 0;
      const unsigned noOwner = numeric_limits<unsigned>::max();

      for (int y = max(0, cellY - 2);
           y <= min((int)state->ny, cellY + 2); y++)
        for (int x = max(0, cellX - 2);
             x <= min((int)state->nx, cellX + 2); x++) {
          unsigned owner =
            (*owners)[(uint64_t)y * (state->nx + 1) + x];
          if (owner == noOwner || state->prepared.size() <= owner) continue;
          bool duplicate = false;
          for (unsigned i = 0; i < candidateCount; i++)
            if (candidates[i] == owner) {
              duplicate = true;
              break;
            }
          if (!duplicate) candidates[candidateCount++] = owner;
        }

      double toolDepth = -numeric_limits<double>::max();
      for (unsigned i = 0; i < candidateCount; i++) {
        const GCode::Move &move = *state->prepared[candidates[i]].move;
        unsigned toolNo = move.getTool();
        if (sweeps.size() <= toolNo || sweeps[toolNo].isNull()) continue;
        Vector3D end = move.getPtAtTime
          (min(currentTime, move.getEndTime()));
        double value = sweeps[toolNo]->depth(move.getStartPt(), end, p);
        if (0 <= value) {
          toolDepth = value;
          break;
        }
        toolDepth = max(toolDepth, value);
      }

      return min(state->sim.workpiece.depth(p), -toolDepth);
    }
  };

  return new ProducerBoundaryField
    (impl, currentTime, owners);
}


SmartPointer<FieldFunction> Dexel::State::getBoundaryField() const {
  return getBoundaryFieldFor(impl->boundaryOwners, impl->currentTime);
}


vector<Dexel::BoundaryTile> Dexel::State::buildBoundaryTiles
(const vector<unsigned> &dirtyTiles, bool full, Task *task) const {
  return buildBoundaryTilesFor
    (dirtyTiles, full, impl->tops, impl->boundaryOwners,
     impl->currentTime, task);
}


vector<Dexel::BoundaryTile> Dexel::State::buildBoundaryTilesFor
(const vector<unsigned> &dirtyTiles, bool full,
 const shared_ptr<const vector<float> > &tops,
 const shared_ptr<const vector<unsigned> > &owners,
 double currentTime, Task *task) const {
  const auto started = chrono::steady_clock::now();
  if (!owners)
    THROW("Dexel boundary producer ownership was not retained");
  if (task) task->begin("Reconstructing dexel boundary");

  const uint64_t tileCount = (uint64_t)impl->tileCols * impl->tileRows;
  vector<unsigned char> selected(tileCount, full ? 1 : 0);
  if (!full)
    for (unsigned tile: dirtyTiles) {
      if (tileCount <= tile) continue;
      unsigned tileX = tile % impl->tileCols;
      unsigned tileY = tile / impl->tileCols;
      selected[tile] = 1;
      if (tileX) selected[tile - 1] = 1;
      if (tileY) selected[tile - impl->tileCols] = 1;
      if (tileX && tileY)
        selected[tile - impl->tileCols - 1] = 1;
    }

  vector<unsigned> tileIDs;
  tileIDs.reserve(tileCount);
  for (unsigned tile = 0; tile < tileCount; tile++)
    if (selected[tile]) tileIDs.push_back(tile);
  if (tileIDs.empty()) return {};

  SmartPointer<FieldFunction> field =
    getBoundaryFieldFor(owners, currentTime);
  const double resolution = impl->sim.resolution;
  const double cliffSpan = 2 * resolution;
  const double halo = resolution;
  const Rectangle3D stockBounds = impl->sim.workpiece.getBounds();
  const Vector3D stockMin = stockBounds.getMin();
  const Grid globalGrid(stockBounds.grow(resolution * 0.9), resolution);
  const Vector3D globalMin = globalGrid.getOffset();
  const Vector3U globalSteps = globalGrid.getSteps();

  vector<BoundaryTile> patches(tileIDs.size());
  vector<RenderStats> tileStats(tileIDs.size());
  atomic<unsigned> nextTile(0);
  atomic<unsigned> workersDone(0);
  atomic<bool> stop(false);
  exception_ptr workerError;
  mutex workerErrorMutex;
  unsigned workerCount = max(1U, impl->sim.threads);
  workerCount = min<unsigned>(workerCount, tileIDs.size());
  vector<thread> workers;
  workers.reserve(workerCount);

  struct Region {
    unsigned x;
    unsigned y;
    unsigned z0;
    unsigned z1;
  };

  auto worker = [&] () {
    try {
      while (!stop.load(memory_order_relaxed)) {
      unsigned outputIndex = nextTile.fetch_add(1, memory_order_relaxed);
      if (tileIDs.size() <= outputIndex) break;
      unsigned tile = tileIDs[outputIndex];
      unsigned tileX = tile % impl->tileCols;
      unsigned tileY = tile / impl->tileCols;
      unsigned x0 = tileX * impl->tileSize;
      unsigned y0 = tileY * impl->tileSize;
      unsigned x1 = min(impl->nx, x0 + impl->tileSize);
      unsigned y1 = min(impl->ny, y0 + impl->tileSize);

      BoundaryTile patch;
      patch.tile = tile;
      patch.cellX = x0;
      patch.cellY = y0;
      patch.cellsX = x1 - x0;
      patch.cellsY = y1 - y0;
      patch.cliffCells.assign
        ((uint64_t)patch.cellsX * patch.cellsY, 0);
      vector<Region> regions;

      for (unsigned y = y0; y < y1; y++)
        for (unsigned x = x0; x < x1; x++) {
          uint64_t i00 = (uint64_t)y * (impl->nx + 1) + x;
          uint64_t i10 = i00 + 1;
          uint64_t i01 = i00 + impl->nx + 1;
          uint64_t i11 = i01 + 1;
          float cellMin = min(min((*tops)[i00], (*tops)[i10]),
                              min((*tops)[i01], (*tops)[i11]));
          float cellMax = max(max((*tops)[i00], (*tops)[i10]),
                              max((*tops)[i01], (*tops)[i11]));
          if (cellMax - cellMin <= cliffSpan) continue;
          patch.cliffCells[(uint64_t)(y - y0) * patch.cellsX + x - x0] = 1;

          double centerX = stockMin.x() + (x + 0.5) * impl->xStep;
          double centerY = stockMin.y() + (y + 0.5) * impl->yStep;
          int gridX =
            (int)floor((centerX - globalMin.x()) / resolution);
          int gridY =
            (int)floor((centerY - globalMin.y()) / resolution);
          gridX = max(0, min((int)globalSteps.x() - 1, gridX));
          gridY = max(0, min((int)globalSteps.y() - 1, gridY));
          int gridZ0 =
            (int)floor(((double)cellMin - halo - globalMin.z()) /
                       resolution);
          int gridZ1 =
            (int)ceil(((double)cellMax + halo - globalMin.z()) /
                      resolution);
          gridZ0 = max(0, min((int)globalSteps.z() - 1, gridZ0));
          gridZ1 =
            max(gridZ0 + 1, min((int)globalSteps.z(), gridZ1));
          regions.push_back
            ({(unsigned)gridX, (unsigned)gridY,
              (unsigned)gridZ0, (unsigned)gridZ1});
        }

      sort(regions.begin(), regions.end(),
           [] (const Region &a, const Region &b) {
             if (a.y != b.y) return a.y < b.y;
             if (a.x != b.x) return a.x < b.x;
             if (a.z0 != b.z0) return a.z0 < b.z0;
             return a.z1 < b.z1;
           });
      vector<Region> uniqueRegions;
      uniqueRegions.reserve(regions.size());
      for (const Region &region: regions)
        if (!uniqueRegions.empty() &&
            uniqueRegions.back().x == region.x &&
            uniqueRegions.back().y == region.y) {
          uniqueRegions.back().z0 = min(uniqueRegions.back().z0, region.z0);
          uniqueRegions.back().z1 = max(uniqueRegions.back().z1, region.z1);
        } else uniqueRegions.push_back(region);

      if (!uniqueRegions.empty()) {
        unsigned minX = uniqueRegions.front().x;
        unsigned maxX = uniqueRegions.front().x;
        unsigned minY = uniqueRegions.front().y;
        unsigned maxY = uniqueRegions.front().y;
        for (const Region &region: uniqueRegions) {
          minX = min(minX, region.x);
          maxX = max(maxX, region.x);
          minY = min(minY, region.y);
          maxY = max(maxY, region.y);
        }
        Vector3U treeSteps(maxX - minX + 1, maxY - minY + 1,
                           globalSteps.z());
        SmartPointer<GridTree> tree =
          new GridTree(Grid(globalMin +
                            Vector3D(minX, minY, 0) * resolution,
                            treeSteps, resolution));
        for (const Region &region: uniqueRegions) {
          Vector3U relative(region.x - minX, region.y - minY, region.z0);
          Vector3U steps(1, 1, region.z1 - region.z0);
          GridTreeRef grid
            (tree.get(), relative, steps,
             Vector3U(region.x, region.y, region.z0));
          Condition condition;
          RenderJob job(condition, *field, impl->sim.mode, grid);
          job.run();
          tileStats[outputIndex].add(job.getStats());
        }
        tree->gather(patch.vertices, patch.normals);
      }
        patches[outputIndex] = move(patch);
      }
    } catch (...) {
      lock_guard<mutex> lock(workerErrorMutex);
      if (!workerError) workerError = current_exception();
      stop.store(true, memory_order_relaxed);
    }
    workersDone.fetch_add(1, memory_order_release);
  };

  try {
    for (unsigned i = 0; i < workerCount; i++)
      workers.emplace_back(worker);
  } catch (...) {
    stop.store(true, memory_order_relaxed);
    for (thread &workerThread: workers) workerThread.join();
    throw;
  }
  while (workersDone.load(memory_order_acquire) < workerCount) {
    if (task) {
      double progress =
        min<unsigned>(nextTile.load(memory_order_relaxed), tileIDs.size()) /
        (double)tileIDs.size();
      if (!task->update(progress)) stop.store(true, memory_order_relaxed);
    }
    this_thread::sleep_for(chrono::milliseconds(10));
  }
  for (thread &workerThread: workers) workerThread.join();
  if (workerError) rethrow_exception(workerError);
  if ((task && task->shouldQuit()) || stop.load(memory_order_relaxed))
    return {};

  RenderStats total;
  uint64_t triangles = 0;
  uint64_t bytes = 0;
  uint64_t cliffCells = 0;
  for (unsigned i = 0; i < patches.size(); i++) {
    total.add(tileStats[i]);
    triangles += patches[i].getTriangleCount();
    bytes += (patches[i].vertices.size() + patches[i].normals.size()) *
      sizeof(float);
    for (unsigned char cliff: patches[i].cliffCells)
      cliffCells += cliff != 0;
  }
  Profile::setMetric("dexel_boundary_tiles", patches.size());
  Profile::setMetric("dexel_boundary_cliff_cells", cliffCells);
  Profile::setMetric("dexel_boundary_triangles", triangles);
  Profile::setMetric("dexel_boundary_bytes", bytes);
  Profile::setMetric("dexel_boundary_cells_visited", total.cellsVisited);
  Profile::setMetric("dexel_boundary_depth_calls", total.depthCalls);
  double elapsedMS =
    chrono::duration<double, milli>
      (chrono::steady_clock::now() - started).count();
  Profile::setMetric("dexel_boundary_ms", elapsedMS);
  LOG_INFO(1, "Dexel boundary tiles: tiles=" << patches.size()
           << " cliff_cells=" << cliffCells
           << " triangles=" << triangles
           << " bytes=" << bytes
           << " cells=" << total.cellsVisited
           << " elapsed_ms=" << elapsedMS);
  return patches;
}


void Dexel::State::publishBoundaryTiles
(GridSurface &surface, vector<BoundaryTile> &&tiles) {
  if (impl->boundaryTiles.empty())
    impl->boundaryTiles.resize
      ((uint64_t)impl->tileCols * impl->tileRows);
  for (BoundaryTile &tile: tiles) {
    const unsigned tileID = tile.tile;
    if (tileID < impl->boundaryTiles.size())
      impl->boundaryTiles[tileID] =
        make_shared<BoundaryTile>(move(tile));
  }
  impl->boundaryTilesStale = false;
  surface.setBoundaryTiles(impl->boundaryTiles);
  Profile::setMetric("dexel_state_current_bytes", getCurrentBytes());
  LOG_INFO(1, "Dexel boundary snapshot: tiles="
           << impl->boundaryTiles.size()
           << " current_bytes=" << getCurrentBytes());
}


CandidateResult Dexel::State::seek
(double targetTime, Task *task, bool buildBoundary) {
  const auto seekStarted = chrono::steady_clock::now();
  CandidateResult result;
  const double pathTime = impl->sim.path->getTime();
  targetTime = max(0.0, min(pathTime, targetTime));
  // The ramp minimizer is not bit-associative across clipped segments.  Keep
  // the values changed by the current partial move so consecutive playback
  // frames can return to its exact start without restoring a full checkpoint.
  const bool resumedPartial = impl->hasPartialBase &&
    impl->partialStartTime <= targetTime;
  bool restored = targetTime < impl->currentTime && !resumedPartial;
  int restoredCheckpoint = -1;
  if (restored)
    for (unsigned i = 0; i < impl->checkpoints.size(); i++) {
      if (targetTime < impl->checkpoints[i].time) break;
      restoredCheckpoint = i;
    }
  const double fromTime = resumedPartial ? impl->partialStartTime :
    restoredCheckpoint < 0 ? (restored ? 0 : impl->currentTime) :
    impl->checkpoints[restoredCheckpoint].time;

  if (task) task->begin(restored ? "Restoring dexel state" :
                        "Updating dexel state");
  vector<float> nextTops;
  if (0 <= restoredCheckpoint)
    nextTops = impl->checkpoints[restoredCheckpoint].tops;
  else if (restored)
    nextTops.assign(impl->tops->size(),
                    (float)impl->sim.workpiece.getMax().z());
  else {
    nextTops = *impl->tops;
    if (resumedPartial)
      for (const auto &entry: impl->partialUndo)
        nextTops[entry.first] = entry.second;
  }

  const unsigned noOwner = numeric_limits<unsigned>::max();
  vector<unsigned> nextBoundaryOwners;
  if (impl->boundaryOwners) {
    if (0 <= restoredCheckpoint)
      nextBoundaryOwners =
        impl->checkpoints[restoredCheckpoint].boundaryOwners;
    else if (restored)
      nextBoundaryOwners.assign(impl->boundaryOwners->size(), noOwner);
    else {
      nextBoundaryOwners = *impl->boundaryOwners;
      if (resumedPartial)
        for (const auto &entry: impl->partialOwnerUndo)
          nextBoundaryOwners[entry.first] = entry.second;
    }
  }

  const unsigned noPreparedMove = numeric_limits<unsigned>::max();
  unsigned targetPartialMove = noPreparedMove;
  auto targetMove = lower_bound
    (impl->prepared.begin(), impl->prepared.end(), targetTime,
     [] (const PreparedMove &item, double time) {
       return item.move->getEndTime() <= time;
     });
  if (targetMove != impl->prepared.end() &&
      targetMove->move->getStartTime() < targetTime &&
      targetTime < targetMove->move->getEndTime())
    targetPartialMove = targetMove - impl->prepared.begin();

  auto preparedBegin = lower_bound
    (impl->prepared.begin(), impl->prepared.end(), fromTime,
     [] (const PreparedMove &item, double time) {
       return item.move->getEndTime() <= time;
     });
  uint64_t replayedMoves = 0;
  for (auto it = preparedBegin; it != impl->prepared.end(); ++it) {
    const PreparedMove &item = *it;
    const GCode::Move &move = *item.move;
    if (targetTime <= move.getStartTime()) break;
    replayedMoves++;
  }

  RasterStats stats;
  stats.columns = nextTops.size();
  stats.moves = replayedMoves;
  stats.tiles = impl->tileMoves.size();
  Vector3D stockMin = impl->sim.workpiece.getMin();
  const uint64_t tileCount = impl->tileMoves.size();
  unsigned workerCount = max(1U, impl->sim.threads);
  workerCount = min<uint64_t>(workerCount, tileCount);
  vector<RasterStats> workerStats(workerCount);
  vector<vector<pair<uint64_t, float> > > workerPartialUndo(workerCount);
  vector<vector<pair<uint64_t, unsigned> > >
    workerPartialOwnerUndo(workerCount);
  atomic<uint64_t> nextTile(0);
  atomic<bool> violation(false);
  atomic<bool> stop(false);
  atomic<unsigned> workersDone(0);
  exception_ptr workerError;
  mutex workerErrorMutex;
  vector<thread> workers;
  workers.reserve(workerCount);
  const auto rasterStarted = chrono::steady_clock::now();

  auto rasterWorker = [&] (unsigned worker) {
    try {
      RasterStats &local = workerStats[worker];
      while (!violation.load(memory_order_relaxed) &&
             !stop.load(memory_order_relaxed)) {
      uint64_t tile = nextTile.fetch_add(1, memory_order_relaxed);
      if (tileCount <= tile) break;
      unsigned tileY = tile / impl->tileCols;
      unsigned tileX = tile % impl->tileCols;
      int tileX0 = tileX * impl->tileSize;
      int tileX1 = min<unsigned>
        (impl->nx, tileX0 + impl->tileSize - 1);
      int tileY0 = tileY * impl->tileSize;
      int tileY1 = min<unsigned>
        (impl->ny, tileY0 + impl->tileSize - 1);

      const vector<unsigned> &tileMoves = impl->tileMoves[tile];
      auto tileMove = lower_bound
        (tileMoves.begin(), tileMoves.end(), fromTime,
         [&] (unsigned preparedIndex, double time) {
           return impl->prepared[preparedIndex].move->getEndTime() <= time;
         });
      for (; tileMove != tileMoves.end(); ++tileMove) {
        if (violation.load(memory_order_relaxed) ||
            stop.load(memory_order_relaxed)) break;
        unsigned preparedIndex = *tileMove;
        const PreparedMove &item = impl->prepared[preparedIndex];
        const GCode::Move &move = *item.move;
        if (targetTime <= move.getStartTime()) break;
        local.tileMoveRefs++;

        const Vector3D a = move.getPtAtTime
          (max(fromTime, move.getStartTime()));
        const Vector3D b = move.getPtAtTime
          (min(targetTime, move.getEndTime()));
        int localX0 = max(item.x0, tileX0);
        int localX1 = min(item.x1, tileX1);
        int localY0 = max(item.y0, tileY0);
        int localY1 = min(item.y1, tileY1);

        for (int y = localY0; y <= localY1; y++) {
          double py = stockMin.y() + y * impl->yStep;
          for (int x = localX0; x <= localX1; x++) {
            local.footprintCells++;
            double px = stockMin.x() + x * impl->xStep;
            double lo = 0;
            double hi = 0;
            if (!footprintInterval(a, b, px, py, item.radius, lo, hi))
              continue;
            local.insideCells++;

            uint64_t index = (uint64_t)y * (impl->nx + 1) + x;
            double currentTop = nextTops[index];
            double lower = minimumLower
              (item.profile, a, b, px, py, lo, hi, local);
            if (currentTop <= lower + impl->epsilon) continue;

            double upper = max(a.z() + (b.z() - a.z()) * lo,
                               a.z() + (b.z() - a.z()) * hi) +
              item.profile.length;
            if (upper + impl->epsilon < currentTop) {
              local.multiIntervalViolations++;
              violation.store(true, memory_order_relaxed);
              break;
            }

            float nextTop = (float)max(stockMin.z(), lower);
            if (nextTop < nextTops[index]) {
              bool wasSolid = stockMin.z() + impl->epsilon < nextTops[index];
              if (preparedIndex == targetPartialMove) {
                workerPartialUndo[worker].emplace_back
                  (index, nextTops[index]);
                if (!nextBoundaryOwners.empty())
                  workerPartialOwnerUndo[worker].emplace_back
                    (index, nextBoundaryOwners[index]);
              }
              nextTops[index] = nextTop;
              if (!nextBoundaryOwners.empty())
                nextBoundaryOwners[index] = preparedIndex;
              local.changedColumns++;
              if (wasSolid && nextTop <= stockMin.z() + impl->epsilon)
                local.emptiedColumns++;
            }
          }
          if (violation.load(memory_order_relaxed) ||
              stop.load(memory_order_relaxed)) break;
        }
        }
      }
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
      double progress = min<uint64_t>
        (nextTile.load(memory_order_relaxed), tileCount) /
        (double)tileCount;
      if (!task->update(progress)) stop.store(true, memory_order_relaxed);
    }
    this_thread::sleep_for(chrono::milliseconds(20));
  }
  for (thread &worker: workers) worker.join();
  if (workerError) rethrow_exception(workerError);
  const auto rasterFinished = chrono::steady_clock::now();

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
    stats.tileMoveRefs += local.tileMoveRefs;
  }
  if (violation.load(memory_order_relaxed)) {
    result.reason = RejectionReason::MULTI_INTERVAL_UPDATE;
    return result;
  }
  if (stats.emptiedColumns) {
    result.reason = RejectionReason::EMPTY_COLUMN_UNSUPPORTED;
    return result;
  }

  vector<unsigned> publishedDirtyTiles;
  publishedDirtyTiles.reserve(tileCount);
  for (uint64_t tile = 0; tile < tileCount; tile++) {
    unsigned tileY = tile / impl->tileCols;
    unsigned tileX = tile % impl->tileCols;
    int x0 = tileX * impl->tileSize;
    int x1 = min<unsigned>(impl->nx, x0 + impl->tileSize - 1);
    int y0 = tileY * impl->tileSize;
    int y1 = min<unsigned>(impl->ny, y0 + impl->tileSize - 1);
    bool dirty = false;
    for (int y = y0; y <= y1 && !dirty; y++) {
      uint64_t row = (uint64_t)y * (impl->nx + 1);
      for (int x = x0; x <= x1; x++)
        if ((*impl->tops)[row + x] != nextTops[row + x]) {
          dirty = true;
          break;
        }
    }
    if (dirty) publishedDirtyTiles.push_back(tile);
  }
  const auto dirtyFinished = chrono::steady_clock::now();

  shared_ptr<vector<float> > snapshot =
    make_shared<vector<float> >(move(nextTops));
  shared_ptr<vector<unsigned> > ownerSnapshot;
  if (impl->boundaryOwners)
    ownerSnapshot =
      make_shared<vector<unsigned> >(move(nextBoundaryOwners));
  vector<pair<uint64_t, float> > partialUndo;
  vector<pair<uint64_t, unsigned> > partialOwnerUndo;
  if (targetPartialMove != noPreparedMove) {
    uint64_t undoEntries = 0;
    uint64_t ownerUndoEntries = 0;
    for (const auto &workerUndo: workerPartialUndo)
      undoEntries += workerUndo.size();
    for (const auto &workerUndo: workerPartialOwnerUndo)
      ownerUndoEntries += workerUndo.size();
    partialUndo.reserve(undoEntries);
    partialOwnerUndo.reserve(ownerUndoEntries);
    for (auto &workerUndo: workerPartialUndo)
      partialUndo.insert(partialUndo.end(), workerUndo.begin(),
                         workerUndo.end());
    for (auto &workerUndo: workerPartialOwnerUndo)
      partialOwnerUndo.insert(partialOwnerUndo.end(), workerUndo.begin(),
                              workerUndo.end());
  }

  vector<BoundaryTile> changedBoundaryTiles;
  vector<shared_ptr<const BoundaryTile> > nextBoundaryTiles;
  if (buildBoundary && ownerSnapshot) {
    const bool fullBoundary =
      impl->boundaryTiles.empty() || impl->boundaryTilesStale;
    changedBoundaryTiles = buildBoundaryTilesFor
      (publishedDirtyTiles, fullBoundary, snapshot, ownerSnapshot,
       targetTime, task);
    if (task && task->shouldQuit()) {
      result.reason = RejectionReason::CANCELLED;
      return result;
    }
    nextBoundaryTiles = impl->boundaryTiles;
    if (nextBoundaryTiles.empty())
      nextBoundaryTiles.resize(tileCount);
    for (BoundaryTile &tile: changedBoundaryTiles) {
      const unsigned tileID = tile.tile;
      if (tileID < nextBoundaryTiles.size())
        nextBoundaryTiles[tileID] =
          make_shared<BoundaryTile>(move(tile));
    }
  }

  if (task) task->begin("Publishing dexel state");
  if (task && !task->update(1)) {
    result.reason = RejectionReason::CANCELLED;
    return result;
  }

  impl->tops = snapshot;
  if (impl->boundaryOwners)
    impl->boundaryOwners = ownerSnapshot;
  if (buildBoundary && ownerSnapshot)
    impl->boundaryTiles = move(nextBoundaryTiles);
  if (buildBoundary && ownerSnapshot)
    impl->boundaryTilesStale = false;
  else if (ownerSnapshot && !publishedDirtyTiles.empty())
    impl->boundaryTilesStale = true;
  impl->currentTime = targetTime;
  impl->hasPartialBase = targetPartialMove != noPreparedMove;
  impl->partialStartTime = impl->hasPartialBase ?
    impl->prepared[targetPartialMove].move->getStartTime() : targetTime;
  impl->partialUndo = move(partialUndo);
  impl->partialOwnerUndo = move(partialOwnerUndo);
  const auto publishFinished = chrono::steady_clock::now();
  auto elapsedMS = [] (const auto &start, const auto &end) {
    return chrono::duration<double, milli>(end - start).count();
  };
  Profile::setMetric("dexel_state_current_bytes", getCurrentBytes());
  Profile::setMetric("dexel_state_checkpoint_count", getCheckpointCount());
  Profile::setMetric("dexel_state_checkpoint_bytes", getCheckpointBytes());
  Profile::setMetric("dexel_state_restored", restored ? 1 : 0);
  Profile::setMetric("dexel_state_restored_index", restoredCheckpoint + 1);
  Profile::setMetric("dexel_state_restored_time", fromTime);
  Profile::setMetric("dexel_state_resumed_partial", resumedPartial ? 1 : 0);
  Profile::setMetric("dexel_state_partial_undo_bytes",
                     impl->partialUndo.size() *
                     sizeof(impl->partialUndo.front()) +
                     impl->partialOwnerUndo.size() *
                     sizeof(impl->partialOwnerUndo.front()));
  Profile::setMetric("dexel_state_replayed_moves", replayedMoves);
  Profile::setMetric("dexel_state_dirty_tiles",
                     publishedDirtyTiles.size());
  Profile::setMetric("dexel_state_tile_move_refs", stats.tileMoveRefs);
  Profile::setMetric("dexel_state_setup_ms",
                     elapsedMS(seekStarted, rasterStarted));
  Profile::setMetric("dexel_state_raster_ms",
                     elapsedMS(rasterStarted, rasterFinished));
  Profile::setMetric("dexel_state_dirty_ms",
                     elapsedMS(rasterFinished, dirtyFinished));
  Profile::setMetric("dexel_state_publish_ms",
                     elapsedMS(dirtyFinished, publishFinished));
  Profile::setMetric("dexel_state_total_ms",
                     elapsedMS(seekStarted, publishFinished));
  LOG_INFO(1, "Dexel state update: target=" << targetTime
           << " restored_checkpoint=" << restoredCheckpoint + 1
           << " restored_time=" << fromTime
           << " resumed_partial=" << (resumedPartial ? 1 : 0)
           << " partial_undo_bytes="
           << impl->partialUndo.size() * sizeof(impl->partialUndo.front()) +
              impl->partialOwnerUndo.size() *
              sizeof(impl->partialOwnerUndo.front())
           << " setup_ms=" << elapsedMS(seekStarted, rasterStarted)
           << " raster_ms=" << elapsedMS(rasterStarted, rasterFinished)
           << " dirty_ms=" << elapsedMS(rasterFinished, dirtyFinished)
           << " publish_ms=" << elapsedMS(dirtyFinished, publishFinished)
           << " total_ms=" << elapsedMS(seekStarted, publishFinished)
           << " replayed_moves=" << replayedMoves
           << " dirty_tiles=" << publishedDirtyTiles.size()
           << " current_bytes=" << getCurrentBytes()
           << " checkpoints=" << getCheckpointCount()
           << " checkpoint_bytes=" << getCheckpointBytes());
  result.accepted = true;
  result.surface = new GridSurface
    (impl->sim, impl->nx, impl->ny, impl->xStep, impl->yStep, snapshot,
     publishedDirtyTiles, false);
  if (buildBoundary && ownerSnapshot)
    dynamic_cast<GridSurface *>(result.surface.get())->setBoundaryTiles
      (impl->boundaryTiles);
  result.state = shared_from_this();
  return result;
}
