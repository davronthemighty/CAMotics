/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

\******************************************************************************/

#include "ToolSweep.h"

#include "Sweep.h"
#include "ConicSweep.h"
#include "CompositeSweep.h"
#include "SpheroidSweep.h"

#include <camotics/Profile.h>

#include <gcode/ToolTable.h>

#include <cbang/log/Logger.h>
#include <cbang/time/TimeInterval.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <sstream>

using namespace std;
using namespace cb;
using namespace CAMotics;


namespace {
  const unsigned MAX_XY_BIN_COUNT = 512;
  const unsigned MAX_XYZ_BIN_COUNT = 64;


  struct ToolSetupStats {
    uint64_t moves = 0;
    uint64_t rapidMoves = 0;
    uint64_t cuttingMoves = 0;
    uint64_t otherMoves = 0;
    uint64_t boxes = 0;
    uint64_t unboundedBoxes = 0;
    uint64_t queryRejectedMoves = 0;
    double pathLength = 0;
    double bboxXYArea = 0;
    double unboundedBBoxXYArea = 0;
  };


  unsigned clampBin(int index, unsigned count) {
    if (index < 0) return 0;
    if ((int)count <= index) return count - 1;
    return index;
  }


  string toolMetricPrefix(int tool) {
    ostringstream out;
    out << "toolsweep_tool_" << tool << "_";
    return out.str();
  }


  const char *toolShapeName(const GCode::ToolShape &shape) {
    switch (shape) {
    case GCode::ToolShape::TS_CYLINDRICAL: return "cylindrical";
    case GCode::ToolShape::TS_CONICAL: return "conical";
    case GCode::ToolShape::TS_BALLNOSE: return "ballnose";
    case GCode::ToolShape::TS_SPHEROID: return "spheroid";
    case GCode::ToolShape::TS_SNUBNOSE: return "snubnose";
    }

    return "unknown";
  }


  uint64_t scaledMicrounits(double value) {
    if (value <= 0) return 0;
    if (!isfinite(value)) return numeric_limits<uint64_t>::max();
    const long double scaled = (long double)value * 1000000.0L;
    const long double maximum = numeric_limits<uint64_t>::max();
    if (maximum <= scaled) return numeric_limits<uint64_t>::max();
    return (uint64_t)floor(scaled + 0.5L);
  }
}


ToolSweep::ToolSweep(const SmartPointer<GCode::ToolPath> &path,
                     double startTime, double endTime, unsigned xyBinCount,
                     unsigned xyzBinCount, bool keepBBoxes,
                     const Rectangle3D &queryBounds) :
  path(path), startTime(startTime), endTime(endTime) {
  if (path.isNull()) THROW("ToolSweep requires a tool path");
  Profile::Scope scope("toolsweep_setup");
  if (MAX_XY_BIN_COUNT < xyBinCount) {
    LOG_WARNING("ToolSweep XY bin count " << xyBinCount
                << " exceeds maximum " << MAX_XY_BIN_COUNT
                << "; using " << MAX_XY_BIN_COUNT);
    xyBinCount = MAX_XY_BIN_COUNT;
  }
  if (1 < xyBinCount) this->xyBinCount = xyBinCount;
  if (MAX_XYZ_BIN_COUNT < xyzBinCount) {
    LOG_WARNING("ToolSweep XYZ bin count " << xyzBinCount
                << " exceeds maximum " << MAX_XYZ_BIN_COUNT
                << "; using " << MAX_XYZ_BIN_COUNT);
    xyzBinCount = MAX_XYZ_BIN_COUNT;
  }
  if (1 < xyzBinCount) this->xyzBinCount = xyzBinCount;

  if (endTime < startTime) {
    swap(startTime, endTime);
    swap(this->startTime, this->endTime);
  }

  unsigned boxes = 0;
  bool captureToolMetrics = Profile::isEnabled();
  bool boundedQuery = queryBounds != Rectangle3D();
  map<int, ToolSetupStats> toolStats;
  Profile::setMetric("toolsweep_query_bounds_enabled", boundedQuery ? 1 : 0);
  if (boundedQuery) {
    Vector3D queryDims = queryBounds.getDimensions();
    Profile::setMetric("toolsweep_query_bounds_x_microunits",
                       scaledMicrounits(queryDims.x()));
    Profile::setMetric("toolsweep_query_bounds_y_microunits",
                       scaledMicrounits(queryDims.y()));
    Profile::setMetric("toolsweep_query_bounds_z_microunits",
                       scaledMicrounits(queryDims.z()));
  }

  if (!path->empty()) {
    int firstMove = path->find(startTime);
    int lastMove = path->find(endTime);

    if (lastMove == -1) lastMove = path->size() - 1;
    if (firstMove == -1) firstMove = lastMove + 1;

    double duration = path->at(lastMove).getEndTime() - startTime;

    LOG_DEBUG(1, "Times: start=" << TimeInterval(startTime) << " end="
              << TimeInterval(startTime + duration) << " duration="
              << TimeInterval(duration));
    LOG_DEBUG(1, "GCode::Moves: first=" << firstMove << " last=" << lastMove);

    GCode::ToolTable &tools = path->getTools();
    vector<Rectangle3D> bboxes;
    vector<Rectangle3D> unboundedBBoxes;

    // Gather nodes in a list
    for (int i = firstMove; i <= lastMove; i++) {
      const GCode::Move &move = path->at(i);
      int tool = move.getTool();

      if (tool < 0) continue;
      if (sweeps.size() <= (unsigned)tool) sweeps.resize(tool + 1);
      if (sweeps[tool].isNull()) sweeps[tool] = getSweep(tools.get(tool));

      Vector3D startPt = move.getPtAtTime(startTime);
      Vector3D endPt = move.getPtAtTime(endTime);

      if (boundedQuery)
        sweeps[tool]->getBBoxesForQuery(startPt, endPt, queryBounds, bboxes);
      else sweeps[tool]->getBBoxes(startPt, endPt, bboxes);

      if (captureToolMetrics && boundedQuery)
        sweeps[tool]->getBBoxes(startPt, endPt, unboundedBBoxes);

      if (captureToolMetrics) {
        ToolSetupStats &stats = toolStats[tool];
        stats.moves++;
        stats.pathLength += startPt.distance(endPt);
        if (move.getType() == GCode::MoveType::MOVE_RAPID) stats.rapidMoves++;
        else if (move.getType() == GCode::MoveType::MOVE_CUTTING)
          stats.cuttingMoves++;
        else stats.otherMoves++;
        stats.boxes += bboxes.size();
        stats.unboundedBoxes += boundedQuery ?
          unboundedBBoxes.size() : bboxes.size();
        if (boundedQuery && bboxes.empty() && !unboundedBBoxes.empty())
          stats.queryRejectedMoves++;
        for (const Rectangle3D &bbox: bboxes) {
          Vector3D dims = bbox.getDimensions();
          stats.bboxXYArea += dims.x() * dims.y();
        }
        const vector<Rectangle3D> &fullBoxes = boundedQuery ?
          unboundedBBoxes : bboxes;
        for (const Rectangle3D &bbox: fullBoxes) {
          Vector3D dims = bbox.getDimensions();
          stats.unboundedBBoxXYArea += dims.x() * dims.y();
        }
      }

      for (unsigned j = 0; j < bboxes.size(); j++) {
        insert(&move, bboxes[j]);
        if (this->xyBinCount || this->xyzBinCount || keepBBoxes)
          binEntries.push_back({&move, bboxes[j]});
      }

      boxes += bboxes.size();
      bboxes.clear();
      unboundedBBoxes.clear();
    }
  }

  {
    Profile::Scope scope("toolsweep_aabb_finalize");
    AABBTree::finalize(); // Finalize MoveLookup
  }

  Profile::setMetric("toolsweep_aabb_boxes", boxes);
  Profile::setMetric("toolsweep_aabb_height", getHeight());

  for (const auto &entry: toolStats) {
    int toolNo = entry.first;
    const ToolSetupStats &stats = entry.second;
    const GCode::Tool &tool = path->getTools().get(toolNo);
    string prefix = toolMetricPrefix(toolNo);

    Profile::setMetric(prefix + "moves", stats.moves);
    Profile::setMetric(prefix + "rapid_moves", stats.rapidMoves);
    Profile::setMetric(prefix + "cutting_moves", stats.cuttingMoves);
    Profile::setMetric(prefix + "other_moves", stats.otherMoves);
    Profile::setMetric(prefix + "aabb_boxes", stats.boxes);
    Profile::setMetric(prefix + "unbounded_aabb_boxes",
                       stats.unboundedBoxes);
    Profile::setMetric(prefix + "query_rejected_moves",
                       stats.queryRejectedMoves);
    Profile::setMetric(prefix + "bbox_xy_area_scaled_1e6",
                       scaledMicrounits(stats.bboxXYArea));
    Profile::setMetric(prefix + "unbounded_bbox_xy_area_scaled_1e6",
                       scaledMicrounits(stats.unboundedBBoxXYArea));
    Profile::setMetric(prefix + "path_length_microunits",
                       scaledMicrounits(stats.pathLength));
    Profile::setMetric(prefix + "shape_" + toolShapeName(tool.getShape()), 1);
    Profile::setMetric(prefix + "radius_microunits",
                       scaledMicrounits(tool.getRadius()));
    Profile::setMetric(prefix + "length_microunits",
                       scaledMicrounits(tool.getLength()));
    Profile::setMetric(prefix + "snub_radius_microunits",
                       scaledMicrounits(tool.getSnubDiameter() / 2));
  }

  if (this->xyzBinCount) buildXYZBins();
  else if (this->xyBinCount) buildXYBins();

  LOG_DEBUG(1, "AABBTree boxes=" << boxes << " height=" << getHeight());
}


void ToolSweep::forEachBBox
(const function<void(const GCode::Move &, const Rectangle3D &)> &cb) const {
  for (const BinEntry &entry: binEntries) cb(*entry.move, entry.bbox);
}


bool ToolSweep::cull(const Rectangle3D &r) const {
  if (change.isNull()) return false;
  return !change->intersects(r);
}

double ToolSweep::depth(const Vector3D &p) const {
  Profile::count(ProfileCounter::TOOLSWEEP_DEPTH_CALLS);

  vector<const GCode::Move *> moves;
  if (xyzBinCount) collisionsXYZBins(p, moves);
  else if (xyBinCount) collisionsXYBins(p, moves);
  else collisions(p, moves);
  Profile::count(ProfileCounter::TOOLSWEEP_COLLISION_CANDIDATES,
                 moves.size());
  if (moves.empty())
    Profile::count(ProfileCounter::TOOLSWEEP_CANDIDATE_CALLS_0);
  else if (moves.size() == 1)
    Profile::count(ProfileCounter::TOOLSWEEP_CANDIDATE_CALLS_1);
  else if (moves.size() < 10)
    Profile::count(ProfileCounter::TOOLSWEEP_CANDIDATE_CALLS_2_9);
  else if (moves.size() < 100)
    Profile::count(ProfileCounter::TOOLSWEEP_CANDIDATE_CALLS_10_99);
  else if (moves.size() < 1000)
    Profile::count(ProfileCounter::TOOLSWEEP_CANDIDATE_CALLS_100_999);
  else
    Profile::count(ProfileCounter::TOOLSWEEP_CANDIDATE_CALLS_1000_PLUS);

  Profile::count(ProfileCounter::TOOLSWEEP_SORTED_CANDIDATES, 0);

  double d2 = -numeric_limits<double>::max();

  for (unsigned i = 0; i < moves.size(); i++) {
    const GCode::Move &move = *moves[i];

    if (move.getEndTime() < startTime || endTime < move.getStartTime())
      continue;

    Vector3D startPt = move.getPtAtTime(startTime);
    Vector3D endPt = move.getPtAtTime(endTime);

    double sd2 = sweeps[move.getTool()]->depth(startPt, endPt, p);
    if (0 <= sd2) return sd2; // Approx 5% faster
    if (d2 < sd2) d2 = sd2;
  }

  return d2;
}


SmartPointer<Sweep> ToolSweep::getSweep(const GCode::Tool &tool) {
  switch (tool.getShape()) {
  case GCode::ToolShape::TS_CYLINDRICAL:
    return new ConicSweep(tool.getLength(), tool.getRadius(), tool.getRadius());

  case GCode::ToolShape::TS_CONICAL:
    return new ConicSweep(tool.getLength(), tool.getRadius(), 0);

  case GCode::ToolShape::TS_BALLNOSE: {
    SmartPointer<CompositeSweep> composite = new CompositeSweep;
    composite->add
      (new SpheroidSweep(tool.getRadius(), 2 * tool.getRadius()), 0);
    composite->add(new ConicSweep(tool.getLength(), tool.getRadius(),
                                  tool.getRadius()), tool.getRadius());
    return composite;
  }

  case GCode::ToolShape::TS_SPHEROID:
    return new SpheroidSweep(tool.getRadius(), tool.getLength());

  case GCode::ToolShape::TS_SNUBNOSE:
    return new ConicSweep(tool.getLength(), tool.getRadius(),
                          tool.getSnubDiameter() / 2);
  }

  THROW("Invalid tool shape " << tool.getShape());
}


void ToolSweep::buildXYBins() {
  if (!xyBinCount || binEntries.empty()) return;

  xyBinBounds = getBounds();
  Vector3D dims = xyBinBounds.getDimensions();
  if (dims.x() <= 0 || dims.y() <= 0) {
    xyBinCount = 0;
    binEntries.clear();
    return;
  }

  xyBins.clear();
  xyBins.resize((size_t)xyBinCount * xyBinCount);

  double xStep = dims.x() / xyBinCount;
  double yStep = dims.y() / xyBinCount;
  uint64_t storedRefs = 0;
  uint64_t usedBins = 0;
  uint64_t maxBoxXSpan = 0;
  uint64_t maxBoxYSpan = 0;
  uint64_t maxBoxRefs = 0;
  bool captureToolMetrics = Profile::isEnabled();
  map<int, uint64_t> toolStoredRefs;
  map<int, uint64_t> toolMaxBoxRefs;

  for (unsigned i = 0; i < binEntries.size(); i++) {
    const Rectangle3D &bbox = binEntries[i].bbox;
    int minX = floor((bbox.getMin().x() - xyBinBounds.getMin().x()) / xStep);
    int maxX = floor((bbox.getMax().x() - xyBinBounds.getMin().x()) / xStep);
    int minY = floor((bbox.getMin().y() - xyBinBounds.getMin().y()) / yStep);
    int maxY = floor((bbox.getMax().y() - xyBinBounds.getMin().y()) / yStep);

    unsigned x0 = clampBin(minX, xyBinCount);
    unsigned x1 = clampBin(maxX, xyBinCount);
    unsigned y0 = clampBin(minY, xyBinCount);
    unsigned y1 = clampBin(maxY, xyBinCount);
    uint64_t xSpan = x1 - x0 + 1;
    uint64_t ySpan = y1 - y0 + 1;
    uint64_t boxRefs = xSpan * ySpan;

    maxBoxXSpan = max(maxBoxXSpan, xSpan);
    maxBoxYSpan = max(maxBoxYSpan, ySpan);
    maxBoxRefs = max(maxBoxRefs, boxRefs);
    if (captureToolMetrics) {
      int tool = binEntries[i].move->getTool();
      toolStoredRefs[tool] += boxRefs;
      toolMaxBoxRefs[tool] = max(toolMaxBoxRefs[tool], boxRefs);
    }

    for (unsigned y = y0; y <= y1; y++)
      for (unsigned x = x0; x <= x1; x++) {
        vector<unsigned> &bin = xyBins[y * xyBinCount + x];
        if (bin.empty()) usedBins++;
        bin.push_back(i);
        storedRefs++;
      }
  }

  uint64_t totalBins = (uint64_t)xyBinCount * xyBinCount;
  uint64_t emptyBins = totalBins - usedBins;
  uint64_t maxRefsPerBin = 0;
  uint64_t hotBins = 0;
  uint64_t avgRefsX1000 =
    usedBins ? (storedRefs * 1000 + usedBins / 2) / usedBins : 0;
  uint64_t hotThreshold =
    usedBins ? max<uint64_t>(16, (4 * storedRefs + usedBins - 1) / usedBins) :
    0;

  for (const vector<unsigned> &bin: xyBins) {
    uint64_t size = bin.size();
    maxRefsPerBin = max(maxRefsPerBin, size);
    if (hotThreshold && hotThreshold <= size) hotBins++;
  }

  Profile::setMetric("toolsweep_xy_bin_count", xyBinCount);
  Profile::setMetric("toolsweep_xy_bin_refs", storedRefs);
  Profile::setMetric("toolsweep_xy_bins_used", usedBins);
  Profile::setMetric("toolsweep_xy_bins_empty", emptyBins);
  Profile::setMetric("toolsweep_xy_bin_entries", binEntries.size());
  Profile::setMetric("toolsweep_xy_bin_max_refs_per_bin", maxRefsPerBin);
  Profile::setMetric("toolsweep_xy_bin_refs_per_used_bin_x1000",
                     avgRefsX1000);
  Profile::setMetric("toolsweep_xy_bin_hot_bins_4x_avg", hotBins);
  Profile::setMetric("toolsweep_xy_bin_hot_threshold", hotThreshold);
  Profile::setMetric("toolsweep_xy_bin_max_box_x_span", maxBoxXSpan);
  Profile::setMetric("toolsweep_xy_bin_max_box_y_span", maxBoxYSpan);
  Profile::setMetric("toolsweep_xy_bin_max_box_refs", maxBoxRefs);
  Profile::setMetric("toolsweep_xy_bin_refs_per_entry_x1000",
                     binEntries.empty() ? 0 :
                     (storedRefs * 1000 + binEntries.size() / 2) /
                     binEntries.size());

  for (const auto &entry: toolStoredRefs) {
    string prefix = toolMetricPrefix(entry.first);
    Profile::setMetric(prefix + "xy_bin_refs", entry.second);
    Profile::setMetric(prefix + "xy_bin_max_box_refs",
                       toolMaxBoxRefs[entry.first]);
  }
}


void ToolSweep::buildXYZBins() {
  if (!xyzBinCount || binEntries.empty()) return;

  xyzBinBounds = getBounds();
  Vector3D dims = xyzBinBounds.getDimensions();
  if (dims.x() <= 0 || dims.y() <= 0 || dims.z() <= 0) {
    xyzBinCount = 0;
    binEntries.clear();
    return;
  }

  uint64_t totalBins =
    (uint64_t)xyzBinCount * xyzBinCount * xyzBinCount;
  xyzBins.clear();
  xyzBins.resize(totalBins);

  double xStep = dims.x() / xyzBinCount;
  double yStep = dims.y() / xyzBinCount;
  double zStep = dims.z() / xyzBinCount;
  uint64_t storedRefs = 0;
  uint64_t usedBins = 0;
  uint64_t maxBoxXSpan = 0;
  uint64_t maxBoxYSpan = 0;
  uint64_t maxBoxZSpan = 0;
  uint64_t maxBoxRefs = 0;
  bool captureToolMetrics = Profile::isEnabled();
  map<int, uint64_t> toolStoredRefs;
  map<int, uint64_t> toolMaxBoxRefs;

  for (unsigned i = 0; i < binEntries.size(); i++) {
    const Rectangle3D &bbox = binEntries[i].bbox;
    int minX = floor((bbox.getMin().x() - xyzBinBounds.getMin().x()) / xStep);
    int maxX = floor((bbox.getMax().x() - xyzBinBounds.getMin().x()) / xStep);
    int minY = floor((bbox.getMin().y() - xyzBinBounds.getMin().y()) / yStep);
    int maxY = floor((bbox.getMax().y() - xyzBinBounds.getMin().y()) / yStep);
    int minZ = floor((bbox.getMin().z() - xyzBinBounds.getMin().z()) / zStep);
    int maxZ = floor((bbox.getMax().z() - xyzBinBounds.getMin().z()) / zStep);

    unsigned x0 = clampBin(minX, xyzBinCount);
    unsigned x1 = clampBin(maxX, xyzBinCount);
    unsigned y0 = clampBin(minY, xyzBinCount);
    unsigned y1 = clampBin(maxY, xyzBinCount);
    unsigned z0 = clampBin(minZ, xyzBinCount);
    unsigned z1 = clampBin(maxZ, xyzBinCount);
    uint64_t xSpan = x1 - x0 + 1;
    uint64_t ySpan = y1 - y0 + 1;
    uint64_t zSpan = z1 - z0 + 1;
    uint64_t boxRefs = xSpan * ySpan * zSpan;

    maxBoxXSpan = max(maxBoxXSpan, xSpan);
    maxBoxYSpan = max(maxBoxYSpan, ySpan);
    maxBoxZSpan = max(maxBoxZSpan, zSpan);
    maxBoxRefs = max(maxBoxRefs, boxRefs);
    if (captureToolMetrics) {
      int tool = binEntries[i].move->getTool();
      toolStoredRefs[tool] += boxRefs;
      toolMaxBoxRefs[tool] = max(toolMaxBoxRefs[tool], boxRefs);
    }

    for (unsigned z = z0; z <= z1; z++)
      for (unsigned y = y0; y <= y1; y++)
        for (unsigned x = x0; x <= x1; x++) {
          vector<unsigned> &bin =
            xyzBins[(z * xyzBinCount + y) * xyzBinCount + x];
          if (bin.empty()) usedBins++;
          bin.push_back(i);
          storedRefs++;
        }
  }

  uint64_t emptyBins = totalBins - usedBins;
  uint64_t maxRefsPerBin = 0;
  uint64_t hotBins = 0;
  uint64_t avgRefsX1000 =
    usedBins ? (storedRefs * 1000 + usedBins / 2) / usedBins : 0;
  uint64_t hotThreshold =
    usedBins ? max<uint64_t>(16, (4 * storedRefs + usedBins - 1) / usedBins) :
    0;

  for (const vector<unsigned> &bin: xyzBins) {
    uint64_t size = bin.size();
    maxRefsPerBin = max(maxRefsPerBin, size);
    if (hotThreshold && hotThreshold <= size) hotBins++;
  }

  Profile::setMetric("toolsweep_xyz_bin_count", xyzBinCount);
  Profile::setMetric("toolsweep_xyz_bin_refs", storedRefs);
  Profile::setMetric("toolsweep_xyz_bins_used", usedBins);
  Profile::setMetric("toolsweep_xyz_bins_empty", emptyBins);
  Profile::setMetric("toolsweep_xyz_bin_entries", binEntries.size());
  Profile::setMetric("toolsweep_xyz_bin_max_refs_per_bin", maxRefsPerBin);
  Profile::setMetric("toolsweep_xyz_bin_refs_per_used_bin_x1000",
                     avgRefsX1000);
  Profile::setMetric("toolsweep_xyz_bin_hot_bins_4x_avg", hotBins);
  Profile::setMetric("toolsweep_xyz_bin_hot_threshold", hotThreshold);
  Profile::setMetric("toolsweep_xyz_bin_max_box_x_span", maxBoxXSpan);
  Profile::setMetric("toolsweep_xyz_bin_max_box_y_span", maxBoxYSpan);
  Profile::setMetric("toolsweep_xyz_bin_max_box_z_span", maxBoxZSpan);
  Profile::setMetric("toolsweep_xyz_bin_max_box_refs", maxBoxRefs);
  Profile::setMetric("toolsweep_xyz_bin_refs_per_entry_x1000",
                     binEntries.empty() ? 0 :
                     (storedRefs * 1000 + binEntries.size() / 2) /
                     binEntries.size());

  for (const auto &entry: toolStoredRefs) {
    string prefix = toolMetricPrefix(entry.first);
    Profile::setMetric(prefix + "xyz_bin_refs", entry.second);
    Profile::setMetric(prefix + "xyz_bin_max_box_refs",
                       toolMaxBoxRefs[entry.first]);
  }
}


void ToolSweep::collisionsXYBins(const Vector3D &p,
                                 vector<const GCode::Move *> &moves) const {
  Profile::count(ProfileCounter::TOOLSWEEP_XY_BIN_QUERIES);

  if (!xyBinCount || !xyBinBounds.contains(p)) {
    Profile::count(ProfileCounter::TOOLSWEEP_XY_BIN_OUT_OF_BOUNDS);
    return;
  }

  Vector3D dims = xyBinBounds.getDimensions();
  unsigned x = clampBin
    (floor((p.x() - xyBinBounds.getMin().x()) / (dims.x() / xyBinCount)),
     xyBinCount);
  unsigned y = clampBin
    (floor((p.y() - xyBinBounds.getMin().y()) / (dims.y() / xyBinCount)),
     xyBinCount);

  const vector<unsigned> &bin = xyBins[y * xyBinCount + x];
  Profile::count(ProfileCounter::TOOLSWEEP_XY_BIN_REFS_SCANNED, bin.size());

  uint64_t hits = 0;
  for (unsigned i = 0; i < bin.size(); i++) {
    const BinEntry &entry = binEntries[bin[i]];
    if (entry.bbox.contains(p)) {
      moves.push_back(entry.move);
      hits++;
    }
  }

  Profile::count(ProfileCounter::TOOLSWEEP_XY_BIN_BBOX_HITS, hits);
}


void ToolSweep::collisionsXYZBins(const Vector3D &p,
                                  vector<const GCode::Move *> &moves) const {
  Profile::count(ProfileCounter::TOOLSWEEP_XYZ_BIN_QUERIES);

  if (!xyzBinCount || !xyzBinBounds.contains(p)) {
    Profile::count(ProfileCounter::TOOLSWEEP_XYZ_BIN_OUT_OF_BOUNDS);
    return;
  }

  Vector3D dims = xyzBinBounds.getDimensions();
  unsigned x = clampBin
    (floor((p.x() - xyzBinBounds.getMin().x()) / (dims.x() / xyzBinCount)),
     xyzBinCount);
  unsigned y = clampBin
    (floor((p.y() - xyzBinBounds.getMin().y()) / (dims.y() / xyzBinCount)),
     xyzBinCount);
  unsigned z = clampBin
    (floor((p.z() - xyzBinBounds.getMin().z()) / (dims.z() / xyzBinCount)),
     xyzBinCount);

  const vector<unsigned> &bin =
    xyzBins[(z * xyzBinCount + y) * xyzBinCount + x];
  Profile::count(ProfileCounter::TOOLSWEEP_XYZ_BIN_REFS_SCANNED, bin.size());

  uint64_t hits = 0;
  for (unsigned i = 0; i < bin.size(); i++) {
    const BinEntry &entry = binEntries[bin[i]];
    if (entry.bbox.contains(p)) {
      moves.push_back(entry.move);
      hits++;
    }
  }

  Profile::count(ProfileCounter::TOOLSWEEP_XYZ_BIN_BBOX_HITS, hits);
}
