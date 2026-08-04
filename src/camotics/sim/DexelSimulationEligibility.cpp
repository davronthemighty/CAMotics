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

#include <camotics/Profile.h>
#include <camotics/Task.h>

#include <gcode/Move.h>
#include <gcode/Tool.h>
#include <gcode/ToolPath.h>
#include <gcode/ToolTable.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <string>


using namespace std;
using namespace cb;
using namespace CAMotics;
using namespace CAMotics::Dexel;


namespace {
  EligibilityReport reject(RejectionReason reason, uint64_t moves,
                           uint64_t tools) {
    EligibilityReport report;
    report.reason = reason;
    report.movesChecked = moves;
    report.toolsChecked = tools;
    return report;
  }


  bool hasRotaryOrAuxAxes(const GCode::Move &move) {
    for (unsigned i = 3; i < 9; i++)
      if (move.getStart().getIndex(i) != 0 ||
          move.getEnd().getIndex(i) != 0)
        return true;
    return false;
  }


  bool finitePoint(const Vector3D &point) {
    return isfinite(point.x()) && isfinite(point.y()) &&
      isfinite(point.z());
  }


  bool validMove(const GCode::Move &move) {
    if (!finitePoint(move.getStartPt()) || !finitePoint(move.getEndPt()) ||
        !isfinite(move.getStartTime()) || !isfinite(move.getEndTime()) ||
        move.getEndTime() < move.getStartTime())
      return false;

    Vector3D delta = move.getEndPt() - move.getStartPt();
    return finitePoint(delta) &&
      isfinite(delta.x() * delta.x() + delta.y() * delta.y());
  }


  bool supportedTool(const GCode::Tool &tool) {
    switch (tool.getShape()) {
    case GCode::ToolShape::TS_CYLINDRICAL:
    case GCode::ToolShape::TS_CONICAL:
    case GCode::ToolShape::TS_SNUBNOSE:
      return true;

    default: return false;
    }
  }


  bool validTool(const GCode::Tool &tool) {
    if (!isfinite(tool.getLength()) || !isfinite(tool.getRadius()) ||
        !isfinite(tool.getSnubDiameter()) ||
        tool.getLength() <= 0 || tool.getRadius() <= 0)
      return false;
    if (tool.getShape() == GCode::ToolShape::TS_SNUBNOSE &&
        (tool.getSnubDiameter() < 0 ||
         tool.getRadius() < tool.getSnubDiameter() / 2))
      return false;
    double bottomRadius = tool.getShape() == GCode::ToolShape::TS_SNUBNOSE ?
      tool.getSnubDiameter() / 2 : 0;
    if (bottomRadius < tool.getRadius() &&
        !isfinite(tool.getLength() / (tool.getRadius() - bottomRadius)))
      return false;
    return true;
  }
}


bool Internal::gridDimension(double length, double resolution,
                             unsigned &cells) {
  if (!isfinite(length) || !isfinite(resolution) ||
      length <= 0 || resolution <= 0)
    return false;

  double value = ceil(length / resolution);
  const double maximum = min<double>
    (numeric_limits<int>::max() - 64,
     numeric_limits<unsigned>::max() - 64);
  if (!isfinite(value) || value < 1 || maximum < value) return false;
  cells = (unsigned)value;
  return true;
}


int Internal::clampGridIndex(double value, unsigned maximum) {
  value = floor(value);
  if (!(0 < value)) return 0;
  if (!isfinite(value) || maximum < value) return (int)maximum;
  return (int)value;
}


const char *Dexel::reasonName(RejectionReason reason) {
  switch (reason) {
  case RejectionReason::NONE: return "none";
  case RejectionReason::INVALID_SIMULATION: return "invalid_simulation";
  case RejectionReason::INVALID_WORKPIECE: return "invalid_workpiece";
  case RejectionReason::INVALID_RESOLUTION: return "invalid_resolution";
  case RejectionReason::INITIAL_SURFACE: return "initial_surface";
  case RejectionReason::UNSUPPORTED_RENDER_MODE:
    return "unsupported_render_mode";
  case RejectionReason::PARTIAL_TIME: return "partial_time";
  case RejectionReason::UNSUPPORTED_MOVE_TYPE:
    return "unsupported_move_type";
  case RejectionReason::INVALID_MOVE: return "invalid_move";
  case RejectionReason::ROTARY_OR_AUX_AXES: return "rotary_or_aux_axes";
  case RejectionReason::MISSING_TOOL: return "missing_tool";
  case RejectionReason::UNSUPPORTED_TOOL: return "unsupported_tool";
  case RejectionReason::INVALID_TOOL: return "invalid_tool";
  case RejectionReason::RASTERIZER_NOT_IMPLEMENTED:
    return "rasterizer_not_implemented";
  case RejectionReason::MULTI_INTERVAL_UPDATE:
    return "multi_interval_update";
  case RejectionReason::EMPTY_COLUMN_UNSUPPORTED:
    return "empty_column_unsupported";
  case RejectionReason::TOPOLOGY_VALIDATION:
    return "topology_validation";
  case RejectionReason::GEOMETRY_VALIDATION:
    return "geometry_validation";
  case RejectionReason::CANCELLED: return "cancelled";
  }

  return "unknown";
}


EligibilityReport Dexel::classify(const Simulation &sim, Task *task) {
  if (sim.path.isNull())
    return reject(RejectionReason::INVALID_SIMULATION, 0, 0);
  if (numeric_limits<unsigned>::max() < sim.path->size() ||
      !isfinite(sim.path->getTime()) || !isfinite(sim.time))
    return reject(RejectionReason::INVALID_SIMULATION, 0, 0);
  if (!sim.workpiece.isValid() ||
      !finitePoint(sim.workpiece.getMin()) ||
      !finitePoint(sim.workpiece.getMax()))
    return reject(RejectionReason::INVALID_WORKPIECE, 0, 0);
  if (!isfinite(sim.resolution) || sim.resolution <= 0)
    return reject(RejectionReason::INVALID_RESOLUTION, 0, 0);
  if (!sim.surface.isNull())
    return reject(RejectionReason::INITIAL_SURFACE, 0, 0);
  if (sim.mode != RenderMode::MCUBES_MODE)
    return reject(RejectionReason::UNSUPPORTED_RENDER_MODE, 0, 0);
  if (sim.time < sim.path->getTime())
    return reject(RejectionReason::PARTIAL_TIME, 0, 0);

  set<int> tools;
  uint64_t moves = 0;
  for (const GCode::Move &move: *sim.path) {
    moves++;
    if (task && !(moves & 4095) && task->shouldQuit())
      return reject(RejectionReason::CANCELLED, moves, tools.size());
    if (move.getType() != GCode::MoveType::MOVE_RAPID &&
        move.getType() != GCode::MoveType::MOVE_CUTTING)
      return reject(RejectionReason::UNSUPPORTED_MOVE_TYPE, moves,
                    tools.size());
    if (!validMove(move))
      return reject(RejectionReason::INVALID_MOVE, moves, tools.size());
    if (hasRotaryOrAuxAxes(move))
      return reject(RejectionReason::ROTARY_OR_AUX_AXES, moves,
                    tools.size());

    int toolNo = move.getTool();
    if (toolNo < 0) continue;
    if (!sim.path->getTools().has(toolNo))
      return reject(RejectionReason::MISSING_TOOL, moves, tools.size());
    if (!tools.insert(toolNo).second) continue;

    const GCode::Tool &tool = sim.path->getTools().get(toolNo);
    if (!supportedTool(tool))
      return reject(RejectionReason::UNSUPPORTED_TOOL, moves, tools.size());
    if (!validTool(tool))
      return reject(RejectionReason::INVALID_TOOL, moves, tools.size());
  }

  EligibilityReport report;
  report.eligible = true;
  report.movesChecked = moves;
  report.toolsChecked = tools.size();
  return report;
}


void Dexel::recordEligibilityMetrics(const EligibilityReport &report) {
  Profile::setMetric("dexel_eligibility_checked", 1);
  Profile::setMetric("dexel_eligibility_accepted", report.eligible ? 1 : 0);
  Profile::setMetric("dexel_eligibility_moves_checked", report.movesChecked);
  Profile::setMetric("dexel_eligibility_tools_checked", report.toolsChecked);
  Profile::setMetric(string("dexel_rejection_") + reasonName(report.reason),
                     report.eligible ? 0 : 1);
}


void Dexel::recordFallback(RejectionReason reason) {
  Profile::setMetric("dexel_fallback", 1);
  Profile::setMetric(string("dexel_fallback_") + reasonName(reason), 1);
}
