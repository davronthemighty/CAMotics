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

#include "RenderJob.h"

#include <camotics/contour/MarchingCubes.h>
#include <camotics/contour/CorrectedMC33.h>
#include <camotics/contour/CubicalMarchingSquares.h>
#include <camotics/Profile.h>

#include <cbang/Exception.h>
#include <cbang/time/Timer.h>
#include <cbang/Catch.h>

#include <sstream>
#include <thread>

using namespace std;
using namespace cb;
using namespace CAMotics;


RenderJob::RenderJob(Condition &condition, FieldFunction &func, RenderMode mode,
                     const GridTreeRef &tree) :
  condition(condition), func(func), tree(tree) {
  switch (mode) {
  case RenderMode::MCUBES_MODE: generator = new MarchingCubes;          break;
  case RenderMode::CMS_MODE:    generator = new CubicalMarchingSquares; break;
  default: THROW("Invalid or unsupported render mode " << mode);
  }
}


void RenderJob::run() {
  double start = Timer::now();
  Profile::setThreadCounterCapture(true);
  ProfileCounters before = Profile::getThreadCounters();

  try {
    generator->run(func, tree);
  } CATCH_WARNING;

  ProfileCounters after = Profile::getThreadCounters();
  Profile::setThreadCounterCapture(false);
  auto delta = [&](ProfileCounter counter) {
    return after.get(counter) - before.get(counter);
  };

  stats.cellsVisited = generator->getCellsVisited();
  stats.cellsCulled = generator->getCellsCulled();
  stats.cellsContoured = generator->getCellsContoured();
  stats.triangles = generator->getTriangles();
  stats.vertexSamples = delta(ProfileCounter::VERTEX_SAMPLES);
  stats.depthCalls = delta(ProfileCounter::DEPTH_CALLS);
  stats.toolsweepDepthCalls = delta(ProfileCounter::TOOLSWEEP_DEPTH_CALLS);
  stats.edgeChecks = delta(ProfileCounter::EDGE_CHECKS);
  stats.edgeIntersections = delta(ProfileCounter::EDGE_INTERSECTIONS);

  if (Profile::isEnabled()) {
    ostringstream threadId;
    threadId << this_thread::get_id();

    ProfileRenderJob job;
    job.thread = threadId.str();
    job.seconds = Timer::now() - start;
    job.cellsVisited = stats.cellsVisited;
    job.cellsCulled = stats.cellsCulled;
    job.cellsContoured = stats.cellsContoured;
    job.triangles = stats.triangles;
    job.vertexSamples = stats.vertexSamples;
    job.depthCalls = stats.depthCalls;
    job.edgeChecks = stats.edgeChecks;
    job.edgeIntersections = stats.edgeIntersections;
    job.linearIntersectIterations =
      delta(ProfileCounter::LINEAR_INTERSECT_ITERATIONS);
    job.contourGridEdgeTriangles =
      delta(ProfileCounter::CONTOUR_GRID_EDGE_TRIANGLES);
    job.contourCellCenterTriangles =
      delta(ProfileCounter::CONTOUR_CELL_CENTER_TRIANGLES);
    job.toolsweepDepthCalls = stats.toolsweepDepthCalls;
    job.toolsweepCollisionCandidates =
      delta(ProfileCounter::TOOLSWEEP_COLLISION_CANDIDATES);
    job.toolsweepSortedCandidates =
      delta(ProfileCounter::TOOLSWEEP_SORTED_CANDIDATES);
    job.toolsweepXYBinQueries =
      delta(ProfileCounter::TOOLSWEEP_XY_BIN_QUERIES);
    job.toolsweepXYBinOutOfBounds =
      delta(ProfileCounter::TOOLSWEEP_XY_BIN_OUT_OF_BOUNDS);
    job.toolsweepXYBinRefsScanned =
      delta(ProfileCounter::TOOLSWEEP_XY_BIN_REFS_SCANNED);
    job.toolsweepXYBinBBoxHits =
      delta(ProfileCounter::TOOLSWEEP_XY_BIN_BBOX_HITS);
    job.toolsweepXYZBinQueries =
      delta(ProfileCounter::TOOLSWEEP_XYZ_BIN_QUERIES);
    job.toolsweepXYZBinOutOfBounds =
      delta(ProfileCounter::TOOLSWEEP_XYZ_BIN_OUT_OF_BOUNDS);
    job.toolsweepXYZBinRefsScanned =
      delta(ProfileCounter::TOOLSWEEP_XYZ_BIN_REFS_SCANNED);
    job.toolsweepXYZBinBBoxHits =
      delta(ProfileCounter::TOOLSWEEP_XYZ_BIN_BBOX_HITS);
    job.aabbNodeVisits = delta(ProfileCounter::AABB_NODE_VISITS);
    job.aabbLeafHits = delta(ProfileCounter::AABB_LEAF_HITS);
    Profile::addRenderJob(job);
  }

  condition.signal();
}


void RenderJob::stop() {
  if (!generator.isNull()) generator->interrupt();
}
