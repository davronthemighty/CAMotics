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

#pragma once


#include "RenderMode.h"

#include <camotics/contour/ContourGenerator.h>
#include <camotics/contour/GridTreeRef.h>

#include <cbang/thread/Thread.h>
#include <cbang/thread/Condition.h>

#include <cstdint>


namespace CAMotics {
  struct RenderStats {
    uint64_t cellsVisited = 0;
    uint64_t cellsCulled = 0;
    uint64_t cellsContoured = 0;
    uint64_t triangles = 0;
    uint64_t vertexSamples = 0;
    uint64_t depthCalls = 0;
    uint64_t toolsweepDepthCalls = 0;
    uint64_t edgeChecks = 0;
    uint64_t edgeIntersections = 0;

    void add(const RenderStats &o) {
      cellsVisited += o.cellsVisited;
      cellsCulled += o.cellsCulled;
      cellsContoured += o.cellsContoured;
      triangles += o.triangles;
      vertexSamples += o.vertexSamples;
      depthCalls += o.depthCalls;
      toolsweepDepthCalls += o.toolsweepDepthCalls;
      edgeChecks += o.edgeChecks;
      edgeIntersections += o.edgeIntersections;
    }
  };


  class RenderJob : public cb::Thread {
    cb::Condition &condition;
    cb::SmartPointer<ContourGenerator> generator;

    FieldFunction &func;
    GridTreeRef tree;
    RenderStats stats;

  public:
    RenderJob(cb::Condition &condition, FieldFunction &func, RenderMode mode,
              const GridTreeRef &tree);

    double getProgress() {return generator->getProgress();}
    const RenderStats &getStats() const {return stats;}

    // From Thread
    void run() override;
    void stop() override;
  };
}
