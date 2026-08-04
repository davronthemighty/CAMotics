/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#pragma once

#include "SparseToolpathStages.h"

#include <cstddef>
#include <vector>


namespace cb {namespace JSON {class Sink;}}


namespace CAMotics { namespace SparseToolpath { namespace Internal {
  struct SparseSurfaceResult {
    cb::SmartPointer<Surface> surface;
    bool topologyAccepted = false;
    bool geometryAccepted = false;

    bool accepted() const {
      return topologyAccepted && geometryAccepted;
    }
  };


  size_t sparseTileCount(unsigned bins);
  unsigned tileEnd(unsigned start, unsigned length, unsigned bins);
  const std::vector<RegionPlanRegion> &
  getSurfaceOwnershipRegions(const RegionPlan &regionPlan);
  bool planNeedsFullCoverageFallback(const RegionPlan &regionPlan,
                                     double resolution);
  void writeBounds(cb::JSON::Sink &sink, const std::string &name,
                   const cb::Rectangle3D &bounds);

  cb::SmartPointer<Surface> renderFullBaseline(const Simulation &sim,
                                               unsigned threads);
  SparseSurfaceResult buildSparseCandidate
    (const Simulation &sim, const RegionPlan &regionPlan,
     const OwnershipBoundaryPlan &boundaryPlan, unsigned threads);

}}}
