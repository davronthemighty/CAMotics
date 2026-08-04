/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#include "SparseToolpathInternal.h"

#include <camotics/GeometrySafetyInternal.h>

#include <cbang/Exception.h>
#include <cbang/json/Sink.h>

#include <algorithm>


using namespace std;
using namespace cb;
using namespace CAMotics;
using namespace CAMotics::SparseToolpath;


namespace CAMotics { namespace SparseToolpath { namespace Internal {
  static const size_t MAX_SPARSE_PLANNING_TILES =
    (size_t)16 * 1024 * 1024;


  size_t sparseTileCount(unsigned bins) {
    size_t count = 0;
    if (!bins || !CAMotics::Internal::multiplySize(bins, bins, count) ||
        MAX_SPARSE_PLANNING_TILES < count)
      THROW("Sparse XY bin count exceeds the planning memory limit.");
    return count;
  }


  unsigned tileEnd(unsigned start, unsigned length, unsigned bins) {
    if (bins <= start) return bins;
    return bins - start < length ? bins : start + length;
  }


  const vector<RegionPlanRegion> &
  getSurfaceOwnershipRegions(const RegionPlan &regionPlan) {
    return regionPlan.renderRegions.empty() ?
      regionPlan.activeRegions : regionPlan.renderRegions;
  }


  bool planNeedsFullCoverageFallback(const RegionPlan &regionPlan,
                                     double resolution) {
    unsigned bins = regionPlan.xyBins;
    if (!bins || regionPlan.stockBounds == Rectangle3D()) return false;

    const vector<RegionPlanRegion> &regions =
      getSurfaceOwnershipRegions(regionPlan);
    if (regions.empty()) return false;

    vector<char> covered(sparseTileCount(bins), false);
    double minZ = regionPlan.stockBounds.getMax().z();
    for (const auto &region: regions) {
      minZ = min<double>(minZ, region.bounds.getMin().z());
      for (unsigned y = region.tileY;
           y < tileEnd(region.tileY, region.tileHeight, bins); y++)
        for (unsigned x = region.tileX;
             x < tileEnd(region.tileX, region.tileWidth, bins); x++)
          covered[(size_t)y * bins + x] = true;
    }

    for (char tileCovered: covered)
      if (!tileCovered) return false;

    double tolerance = max(1e-9, resolution * 0.5);
    return minZ <= regionPlan.stockBounds.getMin().z() + tolerance;
  }


  void writeBounds(JSON::Sink &sink, const string &name,
                   const Rectangle3D &bounds) {
    sink.beginInsert(name);
    bounds.write(sink);
  }
}}}
