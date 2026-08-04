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
#include "ToolSweep.h"

#include <camotics/GeometrySafetyInternal.h>
#include <camotics/Profile.h>

#include <gcode/Move.h>
#include <gcode/ToolPath.h>

#include <cbang/Exception.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <string>
#include <vector>


using namespace std;
using namespace cb;
using namespace CAMotics;
using namespace CAMotics::SparseToolpath;
using namespace CAMotics::SparseToolpath::Internal;


namespace {
  unsigned tileIndex(double coordinate, double minimum, double step,
                     unsigned bins) {
    double index = floor((coordinate - minimum) / step);
    if (!isfinite(index) || index <= 0) return 0;
    if (bins - 1 <= index) return bins - 1;
    return (unsigned)index;
  }


  void incrementSaturated(uint64_t &value, uint64_t amount = 1) {
    value = numeric_limits<uint64_t>::max() - value < amount ?
      numeric_limits<uint64_t>::max() : value + amount;
  }


  string tileRegionID(const string &prefix, unsigned x, unsigned y) {
    return prefix + "-" + to_string(x) + "-" + to_string(y);
  }


  double pointSegmentDistanceSquared2D(double px, double py,
                                       double ax, double ay,
                                       double bx, double by) {
    double dx = bx - ax;
    double dy = by - ay;
    double length2 = dx * dx + dy * dy;
    if (length2 <= 1e-18) {
      double x = px - ax;
      double y = py - ay;
      return x * x + y * y;
    }

    double t = ((px - ax) * dx + (py - ay) * dy) / length2;
    t = max(0.0, min(1.0, t));
    double x = ax + t * dx - px;
    double y = ay + t * dy - py;
    return x * x + y * y;
  }


  double pointRectDistanceSquared2D(double px, double py, double x0,
                                    double y0, double x1, double y1) {
    double dx = 0;
    if (px < x0) dx = x0 - px;
    else if (x1 < px) dx = px - x1;

    double dy = 0;
    if (py < y0) dy = y0 - py;
    else if (y1 < py) dy = py - y1;

    return dx * dx + dy * dy;
  }


  double orientationValue2D(double ax, double ay, double bx, double by,
                            double cx, double cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
  }


  bool onSegment2D(double px, double py, double ax, double ay,
                   double bx, double by, double eps) {
    return fabs(orientationValue2D(ax, ay, bx, by, px, py)) <= eps &&
      min(ax, bx) - eps <= px && px <= max(ax, bx) + eps &&
      min(ay, by) - eps <= py && py <= max(ay, by) + eps;
  }


  bool segmentsIntersect2D(double ax, double ay, double bx, double by,
                           double cx, double cy, double dx, double dy,
                           double eps) {
    double o1 = orientationValue2D(ax, ay, bx, by, cx, cy);
    double o2 = orientationValue2D(ax, ay, bx, by, dx, dy);
    double o3 = orientationValue2D(cx, cy, dx, dy, ax, ay);
    double o4 = orientationValue2D(cx, cy, dx, dy, bx, by);

    if (((eps < o1 && o2 < -eps) || (o1 < -eps && eps < o2)) &&
        ((eps < o3 && o4 < -eps) || (o3 < -eps && eps < o4)))
      return true;

    return onSegment2D(cx, cy, ax, ay, bx, by, eps) ||
      onSegment2D(dx, dy, ax, ay, bx, by, eps) ||
      onSegment2D(ax, ay, cx, cy, dx, dy, eps) ||
      onSegment2D(bx, by, cx, cy, dx, dy, eps);
  }


  bool segmentIntersectsRect2D(double ax, double ay, double bx, double by,
                               double x0, double y0, double x1, double y1,
                               double eps) {
    if (x0 - eps <= ax && ax <= x1 + eps &&
        y0 - eps <= ay && ay <= y1 + eps)
      return true;
    if (x0 - eps <= bx && bx <= x1 + eps &&
        y0 - eps <= by && by <= y1 + eps)
      return true;

    return segmentsIntersect2D(ax, ay, bx, by, x0, y0, x1, y0, eps) ||
      segmentsIntersect2D(ax, ay, bx, by, x1, y0, x1, y1, eps) ||
      segmentsIntersect2D(ax, ay, bx, by, x1, y1, x0, y1, eps) ||
      segmentsIntersect2D(ax, ay, bx, by, x0, y1, x0, y0, eps);
  }


  double segmentRectDistanceSquared2D(double ax, double ay, double bx,
                                      double by, double x0, double y0,
                                      double x1, double y1) {
    double eps = 1e-12;
    if (segmentIntersectsRect2D(ax, ay, bx, by, x0, y0, x1, y1, eps))
      return 0;

    double best =
      min(pointRectDistanceSquared2D(ax, ay, x0, y0, x1, y1),
          pointRectDistanceSquared2D(bx, by, x0, y0, x1, y1));

    best = min(best, pointSegmentDistanceSquared2D(x0, y0, ax, ay, bx, by));
    best = min(best, pointSegmentDistanceSquared2D(x1, y0, ax, ay, bx, by));
    best = min(best, pointSegmentDistanceSquared2D(x1, y1, ax, ay, bx, by));
    best = min(best, pointSegmentDistanceSquared2D(x0, y1, ax, ay, bx, by));
    return best;
  }


  bool moveCapsuleIntersectsTile(const GCode::Move &move,
                                 double toolRadius, double halo,
                                 double tolerance, double x0, double y0,
                                 double x1, double y1) {
    Vector3D start = move.getStartPt();
    Vector3D end = move.getEndPt();
    double radius = max(0.0, toolRadius) + max(0.0, halo) + tolerance;
    double d2 = segmentRectDistanceSquared2D
      (start.x(), start.y(), end.x(), end.y(), x0, y0, x1, y1);
    return d2 <= radius * radius;
  }


}


RegionPlan SparseToolpath::planRegions
(const Simulation &toolpathSim, const RegionPlanOptions &options) {
  if (toolpathSim.path.isNull())
    THROW("Toolpath artifact has no path.");

  Rectangle3D stock = toolpathSim.workpiece.getBounds();
  if (stock == Rectangle3D())
    THROW("Toolpath artifact has no workpiece bounds.");
  if (!CAMotics::Internal::finiteBounds(stock))
    THROW("Toolpath artifact has non-finite workpiece bounds.");
  if (!isfinite(toolpathSim.resolution) || toolpathSim.resolution <= 0)
    THROW("Toolpath artifact has invalid resolution.");

  unsigned bins = options.xyBins ? options.xyBins : 1;
  size_t tileCount = sparseTileCount(bins);
  Vector3D stockMin = stock.getMin();
  Vector3D stockMax = stock.getMax();
  Vector3D dims = stock.getDimensions();
  double stockHeight = stock.getHeight();
  if (dims.x() <= 0 || dims.y() <= 0 || stockHeight <= 0)
    THROW("Toolpath artifact has invalid stock dimensions.");

  double xStep = dims.x() / bins;
  double yStep = dims.y() / bins;
  double halo = options.haloCells * toolpathSim.resolution;
  if (!isfinite(xStep) || !isfinite(yStep) || !isfinite(halo))
    THROW("Sparse plan dimensions are outside the numeric range.");
  vector<unsigned> activeDepthCells(tileCount, 0);
  vector<double> activeDepths(tileCount, 0);
  vector<uint64_t> activeMoveRefs(tileCount, 0);
  unsigned stockDepthCells = 0;
  if (!CAMotics::Internal::ceilToUnsigned
      (stockHeight, toolpathSim.resolution, stockDepthCells))
    THROW("Sparse stock depth exceeds the supported grid range.");

  auto depthToCells = [&] (double depth) {
    if (depth <= 0) return 0u;

    double activeDepth =
      min(stockHeight, max(depth, toolpathSim.resolution));
    double scaled = ceil(activeDepth / toolpathSim.resolution - 1e-12);
    if (!isfinite(scaled) || numeric_limits<unsigned>::max() < scaled)
      THROW("Sparse active depth exceeds the supported grid range.");
    unsigned cells = (unsigned)scaled;
    if (!cells) cells = 1;
    return min(cells, stockDepthCells);
  };

  ToolSweep sweep(toolpathSim.path, 0, numeric_limits<double>::max(),
                  toolpathSim.toolSweepXYBins, toolpathSim.toolSweepXYZBins,
                  true);

  uint64_t bboxCount = 0;
  uint64_t bboxTileRefs = 0;
  uint64_t toolpathFilteredTileRefs = 0;
  Rectangle3D sweptBounds;
  sweep.forEachBBox([&] (const GCode::Move &move, const Rectangle3D &bbox) {
    if (!CAMotics::Internal::finiteBounds(bbox) ||
        !CAMotics::Internal::finitePoint(move.getStartPt()) ||
        !CAMotics::Internal::finitePoint(move.getEndPt()))
      THROW("Tool sweep produced non-finite sparse bounds.");
    incrementSaturated(bboxCount);
    sweptBounds.add(bbox);

    double paddedMinX = bbox.getMin().x() - halo;
    double paddedMaxX = bbox.getMax().x() + halo;
    double paddedMinY = bbox.getMin().y() - halo;
    double paddedMaxY = bbox.getMax().y() + halo;
    if (!isfinite(paddedMinX) || !isfinite(paddedMaxX) ||
        !isfinite(paddedMinY) || !isfinite(paddedMaxY))
      THROW("Sparse halo bounds are outside the numeric range.");

    if (paddedMaxX < stockMin.x() || stockMax.x() < paddedMinX ||
        paddedMaxY < stockMin.y() || stockMax.y() < paddedMinY)
      return;

    unsigned x0 = tileIndex(paddedMinX, stockMin.x(), xStep, bins);
    unsigned x1 = tileIndex(paddedMaxX, stockMin.x(), xStep, bins);
    unsigned y0 = tileIndex(paddedMinY, stockMin.y(), yStep, bins);
    unsigned y1 = tileIndex(paddedMaxY, stockMin.y(), yStep, bins);
    incrementSaturated
      (bboxTileRefs, (uint64_t)(x1 - x0 + 1) * (y1 - y0 + 1));

    double toolRadius = 0;
    int tool = move.getTool();
    if (0 <= tool)
      toolRadius = toolpathSim.path->getTools().get((unsigned)tool).getRadius();
    if (!isfinite(toolRadius) || toolRadius < 0)
      THROW("Tool sweep contains an invalid tool radius.");
    double filterTolerance = max(0.01, toolpathSim.resolution * 0.01);

    double sweptDepth = max(0.0, stockMax.z() - bbox.getMin().z());
    double activeDepth = min(stockHeight, sweptDepth + halo);
    activeDepth = max(activeDepth, toolpathSim.resolution);

    for (unsigned y = y0; y <= y1; y++)
      for (unsigned x = x0; x <= x1; x++) {
        double tx0 = stockMin.x() + x * xStep;
        double tx1 = x + 1 == bins ? stockMax.x() : stockMin.x() + (x + 1) * xStep;
        double ty0 = stockMin.y() + y * yStep;
        double ty1 = y + 1 == bins ? stockMax.y() : stockMin.y() + (y + 1) * yStep;

        if (!moveCapsuleIntersectsTile(move, toolRadius, halo, filterTolerance,
                                       tx0, ty0, tx1, ty1)) {
          incrementSaturated(toolpathFilteredTileRefs);
          continue;
        }

        size_t index = (size_t)y * bins + x;
        incrementSaturated(activeMoveRefs[index]);
        unsigned &cells = activeDepthCells[index];
        double &depth = activeDepths[index];
        unsigned requiredCells = depthToCells(activeDepth);
        if (cells < requiredCells) cells = requiredCells;
        if (depth < activeDepth) depth = activeDepth;
      }
  });

  RegionPlan plan;
  plan.planner = "adaptive-xy-depth-halo";
  plan.ownership = "mc-active-analytic-untouched";
  plan.xyBins = bins;
  plan.haloCells = options.haloCells;
  plan.halo = halo;
  plan.stockBounds = stock;
  plan.sweptBounds = sweptBounds;
  plan.toolSweepBBoxes = bboxCount;
  plan.bboxTileRefs = bboxTileRefs;
  plan.toolpathFilteredTileRefs = toolpathFilteredTileRefs;
  plan.targetRegionCells = options.targetRegionCells;
  plan.fullCells = estimateGridCells(stock, toolpathSim.resolution);

  auto activeDepthCellsAt = [&] (unsigned x, unsigned y) -> unsigned & {
    return activeDepthCells[(size_t)y * bins + x];
  };
  auto activeDepthAt = [&] (unsigned x, unsigned y) -> double & {
    return activeDepths[(size_t)y * bins + x];
  };
  auto activeMoveRefsAt = [&] (unsigned x, unsigned y) -> uint64_t & {
    return activeMoveRefs[(size_t)y * bins + x];
  };
  auto tileXMin = [&] (unsigned x) {
    return stockMin.x() + x * xStep;
  };
  auto tileXMax = [&] (unsigned x) {
    return x + 1 == bins ? stockMax.x() : stockMin.x() + (x + 1) * xStep;
  };
  auto tileYMin = [&] (unsigned y) {
    return stockMin.y() + y * yStep;
  };
  auto tileYMax = [&] (unsigned y) {
    return y + 1 == bins ? stockMax.y() : stockMin.y() + (y + 1) * yStep;
  };

  function<void(unsigned, unsigned, unsigned, unsigned)> buildAdaptiveLeaves;
  buildAdaptiveLeaves =
    [&] (unsigned x, unsigned y, unsigned width, unsigned height) {
      uint64_t tileCount = (uint64_t)width * height;
      uint64_t activeCount = 0;
      unsigned minDepthCells = numeric_limits<unsigned>::max();
      unsigned maxDepthCells = 0;
      uint64_t minMoveRefs = numeric_limits<uint64_t>::max();
      uint64_t maxMoveRefs = 0;
      double activeDepth = 0;

      for (unsigned yy = y; yy < y + height; yy++)
        for (unsigned xx = x; xx < x + width; xx++) {
          unsigned depthCells = activeDepthCellsAt(xx, yy);
          if (!depthCells) continue;
          activeCount++;
          minDepthCells = min(minDepthCells, depthCells);
          maxDepthCells = max(maxDepthCells, depthCells);
          minMoveRefs = min(minMoveRefs, activeMoveRefsAt(xx, yy));
          maxMoveRefs = max(maxMoveRefs, activeMoveRefsAt(xx, yy));
          activeDepth = max(activeDepth, activeDepthAt(xx, yy));
        }

      bool active = activeCount == tileCount;
      bool inactive = !activeCount;
      bool mixedOwnership = !active && !inactive;
      bool mixedDepth = active && minDepthCells != maxDepthCells;
      bool densityVariation = active && minMoveRefs < maxMoveRefs &&
        max<uint64_t>(1, minMoveRefs) < maxMoveRefs - minMoveRefs;
      Rectangle3D activeBounds;
      uint64_t estimatedCells = 0;
      if (activeCount) {
        activeBounds = Rectangle3D
          (Vector3D(tileXMin(x), tileYMin(y),
                    stockMax.z() - activeDepth),
           Vector3D(tileXMax(x + width - 1),
                    tileYMax(y + height - 1), stockMax.z()));
        estimatedCells = estimateGridCells
          (activeBounds, toolpathSim.resolution, false);
      }
      bool targetExceeded = active && options.targetRegionCells &&
        options.targetRegionCells < estimatedCells;
      bool canSplit = 1 < width || 1 < height;

      if (canSplit && (mixedOwnership || mixedDepth || densityVariation ||
                       targetExceeded)) {
        plan.adaptiveSplitCount++;
        if (mixedOwnership) plan.adaptiveOwnershipSplitCount++;
        if (mixedDepth) plan.adaptiveDepthSplitCount++;
        if (densityVariation) plan.adaptiveDensitySplitCount++;
        if (targetExceeded) plan.adaptiveTargetSplitCount++;

        bool splitX = 1 < width &&
          (height == 1 || width * xStep >= height * yStep);
        if (splitX) {
          unsigned left = width / 2;
          buildAdaptiveLeaves(x, y, left, height);
          buildAdaptiveLeaves(x + left, y, width - left, height);
        } else {
          unsigned bottom = height / 2;
          buildAdaptiveLeaves(x, y, width, bottom);
          buildAdaptiveLeaves(x, y + bottom, width, height - bottom);
        }
        return;
      }

      plan.adaptiveLeafCount++;
      if (active) {
        plan.adaptiveActiveLeafCount++;
        plan.adaptiveMaxLeafCells =
          max(plan.adaptiveMaxLeafCells, estimatedCells);
        if (targetExceeded) plan.adaptiveTargetExceededLeaves++;

        RegionPlanRegion region;
        region.id = tileRegionID("adaptive-active", x, y);
        region.ownership = "mc";
        region.role = "adaptive-toolpath-halo";
        region.tileX = x;
        region.tileY = y;
        region.tileWidth = width;
        region.tileHeight = height;
        region.activeDepth = activeDepth;
        region.bounds = activeBounds;
        region.estimatedCells = estimatedCells;
        plan.activeCells += estimatedCells;
        plan.activeRegions.push_back(region);

      } else if (inactive) {
        RegionPlanRegion region;
        region.id = tileRegionID("adaptive-analytic", x, y);
        region.ownership = "analytic";
        region.role = "adaptive-untouched-stock";
        region.tileX = x;
        region.tileY = y;
        region.tileWidth = width;
        region.tileHeight = height;
        region.bounds = Rectangle3D
          (Vector3D(tileXMin(x), tileYMin(y), stockMin.z()),
           Vector3D(tileXMax(x + width - 1),
                    tileYMax(y + height - 1), stockMax.z()));
        region.estimatedCells = estimateGridCells
          (region.bounds, toolpathSim.resolution, false);
        plan.analyticRegions.push_back(region);
      }
    };

  buildAdaptiveLeaves(0, 0, bins, bins);

  vector<char> renderUsed(tileCount, false);
  auto renderUsedAt = [&] (unsigned x, unsigned y) -> char & {
    return renderUsed[(size_t)y * bins + x];
  };

  for (unsigned y = 0; y < bins; y++)
    for (unsigned x = 0; x < bins; x++) {
      if (!activeDepthCellsAt(x, y) || renderUsedAt(x, y)) continue;

      unsigned minX = x;
      unsigned maxX = x;
      unsigned minY = y;
      unsigned maxY = y;
      double activeDepth = 0;
      vector<size_t> stack;
      stack.push_back((size_t)y * bins + x);
      renderUsedAt(x, y) = true;

      while (!stack.empty()) {
        size_t tile = stack.back();
        stack.pop_back();
        unsigned tx = tile % bins;
        unsigned ty = tile / bins;

        minX = min(minX, tx);
        maxX = max(maxX, tx);
        minY = min(minY, ty);
        maxY = max(maxY, ty);
        activeDepth = max(activeDepth, activeDepthAt(tx, ty));

        for (int dy = -1; dy <= 1; dy++)
          for (int dx = -1; dx <= 1; dx++) {
            if (!dx && !dy) continue;

            int nx = (int)tx + dx;
            int ny = (int)ty + dy;
            if (nx < 0 || ny < 0 || (int)bins <= nx || (int)bins <= ny)
              continue;

            unsigned ux = (unsigned)nx;
            unsigned uy = (unsigned)ny;
            if (!activeDepthCellsAt(ux, uy) || renderUsedAt(ux, uy))
              continue;

            renderUsedAt(ux, uy) = true;
            stack.push_back((size_t)uy * bins + ux);
          }
      }

      RegionPlanRegion region;
      region.id = tileRegionID("render-component", minX, minY);
      region.ownership = "mc-render";
      region.role = "connected-active-component";
      region.tileX = minX;
      region.tileY = minY;
      region.tileWidth = maxX - minX + 1;
      region.tileHeight = maxY - minY + 1;
      region.activeDepth = activeDepth;
      region.bounds = Rectangle3D
        (Vector3D(tileXMin(minX), tileYMin(minY),
                  stockMax.z() - activeDepth),
         Vector3D(tileXMax(maxX), tileYMax(maxY), stockMax.z()));
      region.estimatedCells =
        estimateGridCells(region.bounds, toolpathSim.resolution, false);
      plan.renderCells += region.estimatedCells;
      plan.renderRegions.push_back(region);
    }

  plan.skippedCells =
    plan.activeCells < plan.fullCells ? plan.fullCells - plan.activeCells : 0;

  map<unsigned, uint64_t> depthGroups;
  uint64_t activeTiles = 0;
  unsigned maxActiveDepthCells = 0;
  for (unsigned cells: activeDepthCells)
    if (cells) {
      activeTiles++;
      depthGroups[cells]++;
      maxActiveDepthCells = max(maxActiveDepthCells, cells);
    }

  Profile::setMetric("sparse_region_plan_full_cells_est", plan.fullCells);
  Profile::setMetric("sparse_region_plan_active_cells_est", plan.activeCells);
  Profile::setMetric("sparse_region_plan_render_cells_est", plan.renderCells);
  Profile::setMetric("sparse_region_plan_skipped_cells_est",
                     plan.skippedCells);
  Profile::setMetric("sparse_region_plan_tool_sweep_bboxes", bboxCount);
  Profile::setMetric("sparse_region_plan_bbox_tile_refs", bboxTileRefs);
  Profile::setMetric("sparse_region_plan_toolpath_filtered_tile_refs",
                     toolpathFilteredTileRefs);
  Profile::setMetric("sparse_region_plan_active_regions",
                     plan.activeRegions.size());
  Profile::setMetric("sparse_region_plan_render_regions",
                     plan.renderRegions.size());
  Profile::setMetric("sparse_region_plan_analytic_regions",
                     plan.analyticRegions.size());
  Profile::setMetric("sparse_region_plan_active_tiles", activeTiles);
  Profile::setMetric("sparse_region_plan_active_depth_groups",
                     depthGroups.size());
  Profile::setMetric("sparse_region_plan_max_active_depth_cells",
                     maxActiveDepthCells);
  Profile::setMetric("sparse_region_plan_target_region_cells",
                     plan.targetRegionCells);
  Profile::setMetric("sparse_region_plan_adaptive_leaf_count",
                     plan.adaptiveLeafCount);
  Profile::setMetric("sparse_region_plan_adaptive_active_leaf_count",
                     plan.adaptiveActiveLeafCount);
  Profile::setMetric("sparse_region_plan_adaptive_split_count",
                     plan.adaptiveSplitCount);
  Profile::setMetric("sparse_region_plan_adaptive_ownership_split_count",
                     plan.adaptiveOwnershipSplitCount);
  Profile::setMetric("sparse_region_plan_adaptive_depth_split_count",
                     plan.adaptiveDepthSplitCount);
  Profile::setMetric("sparse_region_plan_adaptive_density_split_count",
                     plan.adaptiveDensitySplitCount);
  Profile::setMetric("sparse_region_plan_adaptive_target_split_count",
                     plan.adaptiveTargetSplitCount);
  Profile::setMetric("sparse_region_plan_adaptive_max_leaf_cells",
                     plan.adaptiveMaxLeafCells);
  Profile::setMetric("sparse_region_plan_adaptive_target_exceeded_leaves",
                     plan.adaptiveTargetExceededLeaves);

  return plan;
}
