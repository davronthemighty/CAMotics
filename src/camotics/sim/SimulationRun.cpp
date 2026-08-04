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

#include "SimulationRun.h"
#include "Simulation.h"

#include <camotics/contour/CompositeSurface.h>
#include <camotics/contour/TriangleSurface.h>
#include <camotics/contour/GridTree.h>
#include <camotics/render/Renderer.h>
#include <camotics/sim/CutWorkpiece.h>
#include <camotics/Profile.h>

#include <gcode/Move.h>

#include <cbang/log/Logger.h>
#include <cbang/time/TimeInterval.h>
#include <cbang/time/Timer.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

using namespace cb;
using namespace CAMotics;


namespace {
  struct AdaptiveZPlan {
    bool valid = false;
    bool renderActiveGrid = false;
    Rectangle3D stock;
    Rectangle3D activeStock;
    double stockHeight = 0;
    double initialDepth = 0;
    double slabHeight = 0;
    double margin = 0;
    double sweptDepth = 0;
    double requiredDepth = 0;
    double activeDepth = 0;
    uint64_t totalSlabs = 0;
    uint64_t requiredSlabs = 0;
    uint64_t fullCells = 0;
    uint64_t activeCells = 0;
    uint64_t savedCells = 0;
  };


  struct AdaptiveZRegionPlan {
    bool valid = false;
    bool renderRegions = false;
    Rectangle3D stock;
    unsigned bins = 0;
    double stockHeight = 0;
    double initialDepth = 0;
    double slabHeight = 0;
    double margin = 0;
    uint64_t regionCount = 0;
    uint64_t touchedRegions = 0;
    uint64_t expandedRegions = 0;
    uint64_t untouchedRegions = 0;
    uint64_t renderedRegions = 0;
    uint64_t fullCells = 0;
    uint64_t globalActiveCells = 0;
    uint64_t regionalActiveCells = 0;
    uint64_t regionalRenderCells = 0;
    uint64_t savedVsFull = 0;
    uint64_t savedVsGlobal = 0;
    double averageActiveDepth = 0;
    double maxActiveDepth = 0;
    unsigned deepestRegion = 0;
    std::vector<double> activeDepths;
    std::vector<bool> touched;
  };


  uint64_t toMicrounits(double value) {
    if (value <= 0) return 0;
    return (uint64_t)std::llround(value * 1000000.0);
  }


  uint64_t estimateGridCells(const Rectangle3D &bounds, double resolution) {
    Grid grid(bounds.grow(resolution * 0.9), resolution);
    Vector3U steps = grid.getSteps();

    return (uint64_t)steps.x() * steps.y() * steps.z();
  }


  uint64_t estimateGridCellsExact(const Rectangle3D &bounds,
                                  double resolution) {
    Grid grid(bounds, resolution);
    Vector3U steps = grid.getSteps();

    return (uint64_t)steps.x() * steps.y() * steps.z();
  }


  uint64_t ceilDivDepth(double depth, double slabHeight) {
    if (depth <= 0 || slabHeight <= 0) return 0;
    return (uint64_t)std::ceil(depth / slabHeight);
  }


  AdaptiveZPlan computeAdaptiveZPlan(const Simulation &sim,
                                     const ToolSweep &sweep) {
    AdaptiveZPlan plan;
    if (!sim.adaptiveZSlabMetrics && !sim.adaptiveZRender) return plan;

    plan.stock = sim.workpiece.getBounds();
    if (plan.stock == Rectangle3D() || sim.resolution <= 0) return plan;

    plan.stockHeight = plan.stock.getHeight();
    if (plan.stockHeight <= 0) return plan;

    plan.initialDepth = sim.adaptiveZInitialDepth;
    if (plan.initialDepth <= 0) {
      plan.initialDepth = plan.stockHeight * 0.25;
      plan.initialDepth = std::max(plan.initialDepth, sim.resolution * 4);
    }

    plan.initialDepth = std::min(plan.initialDepth, plan.stockHeight);

    plan.slabHeight = sim.adaptiveZSlabHeight;
    if (plan.slabHeight <= 0) plan.slabHeight = plan.initialDepth;
    plan.slabHeight = std::min(plan.slabHeight, plan.stockHeight);

    plan.margin = sim.adaptiveZMargin;
    if (plan.margin <= 0) plan.margin = sim.resolution * 2;

    Rectangle3D sweepBounds = sweep.getBounds();
    if (sweepBounds != Rectangle3D())
      plan.sweptDepth =
        std::max(0.0, plan.stock.getMax().z() - sweepBounds.getMin().z());

    plan.requiredDepth = std::min(plan.stockHeight,
                                  plan.sweptDepth + plan.margin);
    plan.requiredDepth = std::max(plan.requiredDepth, plan.initialDepth);

    plan.totalSlabs = ceilDivDepth(plan.stockHeight, plan.slabHeight);
    plan.requiredSlabs = ceilDivDepth(plan.requiredDepth, plan.slabHeight);
    if (!plan.requiredSlabs) plan.requiredSlabs = 1;
    if (plan.totalSlabs && plan.totalSlabs < plan.requiredSlabs)
      plan.requiredSlabs = plan.totalSlabs;

    plan.activeDepth =
      std::min(plan.stockHeight, plan.requiredSlabs * plan.slabHeight);
    Vector3D activeMin(plan.stock.getMin().x(), plan.stock.getMin().y(),
                       plan.stock.getMax().z() - plan.activeDepth);
    plan.activeStock = Rectangle3D(activeMin, plan.stock.getMax());

    plan.fullCells = estimateGridCells(plan.stock, sim.resolution);
    plan.activeCells = estimateGridCells(plan.activeStock, sim.resolution);
    plan.savedCells =
      plan.activeCells < plan.fullCells ? plan.fullCells - plan.activeCells : 0;
    plan.renderActiveGrid = sim.adaptiveZRender && plan.savedCells;
    plan.valid = true;

    return plan;
  }


  unsigned clampRegionIndex(int index, unsigned count) {
    if (index < 0) return 0;
    if ((int)count <= index) return count - 1;
    return index;
  }


  double roundDepthToSlab(double depth, double slabHeight, double stockHeight) {
    if (depth <= 0) return 0;
    if (slabHeight <= 0) return std::min(depth, stockHeight);
    return std::min(stockHeight, ceilDivDepth(depth, slabHeight) * slabHeight);
  }


  Rectangle3D getRegionStock(const AdaptiveZRegionPlan &plan, unsigned x,
                             unsigned y, double halo = 0) {
    Vector3D min = plan.stock.getMin();
    Vector3D max = plan.stock.getMax();
    Vector3D dims = plan.stock.getDimensions();
    double xStep = dims.x() / plan.bins;
    double yStep = dims.y() / plan.bins;

    double x0 = min.x() + x * xStep;
    double x1 = x + 1 == plan.bins ? max.x() : min.x() + (x + 1) * xStep;
    double y0 = min.y() + y * yStep;
    double y1 = y + 1 == plan.bins ? max.y() : min.y() + (y + 1) * yStep;
    double depth = plan.activeDepths[y * plan.bins + x];
    double z0 = max.z() - depth;

    if (halo) {
      x0 = std::max(min.x(), x0 - halo);
      x1 = std::min(max.x(), x1 + halo);
      y0 = std::max(min.y(), y0 - halo);
      y1 = std::min(max.y(), y1 + halo);
    }

    return Rectangle3D(Vector3D(x0, y0, z0), Vector3D(x1, y1, max.z()));
  }


  AdaptiveZRegionPlan computeAdaptiveZRegionPlan
  (const Simulation &sim, const ToolSweep &sweep,
   const AdaptiveZPlan &globalPlan) {
    AdaptiveZRegionPlan plan;
    if (!sim.adaptiveZRegionBins) return plan;

    plan.stock = sim.workpiece.getBounds();
    if (plan.stock == Rectangle3D() || sim.resolution <= 0) return plan;

    Vector3D dims = plan.stock.getDimensions();
    plan.stockHeight = plan.stock.getHeight();
    if (dims.x() <= 0 || dims.y() <= 0 || plan.stockHeight <= 0) return plan;

    plan.bins = sim.adaptiveZRegionBins;
    plan.regionCount = (uint64_t)plan.bins * plan.bins;
    plan.initialDepth = sim.adaptiveZInitialDepth;
    if (plan.initialDepth <= 0) {
      plan.initialDepth = plan.stockHeight * 0.25;
      plan.initialDepth = std::max(plan.initialDepth, sim.resolution * 4);
    }
    plan.initialDepth = std::min(plan.initialDepth, plan.stockHeight);

    plan.slabHeight = sim.adaptiveZSlabHeight;
    if (plan.slabHeight <= 0) plan.slabHeight = plan.initialDepth;
    plan.slabHeight = std::min(plan.slabHeight, plan.stockHeight);

    plan.margin = sim.adaptiveZMargin;
    if (plan.margin <= 0) plan.margin = sim.resolution * 2;

    plan.activeDepths.assign(plan.regionCount, plan.initialDepth);
    plan.touched.assign(plan.regionCount, false);

    double xStep = dims.x() / plan.bins;
    double yStep = dims.y() / plan.bins;
    Vector3D min = plan.stock.getMin();
    Vector3D max = plan.stock.getMax();

    sweep.forEachBBox([&](const GCode::Move &, const Rectangle3D &bbox) {
      if (bbox.getMax().x() < min.x() || max.x() < bbox.getMin().x() ||
          bbox.getMax().y() < min.y() || max.y() < bbox.getMin().y())
        return;

      double sweptDepth = max.z() - bbox.getMin().z();
      if (sweptDepth + plan.margin <= 0) return;

      double requiredDepth =
        std::max(plan.initialDepth,
                 std::min(plan.stockHeight, sweptDepth + plan.margin));
      double activeDepth =
        roundDepthToSlab(requiredDepth, plan.slabHeight, plan.stockHeight);

      int minX = floor((bbox.getMin().x() - min.x()) / xStep);
      int maxX = floor((bbox.getMax().x() - min.x()) / xStep);
      int minY = floor((bbox.getMin().y() - min.y()) / yStep);
      int maxY = floor((bbox.getMax().y() - min.y()) / yStep);
      unsigned x0 = clampRegionIndex(minX, plan.bins);
      unsigned x1 = clampRegionIndex(maxX, plan.bins);
      unsigned y0 = clampRegionIndex(minY, plan.bins);
      unsigned y1 = clampRegionIndex(maxY, plan.bins);

      for (unsigned y = y0; y <= y1; y++)
        for (unsigned x = x0; x <= x1; x++) {
          unsigned index = y * plan.bins + x;
          plan.touched[index] = true;
          if (plan.activeDepths[index] < activeDepth)
            plan.activeDepths[index] = activeDepth;
        }
    });

    plan.fullCells = estimateGridCells(plan.stock, sim.resolution);
    plan.globalActiveCells =
      globalPlan.valid ? globalPlan.activeCells : plan.fullCells;

    double totalDepth = 0;
    double halo = sim.resolution * 2;
    for (unsigned y = 0; y < plan.bins; y++)
      for (unsigned x = 0; x < plan.bins; x++) {
        unsigned index = y * plan.bins + x;
        double depth = plan.activeDepths[index];
        totalDepth += depth;
        if (plan.maxActiveDepth < depth) {
          plan.maxActiveDepth = depth;
          plan.deepestRegion = index;
        }
        if (plan.touched[index]) plan.touchedRegions++;
        else plan.untouchedRegions++;
        if (plan.initialDepth < depth) plan.expandedRegions++;

        Rectangle3D region = getRegionStock(plan, x, y);
        plan.regionalActiveCells +=
          estimateGridCellsExact(region, sim.resolution);

        if (plan.touched[index]) {
          plan.renderedRegions++;
          Rectangle3D renderRegion = getRegionStock(plan, x, y, halo);
          plan.regionalRenderCells +=
            estimateGridCells(renderRegion, sim.resolution);
        }
      }

    plan.averageActiveDepth =
      plan.regionCount ? totalDepth / plan.regionCount : 0;
    plan.savedVsFull = plan.regionalActiveCells < plan.fullCells ?
      plan.fullCells - plan.regionalActiveCells : 0;
    plan.savedVsGlobal = plan.regionalActiveCells < plan.globalActiveCells ?
      plan.globalActiveCells - plan.regionalActiveCells : 0;
    plan.renderRegions = sim.adaptiveZRegionRender;
    plan.valid = true;

    return plan;
  }


  void recordAdaptiveZRegionMetrics(const AdaptiveZRegionPlan &plan) {
    if (!plan.valid) return;

    Profile::setMetric("adaptive_z_region_enabled", 1);
    Profile::setMetric("adaptive_z_region_render_enabled",
                       plan.renderRegions ? 1 : 0);
    Profile::setMetric("adaptive_z_region_bins", plan.bins);
    Profile::setMetric("adaptive_z_region_count", plan.regionCount);
    Profile::setMetric("adaptive_z_region_touched_regions",
                       plan.touchedRegions);
    Profile::setMetric("adaptive_z_region_untouched_regions",
                       plan.untouchedRegions);
    Profile::setMetric("adaptive_z_region_expanded_regions",
                       plan.expandedRegions);
    Profile::setMetric("adaptive_z_region_rendered_regions",
                       plan.renderedRegions);
    Profile::setMetric("adaptive_z_region_full_grid_cells_est",
                       plan.fullCells);
    Profile::setMetric("adaptive_z_region_global_active_cells_est",
                       plan.globalActiveCells);
    Profile::setMetric("adaptive_z_region_active_cells_est",
                       plan.regionalActiveCells);
    Profile::setMetric("adaptive_z_region_render_cells_est",
                       plan.regionalRenderCells);
    Profile::setMetric("adaptive_z_region_saved_cells_vs_full",
                       plan.savedVsFull);
    Profile::setMetric("adaptive_z_region_saved_cells_vs_global",
                       plan.savedVsGlobal);
    Profile::setMetric("adaptive_z_region_avg_active_depth_microunits",
                       toMicrounits(plan.averageActiveDepth));
    Profile::setMetric("adaptive_z_region_max_active_depth_microunits",
                       toMicrounits(plan.maxActiveDepth));
    Profile::setMetric("adaptive_z_region_deepest_region",
                       plan.deepestRegion);
  }


  void recordAdaptiveZSlabMetrics(const Simulation &sim,
                                  const AdaptiveZPlan &plan) {
    if (!plan.valid) return;

    Profile::setMetric("adaptive_z_enabled", 1);
    Profile::setMetric("adaptive_z_render_enabled",
                       plan.renderActiveGrid ? 1 : 0);
    Profile::setMetric("adaptive_z_stock_height_microunits",
                       toMicrounits(plan.stockHeight));
    Profile::setMetric("adaptive_z_initial_depth_microunits",
                       toMicrounits(plan.initialDepth));
    Profile::setMetric("adaptive_z_slab_height_microunits",
                       toMicrounits(plan.slabHeight));
    Profile::setMetric("adaptive_z_margin_microunits",
                       toMicrounits(plan.margin));
    Profile::setMetric("adaptive_z_swept_depth_microunits",
                       toMicrounits(plan.sweptDepth));
    Profile::setMetric("adaptive_z_required_depth_microunits",
                       toMicrounits(plan.requiredDepth));
    Profile::setMetric("adaptive_z_active_depth_microunits",
                       toMicrounits(plan.activeDepth));
    Profile::setMetric("adaptive_z_total_slabs", plan.totalSlabs);
    Profile::setMetric("adaptive_z_required_slabs", plan.requiredSlabs);
    Profile::setMetric("adaptive_z_requires_expansion",
                       plan.initialDepth < plan.requiredDepth ? 1 : 0);
    Profile::setMetric("adaptive_z_full_grid_cells_est", plan.fullCells);
    Profile::setMetric("adaptive_z_active_grid_cells_est", plan.activeCells);
    Profile::setMetric("adaptive_z_estimated_saved_cells", plan.savedCells);
  }


  void addQuad(TriangleSurface &surface,
               const Vector3F &a, const Vector3F &b,
               const Vector3F &c, const Vector3F &d) {
    Vector3F t1[3] = {a, b, c};
    Vector3F t2[3] = {a, c, d};
    surface.add(t1);
    surface.add(t2);
  }


  SmartPointer<TriangleSurface>
  createRegionalStockReconstruction(const AdaptiveZRegionPlan &plan) {
    SmartPointer<TriangleSurface> surface = new TriangleSurface;
    Vector3D min = plan.stock.getMin();
    Vector3D max = plan.stock.getMax();
    Vector3D dims = plan.stock.getDimensions();
    double xStep = dims.x() / plan.bins;
    double yStep = dims.y() / plan.bins;
    double z0 = min.z();
    double zTop = max.z();

    Vector3F p000(min.x(), min.y(), z0);
    Vector3F p100(max.x(), min.y(), z0);
    Vector3F p110(max.x(), max.y(), z0);
    Vector3F p010(min.x(), max.y(), z0);
    addQuad(*surface, p000, p010, p110, p100); // Bottom

    uint64_t flatTopTriangles = 0;
    uint64_t sideTriangles = 2;
    for (unsigned y = 0; y < plan.bins; y++)
      for (unsigned x = 0; x < plan.bins; x++) {
        unsigned index = y * plan.bins + x;
        double x0 = min.x() + x * xStep;
        double x1 = x + 1 == plan.bins ? max.x() : min.x() + (x + 1) * xStep;
        double y0 = min.y() + y * yStep;
        double y1 = y + 1 == plan.bins ? max.y() : min.y() + (y + 1) * yStep;
        double z1 = plan.touched[index] ? zTop - plan.activeDepths[index] :
          zTop;

        if (!plan.touched[index]) {
          addQuad(*surface, Vector3F(x0, y0, zTop), Vector3F(x1, y0, zTop),
                  Vector3F(x1, y1, zTop), Vector3F(x0, y1, zTop));
          flatTopTriangles += 2;
        }

        if (z1 <= z0) continue;

        if (!y) {
          addQuad(*surface, Vector3F(x0, y0, z0), Vector3F(x1, y0, z0),
                  Vector3F(x1, y0, z1), Vector3F(x0, y0, z1));
          sideTriangles += 2;
        }
        if (x + 1 == plan.bins) {
          addQuad(*surface, Vector3F(x1, y0, z0), Vector3F(x1, y1, z0),
                  Vector3F(x1, y1, z1), Vector3F(x1, y0, z1));
          sideTriangles += 2;
        }
        if (y + 1 == plan.bins) {
          addQuad(*surface, Vector3F(x1, y1, z0), Vector3F(x0, y1, z0),
                  Vector3F(x0, y1, z1), Vector3F(x1, y1, z1));
          sideTriangles += 2;
        }
        if (!x) {
          addQuad(*surface, Vector3F(x0, y1, z0), Vector3F(x0, y0, z0),
                  Vector3F(x0, y0, z1), Vector3F(x0, y1, z1));
          sideTriangles += 2;
        }
      }

    Profile::setMetric("adaptive_z_region_reconstructed_triangles",
                       surface->getTriangleCount());
    Profile::setMetric("adaptive_z_region_reconstructed_flat_top_triangles",
                       flatTopTriangles);
    Profile::setMetric("adaptive_z_region_reconstructed_side_triangles",
                       sideTriangles);
    Profile::setMetric("adaptive_z_region_reconstructed_step_triangles", 0);

    return surface;
  }


  SmartPointer<Surface> renderAdaptiveZRegions
  (Task &task, CutWorkpiece &cutWP, const Simulation &sim,
   const AdaptiveZRegionPlan &plan) {
    SmartPointer<CompositeSurface> composite = new CompositeSurface;
    double halo = sim.resolution * 2;
    uint64_t renderedTriangles = 0;

    {
      Profile::Scope scope("adaptive_z_region_render");
      for (unsigned y = 0; y < plan.bins && !task.shouldQuit(); y++)
        for (unsigned x = 0; x < plan.bins && !task.shouldQuit(); x++) {
          unsigned index = y * plan.bins + x;
          if (!plan.touched[index]) continue;

          Rectangle3D bbox = getRegionStock(plan, x, y, halo).grow
            (sim.resolution * 0.9);
          SmartPointer<GridTree> regionTree =
            new GridTree(Grid(bbox, sim.resolution));

          Renderer renderer(task);
          renderer.render(cutWP, *regionTree, bbox, sim.threads, sim.mode);
          if (task.shouldQuit()) break;

          SmartPointer<Surface> regionSurface = new TriangleSurface(*regionTree);
          renderedTriangles += regionSurface->getTriangleCount();
          if (regionSurface->getTriangleCount()) composite->add(regionSurface);
        }
    }

    Profile::setMetric("adaptive_z_region_rendered_triangles",
                       renderedTriangles);
    Profile::setMetric("adaptive_z_region_filter_seam_margin_microunits",
                       toMicrounits(halo));
    Profile::setMetric("adaptive_z_region_halo_passthrough_enabled", 1);
    composite->add(createRegionalStockReconstruction(plan));
    return composite;
  }


  SmartPointer<TriangleSurface>
  createAdaptiveZLowerStockSurface(const AdaptiveZPlan &plan) {
    if (!plan.renderActiveGrid || plan.activeDepth >= plan.stockHeight)
      return 0;

    double x0 = plan.stock.getMin().x();
    double y0 = plan.stock.getMin().y();
    double z0 = plan.stock.getMin().z();
    double x1 = plan.stock.getMax().x();
    double y1 = plan.stock.getMax().y();
    double z1 = plan.activeStock.getMin().z();

    if (z1 <= z0) return 0;

    SmartPointer<TriangleSurface> surface = new TriangleSurface;

    Vector3F p000(x0, y0, z0);
    Vector3F p100(x1, y0, z0);
    Vector3F p110(x1, y1, z0);
    Vector3F p010(x0, y1, z0);
    Vector3F p001(x0, y0, z1);
    Vector3F p101(x1, y0, z1);
    Vector3F p111(x1, y1, z1);
    Vector3F p011(x0, y1, z1);

    addQuad(*surface, p000, p010, p110, p100); // Bottom
    addQuad(*surface, p000, p100, p101, p001); // -Y side
    addQuad(*surface, p100, p110, p111, p101); // +X side
    addQuad(*surface, p110, p010, p011, p111); // +Y side
    addQuad(*surface, p010, p000, p001, p011); // -X side

    return surface;
  }
}


SimulationRun::SimulationRun(const Simulation &sim) : sim(sim), lastTime(-1) {}


SimulationRun::~SimulationRun() {}


SmartPointer<MoveLookup> SimulationRun::getMoveLookup() const {
  if (!sweep.isNull() && !sweep->getChange().isNull())
    return sweep->getChange();
  return sweep;
}


void SimulationRun::setEndTime(double endTime) {sim.time = endTime;}


SmartPointer<Surface> SimulationRun::compute(Task &task) {
  Rectangle3D bbox;
  AdaptiveZPlan adaptiveZPlan;
  AdaptiveZRegionPlan adaptiveZRegionPlan;

  double start = Timer::now();
  double simTime = std::min(sim.path->getTime(), sim.time);

  LOG_INFO(1, "Computing surface at " << TimeInterval(simTime));

  // Build full sweep once OR for each file
  if (sweep.isNull()) {
    Rectangle3D sweepQueryBounds;
    if (sim.toolSweepStockBounds)
      sweepQueryBounds = sim.workpiece.getBounds().grow(sim.resolution * 0.9);

    // GCode::Tool sweep
    sweep = new ToolSweep(sim.path, 0, std::numeric_limits<double>::max(),
                          sim.toolSweepXYBins, sim.toolSweepXYZBins,
                          sim.adaptiveZRegionBins ||
                          sim.adaptiveZRegionRender, sweepQueryBounds);
    adaptiveZPlan = computeAdaptiveZPlan(sim, *sweep);
    recordAdaptiveZSlabMetrics(sim, adaptiveZPlan);
    adaptiveZRegionPlan = computeAdaptiveZRegionPlan
      (sim, *sweep, adaptiveZPlan);
    recordAdaptiveZRegionMetrics(adaptiveZRegionPlan);

    // Bounds, increased a little
    bbox = (adaptiveZPlan.renderActiveGrid ?
            adaptiveZPlan.activeStock : sim.workpiece.getBounds()).grow
      (sim.resolution * 0.9);

    // Grid
    {
      Profile::Scope scope("grid_creation");
      tree = new GridTree(Grid(bbox, sim.resolution));
    }

  } else {
    double minTime = simTime;
    double maxTime = simTime;

    if (lastTime < minTime) minTime = lastTime;
    if (maxTime < lastTime) maxTime = lastTime;

    SmartPointer<MoveLookup> change =
      new ToolSweep(sim.path, minTime, maxTime, sim.toolSweepXYBins,
                    sim.toolSweepXYZBins, sim.adaptiveZRegionBins ||
                    sim.adaptiveZRegionRender);
    sweep->setChange(change);
    bbox = change->getBounds().grow(sim.resolution * 1.1);
  }

  // Set target time
  sweep->setEndTime(simTime);

  // Setup cut simulation
  CutWorkpiece cutWP(sweep, sim.workpiece);

  if (adaptiveZRegionPlan.renderRegions) {
    SmartPointer<Surface> surface =
      renderAdaptiveZRegions(task, cutWP, sim, adaptiveZRegionPlan);

    if (task.shouldQuit()) {
      sweep.release();
      tree.release();
      return 0;
    }

    lastTime = simTime;
    LOG_DEBUG(1, "Regional adaptive Z render time "
              << TimeInterval(Timer::now() - start));
    return surface;
  }

  // Render
  Renderer renderer(task);
  renderer.render(cutWP, *tree, bbox, sim.threads, sim.mode);

  if (task.shouldQuit()) {
    sweep.release();
    tree.release();
    return 0;
  }

  LOG_DEBUG(1, "Render time " << TimeInterval(Timer::now() - start));

  // Extract surface
  lastTime = simTime;
  {
    Profile::Scope scope("surface_extraction");
    SmartPointer<Surface> surface = new TriangleSurface(*tree);

    if (adaptiveZPlan.renderActiveGrid) {
      SmartPointer<TriangleSurface> lower =
        createAdaptiveZLowerStockSurface(adaptiveZPlan);

      Profile::setMetric("adaptive_z_reconstructed_lower_triangles",
                         lower.isNull() ? 0 : lower->getTriangleCount());

      if (!lower.isNull()) {
        SmartPointer<CompositeSurface> composite = new CompositeSurface;
        composite->add(surface);
        composite->add(lower);
        surface = composite;
      }
    }

    return surface;
  }
}
