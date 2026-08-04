/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>
  Copyright (C) 2026 davronthemighty

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

#include <camotics/render/RenderMode.h>
#include <camotics/sim/Simulation.h>
#include <camotics/sim/SparseToolpathArtifacts.h>

#include <cbang/SmartPointer.h>

#include <iosfwd>
#include <string>


namespace CAMotics {
  class Surface;

  namespace SparseToolpath {
    struct PathStageOptions {
      std::string input;
      std::string resolution;
      double time = 0;
      RenderMode renderMode = RenderMode::MCUBES_MODE;
      unsigned threads = 1;
      unsigned toolSweepXYBins = 0;
      unsigned toolSweepXYZBins = 0;
    };

    struct RegionPlanOptions {
      unsigned xyBins = 64;
      unsigned haloCells = 1;
      uint64_t targetRegionCells = 1000000;
    };

    RegionPlan planRegions(const Simulation &toolpathSim,
                           const RegionPlanOptions &options);

    OwnershipBoundaryPlan
    planOwnershipBoundaries(const RegionPlan &regionPlan);

    cb::SmartPointer<Surface>
    renderRegionSurface(const Simulation &toolpathSim,
                        const RegionPlan &regionPlan,
                        unsigned threads);

    cb::SmartPointer<Surface>
    stitchStockSurface(const RegionPlan &regionPlan,
                       const OwnershipBoundaryPlan *ownershipBoundary,
                       const Simulation &regionSurfaceSim);

    cb::SmartPointer<Surface>
    computeSparseSurface(const Simulation &sim,
                         const RegionPlanOptions &options,
                         unsigned threads);

    void writeToolpathArtifact(const PathStageOptions &options,
                               std::ostream &stream);

    void writeRegionPlanArtifact(const Simulation &toolpathSim,
                                 const RegionPlanOptions &options,
                                 const ArtifactContract &inputContract,
                                 std::ostream &stream);

    void writeOwnershipBoundaryArtifact(const Simulation &regionPlanSim,
                                        const RegionPlan &regionPlan,
                                        const ArtifactContract &inputContract,
                                        std::ostream &stream);

    cb::SmartPointer<Surface>
    renderRegionSurfaceArtifact(const Simulation &toolpathSim,
                                const RegionPlan &regionPlan,
                                const ArtifactContract &inputContract,
                                unsigned threads,
                                std::ostream &stream);

    void writeStitchedSurfaceArtifact
    (const RegionPlan &regionPlan, const Simulation &regionSurfaceSim,
     const ArtifactContract &inputContract,
     std::ostream &stream);

    void writeStitchedSurfaceArtifact
    (const RegionPlan &regionPlan,
     const OwnershipBoundaryPlan *ownershipBoundary,
     const Simulation &regionSurfaceSim,
     const ArtifactContract &inputContract, std::ostream &stream);

    void writeReduceExport(const Simulation &stitchedSim, std::ostream &stl,
                           bool binary, bool reduce);
  }
}
