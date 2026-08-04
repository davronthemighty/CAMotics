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

#include <camotics/sim/Simulation.h>

#include <cbang/SmartPointer.h>

#include <functional>
#include <iosfwd>
#include <string>
#include <vector>


namespace cb {namespace JSON {class Sink; class Value;}}

namespace CAMotics {
  class Task;
  class Surface;

  namespace SparseToolpath {
    extern const char *ARTIFACT_VERSION;
    extern const char *TOOLPATH_ARTIFACT;
    extern const char *REGION_PLAN_ARTIFACT;
    extern const char *OWNERSHIP_BOUNDARY_ARTIFACT;
    extern const char *REGION_SURFACE_ARTIFACT;
    extern const char *STITCHED_SURFACE_ARTIFACT;

    typedef std::function<void(cb::JSON::Sink &)> ExtraArtifactWriter;

    struct ArtifactContract {
      std::string simulationHash;
      std::string inputHash;
      std::string toolpathHash;
      std::string regionPlanHash;
      std::string ownershipBoundaryHash;
      std::string regionSurfaceHash;

      void write(cb::JSON::Sink &sink) const;
    };

    struct RegionPlanRegion {
      std::string id;
      std::string ownership;
      std::string role;
      cb::Rectangle3D bounds;
      unsigned tileX = 0;
      unsigned tileY = 0;
      unsigned tileWidth = 1;
      unsigned tileHeight = 1;
      double activeDepth = 0;
      uint64_t estimatedCells = 0;

      void read(const cb::JSON::Value &value);
      void write(cb::JSON::Sink &sink) const;
    };

    struct RegionPlan {
      std::string planner;
      std::string ownership;
      unsigned xyBins = 0;
      unsigned haloCells = 0;
      double halo = 0;
      uint64_t fullCells = 0;
      uint64_t activeCells = 0;
      uint64_t renderCells = 0;
      uint64_t skippedCells = 0;
      uint64_t toolSweepBBoxes = 0;
      uint64_t bboxTileRefs = 0;
      uint64_t toolpathFilteredTileRefs = 0;
      uint64_t targetRegionCells = 0;
      uint64_t adaptiveLeafCount = 0;
      uint64_t adaptiveActiveLeafCount = 0;
      uint64_t adaptiveSplitCount = 0;
      uint64_t adaptiveOwnershipSplitCount = 0;
      uint64_t adaptiveDepthSplitCount = 0;
      uint64_t adaptiveDensitySplitCount = 0;
      uint64_t adaptiveTargetSplitCount = 0;
      uint64_t adaptiveMaxLeafCells = 0;
      uint64_t adaptiveTargetExceededLeaves = 0;
      cb::Rectangle3D stockBounds;
      cb::Rectangle3D sweptBounds;
      std::vector<RegionPlanRegion> activeRegions;
      std::vector<RegionPlanRegion> renderRegions;
      std::vector<RegionPlanRegion> analyticRegions;

      void read(const cb::JSON::Value &value);
      void write(cb::JSON::Sink &sink) const;
    };

    struct OwnershipBoundaryPoint {
      double x = 0;
      double y = 0;
      double z = 0;
      unsigned gridX = 0;
      unsigned gridY = 0;

      void read(const cb::JSON::Value &value);
      void write(cb::JSON::Sink &sink) const;
    };

    struct OwnershipBoundaryEdge {
      std::string analyticRegionID;
      std::string adjacentOwnership;
      std::string adjacentRegionID;
      std::string side;

      void read(const cb::JSON::Value &value);
      void write(cb::JSON::Sink &sink) const;
    };

    struct OwnershipBoundaryLoop {
      std::string id;
      std::string plane;
      std::string role;
      double normalX = 0;
      double normalY = 0;
      double normalZ = 1;
      double coordinate = 0;
      double signedArea = 0;
      uint64_t rawVertices = 0;
      uint64_t closed = 1;
      uint64_t touchesStockBorder = 0;
      std::vector<OwnershipBoundaryPoint> vertices;
      std::vector<OwnershipBoundaryEdge> edges;

      void read(const cb::JSON::Value &value);
      void write(cb::JSON::Sink &sink) const;
    };

    struct OwnershipBoundaryPlan {
      std::string planner;
      std::string ownership;
      unsigned xyBins = 0;
      unsigned haloCells = 0;
      double halo = 0;
      uint64_t activeTiles = 0;
      uint64_t analyticTiles = 0;
      uint64_t boundaryEdges = 0;
      uint64_t openLoops = 0;
      uint64_t ambiguousVertices = 0;
      uint64_t rawVertices = 0;
      uint64_t contractedVertices = 0;
      cb::Rectangle3D stockBounds;
      std::vector<OwnershipBoundaryLoop> loops;

      void read(const cb::JSON::Value &value);
      void write(cb::JSON::Sink &sink) const;
    };

    struct SurfaceTopologyReport {
      uint64_t triangles = 0;
      uint64_t uniqueEdges = 0;
      uint64_t boundaryEdges = 0;
      uint64_t nonManifoldEdges = 0;
      uint64_t misorientedEdges = 0;
      uint64_t degenerateTriangles = 0;
      uint64_t duplicateTriangles = 0;
      uint64_t maxEdgeIncidence = 0;

      bool accepted() const;
      void write(cb::JSON::Sink &sink) const;
      void write(cb::JSON::Sink &sink, bool accepted) const;
    };

    cb::SmartPointer<Simulation>
    readSimulationArtifact(const std::string &filename,
                           const std::string &expectedKind);

    ArtifactContract readArtifactContract(const std::string &filename,
                                          const std::string &expectedKind);

    std::string computeToolpathHash(const Simulation &sim);
    std::string computeRegionPlanHash(const RegionPlan &plan);
    std::string computeOwnershipBoundaryHash
      (const OwnershipBoundaryPlan &plan);

    RegionPlan
    readRegionPlanArtifact(const std::string &filename,
                           cb::SmartPointer<Simulation> &sim);

    OwnershipBoundaryPlan
    readOwnershipBoundaryArtifact(const std::string &filename,
                                  cb::SmartPointer<Simulation> &sim);

    void writeSimulationArtifact(std::ostream &stream,
                                 const std::string &kind,
                                 const Simulation &sim,
                                 const ArtifactContract &contract,
                                 const ExtraArtifactWriter &extraWriter =
                                 ExtraArtifactWriter());

    uint64_t estimateGridCells(const cb::Rectangle3D &bounds,
                               double resolution, bool grow = true);

    SurfaceTopologyReport validateSurfaceTopology(const Surface &surface,
                                                   double tolerance,
                                                   Task *task = 0);
  }
}
