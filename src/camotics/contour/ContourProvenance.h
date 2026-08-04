/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2026 Joseph Coffland
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#pragma once

#include <cbang/geom/Vector.h>

#include <cstdint>
#include <vector>


namespace CAMotics {
  struct ContourVertexProvenance {
    enum Kind : uint8_t {
      UNKNOWN,
      GRID_EDGE,
      CELL_CENTER,
    };

    Kind kind = UNKNOWN;
    uint8_t edge = 0;

    static ContourVertexProvenance gridEdge(uint8_t edge) {
      ContourVertexProvenance provenance;
      provenance.kind = GRID_EDGE;
      provenance.edge = edge;
      return provenance;
    }

    static ContourVertexProvenance cellCenter() {
      ContourVertexProvenance provenance;
      provenance.kind = CELL_CENTER;
      return provenance;
    }
  };


  struct ContourTriangleProvenance {
    enum Algorithm : uint8_t {
      UNKNOWN_ALGORITHM,
      MARCHING_CUBES,
      MC33,
    };

    cb::Vector3U cell;
    Algorithm algorithm = UNKNOWN_ALGORITHM;
    ContourVertexProvenance vertices[3];
  };


  struct ContourProvenanceReport {
    uint64_t expectedTriangles = 0;
    uint64_t triangles = 0;
    uint64_t completeTriangles = 0;
    uint64_t unknownTriangles = 0;
    uint64_t gridEdgeTriangles = 0;
    uint64_t cellCenterTriangles = 0;
    uint64_t rawBoundaryEdges = 0;
    uint64_t rawNonManifoldEdges = 0;
    uint64_t rawMisorientedEdges = 0;
    uint64_t rawUniqueEdges = 0;
    uint64_t rawMaxEdgeIncidence = 0;
    uint64_t rawEdgesIncidence1 = 0;
    uint64_t rawEdgesIncidence2 = 0;
    uint64_t rawEdgesIncidence3 = 0;
    uint64_t rawEdgesIncidence4 = 0;
    uint64_t rawEdgesIncidence5Plus = 0;
    uint64_t rawTwinEdgeSlots = 0;
    uint64_t rawBoundaryEdgeSlots = 0;
    uint64_t rawNonManifoldEdgeSlots = 0;
    uint64_t rawGridGridUniqueEdges = 0;
    uint64_t rawGridGridMaxEdgeIncidence = 0;
    uint64_t rawGridGridTwinEdgeSlots = 0;
    uint64_t rawGridGridBoundaryEdgeSlots = 0;
    uint64_t rawGridGridNonManifoldEdgeSlots = 0;
    uint64_t rawGridGridWeldedSpreadEdges = 0;
    uint64_t rawGridGridWeldedSpreadEdgeSlots = 0;
    uint64_t rawGridGridWeldedSpreadMaxAlternateSlots = 0;
    uint64_t rawCenterInvolvedUniqueEdges = 0;
    uint64_t rawCenterInvolvedMaxEdgeIncidence = 0;
    uint64_t rawCenterInvolvedTwinEdgeSlots = 0;
    uint64_t rawCenterInvolvedBoundaryEdgeSlots = 0;
    uint64_t rawCenterInvolvedNonManifoldEdgeSlots = 0;
    uint64_t rawCenterInvolvedWeldedSpreadEdges = 0;
    uint64_t rawCenterInvolvedWeldedSpreadEdgeSlots = 0;
    uint64_t rawCenterInvolvedWeldedSpreadMaxAlternateSlots = 0;
    uint64_t rawGridVertexUniqueKeys = 0;
    uint64_t rawGridVertexWeldedSpreadKeys = 0;
    uint64_t rawGridVertexWeldedSpreadObservations = 0;
    uint64_t rawGridVertexWeldedSpreadMaxAlternateObservations = 0;
    uint64_t rawCenterVertexUniqueKeys = 0;
    uint64_t rawCenterVertexWeldedSpreadKeys = 0;
    uint64_t rawCenterVertexWeldedSpreadObservations = 0;
    uint64_t rawCenterVertexWeldedSpreadMaxAlternateObservations = 0;
    uint64_t boundaryEdges = 0;
    uint64_t nonManifoldEdges = 0;
    uint64_t misorientedEdges = 0;
    uint64_t weldedUniqueEdges = 0;
    uint64_t weldedMaxEdgeIncidence = 0;
    uint64_t weldedEdgesIncidence1 = 0;
    uint64_t weldedEdgesIncidence2 = 0;
    uint64_t weldedEdgesIncidence3 = 0;
    uint64_t weldedEdgesIncidence4 = 0;
    uint64_t weldedEdgesIncidence5Plus = 0;
    uint64_t weldedTwinEdgeSlots = 0;
    uint64_t weldedBoundaryEdgeSlots = 0;
    uint64_t weldedNonManifoldEdgeSlots = 0;
    bool watertight = false;
  };


  ContourProvenanceReport analyzeContourProvenance
    (const std::vector<ContourTriangleProvenance> &provenance,
     const std::vector<float> &vertices, uint64_t startVertexOffset,
     uint64_t expectedTriangles, double tolerance = 1e-4);

  bool buildContourProvenanceNeighbors
    (const std::vector<ContourTriangleProvenance> &provenance,
     const std::vector<float> &vertices, uint64_t startVertexOffset,
     uint64_t expectedTriangles, std::vector<int32_t> &neighbors,
     double tolerance = 1e-4, bool *usedRawProvenance = 0);

  void setContourProvenanceCapture(bool value);
  bool shouldCaptureContourProvenance();

  void emitContourProvenanceMetrics(const ContourProvenanceReport &report);
}
