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

#include <camotics/Application.h>
#include <camotics/Grid.h>
#include <camotics/sim/Simulation.h>
#include <camotics/sim/CamsimOptionsInternal.h>
#include <camotics/sim/CutSim.h>
#include <camotics/sim/SurfaceTask.h>
#include <camotics/sim/DexelSimulation.h>
#include <camotics/sim/DexelHeightMap.h>
#include <camotics/sim/SparseToolpathStages.h>
#include <camotics/sim/ToolSweep.h>
#include <camotics/project/Project.h>
#include <camotics/contour/ContourProvenance.h>
#include <camotics/contour/PlanarReduction.h>
#include <camotics/contour/Surface.h>
#include <camotics/Profile.h>

#include <gcode/Move.h>
#include <gcode/Tool.h>
#include <gcode/ToolShape.h>

#include <stl/Writer.h>

#include <cbang/Exception.h>
#include <cbang/ApplicationMain.h>
#include <cbang/log/Logger.h>
#include <cbang/os/SystemUtilities.h>
#include <cbang/config.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <vector>

#ifdef HAVE_V8
#include <cbang/js/v8/JSImpl.h>
#endif

using namespace cb;
using namespace std;
using namespace CAMotics;


namespace CAMotics {
  struct SurfaceStats {
    uint64_t triangles = 0;
    uint64_t binarySTLBytes = 0;
    uint64_t horizontal = 0;
    uint64_t nearHorizontal = 0;
    uint64_t axisAligned = 0;
    uint64_t nearAxisAligned = 0;
  };


  static double parseNonNegativeDouble(const string &name,
                                       const string &value) {
    if (value.empty()) return 0;

    double result = String::parseDouble(value);
    if (result < 0) THROW("--" << name << " must be non-negative.");
    return result;
  }


  static double parsePositiveDoubleOrDefault
    (const string &name, const string &value, double defaultValue) {
    if (value.empty()) return defaultValue;

    double result = String::parseDouble(value);
    if (result <= 0) THROW("--" << name << " must be positive.");
    return result;
  }


  static PlanarReductionConfig makePlanarReductionConfig
    (bool useProvenanceNeighbors, bool trustProvenanceNeighbors,
     bool applyHoleAware, bool applyBoundaryCoSimplify,
     const string &planeTolerance, const string &normalAngle,
     unsigned threads = 1) {
    PlanarReductionConfig config;
    config.useProvenanceNeighbors = useProvenanceNeighbors;
    config.trustProvenanceNeighbors = trustProvenanceNeighbors;
    config.applyHoleAware = applyHoleAware;
    config.applyBoundaryCoSimplify = applyBoundaryCoSimplify;
    config.threads = max(1U, threads);
    config.planeDistanceTolerance = parsePositiveDoubleOrDefault
      ("safe-reduce-plane-tolerance", planeTolerance,
       config.planeDistanceTolerance);
    config.pairwiseNormalAngleDegrees = parsePositiveDoubleOrDefault
      ("safe-reduce-normal-angle", normalAngle,
       config.pairwiseNormalAngleDegrees);

    if (5 < config.pairwiseNormalAngleDegrees)
      THROW("--safe-reduce-normal-angle must be <= 5 degrees.");

    return config;
  }


  static double getEffectiveCuttingDiameter(const GCode::Tool &tool) {
    if (tool.getShape() == GCode::ToolShape::TS_SNUBNOSE &&
        0 < tool.getSnubDiameter())
      return tool.getSnubDiameter();

    return tool.getDiameter();
  }


  static uint64_t scaledMetric(double value) {
    if (value <= 0) return 0;
    return (uint64_t)llround(value * 1000000);
  }


  static uint64_t estimateBinarySTLBytes(uint64_t triangles) {
    const uint64_t headerBytes = 84;
    const uint64_t triangleBytes = 50;
    if ((numeric_limits<uint64_t>::max() - headerBytes) / triangleBytes <
        triangles)
      return numeric_limits<uint64_t>::max();

    return headerBytes + triangleBytes * triangles;
  }


  static const char *planarReductionSideName(unsigned side) {
    switch (side) {
    case PLANAR_REDUCTION_SIDE_X_MIN: return "x_min";
    case PLANAR_REDUCTION_SIDE_X_MAX: return "x_max";
    case PLANAR_REDUCTION_SIDE_Y_MIN: return "y_min";
    case PLANAR_REDUCTION_SIDE_Y_MAX: return "y_max";
    case PLANAR_REDUCTION_SIDE_Z_MIN: return "z_min";
    case PLANAR_REDUCTION_SIDE_Z_MAX: return "z_max";
    case PLANAR_REDUCTION_SIDE_CUT: return "cut";
    default: return "unknown";
    }
  }


  static void emitPlanarReductionSideMetrics
    (const PlanarReductionReport &report) {
    for (unsigned i = 0; i < PLANAR_REDUCTION_SIDE_COUNT; i++) {
      const PlanarReductionSideReport &side = report.sides[i];
      string prefix =
        string("safe_reduce_side_") + planarReductionSideName(i) + "_";

      Profile::setMetric(prefix + "input_triangles", side.inputTriangles);
      Profile::setMetric(prefix + "component_triangles",
                         side.componentTriangles);
      Profile::setMetric(prefix + "zero_normal_triangles",
                         side.zeroNormalTriangles);
      Profile::setMetric(prefix + "degenerate_triangles",
                         side.degenerateTriangles);
      Profile::setMetric(prefix + "unaccounted_triangles",
                         side.unaccountedTriangles);
      Profile::setMetric(prefix + "components", side.components);
      Profile::setMetric(prefix + "single_triangle_components",
                         side.singleTriangleComponents);
      Profile::setMetric(prefix + "single_triangle_triangles",
                         side.singleTriangleTriangles);
      Profile::setMetric(prefix + "estimated_triangles_after",
                         side.estimatedTrianglesAfter);
      Profile::setMetric(prefix + "estimated_triangle_reduction",
                         side.estimatedTriangleReduction);
      Profile::setMetric(prefix + "output_triangles",
                         side.outputTriangles);
      Profile::setMetric(prefix + "phase1_components",
                         side.phase1Components);
      Profile::setMetric(prefix + "phase1_source_triangles",
                         side.phase1SourceTriangles);
      Profile::setMetric(prefix + "phase1_estimated_output_triangles",
                         side.phase1EstimatedOutputTriangles);
      Profile::setMetric(prefix + "phase1_estimated_reduction",
                         side.phase1EstimatedReduction);
      Profile::setMetric(prefix + "hole_aware_components",
                         side.holeAwareComponents);
      Profile::setMetric(prefix + "hole_aware_source_triangles",
                         side.holeAwareSourceTriangles);
      Profile::setMetric(prefix + "hole_aware_estimated_output_triangles",
                         side.holeAwareEstimatedOutputTriangles);
      Profile::setMetric(prefix + "hole_aware_estimated_reduction",
                         side.holeAwareEstimatedReduction);
      Profile::setMetric(prefix + "rejected_boundary_components",
                         side.rejectedBoundaryComponents);
      Profile::setMetric(prefix + "rejected_boundary_triangles",
                         side.rejectedBoundaryTriangles);
      Profile::setMetric(prefix + "rejected_no_savings_components",
                         side.rejectedNoSavingsComponents);
      Profile::setMetric(prefix + "rejected_no_savings_triangles",
                         side.rejectedNoSavingsTriangles);
      Profile::setMetric(prefix + "rejected_triangulation_components",
                         side.rejectedTriangulationComponents);
      Profile::setMetric(prefix + "rejected_triangulation_triangles",
                         side.rejectedTriangulationTriangles);
      Profile::setMetric(prefix + "applied_components",
                         side.appliedComponents);
      Profile::setMetric(prefix + "applied_source_triangles",
                         side.appliedSourceTriangles);
      Profile::setMetric(prefix + "applied_output_triangles",
                         side.appliedOutputTriangles);
      Profile::setMetric(prefix + "validation_rollback_components",
                         side.validationRollbackComponents);
      Profile::setMetric(prefix + "validation_rollback_source_triangles",
                         side.validationRollbackSourceTriangles);
      Profile::setMetric
        (prefix + "validation_rollback_candidate_output_triangles",
         side.validationRollbackCandidateOutputTriangles);
      Profile::setMetric(prefix + "boundary_cosimplify_candidate_components",
                         side.boundaryCoSimplifyCandidateComponents);
      Profile::setMetric(prefix + "boundary_cosimplify_source_triangles",
                         side.boundaryCoSimplifySourceTriangles);
      Profile::setMetric(prefix + "boundary_cosimplify_boundary_vertices",
                         side.boundaryCoSimplifyBoundaryVertices);
      Profile::setMetric
        (prefix + "boundary_cosimplify_simplified_boundary_vertices",
         side.boundaryCoSimplifySimplifiedBoundaryVertices);
      Profile::setMetric
        (prefix + "boundary_cosimplify_estimated_triangles_after",
         side.boundaryCoSimplifyEstimatedTrianglesAfter);
      Profile::setMetric
        (prefix + "boundary_cosimplify_estimated_triangles_after_simplified",
         side.boundaryCoSimplifyEstimatedTrianglesAfterSimplified);
      Profile::setMetric
        (prefix + "boundary_cosimplify_estimated_extra_reduction",
         side.boundaryCoSimplifyEstimatedExtraReduction);
    }
  }


  static void logPlanarReductionSideReport
    (const PlanarReductionReport &report) {
    for (unsigned i = 0; i < PLANAR_REDUCTION_SIDE_COUNT; i++) {
      const PlanarReductionSideReport &side = report.sides[i];
      LOG_INFO(2, "Safe reduction side "
               << planarReductionSideName(i)
               << ": input_triangles=" << side.inputTriangles
               << " components=" << side.components
               << " component_triangles=" << side.componentTriangles
               << " single_triangle_components="
               << side.singleTriangleComponents
               << " estimated_triangles_after="
               << side.estimatedTrianglesAfter
               << " estimated_reduction="
               << side.estimatedTriangleReduction
               << " rejected_boundary_components="
               << side.rejectedBoundaryComponents
               << " rejected_no_savings_components="
               << side.rejectedNoSavingsComponents
               << " rejected_triangulation_components="
               << side.rejectedTriangulationComponents
               << " applied_components=" << side.appliedComponents
               << " validation_rollback_components="
               << side.validationRollbackComponents
               << " unaccounted_triangles="
               << side.unaccountedTriangles);
    }
  }


  static uint64_t estimateCells(const Rectangle3D &bounds,
                                double resolution, bool grow) {
    if (bounds == Rectangle3D() || resolution <= 0) return 0;

    Grid grid(grow ? bounds.grow(resolution * 0.9) : bounds, resolution);
    Vector3U steps = grid.getSteps();
    return (uint64_t)steps.x() * steps.y() * steps.z();
  }


  static uint64_t ceilDepth(double depth, double slabHeight) {
    if (depth <= 0 || slabHeight <= 0) return 0;
    return (uint64_t)ceil(depth / slabHeight);
  }


  static double roundDepth(double depth, double slabHeight,
                           double stockHeight) {
    if (depth <= 0) return 0;
    if (slabHeight <= 0) return min(depth, stockHeight);
    return min(stockHeight, ceilDepth(depth, slabHeight) * slabHeight);
  }


  static Rectangle3D makeTopStock(const Rectangle3D &stock,
                                  double activeDepth) {
    Vector3D min(stock.getMin().x(), stock.getMin().y(),
                 stock.getMax().z() - activeDepth);
    return Rectangle3D(min, stock.getMax());
  }


  static Rectangle3D makeRegionStock(const Rectangle3D &stock, unsigned bins,
                                     unsigned x, unsigned y,
                                     double depth, double halo) {
    Vector3D stockMin = stock.getMin();
    Vector3D stockMax = stock.getMax();
    Vector3D dims = stock.getDimensions();
    double xStep = dims.x() / bins;
    double yStep = dims.y() / bins;

    double x0 = stockMin.x() + x * xStep;
    double x1 =
      x + 1 == bins ? stockMax.x() : stockMin.x() + (x + 1) * xStep;
    double y0 = stockMin.y() + y * yStep;
    double y1 =
      y + 1 == bins ? stockMax.y() : stockMin.y() + (y + 1) * yStep;

    if (halo) {
      x0 = std::max(stockMin.x(), x0 - halo);
      x1 = std::min(stockMax.x(), x1 + halo);
      y0 = std::max(stockMin.y(), y0 - halo);
      y1 = std::min(stockMax.y(), y1 + halo);
    }

    return Rectangle3D(Vector3D(x0, y0, stockMax.z() - depth),
                       Vector3D(x1, y1, stockMax.z()));
  }


  struct RegionalAdviceStats {
    bool valid = false;
    unsigned bins = 0;
    uint64_t regionCount = 0;
    uint64_t touchedRegions = 0;
    uint64_t expandedRegions = 0;
    uint64_t activeCells = 0;
    uint64_t renderCells = 0;
    uint64_t savedVsFull = 0;
    uint64_t savedVsGlobal = 0;
    uint64_t renderSavedVsFull = 0;
    uint64_t renderSavedVsGlobal = 0;
    bool memoryFit = false;
    bool runtimeFit = false;
    double initialDepth = 0;
    double slabHeight = 0;
    double margin = 0;
    double regionWidthX = 0;
    double regionWidthY = 0;
  };


  static void emitPlanarReductionMetrics(const PlanarReductionReport &report) {
    uint64_t inputBinarySTLBytes =
      estimateBinarySTLBytes(report.triangles);
    uint64_t outputBinarySTLBytes =
      estimateBinarySTLBytes(report.outputTriangles);
    uint64_t estimatedBinarySTLBytes =
      estimateBinarySTLBytes(report.estimatedTrianglesAfter);

    Profile::setMetric("safe_reduce_coord_tolerance_scaled_1e6",
                       scaledMetric(report.coordTolerance));
    Profile::setMetric
      ("safe_reduce_plane_distance_tolerance_scaled_1e6",
       scaledMetric(report.planeDistanceTolerance));
    Profile::setMetric
      ("safe_reduce_pairwise_normal_angle_millidegrees",
       (uint64_t)llround(report.pairwiseNormalAngleDegrees * 1000));
    Profile::setMetric("safe_reduce_input_triangles", report.triangles);
    Profile::setMetric("safe_reduce_analysis_triangle_records",
                       report.analysisTriangleRecords);
    Profile::setMetric("safe_reduce_analysis_adjacency_slots",
                       report.analysisAdjacencySlots);
    Profile::setMetric("safe_reduce_analysis_edge_records",
                       report.analysisEdgeRecords);
    Profile::setMetric("safe_reduce_output_triangles", report.outputTriangles);
    Profile::setMetric("safe_reduce_input_binary_stl_bytes",
                       inputBinarySTLBytes);
    Profile::setMetric("safe_reduce_output_binary_stl_bytes",
                       outputBinarySTLBytes);
    Profile::setMetric("safe_reduce_output_binary_stl_bytes_saved",
                       outputBinarySTLBytes < inputBinarySTLBytes ?
                       inputBinarySTLBytes - outputBinarySTLBytes : 0);
    Profile::setMetric("safe_reduce_estimated_binary_stl_bytes_after",
                       estimatedBinarySTLBytes);
    Profile::setMetric("safe_reduce_estimated_binary_stl_bytes_saved",
                       estimatedBinarySTLBytes < inputBinarySTLBytes ?
                       inputBinarySTLBytes - estimatedBinarySTLBytes : 0);
    Profile::setMetric("safe_reduce_components", report.components);
    Profile::setMetric("safe_reduce_component_records_retained",
                       report.componentRecordsRetained);
    Profile::setMetric("safe_reduce_component_records_skipped",
                       report.componentRecordsSkipped);
    Profile::setMetric("safe_reduce_plane_fit_cache_slots",
                       report.planeFitCacheSlots);
    Profile::setMetric("safe_reduce_plane_fit_cache_threads",
                       report.planeFitCacheThreads);
    Profile::setMetric("safe_reduce_input_boundary_edges",
                       report.globalBoundaryEdges);
    Profile::setMetric("safe_reduce_input_nonmanifold_edges",
                       report.globalNonManifoldEdges);
    Profile::setMetric("safe_reduce_input_misoriented_edges",
                       report.globalMisorientedEdges);
    Profile::setMetric("safe_reduce_input_degenerate_triangles",
                       report.globalDegenerateTriangles);
    Profile::setMetric("safe_reduce_input_watertight",
                       report.watertightInput ? 1 : 0);
    Profile::setMetric("safe_reduce_source_expected_floats",
                       report.sourceExpectedFloats);
    Profile::setMetric("safe_reduce_source_vertex_floats",
                       report.sourceVertexFloats);
    Profile::setMetric("safe_reduce_source_normal_floats",
                       report.sourceNormalFloats);
    Profile::setMetric("safe_reduce_source_vertex_count_mismatch",
                       report.sourceVertexCountMismatch ? 1 : 0);
    Profile::setMetric("safe_reduce_source_normal_count_mismatch",
                       report.sourceNormalCountMismatch ? 1 : 0);
    Profile::setMetric("safe_reduce_source_invalid_coordinates",
                       report.sourceInvalidCoordinates);
    Profile::setMetric("safe_reduce_source_range_mismatch",
                       report.sourceRangeMismatch ? 1 : 0);
    Profile::setMetric("safe_reduce_output_boundary_edges",
                       report.outputBoundaryEdges);
    Profile::setMetric("safe_reduce_output_nonmanifold_edges",
                       report.outputNonManifoldEdges);
    Profile::setMetric("safe_reduce_output_misoriented_edges",
                       report.outputMisorientedEdges);
    Profile::setMetric("safe_reduce_output_degenerate_triangles",
                       report.outputDegenerateTriangles);
    Profile::setMetric("safe_reduce_output_watertight",
                       report.watertightOutput ? 1 : 0);
    Profile::setMetric("safe_reduce_contour_provenance_available",
                       report.contourProvenanceAvailable ? 1 : 0);
    Profile::setMetric("safe_reduce_contour_provenance_triangles",
                       report.contourProvenanceTriangles);
    Profile::setMetric("safe_reduce_contour_provenance_complete_triangles",
                       report.contourProvenanceCompleteTriangles);
    Profile::setMetric("safe_reduce_contour_provenance_unknown_triangles",
                       report.contourProvenanceUnknownTriangles);
    Profile::setMetric("safe_reduce_contour_provenance_raw_boundary_edges",
                       report.contourProvenanceRawBoundaryEdges);
    Profile::setMetric("safe_reduce_contour_provenance_raw_nonmanifold_edges",
                       report.contourProvenanceRawNonManifoldEdges);
    Profile::setMetric("safe_reduce_contour_provenance_raw_misoriented_edges",
                       report.contourProvenanceRawMisorientedEdges);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_unique_edges",
       report.contourProvenanceRawUniqueEdges);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_max_edge_incidence",
       report.contourProvenanceRawMaxEdgeIncidence);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_edges_incidence_1",
       report.contourProvenanceRawEdgesIncidence1);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_edges_incidence_2",
       report.contourProvenanceRawEdgesIncidence2);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_edges_incidence_3",
       report.contourProvenanceRawEdgesIncidence3);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_edges_incidence_4",
       report.contourProvenanceRawEdgesIncidence4);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_edges_incidence_5_plus",
       report.contourProvenanceRawEdgesIncidence5Plus);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_twin_edge_slots",
       report.contourProvenanceRawTwinEdgeSlots);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_boundary_edge_slots",
       report.contourProvenanceRawBoundaryEdgeSlots);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_nonmanifold_edge_slots",
       report.contourProvenanceRawNonManifoldEdgeSlots);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_grid_grid_unique_edges",
       report.contourProvenanceRawGridGridUniqueEdges);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_grid_grid_max_edge_incidence",
       report.contourProvenanceRawGridGridMaxEdgeIncidence);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_grid_grid_twin_edge_slots",
       report.contourProvenanceRawGridGridTwinEdgeSlots);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_grid_grid_boundary_edge_slots",
       report.contourProvenanceRawGridGridBoundaryEdgeSlots);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_grid_grid_nonmanifold_edge_slots",
       report.contourProvenanceRawGridGridNonManifoldEdgeSlots);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_grid_grid_welded_spread_edges",
       report.contourProvenanceRawGridGridWeldedSpreadEdges);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_grid_grid_welded_spread_edge_slots",
       report.contourProvenanceRawGridGridWeldedSpreadEdgeSlots);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_grid_grid_welded_spread_max_alternate_slots",
       report.contourProvenanceRawGridGridWeldedSpreadMaxAlternateSlots);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_center_involved_unique_edges",
       report.contourProvenanceRawCenterInvolvedUniqueEdges);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_center_involved_max_edge_incidence",
       report.contourProvenanceRawCenterInvolvedMaxEdgeIncidence);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_center_involved_twin_edge_slots",
       report.contourProvenanceRawCenterInvolvedTwinEdgeSlots);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_center_involved_boundary_edge_slots",
       report.contourProvenanceRawCenterInvolvedBoundaryEdgeSlots);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_center_involved_nonmanifold_edge_slots",
       report.contourProvenanceRawCenterInvolvedNonManifoldEdgeSlots);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_center_involved_welded_spread_edges",
       report.contourProvenanceRawCenterInvolvedWeldedSpreadEdges);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_center_involved_welded_spread_edge_slots",
       report.contourProvenanceRawCenterInvolvedWeldedSpreadEdgeSlots);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_center_involved_welded_spread_max_alternate_slots",
       report.contourProvenanceRawCenterInvolvedWeldedSpreadMaxAlternateSlots);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_grid_vertex_unique_keys",
       report.contourProvenanceRawGridVertexUniqueKeys);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_grid_vertex_welded_spread_keys",
       report.contourProvenanceRawGridVertexWeldedSpreadKeys);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_grid_vertex_welded_spread_observations",
       report.contourProvenanceRawGridVertexWeldedSpreadObservations);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_grid_vertex_welded_spread_max_alternate_observations",
       report.contourProvenanceRawGridVertexWeldedSpreadMaxAlternateObservations);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_center_vertex_unique_keys",
       report.contourProvenanceRawCenterVertexUniqueKeys);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_center_vertex_welded_spread_keys",
       report.contourProvenanceRawCenterVertexWeldedSpreadKeys);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_center_vertex_welded_spread_observations",
       report.contourProvenanceRawCenterVertexWeldedSpreadObservations);
    Profile::setMetric
      ("safe_reduce_contour_provenance_raw_center_vertex_welded_spread_max_alternate_observations",
       report.contourProvenanceRawCenterVertexWeldedSpreadMaxAlternateObservations);
    Profile::setMetric("safe_reduce_contour_provenance_boundary_edges",
                       report.contourProvenanceBoundaryEdges);
    Profile::setMetric("safe_reduce_contour_provenance_nonmanifold_edges",
                       report.contourProvenanceNonManifoldEdges);
    Profile::setMetric("safe_reduce_contour_provenance_misoriented_edges",
                       report.contourProvenanceMisorientedEdges);
    Profile::setMetric
      ("safe_reduce_contour_provenance_welded_unique_edges",
       report.contourProvenanceWeldedUniqueEdges);
    Profile::setMetric
      ("safe_reduce_contour_provenance_welded_max_edge_incidence",
       report.contourProvenanceWeldedMaxEdgeIncidence);
    Profile::setMetric
      ("safe_reduce_contour_provenance_welded_edges_incidence_1",
       report.contourProvenanceWeldedEdgesIncidence1);
    Profile::setMetric
      ("safe_reduce_contour_provenance_welded_edges_incidence_2",
       report.contourProvenanceWeldedEdgesIncidence2);
    Profile::setMetric
      ("safe_reduce_contour_provenance_welded_edges_incidence_3",
       report.contourProvenanceWeldedEdgesIncidence3);
    Profile::setMetric
      ("safe_reduce_contour_provenance_welded_edges_incidence_4",
       report.contourProvenanceWeldedEdgesIncidence4);
    Profile::setMetric
      ("safe_reduce_contour_provenance_welded_edges_incidence_5_plus",
       report.contourProvenanceWeldedEdgesIncidence5Plus);
    Profile::setMetric
      ("safe_reduce_contour_provenance_welded_twin_edge_slots",
       report.contourProvenanceWeldedTwinEdgeSlots);
    Profile::setMetric
      ("safe_reduce_contour_provenance_welded_boundary_edge_slots",
       report.contourProvenanceWeldedBoundaryEdgeSlots);
    Profile::setMetric
      ("safe_reduce_contour_provenance_welded_nonmanifold_edge_slots",
       report.contourProvenanceWeldedNonManifoldEdgeSlots);
    Profile::setMetric("safe_reduce_contour_provenance_watertight",
                       report.contourProvenanceWatertight ? 1 : 0);
    Profile::setMetric("safe_reduce_contour_provenance_matches_input",
                       report.contourProvenanceMatchesInput ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_contour_provenance_neighbors_available",
       report.contourProvenanceNeighborsAvailable ? 1 : 0);
    Profile::setMetric("safe_reduce_contour_provenance_neighbors_cached",
                       report.contourProvenanceNeighborsCached ? 1 : 0);
    Profile::setMetric("safe_reduce_contour_provenance_neighbors_raw",
                       report.contourProvenanceNeighborsRaw ? 1 : 0);
    Profile::setMetric("safe_reduce_contour_provenance_neighbor_slots",
                       report.contourProvenanceNeighborSlots);
    Profile::setMetric
      ("safe_reduce_contour_provenance_neighbor_mismatches",
       report.contourProvenanceNeighborMismatches);
    Profile::setMetric
      ("safe_reduce_contour_provenance_neighbor_parity_audited",
       report.contourProvenanceNeighborParityAudited ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_contour_provenance_neighbor_parity",
       report.contourProvenanceNeighborParity ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_contour_provenance_component_report_available",
       report.contourProvenanceComponentReportAvailable ? 1 : 0);
    Profile::setMetric("safe_reduce_contour_provenance_components",
                       report.contourProvenanceComponents);
    Profile::setMetric
      ("safe_reduce_contour_provenance_component_decision_fingerprint",
       report.contourProvenanceComponentDecisionFingerprint);
    Profile::setMetric
      ("safe_reduce_contour_provenance_decision_bearing_components",
       report.contourProvenanceDecisionBearingComponents);
    Profile::setMetric
      ("safe_reduce_contour_provenance_decision_bearing_triangles",
       report.contourProvenanceDecisionBearingTriangles);
    Profile::setMetric
      ("safe_reduce_contour_provenance_estimated_triangles_after",
       report.contourProvenanceEstimatedTrianglesAfter);
    Profile::setMetric
      ("safe_reduce_contour_provenance_estimated_triangle_reduction",
       report.contourProvenanceEstimatedTriangleReduction);
    Profile::setMetric("safe_reduce_contour_provenance_phase1_components",
                       report.contourProvenancePhase1Components);
    Profile::setMetric("safe_reduce_contour_provenance_hole_aware_components",
                       report.contourProvenanceHoleAwareComponents);
    Profile::setMetric
      ("safe_reduce_contour_provenance_estimated_replacement_checks",
       report.contourProvenanceEstimatedReplacementChecks);
    Profile::setMetric
      ("safe_reduce_contour_provenance_feasible_replacement_checks",
       report.contourProvenanceFeasibleReplacementChecks);
    Profile::setMetric
      ("safe_reduce_contour_provenance_writable_replacement_checks",
       report.contourProvenanceWritableReplacementChecks);
    Profile::setMetric
      ("safe_reduce_contour_provenance_unwritable_replacement_checks",
       report.contourProvenanceUnwritableReplacementChecks);
    Profile::setMetric
      ("safe_reduce_contour_provenance_phase1_writable_replacement_checks",
       report.contourProvenancePhase1WritableReplacementChecks);
    Profile::setMetric
      ("safe_reduce_contour_provenance_hole_aware_writable_replacement_checks",
       report.contourProvenanceHoleAwareWritableReplacementChecks);
    Profile::setMetric
      ("safe_reduce_contour_provenance_phase1_unwritable_replacement_checks",
       report.contourProvenancePhase1UnwritableReplacementChecks);
    Profile::setMetric
      ("safe_reduce_contour_provenance_hole_aware_unwritable_replacement_checks",
       report.contourProvenanceHoleAwareUnwritableReplacementChecks);
    Profile::setMetric
      ("safe_reduce_contour_provenance_replacement_edge_incidence_checks",
       report.contourProvenanceReplacementEdgeIncidenceChecks);
    Profile::setMetric
      ("safe_reduce_contour_provenance_replacement_edge_incidence_rejected",
       report.contourProvenanceReplacementEdgeIncidenceRejected);
    Profile::setMetric
      ("safe_reduce_contour_provenance_phase1_replacement_edge_incidence_rejected",
       report.contourProvenancePhase1ReplacementEdgeIncidenceRejected);
    Profile::setMetric
      ("safe_reduce_contour_provenance_hole_aware_replacement_edge_incidence_rejected",
       report.contourProvenanceHoleAwareReplacementEdgeIncidenceRejected);
    Profile::setMetric
      ("safe_reduce_contour_provenance_rejected_boundary_components",
       report.contourProvenanceRejectedBoundaryComponents);
    Profile::setMetric
      ("safe_reduce_contour_provenance_rejected_no_savings_components",
       report.contourProvenanceRejectedNoSavingsComponents);
    Profile::setMetric
      ("safe_reduce_contour_provenance_rejected_triangulation_components",
       report.contourProvenanceRejectedTriangulationComponents);
    Profile::setMetric
      ("safe_reduce_contour_provenance_component_metric_mismatches",
       report.contourProvenanceComponentMetricMismatches);
    Profile::setMetric
      ("safe_reduce_contour_provenance_component_parity",
       report.contourProvenanceComponentParity ? 1 : 0);
    Profile::setMetric("safe_reduce_provenance_neighbors_requested",
                       report.useProvenanceNeighborsRequested ? 1 : 0);
    Profile::setMetric("safe_reduce_trust_provenance_neighbors_requested",
                       report.trustProvenanceNeighborsRequested ? 1 : 0);
    Profile::setMetric("safe_reduce_trusted_provenance_neighbors_eligible",
                       report.trustedProvenanceNeighborsEligible ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_rejected_no_triangle_surface",
       report.trustedProvenanceRejectedNoTriangleSurface ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_rejected_no_provenance",
       report.trustedProvenanceRejectedNoProvenance ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_rejected_no_cached_neighbors",
       report.trustedProvenanceRejectedNoCachedNeighbors ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_rejected_triangle_mismatch",
       report.trustedProvenanceRejectedTriangleMismatch ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_rejected_incomplete",
       report.trustedProvenanceRejectedIncomplete ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_rejected_unknown",
       report.trustedProvenanceRejectedUnknown ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_rejected_non_watertight",
       report.trustedProvenanceRejectedNonWatertight ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_rejected_orientation",
       report.trustedProvenanceRejectedOrientation ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_rejected_raw_topology",
       report.trustedProvenanceRejectedRawTopology ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_rejected_raw_welded_spread",
       report.trustedProvenanceRejectedRawWeldedSpread ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_rejected_neighbor_size",
       report.trustedProvenanceRejectedNeighborSize ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_rejected_neighbor_open_slot",
       report.trustedProvenanceRejectedNeighborOpenSlot ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_rejected_neighbor_range",
       report.trustedProvenanceRejectedNeighborRange ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_rejected_neighbor_self",
       report.trustedProvenanceRejectedNeighborSelf ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_rejected_neighbor_duplicate",
       report.trustedProvenanceRejectedNeighborDuplicate ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_rejected_neighbor_asymmetry",
       report.trustedProvenanceRejectedNeighborAsymmetry ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_rejected_neighbor_edge_mismatch",
       report.trustedProvenanceRejectedNeighborEdgeMismatch ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_neighbor_slots_checked",
       report.trustedProvenanceNeighborSlotsChecked);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_neighbor_edge_slots_checked",
       report.trustedProvenanceNeighborEdgeSlotsChecked);
    Profile::setMetric
      ("safe_reduce_trusted_provenance_neighbor_edge_mismatches",
       report.trustedProvenanceNeighborEdgeMismatches);
    Profile::setMetric("safe_reduce_using_provenance_neighbors",
                       report.usingProvenanceNeighbors ? 1 : 0);
    Profile::setMetric("safe_reduce_trusted_provenance_neighbors_used",
                       report.trustedProvenanceNeighborsUsed ? 1 : 0);
    Profile::setMetric("safe_reduce_default_adjacency_skipped",
                       report.defaultAdjacencySkipped ? 1 : 0);
    Profile::setMetric("safe_reduce_hole_aware_apply_requested",
                       report.applyHoleAwareRequested ? 1 : 0);
    Profile::setMetric("safe_reduce_boundary_cosimplify_apply_requested",
                       report.applyBoundaryCoSimplifyRequested ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_candidate_rolled_back",
       report.boundaryCoSimplifyCandidateRolledBack ? 1 : 0);
    Profile::setMetric("safe_reduce_boundary_cosimplify_fallback_used",
                       report.boundaryCoSimplifyFallbackUsed ? 1 : 0);
    Profile::setMetric("safe_reduce_estimated_triangles_after",
                       report.estimatedTrianglesAfter);
    Profile::setMetric("safe_reduce_estimated_triangle_reduction",
                       report.estimatedTriangleReduction);
    Profile::setMetric("safe_reduce_component_decision_fingerprint",
                       report.componentDecisionFingerprint);
    Profile::setMetric("safe_reduce_decision_bearing_components",
                       report.decisionBearingComponents);
    Profile::setMetric("safe_reduce_decision_bearing_triangles",
                       report.decisionBearingTriangles);
    Profile::setMetric("safe_reduce_single_triangle_components",
                       report.singleTriangleComponents);
    Profile::setMetric("safe_reduce_components_lt8_triangles",
                       report.componentsLt8Triangles);
    Profile::setMetric("safe_reduce_components_lt64_triangles",
                       report.componentsLt64Triangles);
    Profile::setMetric("safe_reduce_max_component_triangles",
                       report.maxComponentTriangles);
    Profile::setMetric("safe_reduce_component_neighbor_slots",
                       report.componentNeighborSlots);
    Profile::setMetric("safe_reduce_component_neighbor_candidates",
                       report.componentNeighborCandidates);
    Profile::setMetric("safe_reduce_component_plane_fit_tests",
                       report.componentPlaneFitTests);
    Profile::setMetric("safe_reduce_component_plane_fit_accepted",
                       report.componentPlaneFitAccepted);
    Profile::setMetric("safe_reduce_component_plane_vertex_checks",
                       report.componentPlaneVertexChecks);
    Profile::setMetric("safe_reduce_boundary_edge_scans",
                       report.boundaryEdgeScans);
    Profile::setMetric("safe_reduce_component_boundary_edges",
                       report.componentBoundaryEdges);
    Profile::setMetric("safe_reduce_boundary_info_checks",
                       report.boundaryInfoChecks);
    Profile::setMetric("safe_reduce_phase1_replacement_checks",
                       report.phase1ReplacementChecks);
    Profile::setMetric("safe_reduce_hole_aware_replacement_checks",
                       report.holeAwareReplacementChecks);
    Profile::setMetric("safe_reduce_estimated_replacement_checks",
                       report.estimatedReplacementChecks);
    Profile::setMetric("safe_reduce_feasible_replacement_checks",
                       report.feasibleReplacementChecks);
    Profile::setMetric("safe_reduce_writable_replacement_checks",
                       report.writableReplacementChecks);
    Profile::setMetric("safe_reduce_unwritable_replacement_checks",
                       report.unwritableReplacementChecks);
    Profile::setMetric("safe_reduce_phase1_writable_replacement_checks",
                       report.phase1WritableReplacementChecks);
    Profile::setMetric("safe_reduce_hole_aware_writable_replacement_checks",
                       report.holeAwareWritableReplacementChecks);
    Profile::setMetric("safe_reduce_phase1_unwritable_replacement_checks",
                       report.phase1UnwritableReplacementChecks);
    Profile::setMetric("safe_reduce_hole_aware_unwritable_replacement_checks",
                       report.holeAwareUnwritableReplacementChecks);
    Profile::setMetric("safe_reduce_replacement_edge_incidence_checks",
                       report.replacementEdgeIncidenceChecks);
    Profile::setMetric("safe_reduce_replacement_edge_incidence_rejected",
                       report.replacementEdgeIncidenceRejected);
    Profile::setMetric("safe_reduce_replacement_complexity_rejected",
                       report.replacementComplexityRejected);
    Profile::setMetric
      ("safe_reduce_phase1_replacement_edge_incidence_rejected",
       report.phase1ReplacementEdgeIncidenceRejected);
    Profile::setMetric
      ("safe_reduce_hole_aware_replacement_edge_incidence_rejected",
       report.holeAwareReplacementEdgeIncidenceRejected);
    Profile::setMetric("safe_reduce_phase1_components",
                       report.phase1Components);
    Profile::setMetric("safe_reduce_phase1_source_triangles",
                       report.phase1SourceTriangles);
    Profile::setMetric("safe_reduce_phase1_estimated_reduction",
                       report.phase1EstimatedReduction);
    Profile::setMetric("safe_reduce_hole_aware_components",
                       report.holeAwareComponents);
    Profile::setMetric("safe_reduce_hole_aware_source_triangles",
                       report.holeAwareSourceTriangles);
    Profile::setMetric("safe_reduce_hole_aware_estimated_reduction",
                       report.holeAwareEstimatedReduction);
    Profile::setMetric("safe_reduce_rejected_boundary_components",
                       report.rejectedBoundaryComponents);
    Profile::setMetric("safe_reduce_rejected_boundary_triangles",
                       report.rejectedBoundaryTriangles);
    Profile::setMetric("safe_reduce_rejected_no_savings_components",
                       report.rejectedNoSavingsComponents);
    Profile::setMetric("safe_reduce_rejected_no_savings_triangles",
                       report.rejectedNoSavingsTriangles);
    Profile::setMetric("safe_reduce_rejected_triangulation_components",
                       report.rejectedTriangulationComponents);
    Profile::setMetric("safe_reduce_rejected_triangulation_triangles",
                       report.rejectedTriangulationTriangles);
    Profile::setMetric("safe_reduce_applied_components",
                       report.appliedComponents);
    Profile::setMetric("safe_reduce_applied_source_triangles",
                       report.appliedSourceTriangles);
    Profile::setMetric("safe_reduce_applied_output_triangles",
                       report.appliedOutputTriangles);
    Profile::setMetric("safe_reduce_hole_aware_applied_components",
                       report.holeAwareAppliedComponents);
    Profile::setMetric("safe_reduce_hole_aware_applied_source_triangles",
                       report.holeAwareAppliedSourceTriangles);
    Profile::setMetric("safe_reduce_hole_aware_applied_output_triangles",
                       report.holeAwareAppliedOutputTriangles);
    Profile::setMetric("safe_reduce_boundary_cosimplify_candidate_components",
                       report.boundaryCoSimplifyCandidateComponents);
    Profile::setMetric("safe_reduce_boundary_cosimplify_source_triangles",
                       report.boundaryCoSimplifySourceTriangles);
    Profile::setMetric("safe_reduce_boundary_cosimplify_boundary_vertices",
                       report.boundaryCoSimplifyBoundaryVertices);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_simplified_boundary_vertices",
       report.boundaryCoSimplifySimplifiedBoundaryVertices);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_estimated_triangles_after",
       report.boundaryCoSimplifyEstimatedTrianglesAfter);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_estimated_triangles_after_simplified",
       report.boundaryCoSimplifyEstimatedTrianglesAfterSimplified);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_estimated_extra_reduction",
       report.boundaryCoSimplifyEstimatedExtraReduction);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_max_component_extra_reduction",
       report.boundaryCoSimplifyMaxComponentExtraReduction);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_rejected_candidate_boundary_edges",
       report.boundaryCoSimplifyRejectedCandidateBoundaryEdges);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_rejected_candidate_nonmanifold_edges",
       report.boundaryCoSimplifyRejectedCandidateNonManifoldEdges);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_rejected_candidate_misoriented_edges",
       report.boundaryCoSimplifyRejectedCandidateMisorientedEdges);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_rejected_candidate_degenerate_triangles",
       report.boundaryCoSimplifyRejectedCandidateDegenerateTriangles);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_rejected_candidate_expected_triangles",
       report.boundaryCoSimplifyRejectedCandidateExpectedTriangles);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_rejected_candidate_actual_triangles",
       report.boundaryCoSimplifyRejectedCandidateActualTriangles);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_rejected_candidate_topology_worse",
       report.boundaryCoSimplifyRejectedCandidateTopologyWorse ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_rejected_candidate_degenerate_worse",
       report.boundaryCoSimplifyRejectedCandidateDegenerateWorse ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_rejected_candidate_orientation_worse",
       report.boundaryCoSimplifyRejectedCandidateOrientationWorse ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_rejected_candidate_vertex_count_mismatch",
       report.boundaryCoSimplifyRejectedCandidateVertexCountMismatch ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_rejected_candidate_normal_count_mismatch",
       report.boundaryCoSimplifyRejectedCandidateNormalCountMismatch ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_rejected_candidate_triangle_count_mismatch",
       report.boundaryCoSimplifyRejectedCandidateTriangleCountMismatch ? 1 : 0);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_vertices_considered",
       report.boundaryCoSimplifyContractVerticesConsidered);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_vertices_accepted",
       report.boundaryCoSimplifyContractVerticesAccepted);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_rejected_single_sided",
       report.boundaryCoSimplifyContractRejectedSingleSided);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_rejected_ambiguous",
       report.boundaryCoSimplifyContractRejectedAmbiguous);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_rejected_non_collinear",
       report.boundaryCoSimplifyContractRejectedNonCollinear);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_rejected_ineligible",
       report.boundaryCoSimplifyContractRejectedIneligible);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_rejected_ownership",
       report.boundaryCoSimplifyContractRejectedOwnership);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_interface_edges",
       report.boundaryCoSimplifyContractInterfaceEdges);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_chain_interfaces",
       report.boundaryCoSimplifyContractChainInterfaces);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_chains",
       report.boundaryCoSimplifyContractChains);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_chain_vertices",
       report.boundaryCoSimplifyContractChainVertices);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_chain_interior_vertices",
       report.boundaryCoSimplifyContractChainInteriorVertices);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_chain_vertices_accepted",
       report.boundaryCoSimplifyContractChainVerticesAccepted);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_rejected_missing_owner",
       report.boundaryCoSimplifyContractRejectedMissingOwner);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_rejected_ambiguous_owner",
       report.boundaryCoSimplifyContractRejectedAmbiguousOwner);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_rejected_chain_ineligible",
       report.boundaryCoSimplifyContractRejectedChainIneligible);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_rejected_unsafe_endpoint",
       report.boundaryCoSimplifyContractRejectedUnsafeEndpoint);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_rejected_chain_non_collinear",
       report.boundaryCoSimplifyContractRejectedChainNonCollinear);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_components_considered",
       report.boundaryCoSimplifyContractComponentsConsidered);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_components_affected",
       report.boundaryCoSimplifyContractComponentsAffected);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_replacement_checks",
       report.boundaryCoSimplifyContractReplacementChecks);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_triangulation_rejected",
       report.boundaryCoSimplifyContractTriangulationRejected);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_edge_incidence_rejected",
       report.boundaryCoSimplifyContractEdgeIncidenceRejected);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_no_savings_rejected",
       report.boundaryCoSimplifyContractNoSavingsRejected);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_global_rejected",
       report.boundaryCoSimplifyContractGlobalRejected);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_applied_components",
       report.boundaryCoSimplifyContractAppliedComponents);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_applied_source_triangles",
       report.boundaryCoSimplifyContractAppliedSourceTriangles);
    Profile::setMetric
      ("safe_reduce_boundary_cosimplify_contract_applied_output_triangles",
       report.boundaryCoSimplifyContractAppliedOutputTriangles);
    Profile::setMetric("safe_reduce_validation_topology_worse",
                       report.validationTopologyWorse ? 1 : 0);
    Profile::setMetric("safe_reduce_validation_degenerate_worse",
                       report.validationDegenerateWorse ? 1 : 0);
    Profile::setMetric("safe_reduce_validation_orientation_worse",
                       report.validationOrientationWorse ? 1 : 0);
    Profile::setMetric("safe_reduce_validation_vertex_count_mismatch",
                       report.validationVertexCountMismatch ? 1 : 0);
    Profile::setMetric("safe_reduce_validation_normal_count_mismatch",
                       report.validationNormalCountMismatch ? 1 : 0);
    Profile::setMetric("safe_reduce_validation_triangle_count_mismatch",
                       report.validationTriangleCountMismatch ? 1 : 0);
    Profile::setMetric("safe_reduce_validation_rolled_back",
                       report.validationRolledBack ? 1 : 0);
    Profile::setMetric("safe_reduce_validation_expected_output_triangles",
                       report.validationExpectedOutputTriangles);
    Profile::setMetric("safe_reduce_validation_candidate_triangles",
                       report.validationCandidateTriangles);
    Profile::setMetric("safe_reduce_validation_candidate_checked",
                       report.validationCandidateChecked ? 1 : 0);
    Profile::setMetric("safe_reduce_validation_candidate_boundary_edges",
                       report.validationCandidateBoundaryEdges);
    Profile::setMetric("safe_reduce_validation_candidate_nonmanifold_edges",
                       report.validationCandidateNonManifoldEdges);
    Profile::setMetric("safe_reduce_validation_candidate_misoriented_edges",
                       report.validationCandidateMisorientedEdges);
    Profile::setMetric("safe_reduce_validation_candidate_degenerate_triangles",
                       report.validationCandidateDegenerateTriangles);
    Profile::setMetric("safe_reduce_validation_candidate_watertight",
                       report.validationCandidateWatertight ? 1 : 0);
    Profile::setMetric("safe_reduce_sparse_eligibility_requested",
                       report.sparseEligibilityRequested ? 1 : 0);
    Profile::setMetric("safe_reduce_origin_metadata_valid",
                       report.sparseOriginMetadataValid ? 1 : 0);
    Profile::setMetric("safe_reduce_sparse_metadata_fallback",
                       report.sparseMetadataFallback ? 1 : 0);
    Profile::setMetric("safe_reduce_mc_reducible_triangles",
                       report.sparseMCReducibleTriangles);
    Profile::setMetric("safe_reduce_mc_seam_locked_triangles",
                       report.sparseMCSeamLockedTriangles);
    Profile::setMetric("safe_reduce_analytic_locked_triangles",
                       report.sparseAnalyticLockedTriangles);
    Profile::setMetric("safe_reduce_unknown_locked_triangles",
                       report.sparseUnknownLockedTriangles);
    Profile::setMetric("safe_reduce_locked_seam_vertices",
                       report.sparseLockedSeamVertices);
    Profile::setMetric("safe_reduce_locked_seam_edges",
                       report.sparseLockedSeamEdges);
    Profile::setMetric("safe_reduce_analyzed_analytic_triangles",
                       report.sparseAnalyzedAnalyticTriangles);
    Profile::setMetric("safe_reduce_analytic_adjacency_insertions",
                       report.sparseAnalyticAdjacencyInsertions);
    Profile::setMetric("safe_reduce_analytic_component_memberships",
                       report.sparseAnalyticComponentMemberships);
    Profile::setMetric("safe_reduce_analytic_replacement_triangles",
                       report.sparseAnalyticReplacementTriangles);
    Profile::setMetric("safe_reduce_analytic_identity_preserved",
                       report.sparseAnalyticIdentityPreserved ? 1 : 0);
    Profile::setMetric("safe_reduce_locked_seams_preserved",
                       report.sparseLockedSeamsPreserved ? 1 : 0);
    Profile::setMetric("safe_reduce_whole_surface_validation_checked",
                       report.sparseWholeSurfaceValidationChecked ? 1 : 0);
    Profile::setMetric("safe_reduce_whole_surface_validation_accepted",
                       report.sparseWholeSurfaceValidationAccepted ? 1 : 0);
    Profile::setMetric("safe_reduce_input_duplicate_triangles",
                       report.sparseInputDuplicateTriangles);
    Profile::setMetric("safe_reduce_output_duplicate_triangles",
                       report.sparseOutputDuplicateTriangles);
    emitPlanarReductionSideMetrics(report);
  }


  static void emitPlanarReductionReport
    (const Surface &surface, const PlanarReductionConfig &config) {
    PlanarReductionReport report = analyzePlanarReduction(surface, config);
    emitPlanarReductionMetrics(report);

    LOG_INFO(1, "Safe reduction report: input_triangles=" << report.triangles
             << " watertight=" << (report.watertightInput ? "yes" : "no")
             << " boundary_edges=" << report.globalBoundaryEdges
             << " nonmanifold_edges=" << report.globalNonManifoldEdges
             << " output_watertight="
             << (report.watertightOutput ? "yes" : "no")
             << " output_boundary_edges=" << report.outputBoundaryEdges
             << " output_nonmanifold_edges="
             << report.outputNonManifoldEdges
             << " components=" << report.components
             << " estimated_triangles_after="
             << report.estimatedTrianglesAfter
             << " estimated_reduction="
             << report.estimatedTriangleReduction
             << " phase1_components=" << report.phase1Components
             << " phase1_source_triangles=" << report.phase1SourceTriangles
             << " phase1_estimated_reduction="
             << report.phase1EstimatedReduction
             << " hole_aware_components=" << report.holeAwareComponents
             << " hole_aware_estimated_reduction="
             << report.holeAwareEstimatedReduction
             << " boundary_cosimplify_candidates="
             << report.boundaryCoSimplifyCandidateComponents
             << " boundary_cosimplify_extra_reduction="
             << report.boundaryCoSimplifyEstimatedExtraReduction
             << " boundary_cosimplify_contract_applied="
             << report.boundaryCoSimplifyContractAppliedComponents
             << " provenance_neighbors="
             << (report.contourProvenanceNeighborsAvailable ? "yes" : "no")
             << " using_provenance_neighbors="
             << (report.usingProvenanceNeighbors ? "yes" : "no"));
    logPlanarReductionSideReport(report);
  }


  static void applyPlanarReduction
    (Surface &surface, const PlanarReductionConfig &config) {
    PlanarReductionReport report = reducePlanar(surface, config);
    emitPlanarReductionMetrics(report);

    LOG_INFO(1, "Safe reduction: input_triangles=" << report.triangles
             << " output_triangles=" << report.outputTriangles
             << " applied_components=" << report.appliedComponents
             << " applied_source_triangles="
             << report.appliedSourceTriangles
             << " applied_output_triangles="
             << report.appliedOutputTriangles
             << " estimated_reduction="
             << report.estimatedTriangleReduction
             << " watertight_input="
             << (report.watertightInput ? "yes" : "no")
             << " watertight_output="
             << (report.watertightOutput ? "yes" : "no")
             << " output_boundary_edges="
             << report.outputBoundaryEdges
             << " output_nonmanifold_edges="
             << report.outputNonManifoldEdges
             << " provenance_neighbors="
             << (report.contourProvenanceNeighborsAvailable ? "yes" : "no")
             << " using_provenance_neighbors="
             << (report.usingProvenanceNeighbors ? "yes" : "no"));
    logPlanarReductionSideReport(report);
  }


  static void addUniqueUnsigned(vector<unsigned> &values, unsigned value) {
    if (value < 2 || 256 < value) return;
    if (find(values.begin(), values.end(), value) == values.end())
      values.push_back(value);
  }


  static void addUniqueDepth(vector<double> &values, double value,
                             double stockHeight, double minDepth) {
    if (value <= 0 || stockHeight <= 0) return;

    value = min(stockHeight, max(minDepth, value));
    for (double existing: values)
      if (fabs(existing - value) <= max(1e-9, stockHeight * 1e-9))
        return;

    values.push_back(value);
  }


  static double depthQuantile(vector<double> values, double q) {
    if (values.empty()) return 0;
    sort(values.begin(), values.end());

    size_t index = (size_t)floor(q * (values.size() - 1));
    if (values.size() <= index) index = values.size() - 1;
    return values[index];
  }


  static RegionalAdviceStats evaluateRegionalAdvice
  (const Rectangle3D &bounds, const vector<Rectangle3D> &boxes,
   double resolution, uint64_t fullCells, uint64_t globalActiveCells,
   uint64_t globalSavedCells, unsigned bins, double initialDepth,
   double slabHeight, double margin) {
    RegionalAdviceStats stats;
    if (bounds == Rectangle3D() || boxes.empty() || bins < 2 ||
        resolution <= 0)
      return stats;

    double stockHeight = bounds.getHeight();
    if (stockHeight <= 0) return stats;

    Vector3D stockMin = bounds.getMin();
    Vector3D stockMax = bounds.getMax();
    Vector3D size = bounds.getDimensions();
    double xStep = size.x() / bins;
    double yStep = size.y() / bins;
    if (xStep <= 0 || yStep <= 0) return stats;

    uint64_t regionCount = (uint64_t)bins * bins;
    vector<double> depths(regionCount, initialDepth);
    vector<bool> touched(regionCount, false);

    for (const Rectangle3D &bbox: boxes) {
      if (bbox.getMax().x() < stockMin.x() ||
          stockMax.x() < bbox.getMin().x() ||
          bbox.getMax().y() < stockMin.y() ||
          stockMax.y() < bbox.getMin().y())
        continue;

      double depth = stockMax.z() - bbox.getMin().z();
      if (depth + margin <= 0) continue;

      double required =
        max(initialDepth, min(stockHeight, depth + margin));
      double active = roundDepth(required, slabHeight, stockHeight);

      int minX = (int)floor((bbox.getMin().x() - stockMin.x()) / xStep);
      int maxX = (int)floor((bbox.getMax().x() - stockMin.x()) / xStep);
      int minY = (int)floor((bbox.getMin().y() - stockMin.y()) / yStep);
      int maxY = (int)floor((bbox.getMax().y() - stockMin.y()) / yStep);
      minX = std::max(0, std::min((int)bins - 1, minX));
      maxX = std::max(0, std::min((int)bins - 1, maxX));
      minY = std::max(0, std::min((int)bins - 1, minY));
      maxY = std::max(0, std::min((int)bins - 1, maxY));

      for (int y = minY; y <= maxY; y++)
        for (int x = minX; x <= maxX; x++) {
          unsigned index = y * bins + x;
          touched[index] = true;
          if (depths[index] < active) depths[index] = active;
        }
    }

    stats.valid = true;
    stats.bins = bins;
    stats.regionCount = regionCount;
    stats.initialDepth = initialDepth;
    stats.slabHeight = slabHeight;
    stats.margin = margin;
    stats.regionWidthX = xStep;
    stats.regionWidthY = yStep;

    double halo = resolution * 2;
    for (unsigned y = 0; y < bins; y++)
      for (unsigned x = 0; x < bins; x++) {
        unsigned index = y * bins + x;
        if (touched[index]) stats.touchedRegions++;
        if (initialDepth < depths[index]) stats.expandedRegions++;

        stats.activeCells += estimateCells
          (makeRegionStock(bounds, bins, x, y, depths[index], 0),
           resolution, false);

        if (touched[index])
          stats.renderCells += estimateCells
            (makeRegionStock(bounds, bins, x, y, depths[index], halo),
             resolution, true);
      }

    stats.savedVsFull =
      stats.activeCells < fullCells ? fullCells - stats.activeCells : 0;
    stats.savedVsGlobal =
      stats.activeCells < globalActiveCells ?
      globalActiveCells - stats.activeCells : 0;
    stats.renderSavedVsFull =
      stats.renderCells < fullCells ? fullCells - stats.renderCells : 0;
    stats.renderSavedVsGlobal =
      stats.renderCells < globalActiveCells ?
      globalActiveCells - stats.renderCells : 0;
    stats.memoryFit =
      stats.savedVsGlobal > globalActiveCells / 10 ||
      (globalSavedCells == 0 && stats.savedVsFull > fullCells / 10);
    stats.runtimeFit =
      stats.renderSavedVsGlobal > globalActiveCells / 10 ||
      (globalSavedCells == 0 && stats.renderSavedVsFull > fullCells / 10);

    return stats;
  }


  static bool betterRegionalAdvice(const RegionalAdviceStats &candidate,
                                   const RegionalAdviceStats &best) {
    if (!candidate.valid) return false;
    if (!best.valid) return true;

    if (candidate.renderCells != best.renderCells)
      return candidate.renderCells < best.renderCells;
    if (candidate.activeCells != best.activeCells)
      return candidate.activeCells < best.activeCells;
    if (candidate.touchedRegions != best.touchedRegions)
      return candidate.touchedRegions < best.touchedRegions;

    return candidate.bins < best.bins;
  }


  static void emitPerfWarnings(const GCode::ToolPath &path,
                               double resolution,
                               const Rectangle3D &bounds) {
    if (resolution <= 0 || bounds == Rectangle3D()) return;

    set<unsigned> usedTools;
    double deepestCutZ = numeric_limits<double>::max();

    for (unsigned i = 0; i < path.size(); i++) {
      const GCode::Move &move = path.at(i);
      if (move.getType() == GCode::MoveType::MOVE_RAPID) continue;

      if (0 <= move.getTool()) usedTools.insert(move.getTool());
      deepestCutZ = min(deepestCutZ, move.getStartPt().z());
      deepestCutZ = min(deepestCutZ, move.getEndPt().z());
    }

    double minDiameter = numeric_limits<double>::max();
    unsigned minTool = 0;

    for (unsigned toolNo: usedTools) {
      if (!path.getTools().has(toolNo)) continue;
      double diameter = getEffectiveCuttingDiameter(path.getTools().get(toolNo));
      if (0 < diameter && diameter < minDiameter) {
        minDiameter = diameter;
        minTool = toolNo;
      }
    }

    if (minDiameter != numeric_limits<double>::max()) {
      double samples = minDiameter / resolution;
      if (samples < 4)
        LOG_WARNING("Performance warning: resolution " << resolution
                    << " gives only " << samples << " samples across tool "
                    << minTool << " effective cutting diameter "
                    << minDiameter << ". Tiny-bit detail may be "
                    << "under-resolved; consider a smaller resolution.");
    }

    Rectangle3D gridBounds = bounds.grow(resolution * 0.9);
    Vector3D dims = gridBounds.getDimensions();
    uint64_t x = (uint64_t)ceil(dims.x() / resolution);
    uint64_t y = (uint64_t)ceil(dims.y() / resolution);
    uint64_t z = (uint64_t)ceil(dims.z() / resolution);
    uint64_t cells = x * y * z;

    Profile::setMetric("perf_warning_grid_x", x);
    Profile::setMetric("perf_warning_grid_y", y);
    Profile::setMetric("perf_warning_grid_z", z);
    Profile::setMetric("perf_warning_grid_cells", cells);

    if (50000000ULL < cells)
      LOG_WARNING("Performance warning: estimated grid is " << cells
                  << " cells (" << x << " x " << y << " x " << z
                  << "). Large envelopes at fine resolution can drive high "
                  << "memory, STL size, and simulation cost.");

    bool recommendXYBins = false;
    bool recommendAdaptiveZ = false;
    bool recommendReduce = 50000000ULL < cells;
    bool recommendTrimmedStock = false;

    if (deepestCutZ != numeric_limits<double>::max()) {
      double unusedDepth = deepestCutZ - bounds.getMin().z();
      double threshold = max(2.0, 10 * resolution);
      Profile::setMetric("perf_warning_unused_stock_depth_scaled_1e6",
                         scaledMetric(unusedDepth));
      if (threshold < unusedDepth) {
        recommendAdaptiveZ = true;
        recommendTrimmedStock = true;
        LOG_WARNING("Performance warning: stock extends " << unusedDepth
                    << " below deepest programmed cut. For diagnostic "
                    << "shallow-engraving runs, trimmed stock may reduce "
                    << "memory and output size.");
      }
    }

    if (minDiameter != numeric_limits<double>::max()) {
      double samples = minDiameter / resolution;
      Profile::setMetric("perf_warning_min_tool_diameter_scaled_1e6",
                         scaledMetric(minDiameter));
      Profile::setMetric("perf_warning_min_tool_samples_milli",
                         (uint64_t)llround(samples * 1000));
      recommendXYBins = minDiameter <= 1.0 && samples <= 8;
    }

    if (recommendXYBins)
      LOG_WARNING("Performance recommendation: try --toolsweep-xy-bins 64 "
                  "as the current exact-preserving tiny-bit ToolSweep "
                  "lookup option; validate with the normal regression or "
                  "exact STL comparison.");

    if (recommendAdaptiveZ)
      LOG_WARNING("Performance recommendation: run --adaptive-z-slabs to "
                  "measure active Z depth; use --adaptive-z-render only as "
                  "an opt-in thick-stock path after sampled STL validation.");

    if (recommendReduce)
      LOG_WARNING("Performance recommendation: if STL size is the bottleneck, "
                  "try --reduce as the in-tree export-size option and "
                  "validate detail with sampled STL distance.");

    if (recommendTrimmedStock)
      LOG_WARNING("Performance recommendation: when full physical stock is "
                  "not required, create an explicit trimmed-stock diagnostic "
                  "project instead of relying on an automatic stock shortcut.");

    Profile::setMetric("perf_recommend_toolsweep_xy_bins",
                       recommendXYBins ? 64 : 0);
    Profile::setMetric("perf_recommend_global_adaptive_z",
                       recommendAdaptiveZ ? 1 : 0);
    Profile::setMetric("perf_recommend_reduce", recommendReduce ? 1 : 0);
    Profile::setMetric("perf_recommend_trimmed_stock",
                       recommendTrimmedStock ? 1 : 0);
  }


  static void emitPerfAdvice(const SmartPointer<GCode::ToolPath> &pathPtr,
                             double resolution,
                             const Rectangle3D &bounds,
                             unsigned requestedRegionBins) {
    if (resolution <= 0 || bounds == Rectangle3D()) return;

    const GCode::ToolPath &path = *pathPtr;

    set<unsigned> usedTools;
    uint64_t rapidMoves = 0;
    uint64_t feedMoves = 0;
    uint64_t shortFeedMoves = 0;
    uint64_t movesWithTinyTool = 0;
    uint64_t shortMovesWithTinyTool = 0;
    double deepestCutZ = numeric_limits<double>::max();
    double totalFeedLength = 0;
    double tinyToolFeedLength = 0;

    for (unsigned i = 0; i < path.size(); i++) {
      const GCode::Move &move = path.at(i);
      bool rapid = move.getType() == GCode::MoveType::MOVE_RAPID;
      if (rapid) rapidMoves++;
      else {
        feedMoves++;
        deepestCutZ = min(deepestCutZ, move.getStartPt().z());
        deepestCutZ = min(deepestCutZ, move.getEndPt().z());

        double length = (move.getEndPt() - move.getStartPt()).length();
        totalFeedLength += length;
        if (length <= resolution * 4) shortFeedMoves++;
      }

      if (0 <= move.getTool()) usedTools.insert(move.getTool());
    }

    double minDiameter = numeric_limits<double>::max();
    unsigned minTool = 0;
    for (unsigned toolNo: usedTools) {
      if (!path.getTools().has(toolNo)) continue;
      double diameter = getEffectiveCuttingDiameter(path.getTools().get(toolNo));
      if (0 < diameter && diameter < minDiameter) {
        minDiameter = diameter;
        minTool = toolNo;
      }
    }

    bool foundMinTool = minDiameter != numeric_limits<double>::max();

    if (foundMinTool) {
      for (unsigned i = 0; i < path.size(); i++) {
        const GCode::Move &move = path.at(i);
        if (move.getType() != GCode::MoveType::MOVE_RAPID &&
            0 <= move.getTool() &&
            (unsigned)move.getTool() == minTool) {
          double length = (move.getEndPt() - move.getStartPt()).length();
          movesWithTinyTool++;
          tinyToolFeedLength += length;
          if (length <= minDiameter) shortMovesWithTinyTool++;
        }
      }
    }

    uint64_t fullCells = estimateCells(bounds, resolution, true);
    Vector3D dims = bounds.grow(resolution * 0.9).getDimensions();
    uint64_t gridX = (uint64_t)ceil(dims.x() / resolution);
    uint64_t gridY = (uint64_t)ceil(dims.y() / resolution);
    uint64_t gridZ = (uint64_t)ceil(dims.z() / resolution);

    Profile::setMetric("perf_advice_grid_x", gridX);
    Profile::setMetric("perf_advice_grid_y", gridY);
    Profile::setMetric("perf_advice_grid_z", gridZ);
    Profile::setMetric("perf_advice_grid_cells", fullCells);
    Profile::setMetric("perf_advice_feed_moves", feedMoves);
    Profile::setMetric("perf_advice_rapid_moves", rapidMoves);
    Profile::setMetric("perf_advice_short_feed_moves", shortFeedMoves);
    Profile::setMetric("perf_advice_total_feed_length_microunits",
                       scaledMetric(totalFeedLength));

    double samples = minDiameter == numeric_limits<double>::max() ?
      0 : minDiameter / resolution;
    Profile::setMetric("perf_advice_min_tool", minTool);
    Profile::setMetric("perf_advice_min_tool_diameter_microunits",
                       foundMinTool ? scaledMetric(minDiameter) : 0);
    Profile::setMetric("perf_advice_min_tool_samples_milli",
                       (uint64_t)llround(samples * 1000));
    Profile::setMetric("perf_advice_min_tool_feed_moves", movesWithTinyTool);
    Profile::setMetric("perf_advice_min_tool_short_feed_moves",
                       shortMovesWithTinyTool);
    Profile::setMetric("perf_advice_min_tool_feed_length_microunits",
                       scaledMetric(tinyToolFeedLength));

    LOG_INFO(1, "Performance advice: grid=" << fullCells << " cells ("
             << gridX << " x " << gridY << " x " << gridZ << "), "
             << "feed_moves=" << feedMoves << ", rapid_moves=" << rapidMoves
             << ", short_feed_moves=" << shortFeedMoves << ".");

    bool recommendXYBins =
      foundMinTool && minDiameter <= 1.0 && samples <= 8 &&
      10000 < feedMoves;
    bool recommendReduce = 50000000ULL < fullCells;
    bool recommendBinary = recommendReduce;

    if (recommendXYBins)
      LOG_WARNING("Performance advice: --toolsweep-xy-bins 64 is a good "
                  "candidate because this is a tiny-tool/high-move-count "
                  "job and it preserves exact geometry.");

    if (recommendBinary)
      LOG_WARNING("Performance advice: prefer --binary for STL output on "
                  "this large grid; ASCII STL will be much larger.");

    if (recommendReduce)
      LOG_WARNING("Performance advice: --reduce is worth testing for output "
                  "size, but validate with sampled STL distance because it "
                  "is not exact-triangle-preserving.");

    if (10000 < shortMovesWithTinyTool)
      LOG_WARNING("Performance advice: validate --toolsweep-xy-bins against "
                  "the default for this toolpath; the smallest effective "
                  "tool has " << shortMovesWithTinyTool
                  << " feed moves no longer than its effective diameter.");

    Profile::setMetric("perf_advice_recommend_toolsweep_xy_bins",
                       recommendXYBins ? 64 : 0);
    Profile::setMetric("perf_advice_candidate_toolpath_spatial_indexing",
                       shortMovesWithTinyTool > 10000 ? 1 : 0);
    Profile::setMetric("perf_advice_recommend_binary", recommendBinary ? 1 : 0);
    Profile::setMetric("perf_advice_recommend_reduce", recommendReduce ? 1 : 0);

    double stockHeight = bounds.getHeight();
    if (stockHeight <= 0) return;

    ToolSweep sweep(pathPtr, 0, numeric_limits<double>::max(), 0, 0, true);

    Rectangle3D sweepBounds = sweep.getBounds();
    double defaultInitialDepth = max(stockHeight * 0.25, resolution * 4);
    defaultInitialDepth = min(defaultInitialDepth, stockHeight);
    double defaultSlabHeight = defaultInitialDepth;
    double margin = resolution * 2;
    double sweptDepth = sweepBounds == Rectangle3D() ? 0 :
      max(0.0, bounds.getMax().z() - sweepBounds.getMin().z());
    double requiredDepth =
      max(defaultInitialDepth, min(stockHeight, sweptDepth + margin));
    double activeDepth =
      roundDepth(requiredDepth, defaultSlabHeight, stockHeight);
    uint64_t globalActiveCells =
      estimateCells(makeTopStock(bounds, activeDepth), resolution, true);
    uint64_t globalSavedCells =
      globalActiveCells < fullCells ? fullCells - globalActiveCells : 0;

    Profile::setMetric("perf_advice_adaptive_z_initial_depth_microunits",
                       scaledMetric(defaultInitialDepth));
    Profile::setMetric("perf_advice_adaptive_z_required_depth_microunits",
                       scaledMetric(requiredDepth));
    Profile::setMetric("perf_advice_adaptive_z_active_depth_microunits",
                       scaledMetric(activeDepth));
    Profile::setMetric("perf_advice_adaptive_z_active_cells_est",
                       globalActiveCells);
    Profile::setMetric("perf_advice_adaptive_z_saved_cells_est",
                       globalSavedCells);

    Vector3D stockMin = bounds.getMin();
    Vector3D stockMax = bounds.getMax();
    Vector3D size = bounds.getDimensions();
    vector<Rectangle3D> regionalBoxes;
    vector<double> requiredDepths;

    sweep.forEachBBox([&](const GCode::Move &, const Rectangle3D &bbox) {
      if (bbox.getMax().x() < stockMin.x() ||
          stockMax.x() < bbox.getMin().x() ||
          bbox.getMax().y() < stockMin.y() ||
          stockMax.y() < bbox.getMin().y())
        return;

      double depth = stockMax.z() - bbox.getMin().z();
      if (depth + margin <= 0) return;

      regionalBoxes.push_back(bbox);
      requiredDepths.push_back(min(stockHeight, depth + margin));
    });

    double minCandidateDepth = min(stockHeight, max(resolution * 4, resolution));
    vector<double> depthCandidates;
    addUniqueDepth(depthCandidates, defaultInitialDepth, stockHeight,
                   minCandidateDepth);
    addUniqueDepth(depthCandidates, resolution * 4, stockHeight,
                   minCandidateDepth);
    addUniqueDepth(depthCandidates, resolution * 8, stockHeight,
                   minCandidateDepth);
    if (foundMinTool)
      addUniqueDepth(depthCandidates, minDiameter, stockHeight,
                     minCandidateDepth);
    for (double fraction: {0.125, 0.25, 0.5})
      addUniqueDepth(depthCandidates, stockHeight * fraction, stockHeight,
                     minCandidateDepth);
    for (double q: {0.10, 0.25, 0.50, 0.75})
      addUniqueDepth(depthCandidates, depthQuantile(requiredDepths, q),
                     stockHeight, minCandidateDepth);
    sort(depthCandidates.begin(), depthCandidates.end());

    uint64_t bestGlobalActiveCells = globalActiveCells;
    uint64_t bestGlobalSavedCells = globalSavedCells;
    double bestGlobalInitialDepth = defaultInitialDepth;
    double bestGlobalSlabHeight = defaultSlabHeight;
    double bestGlobalActiveDepth = activeDepth;

    for (double candidateDepth: depthCandidates) {
      double candidateSlabHeight = candidateDepth;
      double candidateRequiredDepth =
        max(candidateDepth, min(stockHeight, sweptDepth + margin));
      double candidateActiveDepth =
        roundDepth(candidateRequiredDepth, candidateSlabHeight, stockHeight);
      uint64_t candidateActiveCells = estimateCells
        (makeTopStock(bounds, candidateActiveDepth), resolution, true);

      if (candidateActiveCells < bestGlobalActiveCells) {
        bestGlobalActiveCells = candidateActiveCells;
        bestGlobalSavedCells =
          candidateActiveCells < fullCells ? fullCells - candidateActiveCells :
          0;
        bestGlobalInitialDepth = candidateDepth;
        bestGlobalSlabHeight = candidateSlabHeight;
        bestGlobalActiveDepth = candidateActiveDepth;
      }
    }

    Profile::setMetric("perf_advice_adaptive_z_best_initial_depth_microunits",
                       scaledMetric(bestGlobalInitialDepth));
    Profile::setMetric("perf_advice_adaptive_z_best_slab_height_microunits",
                       scaledMetric(bestGlobalSlabHeight));
    Profile::setMetric("perf_advice_adaptive_z_best_active_depth_microunits",
                       scaledMetric(bestGlobalActiveDepth));
    Profile::setMetric("perf_advice_adaptive_z_best_active_cells_est",
                       bestGlobalActiveCells);
    Profile::setMetric("perf_advice_adaptive_z_best_saved_cells_est",
                       bestGlobalSavedCells);

    bool recommendAdaptiveZ =
      fullCells && fullCells / 10 < bestGlobalSavedCells;
    if (recommendAdaptiveZ)
      LOG_WARNING("Performance advice: --adaptive-z-render is worth testing; "
                  "best estimated active cells are " << bestGlobalActiveCells
                  << " vs " << fullCells << " full-grid cells using initial "
                  << "depth " << bestGlobalInitialDepth << ".");

    Profile::setMetric("perf_advice_recommend_adaptive_z_render",
                       recommendAdaptiveZ ? 1 : 0);

    vector<unsigned> binCandidates;
    if (requestedRegionBins) addUniqueUnsigned(binCandidates,
                                               requestedRegionBins);
    for (unsigned bins: {8, 12, 16, 24, 32, 48, 64, 96, 128, 192, 256})
      addUniqueUnsigned(binCandidates, bins);

    double maxXY = max(size.x(), size.y());
    for (double targetWidth: {0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0})
      addUniqueUnsigned(binCandidates,
                        (unsigned)max(2.0, round(maxXY / targetWidth)));
    if (foundMinTool) {
      addUniqueUnsigned
        (binCandidates,
         (unsigned)max(2.0, round(maxXY /
                                  max(minDiameter * 4, resolution * 8))));
      addUniqueUnsigned
        (binCandidates,
         (unsigned)max(2.0, round(maxXY /
                                  max(minDiameter * 8, resolution * 16))));
    }
    sort(binCandidates.begin(), binCandidates.end());

    RegionalAdviceStats bestRegion;
    RegionalAdviceStats requestedRegion;
    uint64_t candidateCombinations = 0;
    for (unsigned bins: binCandidates)
      for (double candidateDepth: depthCandidates) {
        candidateCombinations++;
        RegionalAdviceStats candidate = evaluateRegionalAdvice
          (bounds, regionalBoxes, resolution, fullCells, bestGlobalActiveCells,
           bestGlobalSavedCells, bins, candidateDepth, candidateDepth, margin);

        if (betterRegionalAdvice(candidate, bestRegion))
          bestRegion = candidate;
        if (requestedRegionBins && bins == requestedRegionBins &&
            betterRegionalAdvice(candidate, requestedRegion))
          requestedRegion = candidate;
      }

    if (!bestRegion.valid) return;

    bool recommendRegional = bestRegion.memoryFit && bestRegion.runtimeFit;

    Profile::setMetric("perf_advice_region_candidate_bins",
                       binCandidates.size());
    Profile::setMetric("perf_advice_region_candidate_depths",
                       depthCandidates.size());
    Profile::setMetric("perf_advice_region_candidate_combinations",
                       candidateCombinations);
    Profile::setMetric("perf_advice_region_requested_bins",
                       requestedRegionBins);
    Profile::setMetric("perf_advice_region_bins", bestRegion.bins);
    Profile::setMetric("perf_advice_region_best_bins", bestRegion.bins);
    Profile::setMetric("perf_advice_region_best_initial_depth_microunits",
                       scaledMetric(bestRegion.initialDepth));
    Profile::setMetric("perf_advice_region_best_slab_height_microunits",
                       scaledMetric(bestRegion.slabHeight));
    Profile::setMetric("perf_advice_region_best_width_x_microunits",
                       scaledMetric(bestRegion.regionWidthX));
    Profile::setMetric("perf_advice_region_best_width_y_microunits",
                       scaledMetric(bestRegion.regionWidthY));
    Profile::setMetric("perf_advice_region_count", bestRegion.regionCount);
    Profile::setMetric("perf_advice_region_touched_regions",
                       bestRegion.touchedRegions);
    Profile::setMetric("perf_advice_region_expanded_regions",
                       bestRegion.expandedRegions);
    Profile::setMetric("perf_advice_region_active_cells_est",
                       bestRegion.activeCells);
    Profile::setMetric("perf_advice_region_render_cells_est",
                       bestRegion.renderCells);
    Profile::setMetric("perf_advice_region_saved_cells_vs_full",
                       bestRegion.savedVsFull);
    Profile::setMetric("perf_advice_region_saved_cells_vs_global",
                       bestRegion.savedVsGlobal);
    Profile::setMetric("perf_advice_region_render_saved_cells_vs_full",
                       bestRegion.renderSavedVsFull);
    Profile::setMetric("perf_advice_region_render_saved_cells_vs_global",
                       bestRegion.renderSavedVsGlobal);
    Profile::setMetric("perf_advice_region_memory_fit",
                       bestRegion.memoryFit ? 1 : 0);
    Profile::setMetric("perf_advice_region_runtime_fit",
                       bestRegion.runtimeFit ? 1 : 0);
    Profile::setMetric("perf_advice_recommend_adaptive_z_region_render",
                       recommendRegional ? 1 : 0);

    if (requestedRegion.valid) {
      Profile::setMetric("perf_advice_region_requested_active_cells_est",
                         requestedRegion.activeCells);
      Profile::setMetric("perf_advice_region_requested_render_cells_est",
                         requestedRegion.renderCells);
      Profile::setMetric("perf_advice_region_requested_initial_depth_microunits",
                         scaledMetric(requestedRegion.initialDepth));
      Profile::setMetric("perf_advice_region_requested_runtime_fit",
                         requestedRegion.runtimeFit ? 1 : 0);
    }

    if (recommendRegional)
      LOG_WARNING("Performance advice: --adaptive-z-region-render with "
                  "--adaptive-z-region-bins " << bestRegion.bins
                  << " is worth testing; best estimated regional render cells "
                  << "are " << bestRegion.renderCells << " vs "
                  << bestGlobalActiveCells << " best global adaptive-Z cells "
                  << "using initial depth " << bestRegion.initialDepth << ".");
    else
      LOG_INFO(1, "Performance advice: regional adaptive Z is not a strong "
               "runtime fit for this project; best bins=" << bestRegion.bins
               << ", initial_depth=" << bestRegion.initialDepth
               << ", estimated regional active cells=" << bestRegion.activeCells
               << ", regional render cells=" << bestRegion.renderCells
               << ", best global adaptive-Z cells=" << bestGlobalActiveCells
               << ", touched_regions=" << bestRegion.touchedRegions << "/"
               << bestRegion.regionCount << ".");
  }


  static SurfaceStats computeSurfaceStats(const Surface &surface,
                                          bool classifyNormals) {
    SurfaceStats stats;
    stats.triangles = surface.getTriangleCount();
    stats.binarySTLBytes = estimateBinarySTLBytes(stats.triangles);

    if (!classifyNormals) return stats;

    surface.getVertices
      ([&](const vector<float> &, const vector<float> &normals) {
        size_t count = normals.size() / 9;

        for (size_t i = 0; i < count; i++) {
          size_t offset = i * 9;
          double nx = fabs(normals[offset + 0]);
          double ny = fabs(normals[offset + 1]);
          double nz = fabs(normals[offset + 2]);
          double axis = max(nx, max(ny, nz));

          if (0.999 <= nz) stats.horizontal++;
          if (0.990 <= nz) stats.nearHorizontal++;
          if (0.999 <= axis) stats.axisAligned++;
          if (0.990 <= axis) stats.nearAxisAligned++;
        }
      });

    return stats;
  }


  class PhaseCancellingSurfaceTask : public SurfaceTask {
    string phase;

  public:
    PhaseCancellingSurfaceTask(const Simulation &sim, const string &phase) :
      SurfaceTask(sim), phase(phase) {}

    void updated(const string &status, double) override {
      if (status == phase) interrupt();
    }
  };


  static string dexelPhaseStatus(const string &phase) {
    if (phase == "eligibility") return "Checking dexel eligibility";
    if (phase == "preparing") return "Preparing dexel simulation";
    if (phase == "rasterizing") return "Rasterizing dexel simulation";
    if (phase == "building") return "Building dexel surface";
    if (phase == "validating") return "Validating dexel topology";
    return string();
  }


  class SimApp : public Application, private CamsimInternal::Options {

    string input;
    SmartPointer<ostream> output;

    Project::Project project;
    CutSim cutSim;
    SmartPointer<SurfaceTask> activeSurfaceTask;

  public:
    SimApp() : Application("CAMotics Sim") {
      CamsimInternal::Options::add(cmdLine);

      Logger::instance().setLogTime(false);
      Logger::instance().setLogNoInfoHeader(true);
      Logger::instance().setVerbosity(2);
    }


    // From Application
    int init(int argc, char *argv[]) override {
      int ret = Application::init(argc, argv);
      if (ret == -1) return ret;

      vector<string> args = cmdLine.getPositionalArgs();
      if (2 < args.size())
        THROW("Too many (" << args.size() << ") positional arguments.");
      if (safeReduceContractSelfTest) {
        if (!args.empty())
          THROW("--safe-reduce-contract-self-test does not take an input "
                "project or output path.");
        noExport = true;
        return 0;
      }
      if (args.size() < 1)
        THROW("Missing project, GCode or TPL input argument.");
      if (profileOnly) {
        noExport = true;
        if (profile.empty()) THROW("--profile-only requires --profile.");
      }
      if (perfWarningsOnly) {
        perfWarnings = true;
        noExport = true;
      }
      if (perfAdvice) {
        perfWarnings = true;
        noExport = true;
      }
      if (dexelEligibilityOnly) {
        dexel = true;
        noExport = true;
      }
      if (!dexelGridPNG.empty()) {
        if (dexelEligibilityOnly)
          THROW("--dexel-grid-png cannot be combined with "
                "--dexel-eligibility-only.");
        if (!dexel) THROW("--dexel-grid-png requires --dexel.");
        if (args.size() < 2) noExport = true;
      }
      if (safeReduceReport && args.size() < 2)
        noExport = true;
      if (adaptiveZRender && !adaptiveZSlabs)
        THROW("--adaptive-z-render requires --adaptive-z-slabs.");
      if (adaptiveZRegionRender && !adaptiveZSlabs)
        THROW("--adaptive-z-region-render requires --adaptive-z-slabs.");
      if (adaptiveZRegionRender && adaptiveZRegionBins < 2)
        THROW("--adaptive-z-region-render requires "
              "--adaptive-z-region-bins >= 2.");
      if (256 < adaptiveZRegionBins)
        THROW("--adaptive-z-region-bins is capped at 256 to limit planning "
              "memory.");
      if (sparseToolpath && !sparseToolpathHaloCells)
        THROW("--sparse-toolpath-halo-cells must be at least 1.");
      if (sparseToolpath && !sparseToolpathXYBins)
        THROW("--sparse-toolpath-xy-bins must be at least 1.");
      if (sparseToolpath && 256 < sparseToolpathXYBins)
        THROW("--sparse-toolpath-xy-bins is capped at 256 to limit index "
              "memory.");
      if (sparseToolpath &&
          (adaptiveZSlabs || adaptiveZRender || adaptiveZRegionBins ||
           adaptiveZRegionRender))
        THROW("--sparse-toolpath cannot be combined with adaptive-Z "
          "options.");
      if (sparseToolpath && toolSweepStockBounds)
        THROW("--toolsweep-stock-bounds is not yet supported with "
              "--sparse-toolpath.");
      if (dexel && sparseToolpath)
        THROW("--dexel cannot be combined with --sparse-toolpath.");
      if (dexel &&
          (adaptiveZSlabs || adaptiveZRender || adaptiveZRegionBins ||
           adaptiveZRegionRender))
        THROW("--dexel cannot be combined with adaptive-Z options.");
      if (dexelSkipTopologyValidation && !dexel)
        THROW("--dexel-skip-topology-validation requires --dexel.");
      if (!surfaceTaskCancelPhase.empty()) {
        if (!dexel)
          THROW("--surface-task-cancel-phase requires --dexel.");
        if (dexelPhaseStatus(surfaceTaskCancelPhase).empty())
          THROW("Invalid --surface-task-cancel-phase value '"
                << surfaceTaskCancelPhase << "'.");
        noExport = true;
      }
      if (reduce && safeReduce)
        THROW("--safe-reduce cannot be combined with legacy --reduce.");
      if (safeReduceProvenanceNeighbors &&
          !safeReduce && !safeReduceReport)
        THROW("--safe-reduce-provenance-neighbors requires "
              "--safe-reduce or --safe-reduce-report.");
      if (safeReduceTrustProvenanceNeighbors &&
          !safeReduceProvenanceNeighbors)
        THROW("--safe-reduce-trust-provenance-neighbors requires "
              "--safe-reduce-provenance-neighbors.");
      if (safeReduceHoleAware && !safeReduce && !safeReduceReport)
        THROW("--safe-reduce-hole-aware requires --safe-reduce or "
              "--safe-reduce-report.");
      if (safeReduceBoundaryCoSimplify && !safeReduce && !safeReduceReport)
        THROW("--safe-reduce-boundary-cosimplify requires --safe-reduce or "
              "--safe-reduce-report.");
      if ((!safeReducePlaneTolerance.empty() ||
           !safeReduceNormalAngle.empty()) &&
          !safeReduce && !safeReduceReport)
        THROW("--safe-reduce-plane-tolerance and --safe-reduce-normal-angle "
              "require --safe-reduce or --safe-reduce-report.");
      if (safeReduce || safeReduceReport)
        makePlanarReductionConfig(safeReduceProvenanceNeighbors,
                                  safeReduceTrustProvenanceNeighbors,
                                  safeReduceHoleAware,
                                  safeReduceBoundaryCoSimplify,
                                  safeReducePlaneTolerance,
                                  safeReduceNormalAngle);
      if (args.size() < 2 && !noExport) THROW("Missing STL output argument.");

      input = args[0];
      if (!noExport) output = SystemUtilities::oopen(args[1]);

      return 0;
    }


    void run() override {
      if (!profile.empty()) Profile::start(profile);

      if (safeReduceContractSelfTest) {
        string failure;
        if (!runPlanarReductionContractSelfTest(failure))
          THROW("Safe-reduce contract self-test failed: " << failure);
        if (!runSparseReductionEligibilitySelfTest(failure))
          THROW("Sparse safe-reduce eligibility self-test failed: " << failure);

        Profile::setMetric("safe_reduce_contract_self_test_passed", 1);
        LOG_INFO(1, "Safe-reduce contract self-test passed");
        Profile::write();
        return;
      }

      // Open project
      {
        Profile::Scope scope("project_load");
        if (!SystemUtilities::isFile(input))
          THROW("Input file not found: " << input);
        string ext = SystemUtilities::extension(input);
        if (ext == "xml" || ext == "camotics") project.load(input);
        else project.addFile(input); // Assume TPL or G-Code
      }

      // Resolution
      if (!resolution.empty()) {
        ResolutionMode resMode = ResolutionMode::RESOLUTION_MANUAL;
        double res = 0;

        try {
          res = String::parseDouble(resolution);
        } catch (const Exception &e) {}

        if (res) project.setResolution(res);
        else resMode = ResolutionMode::parse(resolution, resMode);

        project.setResolutionMode(resMode);
      }

      Profile::setMetric("simulation_resolution_microunits",
                         scaledMetric(project.getResolution()));

      // Generate tool path
      SmartPointer<GCode::ToolPath> path;
      {
        Profile::Scope scope("toolpath_generation");
        path = cutSim.computeToolPath(project);
      }

      // Simulate
      Rectangle3D bounds = project.getWorkpiece().getBounds();
      project.getWorkpiece().update(*path);

      if (perfWarnings) emitPerfWarnings(*path, project.getResolution(),
                                         bounds);
      if (perfWarningsOnly) {
        Profile::write();
        return;
      }
      if (perfAdvice) {
        emitPerfAdvice(path, project.getResolution(), bounds,
                       adaptiveZRegionBins);
        Profile::write();
        return;
      }

      double adaptiveHeight =
        parseNonNegativeDouble("adaptive-z-slab-height",
                               adaptiveZSlabHeight);
      double adaptiveInitialDepth =
        parseNonNegativeDouble("adaptive-z-initial-depth",
                               adaptiveZInitialDepth);
      double adaptiveMargin =
        parseNonNegativeDouble("adaptive-z-margin", adaptiveZMargin);

      Simulation sim(path, 0, 0, bounds, project.getResolution(),
                     time ? time : numeric_limits<double>::max(),
                     renderMode, threads, toolSweepXYBins, toolSweepXYZBins,
                     adaptiveZSlabs, adaptiveZRender, adaptiveHeight,
                     adaptiveInitialDepth, adaptiveMargin,
                     adaptiveZRegionBins, adaptiveZRegionRender,
                     toolSweepStockBounds);

      SmartPointer<Surface> dexelSurface;
      if (dexel) {
        if (dexelEligibilityOnly) {
          Dexel::EligibilityReport eligibility = Dexel::classify(sim);
          Dexel::recordEligibilityMetrics(eligibility);
          if (eligibility.eligible)
            LOG_INFO(1, "Dexel eligibility accepted: moves="
                     << eligibility.movesChecked << " tools="
                     << eligibility.toolsChecked);
          else
            LOG_WARNING("Dexel eligibility rejected: "
                        << Dexel::reasonName(eligibility.reason));
          Profile::write();
          return;
        }

        sim.backendPolicy = SimulationBackendPolicy::AUTO_DEXEL;
        sim.validateDexelTopology = !dexelSkipTopologyValidation;
        if (surfaceTaskCancelPhase.empty())
          activeSurfaceTask = dexelGridPNG.empty() ?
            new SurfaceTask(sim) : new SurfaceTask(sim, false, false, true);
        else activeSurfaceTask = new PhaseCancellingSurfaceTask
          (sim, dexelPhaseStatus(surfaceTaskCancelPhase));
        activeSurfaceTask->run();

        if (activeSurfaceTask->shouldQuit()) {
          Profile::setMetric("surface_task_cancelled", 1);
          Profile::setMetric("surface_task_cancelled_without_surface",
                             activeSurfaceTask->getSurface().isNull());
          Profile::setMetric("surface_task_cancelled_without_fallback",
                             !activeSurfaceTask->hasFallbackReason());
          activeSurfaceTask.release();
          Profile::write();
          return;
        }

        dexelSurface = activeSurfaceTask->getSurface();
        activeSurfaceTask.release();
      }

      SmartPointer<Surface> surface;
      setContourProvenanceCapture(safeReduceProvenanceNeighbors);
      if (!shouldQuit()) {
        Profile::Scope scope("surface_compute");
        if (!dexelSurface.isNull()) surface = dexelSurface;
        else if (sparseToolpath) {
          SparseToolpath::RegionPlanOptions sparseOptions;
          sparseOptions.xyBins = sparseToolpathXYBins;
          sparseOptions.haloCells = sparseToolpathHaloCells;
          sparseOptions.targetRegionCells = sparseToolpathTargetRegionCells;
          surface =
            SparseToolpath::computeSparseSurface(sim, sparseOptions, threads);
        } else surface = cutSim.computeSurface(sim);
      }
      setContourProvenanceCapture(false);

      if (!dexelGridPNG.empty() && !shouldQuit()) {
        Dexel::GridSurface *grid =
          dynamic_cast<Dexel::GridSurface *>(surface.get());
        if (!grid)
          THROW("--dexel-grid-png is unavailable because the Dexel "
                "candidate fell back to full marching cubes.");

        Profile::Scope scope("dexel_height_map_write");
        Dexel::HeightMap map = Dexel::makeHeightMap(*grid);
        SmartPointer<iostream> png = SystemUtilities::open
          (dexelGridPNG, ios::out | ios::binary);
        uint64_t bytes = Dexel::writeHeightMapPNG(*png, map);
        Profile::setMetric("dexel_height_map_width", map.width);
        Profile::setMetric("dexel_height_map_height", map.height);
        Profile::setRealMetric("dexel_height_map_min_z", map.minZ);
        Profile::setRealMetric("dexel_height_map_max_z", map.maxZ);
        Profile::setMetric("dexel_height_map_png_bytes", bytes);
        LOG_INFO(1, "Dexel height-map PNG written: " << dexelGridPNG
                 << " width=" << map.width << " height=" << map.height
                 << " min_z=" << map.minZ << " max_z=" << map.maxZ
                 << " bytes=" << bytes);
      }

      // Reduce
      if (reduce && !shouldQuit()) {
        Profile::Scope scope("reduction");
        cutSim.reduceSurface(surface);
      }

      if (safeReduce && !shouldQuit()) {
        Profile::Scope scope("safe_reduction");
        applyPlanarReduction
          (*surface, makePlanarReductionConfig
           (safeReduceProvenanceNeighbors,
            safeReduceTrustProvenanceNeighbors, safeReduceHoleAware,
            safeReduceBoundaryCoSimplify,
            safeReducePlaneTolerance, safeReduceNormalAngle, threads));
      } else if (safeReduceReport && !shouldQuit()) {
        Profile::Scope scope("safe_reduce_report");
        emitPlanarReductionReport
          (*surface, makePlanarReductionConfig
           (safeReduceProvenanceNeighbors,
            safeReduceTrustProvenanceNeighbors, safeReduceHoleAware,
            safeReduceBoundaryCoSimplify,
            safeReducePlaneTolerance, safeReduceNormalAngle, threads));
      }

      // Surface stats describe the final surface after optional reduction.
      if (!shouldQuit()) {
        SurfaceStats stats =
          computeSurfaceStats(*surface, surfaceStats || Profile::isEnabled());

        Profile::setMetric("surface_triangles", stats.triangles);
        Profile::setMetric("estimated_binary_stl_bytes",
                           stats.binarySTLBytes);
        Profile::setMetric("surface_horizontal_triangles", stats.horizontal);
        Profile::setMetric("surface_near_horizontal_triangles",
                           stats.nearHorizontal);
        Profile::setMetric("surface_axis_aligned_triangles",
                           stats.axisAligned);
        Profile::setMetric("surface_near_axis_aligned_triangles",
                           stats.nearAxisAligned);

        if (surfaceStats)
          LOG_INFO(1, "Surface stats: triangles=" << stats.triangles
                   << " estimated_binary_stl_bytes=" << stats.binarySTLBytes
                   << " horizontal_triangles=" << stats.horizontal
                   << " near_horizontal_triangles=" << stats.nearHorizontal
                   << " axis_aligned_triangles=" << stats.axisAligned
                   << " near_axis_aligned_triangles="
                   << stats.nearAxisAligned);
      }

      // Export surface unless this is a measurement-only run.
      if (!noExport && !shouldQuit()) {
        Profile::Scope scope("stl_write");
        surface->writeSTL
          (*output, binary, "CAMotics Surface", sim.computeHash());
      }

      Profile::write();
    }


    void requestExit() override {
      Application::requestExit();
      if (!activeSurfaceTask.isNull()) activeSurfaceTask->interrupt();
      cutSim.interrupt();
    }
  };
}


int main(int argc, char *argv[]) {
#ifdef HAVE_V8
  cb::gv8::JSImpl::init(0, 0);
#endif
  return doApplication<CAMotics::SimApp>(argc, argv);
}
