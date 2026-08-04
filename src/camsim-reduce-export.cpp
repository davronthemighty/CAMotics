/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#include <camotics/Application.h>
#include <camotics/Profile.h>
#include <camotics/contour/PlanarReduction.h>
#include <camotics/contour/TriangleSurface.h>
#include <camotics/sim/SparseToolpathArtifacts.h>
#include <camotics/sim/SparseToolpathStages.h>

#include <cbang/ApplicationMain.h>
#include <cbang/os/SystemUtilities.h>

using namespace std;
using namespace cb;
using namespace CAMotics;


class CamsimReduceExportApp : public CAMotics::Application {
  string input;
  string output;
  string profile;
  bool binary = true;
  bool reduce = false;
  bool safeReduce = false;
  bool safeReduceReport = false;
  bool safeReduceProvenanceNeighbors = false;
  bool safeReduceTrustProvenanceNeighbors = false;
  bool safeReduceHoleAware = false;
  bool safeReduceBoundaryCoSimplify = false;
  bool safeReduceWholeSurfaceReference = false;

  static void emitSafeReductionMetrics(const PlanarReductionReport &report) {
#define METRIC(name, value) Profile::setMetric(name, (uint64_t)(value))
    METRIC("safe_reduce_input_triangles", report.triangles);
    METRIC("safe_reduce_analysis_triangle_records",
           report.analysisTriangleRecords);
    METRIC("safe_reduce_analysis_adjacency_slots",
           report.analysisAdjacencySlots);
    METRIC("safe_reduce_analysis_edge_records", report.analysisEdgeRecords);
    METRIC("safe_reduce_output_triangles", report.outputTriangles);
    METRIC("safe_reduce_components", report.components);
    METRIC("safe_reduce_component_records_retained",
           report.componentRecordsRetained);
    METRIC("safe_reduce_component_records_skipped",
           report.componentRecordsSkipped);
    METRIC("safe_reduce_plane_fit_cache_slots", report.planeFitCacheSlots);
    METRIC("safe_reduce_plane_fit_cache_threads", report.planeFitCacheThreads);
    METRIC("safe_reduce_estimated_triangles_after",
           report.estimatedTrianglesAfter);
    METRIC("safe_reduce_estimated_triangle_reduction",
           report.estimatedTriangleReduction);
    METRIC("safe_reduce_applied_components", report.appliedComponents);
    METRIC("safe_reduce_applied_source_triangles",
           report.appliedSourceTriangles);
    METRIC("safe_reduce_applied_output_triangles",
           report.appliedOutputTriangles);
    METRIC("safe_reduce_replacement_complexity_rejected",
           report.replacementComplexityRejected);
    METRIC("safe_reduce_output_boundary_edges", report.outputBoundaryEdges);
    METRIC("safe_reduce_output_nonmanifold_edges",
           report.outputNonManifoldEdges);
    METRIC("safe_reduce_output_misoriented_edges",
           report.outputMisorientedEdges);
    METRIC("safe_reduce_output_degenerate_triangles",
           report.outputDegenerateTriangles);
    METRIC("safe_reduce_validation_rolled_back",
           report.validationRolledBack ? 1 : 0);
    METRIC("safe_reduce_sparse_eligibility_requested",
           report.sparseEligibilityRequested ? 1 : 0);
    METRIC("safe_reduce_origin_metadata_valid",
           report.sparseOriginMetadataValid ? 1 : 0);
    METRIC("safe_reduce_sparse_metadata_fallback",
           report.sparseMetadataFallback ? 1 : 0);
    METRIC("safe_reduce_mc_reducible_triangles",
           report.sparseMCReducibleTriangles);
    METRIC("safe_reduce_mc_seam_locked_triangles",
           report.sparseMCSeamLockedTriangles);
    METRIC("safe_reduce_analytic_locked_triangles",
           report.sparseAnalyticLockedTriangles);
    METRIC("safe_reduce_unknown_locked_triangles",
           report.sparseUnknownLockedTriangles);
    METRIC("safe_reduce_locked_seam_vertices",
           report.sparseLockedSeamVertices);
    METRIC("safe_reduce_locked_seam_edges", report.sparseLockedSeamEdges);
    METRIC("safe_reduce_analyzed_analytic_triangles",
           report.sparseAnalyzedAnalyticTriangles);
    METRIC("safe_reduce_analytic_adjacency_insertions",
           report.sparseAnalyticAdjacencyInsertions);
    METRIC("safe_reduce_analytic_component_memberships",
           report.sparseAnalyticComponentMemberships);
    METRIC("safe_reduce_analytic_replacement_triangles",
           report.sparseAnalyticReplacementTriangles);
    METRIC("safe_reduce_analytic_identity_preserved",
           report.sparseAnalyticIdentityPreserved ? 1 : 0);
    METRIC("safe_reduce_locked_seams_preserved",
           report.sparseLockedSeamsPreserved ? 1 : 0);
    METRIC("safe_reduce_whole_surface_validation_checked",
           report.sparseWholeSurfaceValidationChecked ? 1 : 0);
    METRIC("safe_reduce_whole_surface_validation_accepted",
           report.sparseWholeSurfaceValidationAccepted ? 1 : 0);
    METRIC("safe_reduce_input_duplicate_triangles",
           report.sparseInputDuplicateTriangles);
    METRIC("safe_reduce_output_duplicate_triangles",
           report.sparseOutputDuplicateTriangles);
#undef METRIC
  }

public:
  CamsimReduceExportApp() :
    CAMotics::Application("CAMotics Sparse Toolpath Reduce Export Stage") {
    cmdLine.setUsageArgs("[OPTIONS] <stitched-surface.json> <output.stl>");
    cmdLine.setAllowConfigAsFirstArg(false);
    cmdLine.setAllowPositionalArgs(true);
    cmdLine.addTarget("binary", binary,
                      "Output binary STL, otherwise ASCII.");
    cmdLine.addTarget("reduce", reduce,
                      "Apply legacy surface reduction before export.");
    cmdLine.addTarget("safe-reduce", safeReduce,
                      "Apply filtered safe planar reduction.");
    cmdLine.addTarget("safe-reduce-report", safeReduceReport,
                      "Report filtered safe reduction without applying it.");
    cmdLine.addTarget("safe-reduce-provenance-neighbors",
                      safeReduceProvenanceNeighbors,
                      "Request contour-provenance adjacency.");
    cmdLine.addTarget("safe-reduce-trust-provenance-neighbors",
                      safeReduceTrustProvenanceNeighbors,
                      "Trust validated contour-provenance adjacency.");
    cmdLine.addTarget("safe-reduce-hole-aware", safeReduceHoleAware,
                      "Enable hole-aware safe reduction.");
    cmdLine.addTarget("safe-reduce-boundary-cosimplify",
                      safeReduceBoundaryCoSimplify,
                      "Enable boundary co-simplification.");
    cmdLine.addTarget("safe-reduce-whole-surface-reference",
                      safeReduceWholeSurfaceReference,
                      "Benchmark the pre-filter whole-surface analysis.");
    cmdLine.addTarget("profile", profile, "Write JSON performance profile.");
  }

  int init(int argc, char *argv[]) override {
    int ret = CAMotics::Application::init(argc, argv);
    if (ret == -1) return ret;

    vector<string> args = cmdLine.getPositionalArgs();
    if (args.size() != 2)
      THROW("Expected stitched surface artifact and output STL.");
    input = args[0];
    output = args[1];
    if (reduce && (safeReduce || safeReduceReport))
      THROW("--reduce cannot be combined with safe reduction.");
    if (safeReduce && safeReduceReport)
      THROW("--safe-reduce and --safe-reduce-report are mutually exclusive.");
    if (safeReduceTrustProvenanceNeighbors)
      safeReduceProvenanceNeighbors = true;
    if (safeReduceWholeSurfaceReference && !safeReduce && !safeReduceReport)
      THROW("--safe-reduce-whole-surface-reference requires "
            "safe reduction or report mode.");
    return 0;
  }

  void run() override {
    if (!profile.empty()) Profile::start(profile);
    SmartPointer<Simulation> sim = SparseToolpath::readSimulationArtifact
      (input, SparseToolpath::STITCHED_SURFACE_ARTIFACT);
    if (safeReduceWholeSurfaceReference) {
      sim->surface = sim->surface->copy();
      TriangleSurface *surface =
        dynamic_cast<TriangleSurface *>(sim->surface.get());
      if (!surface)
        THROW("Whole-surface reference requires a triangle surface.");
      surface->clearReductionEligibility();
      surface->markSparseAcceptedSurface(false);
      Profile::setMetric("safe_reduce_whole_surface_reference", 1);
    } else Profile::setMetric("safe_reduce_whole_surface_reference", 0);
    if (safeReduce || safeReduceReport) {
      PlanarReductionConfig config;
      config.useProvenanceNeighbors = safeReduceProvenanceNeighbors;
      config.trustProvenanceNeighbors =
        safeReduceTrustProvenanceNeighbors;
      config.applyHoleAware = safeReduceHoleAware;
      config.applyBoundaryCoSimplify = safeReduceBoundaryCoSimplify;
      PlanarReductionReport report;
      if (safeReduce) {
        if (!safeReduceWholeSurfaceReference)
          sim->surface = sim->surface->copy();
        Profile::Scope scope("sparse_reduce_export_safe_reduction");
        report = reducePlanar(*sim->surface, config);
      } else {
        Profile::Scope scope("sparse_reduce_export_safe_reduction_report");
        report = analyzePlanarReduction(*sim->surface, config);
      }
      emitSafeReductionMetrics(report);
    }
    SmartPointer<ostream> stream = SystemUtilities::oopen(output);
    SparseToolpath::writeReduceExport(*sim, *stream, binary, reduce);
    Profile::write();
  }
};


int main(int argc, char *argv[]) {
  return doApplication<CamsimReduceExportApp>(argc, argv);
}
