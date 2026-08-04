/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2022 Joseph Coffland
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#include "PlanarReduction.h"
#include "PlanarReductionInternal.h"

#include "ContourProvenance.h"
#include "Surface.h"
#include "TriangleSurface.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace std;
using namespace CAMotics;
using namespace CAMotics::PlanarReductionInternal;


namespace {
  void populateSparseEligibilityReport
  (PlanarReductionReport &report, const ReductionEligibility *eligibility,
   bool valid) {
    report.sparseEligibilityRequested = true;
    report.sparseOriginMetadataValid = valid;
    report.sparseMetadataFallback = !valid;
    if (!eligibility) return;

    report.sparseMCReducibleTriangles =
      eligibility->count(REDUCTION_MC_REDUCIBLE);
    report.sparseMCSeamLockedTriangles =
      eligibility->count(REDUCTION_MC_SEAM_LOCKED);
    report.sparseAnalyticLockedTriangles =
      eligibility->count(REDUCTION_ANALYTIC_LOCKED);
    report.sparseUnknownLockedTriangles =
      eligibility->count(REDUCTION_UNKNOWN_LOCKED);
    report.sparseLockedSeamVertices =
      eligibility->lockedSeamVertices.size();
    report.sparseLockedSeamEdges = eligibility->lockedSeamEdges.size();
  }


  void appendTriangle(vector<float> &outputVertices,
                      vector<float> &outputNormals,
                      const vector<float> &vertices,
                      const vector<float> &normals, uint64_t triangle) {
    size_t offset = (size_t)triangle * 9;
    outputVertices.insert(outputVertices.end(), vertices.begin() + offset,
                          vertices.begin() + offset + 9);
    outputNormals.insert(outputNormals.end(), normals.begin() + offset,
                         normals.begin() + offset + 9);
  }


  void buildEligibleSurface(const TriangleSurface &source,
                            const ReductionEligibility &eligibility,
                            TriangleSurface &eligible) {
    const vector<float> &vertices = source.getVertices();
    const vector<float> &normals = source.getNormals();
    for (uint64_t triangle = 0;
         triangle < eligibility.triangleOrigins.size(); triangle++) {
      if (eligibility.triangleOrigins[(size_t)triangle] !=
          REDUCTION_MC_REDUCIBLE)
        continue;

      size_t offset = (size_t)triangle * 9;
      cb::Vector3F points[3];
      for (unsigned corner = 0; corner < 3; corner++) {
        size_t vertexOffset = offset + (size_t)corner * 3;
        points[corner] = cb::Vector3F
          (vertices[vertexOffset], vertices[vertexOffset + 1],
           vertices[vertexOffset + 2]);
      }

      eligible.add(points, cb::Vector3F
                   (normals[offset], normals[offset + 1], normals[offset + 2]));
    }
  }


  vector<float> collectAnalyticTriangles
  (const vector<float> &vertices, const vector<uint8_t> &origins) {
    vector<float> analytic;
    analytic.reserve
      ((size_t)count(origins.begin(), origins.end(),
                     (uint8_t)REDUCTION_ANALYTIC_LOCKED) * 9);
    for (uint64_t triangle = 0; triangle < origins.size(); triangle++)
      if (origins[(size_t)triangle] == REDUCTION_ANALYTIC_LOCKED) {
        size_t offset = (size_t)triangle * 9;
        analytic.insert(analytic.end(), vertices.begin() + offset,
                        vertices.begin() + offset + 9);
      }
    return analytic;
  }


  PlanarReductionReport analyzeOrReduceSparse
  (const Surface &surface, const PlanarReductionConfig &config,
   TriangleSurface *target) {
    PlanarReductionReport report;
    const TriangleSurface *source =
      target ? target : dynamic_cast<const TriangleSurface *>(&surface);
    if (!source || !source->isSparseAcceptedSurface())
      return analyzeOrReduceCore(surface, config, target);

    const ReductionEligibility *eligibility =
      source->hasReductionEligibility() ? &source->getReductionEligibility() : 0;
    bool metadataValid = eligibility != 0;
    populateSparseEligibilityReport(report, eligibility, metadataValid);

    uint64_t inputTriangles = source->getTriangleCount();
    report.triangles = inputTriangles;
    report.outputTriangles = inputTriangles;
    if ((uint64_t)numeric_limits<size_t>::max() / 9 < inputTriangles) {
      report.sourceRangeMismatch = true;
      return report;
    }
    report.sourceExpectedFloats = inputTriangles * 9;
    report.sourceVertexFloats = source->getVertices().size();
    report.sourceNormalFloats = source->getNormals().size();
    report.sourceVertexCountMismatch =
      report.sourceVertexFloats != report.sourceExpectedFloats;
    report.sourceNormalCountMismatch =
      report.sourceNormalFloats != report.sourceExpectedFloats;
    if (report.sourceVertexCountMismatch ||
        report.sourceNormalCountMismatch || !metadataValid)
      return report;
    for (float value: source->getNormals())
      if (!isfinite(value)) report.sourceInvalidCoordinates++;
    if (report.sourceInvalidCoordinates) return report;

    TriangleSurface eligibleSurface;
    buildEligibleSurface(*source, *eligibility, eligibleSurface);
    uint64_t eligibleInput = eligibleSurface.getTriangleCount();
    uint64_t lockedTriangles = inputTriangles - eligibleInput;
    PlanarReductionReport filtered =
      analyzeOrReduceCore(eligibleSurface, config, target ? &eligibleSurface : 0);
    populateSparseEligibilityReport(filtered, eligibility, true);

    filtered.triangles = inputTriangles;
    filtered.sourceExpectedFloats = inputTriangles * 9;
    filtered.sourceVertexFloats = source->getVertices().size();
    filtered.sourceNormalFloats = source->getNormals().size();
    filtered.sourceInvalidCoordinates = report.sourceInvalidCoordinates;
    filtered.sourceRangeMismatch = report.sourceRangeMismatch;
    filtered.estimatedTrianglesAfter += lockedTriangles;
    filtered.estimatedTriangleReduction =
      inputTriangles < filtered.estimatedTrianglesAfter ? 0 :
      inputTriangles - filtered.estimatedTrianglesAfter;
    filtered.outputTriangles += lockedTriangles;
    if (filtered.validationExpectedOutputTriangles)
      filtered.validationExpectedOutputTriangles += lockedTriangles;
    if (filtered.validationCandidateTriangles)
      filtered.validationCandidateTriangles += lockedTriangles;

    PlanarReductionConfig wholeSurfaceConfig = config;
    wholeSurfaceConfig.coordTolerance =
      max(config.coordTolerance, eligibility->quantizationTolerance);
    EdgeIncidenceReport inputValidation =
      validateEdgeIncidenceVertices(source->getVertices(), wholeSurfaceConfig);
    filtered.globalBoundaryEdges = inputValidation.boundaryEdges;
    filtered.globalNonManifoldEdges = inputValidation.nonManifoldEdges;
    filtered.globalMisorientedEdges = inputValidation.misorientedEdges;
    filtered.globalDegenerateTriangles = inputValidation.degenerateTriangles;
    filtered.watertightInput = inputValidation.watertight();
    filtered.sparseInputDuplicateTriangles =
      inputValidation.duplicateTriangles;

    if (!target || !filtered.appliedComponents) {
      filtered.outputBoundaryEdges = filtered.globalBoundaryEdges;
      filtered.outputNonManifoldEdges = filtered.globalNonManifoldEdges;
      filtered.outputMisorientedEdges = filtered.globalMisorientedEdges;
      filtered.outputDegenerateTriangles = filtered.globalDegenerateTriangles;
      filtered.watertightOutput = filtered.watertightInput;
      filtered.sparseAnalyticIdentityPreserved = true;
      filtered.sparseLockedSeamsPreserved = true;
      filtered.sparseWholeSurfaceValidationChecked = true;
      filtered.sparseWholeSurfaceValidationAccepted = true;
      return filtered;
    }

    const vector<float> &sourceVertices = source->getVertices();
    const vector<float> &sourceNormals = source->getNormals();
    const vector<float> &reducedVertices = eligibleSurface.getVertices();
    const vector<float> &reducedNormals = eligibleSurface.getNormals();
    vector<float> candidateVertices;
    vector<float> candidateNormals;
    vector<uint8_t> candidateOrigins;
    candidateVertices.reserve
      ((size_t)(lockedTriangles + eligibleSurface.getTriangleCount()) * 9);
    candidateNormals.reserve(candidateVertices.capacity());
    candidateOrigins.reserve
      ((size_t)(lockedTriangles + eligibleSurface.getTriangleCount()));

    bool insertedEligible = false;
    for (uint64_t triangle = 0; triangle < inputTriangles; triangle++) {
      uint8_t origin = eligibility->triangleOrigins[(size_t)triangle];
      if (origin == REDUCTION_MC_REDUCIBLE) {
        if (insertedEligible) continue;
        candidateVertices.insert(candidateVertices.end(),
                                 reducedVertices.begin(), reducedVertices.end());
        candidateNormals.insert(candidateNormals.end(),
                                reducedNormals.begin(), reducedNormals.end());
        candidateOrigins.insert
          (candidateOrigins.end(), eligibleSurface.getTriangleCount(),
           (uint8_t)REDUCTION_MC_REDUCIBLE);
        insertedEligible = true;
        continue;
      }

      appendTriangle(candidateVertices, candidateNormals, sourceVertices,
                     sourceNormals, triangle);
      candidateOrigins.push_back(origin);
    }

    ReductionEligibility candidateEligibility = *eligibility;
    candidateEligibility.triangleOrigins = candidateOrigins;
    candidateEligibility.seal(candidateVertices);
    filtered.sparseLockedSeamsPreserved =
      candidateEligibility.validFor(candidateVertices);
    filtered.sparseAnalyticIdentityPreserved =
      collectAnalyticTriangles(sourceVertices, eligibility->triangleOrigins) ==
      collectAnalyticTriangles(candidateVertices, candidateOrigins);

    EdgeIncidenceReport candidateValidation =
      validateEdgeIncidenceVertices(candidateVertices, wholeSurfaceConfig);
    filtered.sparseWholeSurfaceValidationChecked = true;
    bool topologyWorse =
      inputValidation.boundaryEdges < candidateValidation.boundaryEdges ||
      inputValidation.nonManifoldEdges < candidateValidation.nonManifoldEdges ||
      inputValidation.misorientedEdges < candidateValidation.misorientedEdges ||
      inputValidation.degenerateTriangles < candidateValidation.degenerateTriangles ||
      inputValidation.duplicateTriangles < candidateValidation.duplicateTriangles;
    filtered.sparseOutputDuplicateTriangles =
      candidateValidation.duplicateTriangles;
    filtered.sparseWholeSurfaceValidationAccepted =
      !topologyWorse && filtered.sparseLockedSeamsPreserved &&
      filtered.sparseAnalyticIdentityPreserved;
    filtered.validationCandidateChecked = true;
    filtered.validationCandidateBoundaryEdges =
      candidateValidation.boundaryEdges;
    filtered.validationCandidateNonManifoldEdges =
      candidateValidation.nonManifoldEdges;
    filtered.validationCandidateMisorientedEdges =
      candidateValidation.misorientedEdges;
    filtered.validationCandidateDegenerateTriangles =
      candidateValidation.degenerateTriangles;
    filtered.validationCandidateWatertight = candidateValidation.watertight();

    if (!filtered.sparseWholeSurfaceValidationAccepted) {
      filtered.validationRolledBack = true;
      resetAppliedReductionReport(filtered);
      filtered.outputTriangles = inputTriangles;
      filtered.outputBoundaryEdges = inputValidation.boundaryEdges;
      filtered.outputNonManifoldEdges = inputValidation.nonManifoldEdges;
      filtered.outputMisorientedEdges = inputValidation.misorientedEdges;
      filtered.outputDegenerateTriangles = inputValidation.degenerateTriangles;
      filtered.watertightOutput = inputValidation.watertight();
      finalizeSideReductionReports(filtered);
      return filtered;
    }

    target->replace(std::move(candidateVertices), std::move(candidateNormals));
    target->markSparseAcceptedSurface();
    target->setReductionEligibility(candidateEligibility);
    filtered.outputTriangles = target->getTriangleCount();
    filtered.outputBoundaryEdges = candidateValidation.boundaryEdges;
    filtered.outputNonManifoldEdges = candidateValidation.nonManifoldEdges;
    filtered.outputMisorientedEdges = candidateValidation.misorientedEdges;
    filtered.outputDegenerateTriangles = candidateValidation.degenerateTriangles;
    filtered.watertightOutput = candidateValidation.watertight();
    finalizeSideReductionReports(filtered);
    return filtered;
  }
}


PlanarReductionReport CAMotics::analyzePlanarReduction
  (const Surface &surface, const PlanarReductionConfig &config) {
  return analyzeOrReduceSparse(surface, config, 0);
}


PlanarReductionReport CAMotics::reducePlanar
  (Surface &surface, const PlanarReductionConfig &config) {
  TriangleSurface *triangleSurface = dynamic_cast<TriangleSurface *>(&surface);
  if (!triangleSurface) return analyzePlanarReduction(surface, config);

  PlanarReductionReport report =
    analyzeOrReduceSparse(surface, config, triangleSurface);
  if (!config.applyBoundaryCoSimplify || !report.validationRolledBack)
    return report;

  PlanarReductionConfig fallbackConfig = config;
  fallbackConfig.applyBoundaryCoSimplify = false;
  PlanarReductionReport fallbackReport =
    analyzeOrReduceSparse(surface, fallbackConfig, triangleSurface);
  fallbackReport.applyBoundaryCoSimplifyRequested = true;
  fallbackReport.boundaryCoSimplifyCandidateRolledBack = true;
  fallbackReport.boundaryCoSimplifyFallbackUsed = true;
  fallbackReport.boundaryCoSimplifyRejectedCandidateBoundaryEdges =
    report.validationCandidateBoundaryEdges;
  fallbackReport.boundaryCoSimplifyRejectedCandidateNonManifoldEdges =
    report.validationCandidateNonManifoldEdges;
  fallbackReport.boundaryCoSimplifyRejectedCandidateMisorientedEdges =
    report.validationCandidateMisorientedEdges;
  fallbackReport.boundaryCoSimplifyRejectedCandidateDegenerateTriangles =
    report.validationCandidateDegenerateTriangles;
  fallbackReport.boundaryCoSimplifyRejectedCandidateExpectedTriangles =
    report.validationExpectedOutputTriangles;
  fallbackReport.boundaryCoSimplifyRejectedCandidateActualTriangles =
    report.validationCandidateTriangles;
  fallbackReport.boundaryCoSimplifyRejectedCandidateTopologyWorse =
    report.validationTopologyWorse;
  fallbackReport.boundaryCoSimplifyRejectedCandidateDegenerateWorse =
    report.validationDegenerateWorse;
  fallbackReport.boundaryCoSimplifyRejectedCandidateOrientationWorse =
    report.validationOrientationWorse;
  fallbackReport.boundaryCoSimplifyRejectedCandidateVertexCountMismatch =
    report.validationVertexCountMismatch;
  fallbackReport.boundaryCoSimplifyRejectedCandidateNormalCountMismatch =
    report.validationNormalCountMismatch;
  fallbackReport.boundaryCoSimplifyRejectedCandidateTriangleCountMismatch =
    report.validationTriangleCountMismatch;
  fallbackReport.boundaryCoSimplifyContractVerticesConsidered =
    report.boundaryCoSimplifyContractVerticesConsidered;
  fallbackReport.boundaryCoSimplifyContractVerticesAccepted =
    report.boundaryCoSimplifyContractVerticesAccepted;
  fallbackReport.boundaryCoSimplifyContractRejectedSingleSided =
    report.boundaryCoSimplifyContractRejectedSingleSided;
  fallbackReport.boundaryCoSimplifyContractRejectedAmbiguous =
    report.boundaryCoSimplifyContractRejectedAmbiguous;
  fallbackReport.boundaryCoSimplifyContractRejectedNonCollinear =
    report.boundaryCoSimplifyContractRejectedNonCollinear;
  fallbackReport.boundaryCoSimplifyContractRejectedIneligible =
    report.boundaryCoSimplifyContractRejectedIneligible;
  fallbackReport.boundaryCoSimplifyContractRejectedOwnership =
    report.boundaryCoSimplifyContractRejectedOwnership;
  fallbackReport.boundaryCoSimplifyContractInterfaceEdges =
    report.boundaryCoSimplifyContractInterfaceEdges;
  fallbackReport.boundaryCoSimplifyContractChainInterfaces =
    report.boundaryCoSimplifyContractChainInterfaces;
  fallbackReport.boundaryCoSimplifyContractChains =
    report.boundaryCoSimplifyContractChains;
  fallbackReport.boundaryCoSimplifyContractChainVertices =
    report.boundaryCoSimplifyContractChainVertices;
  fallbackReport.boundaryCoSimplifyContractChainInteriorVertices =
    report.boundaryCoSimplifyContractChainInteriorVertices;
  fallbackReport.boundaryCoSimplifyContractChainVerticesAccepted =
    report.boundaryCoSimplifyContractChainVerticesAccepted;
  fallbackReport.boundaryCoSimplifyContractRejectedMissingOwner =
    report.boundaryCoSimplifyContractRejectedMissingOwner;
  fallbackReport.boundaryCoSimplifyContractRejectedAmbiguousOwner =
    report.boundaryCoSimplifyContractRejectedAmbiguousOwner;
  fallbackReport.boundaryCoSimplifyContractRejectedChainIneligible =
    report.boundaryCoSimplifyContractRejectedChainIneligible;
  fallbackReport.boundaryCoSimplifyContractRejectedUnsafeEndpoint =
    report.boundaryCoSimplifyContractRejectedUnsafeEndpoint;
  fallbackReport.boundaryCoSimplifyContractRejectedChainNonCollinear =
    report.boundaryCoSimplifyContractRejectedChainNonCollinear;
  fallbackReport.boundaryCoSimplifyContractComponentsConsidered =
    report.boundaryCoSimplifyContractComponentsConsidered;
  fallbackReport.boundaryCoSimplifyContractComponentsAffected =
    report.boundaryCoSimplifyContractComponentsAffected;
  fallbackReport.boundaryCoSimplifyContractReplacementChecks =
    report.boundaryCoSimplifyContractReplacementChecks;
  fallbackReport.boundaryCoSimplifyContractTriangulationRejected =
    report.boundaryCoSimplifyContractTriangulationRejected;
  fallbackReport.boundaryCoSimplifyContractEdgeIncidenceRejected =
    report.boundaryCoSimplifyContractEdgeIncidenceRejected;
  fallbackReport.boundaryCoSimplifyContractNoSavingsRejected =
    report.boundaryCoSimplifyContractNoSavingsRejected;
  fallbackReport.boundaryCoSimplifyContractGlobalRejected =
    report.boundaryCoSimplifyContractGlobalRejected;
  fallbackReport.boundaryCoSimplifyContractAppliedComponents =
    report.boundaryCoSimplifyContractAppliedComponents;
  fallbackReport.boundaryCoSimplifyContractAppliedSourceTriangles =
    report.boundaryCoSimplifyContractAppliedSourceTriangles;
  fallbackReport.boundaryCoSimplifyContractAppliedOutputTriangles =
    report.boundaryCoSimplifyContractAppliedOutputTriangles;
  return fallbackReport;
}


bool CAMotics::runSparseReductionEligibilitySelfTest(string &failure) {
  TriangleSurface surface;
  cb::Vector3F a[3] = {
    cb::Vector3F(0, 0, 0), cb::Vector3F(1, 0, 0), cb::Vector3F(1, 1, 0)};
  cb::Vector3F b[3] = {
    cb::Vector3F(0, 0, 0), cb::Vector3F(1, 1, 0), cb::Vector3F(0, 1, 0)};
  surface.add(a);
  surface.add(b);
  surface.markSparseAcceptedSurface();

  const vector<float> originalVertices = surface.getVertices();
  PlanarReductionConfig config;
  PlanarReductionReport missing = reducePlanar(surface, config);
  if (!missing.sparseEligibilityRequested ||
      missing.sparseOriginMetadataValid ||
      !missing.sparseMetadataFallback || missing.appliedComponents ||
      surface.getVertices() != originalVertices) {
    failure = "missing sparse eligibility did not skip reduction";
    return false;
  }

  ReductionEligibility invalid;
  invalid.quantizationTolerance = 1e-4;
  invalid.triangleOrigins.assign(2, (uint8_t)REDUCTION_MC_REDUCIBLE);
  invalid.bindingHash = "invalid";
  surface.setReductionEligibility(invalid);
  PlanarReductionReport stale = reducePlanar(surface, config);
  if (!stale.sparseEligibilityRequested || stale.sparseOriginMetadataValid ||
      !stale.sparseMetadataFallback || stale.appliedComponents ||
      surface.getVertices() != originalVertices) {
    failure = "stale sparse eligibility did not skip reduction";
    return false;
  }

  failure.clear();
  return true;
}


bool CAMotics::runPlanarReductionContractSelfTest(string &failure) {
  return runPlanarReductionContractSelfTestCore(failure);
}
