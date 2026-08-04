/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2022 Joseph Coffland
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#pragma once

#include <array>
#include <cstdint>
#include <string>


namespace CAMotics {
  class Surface;

  struct PlanarReductionConfig {
    double coordTolerance = 1e-4;
    double planeDistanceTolerance = 1e-4;
    double pairwiseNormalAngleDegrees = 0.25;
    bool useProvenanceNeighbors = false;
    bool trustProvenanceNeighbors = false;
    bool applyHoleAware = false;
    bool applyBoundaryCoSimplify = false;
    unsigned threads = 1;
  };


  enum PlanarReductionSide {
    PLANAR_REDUCTION_SIDE_X_MIN = 0,
    PLANAR_REDUCTION_SIDE_X_MAX = 1,
    PLANAR_REDUCTION_SIDE_Y_MIN = 2,
    PLANAR_REDUCTION_SIDE_Y_MAX = 3,
    PLANAR_REDUCTION_SIDE_Z_MIN = 4,
    PLANAR_REDUCTION_SIDE_Z_MAX = 5,
    PLANAR_REDUCTION_SIDE_CUT = 6,
    PLANAR_REDUCTION_SIDE_COUNT = 7
  };


  struct PlanarReductionSideReport {
    uint64_t inputTriangles = 0;
    uint64_t componentTriangles = 0;
    uint64_t zeroNormalTriangles = 0;
    uint64_t degenerateTriangles = 0;
    uint64_t unaccountedTriangles = 0;

    uint64_t components = 0;
    uint64_t singleTriangleComponents = 0;
    uint64_t singleTriangleTriangles = 0;

    uint64_t estimatedTrianglesAfter = 0;
    uint64_t estimatedTriangleReduction = 0;
    uint64_t outputTriangles = 0;

    uint64_t phase1Components = 0;
    uint64_t phase1SourceTriangles = 0;
    uint64_t phase1EstimatedOutputTriangles = 0;
    uint64_t phase1EstimatedReduction = 0;

    uint64_t holeAwareComponents = 0;
    uint64_t holeAwareSourceTriangles = 0;
    uint64_t holeAwareEstimatedOutputTriangles = 0;
    uint64_t holeAwareEstimatedReduction = 0;

    uint64_t rejectedBoundaryComponents = 0;
    uint64_t rejectedBoundaryTriangles = 0;
    uint64_t rejectedNoSavingsComponents = 0;
    uint64_t rejectedNoSavingsTriangles = 0;
    uint64_t rejectedTriangulationComponents = 0;
    uint64_t rejectedTriangulationTriangles = 0;

    uint64_t appliedComponents = 0;
    uint64_t appliedSourceTriangles = 0;
    uint64_t appliedOutputTriangles = 0;

    uint64_t validationRollbackComponents = 0;
    uint64_t validationRollbackSourceTriangles = 0;
    uint64_t validationRollbackCandidateOutputTriangles = 0;

    uint64_t boundaryCoSimplifyCandidateComponents = 0;
    uint64_t boundaryCoSimplifySourceTriangles = 0;
    uint64_t boundaryCoSimplifyBoundaryVertices = 0;
    uint64_t boundaryCoSimplifySimplifiedBoundaryVertices = 0;
    uint64_t boundaryCoSimplifyEstimatedTrianglesAfter = 0;
    uint64_t boundaryCoSimplifyEstimatedTrianglesAfterSimplified = 0;
    uint64_t boundaryCoSimplifyEstimatedExtraReduction = 0;
  };


  struct PlanarReductionReport {
    double coordTolerance = 0;
    double planeDistanceTolerance = 0;
    double pairwiseNormalAngleDegrees = 0;

    uint64_t triangles = 0;
    uint64_t analysisTriangleRecords = 0;
    uint64_t analysisAdjacencySlots = 0;
    uint64_t analysisEdgeRecords = 0;
    uint64_t components = 0;
    uint64_t componentRecordsRetained = 0;
    uint64_t componentRecordsSkipped = 0;
    uint64_t planeFitCacheSlots = 0;
    unsigned planeFitCacheThreads = 0;
    uint64_t globalBoundaryEdges = 0;
    uint64_t globalNonManifoldEdges = 0;
    uint64_t globalMisorientedEdges = 0;
    uint64_t globalDegenerateTriangles = 0;
    bool watertightInput = false;
    uint64_t sourceExpectedFloats = 0;
    uint64_t sourceVertexFloats = 0;
    uint64_t sourceNormalFloats = 0;
    bool sourceVertexCountMismatch = false;
    bool sourceNormalCountMismatch = false;
    uint64_t sourceInvalidCoordinates = 0;
    bool sourceRangeMismatch = false;
    uint64_t outputBoundaryEdges = 0;
    uint64_t outputNonManifoldEdges = 0;
    uint64_t outputMisorientedEdges = 0;
    uint64_t outputDegenerateTriangles = 0;
    bool watertightOutput = false;

    bool contourProvenanceAvailable = false;
    uint64_t contourProvenanceTriangles = 0;
    uint64_t contourProvenanceCompleteTriangles = 0;
    uint64_t contourProvenanceUnknownTriangles = 0;
    uint64_t contourProvenanceRawBoundaryEdges = 0;
    uint64_t contourProvenanceRawNonManifoldEdges = 0;
    uint64_t contourProvenanceRawMisorientedEdges = 0;
    uint64_t contourProvenanceRawUniqueEdges = 0;
    uint64_t contourProvenanceRawMaxEdgeIncidence = 0;
    uint64_t contourProvenanceRawEdgesIncidence1 = 0;
    uint64_t contourProvenanceRawEdgesIncidence2 = 0;
    uint64_t contourProvenanceRawEdgesIncidence3 = 0;
    uint64_t contourProvenanceRawEdgesIncidence4 = 0;
    uint64_t contourProvenanceRawEdgesIncidence5Plus = 0;
    uint64_t contourProvenanceRawTwinEdgeSlots = 0;
    uint64_t contourProvenanceRawBoundaryEdgeSlots = 0;
    uint64_t contourProvenanceRawNonManifoldEdgeSlots = 0;
    uint64_t contourProvenanceRawGridGridUniqueEdges = 0;
    uint64_t contourProvenanceRawGridGridMaxEdgeIncidence = 0;
    uint64_t contourProvenanceRawGridGridTwinEdgeSlots = 0;
    uint64_t contourProvenanceRawGridGridBoundaryEdgeSlots = 0;
    uint64_t contourProvenanceRawGridGridNonManifoldEdgeSlots = 0;
    uint64_t contourProvenanceRawGridGridWeldedSpreadEdges = 0;
    uint64_t contourProvenanceRawGridGridWeldedSpreadEdgeSlots = 0;
    uint64_t contourProvenanceRawGridGridWeldedSpreadMaxAlternateSlots = 0;
    uint64_t contourProvenanceRawCenterInvolvedUniqueEdges = 0;
    uint64_t contourProvenanceRawCenterInvolvedMaxEdgeIncidence = 0;
    uint64_t contourProvenanceRawCenterInvolvedTwinEdgeSlots = 0;
    uint64_t contourProvenanceRawCenterInvolvedBoundaryEdgeSlots = 0;
    uint64_t contourProvenanceRawCenterInvolvedNonManifoldEdgeSlots = 0;
    uint64_t contourProvenanceRawCenterInvolvedWeldedSpreadEdges = 0;
    uint64_t contourProvenanceRawCenterInvolvedWeldedSpreadEdgeSlots = 0;
    uint64_t contourProvenanceRawCenterInvolvedWeldedSpreadMaxAlternateSlots = 0;
    uint64_t contourProvenanceRawGridVertexUniqueKeys = 0;
    uint64_t contourProvenanceRawGridVertexWeldedSpreadKeys = 0;
    uint64_t contourProvenanceRawGridVertexWeldedSpreadObservations = 0;
    uint64_t contourProvenanceRawGridVertexWeldedSpreadMaxAlternateObservations = 0;
    uint64_t contourProvenanceRawCenterVertexUniqueKeys = 0;
    uint64_t contourProvenanceRawCenterVertexWeldedSpreadKeys = 0;
    uint64_t contourProvenanceRawCenterVertexWeldedSpreadObservations = 0;
    uint64_t contourProvenanceRawCenterVertexWeldedSpreadMaxAlternateObservations = 0;
    uint64_t contourProvenanceBoundaryEdges = 0;
    uint64_t contourProvenanceNonManifoldEdges = 0;
    uint64_t contourProvenanceMisorientedEdges = 0;
    uint64_t contourProvenanceWeldedUniqueEdges = 0;
    uint64_t contourProvenanceWeldedMaxEdgeIncidence = 0;
    uint64_t contourProvenanceWeldedEdgesIncidence1 = 0;
    uint64_t contourProvenanceWeldedEdgesIncidence2 = 0;
    uint64_t contourProvenanceWeldedEdgesIncidence3 = 0;
    uint64_t contourProvenanceWeldedEdgesIncidence4 = 0;
    uint64_t contourProvenanceWeldedEdgesIncidence5Plus = 0;
    uint64_t contourProvenanceWeldedTwinEdgeSlots = 0;
    uint64_t contourProvenanceWeldedBoundaryEdgeSlots = 0;
    uint64_t contourProvenanceWeldedNonManifoldEdgeSlots = 0;
    bool contourProvenanceWatertight = false;
    bool contourProvenanceMatchesInput = false;
    bool contourProvenanceNeighborsAvailable = false;
    bool contourProvenanceNeighborsCached = false;
    bool contourProvenanceNeighborsRaw = false;
    uint64_t contourProvenanceNeighborSlots = 0;
    uint64_t contourProvenanceNeighborMismatches = 0;
    bool contourProvenanceNeighborParityAudited = false;
    bool contourProvenanceNeighborParity = false;
    bool contourProvenanceComponentReportAvailable = false;
    uint64_t contourProvenanceComponents = 0;
    uint64_t contourProvenanceComponentDecisionFingerprint = 0;
    uint64_t contourProvenanceDecisionBearingComponents = 0;
    uint64_t contourProvenanceDecisionBearingTriangles = 0;
    uint64_t contourProvenanceEstimatedTrianglesAfter = 0;
    uint64_t contourProvenanceEstimatedTriangleReduction = 0;
    uint64_t contourProvenancePhase1Components = 0;
    uint64_t contourProvenanceHoleAwareComponents = 0;
    uint64_t contourProvenanceEstimatedReplacementChecks = 0;
    uint64_t contourProvenanceFeasibleReplacementChecks = 0;
    uint64_t contourProvenanceWritableReplacementChecks = 0;
    uint64_t contourProvenanceUnwritableReplacementChecks = 0;
    uint64_t contourProvenancePhase1WritableReplacementChecks = 0;
    uint64_t contourProvenanceHoleAwareWritableReplacementChecks = 0;
    uint64_t contourProvenancePhase1UnwritableReplacementChecks = 0;
    uint64_t contourProvenanceHoleAwareUnwritableReplacementChecks = 0;
    uint64_t contourProvenanceReplacementEdgeIncidenceChecks = 0;
    uint64_t contourProvenanceReplacementEdgeIncidenceRejected = 0;
    uint64_t contourProvenancePhase1ReplacementEdgeIncidenceRejected = 0;
    uint64_t contourProvenanceHoleAwareReplacementEdgeIncidenceRejected = 0;
    uint64_t contourProvenanceRejectedBoundaryComponents = 0;
    uint64_t contourProvenanceRejectedNoSavingsComponents = 0;
    uint64_t contourProvenanceRejectedTriangulationComponents = 0;
    uint64_t contourProvenanceComponentMetricMismatches = 0;
    bool contourProvenanceComponentParity = false;
    bool useProvenanceNeighborsRequested = false;
    bool trustProvenanceNeighborsRequested = false;
    bool trustedProvenanceNeighborsEligible = false;
    bool trustedProvenanceRejectedNoTriangleSurface = false;
    bool trustedProvenanceRejectedNoProvenance = false;
    bool trustedProvenanceRejectedNoCachedNeighbors = false;
    bool trustedProvenanceRejectedTriangleMismatch = false;
    bool trustedProvenanceRejectedIncomplete = false;
    bool trustedProvenanceRejectedUnknown = false;
    bool trustedProvenanceRejectedNonWatertight = false;
    bool trustedProvenanceRejectedOrientation = false;
    bool trustedProvenanceRejectedRawTopology = false;
    bool trustedProvenanceRejectedRawWeldedSpread = false;
    bool trustedProvenanceRejectedNeighborSize = false;
    bool trustedProvenanceRejectedNeighborOpenSlot = false;
    bool trustedProvenanceRejectedNeighborRange = false;
    bool trustedProvenanceRejectedNeighborSelf = false;
    bool trustedProvenanceRejectedNeighborDuplicate = false;
    bool trustedProvenanceRejectedNeighborAsymmetry = false;
    bool trustedProvenanceRejectedNeighborEdgeMismatch = false;
    uint64_t trustedProvenanceNeighborSlotsChecked = 0;
    uint64_t trustedProvenanceNeighborEdgeSlotsChecked = 0;
    uint64_t trustedProvenanceNeighborEdgeMismatches = 0;
    bool usingProvenanceNeighbors = false;
    bool trustedProvenanceNeighborsUsed = false;
    bool defaultAdjacencySkipped = false;
    bool applyHoleAwareRequested = false;
    bool applyBoundaryCoSimplifyRequested = false;
    bool boundaryCoSimplifyCandidateRolledBack = false;
    bool boundaryCoSimplifyFallbackUsed = false;
    uint64_t boundaryCoSimplifyRejectedCandidateBoundaryEdges = 0;
    uint64_t boundaryCoSimplifyRejectedCandidateNonManifoldEdges = 0;
    uint64_t boundaryCoSimplifyRejectedCandidateMisorientedEdges = 0;
    uint64_t boundaryCoSimplifyRejectedCandidateDegenerateTriangles = 0;
    uint64_t boundaryCoSimplifyRejectedCandidateExpectedTriangles = 0;
    uint64_t boundaryCoSimplifyRejectedCandidateActualTriangles = 0;
    bool boundaryCoSimplifyRejectedCandidateTopologyWorse = false;
    bool boundaryCoSimplifyRejectedCandidateDegenerateWorse = false;
    bool boundaryCoSimplifyRejectedCandidateOrientationWorse = false;
    bool boundaryCoSimplifyRejectedCandidateVertexCountMismatch = false;
    bool boundaryCoSimplifyRejectedCandidateNormalCountMismatch = false;
    bool boundaryCoSimplifyRejectedCandidateTriangleCountMismatch = false;

    uint64_t estimatedTrianglesAfter = 0;
    uint64_t estimatedTriangleReduction = 0;
    uint64_t componentDecisionFingerprint = 0;
    uint64_t decisionBearingComponents = 0;
    uint64_t decisionBearingTriangles = 0;

    uint64_t singleTriangleComponents = 0;
    uint64_t componentsLt8Triangles = 0;
    uint64_t componentsLt64Triangles = 0;
    uint64_t maxComponentTriangles = 0;

    uint64_t componentNeighborSlots = 0;
    uint64_t componentNeighborCandidates = 0;
    uint64_t componentPlaneFitTests = 0;
    uint64_t componentPlaneFitAccepted = 0;
    uint64_t componentPlaneVertexChecks = 0;

    uint64_t boundaryEdgeScans = 0;
    uint64_t componentBoundaryEdges = 0;
    uint64_t boundaryInfoChecks = 0;
    uint64_t phase1ReplacementChecks = 0;
    uint64_t holeAwareReplacementChecks = 0;
    uint64_t estimatedReplacementChecks = 0;
    uint64_t feasibleReplacementChecks = 0;
    uint64_t writableReplacementChecks = 0;
    uint64_t unwritableReplacementChecks = 0;
    uint64_t phase1WritableReplacementChecks = 0;
    uint64_t holeAwareWritableReplacementChecks = 0;
    uint64_t phase1UnwritableReplacementChecks = 0;
    uint64_t holeAwareUnwritableReplacementChecks = 0;
    uint64_t replacementEdgeIncidenceChecks = 0;
    uint64_t replacementEdgeIncidenceRejected = 0;
    uint64_t replacementComplexityRejected = 0;
    uint64_t phase1ReplacementEdgeIncidenceRejected = 0;
    uint64_t holeAwareReplacementEdgeIncidenceRejected = 0;

    uint64_t phase1Components = 0;
    uint64_t phase1SourceTriangles = 0;
    uint64_t phase1EstimatedReduction = 0;

    uint64_t holeAwareComponents = 0;
    uint64_t holeAwareSourceTriangles = 0;
    uint64_t holeAwareEstimatedReduction = 0;

    uint64_t rejectedBoundaryComponents = 0;
    uint64_t rejectedBoundaryTriangles = 0;

    uint64_t rejectedNoSavingsComponents = 0;
    uint64_t rejectedNoSavingsTriangles = 0;

    uint64_t rejectedTriangulationComponents = 0;
    uint64_t rejectedTriangulationTriangles = 0;

    uint64_t appliedComponents = 0;
    uint64_t appliedSourceTriangles = 0;
    uint64_t appliedOutputTriangles = 0;
    uint64_t holeAwareAppliedComponents = 0;
    uint64_t holeAwareAppliedSourceTriangles = 0;
    uint64_t holeAwareAppliedOutputTriangles = 0;
    uint64_t boundaryCoSimplifyCandidateComponents = 0;
    uint64_t boundaryCoSimplifySourceTriangles = 0;
    uint64_t boundaryCoSimplifyBoundaryVertices = 0;
    uint64_t boundaryCoSimplifySimplifiedBoundaryVertices = 0;
    uint64_t boundaryCoSimplifyEstimatedTrianglesAfter = 0;
    uint64_t boundaryCoSimplifyEstimatedTrianglesAfterSimplified = 0;
    uint64_t boundaryCoSimplifyEstimatedExtraReduction = 0;
    uint64_t boundaryCoSimplifyMaxComponentExtraReduction = 0;
    uint64_t boundaryCoSimplifyContractVerticesConsidered = 0;
    uint64_t boundaryCoSimplifyContractVerticesAccepted = 0;
    uint64_t boundaryCoSimplifyContractRejectedSingleSided = 0;
    uint64_t boundaryCoSimplifyContractRejectedAmbiguous = 0;
    uint64_t boundaryCoSimplifyContractRejectedNonCollinear = 0;
    uint64_t boundaryCoSimplifyContractRejectedIneligible = 0;
    uint64_t boundaryCoSimplifyContractRejectedOwnership = 0;
    uint64_t boundaryCoSimplifyContractInterfaceEdges = 0;
    uint64_t boundaryCoSimplifyContractChainInterfaces = 0;
    uint64_t boundaryCoSimplifyContractChains = 0;
    uint64_t boundaryCoSimplifyContractChainVertices = 0;
    uint64_t boundaryCoSimplifyContractChainInteriorVertices = 0;
    uint64_t boundaryCoSimplifyContractChainVerticesAccepted = 0;
    uint64_t boundaryCoSimplifyContractRejectedMissingOwner = 0;
    uint64_t boundaryCoSimplifyContractRejectedAmbiguousOwner = 0;
    uint64_t boundaryCoSimplifyContractRejectedChainIneligible = 0;
    uint64_t boundaryCoSimplifyContractRejectedUnsafeEndpoint = 0;
    uint64_t boundaryCoSimplifyContractRejectedChainNonCollinear = 0;
    uint64_t boundaryCoSimplifyContractComponentsConsidered = 0;
    uint64_t boundaryCoSimplifyContractComponentsAffected = 0;
    uint64_t boundaryCoSimplifyContractReplacementChecks = 0;
    uint64_t boundaryCoSimplifyContractTriangulationRejected = 0;
    uint64_t boundaryCoSimplifyContractEdgeIncidenceRejected = 0;
    uint64_t boundaryCoSimplifyContractNoSavingsRejected = 0;
    uint64_t boundaryCoSimplifyContractGlobalRejected = 0;
    uint64_t boundaryCoSimplifyContractAppliedComponents = 0;
    uint64_t boundaryCoSimplifyContractAppliedSourceTriangles = 0;
    uint64_t boundaryCoSimplifyContractAppliedOutputTriangles = 0;
    bool validationTopologyWorse = false;
    bool validationDegenerateWorse = false;
    bool validationOrientationWorse = false;
    bool validationVertexCountMismatch = false;
    bool validationNormalCountMismatch = false;
    bool validationTriangleCountMismatch = false;
    bool validationRolledBack = false;
    uint64_t validationExpectedOutputTriangles = 0;
    uint64_t validationCandidateTriangles = 0;
    bool validationCandidateChecked = false;
    uint64_t validationCandidateBoundaryEdges = 0;
    uint64_t validationCandidateNonManifoldEdges = 0;
    uint64_t validationCandidateMisorientedEdges = 0;
    uint64_t validationCandidateDegenerateTriangles = 0;
    bool validationCandidateWatertight = false;
    uint64_t outputTriangles = 0;

    bool sparseEligibilityRequested = false;
    bool sparseOriginMetadataValid = false;
    bool sparseMetadataFallback = false;
    uint64_t sparseMCReducibleTriangles = 0;
    uint64_t sparseMCSeamLockedTriangles = 0;
    uint64_t sparseAnalyticLockedTriangles = 0;
    uint64_t sparseUnknownLockedTriangles = 0;
    uint64_t sparseLockedSeamVertices = 0;
    uint64_t sparseLockedSeamEdges = 0;
    uint64_t sparseAnalyzedAnalyticTriangles = 0;
    uint64_t sparseAnalyticAdjacencyInsertions = 0;
    uint64_t sparseAnalyticComponentMemberships = 0;
    uint64_t sparseAnalyticReplacementTriangles = 0;
    bool sparseAnalyticIdentityPreserved = false;
    bool sparseLockedSeamsPreserved = false;
    bool sparseWholeSurfaceValidationChecked = false;
    bool sparseWholeSurfaceValidationAccepted = false;
    uint64_t sparseInputDuplicateTriangles = 0;
    uint64_t sparseOutputDuplicateTriangles = 0;

    std::array
      <PlanarReductionSideReport, PLANAR_REDUCTION_SIDE_COUNT> sides;
  };


  PlanarReductionReport analyzePlanarReduction
    (const Surface &surface, const PlanarReductionConfig &config);
  PlanarReductionReport reducePlanar
    (Surface &surface, const PlanarReductionConfig &config);

  bool runPlanarReductionContractSelfTest(std::string &failure);
  bool runSparseReductionEligibilitySelfTest(std::string &failure);
}
