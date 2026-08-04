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

#include <camotics/render/RenderMode.h>

#include <cstdint>
#include <string>


namespace cb {class CommandLine;}


namespace CAMotics {namespace CamsimInternal {
  struct Options {
    double time = 0;
    bool reduce = false;
    bool safeReduce = false;
    bool safeReduceReport = false;
    bool safeReduceProvenanceNeighbors = false;
    bool safeReduceTrustProvenanceNeighbors = false;
    bool safeReduceHoleAware = false;
    bool safeReduceBoundaryCoSimplify = false;
    bool safeReduceContractSelfTest = false;
    std::string safeReducePlaneTolerance;
    std::string safeReduceNormalAngle;
    bool binary = true;
    bool noExport = false;
    bool profileOnly = false;
    bool surfaceStats = false;
    bool perfWarnings = false;
    bool perfWarningsOnly = false;
    bool perfAdvice = false;
    RenderMode renderMode;
    std::string resolution;
    std::string profile;
    unsigned threads;
    unsigned toolSweepXYBins = 0;
    unsigned toolSweepXYZBins = 0;
    bool toolSweepStockBounds = false;
    bool dexel = false;
    bool dexelEligibilityOnly = false;
    bool dexelSkipTopologyValidation = false;
    std::string dexelGridPNG;
    std::string surfaceTaskCancelPhase;
    bool sparseToolpath = false;
    unsigned sparseToolpathXYBins = 64;
    unsigned sparseToolpathHaloCells = 1;
    uint64_t sparseToolpathTargetRegionCells = 1000000;
    bool adaptiveZSlabs = false;
    bool adaptiveZRender = false;
    std::string adaptiveZSlabHeight;
    std::string adaptiveZInitialDepth;
    std::string adaptiveZMargin;
    unsigned adaptiveZRegionBins = 0;
    bool adaptiveZRegionRender = false;

    Options();
    void add(cb::CommandLine &cmdLine);
  };
}}
