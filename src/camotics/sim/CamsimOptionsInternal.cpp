/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2026 Joseph Coffland
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#include "CamsimOptionsInternal.h"

#include <cbang/config/CommandLine.h>
#include <cbang/os/SystemInfo.h>

using namespace cb;
using namespace CAMotics;
using namespace CAMotics::CamsimInternal;


CAMotics::CamsimInternal::Options::Options() :
  threads(SystemInfo::instance().getCPUCount()) {}


void CAMotics::CamsimInternal::Options::add(CommandLine &cmdLine) {
  cmdLine.setUsageArgs
    ("[OPTIONS] <project.camotics | input.gcode | input.tpl> [output.stl]");

  cmdLine.setAllowConfigAsFirstArg(false);
  cmdLine.setAllowPositionalArgs(true);

  cmdLine.addTarget("time", time, "Simulation end time in seconds.  "
                    "A value of zero simulates the entire path.");
  cmdLine.addTarget("reduce", reduce, "Reduce cut workpiece.");
  cmdLine.addTarget("safe-reduce", safeReduce,
                    "Advanced opt-in: apply topology-checked planar STL "
                    "reduction for simple one-loop planar regions.");
  cmdLine.addTarget("safe-reduce-report", safeReduceReport,
                    "Report production-safe planar STL reduction "
                    "candidates without changing output.");
  cmdLine.addTarget("safe-reduce-provenance-neighbors",
                    safeReduceProvenanceNeighbors,
                    "Advanced opt-in: use contour provenance neighbors "
                    "for safe-reduce classification only after parity "
                    "with the default adjacency table is proven.");
  cmdLine.addTarget("safe-reduce-trust-provenance-neighbors",
                    safeReduceTrustProvenanceNeighbors,
                    "Experimental opt-in: when contour provenance is "
                    "complete, watertight, raw/welded topology matches, "
                    "and cached provenance neighbors exist, skip the "
                    "default safe-reduce adjacency "
                    "build and trust those neighbors.  Requires "
                    "--safe-reduce-provenance-neighbors.");
  cmdLine.addTarget("safe-reduce-hole-aware", safeReduceHoleAware,
                    "Advanced opt-in: include hole-preserving multi-loop "
                    "planar replacements in safe-reduce report/apply "
                    "mode after validation.");
  cmdLine.addTarget("safe-reduce-boundary-cosimplify",
                    safeReduceBoundaryCoSimplify,
                    "Experimental opt-in: allow safe-reduce apply mode "
                    "to try collinear boundary co-simplified planar "
                    "replacements.  The whole candidate is validated and "
                    "rolled back if shared-edge simplification creates "
                    "topology, orientation, or degenerate regressions.");
  cmdLine.addTarget("safe-reduce-contract-self-test",
                    safeReduceContractSelfTest,
                    "Run internal safe-reduce replacement contract "
                    "self-tests and exit.");
  cmdLine.addTarget("safe-reduce-plane-tolerance",
                    safeReducePlaneTolerance,
                    "Advanced opt-in: safe-reduce plane-distance "
                    "tolerance in project units.  Default is 0.0001.");
  cmdLine.addTarget("safe-reduce-normal-angle",
                    safeReduceNormalAngle,
                    "Advanced opt-in: safe-reduce pairwise normal angle "
                    "in degrees.  Default is 0.25 and the maximum is 5.");
  cmdLine.addTarget("binary", binary,
                    "Output binary STL, otherwise ASCII.");
  cmdLine.addTarget("no-export", noExport,
                    "Run simulation without writing STL output.");
  cmdLine.addTarget("profile-only", profileOnly,
                    "Run simulation and profile without writing STL "
                    "output.");
  cmdLine.addTarget("surface-stats", surfaceStats,
                    "Report surface triangle and output-size stats.");
  cmdLine.addTarget("perf-warnings", perfWarnings,
                    "Warn about resolution, envelope, and stock setups "
                    "that may cause slow tiny-bit simulations.");
  cmdLine.addTarget("perf-warnings-only", perfWarningsOnly,
                    "Emit performance setup warnings, then exit without "
                    "simulating or writing STL output.");
  cmdLine.addTarget("perf-advice", perfAdvice,
                    "Run fast option suitability analysis, then exit "
                    "without rendering or writing STL output.");
  cmdLine.addTarget("render-mode", renderMode,
                    "Render surface generation mode.");
  cmdLine.addTarget("resolution", resolution, "Valid values are 'low', "
                    "'medium', 'high' or a decimal value.");
  cmdLine.addTarget("threads", threads, "Number of simulation threads.");
  cmdLine.addTarget("toolsweep-xy-bins", toolSweepXYBins,
                    "Use an advanced ToolSweep XY bin count.  Zero "
                    "disables XY bins.  64 is the recommended opt-in "
                    "tiny-bit tuning value.");
  cmdLine.addTarget("toolsweep-xyz-bins", toolSweepXYZBins,
                    "Use an experimental ToolSweep XYZ bin count.  "
                    "Zero disables XYZ bins.  When enabled, XYZ bins "
                    "take precedence over XY bins.");
  cmdLine.addTarget("toolsweep-stock-bounds", toolSweepStockBounds,
                    "Experimental exact-preserving opt-in: tighten "
                    "ToolSweep boxes to the workpiece sampling Z slab.");
  cmdLine.addTarget("dexel", dexel,
                    "Experimental opt-in single-interval Z-dexel "
                    "candidate with automatic full-MC fallback.");
  cmdLine.addTarget("dexel-eligibility-only", dexelEligibilityOnly,
                    "Classify Z-dexel eligibility and exit without "
                    "rendering or writing STL output.");
  cmdLine.addTarget("dexel-skip-topology-validation",
                    dexelSkipTopologyValidation,
                    "Advanced opt-in: skip per-run dexel topology "
                    "validation after the deterministic builder has "
                    "passed the retained validation suite.");
  cmdLine.addTarget("dexel-grid-png", dexelGridPNG,
                    "Write the accepted Dexel height grid as a true "
                    "8-bit grayscale PNG.  Deepest is black, highest "
                    "is white.  Requires --dexel.");
  cmdLine.addTarget("surface-task-cancel-phase", surfaceTaskCancelPhase,
                    "Test-only: cancel shared Auto Dexel orchestration "
                    "during eligibility, preparing, rasterizing, "
                    "building, or validating.");
  cmdLine.addTarget("sparse-toolpath", sparseToolpath,
                    "Experimental opt-in: render only toolpath-adjacent "
                    "uncertain regions and analytically stitch untouched "
                    "stock from sparse ownership boundaries.");
  cmdLine.addTarget("sparse-toolpath-xy-bins", sparseToolpathXYBins,
                    "Sparse toolpath planner XY tile count per axis.  "
                    "Default is 64.");
  cmdLine.addTarget("sparse-toolpath-halo-cells",
                    sparseToolpathHaloCells,
                    "Sparse toolpath untouched halo width in simulation "
                    "cells.  Must be at least 1.");
  cmdLine.addTarget("sparse-toolpath-target-region-cells",
                    sparseToolpathTargetRegionCells,
                    "Sparse adaptive ownership target cells per active "
                    "leaf.  Zero disables the target.");
  cmdLine.addTarget("adaptive-z-slabs", adaptiveZSlabs,
                    "Report adaptive Z-slab planning metrics without "
                    "changing simulation output.");
  cmdLine.addTarget("adaptive-z-render", adaptiveZRender,
                    "Advanced opt-in: render only the adaptive active Z "
                    "slab and append simple untouched lower-stock "
                    "geometry when possible.  Requires "
                    "--adaptive-z-slabs.");
  cmdLine.addTarget("adaptive-z-slab-height", adaptiveZSlabHeight,
                    "Adaptive Z-slab planning slab height in project "
                    "units.  Zero chooses a conservative default.");
  cmdLine.addTarget("adaptive-z-initial-depth", adaptiveZInitialDepth,
                    "Adaptive Z-slab planning initial active depth in "
                    "project units.  Zero chooses a conservative "
                    "default.");
  cmdLine.addTarget("adaptive-z-margin", adaptiveZMargin,
                    "Adaptive Z-slab planning safety margin in project "
                    "units.  Zero uses two simulation cells.");
  cmdLine.addTarget("adaptive-z-region-bins", adaptiveZRegionBins,
                    "Report adaptive Z regional planning metrics using "
                    "this many XY regions per axis.  Zero disables "
                    "regional planning.");
  cmdLine.addTarget("adaptive-z-region-render", adaptiveZRegionRender,
                    "Experimental: render adaptive Z regions with local "
                    "active depths.  Requires --adaptive-z-slabs and "
                    "--adaptive-z-region-bins.");
  cmdLine.addTarget("profile", profile, "Write JSON performance profile.");
}
