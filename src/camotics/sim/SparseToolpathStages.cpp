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

#include "SparseToolpathStages.h"
#include "SparseToolpathInternal.h"

#include "CutSim.h"
#include "SparseToolpathArtifacts.h"

#include <camotics/Profile.h>
#include <camotics/SHA256.h>
#include <camotics/project/Project.h>
#include <camotics/project/ResolutionMode.h>

#include <gcode/ToolPath.h>

#include <cbang/Exception.h>
#include <cbang/String.h>
#include <cbang/json/Sink.h>
#include <cbang/net/Base64.h>
#include <cbang/os/SystemUtilities.h>

#include <cstdint>
#include <fstream>
#include <limits>

using namespace std;
using namespace cb;
using namespace CAMotics;
using namespace CAMotics::SparseToolpath;
using namespace CAMotics::SparseToolpath::Internal;


namespace {
  string hashInputFile(const string &filename) {
    ifstream stream(filename, ios::binary);
    if (!stream) THROW("Failed to open sparse input '" << filename << "'.");

    SHA256 sha256;
    char buffer[65536];
    while (stream) {
      stream.read(buffer, sizeof(buffer));
      if (stream.gcount()) sha256.update(buffer, stream.gcount());
    }
    if (!stream.eof())
      THROW("Failed to hash sparse input '" << filename << "'.");
    return Base64().encode(sha256.finalize());
  }


  void applyResolutionOverride(Project::Project &project,
                               const string &resolution) {
    if (resolution.empty()) return;

    ResolutionMode resMode = ResolutionMode::RESOLUTION_MANUAL;
    double res = 0;

    try {
      res = String::parseDouble(resolution);
    } catch (const Exception &e) {}

    if (res) project.setResolution(res);
    else resMode = ResolutionMode::parse(resolution, resMode);

    project.setResolutionMode(resMode);
  }


  Project::Project loadProjectInput(const string &input,
                                    const string &resolution) {
    Project::Project project;
    string ext = SystemUtilities::extension(input);
    if (ext == "xml" || ext == "camotics") project.load(input);
    else project.addFile(input);

    applyResolutionOverride(project, resolution);
    return project;
  }


}


void SparseToolpath::writeToolpathArtifact(const PathStageOptions &options,
                                           ostream &stream) {
  Project::Project project =
    loadProjectInput(options.input, options.resolution);
  CutSim cutSim;

  SmartPointer<GCode::ToolPath> path;
  {
    Profile::Scope scope("toolpath_generation");
    path = cutSim.computeToolPath(project);
  }

  project.getWorkpiece().update(*path);
  Rectangle3D bounds = project.getWorkpiece().getBounds();

  double simTime =
    options.time ? options.time : numeric_limits<double>::max();
  Simulation sim(path, 0, 0, bounds, project.getResolution(), simTime,
                 options.renderMode, options.threads,
                 options.toolSweepXYBins, options.toolSweepXYZBins);

  Profile::setMetric("sparse_path_moves", path->size());
  Profile::setMetric("sparse_path_time_micros",
                     (uint64_t)(path->getTime() * 1000000.0));
  Profile::setMetric("sparse_path_distance_microunits",
                     (uint64_t)(path->getDistance() * 1000000.0));

  ArtifactContract contract;
  contract.inputHash = hashInputFile(options.input);
  contract.toolpathHash = computeToolpathHash(sim);

  writeSimulationArtifact
    (stream, TOOLPATH_ARTIFACT, sim, contract, [&] (JSON::Sink &sink) {
      sink.beginInsert("toolpath-artifact");
      sink.beginDict();
      sink.insert("input", options.input);
      sink.insert("moves", (uint64_t)path->size());
      sink.insert("time", path->getTime());
      sink.insert("distance", path->getDistance());
      sink.insert("resolution", project.getResolution());
      sink.insert("render-mode", options.renderMode.toString());
      sink.insert("threads", options.threads);
      sink.insert("toolsweep-xy-bins", options.toolSweepXYBins);
      sink.insert("toolsweep-xyz-bins", options.toolSweepXYZBins);
      writeBounds(sink, "workpiece-bounds", bounds);
      writeBounds(sink, "toolpath-bounds", path->getBounds());
      sink.endDict();
    });
}


void SparseToolpath::writeRegionPlanArtifact
(const Simulation &toolpathSim, const RegionPlanOptions &options,
 const ArtifactContract &inputContract, ostream &stream) {
  RegionPlan plan = planRegions(toolpathSim, options);
  ArtifactContract contract = inputContract;
  contract.regionPlanHash = computeRegionPlanHash(plan);

  writeSimulationArtifact
    (stream, REGION_PLAN_ARTIFACT, toolpathSim, contract,
     [&] (JSON::Sink &sink) {
      sink.beginInsert("region-plan");
      plan.write(sink);
    });
}


SmartPointer<Surface> SparseToolpath::computeSparseSurface
(const Simulation &sim, const RegionPlanOptions &options, unsigned threads) {
  Profile::setMetric("sparse_toolpath_integrated_enabled", 1);
  Profile::setMetric("sparse_toolpath_integrated_full_coverage_fallback", 0);
  Profile::setMetric("sparse_toolpath_integrated_geometry_fallback", 0);

  RegionPlan regionPlan;
  {
    Profile::Scope scope("sparse_region_plan");
    regionPlan = planRegions(sim, options);
  }

  if (planNeedsFullCoverageFallback(regionPlan, sim.resolution)) {
    Profile::setMetric("sparse_toolpath_integrated_full_coverage_fallback", 1);
    Profile::setMetric("sparse_toolpath_integrated_topology_fallback", 0);
    Profile::Scope scope("sparse_full_coverage_render_fallback");
    return renderFullBaseline(sim, threads);
  }

  OwnershipBoundaryPlan boundaryPlan;
  {
    Profile::Scope scope("sparse_boundary_plan");
    boundaryPlan = planOwnershipBoundaries(regionPlan);
  }

  SparseSurfaceResult candidate =
    buildSparseCandidate(sim, regionPlan, boundaryPlan, threads);
  if (candidate.accepted()) {
    Profile::setMetric("sparse_toolpath_integrated_topology_fallback", 0);
    return candidate.surface;
  }

  Profile::setMetric("sparse_toolpath_integrated_topology_fallback",
                     candidate.topologyAccepted ? 0 : 1);
  Profile::setMetric("sparse_toolpath_integrated_geometry_fallback",
                     candidate.geometryAccepted ? 0 : 1);
  Profile::Scope scope("sparse_full_render_fallback");
  return renderFullBaseline(sim, threads);
}


void SparseToolpath::writeReduceExport(const Simulation &stitchedSim,
                                       ostream &stl, bool binary,
                                       bool reduce) {
  if (stitchedSim.surface.isNull())
    THROW("Stitched surface artifact has no surface.");

  SmartPointer<Surface> surface = stitchedSim.surface->copy();
  if (reduce) {
    CutSim cutSim;
    Profile::Scope scope("sparse_reduce_export_legacy_reduction");
    cutSim.reduceSurface(surface);
  }

  Profile::setMetric("sparse_reduce_export_triangles",
                     surface->getTriangleCount());
  surface->writeSTL(stl, binary, "CAMotics Sparse Surface",
                    stitchedSim.computeHash());
}
