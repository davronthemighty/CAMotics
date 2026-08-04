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
#include <camotics/sim/SparseToolpathArtifacts.h>
#include <camotics/sim/SparseToolpathStages.h>

#include <cbang/ApplicationMain.h>
#include <cbang/os/SystemUtilities.h>

using namespace std;
using namespace cb;
using namespace CAMotics;


class CamsimStitchStockApp : public CAMotics::Application {
  string regionPlan;
  string regionSurface;
  string ownershipBoundary;
  string output;
  string profile;

public:
  CamsimStitchStockApp() :
    CAMotics::Application("CAMotics Sparse Toolpath Stitch Stock Stage") {
    cmdLine.setUsageArgs("[OPTIONS] <region-plan.json> "
                         "<region-surface.json> <stitched-surface.json>");
    cmdLine.setAllowConfigAsFirstArg(false);
    cmdLine.setAllowPositionalArgs(true);
    cmdLine.addTarget("ownership-boundary", ownershipBoundary,
                      "Optional ownership boundary artifact to validate and "
                      "record in the stitched surface artifact.");
    cmdLine.addTarget("profile", profile, "Write JSON performance profile.");
  }

  int init(int argc, char *argv[]) override {
    int ret = CAMotics::Application::init(argc, argv);
    if (ret == -1) return ret;

    vector<string> args = cmdLine.getPositionalArgs();
    if (args.size() != 3)
      THROW("Expected region plan JSON, region surface artifact, and output "
            "stitched surface artifact.");
    regionPlan = args[0];
    regionSurface = args[1];
    output = args[2];
    return 0;
  }

  void run() override {
    if (!profile.empty()) Profile::start(profile);
    SmartPointer<Simulation> regionPlanSim;
    SparseToolpath::RegionPlan plan =
      SparseToolpath::readRegionPlanArtifact(regionPlan, regionPlanSim);
    SparseToolpath::ArtifactContract regionPlanContract =
      SparseToolpath::readArtifactContract
      (regionPlan, SparseToolpath::REGION_PLAN_ARTIFACT);
    SmartPointer<Simulation> regionSurfaceSim =
      SparseToolpath::readSimulationArtifact
      (regionSurface, SparseToolpath::REGION_SURFACE_ARTIFACT);
    SparseToolpath::ArtifactContract regionSurfaceContract =
      SparseToolpath::readArtifactContract
      (regionSurface, SparseToolpath::REGION_SURFACE_ARTIFACT);
    if (regionPlanContract.inputHash != regionSurfaceContract.inputHash ||
        regionPlanContract.toolpathHash != regionSurfaceContract.toolpathHash ||
        regionPlanContract.regionPlanHash !=
        regionSurfaceContract.regionPlanHash)
      THROW("Region plan and region surface artifact contracts do not match.");
    SparseToolpath::OwnershipBoundaryPlan boundary;
    SparseToolpath::OwnershipBoundaryPlan *boundaryPtr = 0;

    if (!ownershipBoundary.empty()) {
      SmartPointer<Simulation> boundarySim;
      boundary =
        SparseToolpath::readOwnershipBoundaryArtifact
        (ownershipBoundary, boundarySim);
      SparseToolpath::ArtifactContract boundaryContract =
        SparseToolpath::readArtifactContract
        (ownershipBoundary, SparseToolpath::OWNERSHIP_BOUNDARY_ARTIFACT);
      if (boundarySim->computeHash() != regionPlanSim->computeHash())
        THROW("Ownership boundary artifact does not match region plan "
              "simulation hash.");
      if (boundaryContract.inputHash != regionPlanContract.inputHash ||
          boundaryContract.toolpathHash != regionPlanContract.toolpathHash ||
          boundaryContract.regionPlanHash != regionPlanContract.regionPlanHash)
        THROW("Ownership boundary artifact contract does not match region "
              "plan contract.");
      regionSurfaceContract.ownershipBoundaryHash =
        boundaryContract.ownershipBoundaryHash;
      boundaryPtr = &boundary;
    }

    SmartPointer<ostream> stream = SystemUtilities::oopen(output);
    SparseToolpath::writeStitchedSurfaceArtifact
      (plan, boundaryPtr, *regionSurfaceSim, regionSurfaceContract, *stream);
    Profile::write();
  }
};


int main(int argc, char *argv[]) {
  return doApplication<CamsimStitchStockApp>(argc, argv);
}
