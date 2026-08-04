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


class CamsimRenderRegionsApp : public CAMotics::Application {
  string toolpath;
  string regionPlan;
  string output;
  string profile;
  unsigned threads = 0;

public:
  CamsimRenderRegionsApp() :
    CAMotics::Application("CAMotics Sparse Toolpath Render Regions Stage") {
    cmdLine.setUsageArgs("[OPTIONS] <toolpath.json> <region-plan.json> "
                         "<region-surface.json>");
    cmdLine.setAllowConfigAsFirstArg(false);
    cmdLine.setAllowPositionalArgs(true);
    cmdLine.addTarget("threads", threads,
                      "Override simulation thread count.  Zero uses the "
                      "toolpath artifact setting.");
    cmdLine.addTarget("profile", profile, "Write JSON performance profile.");
  }

  int init(int argc, char *argv[]) override {
    int ret = CAMotics::Application::init(argc, argv);
    if (ret == -1) return ret;

    vector<string> args = cmdLine.getPositionalArgs();
    if (args.size() != 3)
      THROW("Expected toolpath JSON, region plan JSON, and output region "
            "surface artifact.");
    toolpath = args[0];
    regionPlan = args[1];
    output = args[2];
    return 0;
  }

  void run() override {
    if (!profile.empty()) Profile::start(profile);
    SmartPointer<Simulation> toolpathSim =
      SparseToolpath::readSimulationArtifact
      (toolpath, SparseToolpath::TOOLPATH_ARTIFACT);
    SparseToolpath::ArtifactContract toolpathContract =
      SparseToolpath::readArtifactContract
      (toolpath, SparseToolpath::TOOLPATH_ARTIFACT);
    SmartPointer<Simulation> regionPlanSim;
    SparseToolpath::RegionPlan plan =
      SparseToolpath::readRegionPlanArtifact(regionPlan, regionPlanSim);
    SparseToolpath::ArtifactContract regionPlanContract =
      SparseToolpath::readArtifactContract
      (regionPlan, SparseToolpath::REGION_PLAN_ARTIFACT);
    if (toolpathContract.inputHash != regionPlanContract.inputHash ||
        toolpathContract.toolpathHash != regionPlanContract.toolpathHash)
      THROW("Toolpath and region plan artifact contracts do not match.");
    SmartPointer<ostream> stream = SystemUtilities::oopen(output);
    SparseToolpath::renderRegionSurfaceArtifact
      (*toolpathSim, plan, regionPlanContract, threads, *stream);
    Profile::write();
  }
};


int main(int argc, char *argv[]) {
  return doApplication<CamsimRenderRegionsApp>(argc, argv);
}
