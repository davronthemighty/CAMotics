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


class CamsimBoundaryPlanApp : public CAMotics::Application {
  string regionPlan;
  string output;
  string profile;

public:
  CamsimBoundaryPlanApp() :
    CAMotics::Application("CAMotics Sparse Toolpath Boundary Plan Stage") {
    cmdLine.setUsageArgs("[OPTIONS] <region-plan.json> "
                         "<ownership-boundary.json>");
    cmdLine.setAllowConfigAsFirstArg(false);
    cmdLine.setAllowPositionalArgs(true);
    cmdLine.addTarget("profile", profile, "Write JSON performance profile.");
  }

  int init(int argc, char *argv[]) override {
    int ret = CAMotics::Application::init(argc, argv);
    if (ret == -1) return ret;

    vector<string> args = cmdLine.getPositionalArgs();
    if (args.size() != 2)
      THROW("Expected input region plan JSON and output ownership boundary "
            "JSON.");
    regionPlan = args[0];
    output = args[1];
    return 0;
  }

  void run() override {
    if (!profile.empty()) Profile::start(profile);
    SmartPointer<Simulation> sim;
    SparseToolpath::RegionPlan plan =
      SparseToolpath::readRegionPlanArtifact(regionPlan, sim);
    SparseToolpath::ArtifactContract contract =
      SparseToolpath::readArtifactContract
      (regionPlan, SparseToolpath::REGION_PLAN_ARTIFACT);
    SmartPointer<ostream> stream = SystemUtilities::oopen(output);
    SparseToolpath::writeOwnershipBoundaryArtifact
      (*sim, plan, contract, *stream);
    Profile::write();
  }
};


int main(int argc, char *argv[]) {
  return doApplication<CamsimBoundaryPlanApp>(argc, argv);
}
