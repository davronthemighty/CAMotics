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


class CamsimRegionPlanApp : public CAMotics::Application {
  SparseToolpath::RegionPlanOptions stageOptions;
  string input;
  string output;
  string profile;

public:
  CamsimRegionPlanApp() :
    CAMotics::Application("CAMotics Sparse Toolpath Region Plan Stage") {
    cmdLine.setUsageArgs("[OPTIONS] <toolpath.json> <region-plan.json>");
    cmdLine.setAllowConfigAsFirstArg(false);
    cmdLine.setAllowPositionalArgs(true);
    cmdLine.addTarget("xy-bins", stageOptions.xyBins,
                      "Initial adaptive XY bin count.");
    cmdLine.addTarget("halo-cells", stageOptions.haloCells,
                      "Untouched ownership halo width in grid cells.");
    cmdLine.addTarget("target-region-cells", stageOptions.targetRegionCells,
                      "Maximum planned cells per adaptive active leaf.  "
                      "Zero disables the cell target.");
    cmdLine.addTarget("profile", profile, "Write JSON performance profile.");
  }

  int init(int argc, char *argv[]) override {
    int ret = CAMotics::Application::init(argc, argv);
    if (ret == -1) return ret;

    vector<string> args = cmdLine.getPositionalArgs();
    if (args.size() != 2)
      THROW("Expected input toolpath JSON and output region plan JSON.");
    input = args[0];
    output = args[1];
    return 0;
  }

  void run() override {
    if (!profile.empty()) Profile::start(profile);
    SmartPointer<Simulation> sim = SparseToolpath::readSimulationArtifact
      (input, SparseToolpath::TOOLPATH_ARTIFACT);
    SparseToolpath::ArtifactContract contract =
      SparseToolpath::readArtifactContract
      (input, SparseToolpath::TOOLPATH_ARTIFACT);
    SmartPointer<ostream> stream = SystemUtilities::oopen(output);
    SparseToolpath::writeRegionPlanArtifact
      (*sim, stageOptions, contract, *stream);
    Profile::write();
  }
};


int main(int argc, char *argv[]) {
  return doApplication<CamsimRegionPlanApp>(argc, argv);
}
