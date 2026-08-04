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
#include <camotics/sim/SparseToolpathStages.h>

#include <cbang/ApplicationMain.h>
#include <cbang/os/SystemInfo.h>
#include <cbang/os/SystemUtilities.h>

#ifdef HAVE_V8
#include <cbang/js/v8/JSImpl.h>
#endif

using namespace std;
using namespace cb;
using namespace CAMotics;


class CamsimPathApp : public CAMotics::Application {
  SparseToolpath::PathStageOptions stageOptions;
  string profile;
  string output;

public:
  CamsimPathApp() :
    CAMotics::Application("CAMotics Sparse Toolpath Path Stage") {
    stageOptions.threads = SystemInfo::instance().getCPUCount();

    cmdLine.setUsageArgs("[OPTIONS] <project.camotics | input.gcode | "
                         "input.tpl> <toolpath.json>");
    cmdLine.setAllowConfigAsFirstArg(false);
    cmdLine.setAllowPositionalArgs(true);
    cmdLine.addTarget("time", stageOptions.time,
                      "Simulation end time in seconds.");
    cmdLine.addTarget("render-mode", stageOptions.renderMode,
                      "Render surface generation mode.");
    cmdLine.addTarget("resolution", stageOptions.resolution,
                      "Valid values are 'low', 'medium', 'high' or a "
                      "decimal value.");
    cmdLine.addTarget("threads", stageOptions.threads,
                      "Number of simulation threads.");
    cmdLine.addTarget("toolsweep-xy-bins", stageOptions.toolSweepXYBins,
                      "ToolSweep XY bin count.");
    cmdLine.addTarget("toolsweep-xyz-bins", stageOptions.toolSweepXYZBins,
                      "ToolSweep XYZ bin count.");
    cmdLine.addTarget("profile", profile, "Write JSON performance profile.");
  }

  int init(int argc, char *argv[]) override {
    int ret = CAMotics::Application::init(argc, argv);
    if (ret == -1) return ret;

    vector<string> args = cmdLine.getPositionalArgs();
    if (args.size() != 2)
      THROW("Expected input project/GCode/TPL and output toolpath JSON.");
    stageOptions.input = args[0];
    output = args[1];
    return 0;
  }

  void run() override {
    if (!profile.empty()) Profile::start(profile);
    SmartPointer<ostream> stream = SystemUtilities::oopen(output);
    SparseToolpath::writeToolpathArtifact(stageOptions, *stream);
    Profile::write();
  }
};


int main(int argc, char *argv[]) {
#ifdef HAVE_V8
  cb::gv8::JSImpl::init(0, 0);
#endif
  return doApplication<CamsimPathApp>(argc, argv);
}
