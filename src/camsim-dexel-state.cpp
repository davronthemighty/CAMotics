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
#include <camotics/Task.h>
#include <camotics/contour/FieldFunction.h>
#include <camotics/project/Project.h>
#include <camotics/sim/CutSim.h>
#include <camotics/sim/DexelSimulation.h>
#include <camotics/sim/Simulation.h>

#include <gcode/ToolPath.h>

#include <cbang/ApplicationMain.h>
#include <cbang/Exception.h>
#include <cbang/os/SystemInfo.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using namespace std;
using namespace cb;
using namespace CAMotics;


namespace {
  string payload(const SmartPointer<Surface> &surface,
                 const Simulation &sim) {
    ostringstream stream(ios::out | ios::binary);
    surface->writeSTL(stream, true, "Dexel state contract",
                      sim.computeHash());
    string data = stream.str();
    if (data.size() < 84) THROW("Dexel state contract wrote a short STL");
    return data.substr(80);
  }


  class CancellingTask : public Task {
    string phase;

  public:
    CancellingTask(const string &phase) : phase(phase) {}

    void updated(const string &status, double progress) override {
      if (status == phase) interrupt();
    }
  };
}


class CamsimDexelStateApp : public CAMotics::Application {
  string input;
  unsigned threads;

public:
  CamsimDexelStateApp() :
    CAMotics::Application("CAMotics Dexel State Contract"),
    threads(SystemInfo::instance().getCPUCount()) {
    cmdLine.setUsageArgs("[OPTIONS] <project.camotics>");
    cmdLine.setAllowConfigAsFirstArg(false);
    cmdLine.setAllowPositionalArgs(true);
    cmdLine.addTarget("threads", threads, "Number of raster worker threads.");
  }

  int init(int argc, char *argv[]) override {
    int ret = CAMotics::Application::init(argc, argv);
    if (ret == -1) return ret;
    vector<string> args = cmdLine.getPositionalArgs();
    if (args.size() != 1) THROW("Expected one CAMotics project path");
    input = args[0];
    return 0;
  }

  void run() override {
    Project::Project project;
    project.load(input);
    CutSim cutSim;
    SmartPointer<GCode::ToolPath> path = cutSim.computeToolPath(project);
    Rectangle3D bounds = project.getWorkpiece().getBounds();
    project.getWorkpiece().update(*path);
    Simulation sim(path, 0, 0, bounds, project.getResolution(),
                   numeric_limits<double>::max(), RenderMode(), threads);

    Dexel::CandidateResult oneShot = Dexel::compute(sim, false);
    if (!oneShot.accepted || oneShot.surface.isNull())
      THROW("One-shot Dexel control was not accepted: "
            << Dexel::reasonName(oneShot.reason));

    auto makeState = [&] () {
      Dexel::CandidateResult candidate =
        Dexel::compute(sim, false, 0, 0, true, true);
      if (!candidate.accepted || candidate.surface.isNull() ||
          !candidate.state)
        THROW("Retained Dexel state was not accepted: "
              << Dexel::reasonName(candidate.reason));
      return candidate;
    };
    auto boundarySamples = [&] (const shared_ptr<Dexel::State> &state) {
      SmartPointer<FieldFunction> field = state->getBoundaryField();
      vector<double> samples;
      const Vector3D min = bounds.getMin();
      const Vector3D dims = bounds.getDimensions();
      for (unsigned z = 1; z < 4; z++)
        for (unsigned y = 1; y < 16; y++)
          for (unsigned x = 1; x < 16; x++)
            samples.push_back(field->depth
              (min + Vector3D(dims.x() * x / 16,
                              dims.y() * y / 16,
                              dims.z() * z / 4)));
      return samples;
    };
    auto boundaryTiles = [&] (const shared_ptr<Dexel::State> &state) {
      return state->buildBoundaryTiles({}, true);
    };
    auto sameBoundaryTiles =
      [] (const vector<Dexel::BoundaryTile> &a,
          const vector<Dexel::BoundaryTile> &b) {
        if (a.size() != b.size()) return false;
        for (unsigned i = 0; i < a.size(); i++)
          if (a[i].tile != b[i].tile ||
              a[i].cellX != b[i].cellX ||
              a[i].cellY != b[i].cellY ||
              a[i].cellsX != b[i].cellsX ||
              a[i].cellsY != b[i].cellsY ||
              a[i].cliffCells != b[i].cliffCells ||
              a[i].vertices != b[i].vertices ||
              a[i].normals != b[i].normals)
            return false;
        return true;
      };

    Dexel::CandidateResult retained = makeState();
    vector<Dexel::BoundaryTile> finalBoundary =
      boundaryTiles(retained.state);
    uint64_t finalBoundaryTriangles = 0;
    for (const auto &tile: finalBoundary)
      finalBoundaryTriangles += tile.getTriangleCount();
    if (!finalBoundaryTriangles)
      THROW("Retained boundary tiles produced no triangles");
    if (payload(oneShot.surface, sim) != payload(retained.surface, sim))
      THROW("Retained final surface differs from one-shot Dexel output");
    uint64_t callbackTriangles = 0;
    retained.surface->getVertices
      ([&] (const vector<float> &vertices, const vector<float> &normals) {
        if (vertices.size() != normals.size())
          THROW("Lazy legacy vertices and normals differ in size");
        callbackTriangles += vertices.size() / 9;
      });
    if (callbackTriangles != retained.surface->getTriangleCount())
      THROW("Lazy wireframe vertex path changed the triangle count");
    SmartPointer<Surface> reduced = retained.surface->copy();
    Task reduceTask;
    reduced->reduce(reduceTask);
    if (retained.surface->getTriangleCount() < reduced->getTriangleCount())
      THROW("Legacy Reduce increased the retained surface triangle count");
    if (!retained.state->getCheckpointCount() ||
        16 < retained.state->getCheckpointCount())
      THROW("Dexel checkpoint count is outside the contract");
    if (256ULL * 1024 * 1024 < retained.state->getCheckpointBytes())
      THROW("Dexel checkpoint bytes exceed the 256 MiB contract");

    const double total = path->getTime();
    const double time40 = total * 0.4;
    const double time70 = total * 0.7;
    Dexel::CandidateResult warm40 = retained.state->seek(time40);
    vector<double> warm40Boundary = boundarySamples(retained.state);
    vector<Dexel::BoundaryTile> warm40Tiles =
      boundaryTiles(retained.state);
    Dexel::CandidateResult warm70 = retained.state->seek(time70);
    vector<double> warm70Boundary = boundarySamples(retained.state);
    vector<Dexel::BoundaryTile> warm70Tiles =
      boundaryTiles(retained.state);
    if (!warm40.accepted || !warm70.accepted)
      THROW("Warm forward Dexel seek was rejected");

    Dexel::CandidateResult cold = makeState();
    Dexel::CandidateResult cold70 = cold.state->seek(time70);
    if (!cold70.accepted ||
        payload(warm70.surface, sim) != payload(cold70.surface, sim) ||
        warm70Boundary != boundarySamples(cold.state) ||
        !sameBoundaryTiles(warm70Tiles, boundaryTiles(cold.state)))
      THROW("Warm and cold 70% Dexel states differ");

    Dexel::CandidateResult reverse40 = retained.state->seek(time40);
    if (!reverse40.accepted ||
        payload(warm40.surface, sim) != payload(reverse40.surface, sim) ||
        warm40Boundary != boundarySamples(retained.state) ||
        !sameBoundaryTiles(warm40Tiles, boundaryTiles(retained.state)))
      THROW("Reverse-restored and cold 40% Dexel states differ");
    Dexel::CandidateResult repeat40 = retained.state->seek(time40);
    if (!repeat40.accepted ||
        payload(warm40.surface, sim) != payload(repeat40.surface, sim))
      THROW("Repeated 40% Dexel seek is not deterministic");

    const string stablePayload = payload(repeat40.surface, sim);
    const vector<double> stableBoundary = boundarySamples(retained.state);
    const double stableTime = retained.state->getTime();

    // Fast display LOD intentionally skips exact cliff reconstruction.  The
    // first subsequent full-detail request must rebuild a complete snapshot,
    // even when it seeks to the same height state and has no newly dirty tile.
    Dexel::CandidateResult skippedBoundary =
      retained.state->seek(total * 0.6, 0, false);
    Dexel::CandidateResult caughtUpBoundary =
      retained.state->seek(total * 0.6, 0, true);
    Dexel::GridSurface *caughtUpGrid = dynamic_cast<Dexel::GridSurface *>
      (caughtUpBoundary.surface.get());
    if (!skippedBoundary.accepted || !caughtUpBoundary.accepted ||
        !caughtUpGrid || caughtUpGrid->getBoundaryTiles().empty())
      THROW("Full-detail boundary catch-up was not published");
    for (const auto &tile: caughtUpGrid->getBoundaryTiles())
      if (!tile) THROW("Full-detail boundary snapshot is incomplete");
    Dexel::CandidateResult restoredStable =
      retained.state->seek(stableTime);
    if (!restoredStable.accepted ||
        payload(restoredStable.surface, sim) != stablePayload)
      THROW("Boundary catch-up changed the retained height state");

    CancellingTask restoreCancel("Restoring dexel state");
    Dexel::CandidateResult cancelled =
      retained.state->seek(total * 0.2, &restoreCancel);
    if (cancelled.reason != Dexel::RejectionReason::CANCELLED ||
        retained.state->getTime() != stableTime)
      THROW("Cancelled restore committed Dexel state");

    CancellingTask updateCancel("Updating dexel state");
    cancelled = retained.state->seek(stableTime, &updateCancel);
    if (cancelled.reason != Dexel::RejectionReason::CANCELLED ||
        retained.state->getTime() != stableTime)
      THROW("Cancelled incremental update committed Dexel state");

    CancellingTask publishCancel("Publishing dexel state");
    cancelled = retained.state->seek(total * 0.6, &publishCancel);
    if (cancelled.reason != Dexel::RejectionReason::CANCELLED ||
        retained.state->getTime() != stableTime)
      THROW("Cancelled publication committed Dexel state");

    CancellingTask boundaryCancel("Reconstructing dexel boundary");
    cancelled = retained.state->seek(total * 0.6, &boundaryCancel, true);
    if (cancelled.reason != Dexel::RejectionReason::CANCELLED ||
        retained.state->getTime() != stableTime)
      THROW("Cancelled boundary reconstruction committed Dexel state");
    Dexel::CandidateResult afterCancel = retained.state->seek(stableTime);
    if (!afterCancel.accepted ||
        stablePayload != payload(afterCancel.surface, sim) ||
        stableBoundary != boundarySamples(retained.state))
      THROW("Cancelled request changed the published Dexel payload");

    cout << "Dexel state contract passed: checkpoints="
         << retained.state->getCheckpointCount()
         << " checkpoint_bytes=" << retained.state->getCheckpointBytes()
         << " current_bytes=" << retained.state->getCurrentBytes()
         << " boundary_triangles=" << finalBoundaryTriangles << '\n';
  }
};


int main(int argc, char *argv[]) {
  return doApplication<CamsimDexelStateApp>(argc, argv);
}
