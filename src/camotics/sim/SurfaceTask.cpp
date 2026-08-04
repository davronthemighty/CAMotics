/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>

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

#include "SurfaceTask.h"

#include <camotics/sim/SimulationRun.h>
#include <camotics/sim/DexelSimulation.h>
#include <camotics/contour/Surface.h>
#include <camotics/Profile.h>

#include <cbang/String.h>
#include <cbang/time/Timer.h>
#include <cbang/time/TimeInterval.h>
#include <cbang/log/Logger.h>

#include <utility>
#include <vector>

using namespace cb;
using namespace CAMotics;


SurfaceTask::SurfaceTask(const Simulation &sim) :
  simRun(new SimulationRun(sim)) {}


SurfaceTask::SurfaceTask(const Simulation &sim, bool retainDexelState,
                         bool buildDexelBoundary, bool retainDexelGrid) :
  simRun(new SimulationRun(sim)), retainDexelState(retainDexelState),
  buildDexelBoundary(buildDexelBoundary), retainDexelGrid(retainDexelGrid) {}


SurfaceTask::SurfaceTask(const SmartPointer<SimulationRun> &simRun) :
  simRun(simRun) {}


SurfaceTask::SurfaceTask(const SmartPointer<SimulationRun> &simRun,
                         double targetTime, uint64_t generation,
                         bool buildDexelBoundary) :
  simRun(simRun), targetTimeSet(true), targetTime(targetTime),
  generation(generation), buildDexelBoundary(buildDexelBoundary) {}


SurfaceTask::~SurfaceTask() {}


void SurfaceTask::run() {
  double startTime = Timer::now();

  // Timeline requests are captured by value on the GUI thread and applied only
  // after this task owns the SimulationRun.  This keeps the GUI from mutating
  // simulation state while a worker is reading it.
  if (targetTimeSet) simRun->setEndTime(targetTime);

  Simulation &sim = simRun->getSimulation();
  bool autoDexel =
    sim.backendPolicy == SimulationBackendPolicy::AUTO_DEXEL;
  Profile::setMetric("simulation_backend_requested_auto_dexel", autoDexel);
  auto attachBoundaryTiles =
    [&] (Dexel::CandidateResult &candidate, bool full) {
      if (!buildDexelBoundary) return true;
      if (!candidate.state || !candidate.state->hasBoundaryOwners())
        return true;
      Dexel::GridSurface *grid =
        dynamic_cast<Dexel::GridSurface *>(candidate.surface.get());
      if (!grid) return true;
      std::vector<Dexel::BoundaryTile> tiles =
        candidate.state->buildBoundaryTiles
          (grid->getDirtyTiles(), full, this);
      if (shouldQuit()) return false;
      candidate.state->publishBoundaryTiles(*grid, std::move(tiles));
      return true;
    };

  bool stateAttempted = false;
  if (autoDexel && targetTimeSet && simRun->hasDexelState()) {
    stateAttempted = true;
    dexelAttempted = true;
    Profile::setMetric("simulation_backend_dexel_attempted", 1);
    Dexel::CandidateResult candidate =
      simRun->getDexelState()->seek
        (targetTime, this, buildDexelBoundary);
    if (shouldQuit() || candidate.reason == Dexel::RejectionReason::CANCELLED)
      return;

    if (candidate.accepted) {
      backend = SimulationBackend::DEXEL;
      surface = candidate.surface;
      simRun->setDexelState(candidate.state);
      Profile::setMetric("dexel_candidate_accepted", 1);
      Profile::setMetric("simulation_backend_selected_dexel", 1);
      LOG_INFO(1, "Simulation backend: dexel-state");
    } else {
      fallback = true;
      fallbackReason = candidate.reason;
      Dexel::recordFallback(fallbackReason);
      LOG_WARNING("Dexel state fallback to full MC: "
                  << Dexel::reasonName(fallbackReason));
    }
  }

  if (autoDexel && !stateAttempted && !simRun->isInitialized()) {
    dexelAttempted = true;
    Profile::setMetric("simulation_backend_dexel_attempted", 1);
    begin("Checking dexel eligibility");
    Dexel::EligibilityReport eligibility = Dexel::classify(sim, this);
    Dexel::recordEligibilityMetrics(eligibility);

    if (shouldQuit()) return;

    if (eligibility.eligible) {
      LOG_INFO(1, "Dexel eligibility accepted: moves="
               << eligibility.movesChecked << " tools="
               << eligibility.toolsChecked);
      Dexel::CandidateResult candidate = Dexel::compute
        (sim, sim.validateDexelTopology, this, &eligibility,
         retainDexelState, retainDexelState, retainDexelGrid);

      if (shouldQuit() || candidate.reason == Dexel::RejectionReason::CANCELLED)
        return;

      if (candidate.accepted) {
        if (!attachBoundaryTiles(candidate, true)) return;
        backend = SimulationBackend::DEXEL;
        surface = candidate.surface;
        simRun->setDexelState(candidate.state);
        Profile::setMetric("dexel_candidate_accepted", 1);
      } else {
        fallback = true;
        fallbackReason = candidate.reason;
      }

    } else {
      fallback = true;
      fallbackReason = eligibility.reason;
    }

    if (!surface.isNull()) {
      Profile::setMetric("simulation_backend_selected_dexel", 1);
      LOG_INFO(1, "Simulation backend: dexel");
    } else {
      Dexel::recordFallback(fallbackReason);
      LOG_WARNING("Dexel fallback to full MC: "
                  << Dexel::reasonName(fallbackReason));
    }
  }

  if (surface.isNull()) {
    if (shouldQuit()) return;
    backend = SimulationBackend::FULL_MC;
    surface = simRun->compute(*this);
    if (!shouldQuit()) {
      Profile::setMetric("simulation_backend_selected_full_mc", 1);
      LOG_INFO(1, "Simulation backend: full-mc");
    }
  }

  // Time
  if (shouldQuit()) {
    LOG_INFO(1, "Render aborted");
    return;
  }

  if (surface.isNull()) return;

  // Done
  double delta = Timer::now() - startTime;
  unsigned triangles = surface->getTriangleCount();
  LOG_INFO(1, "Time: " << TimeInterval(delta)
           << " Triangles: " << triangles
           << " Triangles/sec: " << String::printf("%0.2f", triangles / delta));
}
