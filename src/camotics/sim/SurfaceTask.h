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

#pragma once


#include <camotics/Task.h>
#include <camotics/sim/DexelSimulation.h>

#include <cbang/SmartPointer.h>

#include <cstdint>


namespace CAMotics {
  class Simulation;
  class SimulationRun;
  class Surface;
  class SurfaceTask : public Task {
    cb::SmartPointer<SimulationRun> simRun;
    cb::SmartPointer<Surface> surface;
    SimulationBackend backend = SimulationBackend::FULL_MC;
    bool dexelAttempted = false;
    bool fallback = false;
    Dexel::RejectionReason fallbackReason = Dexel::RejectionReason::NONE;
    bool targetTimeSet = false;
    double targetTime = 0;
    uint64_t generation = 0;
    bool retainDexelState = false;
    bool buildDexelBoundary = true;
    bool retainDexelGrid = false;

  public:
    SurfaceTask(const Simulation &sim);
    SurfaceTask(const Simulation &sim, bool retainDexelState,
                bool buildDexelBoundary = true,
                bool retainDexelGrid = false);
    SurfaceTask(const cb::SmartPointer<SimulationRun> &simRun);
    SurfaceTask(const cb::SmartPointer<SimulationRun> &simRun,
                double targetTime, uint64_t generation,
                bool buildDexelBoundary = true);
    ~SurfaceTask();

    const cb::SmartPointer<SimulationRun> &getSimRun() const {return simRun;}
    const cb::SmartPointer<Surface> &getSurface() const {return surface;}
    SimulationBackend getBackend() const {return backend;}
    bool wasDexelAttempted() const {return dexelAttempted;}
    bool hasFallbackReason() const {return fallback;}
    Dexel::RejectionReason getFallbackReason() const {return fallbackReason;}
    bool hasTargetTime() const {return targetTimeSet;}
    double getTargetTime() const {return targetTime;}
    uint64_t getGeneration() const {return generation;}

    // From Task
    void run() override;
  };
}
