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


#include "Simulation.h"

#include <cbang/SmartPointer.h>

#include <memory>


namespace CAMotics {
  class ToolSweep;
  class GridTree;
  class Surface;
  class MoveLookup;
  class Task;
  namespace Dexel {class State;}


  class SimulationRun {
    Simulation sim;
    cb::SmartPointer<ToolSweep> sweep;
    cb::SmartPointer<GridTree> tree;
    std::shared_ptr<Dexel::State> dexelState;

    double lastTime = 0;

  public:
    SimulationRun(const Simulation &sim);
    ~SimulationRun();

    Simulation &getSimulation() {return sim;}

    cb::SmartPointer<MoveLookup> getMoveLookup() const;
    bool isInitialized() const {return !sweep.isNull();}
    bool hasDexelState() const {return (bool)dexelState;}
    const std::shared_ptr<Dexel::State> &getDexelState() const
    {return dexelState;}
    void setDexelState(const std::shared_ptr<Dexel::State> &state)
    {dexelState = state;}

    void setEndTime(double endTime);

    cb::SmartPointer<Surface> compute(Task &task);
  };
}
