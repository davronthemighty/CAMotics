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

#include "Renderer.h"

#include "RenderJob.h"

#include <camotics/Grid.h>
#include <camotics/Profile.h>
#include <camotics/sim/CutWorkpiece.h>

#include <cbang/String.h>
#include <cbang/log/Logger.h>
#include <cbang/time/TimeInterval.h>
#include <cbang/time/Timer.h>
#include <cbang/thread/SmartLock.h>
#include <cbang/Catch.h>

#include <cmath>
#include <cstdint>
#include <list>

using namespace std;
using namespace cb;
using namespace CAMotics;


void Renderer::render(CutWorkpiece &cutWorkpiece, GridTree &tree,
                      const Rectangle3D &bbox, unsigned threads,
                      RenderMode mode) {
  stats = RenderStats();
  // Check for empty workpiece
  auto bounds = tree.getBounds();
  if (!bounds.isValid()) {
    LOG_WARNING("Empty workpiece, nothing to simulate");
    return;
  }

  typedef list<SmartPointer<RenderJob> > jobs_t;
  jobs_t jobs;
  vector<GridTreeRef> jobGrids;

  try {
    SmartLock lock(this);

    // Divide work
    unsigned targetJobCount = pow(2, ceil(log(threads) / log(2)) + 2);

    task.begin("Partitioning 3D space");
    {
      Profile::Scope scope("partitioning");
      tree.partition(jobGrids, bbox, targetJobCount);
    }
    unsigned totalJobCount = jobGrids.size();

    uint64_t treeCells = tree.getTotalCells();
    uint64_t partitionCells = 0;
    uint64_t minJobCells =
      jobGrids.empty() ? 0 : jobGrids.front().getTotalCells();
    uint64_t maxJobCells = 0;
    for (const auto &jobGrid: jobGrids) {
      uint64_t cells = jobGrid.getTotalCells();
      partitionCells += cells;
      if (cells < minJobCells) minJobCells = cells;
      if (maxJobCells < cells) maxJobCells = cells;
    }

    Profile::setMetric("render_target_jobs", targetJobCount);
    Profile::setMetric("render_actual_jobs", totalJobCount);
    Profile::setMetric("render_tree_cells", treeCells);
    Profile::setMetric("render_partition_cells", partitionCells);
    uint64_t extraCells =
      treeCells < partitionCells ? partitionCells - treeCells : 0;
    uint64_t missingCells =
      partitionCells < treeCells ? treeCells - partitionCells : 0;
    bool fullTreeCoverage = bbox.contains(bounds);

    Profile::setMetric("render_partition_extra_cells", extraCells);
    Profile::setMetric("render_partition_missing_cells", missingCells);
    Profile::setMetric("render_partition_clipped_cells",
                       fullTreeCoverage ? 0 : missingCells);
    Profile::setMetric("render_partition_full_tree_coverage",
                       fullTreeCoverage ? 1 : 0);
    Profile::setMetric("render_job_cells_min", minJobCells);
    Profile::setMetric("render_job_cells_max", maxJobCells);

    LOG_DEBUG(1, "Partitioned in to " << totalJobCount << " jobs");
    LOG_INFO(1, "Computing surface bounded by " << bounds << " at "
             << tree.getResolution() << " grid resolution");

    // Run jobs
    double lastUpdate = 0;
    task.begin("Computing cut surface");
    Profile::Scope renderScope("render");
    while (!task.shouldQuit() && !(jobGrids.empty() && jobs.empty())) {
      // Start new jobs
      while (!jobGrids.empty() && jobs.size() < threads) {
        SmartPointer<RenderJob> job =
          new RenderJob(*this, cutWorkpiece, mode, jobGrids.back());
        job->start();
        jobs.push_back(job);
        jobGrids.pop_back();
      }

      // Reap completed jobs
      jobs_t::iterator it;
      for (it = jobs.begin(); it != jobs.end() && !task.shouldQuit();)
        if ((*it)->getState() == Thread::THREAD_DONE) {
          (*it)->join();
          stats.add((*it)->getStats());
          it = jobs.erase(it);
        } else it++;


      // Update Progress
      double progress = 0;

      // Add running jobs
      for (it = jobs.begin(); it != jobs.end() && !task.shouldQuit(); it++)
        progress += (*it)->getProgress();

      // Add completed jobs
      progress += totalJobCount - jobGrids.size() - jobs.size();
      progress /= totalJobCount;

      task.update(progress);

      // Log progress
      double now = Timer::now();
      if (lastUpdate + 1 < now) {
        lastUpdate = now;
        LOG_INFO(2, String::printf("Progress: %0.2f%%", progress * 100)
                 << " Time: " << TimeInterval(task.getTime())
                 << " ETA: " << TimeInterval(task.getETA()));
      }

      // Wait
      timedWait(0.1);
    }
  } CATCH_ERROR;

  // Clean up remaining jobs in case of an early exit
  for (jobs_t::iterator it = jobs.begin(); it != jobs.end(); it++) {
    (*it)->join();
    stats.add((*it)->getStats());
  }
}
