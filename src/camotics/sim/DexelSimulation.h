/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#pragma once

#include <camotics/sim/Simulation.h>
#include <camotics/contour/Surface.h>

#include <cstdint>
#include <memory>
#include <vector>


namespace CAMotics {
class FieldFunction;
class Task;
namespace Dexel {
  class State;
  class GridSurface;

  enum class RejectionReason {
    NONE,
    INVALID_SIMULATION,
    INVALID_WORKPIECE,
    INVALID_RESOLUTION,
    INITIAL_SURFACE,
    UNSUPPORTED_RENDER_MODE,
    PARTIAL_TIME,
    UNSUPPORTED_MOVE_TYPE,
    INVALID_MOVE,
    ROTARY_OR_AUX_AXES,
    MISSING_TOOL,
    UNSUPPORTED_TOOL,
    INVALID_TOOL,
    RASTERIZER_NOT_IMPLEMENTED,
    MULTI_INTERVAL_UPDATE,
    EMPTY_COLUMN_UNSUPPORTED,
    TOPOLOGY_VALIDATION,
    GEOMETRY_VALIDATION,
    CANCELLED,
  };

  struct EligibilityReport {
    bool eligible = false;
    RejectionReason reason = RejectionReason::NONE;
    uint64_t movesChecked = 0;
    uint64_t toolsChecked = 0;
  };

  struct CandidateResult {
    bool accepted = false;
    RejectionReason reason = RejectionReason::NONE;
    cb::SmartPointer<Surface> surface;
    std::shared_ptr<State> state;
  };

  struct BoundaryTile {
    unsigned tile = 0;
    unsigned cellX = 0;
    unsigned cellY = 0;
    unsigned cellsX = 0;
    unsigned cellsY = 0;
    std::vector<unsigned char> cliffCells;
    std::vector<float> vertices;
    std::vector<float> normals;

    uint64_t getTriangleCount() const {return vertices.size() / 9;}
  };

  class State : public std::enable_shared_from_this<State> {
    struct Impl;
    std::shared_ptr<Impl> impl;

  public:
    explicit State(const std::shared_ptr<Impl> &impl);

    CandidateResult seek(double targetTime, Task *task = 0,
                         bool buildBoundary = false);
    double getTime() const;
    uint64_t getCurrentBytes() const;
    uint64_t getCheckpointBytes() const;
    uint64_t getCheckpointCount() const;
    bool hasBoundaryOwners() const;
    cb::SmartPointer<FieldFunction> getBoundaryField() const;
    std::vector<BoundaryTile> buildBoundaryTiles
      (const std::vector<unsigned> &dirtyTiles, bool full = false,
       Task *task = 0) const;
    void publishBoundaryTiles
      (GridSurface &surface, std::vector<BoundaryTile> &&tiles);

  private:
    cb::SmartPointer<FieldFunction> getBoundaryFieldFor
      (const std::shared_ptr<const std::vector<unsigned> > &owners,
       double currentTime) const;
    std::vector<BoundaryTile> buildBoundaryTilesFor
      (const std::vector<unsigned> &dirtyTiles, bool full,
       const std::shared_ptr<const std::vector<float> > &tops,
       const std::shared_ptr<const std::vector<unsigned> > &owners,
       double currentTime, Task *task) const;

    friend CandidateResult compute(const Simulation &, bool, Task *,
                                   const EligibilityReport *, bool, bool,
                                   bool);
  };

  class GridSurface : public CAMotics::Surface {
    Simulation sim;
    unsigned nx;
    unsigned ny;
    double xStep;
    double yStep;
    std::shared_ptr<const std::vector<float> > tops;
    std::vector<unsigned> dirtyTiles;
    std::vector<std::shared_ptr<const BoundaryTile> > boundaryTiles;
    bool fullUpload;
    mutable cb::SmartPointer<CAMotics::Surface> materialized;
    bool reduced = false;

    const cb::SmartPointer<CAMotics::Surface> &materialize
      (Task *task = 0) const;

  public:
    GridSurface(const Simulation &sim, unsigned nx, unsigned ny,
                double xStep, double yStep,
                const std::shared_ptr<const std::vector<float> > &tops,
                const std::vector<unsigned> &dirtyTiles = {},
                bool fullUpload = true);

    unsigned getNX() const {return nx;}
    unsigned getNY() const {return ny;}
    double getXStep() const {return xStep;}
    double getYStep() const {return yStep;}
    const std::vector<float> &getTops() const {return *tops;}
    const std::vector<unsigned> &getDirtyTiles() const {return dirtyTiles;}
    const std::vector<std::shared_ptr<const BoundaryTile> > &
      getBoundaryTiles() const
    {return boundaryTiles;}
    void setBoundaryTiles
      (const std::vector<std::shared_ptr<const BoundaryTile> > &tiles);
    void clearBoundaryTiles();
    bool needsFullUpload() const {return fullUpload;}
    bool isDirect() const {return !reduced;}

    cb::SmartPointer<CAMotics::Surface> copy() const override;
    uint64_t getTriangleCount() const override;
    cb::Rectangle3D getBounds() const override;
    void getVertices(vert_cb_t cb) const override;
    void write(STL::Sink &sink, Task *task = 0) const override;
    void reduce(Task &task) override;
    void read(const cb::JSON::Value &value) override;
    void write(cb::JSON::Sink &sink) const override;
    using CAMotics::Surface::read;
    using CAMotics::Surface::write;
  };

  const char *reasonName(RejectionReason reason);
  EligibilityReport classify(const Simulation &sim, Task *task = 0);
  CandidateResult compute(const Simulation &sim,
                          bool validateTopology = true, Task *task = 0,
                          const EligibilityReport *preclassified = 0,
                          bool retainState = false,
                          bool retainBoundaryOwners = false,
                          bool retainGrid = false);
  void recordEligibilityMetrics(const EligibilityReport &report);
  void recordFallback(RejectionReason reason);
}}
