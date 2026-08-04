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

#include "DexelSimulation.h"

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>


namespace GCode {
  class Move;
  class Tool;
}


namespace CAMotics {class TriangleSurface;}


namespace CAMotics { namespace Dexel { namespace Internal {
  struct RasterStats {
    uint64_t columns = 0;
    uint64_t moves = 0;
    uint64_t footprintCells = 0;
    uint64_t insideCells = 0;
    uint64_t changedColumns = 0;
    uint64_t emptiedColumns = 0;
    uint64_t profileEvaluations = 0;
    uint64_t multiIntervalViolations = 0;
    uint64_t tiles = 0;
    uint64_t tileMoveRefs = 0;
  };


  struct CutterProfile {
    double length = 0;
    double bottomRadius = 0;
    double topRadius = 0;

    double radiusAt(double height) const {
      if (topRadius == bottomRadius) return bottomRadius;
      return bottomRadius +
        (topRadius - bottomRadius) * (height / length);
    }

    double surfaceHeight(double radius) const {
      if (radius <= bottomRadius) return 0;
      if (topRadius <= bottomRadius) return 0;
      return (radius - bottomRadius) /
        (topRadius - bottomRadius) * length;
    }
  };


  struct PreparedMove {
    const GCode::Move *move = 0;
    CutterProfile profile;
    double radius = 0;
    int x0 = 0;
    int x1 = 0;
    int y0 = 0;
    int y1 = 0;
  };


  struct StateCheckpoint {
    unsigned preparedEnd = 0;
    double time = 0;
    std::vector<float> tops;
    std::vector<unsigned> boundaryOwners;
  };


  struct MeshBuilder {
    std::vector<float> vertices;
    std::vector<float> normals;

    void triangle(const cb::Vector3F &a, const cb::Vector3F &b,
                  const cb::Vector3F &c);
    void quad(const cb::Vector3F &a, const cb::Vector3F &b,
              const cb::Vector3F &c, const cb::Vector3F &d);
  };


  CutterProfile getProfile(const GCode::Tool &tool);
  bool footprintInterval(const cb::Vector3D &a, const cb::Vector3D &b,
                         double x, double y, double radius,
                         double &lo, double &hi);
  double minimumLower(const CutterProfile &profile, const cb::Vector3D &a,
                      const cb::Vector3D &b, double x, double y,
                      double lo, double hi, RasterStats &stats);
  double stockRadius(const CutterProfile &profile, const cb::Vector3D &a,
                     const cb::Vector3D &b, double bottom, double top);
  void recordRasterMetrics(const RasterStats &stats);
  bool gridDimension(double length, double resolution, unsigned &cells);
  int clampGridIndex(double value, unsigned maximum);

  void addXWall(MeshBuilder &mesh, float x, float y0, float y1,
                float low, float high, bool positive);
  void addYWall(MeshBuilder &mesh, float x0, float x1, float y,
                float low, float high, bool positive);
  cb::SmartPointer<TriangleSurface> materializeStateSurface
    (const Simulation &sim, unsigned nx, unsigned ny, double xStep,
     double yStep, const std::vector<float> &tops, Task *task,
     RejectionReason &reason);
  cb::SmartPointer<TriangleSurface> buildSurface
    (const Simulation &sim, unsigned nx, unsigned ny, double xStep,
     double yStep, const std::vector<float> &tops, Task *task,
     RejectionReason &reason);
}}}


struct CAMotics::Dexel::State::Impl {
  Simulation sim;
  unsigned nx = 0;
  unsigned ny = 0;
  unsigned tileSize = 64;
  unsigned tileCols = 0;
  unsigned tileRows = 0;
  double xStep = 0;
  double yStep = 0;
  double epsilon = 0;
  double currentTime = 0;
  std::shared_ptr<std::vector<float> > tops;
  std::vector<Internal::PreparedMove> prepared;
  std::vector<std::vector<unsigned> > tileMoves;
  std::vector<Internal::StateCheckpoint> checkpoints;
  std::shared_ptr<std::vector<unsigned> > boundaryOwners;
  bool hasPartialBase = false;
  double partialStartTime = 0;
  std::vector<std::pair<uint64_t, float> > partialUndo;
  std::vector<std::pair<uint64_t, unsigned> > partialOwnerUndo;
  std::vector<std::shared_ptr<const BoundaryTile> > boundaryTiles;
  bool boundaryTilesStale = false;

  Impl(const Simulation &sim) : sim(sim) {}
};
