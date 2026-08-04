/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#include "DexelSimulationInternal.h"

#include <camotics/Profile.h>
#include <camotics/Task.h>
#include <camotics/contour/TriangleSurface.h>

#include <algorithm>
#include <utility>


using namespace std;
using namespace cb;
using namespace CAMotics;
using namespace CAMotics::Dexel;
using namespace CAMotics::Dexel::Internal;


void MeshBuilder::triangle(const Vector3F &a, const Vector3F &b,
                           const Vector3F &c) {
  Vector3F normal = (b - a).cross(c - a);
  double length = normal.length();
  if (!length) return;
  normal /= length;
  const Vector3F points[3] = {a, b, c};
  for (const Vector3F &point: points)
    for (unsigned axis = 0; axis < 3; axis++) {
      vertices.push_back(point[axis]);
      normals.push_back(normal[axis]);
    }
}


void MeshBuilder::quad(const Vector3F &a, const Vector3F &b,
                       const Vector3F &c, const Vector3F &d) {
  triangle(a, b, c);
  triangle(a, c, d);
}


namespace CAMotics { namespace Dexel { namespace Internal {
  void addXWall(MeshBuilder &mesh, float x, float y0, float y1,
                float low, float high, bool positive) {
    if (positive)
      mesh.quad(Vector3F(x, y0, low), Vector3F(x, y1, low),
                Vector3F(x, y1, high), Vector3F(x, y0, high));
    else
      mesh.quad(Vector3F(x, y1, low), Vector3F(x, y0, low),
                Vector3F(x, y0, high), Vector3F(x, y1, high));
  }


  void addYWall(MeshBuilder &mesh, float x0, float x1, float y,
                float low, float high, bool positive) {
    if (positive)
      mesh.quad(Vector3F(x1, y, low), Vector3F(x0, y, low),
                Vector3F(x0, y, high), Vector3F(x1, y, high));
    else
      mesh.quad(Vector3F(x0, y, low), Vector3F(x1, y, low),
                Vector3F(x1, y, high), Vector3F(x0, y, high));
  }


  SmartPointer<TriangleSurface> materializeStateSurface
    (const Simulation &sim, unsigned nx, unsigned ny, double xStep,
     double yStep, const vector<float> &tops, Task *task,
     RejectionReason &reason) {
    Vector3D stockMin = sim.workpiece.getMin();
    Vector3D stockMax = sim.workpiece.getMax();
    vector<float> xEdges(nx + 1);
    vector<float> yEdges(ny + 1);
    for (unsigned x = 0; x <= nx; x++)
      xEdges[x] = (float)(x == nx ? stockMax.x() :
                          stockMin.x() + x * xStep);
    for (unsigned y = 0; y <= ny; y++)
      yEdges[y] = (float)(y == ny ? stockMax.y() :
                          stockMin.y() + y * yStep);

    float bottom = (float)stockMin.z();
    MeshBuilder mesh;
    for (unsigned y = 0; y < ny; y++) {
      if (task && !(y & 31) && !task->update((double)y / ny)) {
        reason = RejectionReason::CANCELLED;
        return 0;
      }
      for (unsigned x = 0; x < nx; x++) {
        uint64_t i00 = (uint64_t)y * (nx + 1) + x;
        uint64_t i10 = i00 + 1;
        uint64_t i01 = i00 + nx + 1;
        uint64_t i11 = i01 + 1;
        float x0 = xEdges[x];
        float x1 = xEdges[x + 1];
        float y0 = yEdges[y];
        float y1 = yEdges[y + 1];
        mesh.quad(Vector3F(x0, y0, tops[i00]),
                  Vector3F(x1, y0, tops[i10]),
                  Vector3F(x1, y1, tops[i11]),
                  Vector3F(x0, y1, tops[i01]));
      }
    }

    vector<Vector3F> bottomBoundary;
    bottomBoundary.reserve(2 * ((uint64_t)nx + ny));
    for (unsigned x = 0; x < nx; x++)
      bottomBoundary.emplace_back(xEdges[x], yEdges[0], bottom);
    for (unsigned y = 0; y < ny; y++)
      bottomBoundary.emplace_back(xEdges[nx], yEdges[y], bottom);
    for (unsigned x = nx; 0 < x; x--)
      bottomBoundary.emplace_back(xEdges[x], yEdges[ny], bottom);
    for (unsigned y = ny; 0 < y; y--)
      bottomBoundary.emplace_back(xEdges[0], yEdges[y], bottom);

    Vector3F bottomCenter
      ((xEdges[0] + xEdges[nx]) / 2,
       (yEdges[0] + yEdges[ny]) / 2, bottom);
    for (unsigned i = 0; i < bottomBoundary.size(); i++)
      mesh.triangle(bottomCenter,
                    bottomBoundary[(i + 1) % bottomBoundary.size()],
                    bottomBoundary[i]);

    for (unsigned y = 0; y < ny; y++) {
      if (task && !(y & 63) && task->shouldQuit()) {
        reason = RejectionReason::CANCELLED;
        return 0;
      }
      uint64_t left0 = (uint64_t)y * (nx + 1);
      uint64_t left1 = left0 + nx + 1;
      mesh.quad(Vector3F(xEdges[0], yEdges[y + 1], bottom),
                Vector3F(xEdges[0], yEdges[y], bottom),
                Vector3F(xEdges[0], yEdges[y], tops[left0]),
                Vector3F(xEdges[0], yEdges[y + 1], tops[left1]));

      uint64_t right0 = left0 + nx;
      uint64_t right1 = left1 + nx;
      mesh.quad(Vector3F(xEdges[nx], yEdges[y], bottom),
                Vector3F(xEdges[nx], yEdges[y + 1], bottom),
                Vector3F(xEdges[nx], yEdges[y + 1], tops[right1]),
                Vector3F(xEdges[nx], yEdges[y], tops[right0]));
    }

    for (unsigned x = 0; x < nx; x++) {
      if (task && !(x & 63) && task->shouldQuit()) {
        reason = RejectionReason::CANCELLED;
        return 0;
      }
      uint64_t lower0 = x;
      uint64_t lower1 = x + 1;
      mesh.quad(Vector3F(xEdges[x], yEdges[0], bottom),
                Vector3F(xEdges[x + 1], yEdges[0], bottom),
                Vector3F(xEdges[x + 1], yEdges[0], tops[lower1]),
                Vector3F(xEdges[x], yEdges[0], tops[lower0]));

      uint64_t upper0 = (uint64_t)ny * (nx + 1) + x;
      uint64_t upper1 = upper0 + 1;
      mesh.quad(Vector3F(xEdges[x + 1], yEdges[ny], bottom),
                Vector3F(xEdges[x], yEdges[ny], bottom),
                Vector3F(xEdges[x], yEdges[ny], tops[upper0]),
                Vector3F(xEdges[x + 1], yEdges[ny], tops[upper1]));
    }

    SmartPointer<TriangleSurface> surface = new TriangleSurface;
    surface->replace(move(mesh.vertices), move(mesh.normals));
    return surface;
  }

}}}


Dexel::GridSurface::GridSurface
  (const Simulation &sim, unsigned nx, unsigned ny,
   double xStep, double yStep,
   const shared_ptr<const vector<float> > &tops,
   const vector<unsigned> &dirtyTiles, bool fullUpload) :
  sim(sim), nx(nx), ny(ny), xStep(xStep), yStep(yStep), tops(tops),
  dirtyTiles(dirtyTiles), fullUpload(fullUpload) {}


void Dexel::GridSurface::setBoundaryTiles
(const vector<shared_ptr<const BoundaryTile> > &tiles) {
  boundaryTiles = tiles;
}


void Dexel::GridSurface::clearBoundaryTiles() {
  vector<shared_ptr<const BoundaryTile> >().swap(boundaryTiles);
}


const SmartPointer<Surface> &Dexel::GridSurface::materialize(Task *task) const {
  if (materialized.isNull()) {
    RejectionReason reason = RejectionReason::NONE;
    materialized = materializeStateSurface
      (sim, nx, ny, xStep, yStep, *tops, task, reason);
    if (reason == RejectionReason::CANCELLED || materialized.isNull())
      THROW("Dexel surface materialization cancelled");
  }
  return materialized;
}


SmartPointer<Surface> Dexel::GridSurface::copy() const {
  if (reduced) return materialize()->copy();
  return new GridSurface
    (sim, nx, ny, xStep, yStep, tops, dirtyTiles, fullUpload);
}


uint64_t Dexel::GridSurface::getTriangleCount() const {
  if (reduced) return materialize()->getTriangleCount();
  return (uint64_t)2 * nx * ny + (uint64_t)6 * (nx + ny);
}


Rectangle3D Dexel::GridSurface::getBounds() const {return sim.workpiece;}


void Dexel::GridSurface::getVertices(vert_cb_t cb) const {
  materialize()->getVertices(cb);
}


void Dexel::GridSurface::write(STL::Sink &sink, Task *task) const {
  materialize(task)->write(sink, task);
}


void Dexel::GridSurface::reduce(Task &task) {
  materialize(&task)->reduce(task);
  reduced = true;
}


void Dexel::GridSurface::read(const cb::JSON::Value &value) {
  materialize()->read(value);
  reduced = true;
}


void Dexel::GridSurface::write(cb::JSON::Sink &sink) const {
  materialize()->write(sink);
}


SmartPointer<TriangleSurface> Internal::buildSurface
(const Simulation &sim, unsigned nx, unsigned ny, double xStep,
 double yStep, const vector<float> &tops, Task *task,
 RejectionReason &reason) {
  if (task) task->begin("Building dexel surface");
  unique_ptr<Profile::Scope> surfaceScope
    (new Profile::Scope("dexel_surface_build"));
  Vector3D stockMin = sim.workpiece.getMin();
  Vector3D stockMax = sim.workpiece.getMax();

  vector<float> xEdges(nx + 1);
  vector<float> yEdges(ny + 1);
  for (unsigned x = 0; x <= nx; x++)
    xEdges[x] = (float)(x == nx ? stockMax.x() :
                        stockMin.x() + x * xStep);
  for (unsigned y = 0; y <= ny; y++)
    yEdges[y] = (float)(y == ny ? stockMax.y() :
                        stockMin.y() + y * yStep);

  float bottom = (float)stockMin.z();
  MeshBuilder mesh;
  uint64_t topTriangles = 0;
  uint64_t bottomTriangles = 0;
  uint64_t sideTriangles = 0;

  for (unsigned y = 0; y < ny; y++) {
    if (task && !(y & 31) && !task->update((double)y / ny)) {
      reason = RejectionReason::CANCELLED;
      return 0;
    }
    for (unsigned x = 0; x < nx; x++) {
      uint64_t i00 = (uint64_t)y * (nx + 1) + x;
      uint64_t i10 = i00 + 1;
      uint64_t i01 = i00 + nx + 1;
      uint64_t i11 = i01 + 1;
      float x0 = xEdges[x];
      float x1 = xEdges[x + 1];
      float y0 = yEdges[y];
      float y1 = yEdges[y + 1];

      mesh.quad(Vector3F(x0, y0, tops[i00]),
                Vector3F(x1, y0, tops[i10]),
                Vector3F(x1, y1, tops[i11]),
                Vector3F(x0, y1, tops[i01]));
      topTriangles += 2;
    }
  }

  // One triangle fan preserves every segmented side-wall bottom edge without
  // retaining two coplanar bottom triangles per XY cell.
  vector<Vector3F> bottomBoundary;
  bottomBoundary.reserve(2 * ((uint64_t)nx + ny));
  for (unsigned x = 0; x < nx; x++)
    bottomBoundary.emplace_back(xEdges[x], yEdges[0], bottom);
  for (unsigned y = 0; y < ny; y++)
    bottomBoundary.emplace_back(xEdges[nx], yEdges[y], bottom);
  for (unsigned x = nx; 0 < x; x--)
    bottomBoundary.emplace_back(xEdges[x], yEdges[ny], bottom);
  for (unsigned y = ny; 0 < y; y--)
    bottomBoundary.emplace_back(xEdges[0], yEdges[y], bottom);

  Vector3F bottomCenter
    ((xEdges[0] + xEdges[nx]) / 2,
     (yEdges[0] + yEdges[ny]) / 2, bottom);
  for (unsigned i = 0; i < bottomBoundary.size(); i++)
    mesh.triangle(bottomCenter,
                  bottomBoundary[(i + 1) % bottomBoundary.size()],
                  bottomBoundary[i]);
  bottomTriangles = bottomBoundary.size();

  for (unsigned y = 0; y < ny; y++) {
    if (task && !(y & 63) && task->shouldQuit()) {
      reason = RejectionReason::CANCELLED;
      return 0;
    }
    uint64_t left0 = (uint64_t)y * (nx + 1);
    uint64_t left1 = left0 + nx + 1;
    mesh.quad(Vector3F(xEdges[0], yEdges[y + 1], bottom),
              Vector3F(xEdges[0], yEdges[y], bottom),
              Vector3F(xEdges[0], yEdges[y], tops[left0]),
              Vector3F(xEdges[0], yEdges[y + 1], tops[left1]));

    uint64_t right0 = left0 + nx;
    uint64_t right1 = left1 + nx;
    mesh.quad(Vector3F(xEdges[nx], yEdges[y], bottom),
              Vector3F(xEdges[nx], yEdges[y + 1], bottom),
              Vector3F(xEdges[nx], yEdges[y + 1], tops[right1]),
              Vector3F(xEdges[nx], yEdges[y], tops[right0]));
    sideTriangles += 4;
  }

  for (unsigned x = 0; x < nx; x++) {
    if (task && !(x & 63) && task->shouldQuit()) {
      reason = RejectionReason::CANCELLED;
      return 0;
    }
    uint64_t lower0 = x;
    uint64_t lower1 = x + 1;
    mesh.quad(Vector3F(xEdges[x], yEdges[0], bottom),
              Vector3F(xEdges[x + 1], yEdges[0], bottom),
              Vector3F(xEdges[x + 1], yEdges[0], tops[lower1]),
              Vector3F(xEdges[x], yEdges[0], tops[lower0]));

    uint64_t upper0 = (uint64_t)ny * (nx + 1) + x;
    uint64_t upper1 = upper0 + 1;
    mesh.quad(Vector3F(xEdges[x + 1], yEdges[ny], bottom),
              Vector3F(xEdges[x], yEdges[ny], bottom),
              Vector3F(xEdges[x], yEdges[ny], tops[upper0]),
              Vector3F(xEdges[x + 1], yEdges[ny], tops[upper1]));
    sideTriangles += 4;
  }

  SmartPointer<TriangleSurface> surface = new TriangleSurface;
  surface->replace(move(mesh.vertices), move(mesh.normals));
  Profile::setMetric("dexel_mesh_top_triangles", topTriangles);
  Profile::setMetric("dexel_mesh_bottom_triangles", bottomTriangles);
  Profile::setMetric("dexel_mesh_transition_triangles", 0);
  Profile::setMetric("dexel_mesh_side_triangles", sideTriangles);
  Profile::setMetric("dexel_mesh_triangles", surface->getTriangleCount());

  return surface;
}
