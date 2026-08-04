/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#include "DexelMesh.h"

#include <camotics/Profile.h>
#include <camotics/sim/DexelSimulation.h>

#include <cbang/log/Logger.h>
#include <cbang/time/Timer.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

using namespace std;
using namespace cb;
using namespace CAMotics;


DexelMesh::~DexelMesh() {
  try {
    if (GLContext::isActive()) {
      GLContext gl;
      if (indices) gl.glDeleteBuffers(1, &indices);
      if (mediumPlaybackIndices)
        gl.glDeleteBuffers(1, &mediumPlaybackIndices);
      if (fastPlaybackIndices) gl.glDeleteBuffers(1, &fastPlaybackIndices);
    }
  } catch (...) {}
}


void DexelMesh::clear() {
  vertices.allocate(0);
  normals.allocate(0);
  indexCount = 0;
  mediumPlaybackIndexCount = 0;
  fastPlaybackIndexCount = 0;
  gridX = 0;
  gridY = 0;
  displayedTops.clear();
  boundaryMeshes.clear();
  displayedBoundaryTiles.clear();
}


void DexelMesh::updateGrid(const Dexel::GridSurface &surface) {
  double start = Timer::now();
  const unsigned nx = surface.getNX();
  const unsigned ny = surface.getNY();
  const double xStep = surface.getXStep();
  const double yStep = surface.getYStep();
  const Vector3D stockMin = surface.getBounds().getMin();
  const Vector3D stockMax = surface.getBounds().getMax();
  const vector<float> &tops = surface.getTops();
  uint64_t uploaded = 0;

  auto xAt = [&] (unsigned x) {
    return (float)(x == nx ? stockMax.x() : stockMin.x() + x * xStep);
  };
  auto yAt = [&] (unsigned y) {
    return (float)(y == ny ? stockMax.y() : stockMin.y() + y * yStep);
  };

  const unsigned tileSize = 64;
  const unsigned tileCols = (nx + tileSize) / tileSize;
  for (unsigned tile: surface.getDirtyTiles()) {
    unsigned tileY = tile / tileCols;
    unsigned tileX = tile % tileCols;
    unsigned x0 = tileX * tileSize;
    unsigned x1 = min(nx, x0 + tileSize - 1);
    unsigned y0 = tileY * tileSize;
    unsigned y1 = min(ny, y0 + tileSize - 1);
    if (x0) x0--;
    if (y0) y0--;
    if (x1 < nx) x1++;
    if (y1 < ny) y1++;

    vector<float> rowVertices;
    vector<float> rowNormals;
    rowVertices.reserve((size_t)(x1 - x0 + 1) * 3);
    rowNormals.reserve(rowVertices.capacity());
    for (unsigned y = y0; y <= y1; y++) {
      rowVertices.clear();
      rowNormals.clear();
      for (unsigned x = x0; x <= x1; x++) {
        unsigned xa = x ? x - 1 : x;
        unsigned xb = x < nx ? x + 1 : x;
        unsigned ya = y ? y - 1 : y;
        unsigned yb = y < ny ? y + 1 : y;
        double dx = (tops[(uint64_t)y * (nx + 1) + xb] -
                     tops[(uint64_t)y * (nx + 1) + xa]) /
          max(1e-30, (double)xAt(xb) - xAt(xa));
        double dy = (tops[(uint64_t)yb * (nx + 1) + x] -
                     tops[(uint64_t)ya * (nx + 1) + x]) /
          max(1e-30, (double)yAt(yb) - yAt(ya));
        double length = sqrt(dx * dx + dy * dy + 1);
        rowVertices.push_back(xAt(x));
        rowVertices.push_back(yAt(y));
        rowVertices.push_back(tops[(uint64_t)y * (nx + 1) + x]);
        rowNormals.push_back(-dx / length);
        rowNormals.push_back(-dy / length);
        rowNormals.push_back(1 / length);
      }
      unsigned offset =
        ((uint64_t)y * (nx + 1) + x0) * 3 * sizeof(float);
      vertices.update(offset, rowVertices.size(), rowVertices.data());
      normals.update(offset, rowNormals.size(), rowNormals.data());
      uploaded += (rowVertices.size() + rowNormals.size()) * sizeof(float);
    }
  }

  displayedTops = tops;
  Profile::setMetric("dexel_display_uploaded_bytes", uploaded);
  Profile::setMetric("dexel_display_updated_tiles",
                     surface.getDirtyTiles().size());
  Profile::setMetric("dexel_display_build_us",
                     (Timer::now() - start) * 1000000);
  LOG_INFO(1, "Dexel direct display update: dirty_tiles="
           << surface.getDirtyTiles().size()
           << " uploaded_bytes=" << uploaded
           << " build_seconds=" << Timer::now() - start);
}


void DexelMesh::updateBoundaryTiles(Dexel::GridSurface &surface) {
  const vector<shared_ptr<const Dexel::BoundaryTile> > &patches =
    surface.getBoundaryTiles();
  if (patches.empty()) return;
  double start = Timer::now();
  const unsigned nx = surface.getNX();
  const unsigned ny = surface.getNY();
  const unsigned tileSize = 64;
  const unsigned tileCols = (nx + tileSize) / tileSize;
  const unsigned tileRows = (ny + tileSize) / tileSize;
  if (boundaryMeshes.size() != (uint64_t)tileCols * tileRows)
    boundaryMeshes.resize((uint64_t)tileCols * tileRows);
  if (displayedBoundaryTiles.size() != boundaryMeshes.size())
    displayedBoundaryTiles.resize(boundaryMeshes.size());

  unsigned patchCount = 0;
  uint64_t uploaded = 0;
  uint64_t triangles = 0;
  uint64_t underlayCells = 0;
  for (unsigned tile = 0; tile < patches.size(); tile++) {
    const shared_ptr<const Dexel::BoundaryTile> &patchPtr = patches[tile];
    if (!patchPtr || boundaryMeshes.size() <= tile ||
        displayedBoundaryTiles[tile].get() == patchPtr.get())
      continue;
    const Dexel::BoundaryTile &patch = *patchPtr;
    if (boundaryMeshes.size() <= patch.tile ||
        patch.tile != tile ||
        nx < patch.cellX + patch.cellsX ||
        ny < patch.cellY + patch.cellsY ||
        patch.cliffCells.size() !=
        (uint64_t)patch.cellsX * patch.cellsY)
      continue;
    patchCount++;

    // Keep the complete, watertight direct grid underneath the exact patches.
    // Boundary patches are independent producer-local MC meshes and are not
    // welded to the grid.  Degenerating an entire cliff cell therefore opens a
    // visible hole whenever the patch does not cover that cell to its edges.
    // The depth-biased overlay below supplies the representative contour while
    // this underlay guarantees that partial playback never exposes the
    // background through a display-only seam.
    for (unsigned char cliff: patch.cliffCells)
      underlayCells += cliff != 0;

    unsigned patchTriangles = patch.getTriangleCount();
    triangles += patchTriangles;
    SmartPointer<Mesh> &mesh = boundaryMeshes[patch.tile];
    if (!mesh.isNull() || patchTriangles) {
      if (mesh.isNull()) mesh = new Mesh(patchTriangles);
      else mesh->reset(patchTriangles);
      if (patchTriangles)
        mesh->add(patch.vertices, patch.normals);
    }
    uploaded += (patch.vertices.size() + patch.normals.size()) *
      sizeof(float);
    displayedBoundaryTiles[tile] = patchPtr;
  }
  surface.clearBoundaryTiles();

  Profile::setMetric("dexel_display_boundary_tiles", patchCount);
  Profile::setMetric("dexel_display_boundary_triangles", triangles);
  Profile::setMetric("dexel_display_boundary_uploaded_bytes", uploaded);
  Profile::setMetric("dexel_display_boundary_underlay_cells", underlayCells);
  Profile::setMetric("dexel_display_boundary_upload_us",
                     (Timer::now() - start) * 1000000);
  LOG_INFO(1, "Dexel boundary display update: tiles=" << patchCount
           << " triangles=" << triangles
           << " underlay_cells=" << underlayCells
           << " uploaded_bytes=" << uploaded
           << " seconds=" << Timer::now() - start);
}


void DexelMesh::load(Dexel::GridSurface &surface) {
  double start = Timer::now();
  const unsigned nx = surface.getNX();
  const unsigned ny = surface.getNY();
  const double xStep = surface.getXStep();
  const double yStep = surface.getYStep();
  const Rectangle3D bounds = surface.getBounds();
  const Vector3D stockMin = bounds.getMin();
  const Vector3D stockMax = bounds.getMax();
  const vector<float> &tops = surface.getTops();
  bool boundaryChanged = displayedTops.size() != tops.size();
  if (!boundaryChanged && !tops.empty()) {
    for (unsigned x = 0; x <= nx && !boundaryChanged; x++)
      boundaryChanged =
        displayedTops[x] != tops[x] ||
        displayedTops[(uint64_t)ny * (nx + 1) + x] !=
          tops[(uint64_t)ny * (nx + 1) + x];
    for (unsigned y = 0; y <= ny && !boundaryChanged; y++)
      boundaryChanged =
        displayedTops[(uint64_t)y * (nx + 1)] !=
          tops[(uint64_t)y * (nx + 1)] ||
        displayedTops[(uint64_t)y * (nx + 1) + nx] !=
          tops[(uint64_t)y * (nx + 1) + nx];
  }
  if (!surface.needsFullUpload() && indexCount &&
      gridX == nx && gridY == ny && !boundaryChanged) {
    updateGrid(surface);
    updateBoundaryTiles(surface);
    return;
  }
  boundaryMeshes.clear();
  displayedBoundaryTiles.clear();
  vector<float> v;
  vector<float> n;
  vector<uint32_t> elements;
  vector<uint32_t> mediumPlaybackElements;
  vector<uint32_t> fastPlaybackElements;
  const uint64_t columns = (uint64_t)(nx + 1) * (ny + 1);
  v.reserve(columns * 3 + (uint64_t)24 * (nx + ny) + 3);
  n.reserve(v.capacity());
  elements.reserve((uint64_t)6 * nx * ny +
                   (uint64_t)18 * (nx + ny));

  auto xAt = [&] (unsigned x) {
    return (float)(x == nx ? stockMax.x() : stockMin.x() + x * xStep);
  };
  auto yAt = [&] (unsigned y) {
    return (float)(y == ny ? stockMax.y() : stockMin.y() + y * yStep);
  };
  auto addVertex = [&] (float x, float y, float z,
                        float a, float b, float c) {
    uint32_t index = v.size() / 3;
    v.push_back(x); v.push_back(y); v.push_back(z);
    n.push_back(a); n.push_back(b); n.push_back(c);
    return index;
  };
  auto triangle = [&] (uint32_t a, uint32_t b, uint32_t c) {
    elements.push_back(a); elements.push_back(b); elements.push_back(c);
  };
  auto quad = [&] (uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
    triangle(a, b, c); triangle(a, c, d);
  };

  for (unsigned y = 0; y <= ny; y++)
    for (unsigned x = 0; x <= nx; x++) {
      unsigned x0 = x ? x - 1 : x;
      unsigned x1 = x < nx ? x + 1 : x;
      unsigned y0 = y ? y - 1 : y;
      unsigned y1 = y < ny ? y + 1 : y;
      double dx = (tops[(uint64_t)y * (nx + 1) + x1] -
                   tops[(uint64_t)y * (nx + 1) + x0]) /
        max(1e-30, (double)xAt(x1) - xAt(x0));
      double dy = (tops[(uint64_t)y1 * (nx + 1) + x] -
                   tops[(uint64_t)y0 * (nx + 1) + x]) /
        max(1e-30, (double)yAt(y1) - yAt(y0));
      double length = sqrt(dx * dx + dy * dy + 1);
      uint64_t index = (uint64_t)y * (nx + 1) + x;
      addVertex(xAt(x), yAt(y), tops[index],
                -dx / length, -dy / length, 1 / length);
    }

  for (unsigned y = 0; y < ny; y++)
    for (unsigned x = 0; x < nx; x++) {
      uint32_t i00 = (uint64_t)y * (nx + 1) + x;
      uint32_t i10 = i00 + 1;
      uint32_t i01 = i00 + nx + 1;
      uint32_t i11 = i01 + 1;
      quad(i00, i10, i11, i01);
    }
  const uint64_t topIndexCount = elements.size();

  const float bottom = stockMin.z();
  auto sideQuad = [&] (float x0, float y0, float z0,
                       float x1, float y1, float z1,
                       float x2, float y2, float z2,
                       float x3, float y3, float z3,
                       float a, float b, float c) {
    uint32_t i0 = addVertex(x0, y0, z0, a, b, c);
    uint32_t i1 = addVertex(x1, y1, z1, a, b, c);
    uint32_t i2 = addVertex(x2, y2, z2, a, b, c);
    uint32_t i3 = addVertex(x3, y3, z3, a, b, c);
    quad(i0, i1, i2, i3);
  };

  for (unsigned y = 0; y < ny; y++) {
    uint64_t left0 = (uint64_t)y * (nx + 1);
    uint64_t left1 = left0 + nx + 1;
    sideQuad(xAt(0), yAt(y + 1), bottom,
             xAt(0), yAt(y), bottom,
             xAt(0), yAt(y), tops[left0],
             xAt(0), yAt(y + 1), tops[left1], -1, 0, 0);
    uint64_t right0 = left0 + nx;
    uint64_t right1 = left1 + nx;
    sideQuad(xAt(nx), yAt(y), bottom,
             xAt(nx), yAt(y + 1), bottom,
             xAt(nx), yAt(y + 1), tops[right1],
             xAt(nx), yAt(y), tops[right0], 1, 0, 0);
  }
  for (unsigned x = 0; x < nx; x++) {
    uint64_t lower0 = x;
    uint64_t lower1 = x + 1;
    sideQuad(xAt(x), yAt(0), bottom,
             xAt(x + 1), yAt(0), bottom,
             xAt(x + 1), yAt(0), tops[lower1],
             xAt(x), yAt(0), tops[lower0], 0, -1, 0);
    uint64_t upper0 = (uint64_t)ny * (nx + 1) + x;
    uint64_t upper1 = upper0 + 1;
    sideQuad(xAt(x + 1), yAt(ny), bottom,
             xAt(x), yAt(ny), bottom,
             xAt(x), yAt(ny), tops[upper0],
             xAt(x + 1), yAt(ny), tops[upper1], 0, 1, 0);
  }

  vector<uint32_t> boundary;
  boundary.reserve(2 * ((uint64_t)nx + ny));
  for (unsigned x = 0; x < nx; x++)
    boundary.push_back(addVertex(xAt(x), yAt(0), bottom, 0, 0, -1));
  for (unsigned y = 0; y < ny; y++)
    boundary.push_back(addVertex(xAt(nx), yAt(y), bottom, 0, 0, -1));
  for (unsigned x = nx; 0 < x; x--)
    boundary.push_back(addVertex(xAt(x), yAt(ny), bottom, 0, 0, -1));
  for (unsigned y = ny; 0 < y; y--)
    boundary.push_back(addVertex(xAt(0), yAt(y), bottom, 0, 0, -1));
  uint32_t center = addVertex((xAt(0) + xAt(nx)) / 2,
                              (yAt(0) + yAt(ny)) / 2,
                              bottom, 0, 0, -1);
  for (unsigned i = 0; i < boundary.size(); i++)
    triangle(center, boundary[(i + 1) % boundary.size()], boundary[i]);

  // Playback changes only the top-surface index buffer.  Every level references
  // the same exact height vertices and complete stock shell.  Normal-speed Play
  // uses every cell; medium and fast playback reduce raster work by about 4x
  // and 16x respectively.
  auto buildPlaybackElements =
    [&] (vector<uint32_t> &playbackElements, unsigned stride) {
      playbackElements.reserve
        (((uint64_t)nx + stride - 1) / stride *
         ((uint64_t)ny + stride - 1) / stride * 6 +
         elements.size() - topIndexCount);
      for (unsigned y = 0; y < ny; y += stride)
        for (unsigned x = 0; x < nx; x += stride) {
          unsigned x1 = min(nx, x + stride);
          unsigned y1 = min(ny, y + stride);
          uint32_t i00 = (uint64_t)y * (nx + 1) + x;
          uint32_t i10 = (uint64_t)y * (nx + 1) + x1;
          uint32_t i01 = (uint64_t)y1 * (nx + 1) + x;
          uint32_t i11 = (uint64_t)y1 * (nx + 1) + x1;
          playbackElements.push_back(i00);
          playbackElements.push_back(i10);
          playbackElements.push_back(i11);
          playbackElements.push_back(i00);
          playbackElements.push_back(i11);
          playbackElements.push_back(i01);
        }
      playbackElements.insert(playbackElements.end(),
                              elements.begin() + topIndexCount, elements.end());
    };
  buildPlaybackElements(mediumPlaybackElements, 2);
  buildPlaybackElements(fastPlaybackElements, 4);

  vertices.allocate(v.size() * sizeof(float));
  normals.allocate(n.size() * sizeof(float));
  if (!v.empty()) vertices.add(v.size(), v.data());
  if (!n.empty()) normals.add(n.size(), n.data());

  GLContext gl;
  if (!indices) gl.glGenBuffers(1, &indices);
  gl.glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices);
  gl.glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                  elements.size() * sizeof(uint32_t), elements.data(),
                  GL_STATIC_DRAW);
  gl.glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  indexCount = elements.size();
  auto uploadElements =
    [&] (unsigned &buffer, const vector<uint32_t> &data) {
      if (!buffer) gl.glGenBuffers(1, &buffer);
      gl.glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer);
      gl.glBufferData(GL_ELEMENT_ARRAY_BUFFER, data.size() * sizeof(uint32_t),
                      data.data(), GL_STATIC_DRAW);
    };
  uploadElements(mediumPlaybackIndices, mediumPlaybackElements);
  uploadElements(fastPlaybackIndices, fastPlaybackElements);
  gl.glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  mediumPlaybackIndexCount = mediumPlaybackElements.size();
  fastPlaybackIndexCount = fastPlaybackElements.size();
  gridX = nx;
  gridY = ny;
  displayedTops = tops;
  updateBoundaryTiles(surface);

  uint64_t uploaded =
    (v.size() + n.size()) * sizeof(float) +
    (elements.size() + mediumPlaybackElements.size() +
     fastPlaybackElements.size()) * sizeof(uint32_t);
  Profile::setMetric("dexel_display_uploaded_bytes", uploaded);
  Profile::setMetric("dexel_display_vertices", v.size() / 3);
  Profile::setMetric("dexel_display_indices", indexCount);
  Profile::setMetric("dexel_display_medium_playback_indices",
                     mediumPlaybackIndexCount);
  Profile::setMetric("dexel_display_playback_indices",
                     fastPlaybackIndexCount);
  Profile::setMetric("dexel_display_updated_tiles",
                     surface.getDirtyTiles().size());
  Profile::setMetric("dexel_display_build_us",
                     (Timer::now() - start) * 1000000);
  LOG_INFO(1, "Dexel direct display: vertices=" << v.size() / 3
           << " indices=" << indexCount << " uploaded_bytes=" << uploaded
           << " medium_playback_indices=" << mediumPlaybackIndexCount
           << " playback_indices=" << fastPlaybackIndexCount
           << " build_seconds=" << Timer::now() - start);
}


void DexelMesh::setPlayback(bool playback, unsigned speed) {
  unsigned nextStride = !playback || speed <= 8 ? 1 : speed <= 64 ? 2 : 4;
  if (playback && (!this->playback || playbackStride != nextStride))
    LOG_INFO(1, "Dexel playback display LOD: speed=" << speed
             << " stride=" << nextStride);
  this->playback = playback;
  playbackStride = nextStride;
  Profile::setMetric("dexel_display_playback_stride", playbackStride);
}


void DexelMesh::glDraw(GLContext &gl) {
  if (!indexCount) return;
  vertices.enable(3);
  normals.enable(3);
  unsigned drawIndices = indices;
  unsigned drawIndexCount = indexCount;
  if (playback && playbackStride == 2 && mediumPlaybackIndexCount) {
    drawIndices = mediumPlaybackIndices;
    drawIndexCount = mediumPlaybackIndexCount;
  } else if (playback && playbackStride == 4 && fastPlaybackIndexCount) {
    drawIndices = fastPlaybackIndices;
    drawIndexCount = fastPlaybackIndexCount;
  }
  gl.glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, drawIndices);
  gl.glDrawElements(GL_TRIANGLES, drawIndexCount, GL_UNSIGNED_INT, 0);
  gl.glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  vertices.disable();
  normals.disable();
  if (!playback || playbackStride == 1) {
    // Pull coplanar exact patches slightly toward the camera.  The complete
    // coarse surface remains a closed fallback, while the exact producer-local
    // surface wins the depth test wherever both representations overlap.
    gl.glEnable(GL_POLYGON_OFFSET_FILL);
    gl.glPolygonOffset(-1, -1);
    for (const SmartPointer<Mesh> &mesh: boundaryMeshes)
      if (!mesh.isNull() && !mesh->empty()) mesh->glDraw(gl);
    gl.glDisable(GL_POLYGON_OFFSET_FILL);
  }
}
