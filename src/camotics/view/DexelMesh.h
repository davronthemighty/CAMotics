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

#include "GLObject.h"
#include "Mesh.h"
#include "VBO.h"

#include <memory>
#include <vector>

namespace CAMotics {
namespace Dexel {
  struct BoundaryTile;
  class GridSurface;
}

  class DexelMesh : public GLObject {
    VBO vertices = GL_ATTR_POSITION;
    VBO normals = GL_ATTR_NORMAL;
    unsigned indices = 0;
    unsigned indexCount = 0;
    unsigned mediumPlaybackIndices = 0;
    unsigned mediumPlaybackIndexCount = 0;
    unsigned fastPlaybackIndices = 0;
    unsigned fastPlaybackIndexCount = 0;
    unsigned gridX = 0;
    unsigned gridY = 0;
    bool playback = false;
    unsigned playbackStride = 1;
    std::vector<float> displayedTops;
    std::vector<cb::SmartPointer<Mesh> > boundaryMeshes;
    std::vector<std::shared_ptr<const Dexel::BoundaryTile> >
      displayedBoundaryTiles;

    void updateGrid(const Dexel::GridSurface &surface);
    void updateBoundaryTiles(Dexel::GridSurface &surface);

  public:
    ~DexelMesh();

    void clear();
    void load(Dexel::GridSurface &surface);
    void setPlayback(bool playback, unsigned speed);

    void glDraw(GLContext &gl) override;
  };
}
