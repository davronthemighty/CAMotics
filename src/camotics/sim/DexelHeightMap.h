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

#include <cstdint>
#include <iosfwd>
#include <vector>


namespace CAMotics {
namespace Dexel {
  class GridSurface;

  struct HeightMap {
    unsigned width = 0;
    unsigned height = 0;
    float minZ = 0;
    float maxZ = 0;
    std::vector<unsigned char> pixels;
  };

  HeightMap makeHeightMap(const GridSurface &grid);
  uint64_t writeHeightMapPNG(std::ostream &stream, const HeightMap &map);
}}
