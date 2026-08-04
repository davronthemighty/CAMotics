/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#include "DexelHeightMap.h"
#include "DexelSimulation.h"

#include <cbang/Exception.h>

#include <zlib.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <ostream>
#include <vector>

using namespace std;
using namespace CAMotics;


namespace {
  void writeU32(ostream &stream, uint32_t value) {
    const unsigned char bytes[4] = {
      (unsigned char)(value >> 24),
      (unsigned char)(value >> 16),
      (unsigned char)(value >> 8),
      (unsigned char)value,
    };
    stream.write((const char *)bytes, sizeof(bytes));
  }


  uint64_t writeChunk(ostream &stream, const char type[4],
                      const vector<unsigned char> &data) {
    if (numeric_limits<uint32_t>::max() < data.size())
      THROW("PNG chunk exceeds the 32-bit format limit");

    writeU32(stream, (uint32_t)data.size());
    stream.write(type, 4);
    if (!data.empty())
      stream.write((const char *)data.data(), data.size());

    uLong crc = crc32(0L, Z_NULL, 0);
    crc = crc32(crc, reinterpret_cast<const Bytef *>(type), 4);
    if (!data.empty())
      crc = crc32(crc, data.data(), data.size());
    writeU32(stream, (uint32_t)crc);
    if (!stream) THROW("Failed writing Dexel height-map PNG");
    return data.size() + 12;
  }
}


Dexel::HeightMap Dexel::makeHeightMap(const GridSurface &grid) {
  HeightMap map;
  map.width = grid.getNX() + 1;
  map.height = grid.getNY() + 1;

  const vector<float> &tops = grid.getTops();
  uint64_t pixelCount = (uint64_t)map.width * map.height;
  if (!map.width || !map.height || pixelCount != tops.size() ||
      numeric_limits<size_t>::max() < pixelCount)
    THROW("Invalid Dexel grid dimensions for height-map export");

  auto minMax = minmax_element(tops.begin(), tops.end());
  map.minZ = *minMax.first;
  map.maxZ = *minMax.second;
  if (!isfinite(map.minZ) || !isfinite(map.maxZ))
    THROW("Dexel height map contains a non-finite height");

  map.pixels.resize((size_t)pixelCount);
  const double range = (double)map.maxZ - map.minZ;
  for (unsigned imageY = 0; imageY < map.height; imageY++) {
    // Image rows run top-to-bottom; grid Y runs minimum-to-maximum.
    unsigned gridY = map.height - imageY - 1;
    uint64_t sourceRow = (uint64_t)gridY * map.width;
    uint64_t targetRow = (uint64_t)imageY * map.width;
    for (unsigned x = 0; x < map.width; x++) {
      double scaled = range ?
        255 * (tops[sourceRow + x] - map.minZ) / range : 255;
      map.pixels[targetRow + x] =
        (unsigned char)lround(max(0.0, min(255.0, scaled)));
    }
  }

  return map;
}


uint64_t Dexel::writeHeightMapPNG(ostream &stream, const HeightMap &map) {
  uint64_t pixelCount = (uint64_t)map.width * map.height;
  if (!map.width || !map.height || pixelCount != map.pixels.size())
    THROW("Invalid grayscale height map for PNG output");

  uint64_t rawSize = ((uint64_t)map.width + 1) * map.height;
  if (numeric_limits<size_t>::max() < rawSize ||
      numeric_limits<uLong>::max() < rawSize)
    THROW("Dexel height-map PNG is too large to compress");

  vector<unsigned char> raw((size_t)rawSize);
  for (unsigned y = 0; y < map.height; y++) {
    uint64_t row = (uint64_t)y * (map.width + 1);
    raw[row] = 0; // PNG filter: None
    memcpy(raw.data() + row + 1,
           map.pixels.data() + (uint64_t)y * map.width, map.width);
  }

  uLongf compressedSize = compressBound((uLong)raw.size());
  vector<unsigned char> compressed(compressedSize);
  int result = compress2(compressed.data(), &compressedSize, raw.data(),
                         (uLong)raw.size(), Z_BEST_SPEED);
  if (result != Z_OK)
    THROW("Failed compressing Dexel height-map PNG: zlib error " << result);
  compressed.resize(compressedSize);

  static const unsigned char signature[8] =
    {137, 80, 78, 71, 13, 10, 26, 10};
  stream.write((const char *)signature, sizeof(signature));

  vector<unsigned char> header(13, 0);
  header[0] = map.width >> 24;
  header[1] = map.width >> 16;
  header[2] = map.width >> 8;
  header[3] = map.width;
  header[4] = map.height >> 24;
  header[5] = map.height >> 16;
  header[6] = map.height >> 8;
  header[7] = map.height;
  header[8] = 8; // bit depth: 256 grayscale values
  header[9] = 0; // color type: grayscale
  header[10] = 0; // compression
  header[11] = 0; // filter
  header[12] = 0; // no interlace

  uint64_t bytes = sizeof(signature);
  bytes += writeChunk(stream, "IHDR", header);
  bytes += writeChunk(stream, "IDAT", compressed);
  bytes += writeChunk(stream, "IEND", {});
  return bytes;
}
