/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2026 Joseph Coffland
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#pragma once

#include <cbang/geom/Rectangle.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>


namespace CAMotics {namespace Internal {
  inline bool finitePoint(const cb::Vector3D &point) {
    return std::isfinite(point.x()) && std::isfinite(point.y()) &&
      std::isfinite(point.z());
  }


  inline bool finitePoint(const cb::Vector3F &point) {
    return std::isfinite(point.x()) && std::isfinite(point.y()) &&
      std::isfinite(point.z());
  }


  inline bool finiteBounds(const cb::Rectangle3D &bounds) {
    return finitePoint(bounds.getMin()) && finitePoint(bounds.getMax()) &&
      finitePoint(bounds.getDimensions());
  }


  inline bool multiplySize(size_t a, size_t b, size_t &result) {
    if (a && std::numeric_limits<size_t>::max() / a < b) return false;
    result = a * b;
    return true;
  }


  inline bool ceilToUnsigned(double length, double resolution,
                             unsigned &result) {
    if (!std::isfinite(length) || !std::isfinite(resolution) ||
        length <= 0 || resolution <= 0)
      return false;

    double value = std::ceil(length / resolution);
    if (!std::isfinite(value) || value < 1 ||
        std::numeric_limits<unsigned>::max() < value)
      return false;
    result = (unsigned)value;
    return true;
  }


  inline bool quantizeCoordinate(double value, double tolerance,
                                 int64_t &result) {
    if (!std::isfinite(value) || !std::isfinite(tolerance) ||
        tolerance <= 0)
      return false;

    double scaled = value / tolerance;
    double limit = std::nextafter
      ((double)std::numeric_limits<int64_t>::max(), 0.0);
    if (!std::isfinite(scaled) || std::fabs(scaled) > limit) return false;
    result = (int64_t)std::llround(scaled);
    return true;
  }
}}
