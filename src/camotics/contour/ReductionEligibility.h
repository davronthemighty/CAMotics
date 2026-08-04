/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2022 Joseph Coffland
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>


namespace CAMotics {
  enum ReductionTriangleOrigin : uint8_t {
    REDUCTION_MC_REDUCIBLE = 0,
    REDUCTION_MC_SEAM_LOCKED = 1,
    REDUCTION_ANALYTIC_LOCKED = 2,
    REDUCTION_UNKNOWN_LOCKED = 3
  };


  struct ReductionLockedVertex {
    int64_t x = 0;
    int64_t y = 0;
    int64_t z = 0;

    bool operator==(const ReductionLockedVertex &o) const {
      return x == o.x && y == o.y && z == o.z;
    }

    bool operator<(const ReductionLockedVertex &o) const {
      if (x != o.x) return x < o.x;
      if (y != o.y) return y < o.y;
      return z < o.z;
    }
  };


  struct ReductionLockedEdge {
    ReductionLockedVertex a;
    ReductionLockedVertex b;

    ReductionLockedEdge() {}
    ReductionLockedEdge(const ReductionLockedVertex &a,
                        const ReductionLockedVertex &b) : a(a), b(b) {
      if (this->b < this->a) std::swap(this->a, this->b);
    }

    bool operator==(const ReductionLockedEdge &o) const {
      return a == o.a && b == o.b;
    }

    bool operator<(const ReductionLockedEdge &o) const {
      if (a < o.a) return true;
      if (o.a < a) return false;
      return b < o.b;
    }
  };


  struct ReductionEligibility {
    double quantizationTolerance = 0;
    std::vector<uint8_t> triangleOrigins;
    std::vector<ReductionLockedVertex> lockedSeamVertices;
    std::vector<ReductionLockedEdge> lockedSeamEdges;
    std::string bindingHash;

    uint64_t count(ReductionTriangleOrigin origin) const;
    std::string computeBindingHash
      (const std::vector<float> &vertices) const;
    void seal(const std::vector<float> &vertices);
    bool validFor(const std::vector<float> &vertices) const;
  };


  ReductionLockedVertex quantizeReductionVertex
    (float x, float y, float z, double tolerance);
}
