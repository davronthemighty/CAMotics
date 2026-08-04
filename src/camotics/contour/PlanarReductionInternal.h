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

#include "PlanarReduction.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <utility>
#include <unordered_map>
#include <vector>

namespace CAMotics {
  class Surface;
  class TriangleSurface;

  namespace PlanarReductionInternal {
    struct Vec2 {
      double x = 0;
      double y = 0;
    };


    struct Vec3 {
      double x = 0;
      double y = 0;
      double z = 0;
    };


    inline double dot(const Vec3 &a, const Vec3 &b) {
      return a.x * b.x + a.y * b.y + a.z * b.z;
    }


    inline Vec3 operator-(const Vec3 &a, const Vec3 &b) {
      return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
    }


    inline Vec3 cross(const Vec3 &a, const Vec3 &b) {
      return Vec3{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
      };
    }


    inline double length(const Vec3 &v) {return std::sqrt(dot(v, v));}


    inline Vec3 normalize(const Vec3 &v) {
      double l = length(v);
      if (!l) return Vec3();
      return Vec3{v.x / l, v.y / l, v.z / l};
    }


    struct EdgeKey {
      uint32_t a = 0;
      uint32_t b = 0;

      EdgeKey() {}
      EdgeKey(uint32_t a, uint32_t b) :
        a(std::min(a, b)), b(std::max(a, b)) {}

      bool operator==(const EdgeKey &o) const {return a == o.a && b == o.b;}
    };


    struct EdgeKeyHash {
      size_t operator()(const EdgeKey &key) const {
        uint64_t value = ((uint64_t)key.a << 32) | key.b;
        value ^= value >> 33;
        value *= 0xff51afd7ed558ccdULL;
        value ^= value >> 33;
        value *= 0xc4ceb9fe1a85ec53ULL;
        value ^= value >> 33;
        return (size_t)value;
      }
    };


    struct EdgeCount {
      uint32_t count = 0;
      uint32_t forwardCount = 0;
    };


    struct VertexKey {
      int64_t x = 0;
      int64_t y = 0;
      int64_t z = 0;

      bool operator==(const VertexKey &o) const {
        return x == o.x && y == o.y && z == o.z;
      }
    };


    struct VertexKeyHash {
      size_t operator()(const VertexKey &key) const {
        uint64_t h = 1469598103934665603ULL;
        auto mix = [&](uint64_t v) {
          h ^= v;
          h *= 1099511628211ULL;
        };

        mix((uint64_t)key.x);
        mix((uint64_t)key.y);
        mix((uint64_t)key.z);
        return (size_t)h;
      }
    };


    struct EdgeInfo {
      int32_t owner = -1;
      uint32_t count = 0;
      uint32_t forwardCount = 0;
    };


    struct MeshData {
      uint64_t triangleCount = 0;
      uint64_t nonManifoldEdges = 0;
      uint64_t degenerateTriangles = 0;
      uint64_t sourceExpectedFloats = 0;
      uint64_t sourceVertexFloats = 0;
      uint64_t sourceNormalFloats = 0;
      bool sourceVertexCountMismatch = false;
      bool sourceNormalCountMismatch = false;
      uint64_t sourceInvalidCoordinates = 0;
      bool sourceRangeMismatch = false;
      std::vector<std::array<uint32_t, 3> > triangles;
      std::vector<Vec3> normals;
      std::vector<Vec3> points;
      std::vector<int32_t> neighbors;
      std::unordered_map<EdgeKey, EdgeInfo, EdgeKeyHash> edgeInfo;
      Vec3 boundsMin;
      Vec3 boundsMax;
      bool boundsValid = false;
      double coordTolerance = 0;

      bool sourceBufferMismatch() const {
        return sourceVertexCountMismatch || sourceNormalCountMismatch ||
          sourceInvalidCoordinates || sourceRangeMismatch;
      }
    };


    struct TrustedNeighborValidation {
      bool openSlot = false;
      bool range = false;
      bool self = false;
      bool duplicate = false;
      bool asymmetry = false;
      bool orientation = false;
      bool edgeMismatch = false;
      uint64_t orientationMismatches = 0;
      uint64_t edgeMismatches = 0;
    };


    constexpr uint8_t INVALID_TRUSTED_RECIPROCAL_SLOT = 255;


    struct EdgeIncidenceReport {
      uint64_t boundaryEdges = 0;
      uint64_t nonManifoldEdges = 0;
      uint64_t misorientedEdges = 0;
      uint64_t degenerateTriangles = 0;
      uint64_t duplicateTriangles = 0;

      bool watertight() const {return !boundaryEdges && !nonManifoldEdges;}
    };


    struct ReplacementCheck {
      bool checked = false;
      bool estimateAvailable = false;
      bool feasible = false;
      bool edgeIncidenceChecked = false;
      bool edgeIncidenceOk = false;
      bool complexityRejected = false;
      uint64_t trianglesAfter = 0;
      std::vector<std::array<uint32_t, 3> > triangles;
    };


    inline std::pair<uint32_t, uint32_t>
    getTriangleEdge(const std::array<uint32_t, 3> &triangle, unsigned slot) {
      if (slot == 0) return std::make_pair(triangle[0], triangle[1]);
      if (slot == 1) return std::make_pair(triangle[1], triangle[2]);
      return std::make_pair(triangle[2], triangle[0]);
    }


    inline bool isForwardEdge(const std::pair<uint32_t, uint32_t> &edge,
                              const EdgeKey &key) {
      return edge.first == key.a && edge.second == key.b;
    }


    template<typename EdgeMap>
    uint64_t countMisorientedEdges(const EdgeMap &edges) {
      uint64_t misoriented = 0;
      for (const auto &entry: edges) {
        const auto &info = entry.second;
        if (info.count == 2 &&
            (info.forwardCount == 0 || info.forwardCount == 2))
          misoriented++;
      }
      return misoriented;
    }


    template<typename T>
    void releaseVector(std::vector<T> &values) {
      std::vector<T>().swap(values);
    }


    template<typename K, typename V, typename H>
    void releaseMap(std::unordered_map<K, V, H> &values) {
      std::unordered_map<K, V, H>().swap(values);
    }


    VertexKey packVertex(const std::vector<float> &vertices, uint64_t offset,
                         double tolerance);
    uint32_t getVertexId(
      const VertexKey &key, const Vec3 &point,
      std::unordered_map<VertexKey, uint32_t, VertexKeyHash> &vertexIds,
      std::vector<Vec3> &points);
    uint32_t getVertexId(
      const VertexKey &key,
      std::unordered_map<VertexKey, uint32_t, VertexKeyHash> &vertexIds);
    EdgeIncidenceReport validateEdgeIncidenceVertices(
      const std::vector<float> &vertices,
      const PlanarReductionConfig &config);
    void appendOriginalTriangle(
      std::vector<float> &vertices, std::vector<float> &normals,
      const std::vector<float> &sourceVertices,
      const std::vector<float> &sourceNormals, uint64_t tri);
    void appendReducedTriangles(
      std::vector<float> &vertices, std::vector<float> &normals,
      const std::vector<std::array<uint32_t, 3> > &replacement,
      const std::vector<Vec3> &points, const Vec3 &seedNormal);
    void releaseMeshData(MeshData &mesh);
    void releaseMeshConnectivity(MeshData &mesh);
    TrustedNeighborValidation validateTrustedNeighbors(
      const std::vector<int32_t> &neighbors, uint64_t triangleCount,
      std::vector<uint8_t> *reciprocalSlots = 0);
    TrustedNeighborValidation validateTrustedNeighborEdges(
      const MeshData &mesh, const std::vector<int32_t> &neighbors,
      const std::vector<uint8_t> *reciprocalSlots = 0);


    bool orderBoundaryLoop(
      const std::vector<std::pair<uint32_t, uint32_t> > &edges,
      std::vector<uint32_t> &loop);
    bool orderBoundaryLoops(
      const std::vector<std::pair<uint32_t, uint32_t> > &edges,
      std::vector<std::vector<uint32_t> > &loops);
    bool triangulateLoop(
      const std::vector<uint32_t> &inputLoop,
      const std::vector<Vec3> &points, const Vec3 &seedNormal,
      std::vector<std::array<uint32_t, 3> > &out,
      bool *complexityRejected = 0);
    bool triangulateHoleAwareLoops(
      const std::vector<std::vector<uint32_t> > &loops,
      const std::vector<Vec3> &points, const Vec3 &seedNormal,
      std::vector<std::array<uint32_t, 3> > &out);
    ReplacementCheck checkPhase1Replacement(
      const std::vector<std::pair<uint32_t, uint32_t> > &boundaryEdges,
      const std::vector<Vec3> &points, const Vec3 &seedNormal);
    ReplacementCheck checkHoleAwareReplacement(
      const std::vector<std::pair<uint32_t, uint32_t> > &boundaryEdges,
      const std::vector<Vec3> &points, const Vec3 &seedNormal);
    unsigned projectionDropAxis(const Vec3 &normal);
    Vec2 projectPoint(const Vec3 &point, unsigned dropAxis);
    double polygonArea2D(const std::vector<uint32_t> &loop,
                         const std::vector<Vec3> &points,
                         unsigned dropAxis);
    bool replacementBoundaryMatches(
      const std::vector<std::pair<uint32_t, uint32_t> > &boundaryEdges,
      const std::vector<std::array<uint32_t, 3> > &replacement);
    bool replacementEdgeIncidenceOk(
      const std::vector<std::pair<uint32_t, uint32_t> > &boundaryEdges,
      const std::vector<std::array<uint32_t, 3> > &replacement);
    bool orientReplacementToBoundary(
      const std::vector<std::pair<uint32_t, uint32_t> > &boundaryEdges,
      std::vector<std::array<uint32_t, 3> > &replacement);
    bool replacementBoundaryMatchesDirection(
      const std::vector<std::pair<uint32_t, uint32_t> > &boundaryEdges,
      const std::vector<std::array<uint32_t, 3> > &replacement);
    void flipReplacementTriangles(
      std::vector<std::array<uint32_t, 3> > &replacement);
    PlanarReductionReport analyzeOrReduceCore(
      const Surface &surface, const PlanarReductionConfig &config,
      TriangleSurface *target);
    void resetAppliedReductionReport(PlanarReductionReport &report);
    void finalizeSideReductionReports(PlanarReductionReport &report);
    bool runPlanarReductionContractSelfTestCore(std::string &failure);
  }
}
