/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2026 Joseph Coffland
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

\******************************************************************************/

#include "ContourProvenance.h"

#include <camotics/Profile.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

using namespace std;
using namespace CAMotics;


namespace {
  bool captureContourProvenance = false;


  struct ProvenanceVertexKey {
    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t z = 0;
    uint8_t kind = 0;
    uint8_t axis = 0;

    bool operator==(const ProvenanceVertexKey &o) const {
      return x == o.x && y == o.y && z == o.z && kind == o.kind &&
        axis == o.axis;
    }
  };


  struct ProvenanceVertexKeyHash {
    size_t operator()(const ProvenanceVertexKey &key) const {
      size_t seed = key.x;
      seed ^= key.y + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
      seed ^= key.z + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
      seed ^= key.kind + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
      seed ^= key.axis + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
      return seed;
    }
  };


  bool operator<(const ProvenanceVertexKey &a, const ProvenanceVertexKey &b) {
    if (a.kind != b.kind) return a.kind < b.kind;
    if (a.axis != b.axis) return a.axis < b.axis;
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    return a.z < b.z;
  }


  struct ProvenanceEdgeKey {
    ProvenanceVertexKey a;
    ProvenanceVertexKey b;

    ProvenanceEdgeKey(const ProvenanceVertexKey &a,
                      const ProvenanceVertexKey &b) {
      if (b < a) {
        this->a = b;
        this->b = a;
      } else {
        this->a = a;
        this->b = b;
      }
    }

    bool operator==(const ProvenanceEdgeKey &o) const {
      return a == o.a && b == o.b;
    }
  };


  struct ProvenanceEdgeKeyHash {
    size_t operator()(const ProvenanceEdgeKey &key) const {
      ProvenanceVertexKeyHash hash;
      size_t seed = hash(key.a);
      seed ^= hash(key.b) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
      return seed;
    }
  };


  struct QuantizedVertexKey {
    int64_t x = 0;
    int64_t y = 0;
    int64_t z = 0;

    bool operator==(const QuantizedVertexKey &o) const {
      return x == o.x && y == o.y && z == o.z;
    }
  };


  struct QuantizedVertexKeyHash {
    size_t operator()(const QuantizedVertexKey &key) const {
      size_t seed = (uint64_t)key.x;
      seed ^= (uint64_t)key.y + 0x9e3779b97f4a7c15ULL +
        (seed << 6) + (seed >> 2);
      seed ^= (uint64_t)key.z + 0x9e3779b97f4a7c15ULL +
        (seed << 6) + (seed >> 2);
      return seed;
    }
  };


  bool operator<(const QuantizedVertexKey &a, const QuantizedVertexKey &b) {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    return a.z < b.z;
  }


  struct QuantizedEdgeKey {
    QuantizedVertexKey a;
    QuantizedVertexKey b;

    QuantizedEdgeKey(const QuantizedVertexKey &a, const QuantizedVertexKey &b) {
      if (b < a) {
        this->a = b;
        this->b = a;
      } else {
        this->a = a;
        this->b = b;
      }
    }

    bool operator==(const QuantizedEdgeKey &o) const {
      return a == o.a && b == o.b;
    }
  };


  struct QuantizedEdgeKeyHash {
    size_t operator()(const QuantizedEdgeKey &key) const {
      QuantizedVertexKeyHash hash;
      size_t seed = hash(key.a);
      seed ^= hash(key.b) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
      return seed;
    }
  };


  struct RawWeldedSpread {
    QuantizedVertexKey firstA;
    QuantizedVertexKey firstB;
    bool initialized = false;
    uint64_t alternateSlots = 0;
  };


  struct RawVertexWeldedSpread {
    QuantizedVertexKey first;
    bool initialized = false;
    uint64_t alternateObservations = 0;
  };


  struct EdgeUse {
    uint32_t count = 0;
    uint32_t forwardCount = 0;
  };


  struct LocalEdge {
    uint8_t dx;
    uint8_t dy;
    uint8_t dz;
    uint8_t axis;
  };


  const LocalEdge localEdges[12] = {
    {0, 0, 0, 0}, {1, 0, 0, 1}, {0, 1, 0, 0}, {0, 0, 0, 1},
    {0, 0, 1, 0}, {1, 0, 1, 1}, {0, 1, 1, 0}, {0, 0, 1, 1},
    {0, 0, 0, 2}, {1, 0, 0, 2}, {1, 1, 0, 2}, {0, 1, 0, 2},
  };


  bool setProvenanceVertexCoords(ProvenanceVertexKey &key, uint64_t x,
                                 uint64_t y, uint64_t z) {
    const uint64_t maxCoord = numeric_limits<uint32_t>::max();
    if (maxCoord < x || maxCoord < y || maxCoord < z) return false;

    key.x = (uint32_t)x;
    key.y = (uint32_t)y;
    key.z = (uint32_t)z;
    return true;
  }


  bool getProvenanceVertexKey(const ContourTriangleProvenance &triangle,
                              unsigned index,
                              ProvenanceVertexKey &key) {
    const ContourVertexProvenance &vertex = triangle.vertices[index];

    if (vertex.kind == ContourVertexProvenance::CELL_CENTER) {
      key.kind = vertex.kind;
      key.axis = 3;
      return setProvenanceVertexCoords
        (key, triangle.cell[0], triangle.cell[1], triangle.cell[2]);
    }

    if (vertex.kind != ContourVertexProvenance::GRID_EDGE ||
        12 <= vertex.edge) return false;

    const LocalEdge &edge = localEdges[vertex.edge];
    key.kind = vertex.kind;
    key.axis = edge.axis;
    return setProvenanceVertexCoords
      (key, (uint64_t)triangle.cell[0] + edge.dx,
       (uint64_t)triangle.cell[1] + edge.dy,
       (uint64_t)triangle.cell[2] + edge.dz);
  }


  QuantizedVertexKey getQuantizedVertexKey(const vector<float> &vertices,
                                           uint64_t offset,
                                           double tolerance) {
    return QuantizedVertexKey{
      (int64_t)llround(vertices[offset + 0] / tolerance),
      (int64_t)llround(vertices[offset + 1] / tolerance),
      (int64_t)llround(vertices[offset + 2] / tolerance),
    };
  }


  void updateRawWeldedSpread
    (unordered_map<ProvenanceEdgeKey, RawWeldedSpread,
                   ProvenanceEdgeKeyHash> &spreads,
     const ProvenanceEdgeKey &rawKey, const QuantizedEdgeKey &weldedKey) {
    RawWeldedSpread &spread = spreads[rawKey];
    if (!spread.initialized) {
      spread.firstA = weldedKey.a;
      spread.firstB = weldedKey.b;
      spread.initialized = true;
      return;
    }

    if (!(spread.firstA == weldedKey.a && spread.firstB == weldedKey.b))
      spread.alternateSlots++;
  }


  void updateRawVertexWeldedSpread
    (unordered_map<ProvenanceVertexKey, RawVertexWeldedSpread,
                   ProvenanceVertexKeyHash> &spreads,
     const ProvenanceVertexKey &rawKey, const QuantizedVertexKey &weldedKey) {
    RawVertexWeldedSpread &spread = spreads[rawKey];
    if (!spread.initialized) {
      spread.first = weldedKey;
      spread.initialized = true;
      return;
    }

    if (!(spread.first == weldedKey)) spread.alternateObservations++;
  }


  void summarizeRawWeldedSpread
    (const unordered_map<ProvenanceEdgeKey, RawWeldedSpread,
                         ProvenanceEdgeKeyHash> &spreads,
     uint64_t &edges, uint64_t &edgeSlots, uint64_t &maxAlternateSlots) {
    for (const auto &entry: spreads) {
      uint64_t alternateSlots = entry.second.alternateSlots;
      if (!alternateSlots) continue;

      edges++;
      edgeSlots += alternateSlots;
      maxAlternateSlots = max(maxAlternateSlots, alternateSlots);
    }
  }


  void summarizeRawVertexWeldedSpread
    (const unordered_map<ProvenanceVertexKey, RawVertexWeldedSpread,
                         ProvenanceVertexKeyHash> &spreads,
     uint64_t &uniqueKeys, uint64_t &spreadKeys,
     uint64_t &spreadObservations, uint64_t &maxAlternateObservations) {
    uniqueKeys = spreads.size();
    for (const auto &entry: spreads) {
      uint64_t alternate = entry.second.alternateObservations;
      if (!alternate) continue;

      spreadKeys++;
      spreadObservations += alternate;
      maxAlternateObservations = max(maxAlternateObservations, alternate);
    }
  }


  void addEdgeUse(EdgeUse &use, bool forward) {
    use.count++;
    if (forward) use.forwardCount++;
  }


  bool isForwardEdge(const ProvenanceVertexKey &a,
                     const ProvenanceVertexKey &b,
                     const ProvenanceEdgeKey &key) {
    return a == key.a && b == key.b;
  }


  bool isForwardEdge(const QuantizedVertexKey &a,
                     const QuantizedVertexKey &b,
                     const QuantizedEdgeKey &key) {
    return a == key.a && b == key.b;
  }


  template<typename EdgeMap>
  void countEdgeIncidence(const EdgeMap &edges, uint64_t &boundaryEdges,
                          uint64_t &nonManifoldEdges,
                          uint64_t &misorientedEdges,
                          uint64_t &twinEdgeSlots,
                          uint64_t &boundaryEdgeSlots,
                          uint64_t &nonManifoldEdgeSlots,
                          uint64_t &uniqueEdges,
                          uint64_t &maxEdgeIncidence,
                          uint64_t &edgesIncidence1,
                          uint64_t &edgesIncidence2,
                          uint64_t &edgesIncidence3,
                          uint64_t &edgesIncidence4,
                          uint64_t &edgesIncidence5Plus) {
    uniqueEdges = edges.size();
    for (const auto &entry: edges) {
      const EdgeUse &use = entry.second;
      uint32_t count = use.count;
      maxEdgeIncidence = max<uint64_t>(maxEdgeIncidence, count);

      if (count == 1) edgesIncidence1++;
      else if (count == 2) edgesIncidence2++;
      else if (count == 3) edgesIncidence3++;
      else if (count == 4) edgesIncidence4++;
      else edgesIncidence5Plus++;

      if (count == 1) {
        boundaryEdges++;
        boundaryEdgeSlots++;

      } else if (count == 2) {
        twinEdgeSlots += 2;
        if (use.forwardCount == 0 || use.forwardCount == 2)
          misorientedEdges++;
      }

      else if (2 < count) {
        nonManifoldEdges++;
        nonManifoldEdgeSlots += count;
      }
    }
  }
}


void CAMotics::setContourProvenanceCapture(bool value) {
  captureContourProvenance = value;
}


bool CAMotics::shouldCaptureContourProvenance() {
  return captureContourProvenance;
}


ContourProvenanceReport CAMotics::analyzeContourProvenance
  (const vector<ContourTriangleProvenance> &provenance,
   const vector<float> &vertices, uint64_t startVertexOffset,
   uint64_t expectedTriangles, double tolerance) {
  ContourProvenanceReport report;
  report.expectedTriangles = expectedTriangles;
  report.triangles = provenance.size();

  unordered_map<ProvenanceEdgeKey, EdgeUse, ProvenanceEdgeKeyHash> rawEdges;
  unordered_map<ProvenanceEdgeKey, EdgeUse, ProvenanceEdgeKeyHash>
    rawGridGridEdges;
  unordered_map<ProvenanceEdgeKey, EdgeUse, ProvenanceEdgeKeyHash>
    rawCenterInvolvedEdges;
  unordered_map<ProvenanceEdgeKey, RawWeldedSpread, ProvenanceEdgeKeyHash>
    rawGridGridWeldedSpreads;
  unordered_map<ProvenanceEdgeKey, RawWeldedSpread, ProvenanceEdgeKeyHash>
    rawCenterInvolvedWeldedSpreads;
  unordered_map<ProvenanceVertexKey, RawVertexWeldedSpread,
                ProvenanceVertexKeyHash> rawGridVertexWeldedSpreads;
  unordered_map<ProvenanceVertexKey, RawVertexWeldedSpread,
                ProvenanceVertexKeyHash> rawCenterVertexWeldedSpreads;
  unordered_map<QuantizedEdgeKey, EdgeUse, QuantizedEdgeKeyHash> weldedEdges;
  rawEdges.reserve(provenance.size() * 3 / 2);
  rawGridGridEdges.reserve(provenance.size() * 3 / 2);
  rawCenterInvolvedEdges.reserve(provenance.size() / 2);
  rawGridGridWeldedSpreads.reserve(provenance.size() * 3 / 2);
  rawCenterInvolvedWeldedSpreads.reserve(provenance.size() / 2);
  rawGridVertexWeldedSpreads.reserve(provenance.size() * 3 / 2);
  rawCenterVertexWeldedSpreads.reserve(provenance.size() / 2);
  weldedEdges.reserve(provenance.size() * 3 / 2);

  for (uint64_t tri = 0; tri < provenance.size(); tri++) {
    const ContourTriangleProvenance &triangle = provenance[tri];
    ProvenanceVertexKey keys[3];
    QuantizedVertexKey weldedKeys[3];
    bool complete =
      triangle.algorithm != ContourTriangleProvenance::UNKNOWN_ALGORITHM;
    bool hasCellCenter = false;

    for (unsigned i = 0; i < 3; i++) {
      if (triangle.vertices[i].kind == ContourVertexProvenance::CELL_CENTER)
        hasCellCenter = true;
      if (!getProvenanceVertexKey(triangle, i, keys[i])) complete = false;
      weldedKeys[i] = getQuantizedVertexKey
        (vertices, startVertexOffset + tri * 9 + (uint64_t)i * 3,
         tolerance);
    }

    if (!complete) {
      report.unknownTriangles++;
      continue;
    }

    report.completeTriangles++;
    if (hasCellCenter) report.cellCenterTriangles++;
    else report.gridEdgeTriangles++;

    for (unsigned i = 0; i < 3; i++)
      if (keys[i].kind == ContourVertexProvenance::GRID_EDGE)
        updateRawVertexWeldedSpread
          (rawGridVertexWeldedSpreads, keys[i], weldedKeys[i]);
      else
        updateRawVertexWeldedSpread
          (rawCenterVertexWeldedSpreads, keys[i], weldedKeys[i]);

    for (unsigned i = 0; i < 3; i++) {
      ProvenanceEdgeKey rawKey(keys[i], keys[(i + 1) % 3]);
      QuantizedEdgeKey weldedKey(weldedKeys[i], weldedKeys[(i + 1) % 3]);
      addEdgeUse(rawEdges[rawKey],
                 isForwardEdge(keys[i], keys[(i + 1) % 3], rawKey));
      if (keys[i].kind == ContourVertexProvenance::GRID_EDGE &&
          keys[(i + 1) % 3].kind == ContourVertexProvenance::GRID_EDGE) {
        addEdgeUse
          (rawGridGridEdges[rawKey],
           isForwardEdge(keys[i], keys[(i + 1) % 3], rawKey));
        updateRawWeldedSpread(rawGridGridWeldedSpreads, rawKey, weldedKey);

      } else {
        addEdgeUse
          (rawCenterInvolvedEdges[rawKey],
           isForwardEdge(keys[i], keys[(i + 1) % 3], rawKey));
        updateRawWeldedSpread
          (rawCenterInvolvedWeldedSpreads, rawKey, weldedKey);
      }

      addEdgeUse(weldedEdges[weldedKey],
                 isForwardEdge(weldedKeys[i], weldedKeys[(i + 1) % 3],
                               weldedKey));
    }
  }

  countEdgeIncidence(rawEdges, report.rawBoundaryEdges,
                     report.rawNonManifoldEdges, report.rawMisorientedEdges,
                     report.rawTwinEdgeSlots,
                     report.rawBoundaryEdgeSlots,
                     report.rawNonManifoldEdgeSlots, report.rawUniqueEdges,
                     report.rawMaxEdgeIncidence,
                     report.rawEdgesIncidence1,
                     report.rawEdgesIncidence2,
                     report.rawEdgesIncidence3,
                     report.rawEdgesIncidence4,
                     report.rawEdgesIncidence5Plus);
  uint64_t ignoredBoundaryEdges = 0;
  uint64_t ignoredNonManifoldEdges = 0;
  uint64_t ignoredMisorientedEdges = 0;
  uint64_t ignoredEdgesIncidence1 = 0;
  uint64_t ignoredEdgesIncidence2 = 0;
  uint64_t ignoredEdgesIncidence3 = 0;
  uint64_t ignoredEdgesIncidence4 = 0;
  uint64_t ignoredEdgesIncidence5Plus = 0;
  countEdgeIncidence(rawGridGridEdges, ignoredBoundaryEdges,
                     ignoredNonManifoldEdges,
                     ignoredMisorientedEdges,
                     report.rawGridGridTwinEdgeSlots,
                     report.rawGridGridBoundaryEdgeSlots,
                     report.rawGridGridNonManifoldEdgeSlots,
                     report.rawGridGridUniqueEdges,
                     report.rawGridGridMaxEdgeIncidence,
                     ignoredEdgesIncidence1,
                     ignoredEdgesIncidence2,
                     ignoredEdgesIncidence3,
                     ignoredEdgesIncidence4,
                     ignoredEdgesIncidence5Plus);
  summarizeRawWeldedSpread(rawGridGridWeldedSpreads,
                           report.rawGridGridWeldedSpreadEdges,
                           report.rawGridGridWeldedSpreadEdgeSlots,
                           report.rawGridGridWeldedSpreadMaxAlternateSlots);
  ignoredBoundaryEdges = 0;
  ignoredNonManifoldEdges = 0;
  ignoredMisorientedEdges = 0;
  ignoredEdgesIncidence1 = 0;
  ignoredEdgesIncidence2 = 0;
  ignoredEdgesIncidence3 = 0;
  ignoredEdgesIncidence4 = 0;
  ignoredEdgesIncidence5Plus = 0;
  countEdgeIncidence(rawCenterInvolvedEdges, ignoredBoundaryEdges,
                     ignoredNonManifoldEdges,
                     ignoredMisorientedEdges,
                     report.rawCenterInvolvedTwinEdgeSlots,
                     report.rawCenterInvolvedBoundaryEdgeSlots,
                     report.rawCenterInvolvedNonManifoldEdgeSlots,
                     report.rawCenterInvolvedUniqueEdges,
                     report.rawCenterInvolvedMaxEdgeIncidence,
                     ignoredEdgesIncidence1,
                     ignoredEdgesIncidence2,
                     ignoredEdgesIncidence3,
                     ignoredEdgesIncidence4,
                     ignoredEdgesIncidence5Plus);
  summarizeRawWeldedSpread
    (rawCenterInvolvedWeldedSpreads,
     report.rawCenterInvolvedWeldedSpreadEdges,
     report.rawCenterInvolvedWeldedSpreadEdgeSlots,
     report.rawCenterInvolvedWeldedSpreadMaxAlternateSlots);
  summarizeRawVertexWeldedSpread
    (rawGridVertexWeldedSpreads,
     report.rawGridVertexUniqueKeys,
     report.rawGridVertexWeldedSpreadKeys,
     report.rawGridVertexWeldedSpreadObservations,
     report.rawGridVertexWeldedSpreadMaxAlternateObservations);
  summarizeRawVertexWeldedSpread
    (rawCenterVertexWeldedSpreads,
     report.rawCenterVertexUniqueKeys,
     report.rawCenterVertexWeldedSpreadKeys,
     report.rawCenterVertexWeldedSpreadObservations,
     report.rawCenterVertexWeldedSpreadMaxAlternateObservations);
  countEdgeIncidence(weldedEdges, report.boundaryEdges,
                     report.nonManifoldEdges, report.misorientedEdges,
                     report.weldedTwinEdgeSlots,
                     report.weldedBoundaryEdgeSlots,
                     report.weldedNonManifoldEdgeSlots,
                     report.weldedUniqueEdges,
                     report.weldedMaxEdgeIncidence,
                     report.weldedEdgesIncidence1,
                     report.weldedEdgesIncidence2,
                     report.weldedEdgesIncidence3,
                     report.weldedEdgesIncidence4,
                     report.weldedEdgesIncidence5Plus);

  report.watertight =
    !report.unknownTriangles && !report.boundaryEdges &&
    !report.nonManifoldEdges;

  return report;
}


bool CAMotics::buildContourProvenanceNeighbors
  (const vector<ContourTriangleProvenance> &provenance,
   const vector<float> &vertices, uint64_t startVertexOffset,
   uint64_t expectedTriangles, vector<int32_t> &neighbors,
   double tolerance, bool *usedRawProvenance) {
  if (usedRawProvenance) *usedRawProvenance = false;
  if (provenance.size() != expectedTriangles) return false;
  if ((uint64_t)numeric_limits<int32_t>::max() / 3 < expectedTriangles)
    return false;

  auto buildRaw = [&]() -> bool {
    vector<int32_t> candidate((size_t)expectedTriangles * 3, -1);
    unordered_map<ProvenanceEdgeKey, int32_t, ProvenanceEdgeKeyHash> owners;
    owners.reserve(provenance.size() * 3 / 2);

    for (uint64_t tri = 0; tri < provenance.size(); tri++) {
      const ContourTriangleProvenance &triangle = provenance[tri];
      if (triangle.algorithm == ContourTriangleProvenance::UNKNOWN_ALGORITHM)
        return false;

      ProvenanceVertexKey keys[3];
      for (unsigned i = 0; i < 3; i++)
        if (!getProvenanceVertexKey(triangle, i, keys[i])) return false;

      for (unsigned slot = 0; slot < 3; slot++) {
        ProvenanceEdgeKey key(keys[slot], keys[(slot + 1) % 3]);
        int32_t ownerSlot = (int32_t)(tri * 3 + slot);
        auto inserted = owners.emplace(key, ownerSlot);

        if (inserted.second) continue;
        if (inserted.first->second < 0) return false;

        int32_t otherSlot = inserted.first->second;
        uint64_t otherTri = (uint64_t)otherSlot / 3;
        candidate[(size_t)ownerSlot] = (int32_t)otherTri;
        candidate[(size_t)otherSlot] = (int32_t)tri;
        inserted.first->second = -2;
      }
    }

    for (const auto &entry: owners)
      if (0 <= entry.second) return false;

    neighbors.swap(candidate);
    if (usedRawProvenance) *usedRawProvenance = true;
    return true;
  };

  auto buildWelded = [&]() -> bool {
    vector<int32_t> candidate((size_t)expectedTriangles * 3, -1);
    unordered_map<QuantizedEdgeKey, int32_t, QuantizedEdgeKeyHash> owners;
    owners.reserve(provenance.size() * 3 / 2);

    for (uint64_t tri = 0; tri < provenance.size(); tri++) {
      const ContourTriangleProvenance &triangle = provenance[tri];
      if (triangle.algorithm == ContourTriangleProvenance::UNKNOWN_ALGORITHM)
        return false;

      QuantizedVertexKey keys[3];
      for (unsigned i = 0; i < 3; i++) {
        ProvenanceVertexKey ignored;
        if (!getProvenanceVertexKey(triangle, i, ignored)) return false;
        keys[i] = getQuantizedVertexKey
          (vertices, startVertexOffset + tri * 9 + (uint64_t)i * 3,
           tolerance);
      }

      for (unsigned slot = 0; slot < 3; slot++) {
        QuantizedEdgeKey key(keys[slot], keys[(slot + 1) % 3]);
        int32_t ownerSlot = (int32_t)(tri * 3 + slot);
        auto inserted = owners.emplace(key, ownerSlot);

        if (inserted.second) continue;
        if (inserted.first->second < 0) return false;

        int32_t otherSlot = inserted.first->second;
        uint64_t otherTri = (uint64_t)otherSlot / 3;
        candidate[(size_t)ownerSlot] = (int32_t)otherTri;
        candidate[(size_t)otherSlot] = (int32_t)tri;
        inserted.first->second = -2;
      }
    }

    neighbors.swap(candidate);
    return true;
  };

  if (buildRaw()) return true;
  return buildWelded();
}


void CAMotics::emitContourProvenanceMetrics
  (const ContourProvenanceReport &report) {
  Profile::setMetric("surface_provenance_expected_triangles",
                     report.expectedTriangles);
  Profile::setMetric("surface_provenance_triangles", report.triangles);
  Profile::setMetric("surface_provenance_complete_triangles",
                     report.completeTriangles);
  Profile::setMetric("surface_provenance_unknown_triangles",
                     report.unknownTriangles);
  Profile::setMetric("surface_provenance_grid_edge_triangles",
                     report.gridEdgeTriangles);
  Profile::setMetric("surface_provenance_cell_center_triangles",
                     report.cellCenterTriangles);
  Profile::setMetric("surface_provenance_raw_boundary_edges",
                     report.rawBoundaryEdges);
  Profile::setMetric("surface_provenance_raw_nonmanifold_edges",
                     report.rawNonManifoldEdges);
  Profile::setMetric("surface_provenance_raw_misoriented_edges",
                     report.rawMisorientedEdges);
  Profile::setMetric("surface_provenance_raw_unique_edges",
                     report.rawUniqueEdges);
  Profile::setMetric("surface_provenance_raw_max_edge_incidence",
                     report.rawMaxEdgeIncidence);
  Profile::setMetric("surface_provenance_raw_edges_incidence_1",
                     report.rawEdgesIncidence1);
  Profile::setMetric("surface_provenance_raw_edges_incidence_2",
                     report.rawEdgesIncidence2);
  Profile::setMetric("surface_provenance_raw_edges_incidence_3",
                     report.rawEdgesIncidence3);
  Profile::setMetric("surface_provenance_raw_edges_incidence_4",
                     report.rawEdgesIncidence4);
  Profile::setMetric("surface_provenance_raw_edges_incidence_5_plus",
                     report.rawEdgesIncidence5Plus);
  Profile::setMetric("surface_provenance_raw_twin_edge_slots",
                     report.rawTwinEdgeSlots);
  Profile::setMetric("surface_provenance_raw_boundary_edge_slots",
                     report.rawBoundaryEdgeSlots);
  Profile::setMetric("surface_provenance_raw_nonmanifold_edge_slots",
                     report.rawNonManifoldEdgeSlots);
  Profile::setMetric("surface_provenance_raw_grid_grid_unique_edges",
                     report.rawGridGridUniqueEdges);
  Profile::setMetric("surface_provenance_raw_grid_grid_max_edge_incidence",
                     report.rawGridGridMaxEdgeIncidence);
  Profile::setMetric("surface_provenance_raw_grid_grid_twin_edge_slots",
                     report.rawGridGridTwinEdgeSlots);
  Profile::setMetric("surface_provenance_raw_grid_grid_boundary_edge_slots",
                     report.rawGridGridBoundaryEdgeSlots);
  Profile::setMetric
    ("surface_provenance_raw_grid_grid_nonmanifold_edge_slots",
     report.rawGridGridNonManifoldEdgeSlots);
  Profile::setMetric
    ("surface_provenance_raw_grid_grid_welded_spread_edges",
     report.rawGridGridWeldedSpreadEdges);
  Profile::setMetric
    ("surface_provenance_raw_grid_grid_welded_spread_edge_slots",
     report.rawGridGridWeldedSpreadEdgeSlots);
  Profile::setMetric
    ("surface_provenance_raw_grid_grid_welded_spread_max_alternate_slots",
     report.rawGridGridWeldedSpreadMaxAlternateSlots);
  Profile::setMetric("surface_provenance_raw_center_involved_unique_edges",
                     report.rawCenterInvolvedUniqueEdges);
  Profile::setMetric
    ("surface_provenance_raw_center_involved_max_edge_incidence",
     report.rawCenterInvolvedMaxEdgeIncidence);
  Profile::setMetric("surface_provenance_raw_center_involved_twin_edge_slots",
                     report.rawCenterInvolvedTwinEdgeSlots);
  Profile::setMetric
    ("surface_provenance_raw_center_involved_boundary_edge_slots",
     report.rawCenterInvolvedBoundaryEdgeSlots);
  Profile::setMetric
    ("surface_provenance_raw_center_involved_nonmanifold_edge_slots",
     report.rawCenterInvolvedNonManifoldEdgeSlots);
  Profile::setMetric
    ("surface_provenance_raw_center_involved_welded_spread_edges",
     report.rawCenterInvolvedWeldedSpreadEdges);
  Profile::setMetric
    ("surface_provenance_raw_center_involved_welded_spread_edge_slots",
     report.rawCenterInvolvedWeldedSpreadEdgeSlots);
  Profile::setMetric
    ("surface_provenance_raw_center_involved_welded_spread_max_alternate_slots",
     report.rawCenterInvolvedWeldedSpreadMaxAlternateSlots);
  Profile::setMetric("surface_provenance_raw_grid_vertex_unique_keys",
                     report.rawGridVertexUniqueKeys);
  Profile::setMetric("surface_provenance_raw_grid_vertex_welded_spread_keys",
                     report.rawGridVertexWeldedSpreadKeys);
  Profile::setMetric
    ("surface_provenance_raw_grid_vertex_welded_spread_observations",
     report.rawGridVertexWeldedSpreadObservations);
  Profile::setMetric
    ("surface_provenance_raw_grid_vertex_welded_spread_max_alternate_observations",
     report.rawGridVertexWeldedSpreadMaxAlternateObservations);
  Profile::setMetric("surface_provenance_raw_center_vertex_unique_keys",
                     report.rawCenterVertexUniqueKeys);
  Profile::setMetric("surface_provenance_raw_center_vertex_welded_spread_keys",
                     report.rawCenterVertexWeldedSpreadKeys);
  Profile::setMetric
    ("surface_provenance_raw_center_vertex_welded_spread_observations",
     report.rawCenterVertexWeldedSpreadObservations);
  Profile::setMetric
    ("surface_provenance_raw_center_vertex_welded_spread_max_alternate_observations",
     report.rawCenterVertexWeldedSpreadMaxAlternateObservations);
  Profile::setMetric("surface_provenance_boundary_edges",
                     report.boundaryEdges);
  Profile::setMetric("surface_provenance_nonmanifold_edges",
                     report.nonManifoldEdges);
  Profile::setMetric("surface_provenance_misoriented_edges",
                     report.misorientedEdges);
  Profile::setMetric("surface_provenance_welded_unique_edges",
                     report.weldedUniqueEdges);
  Profile::setMetric("surface_provenance_welded_max_edge_incidence",
                     report.weldedMaxEdgeIncidence);
  Profile::setMetric("surface_provenance_welded_edges_incidence_1",
                     report.weldedEdgesIncidence1);
  Profile::setMetric("surface_provenance_welded_edges_incidence_2",
                     report.weldedEdgesIncidence2);
  Profile::setMetric("surface_provenance_welded_edges_incidence_3",
                     report.weldedEdgesIncidence3);
  Profile::setMetric("surface_provenance_welded_edges_incidence_4",
                     report.weldedEdgesIncidence4);
  Profile::setMetric("surface_provenance_welded_edges_incidence_5_plus",
                     report.weldedEdgesIncidence5Plus);
  Profile::setMetric("surface_provenance_welded_twin_edge_slots",
                     report.weldedTwinEdgeSlots);
  Profile::setMetric("surface_provenance_welded_boundary_edge_slots",
                     report.weldedBoundaryEdgeSlots);
  Profile::setMetric("surface_provenance_welded_nonmanifold_edge_slots",
                     report.weldedNonManifoldEdgeSlots);
  Profile::setMetric("surface_provenance_watertight",
                     report.watertight ? 1 : 0);
}
