/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>
  Copyright (C) 2026 davronthemighty

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

\******************************************************************************/

#include "SparseToolpathArtifacts.h"

#include <camotics/GeometrySafetyInternal.h>
#include <camotics/Grid.h>
#include <camotics/SHA256.h>
#include <camotics/Task.h>
#include <camotics/contour/ReductionEligibility.h>
#include <camotics/contour/Surface.h>
#include <camotics/contour/TriangleSurface.h>
#include <camotics/sim/Workpiece.h>

#include <cbang/Exception.h>
#include <cbang/json/Reader.h>
#include <cbang/json/Value.h>
#include <cbang/json/Writer.h>
#include <cbang/net/Base64.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_map>

using namespace std;
using namespace cb;
using namespace CAMotics;
using namespace CAMotics::SparseToolpath;


const char *SparseToolpath::ARTIFACT_VERSION = "3";
const char *SparseToolpath::TOOLPATH_ARTIFACT =
  "CAMotics sparse toolpath JSON";
const char *SparseToolpath::REGION_PLAN_ARTIFACT =
  "CAMotics sparse region plan JSON";
const char *SparseToolpath::OWNERSHIP_BOUNDARY_ARTIFACT =
  "CAMotics sparse ownership boundary JSON";
const char *SparseToolpath::REGION_SURFACE_ARTIFACT =
  "CAMotics sparse region surface artifact";
const char *SparseToolpath::STITCHED_SURFACE_ARTIFACT =
  "CAMotics sparse stitched surface artifact";


namespace {
  ReductionLockedVertex readLockedVertex(const JSON::Value &value) {
    ReductionLockedVertex vertex;
    vertex.x = (int64_t)value.getNumber("x", 0);
    vertex.y = (int64_t)value.getNumber("y", 0);
    vertex.z = (int64_t)value.getNumber("z", 0);
    return vertex;
  }


  void readStitchedReductionEligibility
  (const JSON::Value &root, const string &filename,
   const ArtifactContract &contract, Simulation &sim) {
    if (!root.hasDict("reduction-eligibility"))
      THROW("Sparse stitched artifact '" << filename
            << "' is missing reduction eligibility metadata.");

    TriangleSurface *surface =
      dynamic_cast<TriangleSurface *>(sim.surface.get());
    if (!surface)
      THROW("Sparse stitched artifact '" << filename
            << "' does not contain a triangle surface.");

    const JSON::Value &value = root.getDict("reduction-eligibility");
    if (value.getString("metadata-version", "") != "1")
      THROW("Sparse stitched artifact '" << filename
            << "' has an unsupported reduction eligibility version.");
    if (value.getString("lineage-region-surface-hash", "") !=
        contract.regionSurfaceHash)
      THROW("Sparse stitched artifact '" << filename
            << "' reduction eligibility lineage does not match its contract.");

    ReductionEligibility eligibility;
    eligibility.quantizationTolerance =
      value.getNumber("quantization-tolerance", 0);
    eligibility.bindingHash = value.getString("binding-hash", "");

    if (!value.hasList("triangle-origins"))
      THROW("Sparse stitched artifact '" << filename
            << "' is missing triangle origins.");
    const JSON::Value &origins = value.getList("triangle-origins");
    eligibility.triangleOrigins.reserve(origins.size());
    for (unsigned i = 0; i < origins.size(); i++)
      eligibility.triangleOrigins.push_back((uint8_t)origins.getNumber(i));

    if (!value.hasList("locked-seam-vertices") ||
        !value.hasList("locked-seam-edges"))
      THROW("Sparse stitched artifact '" << filename
            << "' is missing locked seam constraints.");
    const JSON::Value &vertices = value.getList("locked-seam-vertices");
    for (unsigned i = 0; i < vertices.size(); i++)
      eligibility.lockedSeamVertices.push_back
        (readLockedVertex(vertices.getDict(i)));
    const JSON::Value &edges = value.getList("locked-seam-edges");
    for (unsigned i = 0; i < edges.size(); i++) {
      const JSON::Value &edge = edges.getDict(i);
      if (!edge.hasDict("a") || !edge.hasDict("b"))
        THROW("Sparse stitched artifact '" << filename
              << "' has an incomplete locked seam edge.");
      eligibility.lockedSeamEdges.push_back
        (ReductionLockedEdge(readLockedVertex(edge.getDict("a")),
                             readLockedVertex(edge.getDict("b"))));
    }

    uint64_t triangleCount = value.getNumber("triangle-count", 0);
    if (triangleCount != surface->getTriangleCount() ||
        value.getNumber("mc-reducible-triangles", 0) !=
        eligibility.count(REDUCTION_MC_REDUCIBLE) ||
        value.getNumber("mc-seam-locked-triangles", 0) !=
        eligibility.count(REDUCTION_MC_SEAM_LOCKED) ||
        value.getNumber("analytic-locked-triangles", 0) !=
        eligibility.count(REDUCTION_ANALYTIC_LOCKED) ||
        value.getNumber("unknown-locked-triangles", 0) !=
        eligibility.count(REDUCTION_UNKNOWN_LOCKED) ||
        !eligibility.validFor(surface->getVertices()))
      THROW("Sparse stitched artifact '" << filename
            << "' has invalid or stale reduction eligibility metadata.");

    surface->markSparseAcceptedSurface();
    surface->setReductionEligibility(eligibility);
  }


  string hashJSON(const function<void(JSON::Sink &)> &write) {
    ostringstream stream;
    {
      JSON::Writer writer(stream);
      write(writer);
    }

    SHA256 sha256;
    sha256.update(stream.str());
    return Base64().encode(sha256.finalize());
  }


  string hashSimulation(const Simulation &sim) {
    ostringstream stream;
    {
      JSON::Writer writer(stream);
      sim.write(writer);
    }
    SmartPointer<JSON::Value> value = JSON::Reader::parse(stream.str());
    Simulation normalized
      (0, 0, 0, Workpiece(), 0, numeric_limits<double>::max(),
       RenderMode(), 1);
    normalized.read(*value);
    return normalized.computeHash();
  }


  ArtifactContract readContract(const JSON::Value &root,
                                const string &filename,
                                const string &kind) {
    if (!root.hasDict("contract"))
      THROW("Sparse artifact '" << filename << "' is missing contract data.");

    const JSON::Value &value = root.getDict("contract");
    ArtifactContract contract;
    contract.simulationHash = root.getString("simulation-hash", "");
    contract.inputHash = value.getString("input-hash", "");
    contract.toolpathHash = value.getString("toolpath-hash", "");
    contract.regionPlanHash = value.getString("region-plan-hash", "");
    contract.ownershipBoundaryHash =
      value.getString("ownership-boundary-hash", "");
    contract.regionSurfaceHash =
      value.getString("region-surface-hash", "");

    if (contract.simulationHash.empty() || contract.inputHash.empty() ||
        contract.toolpathHash.empty())
      THROW("Sparse artifact '" << filename
            << "' has an incomplete base contract.");
    if (kind != TOOLPATH_ARTIFACT && contract.regionPlanHash.empty())
      THROW("Sparse artifact '" << filename
            << "' is missing region-plan-hash.");
    if (kind == OWNERSHIP_BOUNDARY_ARTIFACT &&
        contract.ownershipBoundaryHash.empty())
      THROW("Sparse artifact '" << filename
            << "' is missing ownership-boundary-hash.");
    if (kind == STITCHED_SURFACE_ARTIFACT &&
        contract.regionSurfaceHash.empty())
      THROW("Sparse artifact '" << filename
            << "' is missing region-surface-hash.");

    return contract;
  }


  struct QuantizedVertexKey {
    int64_t x = 0;
    int64_t y = 0;
    int64_t z = 0;

    bool operator==(const QuantizedVertexKey &o) const {
      return x == o.x && y == o.y && z == o.z;
    }
  };


  bool operator<(const QuantizedVertexKey &a, const QuantizedVertexKey &b) {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    return a.z < b.z;
  }


  struct QuantizedVertexKeyHash {
    size_t operator()(const QuantizedVertexKey &key) const {
      uint64_t h = 1469598103934665603ULL;
      auto mix = [&] (uint64_t value) {
        h ^= value;
        h *= 1099511628211ULL;
      };

      mix((uint64_t)key.x);
      mix((uint64_t)key.y);
      mix((uint64_t)key.z);
      return (size_t)h;
    }
  };


  struct QuantizedEdgeKey {
    QuantizedVertexKey a;
    QuantizedVertexKey b;

    QuantizedEdgeKey() {}
    QuantizedEdgeKey(const QuantizedVertexKey &a,
                     const QuantizedVertexKey &b) {
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
      seed ^= hash(key.b) + 0x9e3779b97f4a7c15ULL +
        (seed << 6) + (seed >> 2);
      return seed;
    }
  };


  struct QuantizedTriangleKey {
    array<QuantizedVertexKey, 3> vertices;

    QuantizedTriangleKey(const array<QuantizedVertexKey, 3> &vertices) :
      vertices(vertices) {
      sort(this->vertices.begin(), this->vertices.end());
    }

    bool operator==(const QuantizedTriangleKey &o) const {
      return vertices == o.vertices;
    }
  };


  struct QuantizedTriangleKeyHash {
    size_t operator()(const QuantizedTriangleKey &key) const {
      QuantizedVertexKeyHash hash;
      size_t seed = 0;
      for (const auto &vertex: key.vertices)
        seed ^= hash(vertex) + 0x9e3779b97f4a7c15ULL +
          (seed << 6) + (seed >> 2);
      return seed;
    }
  };


  struct EdgeUse {
    uint64_t count = 0;
    uint64_t forwardCount = 0;
  };


  bool isForwardEdge(const QuantizedVertexKey &a,
                     const QuantizedVertexKey &b,
                     const QuantizedEdgeKey &key) {
    return a == key.a && b == key.b;
  }


  bool hasFiniteVertex(const vector<float> &vertices, uint64_t offset) {
    return isfinite(vertices[offset + 0]) && isfinite(vertices[offset + 1]) &&
      isfinite(vertices[offset + 2]);
  }


  QuantizedVertexKey getQuantizedVertexKey(const vector<float> &vertices,
                                           uint64_t offset,
                                           double tolerance) {
    QuantizedVertexKey key;
    if (!Internal::quantizeCoordinate
        (vertices[offset + 0], tolerance, key.x) ||
        !Internal::quantizeCoordinate
        (vertices[offset + 1], tolerance, key.y) ||
        !Internal::quantizeCoordinate
        (vertices[offset + 2], tolerance, key.z))
      THROW("Surface vertex is outside the topology quantization range.");
    return key;
  }


  double triangleAreaSquared(const vector<float> &vertices, uint64_t offset) {
    double ax = vertices[offset + 0];
    double ay = vertices[offset + 1];
    double az = vertices[offset + 2];
    double bx = vertices[offset + 3];
    double by = vertices[offset + 4];
    double bz = vertices[offset + 5];
    double cx = vertices[offset + 6];
    double cy = vertices[offset + 7];
    double cz = vertices[offset + 8];

    double ux = bx - ax;
    double uy = by - ay;
    double uz = bz - az;
    double vx = cx - ax;
    double vy = cy - ay;
    double vz = cz - az;
    double nx = uy * vz - uz * vy;
    double ny = uz * vx - ux * vz;
    double nz = ux * vy - uy * vx;
    return nx * nx + ny * ny + nz * nz;
  }


  bool isDegenerateTriangle(const vector<float> &vertices, uint64_t offset,
                            const array<QuantizedVertexKey, 3> &keys,
                            double tolerance) {
    if (keys[0] == keys[1] || keys[1] == keys[2] || keys[2] == keys[0])
      return true;

    double areaTolerance = tolerance * tolerance;
    return triangleAreaSquared(vertices, offset) <=
      areaTolerance * areaTolerance;
  }


  SmartPointer<JSON::Value> readArtifactRoot(const string &filename,
                                             const string &expectedKind) {
    SmartPointer<JSON::Value> root = JSON::Reader::parseFile(filename);

    string kind = root->getString("artifact-kind", "");
    if (!expectedKind.empty() && kind != expectedKind)
      THROW("Expected " << expectedKind << " artifact in '" << filename
            << "' but found '" << kind << "'.");

    if (root->getString("artifact-version", "") !=
        SparseToolpath::ARTIFACT_VERSION)
      THROW("Unsupported sparse artifact version in '" << filename << "'.");

    return root;
  }


  void writeBounds(JSON::Sink &sink, const string &name,
                   const Rectangle3D &bounds) {
    sink.beginInsert(name);
    bounds.write(sink);
  }


  void readRegionList(const JSON::Value &value, const string &name,
                      vector<RegionPlanRegion> &regions) {
    regions.clear();
    if (!value.hasList(name)) return;

    auto &list = value.getList(name);
    for (unsigned i = 0; i < list.size(); i++) {
      RegionPlanRegion region;
      region.read(list.getDict(i));
      regions.push_back(region);
    }
  }


  void writeRegionList(JSON::Sink &sink, const string &name,
                       const vector<RegionPlanRegion> &regions) {
    sink.insertList(name);
    for (const auto &region: regions) {
      sink.appendDict(true);
      region.write(sink);
      sink.endDict();
    }
    sink.endList();
  }


  void readBoundaryPointList(const JSON::Value &value, const string &name,
                             vector<OwnershipBoundaryPoint> &points) {
    points.clear();
    if (!value.hasList(name)) return;

    auto &list = value.getList(name);
    for (unsigned i = 0; i < list.size(); i++) {
      OwnershipBoundaryPoint point;
      point.read(list.getDict(i));
      points.push_back(point);
    }
  }


  void writeBoundaryPointList
    (JSON::Sink &sink, const string &name,
     const vector<OwnershipBoundaryPoint> &points) {
    sink.insertList(name);
    for (const auto &point: points) {
      sink.appendDict(true);
      point.write(sink);
      sink.endDict();
    }
    sink.endList();
  }


  void readBoundaryEdgeList(const JSON::Value &value, const string &name,
                            vector<OwnershipBoundaryEdge> &edges) {
    edges.clear();
    if (!value.hasList(name)) return;

    auto &list = value.getList(name);
    for (unsigned i = 0; i < list.size(); i++) {
      OwnershipBoundaryEdge edge;
      edge.read(list.getDict(i));
      edges.push_back(edge);
    }
  }


  void writeBoundaryEdgeList
    (JSON::Sink &sink, const string &name,
     const vector<OwnershipBoundaryEdge> &edges) {
    sink.insertList(name);
    for (const auto &edge: edges) {
      sink.appendDict(true);
      edge.write(sink);
      sink.endDict();
    }
    sink.endList();
  }


  void readBoundaryLoopList(const JSON::Value &value, const string &name,
                            vector<OwnershipBoundaryLoop> &loops) {
    loops.clear();
    if (!value.hasList(name)) return;

    auto &list = value.getList(name);
    for (unsigned i = 0; i < list.size(); i++) {
      OwnershipBoundaryLoop loop;
      loop.read(list.getDict(i));
      loops.push_back(loop);
    }
  }


  void writeBoundaryLoopList
    (JSON::Sink &sink, const string &name,
     const vector<OwnershipBoundaryLoop> &loops) {
    sink.insertList(name);
    for (const auto &loop: loops) {
      sink.appendDict(true);
      loop.write(sink);
      sink.endDict();
    }
    sink.endList();
  }


  void addTopologyEdge
    (unordered_map<QuantizedEdgeKey, EdgeUse, QuantizedEdgeKeyHash> &edges,
     const QuantizedVertexKey &a, const QuantizedVertexKey &b) {
    QuantizedEdgeKey key(a, b);
    EdgeUse &use = edges[key];
    use.count++;
    if (isForwardEdge(a, b, key)) use.forwardCount++;
  }
}


void RegionPlanRegion::read(const JSON::Value &value) {
  id = value.getString("id", id);
  ownership = value.getString("ownership", ownership);
  role = value.getString("role", role);
  tileX = value.getNumber("tile-x", tileX);
  tileY = value.getNumber("tile-y", tileY);
  tileWidth = value.getNumber("tile-width", tileWidth);
  tileHeight = value.getNumber("tile-height", tileHeight);
  activeDepth = value.getNumber("active-depth", activeDepth);
  estimatedCells = value.getNumber("estimated-cells", estimatedCells);
  if (value.hasDict("bounds")) bounds.read(value.getDict("bounds"));
}


void RegionPlanRegion::write(JSON::Sink &sink) const {
  sink.insert("id", id);
  sink.insert("ownership", ownership);
  sink.insert("role", role);
  sink.insert("tile-x", tileX);
  sink.insert("tile-y", tileY);
  sink.insert("tile-width", tileWidth);
  sink.insert("tile-height", tileHeight);
  sink.insert("active-depth", activeDepth);
  sink.insert("estimated-cells", estimatedCells);
  writeBounds(sink, "bounds", bounds);
}


void RegionPlan::read(const JSON::Value &value) {
  planner = value.getString("planner", planner);
  ownership = value.getString("ownership", ownership);
  xyBins = value.getNumber("xy-bins", xyBins);
  haloCells = value.getNumber("halo-cells", haloCells);
  halo = value.getNumber("halo", halo);
  fullCells = value.getNumber("full-grid-cells-est", fullCells);
  activeCells = value.getNumber("active-cells-est", activeCells);
  renderCells = value.getNumber("render-cells-est", renderCells);
  skippedCells = value.getNumber("skipped-cells-est", skippedCells);
  toolSweepBBoxes =
    value.getNumber("tool-sweep-bboxes", toolSweepBBoxes);
  bboxTileRefs = value.getNumber("bbox-tile-refs", bboxTileRefs);
  toolpathFilteredTileRefs =
    value.getNumber("toolpath-filtered-tile-refs", toolpathFilteredTileRefs);
  targetRegionCells =
    value.getNumber("target-region-cells", targetRegionCells);
  adaptiveLeafCount =
    value.getNumber("adaptive-leaf-count", adaptiveLeafCount);
  adaptiveActiveLeafCount =
    value.getNumber("adaptive-active-leaf-count", adaptiveActiveLeafCount);
  adaptiveSplitCount =
    value.getNumber("adaptive-split-count", adaptiveSplitCount);
  adaptiveOwnershipSplitCount = value.getNumber
    ("adaptive-ownership-split-count", adaptiveOwnershipSplitCount);
  adaptiveDepthSplitCount =
    value.getNumber("adaptive-depth-split-count", adaptiveDepthSplitCount);
  adaptiveDensitySplitCount = value.getNumber
    ("adaptive-density-split-count", adaptiveDensitySplitCount);
  adaptiveTargetSplitCount =
    value.getNumber("adaptive-target-split-count", adaptiveTargetSplitCount);
  adaptiveMaxLeafCells =
    value.getNumber("adaptive-max-leaf-cells", adaptiveMaxLeafCells);
  adaptiveTargetExceededLeaves = value.getNumber
    ("adaptive-target-exceeded-leaves", adaptiveTargetExceededLeaves);
  if (value.hasDict("stock-bounds"))
    stockBounds.read(value.getDict("stock-bounds"));
  if (value.hasDict("swept-bounds"))
    sweptBounds.read(value.getDict("swept-bounds"));
  readRegionList(value, "active-region-list", activeRegions);
  readRegionList(value, "render-region-list", renderRegions);
  if (renderRegions.empty()) renderRegions = activeRegions;
  readRegionList(value, "analytic-region-list", analyticRegions);
}


void RegionPlan::write(JSON::Sink &sink) const {
  sink.beginDict();
  sink.insert("planner", planner);
  sink.insert("ownership", ownership);
  sink.insert("active-regions", (uint64_t)activeRegions.size());
  sink.insert("render-regions", (uint64_t)renderRegions.size());
  sink.insert("analytic-regions", (uint64_t)analyticRegions.size());
  sink.insert("interior-islands", 0);
  sink.insert("xy-bins", xyBins);
  sink.insert("halo-cells", haloCells);
  sink.insert("halo", halo);
  sink.insert("full-grid-cells-est", fullCells);
  sink.insert("active-cells-est", activeCells);
  sink.insert("render-cells-est", renderCells);
  sink.insert("skipped-cells-est", skippedCells);
  sink.insert("tool-sweep-bboxes", toolSweepBBoxes);
  sink.insert("bbox-tile-refs", bboxTileRefs);
  sink.insert("toolpath-filtered-tile-refs", toolpathFilteredTileRefs);
  sink.insert("target-region-cells", targetRegionCells);
  sink.insert("adaptive-leaf-count", adaptiveLeafCount);
  sink.insert("adaptive-active-leaf-count", adaptiveActiveLeafCount);
  sink.insert("adaptive-split-count", adaptiveSplitCount);
  sink.insert("adaptive-ownership-split-count", adaptiveOwnershipSplitCount);
  sink.insert("adaptive-depth-split-count", adaptiveDepthSplitCount);
  sink.insert("adaptive-density-split-count", adaptiveDensitySplitCount);
  sink.insert("adaptive-target-split-count", adaptiveTargetSplitCount);
  sink.insert("adaptive-max-leaf-cells", adaptiveMaxLeafCells);
  sink.insert("adaptive-target-exceeded-leaves",
              adaptiveTargetExceededLeaves);
  writeBounds(sink, "stock-bounds", stockBounds);
  writeBounds(sink, "swept-bounds", sweptBounds);
  writeRegionList(sink, "active-region-list", activeRegions);
  writeRegionList(sink, "render-region-list", renderRegions);
  writeRegionList(sink, "analytic-region-list", analyticRegions);
  sink.endDict();
}


void OwnershipBoundaryPoint::read(const JSON::Value &value) {
  x = value.getNumber("x", x);
  y = value.getNumber("y", y);
  z = value.getNumber("z", z);
  gridX = value.getNumber("grid-x", gridX);
  gridY = value.getNumber("grid-y", gridY);
}


void OwnershipBoundaryPoint::write(JSON::Sink &sink) const {
  sink.insert("x", x);
  sink.insert("y", y);
  sink.insert("z", z);
  sink.insert("grid-x", gridX);
  sink.insert("grid-y", gridY);
}


void OwnershipBoundaryEdge::read(const JSON::Value &value) {
  analyticRegionID =
    value.getString("analytic-region-id", analyticRegionID);
  adjacentOwnership =
    value.getString("adjacent-ownership", adjacentOwnership);
  adjacentRegionID =
    value.getString("adjacent-region-id", adjacentRegionID);
  side = value.getString("side", side);
}


void OwnershipBoundaryEdge::write(JSON::Sink &sink) const {
  sink.insert("analytic-region-id", analyticRegionID);
  sink.insert("adjacent-ownership", adjacentOwnership);
  sink.insert("adjacent-region-id", adjacentRegionID);
  sink.insert("side", side);
}


void OwnershipBoundaryLoop::read(const JSON::Value &value) {
  id = value.getString("id", id);
  plane = value.getString("plane", plane);
  role = value.getString("role", role);
  normalX = value.getNumber("normal-x", normalX);
  normalY = value.getNumber("normal-y", normalY);
  normalZ = value.getNumber("normal-z", normalZ);
  coordinate = value.getNumber("coordinate", coordinate);
  signedArea = value.getNumber("signed-area", signedArea);
  rawVertices = value.getNumber("raw-vertices", rawVertices);
  closed = value.getNumber("closed", closed);
  touchesStockBorder =
    value.getNumber("touches-stock-border", touchesStockBorder);
  readBoundaryPointList(value, "vertices", vertices);
  readBoundaryEdgeList(value, "edges", edges);
}


void OwnershipBoundaryLoop::write(JSON::Sink &sink) const {
  sink.insert("id", id);
  sink.insert("plane", plane);
  sink.insert("role", role);
  sink.insert("normal-x", normalX);
  sink.insert("normal-y", normalY);
  sink.insert("normal-z", normalZ);
  sink.insert("coordinate", coordinate);
  sink.insert("signed-area", signedArea);
  sink.insert("raw-vertices", rawVertices);
  sink.insert("closed", closed);
  sink.insert("touches-stock-border", touchesStockBorder);
  writeBoundaryPointList(sink, "vertices", vertices);
  writeBoundaryEdgeList(sink, "edges", edges);
}


void OwnershipBoundaryPlan::read(const JSON::Value &value) {
  planner = value.getString("planner", planner);
  ownership = value.getString("ownership", ownership);
  xyBins = value.getNumber("xy-bins", xyBins);
  haloCells = value.getNumber("halo-cells", haloCells);
  halo = value.getNumber("halo", halo);
  activeTiles = value.getNumber("active-tiles", activeTiles);
  analyticTiles = value.getNumber("analytic-tiles", analyticTiles);
  boundaryEdges = value.getNumber("boundary-edges", boundaryEdges);
  openLoops = value.getNumber("open-loops", openLoops);
  ambiguousVertices =
    value.getNumber("ambiguous-vertices", ambiguousVertices);
  rawVertices = value.getNumber("raw-vertices", rawVertices);
  contractedVertices =
    value.getNumber("contracted-vertices", contractedVertices);
  if (value.hasDict("stock-bounds"))
    stockBounds.read(value.getDict("stock-bounds"));
  readBoundaryLoopList(value, "loops", loops);
}


void OwnershipBoundaryPlan::write(JSON::Sink &sink) const {
  sink.beginDict();
  sink.insert("planner", planner);
  sink.insert("ownership", ownership);
  sink.insert("xy-bins", xyBins);
  sink.insert("halo-cells", haloCells);
  sink.insert("halo", halo);
  sink.insert("loop-count", (uint64_t)loops.size());
  sink.insert("active-tiles", activeTiles);
  sink.insert("analytic-tiles", analyticTiles);
  sink.insert("boundary-edges", boundaryEdges);
  sink.insert("open-loops", openLoops);
  sink.insert("ambiguous-vertices", ambiguousVertices);
  sink.insert("raw-vertices", rawVertices);
  sink.insert("contracted-vertices", contractedVertices);
  writeBounds(sink, "stock-bounds", stockBounds);
  writeBoundaryLoopList(sink, "loops", loops);
  sink.endDict();
}


void ArtifactContract::write(JSON::Sink &sink) const {
  sink.beginDict();
  sink.insert("input-hash", inputHash);
  sink.insert("toolpath-hash", toolpathHash);
  if (!regionPlanHash.empty())
    sink.insert("region-plan-hash", regionPlanHash);
  if (!ownershipBoundaryHash.empty())
    sink.insert("ownership-boundary-hash", ownershipBoundaryHash);
  if (!regionSurfaceHash.empty())
    sink.insert("region-surface-hash", regionSurfaceHash);
  sink.endDict();
}


string SparseToolpath::computeToolpathHash(const Simulation &sim) {
  Simulation toolpathSim
    (sim.path, sim.planConf, 0, sim.workpiece, sim.resolution, sim.time,
     sim.mode, sim.threads, sim.toolSweepXYBins, sim.toolSweepXYZBins,
     sim.adaptiveZSlabMetrics, sim.adaptiveZRender,
     sim.adaptiveZSlabHeight, sim.adaptiveZInitialDepth,
     sim.adaptiveZMargin, sim.adaptiveZRegionBins,
     sim.adaptiveZRegionRender);
  return hashSimulation(toolpathSim);
}


string SparseToolpath::computeRegionPlanHash(const RegionPlan &plan) {
  return hashJSON([&] (JSON::Sink &sink) {plan.write(sink);});
}


string SparseToolpath::computeOwnershipBoundaryHash
(const OwnershipBoundaryPlan &plan) {
  return hashJSON([&] (JSON::Sink &sink) {plan.write(sink);});
}


bool SurfaceTopologyReport::accepted() const {
  return !boundaryEdges && !nonManifoldEdges && !misorientedEdges &&
    !degenerateTriangles && !duplicateTriangles;
}


void SurfaceTopologyReport::write(JSON::Sink &sink) const {
  write(sink, accepted());
}


void SurfaceTopologyReport::write(JSON::Sink &sink,
                                  bool acceptedValue) const {
  sink.insert("topology-triangles", triangles);
  sink.insert("unique-edges", uniqueEdges);
  sink.insert("boundary-edges", boundaryEdges);
  sink.insert("nonmanifold-edges", nonManifoldEdges);
  sink.insert("misoriented-edges", misorientedEdges);
  sink.insert("degenerate-triangles", degenerateTriangles);
  sink.insert("duplicate-triangles", duplicateTriangles);
  sink.insert("max-edge-incidence", maxEdgeIncidence);
  sink.insertBoolean("accepted", acceptedValue);
}


SmartPointer<Simulation>
SparseToolpath::readSimulationArtifact(const string &filename,
                                       const string &expectedKind) {
  SmartPointer<JSON::Value> root = readArtifactRoot(filename, expectedKind);

  if (!root->hasDict("simulation"))
    THROW("Sparse artifact '" << filename << "' is missing simulation data.");

  SmartPointer<Simulation> sim =
    new Simulation(0, 0, 0, Workpiece(), 0,
                   numeric_limits<double>::max(), RenderMode(), 1);
  sim->read(root->getDict("simulation"));
  ArtifactContract contract = readContract(*root, filename, expectedKind);
  if (contract.simulationHash != sim->computeHash())
    THROW("Sparse artifact '" << filename
          << "' simulation-hash does not match embedded simulation.");
  if (expectedKind == TOOLPATH_ARTIFACT &&
      contract.toolpathHash != contract.simulationHash)
    THROW("Sparse artifact '" << filename
          << "' toolpath-hash does not match toolpath simulation.");
  if (expectedKind == STITCHED_SURFACE_ARTIFACT)
    readStitchedReductionEligibility(*root, filename, contract, *sim);
  return sim;
}


ArtifactContract SparseToolpath::readArtifactContract
(const string &filename, const string &expectedKind) {
  SmartPointer<JSON::Value> root = readArtifactRoot(filename, expectedKind);
  return readContract(*root, filename, expectedKind);
}


RegionPlan SparseToolpath::readRegionPlanArtifact
(const string &filename, SmartPointer<Simulation> &sim) {
  SmartPointer<JSON::Value> root =
    readArtifactRoot(filename, REGION_PLAN_ARTIFACT);

  if (!root->hasDict("region-plan"))
    THROW("Sparse region plan artifact '" << filename
          << "' is missing region-plan data.");

  sim = readSimulationArtifact(filename, REGION_PLAN_ARTIFACT);
  RegionPlan plan;
  plan.read(root->getDict("region-plan"));
  ArtifactContract contract = readContract(*root, filename,
                                           REGION_PLAN_ARTIFACT);
  if (contract.regionPlanHash != computeRegionPlanHash(plan))
    THROW("Sparse region plan artifact '" << filename
          << "' region-plan-hash does not match region-plan data.");
  return plan;
}


OwnershipBoundaryPlan SparseToolpath::readOwnershipBoundaryArtifact
(const string &filename, SmartPointer<Simulation> &sim) {
  SmartPointer<JSON::Value> root =
    readArtifactRoot(filename, OWNERSHIP_BOUNDARY_ARTIFACT);

  if (!root->hasDict("ownership-boundary"))
    THROW("Sparse ownership boundary artifact '" << filename
          << "' is missing ownership-boundary data.");

  sim = readSimulationArtifact(filename, OWNERSHIP_BOUNDARY_ARTIFACT);
  OwnershipBoundaryPlan plan;
  plan.read(root->getDict("ownership-boundary"));
  ArtifactContract contract = readContract(*root, filename,
                                           OWNERSHIP_BOUNDARY_ARTIFACT);
  if (contract.ownershipBoundaryHash !=
      computeOwnershipBoundaryHash(plan))
    THROW("Sparse ownership boundary artifact '" << filename
          << "' ownership-boundary-hash does not match boundary data.");
  return plan;
}


void SparseToolpath::writeSimulationArtifact
(ostream &stream, const string &kind, const Simulation &sim,
 const ArtifactContract &contract,
 const ExtraArtifactWriter &extraWriter) {
  JSON::Writer writer(stream, 2, false);

  writer.beginDict();
  writer.insert("artifact-kind", kind);
  writer.insert("artifact-version", ARTIFACT_VERSION);
  writer.insert("simulation-hash", hashSimulation(sim));
  writer.beginInsert("contract");
  contract.write(writer);

  writer.beginInsert("simulation");
  sim.write(writer);

  if (extraWriter) extraWriter(writer);

  writer.endDict();
}


uint64_t SparseToolpath::estimateGridCells(const Rectangle3D &bounds,
                                           double resolution, bool grow) {
  if (bounds == Rectangle3D() || !Internal::finiteBounds(bounds) ||
      !isfinite(resolution) || resolution <= 0)
    return 0;

  Rectangle3D measured = grow ? bounds.grow(resolution * 0.9) : bounds;
  if (!Internal::finiteBounds(measured)) return 0;
  Vector3D dimensions = measured.getDimensions();
  unsigned steps[3] = {};
  if (!Internal::ceilToUnsigned(dimensions.x(), resolution, steps[0]) ||
      !Internal::ceilToUnsigned(dimensions.y(), resolution, steps[1]) ||
      !Internal::ceilToUnsigned(dimensions.z(), resolution, steps[2]))
    return numeric_limits<uint64_t>::max();

  uint64_t xy = (uint64_t)steps[0] * steps[1];
  if (steps[2] && numeric_limits<uint64_t>::max() / steps[2] < xy)
    return numeric_limits<uint64_t>::max();
  return xy * steps[2];
}


SurfaceTopologyReport SparseToolpath::validateSurfaceTopology
(const Surface &surface, double tolerance, Task *task) {
  if (!isfinite(tolerance) || tolerance <= 0) tolerance = 1e-6;

  SurfaceTopologyReport report;
  unordered_map<QuantizedEdgeKey, EdgeUse, QuantizedEdgeKeyHash> edges;
  unordered_map<QuantizedTriangleKey, uint64_t,
                QuantizedTriangleKeyHash> triangles;
  edges.reserve((size_t)surface.getTriangleCount() * 3 / 2);
  triangles.reserve((size_t)surface.getTriangleCount());
  uint64_t totalTriangles = surface.getTriangleCount();
  uint64_t processedTriangles = 0;
  bool cancelled = false;

  surface.getVertices([&] (const vector<float> &vertices,
                           const vector<float> &) {
    if (cancelled) return;
    uint64_t triangleCount = vertices.size() / 9;
    for (uint64_t tri = 0; tri < triangleCount; tri++) {
      if (task && !(tri & 8191) &&
          !task->update(totalTriangles ?
                        0.5 * processedTriangles / totalTriangles : 0.5)) {
        cancelled = true;
        return;
      }
      uint64_t offset = tri * 9;
      report.triangles++;
      processedTriangles++;

      if (!hasFiniteVertex(vertices, offset) ||
          !hasFiniteVertex(vertices, offset + 3) ||
          !hasFiniteVertex(vertices, offset + 6)) {
        report.degenerateTriangles++;
        continue;
      }

      array<QuantizedVertexKey, 3> keys = {{
        getQuantizedVertexKey(vertices, offset + 0, tolerance),
        getQuantizedVertexKey(vertices, offset + 3, tolerance),
        getQuantizedVertexKey(vertices, offset + 6, tolerance),
      }};

      if (isDegenerateTriangle(vertices, offset, keys, tolerance)) {
        report.degenerateTriangles++;
        continue;
      }

      uint64_t &faceUses = triangles[QuantizedTriangleKey(keys)];
      if (faceUses++) report.duplicateTriangles++;

      addTopologyEdge(edges, keys[0], keys[1]);
      addTopologyEdge(edges, keys[1], keys[2]);
      addTopologyEdge(edges, keys[2], keys[0]);
    }
  });

  if (cancelled) return report;

  report.uniqueEdges = edges.size();
  uint64_t processedEdges = 0;
  for (const auto &entry: edges) {
    if (task && !(processedEdges & 8191) &&
        !task->update(edges.empty() ? 1 :
                      0.5 + 0.5 * processedEdges / edges.size()))
      return report;
    processedEdges++;
    uint64_t count = entry.second.count;
    report.maxEdgeIncidence = max(report.maxEdgeIncidence, count);

    if (count == 1) report.boundaryEdges++;
    else if (2 < count) report.nonManifoldEdges++;
    else if (count == 2 &&
             (entry.second.forwardCount == 0 ||
              entry.second.forwardCount == 2))
      report.misorientedEdges++;
  }

  return report;
}
