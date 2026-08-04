/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>

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

#include "TriangleSurface.h"

#include "ContourProvenance.h"
#include "TriangleMesh.h"
#include "GridTree.h"

#include <cbang/Exception.h>
#include <cbang/log/Logger.h>

#include <camotics/Profile.h>
#include <camotics/Task.h>

#include <stl/Source.h>
#include <stl/Sink.h>

#include <limits>
#include <utility>

using namespace std;
using namespace cb;
using namespace CAMotics;


namespace {
  ContourProvenanceReport getLightweightProvenanceReport
    (const vector<ContourTriangleProvenance> &provenance,
     uint64_t expectedTriangles) {
    ContourProvenanceReport report;
    report.expectedTriangles = expectedTriangles;
    report.triangles = provenance.size();

    for (const auto &triangle: provenance)
      if (triangle.algorithm == ContourTriangleProvenance::UNKNOWN_ALGORITHM)
        report.unknownTriangles++;
      else report.completeTriangles++;

    return report;
  }
}


TriangleSurface::TriangleSurface(const GridTree &tree) {add(tree);}


TriangleSurface::TriangleSurface(STL::Source &source, Task *task) {
  read(source, task);
}


TriangleSurface::TriangleSurface(vector<SmartPointer<Surface> > &surfaces) {
  bool preserveProvenance =
    Profile::isEnabled() || shouldCaptureContourProvenance();
  if (preserveProvenance) {
    uint64_t provenanceTriangles = 0;
    for (unsigned i = 0; i < surfaces.size(); i++) {
      TriangleSurface *s = dynamic_cast<TriangleSurface *>(surfaces[i].get());
      if (!s || !s->hasContourProvenance()) {
        preserveProvenance = false;
        break;
      }
      uint64_t triangles = s->getTriangleCount();
      if (numeric_limits<uint64_t>::max() - provenanceTriangles <
          triangles) {
        preserveProvenance = false;
        break;
      }
      provenanceTriangles += triangles;
    }

    if (preserveProvenance &&
        provenanceTriangles <= contourProvenance.max_size())
      contourProvenance.reserve((size_t)provenanceTriangles);
    else preserveProvenance = false;
  }

  for (unsigned i = 0; i < surfaces.size(); i++) {
    TriangleSurface *s = dynamic_cast<TriangleSurface *>(surfaces[i].get());
    if (!s) THROW("Expected an TriangleSurface");

    // Copy surface data
    vertices.insert(vertices.end(), s->vertices.begin(), s->vertices.end());
    normals.insert(normals.end(), s->normals.begin(), s->normals.end());
    bounds.add(s->bounds);
    if (preserveProvenance)
      contourProvenance.insert
        (contourProvenance.end(), s->contourProvenance.begin(),
         s->contourProvenance.end());

    surfaces[i] = 0; // Free memory as we go
  }

  if (preserveProvenance) {
    const uint64_t triangleCount = vertices.size() / 9;
    ContourProvenanceReport report;
    if (Profile::isEnabled()) {
      Profile::Scope scope("surface_merge_analyze_contour_provenance");
      report = analyzeContourProvenance
        (contourProvenance, vertices, 0, triangleCount);
    } else
      report = getLightweightProvenanceReport
        (contourProvenance, triangleCount);
    if (Profile::isEnabled()) emitContourProvenanceMetrics(report);

    if (report.triangles == report.expectedTriangles) {
      contourProvenanceReport = report;
      contourProvenanceValid = true;
      if (shouldCaptureContourProvenance()) {
        Profile::Scope scope("surface_merge_build_contour_provenance_neighbors");
        contourProvenanceNeighborsValid = buildContourProvenanceNeighbors
          (contourProvenance, vertices, 0, report.expectedTriangles,
           contourProvenanceNeighbors, 1e-4, &contourProvenanceNeighborsRaw);
        if (!Profile::isEnabled())
          contourProvenanceReport.watertight =
            contourProvenanceNeighborsValid;
        if (!Profile::isEnabled() && contourProvenanceNeighborsValid)
          releaseContourProvenanceRecords();
      }

    } else clearContourProvenance();
  }
}


TriangleSurface::TriangleSurface(const TriangleSurface &o) :
  TriangleMesh(o), bounds(o.bounds),
  contourProvenance(o.contourProvenance),
  contourProvenanceNeighbors(o.contourProvenanceNeighbors),
  contourProvenanceReport(o.contourProvenanceReport),
  contourProvenanceValid(o.contourProvenanceValid),
  contourProvenanceNeighborsValid(o.contourProvenanceNeighborsValid),
  contourProvenanceNeighborsRaw(o.contourProvenanceNeighborsRaw),
  reductionEligibility(o.reductionEligibility),
  reductionEligibilityPresent(o.reductionEligibilityPresent),
  sparseAcceptedSurface(o.sparseAcceptedSurface) {}


void TriangleSurface::add(const Vector3F vertices[3]) {
  // Compute face normal
  Vector3F normal =
    (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);

  // Normalize
  double length = normal.length();
  if (length == 0) return; // Degenerate element, skip
  normal /= length;

  // Add it
  add(vertices, normal);
}


void TriangleSurface::add(const Vector3F vertices[3], const Vector3F &normal) {
  clearContourProvenance();
  clearReductionEligibility();

  for (unsigned i = 0; i < 3; i++) {
    bounds.add(vertices[i]);

    for (unsigned j = 0; j < 3; j++) {
      this->vertices.push_back(vertices[i][j]);
      normals.push_back(normal[j]);
    }
  }
}


void TriangleSurface::add(const GridTree &tree) {
  uint64_t startTriangles = vertices.size() / 9;
  uint64_t start = vertices.size();
  clearContourProvenance();
  clearReductionEligibility();

  if (Profile::isEnabled() || shouldCaptureContourProvenance()) {
    vector<ContourTriangleProvenance> provenance;
    if (tree.getCount() <= provenance.max_size())
      provenance.reserve((size_t)tree.getCount());
    {
      Profile::Scope scope("surface_gather_contour_provenance");
      tree.gather(vertices, normals, &provenance);
    }
    const uint64_t addedTriangles = vertices.size() / 9 - startTriangles;
    ContourProvenanceReport report;
    if (Profile::isEnabled()) {
      Profile::Scope scope("surface_analyze_contour_provenance");
      report = analyzeContourProvenance
        (provenance, vertices, start, addedTriangles);
    } else
      report = getLightweightProvenanceReport
        (provenance, addedTriangles);
    if (Profile::isEnabled()) emitContourProvenanceMetrics(report);

    if (!startTriangles && report.triangles == report.expectedTriangles) {
      contourProvenance = std::move(provenance);
      contourProvenanceReport = report;
      contourProvenanceValid = true;
      if (shouldCaptureContourProvenance()) {
        Profile::Scope scope("surface_build_contour_provenance_neighbors");
        contourProvenanceNeighborsValid = buildContourProvenanceNeighbors
          (contourProvenance, vertices, start, report.expectedTriangles,
           contourProvenanceNeighbors, 1e-4, &contourProvenanceNeighborsRaw);
        if (!Profile::isEnabled())
          contourProvenanceReport.watertight =
            contourProvenanceNeighborsValid;
        if (!Profile::isEnabled() && contourProvenanceNeighborsValid)
          releaseContourProvenanceRecords();
      }
    }

  } else tree.gather(vertices, normals);

  for (size_t i = start; i < vertices.size(); i += 3)
    bounds.add(Vector3F(vertices[i], vertices[i + 1], vertices[i + 2]));
}


void TriangleSurface::clear() {
  vertices.clear();
  normals.clear();
  clearContourProvenance();
  clearReductionEligibility();

  bounds = Rectangle3D();
}


void TriangleSurface::releaseContourProvenanceRecords() {
  vector<ContourTriangleProvenance>().swap(contourProvenance);
}


void TriangleSurface::clearContourProvenance() {
  contourProvenance.clear();
  contourProvenanceNeighbors.clear();
  contourProvenanceReport = ContourProvenanceReport();
  contourProvenanceValid = false;
  contourProvenanceNeighborsValid = false;
  contourProvenanceNeighborsRaw = false;
}


void TriangleSurface::clearReductionEligibility() {
  reductionEligibility = ReductionEligibility();
  reductionEligibilityPresent = false;
}


void TriangleSurface::setReductionEligibility
(const ReductionEligibility &eligibility) {
  reductionEligibility = eligibility;
  reductionEligibilityPresent = eligibility.validFor(vertices);
}


void TriangleSurface::replace(const vector<float> &vertices,
                              const vector<float> &normals) {
  this->vertices = vertices;
  this->normals = normals;
  clearContourProvenance();
  clearReductionEligibility();
  bounds = Rectangle3D();

  for (size_t i = 0; i + 2 < this->vertices.size(); i += 3)
    bounds.add(Vector3F(this->vertices[i], this->vertices[i + 1],
                        this->vertices[i + 2]));
}


void TriangleSurface::replace(vector<float> &&vertices,
                              vector<float> &&normals) {
  this->vertices = std::move(vertices);
  this->normals = std::move(normals);
  clearContourProvenance();
  clearReductionEligibility();
  bounds = Rectangle3D();

  for (size_t i = 0; i + 2 < this->vertices.size(); i += 3)
    bounds.add(Vector3F(this->vertices[i], this->vertices[i + 1],
                        this->vertices[i + 2]));
}


void TriangleSurface::swap(TriangleSurface &surface) {
  this->vertices.swap(surface.vertices);
  this->normals.swap(surface.normals);
  std::swap(bounds, surface.bounds);
  contourProvenance.swap(surface.contourProvenance);
  contourProvenanceNeighbors.swap(surface.contourProvenanceNeighbors);
  std::swap(contourProvenanceReport, surface.contourProvenanceReport);
  std::swap(contourProvenanceValid, surface.contourProvenanceValid);
  std::swap(contourProvenanceNeighborsValid,
            surface.contourProvenanceNeighborsValid);
  std::swap(contourProvenanceNeighborsRaw,
             surface.contourProvenanceNeighborsRaw);
  std::swap(reductionEligibility, surface.reductionEligibility);
  std::swap(reductionEligibilityPresent, surface.reductionEligibilityPresent);
  std::swap(sparseAcceptedSurface, surface.sparseAcceptedSurface);
}


void TriangleSurface::read(STL::Source &source, Task *task) {
  clear();

  uint32_t facets = source.getFacetCount(); // ASCII STL files return 0

  Vector3F v[3];
  Vector3F n;

  if (task) task->begin("Reading STL surface");

  for (unsigned i = 0; source.hasMore() && (!task || !task->shouldQuit());
       i++) {
    // Read facet
    source.readFacet(v[0], v[1], v[2], n);

    // Validate
    bool valid = n.isReal();
    for (unsigned j = 0; j < 3; j++)
      if (!v[j].isReal()) valid = false;

    if (!valid) {
      LOG_ERROR("Invalid facet in STL: normal=" << n << " triangle=("
                << v[0] << ", " << v[1] << ", " << v[2] << ")");
      continue;
    }

    add(v, n);

    if (task) {
      if (!facets && 100000 < i) i = 0;
      if (!task->update((double)i / (facets ? facets : 100000))) return;
    }
  }
}


SmartPointer<Surface> TriangleSurface::copy() const {
  return new TriangleSurface(*this);
}


void TriangleSurface::getVertices(vert_cb_t cb) const {cb(vertices, normals);}


void TriangleSurface::write(STL::Sink &sink, Task *task) const {
  Vector3F p[3];

  if (task) task->begin("Writing STL surface");

  for (unsigned i = 0; i < getTriangleCount() &&
         (!task || !task->shouldQuit()); i++) {
    unsigned offset = i * 9;

    // In an STL file, there's only one normal per facet.
    // They are the same anyway.
    Vector3F normal =
      Vector3F(normals[offset + 0], normals[offset + 1], normals[offset + 2]);

    for (unsigned j = 0; j < 3; j++)
      for (unsigned k = 0; k < 3; k++)
        p[j][k] = vertices[offset + j * 3 + k];

    sink.writeFacet(p[0], p[1], p[2], normal);

    if (task) task->update((double)i / getTriangleCount());
  }
}


void TriangleSurface::reduce(Task &task) {
  weld(task);
  TriangleMesh::reduce(task);
}


void TriangleSurface::read(const JSON::Value &value) {
  if (value.hasList("vertices")) {
    vertices.clear();
    bounds = Rectangle3D();

    for (auto &vert: *value.get("vertices")) {
      vertices.push_back(vert->getNumber());

      unsigned n = vertices.size();
      if (n % 3 == 0)
        bounds.add(Vector3D(vertices[n - 3], vertices[n - 2], vertices[n - 1]));
    }
  }

  if (value.hasList("normals")) {
    normals.clear();
    auto &l = value.get("normals");

    // Load each normal 3 times
    for (unsigned i = 0; i < l->size(); i += 3)
      for (unsigned j = 0; j < 3; j++)
        for (unsigned k = 0; k < 3; k++)
          normals.push_back(l->getNumber(i + k));
  }
}


void TriangleSurface::write(JSON::Sink &sink) const {
  sink.beginDict();

  sink.insertList("vertices");
  for (unsigned i = 0; i < vertices.size(); i++)
    sink.append(vertices[i]);
  sink.endList();

  // Only output every third normal since they are the same for a triangle.
  sink.insertList("normals");
  for (unsigned i = 0; i < normals.size(); i += 9)
    for (unsigned j = 0; j < 3; j++)
      sink.append(normals[i + j]);
  sink.endList();

  sink.endDict();
}
