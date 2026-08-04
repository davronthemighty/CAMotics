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

#pragma once


#include "ContourProvenance.h"
#include "ReductionEligibility.h"
#include "Surface.h"
#include "TriangleMesh.h"

#include <cstdint>


#include <cbang/SmartPointer.h>

#include <vector>


namespace STL {class Source;}

namespace CAMotics {
  class GridTree;

  class TriangleSurface : public Surface, public TriangleMesh {
    cb::Rectangle3D bounds;
    std::vector<ContourTriangleProvenance> contourProvenance;
    std::vector<int32_t> contourProvenanceNeighbors;
    ContourProvenanceReport contourProvenanceReport;
    bool contourProvenanceValid = false;
    bool contourProvenanceNeighborsValid = false;
    bool contourProvenanceNeighborsRaw = false;
    ReductionEligibility reductionEligibility;
    bool reductionEligibilityPresent = false;
    bool sparseAcceptedSurface = false;

    void releaseContourProvenanceRecords();

  public:
    TriangleSurface() {}
    TriangleSurface(const GridTree &tree);
    TriangleSurface(STL::Source &source, Task *task = 0);
    TriangleSurface(std::vector<cb::SmartPointer<Surface> > &surfaces);
    TriangleSurface(const TriangleSurface &o);

    void add(const cb::Vector3F vertices[3]);
    void add(const cb::Vector3F vertices[3], const cb::Vector3F &normal);
    void add(const GridTree &tree);

    void clear();
    void replace(const std::vector<float> &vertices,
                 const std::vector<float> &normals);
    void replace(std::vector<float> &&vertices, std::vector<float> &&normals);
    void swap(TriangleSurface &surface);
    void read(STL::Source &source, Task *task = 0);

    const std::vector<float> &getVertices() const {return vertices;}
    const std::vector<float> &getNormals()  const {return normals;}
    bool hasContourProvenance() const {return contourProvenanceValid;}
    const std::vector<ContourTriangleProvenance> &getContourProvenance() const
      {return contourProvenance;}
    const ContourProvenanceReport &getContourProvenanceReport() const
      {return contourProvenanceReport;}
    bool hasContourProvenanceNeighbors() const
      {return contourProvenanceNeighborsValid;}
    bool hasRawContourProvenanceNeighbors() const
      {return contourProvenanceNeighborsValid && contourProvenanceNeighborsRaw;}
    const std::vector<int32_t> &getContourProvenanceNeighbors() const
      {return contourProvenanceNeighbors;}
    void clearContourProvenance();
    bool hasReductionEligibility() const {return reductionEligibilityPresent;}
    const ReductionEligibility &getReductionEligibility() const
      {return reductionEligibility;}
    void setReductionEligibility(const ReductionEligibility &eligibility);
    void clearReductionEligibility();
    bool isSparseAcceptedSurface() const {return sparseAcceptedSurface;}
    void markSparseAcceptedSurface(bool value = true)
      {sparseAcceptedSurface = value;}

    // From Surface
    cb::SmartPointer<Surface> copy() const override;
    uint64_t getTriangleCount() const override
      {return TriangleMesh::getTriangleCount();}
    cb::Rectangle3D getBounds() const override {return bounds;}
    void getVertices(vert_cb_t cb) const override;
    void write(STL::Sink &sink, Task *task = 0) const override;
    void reduce(Task &task) override;

    // From cb::JSON::Serializable
    void read(const cb::JSON::Value &value) override;
    void write(cb::JSON::Sink &sink) const override;
    using Surface::read;
    using Surface::write;
  };
}
