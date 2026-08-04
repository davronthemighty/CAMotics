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

#include "GridTreeLeaf.h"

#include <camotics/Profile.h>

using namespace std;
using namespace cb;
using namespace CAMotics;


void GridTreeLeaf::add(const Triangle &t) {
  if (!t.normal.isReal()) return; // Degenerate, skip
  triangles.push_back(t);
}


void GridTreeLeaf::add(const Triangle &t,
                       const ContourTriangleProvenance &provenance) {
  if (!t.normal.isReal()) return; // Degenerate, skip

  if (Profile::isEnabled() || shouldCaptureContourProvenance()) {
    if (this->provenance.empty() && !triangles.empty())
      this->provenance.resize(triangles.size());
    this->provenance.push_back(provenance);
  }

  triangles.push_back(t);
}


void GridTreeLeaf::translateProvenance(const Vector3U &offset) {
  if (provenance.empty()) return;

  for (auto &entry: provenance)
    entry.cell += offset;
}


void GridTreeLeaf::gather(vector<float> &vertices, vector<float> &normals,
                          vector<ContourTriangleProvenance> *provenance)
  const {
  bool hasProvenance = provenance && this->provenance.size() == getCount();

  for (unsigned i = 0; i < getCount(); i++) {
    for (unsigned j = 0; j < 3; j++)
      for (unsigned k = 0; k < 3; k++) {
        vertices.push_back(triangles[i][j][k]);
        normals.push_back(triangles[i].normal[k]);
      }

    if (provenance)
      provenance->push_back(hasProvenance ?
                            this->provenance[i] :
                            ContourTriangleProvenance());
  }
}
