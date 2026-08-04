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

#include "MachinePart.h"

#include <cbang/Exception.h>

using namespace CAMotics;
using namespace cb;
using namespace std;


MachinePart::MachinePart(const string &name, const JSON::ValuePtr &config) :
  name(name) {read(*config);}


void MachinePart::addBox(const Vector3D &min, const Vector3D &max) {
  Vector3F p[] = {
    Vector3F(min.x(), min.y(), min.z()),
    Vector3F(max.x(), min.y(), min.z()),
    Vector3F(max.x(), max.y(), min.z()),
    Vector3F(min.x(), max.y(), min.z()),
    Vector3F(min.x(), min.y(), max.z()),
    Vector3F(max.x(), min.y(), max.z()),
    Vector3F(max.x(), max.y(), max.z()),
    Vector3F(min.x(), max.y(), max.z()),
  };
  static const unsigned faces[][3] = {
    {0, 2, 1}, {0, 3, 2}, // bottom
    {4, 5, 6}, {4, 6, 7}, // top
    {0, 1, 5}, {0, 5, 4}, // front
    {3, 7, 6}, {3, 6, 2}, // back
    {0, 4, 7}, {0, 7, 3}, // left
    {1, 2, 6}, {1, 6, 5}, // right
  };
  for (const auto &face: faces) {
    Vector3F triangle[] = {p[face[0]], p[face[1]], p[face[2]]};
    add(triangle);
  }

  static const unsigned edges[][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0},
    {4, 5}, {5, 6}, {6, 7}, {7, 4},
    {0, 4}, {1, 5}, {2, 6}, {3, 7},
  };
  for (const auto &edge: edges)
    for (unsigned vertex: edge)
      for (unsigned axis = 0; axis < 3; axis++)
        lines.push_back(p[vertex][axis]);
}


void MachinePart::read(const JSON::Value &value) {
  clear();
  color   .read(value.getList("color"));
  init    .read(value.getList("init"));
  home    .read(value.getList("home"));
  min     .read(value.getList("min"));
  max     .read(value.getList("max"));
  movement.read(value.getList("movement"));

  if (value.hasList("lines")) {
    auto &lines = value.getList("lines");
    for (auto &line: lines)
      this->lines.push_back(line->getNumber());
  }

  bool hasGeometry = false;
  if (value.hasList("boxes")) {
    auto &boxes = value.getList("boxes");
    for (unsigned i = 0; i < boxes.size(); i++) {
      auto &box = boxes.getList(i);
      if (box.size() != 6)
        THROW("Machine part box expected six coordinates");
      addBox(Vector3D(box.getNumber(0), box.getNumber(1), box.getNumber(2)),
             Vector3D(box.getNumber(3), box.getNumber(4), box.getNumber(5)));
      hasGeometry = true;
    }
  }

  if (value.hasList("mesh")) {
    auto &vertices = value.getList("mesh");
    if (vertices.size() % 9)
      THROW("Machine part mesh expected complete triangles");

    for (unsigned i = 0; i < vertices.size(); i += 9) {
      Vector3F t[3];

      for (unsigned j = 0; j < 3; j++)
        for (unsigned k = 0; k < 3; k++)
          t[j][k] = vertices.getNumber(i + j * 3 + k);

      add(t);
    }
    hasGeometry = true;
  }

  if (!hasGeometry) THROW("Machine part expected mesh or boxes");
}


void MachinePart::write(JSON::Sink &sink) const {
  sink.beginDict();

  sink.beginInsert("color");
  color.write(sink);

  sink.beginInsert("init");
  init.write(sink);

  sink.beginInsert("home");
  home.write(sink);

  sink.beginInsert("min");
  min.write(sink);

  sink.beginInsert("max");
  max.write(sink);

  sink.beginInsert("movement");
  movement.write(sink);

  if (!lines.empty()) {
    sink.insertList("lines");
    for (float x: lines) sink.append(x);
    sink.endList();
  }

  sink.insertList("mesh");
  for (auto &v: getVertices())
    sink.append(v);
  sink.endList();

  sink.endDict();
}
