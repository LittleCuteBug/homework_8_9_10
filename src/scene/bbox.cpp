#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

  // compute intersect for x

  bool flag = false;
  double t_min = 1e9;
  double t_max = -1e9;
  double t;
  Vector3D p;


  t = dot((min - r.o), Vector3D(1, 0, 0)) / dot(r.d, Vector3D(1, 0, 0));
  p = r.at_time(t);
  if (p.y >= min.y && p.y <= max.y && p.z >= min.z && p.z <= max.z) {
    flag = true;
    if (t < t_min) t_min = t;
    if (t > t_max) t_max = t;
  }

  t = dot((max - r.o), Vector3D(1, 0, 0)) / dot(r.d, Vector3D(1, 0, 0));
  p = r.at_time(t);
  if (p.y >= min.y && p.y <= max.y && p.z >= min.z && p.z <= max.z) {
    flag = true;
    if (t < t_min) t_min = t;
    if (t > t_max) t_max = t;
  }

  t = dot((min - r.o), Vector3D(0, 1, 0)) / dot(r.d, Vector3D(0, 1, 0));
  p = r.at_time(t);
  if (p.x >= min.x && p.x <= max.x && p.z >= min.z && p.z <= max.z) {
    flag = true;
    if (t < t_min) t_min = t;
    if (t > t_max) t_max = t;
  }

  t = dot((max - r.o), Vector3D(0, 1, 0)) / dot(r.d, Vector3D(0, 1, 0));
  p = r.at_time(t);
  if (p.x >= min.x && p.x <= max.x && p.z >= min.z && p.z <= max.z) {
    flag = true;
    if (t < t_min) t_min = t;
    if (t > t_max) t_max = t;
  }

  t = dot((min - r.o), Vector3D(0, 0, 1)) / dot(r.d, Vector3D(0, 0, 1));
  p = r.at_time(t);
  if (p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y) {
    flag = true;
    if (t < t_min) t_min = t;
    if (t > t_max) t_max = t;
  }

  t = dot((max - r.o), Vector3D(0, 0, 1)) / dot(r.d, Vector3D(0, 0, 1));
  p = r.at_time(t);
  if (p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y) {
    flag = true;
    if (t < t_min) t_min = t;
    if (t > t_max) t_max = t;
  }

  if (flag){
    t0 = t_min;
    t1 = t_max;
    return true;
  } else {
    return false;
  }
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
