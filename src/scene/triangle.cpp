#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.

  Vector3D AB = p2 - p1;
  Vector3D AC = p3 - p1;
  Vector3D BC = p3 - p2;
  Vector3D CA = p1 - p3;

  Vector3D N = cross(AB, AC);

  double t = (dot((p1 - r.o), N)) / dot(r.d, N);

  if (t<0  || t < r.min_t || t > r.max_t)
    return false;

  Vector3D X = r.o + t * r.d;

  Vector3D AX = X - p1;
  Vector3D BX = X - p2;
  Vector3D CX = X - p3;

  double f1 = sqrt(cross(AX, AB).norm2());
  double f2 = sqrt(cross(BX, BC).norm2());
  double f3 = sqrt(cross(CX, CA).norm2());
  double s = sqrt(cross(AB, AC).norm2());

  return (abs(s-f1-f2-f3) <= 0.1e-5);
}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly

  Vector3D AB = p2 - p1;
  Vector3D AC = p3 - p1;
  Vector3D BC = p3 - p2;
  Vector3D CA = p1 - p3;

  Vector3D N = cross(AB, AC).unit();

  double t = (dot((p1 - r.o), N)) / dot(r.d, N);

  if (t<0 || t < r.min_t || t > r.max_t)
    return false;

  Vector3D X = r.o + t * r.d;
  Vector3D AX = X - p1;
  Vector3D BX = X - p2;
  Vector3D CX = X - p3;

  double f1 = sqrt(cross(AX, AB).norm2()); // area of ABX
  double f2 = sqrt(cross(BX, BC).norm2()); // area of BCX
  double f3 = sqrt(cross(CX, CA).norm2()); // area of CAX
  double s = sqrt(cross(AB, AC).norm2()); // area of ABC

  if(abs(s-f1-f2-f3) <= 0.1e-5) { // f1 + f2 + f3 == s
    r.max_t = t;
    isect->t = t;
    isect->n = n1 * f1/s + n2 * f2/s + n3 * f3/s;
    isect->bsdf = get_bsdf();
    isect->primitive = this;
    return true;
  }
  return false;
}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
