#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox bbox;
  int cntBox = 0;
  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
    cntBox++;
  }

  BVHNode *node = new BVHNode(bbox);

  if(cntBox <= max_leaf_size) {
    node->start = start;
    node->end = end;
    return node;
  }

  Vector3D centroid = bbox.centroid();

  double length_x = bbox.max.x - bbox.min.x;
  double length_y = bbox.max.y - bbox.min.y;
  double length_z = bbox.max.z - bbox.min.z;

  std::vector<Primitive *>::iterator it1, it2;
  it1 = start;
  it2 = end;

  if(length_x >= length_y && length_x >= length_z) {  // split by x
    while (it1 != it2)
    {
      Vector3D bb_centroid = (*it1)->get_bbox().centroid();
      if(bb_centroid.x >= centroid.x) {
        swap(*(it1), *(--it2));
      } else {
        it1 ++;
      }
    }
  } else if(length_y >= length_x && length_y >= length_z) {  // split by y
    while (it1 != it2)
    {
      Vector3D bb_centroid = (*it1)->get_bbox().centroid();
      if(bb_centroid.y >= centroid.y) {
        swap(*(it1), *(--it2));
      } else {
        it1 ++;
      }
    }
  } else {  // split by z
    while (it1 != it2)
    {
      Vector3D bb_centroid = (*it1)->get_bbox().centroid();
      if(bb_centroid.z >= centroid.z) {
        swap(*(it1), *(--it2));
      } else {
        it1 ++;
      }
    }
  }

  if(it1 == start || it1 == end) {
    int dist = distance(start,end);
    advance(it1, dist/2);
  }

  node->l = construct_bvh(start, it1, max_leaf_size);
  node->r = construct_bvh(it1, end, max_leaf_size);

  return node;

}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  double t0, t1;
  bool flag = node->bb.intersect(ray,t0,t1);
  if(!flag || t0 > ray.max_t || t1 < ray.min_t)
    return false;

  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      if ((*p)->has_intersection(ray))
        return true;
    }
    return false;
  } else {
    return has_intersection(ray, node->l) || has_intersection(ray, node->r);
  }
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.

  double t0, t1;
  bool flag;
  flag = node->bb.intersect(ray,t0,t1);
  if(!flag || t0 > ray.max_t || t1 < ray.min_t)
    return false;

  if (node->isLeaf()) {
    flag = false;
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      if ((*p)->intersect(ray,i))
        flag = true;
    }
    return flag;
  } else {
    bool flag1 = intersect(ray, i, node->l);
    bool flag2 = intersect(ray, i , node->r);
    return flag1 || flag2;
  }
}

} // namespace SceneObjects
} // namespace CGL
