// Copyright 2015
// Author: Christopher Van Arsdale

#include "common/log/log.h"
#include "printer/base/point_octree.h"
#include "printer/objects/point_octree_object.h"

namespace printer {

PointOctreeObject::PointOctreeObject(double horizontal_resolution,
                                     double vertical_resolution,
                                     PointMode mode,
                                     PointOctree* octree)
    : mode_(mode),
      expand_(Point(horizontal_resolution / 2,
                    horizontal_resolution / 2,
                    vertical_resolution / 2)),
      octree_(octree) {
}

PointOctreeObject::~PointOctreeObject() {
}

float PointOctreeObject::ISOValue(const Point& p) {
  // Look in an expanding window to find our nearest neighbors.
  int s = 0;
  PointList points;
  do {
    Point expansion = expand_ * (1 << s);
    points.Reset(Box(p - expansion, p + expansion));
    octree_->GetAll(&points);
  } while (++s < 3 && !points.HasAllQuadrants());

  // Compute the value at xyz based on a linear combination of our
  // nearest neighbors.
  return points.GetValue(mode_);
}

bool PointOctreeObject::BoundingBox(Box* b) {
  *b = octree_->range();
  return true;
}

}  // namespace printer
