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
      expand_(Point(2 * horizontal_resolution,
                    2 * horizontal_resolution,
                    2 * vertical_resolution)),
      octree_(octree) {
}

PointOctreeObject::~PointOctreeObject() {
}

float PointOctreeObject::ISOValue(const Point& p) {
  PointList points;

#if 0  // TODO(cvanarsdale): Figure out proper interpolation.
  // Look in an expanding window to find our nearest neighbors.
  const int kMaxExpansion = 2;
  int s = 0;
  do {
    Point expansion = expand_ * (1 << s);
    points.Reset(Box(p - expansion, p + expansion));
    octree_->GetAll(&points);
  } while (++s <= kMaxExpansion && !points.HasAllQuadrants());
#else
  points.Reset(Box(p - expand_, p + expand_));
  octree_->GetAll(&points);
#endif

  float val = points.GetValue(mode_);

  // Some logging.
  if (VLOG_IS_ON(3)) {
    if (!points.HasAllQuadrants()) {
      VLOG(3) << "Point missing quadrants: " << points.DebugString();
    }
    if (VLOG_IS_ON(4)) {
      if (VLOG_IS_ON(5) || val > 0) {
        VLOG(4) << points.DebugString();
      }
    }
  }

  return val;
}

bool PointOctreeObject::BoundingBox(Box* b) {
  *b = octree_->range();
  return true;
}

}  // namespace printer
