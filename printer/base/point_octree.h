// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_BASE_POINT_OCTREE_H__
#define _PRINTER_BASE_POINT_OCTREE_H__

#include <memory>
#include <string>
#include "printer/base/geometry.h"

namespace printer {
class PointList;

class PointOctree {
 public:
  explicit PointOctree(const Box& range);
  ~PointOctree();

  void SetPoint(const Point& point, float value);
  bool GetPoint(const Point& point, float* value);
  void GetAll(PointList* list) const;
  const Box& range() const;
  size_t size() const;

  bool Merge(PointOctree* to_own);

 private:
  class InternalOctree;
  std::unique_ptr<InternalOctree> root_;
};

class PointList {
 public:
  enum PointMode {
    INTERPOLATE,
    CLOSEST,
    MAXIMUM,
  };
  static PointMode FromString(const std::string& name);

  PointList();
  ~PointList() {}

  void Reset(const Box& input);
  void SetPoint(const Point& p, float val);
  float GetValue(PointMode mode) const {
    switch (mode) {
      case CLOSEST: return GetClosestValue();
      case MAXIMUM: return GetMaxValue();
      default: return GetInterpolate();
    }
  }

  float GetInterpolate() const;
  float GetMaxValue() const;
  float GetClosestValue() const;

  bool MayContain(const Box& range) const;
  bool HasAllQuadrants() const;

 private:
  int QuadrantFor(const Point& test) const {
    return ((test.x() < center_.x() ? 1 : 0) |
            (test.y() < center_.y() ? 2 : 0) |
            (test.z() < center_.z() ? 4 : 0));
  }

  Box range_;
  Point center_;
  double x_, y_, z_;
  double dist2_[8];
  double val_[8];
  Point point_[8];
};

// Inlined.
inline PointList::PointList() {
  Reset(Box());
}

inline void PointList::Reset(const Box& input) {
  range_ = input;
  center_ = input.center();
  x_ = input.size_x()*input.size_x();
  y_ = input.size_y()*input.size_y();
  z_ = input.size_z()*input.size_z();
  for (int i = 0; i < 8; ++i) {
    dist2_[i] = std::numeric_limits<double>::max();
  }
}

inline bool PointList::MayContain(const Box& range) const {
  return range.Intersects(range_);
}

inline void PointList::SetPoint(const Point& p, float val) {
  if (!range_.Contains(p)) {
    return;
  }

  Point tmp = (p - center_);
  if (((tmp.x() * tmp.x()) / x_ +
       (tmp.y() * tmp.y()) / y_ +
       (tmp.z() * tmp.z()) / z_) > 1) {
    return;
  }

  double dist = tmp.magnitude2();
  int quad = QuadrantFor(p);
  if (dist < dist2_[quad]) {
    dist2_[quad] = dist;
    val_[quad] = val;
    point_[quad] = p;
  }
}

inline bool PointList::HasAllQuadrants() const {
  for (int i = 0; i < 8; ++i) {
    if (dist2_[i] == std::numeric_limits<double>::max()) {
      return false;
    }
  }
  return true;
}

}  // namespace printer

#endif  // _PRINTER_BASE_POINT_OCTREE_H__
