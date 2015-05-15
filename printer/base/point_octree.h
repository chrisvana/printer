// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_BASE_POINT_OCTREE_H__
#define _PRINTER_BASE_POINT_OCTREE_H__

#include <memory>
#include <string>
#include "printer/base/geometry.h"

#include "common/log/log.h"

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

  // Mutators.
  void Reset(const Box& input);
  void SetPoint(const Point& p, float val);

  // Accessors
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

  // Misc.
  bool MayContain(const Box& range) const;
  bool HasAllQuadrants() const;
  std::string DebugString() const;

 private:
  void UpdateInternal(double dist2, double dnorm, double val);
  int QuadrantFor(const Point& test) const {
    return ((test.x() < center_.x() ? 1 : 0) |
            (test.y() < center_.y() ? 2 : 0) |
            (test.z() < center_.z() ? 4 : 0));
  }

  Box range_;
  Point center_;
  double x_, y_, z_;

  bool quad_[8];
  double closest_val_, max_val_;
  double closest_dist2_;
  double total_weight_;
  double total_average_;
};

// Inlined.
inline PointList::PointList() {
  Reset(Box());
}

inline void PointList::Reset(const Box& input) {
  range_ = input;
  center_ = input.center();
  x_ = (input.size_x()*input.size_x()) / 4;
  y_ = (input.size_y()*input.size_y()) / 4;
  z_ = (input.size_z()*input.size_z()) / 4;
  for (int i = 0; i < 8; ++i) {
    quad_[i] = false;
  }
  closest_val_ = max_val_ = total_average_ = total_weight_ = 0;
  closest_dist2_ = std::numeric_limits<double>::max();
}

inline bool PointList::MayContain(const Box& range) const {
  return range.Intersects(range_);
}

inline void PointList::SetPoint(const Point& p, float val) {
  if (!range_.Contains(p)) {
    return;
  }

  Point tmp = (p - center_);
  double r2 = ((tmp.x() * tmp.x()) / x_ +
               (tmp.y() * tmp.y()) / y_ +
               (tmp.z() * tmp.z()) / z_);
  if (r2 > 1) {
    return;
  }
  quad_[QuadrantFor(p)] = true;
  UpdateInternal(tmp.magnitude2(), r2, val);
}

inline bool PointList::HasAllQuadrants() const {
  for (int i = 0; i < 8; ++i) {
    if (!quad_[i]) {
      return false;
    }
  }
  return true;
}

inline void PointList::UpdateInternal(double dist2, double dnorm, double val) {
  if (dist2 < closest_dist2_) {
    closest_dist2_ = dist2;
    closest_val_ = val;
  }
  if (val > max_val_) {
    max_val_ = val;
  }
  double dist = sqrt(dist2);
  if (dist > Triangle::kDefaultIntersectEpsilon) {
    double weight = (1.0 - dnorm)*(1.0 - dnorm)*(dnorm+1)*(dnorm+1) / dist;
    // double weight = (cos(3.14159265358979*dnorm)+1) / 2 / dist;
    total_weight_ += weight;
    total_average_ += weight * val;
  }
}

inline float PointList::GetInterpolate() const {
  if (closest_dist2_ < Triangle::kDefaultIntersectEpsilon) {
    return closest_val_;
  }
  if (total_weight_ == 0) {
    return 0;
  }

  // Pretend like we saw just as many 0s in the uncharted quadrants.
  int total_seen = 0;
  for (int i = 0; i < 8; ++i) {
    total_seen += quad_[i];
  }
  return total_average_ / (total_weight_ * 8 / total_seen);
}

inline float PointList::GetMaxValue() const {
  return max_val_;
}

inline float PointList::GetClosestValue() const {
  return closest_val_;
}

}  // namespace printer

#endif  // _PRINTER_BASE_POINT_OCTREE_H__
