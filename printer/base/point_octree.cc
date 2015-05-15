// Copyright 2015
// Author: Christopher Van Arsdale

#include <map>
#include <memory>
#include <string>
#include <sstream>
#include "common/log/log.h"
#include "printer/base/geometry.h"
#include "printer/base/point_octree.h"

using std::map;

namespace printer {

// static 
PointList::PointMode PointList::FromString(
    const std::string& name) {
  if (name == "CLOSEST") {
    return CLOSEST;
  }
  if (name == "MAXIMUM") {
    return MAXIMUM;
  }
  if (name == "INTERPOLATE") {
    return INTERPOLATE;
  }
  LOG(FATAL) << "Unknown PointMode: " << name;
}

std::string PointList::DebugString() const {
  std::stringstream out;
  out << "Point: " << center_.DebugString() << std::endl;
  out << "Range: " << range_.DebugString() << std::endl;
  for (int i = 0; i < 8; ++i) {
    out << "quad[" << i << "] = " << (quad_[i] ? "TRUE" : "FALSE") << std::endl;
  }
  out << "Interpolate: " << GetInterpolate() << std::endl;
  out << "Max: " << GetMaxValue() << std::endl;
  out << "Closest: " << GetClosestValue() << std::endl;
  return out.str();
}

class PointOctree::InternalOctree {
 public:
  explicit InternalOctree(const Box& range)
      : range_(range),
        center_(range.center()),
        has_children_(false) {
    for (int i = 0; i < 8; ++i) {
      children_[i] = NULL;
    }
  }

  ~InternalOctree() {
    for (int i = 0; i < 8; ++i) {
      delete children_[i];
    }
  }

  void SetPoint(const Point& point, float value) {
    if (has_children_) {
      MutableChild(GetChildFor(point))->SetPoint(point, value);
      return;
    }

    points_[point] = value;
    MaybePushPointsDown();
  }

  void Merge(InternalOctree* other_root) {
    // Merge leaves.
    if (points_.empty()) {
      points_.swap(other_root->points_);
    } else if (!other_root->points_.empty()) {
      points_.insert(other_root->points_.begin(),
                     other_root->points_.end());
    }

    // Merge children
    if (has_children_ || other_root->has_children_) {
      has_children_ = true;
      for (int i = 0; i < 8; ++i) {
        if (other_root->children_[i] != NULL) {
          if (children_[i] == NULL) {
            children_[i] = other_root->children_[i];
          } else {
            children_[i]->Merge(other_root->children_[i]);
          }
          other_root->children_[i] = NULL;  // we took ownership.
        }
      }
    }

    MaybePushPointsDown();
    delete other_root;
  }

  bool GetPoint(const Point& point, float* value) const {
    auto it =  points_.find(point);
    if (it != points_.end()) {
      *value = it->second;
      return true;
    }

    const InternalOctree* child = children_[GetChildFor(point)];
    return child != NULL && child->GetPoint(point, value);
  }

  void GetAll(PointList* list) const {
    if (!list->MayContain(range_)) {
      return;
    }

    for (auto it : points_) {
      list->SetPoint(it.first, it.second);
    }

    if (has_children_) {
      for (int i = 0; i < 8; ++i) {
        if (children_[i] != NULL) {
          children_[i]->GetAll(list);
        }
      }
    }
  }

  const Box& range() const {
    return range_;
  }

  size_t size() const {
    size_t val = points_.size();
    if (has_children_) {
      for (int i = 0; i < 8; ++i) {
        val += (children_[i] == NULL ? 0 : children_[i]->size());
      }
    }
    return val;
  }

 private:
  void MaybePushPointsDown() {
    if (points_.size() > 10 || has_children_) {
      for (auto it : points_) {
        MutableChild(GetChildFor(it.first))->SetPoint(it.first, it.second);
      }
      points_.clear();
    }
  }

  int GetChildFor(const Point& p) const {
    return (((p.x() >= center_.x()) << 0) |
            ((p.y() >= center_.y()) << 1) |
            ((p.z() >= center_.z()) << 2));
  }

  InternalOctree* MutableChild(int index) {
    has_children_ = true;
    if (children_[index] == NULL) {
      children_[index] = new InternalOctree(RangeForChild(index));
    }
    return children_[index];
  }

  Box RangeForChild(int index) const {
    Range x = (index & 1 ?
               Range(center_.x(), range_.top().x()) :
               Range(range_.bottom().x(), center_.x()));
    Range y = (index & 2 ?
               Range(center_.y(), range_.top().y()) :
               Range(range_.bottom().y(), center_.y()));
    Range z = (index & 4 ?
               Range(center_.z(), range_.top().z()) :
               Range(range_.bottom().z(), center_.z()));
    return Box(Point(x.start(), y.start(), z.start()),
               Point(x.end(), y.end(), z.end()));
  }

  // Tree info
  Box range_;
  Point center_;
  bool has_children_;
  InternalOctree* children_[8];

  // Point info.
  map<Point, float> points_;
};

PointOctree::PointOctree(const Box& range)
    : root_(new InternalOctree(range)) {
}

PointOctree::~PointOctree() {
}

void PointOctree::SetPoint(const Point& point, float value) {
  root_->SetPoint(point, value);
}

bool PointOctree::GetPoint(const Point& point, float* value) {
  return root_->GetPoint(point, value);
}

void PointOctree::GetAll(PointList* list) const {
  root_->GetAll(list);
}

const Box& PointOctree::range() const {
  return root_->range();
}

bool PointOctree::Merge(PointOctree* to_own) {
  if (to_own->range() != range()) {
    return false;
  }
  root_->Merge(to_own->root_.release());
  delete to_own;
  return true;
}

size_t PointOctree::size() const {
  return root_->size();
}

}  // namespace printer
