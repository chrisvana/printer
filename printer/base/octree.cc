// Copyright 2015
// Author: Christopher Van Arsdale

#include <set>
#include <unordered_set>
#include "common/base/flags.h"
#include "common/log/log.h"
#include "printer/base/geometry.h"
#include "printer/base/aabb_tree.h"
#include "printer/base/octree.h"

DEFINE_int32(octree_node_max_leaf_size, 5,
             "Number of triangles to keep in an octree leaf before trying "
             "to split it.");

using std::set;

namespace printer {
class Octree::Node {
 public:
  Node(const Box& range, int level)
      : total_triangles_(0),
        level_(level),
        range_(range),
        center_(range.center()),
        has_children_(false),
        acting_as_leaf_(true) {
    // Initialize as a leaf node.
    for (int i = 0; i < 8; ++i) {
      children_[i] = NULL;
    }
  }

  ~Node() {
    for (int i = 0; i < 8; ++i) {
      delete children_[i];
    }
  }

  size_t num_triangles() const { return total_triangles_; }
  size_t total_children() const {
    size_t total = 0;
    for (int i = 0; i < 8; ++i) {
      if (children_[i] != NULL) {
        total += 1 + children_[i]->total_children();
      }
    }
    return total;
  }
  const Box& range() const { return range_; }

  bool Remove(const FixedTriangle& t) {
    int child_index = GetChildFor(t);

    // Subnode.
    if (child_index >= 0 && !acting_as_leaf_) {
      if (children_[child_index] != NULL) {
        if (children_[child_index]->Remove(t)) {
          --total_triangles_;
          return true;
        }
      }
      return false;
    }

    // Top triangles or leaf node.
    if (triangles_.RemoveTriangle(t)) {
      --total_triangles_;
      return true;
    }

    return false;
  }

  bool AddTriangle(const FixedTriangle& t) {
    int child_index = GetChildFor(t);

    // End case: we are the last node in the tree and need to expand.
    if (acting_as_leaf_) {
      if (!triangles_.AddTriangle(t)) {
        return false;
      }

      ++total_triangles_;
      if (triangles_.size() > FLAGS_octree_node_max_leaf_size) {
        ExpandLeaf();
      }
      return true;
    }

    // Should belong to sub-node.
    if (child_index >= 0) {
      // Normal case, we already have children.
      if (mutable_child(child_index)->AddTriangle(t)) {
        ++total_triangles_;
        return true;
      }
      return false;
    }

    // Doesn't belong to subnode.
    if (triangles_.AddTriangle(t)) {
      ++total_triangles_;
      return true;
    }
    return false;
  }

  template <bool coincident_intersects>
  bool IntersectsNode(const Node& other) const {
    VLOG(2) << "Intersect (" << level_ << ", " << range_.DebugString() << "): "
            << "Triangle count "
            << total_triangles_ << "(" << triangles_.size() << " top) vs "
            << other.total_triangles_ << "(" << other.triangles_.size()
            << " top).";
    CHECK(other.range_ == range_);
    if (total_triangles_ == 0 || other.total_triangles_ == 0) {
      return false;
    }

    // Check child trees.
    if (has_children_) {
      for (int i = 0; i < 8; ++i) {
        if (children_[i] != NULL) {
          // Check their subnode.
          if (other.children_[i] != NULL &&
              children_[i]->IntersectsNode<coincident_intersects>(
                  *other.children_[i])) {
            return true;
          }

          // Check their top triangles
          if (children_[i]->IntersectsTriangleSet<coincident_intersects>(
                  other.triangles_)) {
            return true;
          }
        }
      }
    }

    // Check our top triangles against their node.
    if (other.IntersectsTriangleSet<coincident_intersects>(triangles_)) {
      return true;
    }

    return false;
  }

  template <bool coincident_intersects>
  bool IntersectsTriangle(const FixedTriangle& triangle) const {
    if (!range_.Intersects(triangle)) {
      return false;
    }
    return IntersectsTriangleInternal<coincident_intersects>(triangle);
  }

 private:
  int GetChildFor(const FixedTriangle& t) const {
    int child_for_p0 = GetChildFor(t.p0());
    int child_for_p1 = GetChildFor(t.p1());
    int child_for_p2 = GetChildFor(t.p2());
    if (child_for_p0 != child_for_p1 || child_for_p0 != child_for_p2) {
      return -1;
    }
    return child_for_p0;
  }

  template <bool coincident_intersects>
  bool IntersectsTriangleSet(const AABBTree& triangles) const {
    // Top level.
    if (coincident_intersects ?
        triangles.Intersects(triangles_) :
        triangles.IntersectsNotCoincident(triangles_)) {
      return true;
    }

    // TODO: figure out how to split triangles better.
    // A -> B, C
    // B -> D, E
    // C -> F, G
    // => B may overlap a child, but not C. It would be better to pass down just
    // B, instead of C. This may be multiple levels removed. TODO.
    // Children.
    if (has_children_ && triangles.root() != NULL) {
      for (int i = 0; i < 8; ++i) {
        if (children_[i] != NULL &&
            children_[i]->IntersectsTriangleNode<coincident_intersects>(
                *triangles.root())) {
          return true;
        }
      }
    }

    return false;
  }

  template <bool coincident_intersects>
  bool IntersectsTriangleNode(const AABBTree::Node& root) const {
    // Top level.
    VLOG(3) << "Node - Node intersection: "
            << root.size() << " against " << triangles_.size();
    if (coincident_intersects ?
        triangles_.Intersects(root) :
        triangles_.IntersectsNotCoincident(root)) {
      return true;
    }

    // Children quick check.
    if (!has_children_) {
      return false;
    }

    // Single triangle base case.
    if (root.is_leaf()) {
      const FixedTriangle& triangle = *root.triangle();
      for (int i = 0; i < 8; ++i) {
        if (children_[i] != NULL &&
            children_[i]->IntersectsTriangle<coincident_intersects>(triangle)) {
          return true;
        }
      }
      return false;
    }

    // Try to split the root node.
    // TODO: figure out how to split triangles better.
    for (int i = 0; i < 8; ++i) {
      if (children_[i] == NULL) {
        continue;
      }
      VLOG(3) << "Testing node: " << root.size();

      // left
      const AABBTree::Node& left = *root.left();
      if (!children_[i]->range().Intersects(left.BoundingBox())) {
        VLOG(3) << "Pruned left: " << left.size();
      } else if (
          children_[i]->IntersectsTriangleNode<coincident_intersects>(left)) {
        return true;
      }

      // Right
      const AABBTree::Node& right = *root.right();
      if (!children_[i]->range().Intersects(right.BoundingBox())) {
        VLOG(3) << "Pruned right: " << right.size();
      } else if (
          children_[i]->IntersectsTriangleNode<coincident_intersects>(right)) {
        return true;
      }
    }

    return false;
  }

  int GetChildFor(const Point& p) const {
    return (((p.x() >= center_.x()) << 0) |
            ((p.y() >= center_.y()) << 1) |
            ((p.z() >= center_.z()) << 2));
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

  template <bool coincident_intersects>
  bool IntersectsTriangleInternal(const FixedTriangle& triangle) const {
    // 1) Check top level triangles.
    // (TODO, AABB interval tree?)
    if (coincident_intersects ?
        triangles_.Intersects(triangle) :
        triangles_.IntersectsNotCoincident(triangle)) {
      return true;
    }

    // 2) Check children.
    for (int i = 0; i < 8; ++i) {
      if (children_[i] != NULL &&
          children_[i]->IntersectsTriangle<coincident_intersects>(triangle)) {
        return true;
      }
    }

    return false;
  }

  void ExpandLeaf() {
    acting_as_leaf_ = false;
    AABBTree tmp;
    tmp.Swap(&triangles_);
    for (AABBTree::Iterator iter = tmp.iterator();
         !iter.Done(); iter.Next()) {
      const FixedTriangle& t = iter.triangle();
      int old_index = GetChildFor(t);
      if (old_index >= 0) {
        CHECK(mutable_child(old_index)->AddTriangle(t));
      } else {
        CHECK(triangles_.AddTriangle(t));
      }
    }
  }

  Node* mutable_child(int index) {
    has_children_ = true;
    Node* child = children_[index];
    if (child == NULL) {
      child = new Node(RangeForChild(index), level_ + 1);
      children_[index] = child;
    }
    return child;
  }

  size_t total_triangles_;
  int level_;
  Box range_;
  Point center_;
  Node* children_[8];
  AABBTree triangles_;
  bool has_children_, acting_as_leaf_;
};

Octree::Octree(const Box& range)
    : root_(new Node(range, 0)) {
}

Octree::~Octree() {
}

const Box& Octree::range() const {
  return root_->range();
}

size_t Octree::total_nodes() const {
  return 1 + root_->total_children();
}

bool Octree::AddTriangle(const Triangle& t) {
  return root_->AddTriangle(FixedTriangle(t));
}

bool Octree::RemoveTriangle(const Triangle& t) {
  return root_->Remove(FixedTriangle(t));
}

bool Octree::Intersects(const Octree& other) const {
  return root_->IntersectsNode<true>(*other.root_);
}

bool Octree::IntersectsNotCoincident(const Octree& other) const {
  return root_->IntersectsNode<false>(*other.root_);
}

bool Octree::Intersects(const Triangle& t) const {
  return root_->IntersectsTriangle<true>(FixedTriangle(t));
}

bool Octree::IntersectsNotCoincident(const Triangle& t) const {
  return root_->IntersectsTriangle<false>(FixedTriangle(t));
}

}  // namespace printer
