// Copyright 2015
// Author: Christopher Van Arsdale

#include <set>
#include <unordered_set>
#include <memory>
#include "printer/base/aabb_tree.h"
#include "printer/base/geometry.h"
#include "common/log/log.h"

namespace printer {
namespace {
double BoundingBoxScore(const Box& input) {
  double max_dimension = std::max(std::max(input.size_x(),
                                           input.size_y()),
                                  input.size_z());
  return max_dimension;
  /*
  double x = std::max(0.01, input.size_x());
  double y = std::max(0.01, input.size_y());
  double z = std::max(0.01, input.size_z());
  return x*y*z;
  */
}
double ScorePath(const Box& input, const Box& path) {
  double orig = BoundingBoxScore(path);
  double new_value = BoundingBoxScore(path.Unioned(input));
  double delta = new_value - orig;
  return delta + (orig / 100);
}
}  // anonymous namespace

void AABBTree::Node::MaybeUpdateBox() {
  Node* current = this;
  while (current != NULL) {
    Box new_box = (current->is_leaf() ? current->triangle_->BoundingBox() :
                   current->left_->BoundingBox().Unioned(
                       current->right_->BoundingBox()));
    if (new_box == current->bounding_box_) {
      break;
    }
    current->bounding_box_ = new_box;
    current = current->parent_;
  }
}

size_t AABBTree::Node::size() const {
  if (is_leaf()) { return 1; }
  return left_->size() + right_->size();
}

bool AABBTree::Node::Intersects(const FixedTriangle& triangle) const {
  if (is_leaf()) {
    if (triangle_->Intersects(triangle)) {
      VLOG(3) << "Intersection: "
              << triangle.DebugString() << " against "
              << triangle_->DebugString();
      return true;
    }
    return false;
  }

  return (bounding_box_.Intersects(triangle) &&
          (left_->Intersects(triangle) || right_->Intersects(triangle)));
}

bool AABBTree::Node::IntersectsNotCoincident(
    const FixedTriangle& triangle) const {
  if (is_leaf()) {
    if (triangle_->IntersectsNotCoincident(triangle)) {
      VLOG(3) << "Intersection (not coincident): "
              << triangle.DebugString() << " against "
              << triangle_->DebugString();
      return true;
    }
    return false;
  }
  return (bounding_box_.Intersects(triangle) &&
          (left_->IntersectsNotCoincident(triangle) ||
           right_->IntersectsNotCoincident(triangle)));
}

bool AABBTree::Node::IntersectsNode(const AABBTree::Node& other) const {
  // We want to split our nodes, so we alternate which node is in charge of
  // intersecting. Otherwise we would just pass an entire tree down to each
  // leaf, and do an n^2 comparison.
  if (is_leaf()) {
    return other.Intersects(*triangle_);
  }
  return other.IntersectsNode(*left_) || other.IntersectsNode(*right_);
}

bool AABBTree::Node::IntersectsNodeNotCoincident(
    const AABBTree::Node& other) const {
  if (is_leaf()) {
    return other.IntersectsNotCoincident(*triangle_);
  }
  return (other.IntersectsNodeNotCoincident(*left_) ||
          other.IntersectsNodeNotCoincident(*right_));
}

AABBTree::~AABBTree() {
  delete tree_;
  for (auto it : triangles_) {
    delete it.first;
  }
}

bool AABBTree::AddTriangleInternal(bool allow_rebalance,
                                   const FixedTriangle* t) {
  if (triangles_.find(t) != triangles_.end()) {
    // We already have this triangle.
    delete t;
    return false;
  }

  int depth = 0;
  Node* final = NULL;
  if (tree_ == NULL) {
    final = tree_ = new Node(t, NULL);
  } else {
    // Find the best leaf
    Node* current = tree_;
    while (!current->is_leaf()) {
      double left = ScorePath(t->BoundingBox(),
                              current->left_->bounding_box_);
      double right = ScorePath(t->BoundingBox(),
                               current->right_->bounding_box_);

      // Pick whichever has the smallest.
      current = (left <= right ? current->left_ : current->right_);
      ++depth;
    }

    // Insert node above leaf to split leaf.
    ++depth;
    Node* new_node = new Node();
    new_node->right_ = current;
    final = new_node->left_ = new Node(t, new_node);

    // Copy parent from old one.
    new_node->parent_ = current->parent_;
    current->parent_ = new_node;

    // Fix up.
    if (new_node->parent_ == NULL) {
      tree_ = new_node;
    } else if (new_node->parent_->left_ == current) {
      new_node->parent_->left_ = new_node;
    } else {
      new_node->parent_->right_ = new_node;
    }
    new_node->MaybeUpdateBox();
  }
  triangles_[t] = final;

  VLOG(5) << "Depth: " << depth
          << ", size: " << size()
          << ", optimal: " << 1 + floor(log(size()));
  if (allow_rebalance && depth > 20 && (size() / depth) < 3) {
    Rebalance();
  }

  return true;
}

bool AABBTree::AddTriangle(const FixedTriangle& t) {
  return AddTriangleInternal(true, new FixedTriangle(t));
}

bool AABBTree::AddTriangle(const FixedTriangle* t) {
  return AddTriangleInternal(true, t);
}

void AABBTree::Rebalance() {
  VLOG(4) << "Rebalance.";
  delete tree_;
  tree_ = NULL;
  TriangleMap tmp;
  tmp.swap(triangles_);
  for (auto it : tmp) {
    CHECK(AddTriangleInternal(false, it.first));
  }
}

bool AABBTree::RemoveTriangle(const FixedTriangle& t) {
  auto it = triangles_.find(&t);
  if (it == triangles_.end()) {
    return false;
  }

  Node* current = it->second;
  const FixedTriangle* triangle = it->first;
  CHECK(current->is_leaf());
  triangles_.erase(it);

  // Clean up triangle
  delete triangle;

  // Remove "current".

  // 1) Check root.
  if (current->parent_ == NULL) {
    CHECK_EQ(0, triangles_.size());
    CHECK_EQ(current, tree_);
    delete current;
    tree_ = NULL;
    return true;
  }

  // 2) Fix leaf node, and parent bounding boxes up the tree.
  // Replace "parent" of current with the other child.
  Node* parent = current->parent_;
  Node* pp = parent->parent_;
  Node* to_merge = (current == parent->left_ ? parent->right_ : parent->left_);
  to_merge->parent_ = parent->parent_;
  if (pp == NULL) {
    tree_ = to_merge;
  } else {
    (pp->left_ == parent ? pp->left_ : pp->right_) = to_merge;
    pp->MaybeUpdateBox();
  }

  // Clean up old nodes.
  parent->left_ = parent->right_ = NULL;
  delete parent;
  delete current;

  return true;
}

bool AABBTree::Intersects(const AABBTree& other) const {
  if (other.triangles_.empty() || triangles_.empty()) {
    return false;
  }
  return tree_->IntersectsNode(*other.tree_);
}

bool AABBTree::IntersectsNotCoincident(const AABBTree& other) const {
  if (other.triangles_.empty() || triangles_.empty()) {
    return false;
  }
  return tree_->IntersectsNodeNotCoincident(*other.tree_);
}

bool AABBTree::Intersects(const AABBTree::Node& root) const {
  if (triangles_.empty()) {
    return false;
  }
  return tree_->IntersectsNode(root);
}

bool AABBTree::IntersectsNotCoincident(const AABBTree::Node& root) const {
  if (triangles_.empty()) {
    return false;
  }
  return tree_->IntersectsNodeNotCoincident(root);
}

bool AABBTree::Intersects(const FixedTriangle& triangle) const {
  if (triangles_.empty()) {
    return false;
  }
  return tree_->Intersects(triangle);
}

bool AABBTree::IntersectsNotCoincident(const FixedTriangle& triangle) const {
  if (triangles_.empty()) {
    return false;
  }
  return tree_->IntersectsNotCoincident(triangle);
}

void AABBTree::Swap(AABBTree* other) {
  std::swap(tree_, other->tree_);
  triangles_.swap(other->triangles_);
}

}  // namespace printer
