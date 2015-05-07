// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_BASE_AABB_TREE_H__
#define _PRINTER_BASE_AABB_TREE_H__

#include <memory>
#include <unordered_map>
#include "printer/base/geometry.h"
#include "common/log/log.h"

namespace printer {

class AABBTree {
 public:
  class Node;

 private:
  struct HashTrianglePtr {
    size_t operator()(const FixedTriangle* t) const {
      return HashFixedTriangle()(*t);
    }
  };
  struct EqTrianglePtr {
    size_t operator()(const FixedTriangle* a, const FixedTriangle* b) const {
      return *a == *b;
    }
  };
  typedef std::unordered_map<const FixedTriangle*, Node*,
                             HashTrianglePtr, EqTrianglePtr> TriangleMap;

 public:
  class Node {
   public:
    bool is_leaf() const { return triangle_ != NULL; }
    const Node* left() const { return left_; }
    const Node* right() const { return right_; }
    const Node* parent() const { return parent_; }
    const FixedTriangle* triangle() const { return triangle_; }
    size_t size() const;
    const Box& BoundingBox() const { return bounding_box_; }
    bool IntersectsNode(const Node& other) const;
    bool IntersectsNodeNotCoincident(const Node& other) const;
    bool Intersects(const FixedTriangle& triangle) const;
    bool IntersectsNotCoincident(const FixedTriangle& triangle) const;

   private:
    friend class AABBTree;
    explicit Node()
        : triangle_(NULL), left_(NULL), right_(NULL), parent_(NULL) {
    }
    Node(const FixedTriangle* t, Node* parent)
        : bounding_box_(t->BoundingBox()),
          triangle_(t),
          left_(NULL),
          right_(NULL),
          parent_(parent) {
    }
    ~Node() {
      delete left_;
      delete right_;
    }
    void MaybeUpdateBox();

    Box bounding_box_;
    const FixedTriangle* triangle_;
    Node* left_;
    Node* right_;
    Node* parent_;
  };

  AABBTree() : tree_(NULL) {}
  ~AABBTree();

  bool AddTriangle(const FixedTriangle& t);
  bool AddTriangle(const FixedTriangle* t /* takes ownership */);
  bool RemoveTriangle(const FixedTriangle& t);
  bool Intersects(const AABBTree& other) const;
  bool IntersectsNotCoincident(const AABBTree& other) const;
  bool Intersects(const FixedTriangle& triangle) const;
  bool IntersectsNotCoincident(const FixedTriangle& triangle) const;
  bool Intersects(const AABBTree::Node& root) const;
  bool IntersectsNotCoincident(const AABBTree::Node& root) const;
  void Swap(AABBTree* other);
  size_t size() const { return triangles_.size(); }
  const Node* root() const { return tree_; }

  class Iterator {
   public:
    Iterator(const AABBTree& tree)
        : iter_(tree.triangles_.begin()), end_(tree.triangles_.end()) {
    }

    ~Iterator() {}
    bool Done() const { return iter_ == end_; }
    void Next() { ++iter_; }
    const FixedTriangle& triangle() const { return *iter_->first; }

   private:
    TriangleMap::const_iterator iter_, end_;
  };
  Iterator iterator() const { return Iterator(*this); }

 private:
  bool AddTriangleInternal(bool allow_rebalance,
                           const FixedTriangle* t);
  void Rebalance();

  Node* tree_;
  TriangleMap triangles_;
};
}  // namespace printer

#endif  // _PRINTER_BASE_AABB_TREE_H__
