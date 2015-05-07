// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_BASE_OCTREE_H__
#define _PRINTER_BASE_OCTREE_H__

#include <memory>

namespace printer {

class Octree {
 public:
  explicit Octree(const Box& range);
  ~Octree();

  const Box& range() const;
  size_t total_nodes() const;
  bool AddTriangle(const Triangle& t);
  bool RemoveTriangle(const Triangle& t);

  bool Intersects(const Octree& other) const;
  bool Intersects(const Triangle& t) const;
  bool IntersectsNotCoincident(const Octree& other) const;
  bool IntersectsNotCoincident(const Triangle& t) const;

 private:
  class Node;
  std::unique_ptr<Node> root_;
};

}  // namespace printer

#endif
