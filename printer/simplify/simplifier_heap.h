// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_SIMPLIFY_SIMPLIFIER_HEAP_H__
#define _PRINTER_SIMPLIFY_SIMPLIFIER_HEAP_H__

#include <set>
#include <vector>
#include <unordered_set>
#include "printer/base/geometry.h"
#include "printer/base/mesh.h"

namespace printer {

class SimplifierHeap {
 public:
  struct Simplification {
    double score;
    Edge edge;
    Point new_vertex;
  };

  SimplifierHeap();
  ~SimplifierHeap();

  size_t size() const { return heap_.size(); }

  // Insertion
  void AddSimplification(Simplification* simplification);

  // Removal
  Simplification* PopBest();

  // Updating (called after you accept the removed simplification)
  void RemoveOverlapping(const TriangleMesh::TriangleList& old_triangles);

 private:
  struct InternalElement;
  struct PointElement {
    PointElement() {}
    PointElement(const Point& pin, InternalElement* ein)
        : p(pin), e(ein) {
    }
    bool operator<(const PointElement& other) const;
    static PointElement Begin(const Point& t);

    Point p;
    InternalElement* e;
  };

  // Heap operations
  void PushUp(int index);
  void PushDown(int index);

  // Cleanup
  Simplification* Erase(InternalElement* e);

  // Point -> InternalElement mapping.
  void GetPointRange(const Point& p, std::set<InternalElement*>* old);
  void AddPointElements(InternalElement* e);
  void ErasePointElements(InternalElement* e);

  std::vector<InternalElement*> heap_;
  std::set<PointElement> point_to_elem_;
};

inline bool SimplifierHeap::PointElement::operator<(
    const PointElement& other) const {
  return p != other.p ? p < other.p : e < other.e;
}

}  // namespace printer

#endif  // _PRINTER_SIMPLIFY_SIMPLIFIER_HEAP_H__
