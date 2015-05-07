// Copyright 2015
// Author: Christopher Van Arsdale

#include <vector>
#include <set>
#include "common/log/log.h"
#include "printer/base/mesh.h"
#include "printer/simplify/simplifier_heap.h"

using std::set;
using std::vector;

namespace printer {

struct SimplifierHeap::InternalElement {
  explicit InternalElement(SimplifierHeap::Simplification* s)
      : simplification(s), index(-1) {
  }

  Simplification* simplification;
  int index;

  bool operator<(const InternalElement& other) const {
    return (simplification->score != other.simplification->score ?
            simplification->score < other.simplification->score :
            simplification->edge < other.simplification->edge);
  }
};

// static
SimplifierHeap::PointElement SimplifierHeap::PointElement::Begin(
    const Point& p) {
  return PointElement(p, NULL);
}

SimplifierHeap::SimplifierHeap() {
}

SimplifierHeap::~SimplifierHeap() {
  for (InternalElement* e : heap_) {
    delete e->simplification;
    delete e;
  }
}

void SimplifierHeap::AddSimplification(Simplification* simplification) {
  InternalElement* elem = new InternalElement(simplification);
  elem->index = heap_.size();
  heap_.push_back(elem);
  AddPointElements(elem);
  PushUp(heap_.size() - 1);
}

SimplifierHeap::Simplification* SimplifierHeap::PopBest() {
  if (heap_.empty()) {
    return NULL;
  }
  return Erase(heap_[0]);
}

void SimplifierHeap::RemoveOverlapping(
    const TriangleMesh::TriangleList& triangles) {
  // Find outdated internal elements.
  set<InternalElement*> old;
  for (const Triangle& t : triangles) {
    GetPointRange(t.p0(), &old);
    GetPointRange(t.p1(), &old);
    GetPointRange(t.p2(), &old);
  }

  // Erase them all
  for (InternalElement* e : old) {
    delete Erase(e);
  }
}

void SimplifierHeap::PushUp(int index) {
  while (true) {
    int p_index = index/2;
    InternalElement* elem = heap_[index];
    InternalElement* parent = heap_[p_index];
    if (*parent < *elem) {
      heap_[index] = parent;
      parent->index = index;
      heap_[p_index] = elem;
      elem->index = p_index;
      index = p_index;
      continue;
    }
    return;
  }
}

void SimplifierHeap::PushDown(int index) {
  while (true) {
    InternalElement* elem = heap_[index];
    
    int c1_index = index * 2;
    if (c1_index < heap_.size()) {
      InternalElement* child = heap_[c1_index];
      if (*elem < *child) {
        heap_[index] = child;
        child->index = index;
        heap_[c1_index] = elem;
        elem->index = c1_index;
        index = c1_index;
        continue;
      }
    }

    int c2_index = c1_index + 1;
    if (c2_index < heap_.size()) {
      InternalElement* child = heap_[c2_index];
      if (*elem < *child) {
        heap_[index] = child;
        child->index = index;
        heap_[c2_index] = elem;
        elem->index = c2_index;
        index = c2_index;
        continue;
      }
    }
    return;
  }
}

SimplifierHeap::Simplification* SimplifierHeap::Erase(InternalElement* e) {
  CHECK_NE(e->index, -1);
  CHECK_LT(e->index, heap_.size());

  // Update the heap to remove "e".
  InternalElement* back = heap_.back();
  heap_[e->index] = back;
  back->index = e->index;
  heap_.resize(heap_.size() - 1);
  if (heap_.size() > 0) {
    PushDown(e->index);
  }

  // Remove triangle -> InternalElement map.
  ErasePointElements(e);

  // Finish and returns Simplification
  Simplification* out = e->simplification;
  delete e;
  return out;
}

void SimplifierHeap::GetPointRange(const Point& point,
                                   set<InternalElement*>* old) {
  for (auto it = point_to_elem_.lower_bound(PointElement::Begin(point));
       it != point_to_elem_.end() && it->p == point; ++it) {
    old->insert(it->e);
  }
}

void SimplifierHeap::AddPointElements(InternalElement* e) {
  point_to_elem_.insert(PointElement(e->simplification->edge.p0(), e));
  point_to_elem_.insert(PointElement(e->simplification->edge.p1(), e));
}

void SimplifierHeap::ErasePointElements(InternalElement* e) {
  CHECK_EQ(1, point_to_elem_.erase(PointElement(
      e->simplification->edge.p0(), e)));
  CHECK_EQ(1, point_to_elem_.erase(PointElement(
      e->simplification->edge.p1(), e)));
}

}  // namespace printer
