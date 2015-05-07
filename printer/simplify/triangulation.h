// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_SIMPLIFY_TRIANGULATION_H__
#define _PRINTER_SIMPLIFY_TRIANGULATION_H__

#include <vector>
#include "printer/base/geometry.h"

namespace printer {

class Triangulation {
 public:
  typedef std::vector<Point> PolygonLoop;

  Triangulation(const Point& u, const Point& v) : u_(u), v_(v), loop_(NULL) {}
  ~Triangulation() {}
  void SetOuterLoop(const PolygonLoop* loop) { loop_ = loop; }
  void AddHole(const PolygonLoop* hole) { holes_.push_back(hole); }
  bool Triangulate(std::vector<Triangle>* output) const;

 private:
  size_t num_loops() const { return 1 + holes_.size(); }
  const PolygonLoop& loop(int i) const {
    return i == 0 ? *loop_ : *holes_[i - 1];
  }

  Point u_, v_;
  const PolygonLoop* loop_;
  std::vector<const PolygonLoop*> holes_;
};

}  // namespace printer

#endif  // _PRINTER_SIMPLIFY_TRIANGULATION_H__
