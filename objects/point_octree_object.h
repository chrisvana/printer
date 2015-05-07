// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_OBJECTS_POINT_OCTREE_OBJECT_H__
#define _PRINTER_OBJECTS_POINT_OCTREE_OBJECT_H__

#include <memory>
#include <string>
#include "printer/base/geometry.h"
#include "printer/objects/object.h"
#include "printer/base/point_octree.h"


namespace printer {

class PointOctreeObject : public PrintObject {
 public:
  typedef PointList::PointMode PointMode;
  static PointMode FromString(const std::string& name) {
    return PointList::FromString(name);
  }

  explicit PointOctreeObject(double horizontal_resolution,
                             double vertical_resolution,
                             PointMode mode,
                             PointOctree* octree);
  virtual ~PointOctreeObject();

  virtual float ISOValue(const Point& p);
  virtual bool BoundingBox(Box* b);
  virtual bool FullyContains(const Box& box) { return false; }
  virtual bool ThreadSafe() { return true; }

 private:
  PointMode mode_;
  Point expand_;
  std::unique_ptr<PointOctree> octree_;
};


}  // namespace printer

#endif  // _PRINTER_OBJECTS_POINT_OCTREE_OBJECT_H__
