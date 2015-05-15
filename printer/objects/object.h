// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_OBJECTS_OBJECT_H__
#define _PRINTER_OBJECTS_OBJECT_H__

#include "printer/base/geometry.h"

namespace printer {

class PrintObject {
 public:
  PrintObject() {}
  virtual ~PrintObject() {}

  // Interface
  virtual bool ContainsPoint(const Point& p) {
    return ISOValue(p) > 0;
  }
  virtual float ISOValue(const Point& p) = 0;
  virtual bool BoundingBox(Box* b) = 0;
  virtual bool MayIntersectRegion(const Box& box) {
    Box bounding;
    return !BoundingBox(&bounding) || bounding.Intersects(box);
  }
  virtual bool FullyContains(const Box& box) = 0;  // Will set region ISO=1.
  virtual bool ThreadSafe() = 0;

  // Optional.
  virtual void MaybeCleanupMemory() {}
};

}  // namespace printer

#endif  // _PRINTER_OBJECTS_OBJECT_H__
