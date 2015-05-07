// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_OBJECTS_PRIMITIVES_H__
#define _PRINTER_OBJECTS_PRIMITIVES_H__

#include "printer/base/geometry.h"
#include "printer/objects/object.h"

namespace printer {

class RectangleObject : public PrintObject {
 public:
  explicit RectangleObject(const Box& box);
  virtual ~RectangleObject();

  // Interface
  virtual float ISOValue(const Point& p);
  virtual bool BoundingBox(Box* b);
  virtual bool FullyContains(const Box& b);
  virtual bool ThreadSafe() { return true; }

 private:
  Box box_;
};

class SphereObject : public PrintObject {
 public:
  explicit SphereObject(const Sphere& sphere);
  virtual ~SphereObject();

  // Interface
  virtual float ISOValue(const Point& p);
  virtual bool BoundingBox(Box* b);
  virtual bool FullyContains(const Box& b);
  virtual bool ThreadSafe() { return true; }

 private:
  Sphere sphere_;
};

class CylinderObject : public PrintObject {
 public:
  explicit CylinderObject(const Box& bounds);
  virtual ~CylinderObject();

  // Interface
  virtual float ISOValue(const Point& p);
  virtual bool BoundingBox(Box* b);
  virtual bool FullyContains(const Box& b);
  virtual bool ThreadSafe() { return true; }

 private:
  Box bounds_;
  Point bottom_center_;
};

class SpheroidObject : public PrintObject {
 public:
  explicit SpheroidObject(const Box& bounds);
  virtual ~SpheroidObject();

  // Interface
  virtual float ISOValue(const Point& p);
  virtual bool BoundingBox(Box* b);
  virtual bool FullyContains(const Box& b);
  virtual bool ThreadSafe() { return true; }

 private:
  Box bounds_;
  Point center_;
};

}  // namespace printer

#endif  // _PRINTER_OBJECTS_PRIMITIVES_H__
