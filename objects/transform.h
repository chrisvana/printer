// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_OBJECTS_TRANSFORM_H__
#define _PRINTER_OBJECTS_TRANSFORM_H__

#include <memory>
#include "printer/base/geometry.h"
#include "printer/objects/object.h"

namespace printer {

class TransformObject : public PrintObject {
 public:
  TransformObject(PrintObject* obj, GeometryTransform* transform);  // owns objs
  virtual ~TransformObject();

  // Interface
  virtual float ISOValue(const Point& p);
  virtual bool BoundingBox(Box* b);
  virtual bool FullyContains(const Box& b);
  virtual bool ThreadSafe();

 private:
  std::unique_ptr<PrintObject> object_;
  std::unique_ptr<GeometryTransform> transform_;
};

class IntersectObject : public PrintObject {
 public:
  IntersectObject(PrintObject* object_a,
                  PrintObject* object_b);  // both owned
  virtual ~IntersectObject();

  // Interface
  virtual float ISOValue(const Point& p);
  virtual bool BoundingBox(Box* b);
  virtual bool FullyContains(const Box& b);
  virtual bool MayIntersectRegion(const Box& box);
  virtual bool ThreadSafe();

 private:
  std::unique_ptr<PrintObject> object_a_, object_b_;
};

class DifferenceObject : public PrintObject {
 public:
  DifferenceObject(PrintObject* object_a,
                   PrintObject* object_b);  // both owned
  virtual ~DifferenceObject();

  // Interface
  virtual float ISOValue(const Point& p);
  virtual bool BoundingBox(Box* b);
  virtual bool FullyContains(const Box& b);
  virtual bool MayIntersectRegion(const Box& box);
  virtual bool ThreadSafe();

 private:
  std::unique_ptr<PrintObject> object_a_, object_b_;
};

// NB: This is a boolean transform.
class SubtractObject : public PrintObject {
 public:
  SubtractObject(PrintObject* object_a,
                 PrintObject* object_b_to_remove_from_a);  // both owned
  virtual ~SubtractObject();

  // Interface
  virtual float ISOValue(const Point& p);
  virtual bool BoundingBox(Box* b);
  virtual bool FullyContains(const Box& b);
  virtual bool MayIntersectRegion(const Box& box);
  virtual bool ThreadSafe();

 private:
  std::unique_ptr<PrintObject> object_a_, object_b_;
};

class UnionObject : public PrintObject {
 public:
  UnionObject(PrintObject* object_a,
              PrintObject* object_b);
  virtual ~UnionObject();

  // Interface
  virtual float ISOValue(const Point& p);
  virtual bool BoundingBox(Box* b);
  virtual bool FullyContains(const Box& b);
  virtual bool MayIntersectRegion(const Box& box);
  virtual bool ThreadSafe();

 private:
  std::unique_ptr<PrintObject> object_a_, object_b_;
};

// NB: This is a boolean transform.
class InvertObject : public PrintObject {
 public:
  InvertObject(PrintObject* object_a);  // owned;
  virtual ~InvertObject();

  // Interface
  virtual float ISOValue(const Point& p);
  virtual bool BoundingBox(Box* b);
  virtual bool FullyContains(const Box& b);
  virtual bool MayIntersectRegion(const Box& box);
  virtual bool ThreadSafe();

 private:
  std::unique_ptr<PrintObject> object_;
};

}  // namespace printer

#endif  // _PRINTER_OBJECTS_TRANSFORM_H__
