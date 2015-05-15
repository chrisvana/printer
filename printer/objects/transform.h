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

 protected:
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

 protected:
  std::unique_ptr<PrintObject> object_a_, object_b_;
};

// NB: This is a semi-boolean transform.
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

 protected:
  std::unique_ptr<PrintObject> object_a_, object_b_;
};

// NB: This is a boolean transform.
class RemoveObject : public PrintObject {
 public:
  RemoveObject(PrintObject* object_a,
               PrintObject* object_b_to_remove_from_a);  // both owned
  virtual ~RemoveObject();

  // Interface
  virtual float ISOValue(const Point& p);
  virtual bool BoundingBox(Box* b);
  virtual bool FullyContains(const Box& b);
  virtual bool MayIntersectRegion(const Box& box);
  virtual bool ThreadSafe();

 protected:
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

 protected:
  std::unique_ptr<PrintObject> object_a_, object_b_;
};

class SmoothUnion : public UnionObject {
 public:
  SmoothUnion(PrintObject* a, PrintObject* b);
  virtual ~SmoothUnion();

  virtual float ISOValue(const Point& p);
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

 protected:
  std::unique_ptr<PrintObject> object_;
};

class AddObject : public UnionObject {
 public:
  AddObject(PrintObject* object_a, PrintObject* object_b);  // owned;
  virtual ~AddObject();

  virtual float ISOValue(const Point& p);
};

class SubtractObject : public RemoveObject {
 public:
  SubtractObject(PrintObject* object_a, PrintObject* object_b);  // owned;
  virtual ~SubtractObject();

  virtual float ISOValue(const Point& p);
};

class MultiplyObject : public IntersectObject {
 public:
  MultiplyObject(PrintObject* object_a, PrintObject* object_b);  // owned;
  virtual ~MultiplyObject();

  virtual float ISOValue(const Point& p);
};

class SmoothInvertObject : public InvertObject {
 public:
  SmoothInvertObject(PrintObject* object_a);  // owned;
  virtual ~SmoothInvertObject();

  virtual float ISOValue(const Point& p);
};

}  // namespace printer

#endif  // _PRINTER_OBJECTS_TRANSFORM_H__
