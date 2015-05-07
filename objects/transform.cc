// Copyright 2015
// Author: Christopher Van Arsdale

#include <algorithm>
#include "printer/objects/transform.h"

#include "common/log/log.h"

namespace printer {

TransformObject::TransformObject(PrintObject* obj, GeometryTransform* trans)
    : object_(obj),
      transform_(trans) {
}

TransformObject::~TransformObject() {
}

float TransformObject::ISOValue(const Point& p) {
  return object_->ISOValue(transform_->InverseTransform(p));
}

bool TransformObject::ThreadSafe() {
  return object_->ThreadSafe();
}

namespace {
template <bool standard>
void FillMinMax(GeometryTransform* transform,
                const Point& p,
                Point* min_point,
                Point* max_point) {
  Point transformed = (standard ? transform->Transform(p) :
                       transform->InverseTransform(p));
  *min_point = Point(std::min(min_point->x(), transformed.x()),
                     std::min(min_point->y(), transformed.y()),
                     std::min(min_point->z(), transformed.z()));
  *max_point = Point(std::max(max_point->x(), transformed.x()),
                     std::max(max_point->y(), transformed.y()),
                     std::max(max_point->z(), transformed.z()));
}

template <bool standard>
Box TransformBox(GeometryTransform* transform,
                 const Box& b) {
    Point min_point = b.corner1();
    Point max_point = b.corner1();
    FillMinMax<standard>(transform, b.corner2(), &min_point, &max_point);
    FillMinMax<standard>(transform, b.corner3(), &min_point, &max_point);
    FillMinMax<standard>(transform, b.corner4(), &min_point, &max_point);
    FillMinMax<standard>(transform, b.corner5(), &min_point, &max_point);
    FillMinMax<standard>(transform, b.corner6(), &min_point, &max_point);
    FillMinMax<standard>(transform, b.corner7(), &min_point, &max_point);
    FillMinMax<standard>(transform, b.corner8(), &min_point, &max_point);
    return Box(min_point, max_point);
}
}  // anonymous namespace

bool TransformObject::BoundingBox(Box* b) {
  if (transform_->IsLinear() && object_->BoundingBox(b)) {
    *b = TransformBox<true>(transform_.get(), *b);
    return true;
  }
  return false;
}

bool TransformObject::FullyContains(const Box& b) {
  return transform_->IsLinear() &&
      object_->FullyContains(TransformBox<false>(transform_.get(), b));
}

IntersectObject::IntersectObject(PrintObject* object_a,
                                 PrintObject* object_b)
    : object_a_(object_a),
      object_b_(object_b) {
}

IntersectObject::~IntersectObject() {
}

float IntersectObject::ISOValue(const Point& p) {
  float a = object_a_->ISOValue(p);
  return a == 0 ? 0 : std::min(a, object_b_->ISOValue(p));
}

bool IntersectObject::BoundingBox(Box* out) {
  Box a, b;
  if (!object_a_->BoundingBox(&a) || !object_b_->BoundingBox(&b)) {
    return false;
  }
  if (!a.Intersects(b)) {
    *out = Box(Point(-1, -1, -1), Point(-1, -1, -1));
  } else {
    *out = Box(Point(std::max(a.bottom().x(), b.bottom().x()),
                     std::max(a.bottom().y(), b.bottom().y()),
                     std::max(a.bottom().z(), b.bottom().z())),
               Point(std::min(a.top().x(), b.top().x()),
                     std::min(a.top().y(), b.top().y()),
                     std::min(a.top().z(), b.top().z())));
  }
  return true;
}

bool IntersectObject::ThreadSafe() {
  return object_a_->ThreadSafe() && object_b_->ThreadSafe();
}

bool IntersectObject::FullyContains(const Box& b) {
  return object_a_->FullyContains(b) && object_b_->FullyContains(b);
}

bool IntersectObject::MayIntersectRegion(const Box& b) {
  return object_a_->MayIntersectRegion(b) && object_b_->MayIntersectRegion(b);
}

DifferenceObject::DifferenceObject(PrintObject* object_a,
                                   PrintObject* object_b)
    : object_a_(object_a),
      object_b_(object_b) {
}

DifferenceObject::~DifferenceObject() {
}

float DifferenceObject::ISOValue(const Point& p) {
  float a = object_a_->ISOValue(p);
  float b = object_b_->ISOValue(p);
  return (a == 0 ? b : (b == 0 ? a : 0));
}

bool DifferenceObject::BoundingBox(Box* out) {
  Box a, b;
  if (!object_a_->BoundingBox(&a) || !object_b_->BoundingBox(&b)) {
    return false;
  }
  *out = Box(Point(std::min(a.bottom().x(), b.bottom().x()),
                   std::min(a.bottom().y(), b.bottom().y()),
                   std::min(a.bottom().z(), b.bottom().z())),
             Point(std::max(a.top().x(), b.top().x()),
                   std::max(a.top().y(), b.top().y()),
                   std::max(a.top().z(), b.top().z())));
  return true;
}

bool DifferenceObject::FullyContains(const Box& input) {
  Box a, b;
  if (!object_a_->BoundingBox(&a) || !object_b_->BoundingBox(&b)) {
    return false;
  }
  return ((!a.Intersects(input) && object_b_->FullyContains(input)) ||
          (!b.Intersects(input) && object_a_->FullyContains(input)));
}

bool DifferenceObject::MayIntersectRegion(const Box& b) {
  return (object_a_->MayIntersectRegion(b) ||
          object_b_->MayIntersectRegion(b));
}

bool DifferenceObject::ThreadSafe() {
  return object_a_->ThreadSafe() && object_b_->ThreadSafe();
}

SubtractObject::SubtractObject(PrintObject* object_a,
                               PrintObject* object_b_to_remove_from_a)
    : object_a_(object_a),
      object_b_(object_b_to_remove_from_a) {
}

SubtractObject::~SubtractObject() {
}

float SubtractObject::ISOValue(const Point& p) {
  float a = object_a_->ISOValue(p);
  return (a > 0 && !object_b_->ContainsPoint(p) ? a : 0);
}

bool SubtractObject::BoundingBox(Box* out) {
  return object_a_->BoundingBox(out);
}

bool SubtractObject::FullyContains(const Box& input) {
  return (object_a_->FullyContains(input) &&
          !object_b_->MayIntersectRegion(input));
}

bool SubtractObject::MayIntersectRegion(const Box& b) {
  return (object_a_->MayIntersectRegion(b) &&
          (!object_b_->MayIntersectRegion(b) || !object_b_->FullyContains(b)));
}

bool SubtractObject::ThreadSafe() {
  return object_a_->ThreadSafe() && object_b_->ThreadSafe();
}

UnionObject::UnionObject(PrintObject* object_a,
                         PrintObject* object_b)
    : object_a_(object_a),
      object_b_(object_b) {
}

UnionObject::~UnionObject() {
}

float UnionObject::ISOValue(const Point& p) {
  float a = object_a_->ISOValue(p);
  return a == 1 ? 1 : std::max(a, object_b_->ISOValue(p));
}

bool UnionObject::BoundingBox(Box* out) {
  Box a, b;
  if (!object_a_->BoundingBox(&a) || !object_b_->BoundingBox(&b)) {
    return false;
  }
  *out = Box(Point(std::min(a.bottom().x(), b.bottom().x()),
                   std::min(a.bottom().y(), b.bottom().y()),
                   std::min(a.bottom().z(), b.bottom().z())),
             Point(std::max(a.top().x(), b.top().x()),
                   std::max(a.top().y(), b.top().y()),
                   std::max(a.top().z(), b.top().z())));
  return true;
}

bool UnionObject::FullyContains(const Box& input) {
  return object_a_->FullyContains(input) || object_b_->FullyContains(input);
}

bool UnionObject::MayIntersectRegion(const Box& b) {
  return (object_a_->MayIntersectRegion(b) ||
          object_b_->MayIntersectRegion(b));
}

bool UnionObject::ThreadSafe() {
  return object_a_->ThreadSafe() && object_b_->ThreadSafe();
}

InvertObject::InvertObject(PrintObject* object)
    : object_(object) {
}

InvertObject::~InvertObject() {
}

float InvertObject::ISOValue(const Point& p) {
  return object_->ISOValue(p) == 0 ? 1 : 0;
}

bool InvertObject::BoundingBox(Box* out) {
  return false;
}

bool InvertObject::FullyContains(const Box& input) {
  Box box;
  if (!object_->BoundingBox(&box)) {
    return false;
  }
  return !box.Intersects(input);
}

bool InvertObject::MayIntersectRegion(const Box& input) {
  return !object_->FullyContains(input);
}

bool InvertObject::ThreadSafe() {
  return object_->ThreadSafe();
}

}  // namespace printer
