// Copyright 2015
// Author: Christopher Van Arsdale

#include "common/log/log.h"  // tmp tmp.

#include "printer/base/geometry.h"
#include "printer/objects/primitives.h"

namespace printer {

RectangleObject::RectangleObject(const Box& box)
    : box_(box) {
}

RectangleObject::~RectangleObject() {
}

float RectangleObject::ISOValue(const Point& p) {
  return box_.Contains(p) ? 1 : 0;
}

bool RectangleObject::BoundingBox(Box* b) {
  *b = box_;
  return true;
}

bool RectangleObject::FullyContains(const Box& b) {
  return (box_.Contains(b.top()) &&
          box_.Contains(b.bottom()));
}

SphereObject::SphereObject(const Sphere& sphere)
    : sphere_(sphere) {
}

SphereObject::~SphereObject() {
}

float SphereObject::ISOValue(const Point& p) {
  return sphere_.Contains(p) ? 1 : 0;
}

bool SphereObject::BoundingBox(Box* b) {
  *b = sphere_.BoundingBox();
  return true;
}

bool SphereObject::FullyContains(const Box& b) {
  return (sphere_.Contains(b.corner1()) &&
          sphere_.Contains(b.corner2()) &&
          sphere_.Contains(b.corner3()) &&
          sphere_.Contains(b.corner4()) &&
          sphere_.Contains(b.corner5()) &&
          sphere_.Contains(b.corner6()) &&
          sphere_.Contains(b.corner7()) &&
          sphere_.Contains(b.corner8()));
}

CylinderObject::CylinderObject(const Box& bounds)
    : bounds_(bounds),
      bottom_center_(bounds.center()) {
  bottom_center_.set_z(0);
}

CylinderObject::~CylinderObject() {
}

float CylinderObject::ISOValue(const Point& p) {
  if (!bounds_.Contains(p)) {
    return 0;
  }
  Point r = Point(p.x(), p.y(), 0) - bottom_center_;
  double dx = (r.x()/(bounds_.size_x()/2));
  double dy = (r.y()/(bounds_.size_y()/2));
  double val = dx*dx + dy*dy;
  return val <= 1 ? 1 : 0;
}

bool CylinderObject::BoundingBox(Box* b) {
  *b = bounds_;
  return true;
}

bool CylinderObject::FullyContains(const Box& b) {
  return (bounds_.FullyContains(b) &&
          ContainsPoint(b.corner1()) &&
          ContainsPoint(b.corner2()) &&
          ContainsPoint(b.corner3()) &&
          ContainsPoint(b.corner4()) &&
          ContainsPoint(b.corner5()) &&
          ContainsPoint(b.corner6()) &&
          ContainsPoint(b.corner7()) &&
          ContainsPoint(b.corner8()));
}

SpheroidObject::SpheroidObject(const Box& bounds)
    : bounds_(bounds),
      center_(bounds.center()) {
  center_.set_z(0);
}

SpheroidObject::~SpheroidObject() {
}

float SpheroidObject::ISOValue(const Point& p) {
  if (!bounds_.Contains(p)) {
    return false;
  }
  Point r = p - center_;
  double dx = (r.x()/(bounds_.size_x()/2));
  double dy = (r.y()/(bounds_.size_y()/2));
  double dz = (r.z()/(bounds_.size_z()/2));
  double val = dx*dx + dy*dy + dz*dz;
  return val <= 1 ? 1 : 0;
}

bool SpheroidObject::BoundingBox(Box* b) {
  *b = bounds_;
  return true;
}

bool SpheroidObject::FullyContains(const Box& b) {
  return (bounds_.FullyContains(b) &&
          ContainsPoint(b.corner1()) &&
          ContainsPoint(b.corner2()) &&
          ContainsPoint(b.corner3()) &&
          ContainsPoint(b.corner4()) &&
          ContainsPoint(b.corner5()) &&
          ContainsPoint(b.corner6()) &&
          ContainsPoint(b.corner7()) &&
          ContainsPoint(b.corner8()));
}

}  // namespace printer
