// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_BASE_GEOMETRY_H__
#define _PRINTER_BASE_GEOMETRY_H__

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace printer {

class Box;
class Range;

class Point {
 public:
  Point() : x_(0), y_(0), z_(0) {}
  Point(double x, double y, double z) : x_(x), y_(y), z_(z) {}
  ~Point() {}

  // Accessors.
  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  double magnitude2() const;  // squared.
  double magnitude() const;

  // Mutators
  void set_x(double val) { x_ = val; }
  void set_y(double val) { y_ = val; }
  void set_z(double val) { z_ = val; }
  void set(double x, double y, double z) { x_ = x; y_ = y; z_ = z; }

  void add(const Point& other);
  void sub(const Point& other);

  void scale_x(double s) { x_ *= s; }
  void scale_y(double s) { y_ *= s; }
  void scale_z(double s) { z_ *= s; }
  void scale(double s) { scale_x(s); scale_y(s); scale_z(s); }

  void Normalize();
  void RotateAround(const Point& vec, double angle_radians);

  // Arithmetic.
  Point operator+(const Point& other) const;
  Point operator-(const Point& other) const;
  Point operator*(double s) const;
  Point operator/(double s) const;

  // Dot product
  double operator*(const Point& other) const;

  // Cross
  Point Cross(const Point& other) const;

  // Comparisons (x then y then z).
  bool operator<(const Point& other) const;
  bool operator<=(const Point& other) const;
  bool operator>=(const Point& other) const;
  bool operator==(const Point& other) const;
  bool operator!=(const Point& other) const;
  bool ApproxEqual(const Point& other, double epsilon) const;

  // Other functions.
  double DistanceFrom(const Point& other) const;
  Point Normalized() const;
  std::string DebugString() const;
  std::string FullString() const;

 private:
  double x_, y_, z_;
};

// [start, end)
class Range {
 public:
  Range() : start_(0), end_(0) {}
  Range(double s, double e) : start_(s), end_(e) {}
  static Range FromValues(double v1, double v2) {
    return Range(std::min(v1, v2), std::max(v1, v2));
  }
  static Range FromValues(double v1, double v2, double v3) {
    return Range(std::min(std::min(v1, v2), v3),
                 std::max(std::max(v1, v2), v3));
  }

  // Accessors
  double start() const { return start_; }
  double end() const { return end_; }
  bool empty() const { return start_ == end_; }
  bool contains(double val) const { return start_ <= val && end_ > val; }
  bool contains_closed(double val) const {
    return start_ <= val && end_ >= val;
  }
  double size() const { return end_ - start_; }

  // Operators
  Range& operator=(const Range& other) {
    start_ = other.start_;
    end_ = other.end_;
    return *this;
  }

  // Mutators
  void set_start(double start) { start_ = start; }
  void set_end(double end) { end_ = end; }
  void set_range(double start, double end) { start_ = start; end_ = end; }
  void Merge(const Range& other) {
    if (empty()) {
      *this = other;
    } else if (!other.empty()) {
      start_ = std::min(start_, other.start_);
      end_ = std::max(end_, other.end_);
    }
  }

  std::string DebugString() const;

 private:
  double start_, end_;
};

class Edge {
 public:
  Edge() {}
  Edge(const Point& p0, const Point& p1) : p0_(p0), p1_(p1) {}
  ~Edge() {}

  Edge flipped() const { return Edge(p1_, p0_); }
  Point direction() const { return p1_ - p0_; }
  Edge Deduped() const { return p0_ < p1_ ? *this : flipped(); }
  const Point& p0() const { return p0_; }
  const Point& p1() const { return p1_; }
  Range x_range() const { return Range::FromValues(p0_.x(), p1_.x()); }
  Range y_range() const { return Range::FromValues(p0_.y(), p1_.y()); }
  Range z_range() const { return Range::FromValues(p0_.z(), p1_.z()); }
  double length() const { return direction().magnitude(); }
  bool is_single_point() const { return p0_ == p1_; }

  void set_p0(const Point& p) { p0_ = p; }
  void set_p1(const Point& p) { p1_ = p; }

  bool operator==(const Edge& other) const {
    return p0_ == other.p0_ && p1_ == other.p1_;
  }
  bool operator!=(const Edge& other) const {
    return p0_ != other.p0_ && p1_ != other.p1_;
  }
  void operator=(const Edge& other) { p0_ = other.p0_; p1_ = other.p1_; }
  bool operator<(const Edge& other) const {
    return (p0_ != other.p0_ ? p0_ < other.p0_ : p1_ < other.p1_);
  }

  std::string DebugString() const;

 private:
  Point p0_, p1_;
};

class FixedTriangle;

class Triangle {
 public:
  Triangle() {}
  explicit Triangle(const Point& p0, const Point& p1, const Point& p2)
      : p0_(p0), p1_(p1), p2_(p2) {
  }
  ~Triangle() {}

  // Accessors
  const Point& p0() const { return p0_; }
  const Point& p1() const { return p1_; }
  const Point& p2() const { return p2_; }
  Edge e0() const { return Edge(p0_, p1_); }
  Edge e1() const { return Edge(p1_, p2_); }
  Edge e2() const { return Edge(p2_, p0_); }
  Point normal() const { return (p1_ - p0_).Cross(p2_ - p0_); }
  Range x_range() const { return Range::FromValues(p0_.x(), p1_.x(), p2_.x()); }
  Range y_range() const { return Range::FromValues(p0_.y(), p1_.y(), p2_.y()); }
  Range z_range() const { return Range::FromValues(p0_.z(), p1_.z(), p2_.z()); }
  Box BoundingBox() const;
  double area() const { return (p1_ - p0_).Cross(p2_ - p0_).magnitude() / 2; }

  // Mutators
  void set_p0(const Point& p) { p0_ = p; }
  void set_p1(const Point& p) { p1_ = p; }
  void set_p2(const Point& p) { p2_ = p; }
  Point* mutable_p0() { return &p0_; }
  Point* mutable_p1() { return &p1_; }
  Point* mutable_p2() { return &p2_; }

  // Geometry operations
  bool WellFormed() const {
    return p0() != p1() && p1() != p2() && p2() != p0();
  }
  bool Intersects(const Triangle& other) const {
    return IntersectsEpsilon(other, kDefaultIntersectEpsilon);
  }
  bool IntersectsEpsilon(const Triangle& other, double epsilon) const;
  bool get_z_value(double x, double y, double* z) const;
  bool SharesOpposingHalfedge(const Triangle& other) const;

  // This is a funky definition of intersection. Single point intersections
  // (e.g. point against plane, or any point-point) do not count. Also,
  // half-edge intersections do not count if they would form a valid mesh (i.e.
  // the edges are going in opposite directions).
  bool IntersectsNotCoincident(const Triangle& other) const {
    return IntersectsNotCoincidentEpsilon(other, kDefaultIntersectEpsilon);
  }
  bool IntersectsNotCoincidentEpsilon(const Triangle& other,
                                      double epsilon) const;

  // Shuffles points so that minimum point is first.
  Triangle Normalized() const;
  Triangle Shuffled() const;
  Triangle Flipped() const;

  // Operators
  void operator=(const Triangle& other);
  bool operator<(const Triangle& other) const;
  bool operator==(const Triangle& other) const;
  bool operator!=(const Triangle& other) const;
  Triangle operator-(const Point& offset) const;
  Triangle operator+(const Point& offset) const;

  std::string DebugString() const;
  static const double kDefaultIntersectEpsilon;

 protected:
  // Intersection helpers.
  struct PlaneIntersection {
    Point normal;
    double p0_dist, p1_dist, p2_dist;
  };
  bool PlaneIntersects(const Triangle& other,
                       double epsilon,
                       PlaneIntersection& results) const;
  static Edge GetPlaneSegment(const Triangle& other,
                              const PlaneIntersection& sect);

  template <bool coincident, typename T>
  static bool IntersectsInternal(const T& t,
                                 const T& other,
                                 double epsilon);
  template <bool coincident, typename T>
  static bool IntersectsComplicated(const T& t,
                                    const T& other,
                                    double epsilon);

  static bool IntersectsTriangle2D(bool coincident_ok,
                                   double epsilon,
                                   const Triangle& t,
                                   const Triangle& other);
  static bool IntersectsTriangle3D(bool coincident_ok,
                                   double epsilon,
                                   const Triangle& t,
                                   const Triangle& other,
                                   const PlaneIntersection& t_on_other_plane,
                                   const PlaneIntersection& other_on_t_plane);
  static bool IntersectsTriangle2D(bool coincident_ok,
                                   double epsilon,
                                   const FixedTriangle& t,
                                   const FixedTriangle& other);
  static bool IntersectsTriangle3D(bool coincident_ok,
                                   double epsilon,
                                   const FixedTriangle& t,
                                   const FixedTriangle& other,
                                   const PlaneIntersection& t_on_other_plane,
                                   const PlaneIntersection& other_on_t_plane);

  template <bool coincident_intersects, typename T>
  static bool IntersectsTriangle2D(double epsilon,
                                   const T& t,
                                   const T& other);

  template <bool coincident_intersects, typename T>
  static bool IntersectsTriangle3D(double epsilon,
                                   const T& t,
                                   const T& other,
                                   const PlaneIntersection& t_on_other_plane,
                                   const PlaneIntersection& other_on_t_plane);
  Point p0_, p1_, p2_;
};

class Box {
 public:
  Box() {}
  Box(const Point& a, const Point& b) {
    bottom_ = Point(std::min(a.x(), b.x()), std::min(a.y(), b.y()),
                    std::min(a.z(), b.z()));
    top_ = Point(std::max(a.x(), b.x()), std::max(a.y(), b.y()),
                 std::max(a.z(), b.z()));
  }
  ~Box() {}

  const Point& bottom() const { return bottom_; }
  const Point& top() const { return top_; }
  Point center() const {
    return Point((bottom_.x() + top_.x()) / 2,
                 (bottom_.y() + top_.y()) / 2,
                 (bottom_.z() + top_.z()) / 2);
  }
  Range x_range() const { return Range(bottom_.x(), top_.x()); }
  Range y_range() const { return Range(bottom_.y(), top_.y()); }
  Range z_range() const { return Range(bottom_.z(), top_.z()); }

  double size_x() const { return top_.x() - bottom_.x(); }
  double size_y() const { return top_.y() - bottom_.y(); }
  double size_z() const { return top_.z() - bottom_.z(); }
  double volume() const { return size_x() * size_y() * size_z(); }

  // Corners of box
  Point corner1() const { return bottom_; }
  Point corner2() const { return Point(bottom_.x(), bottom_.y(), top_.z()); }
  Point corner3() const { return Point(bottom_.x(), top_.y(), top_.z()); }
  Point corner4() const { return Point(bottom_.x(), top_.y(), bottom_.z()); }
  Point corner5() const { return top_; }
  Point corner6() const { return Point(top_.x(), bottom_.y(), top_.z()); }
  Point corner7() const { return Point(top_.x(), bottom_.y(), bottom_.z()); }
  Point corner8() const { return Point(top_.x(), top_.y(), bottom_.z()); }

  bool Contains(const Point& p) const {
    return (p.x() <= top_.x() && p.y() <= top_.y() && p.z() <= top_.z() &&
            p.x() >= bottom_.x() && p.y() >= bottom_.y() &&
            p.z() >= bottom_.z());
  }
  bool FullyContains(const Box& b) const {
    return (Contains(b.corner1()) &&
            Contains(b.corner2()) &&
            Contains(b.corner3()) &&
            Contains(b.corner4()) &&
            Contains(b.corner5()) &&
            Contains(b.corner6()) &&
            Contains(b.corner7()) &&
            Contains(b.corner8()));
  }
  bool FullyContains(const Triangle& c) const {
    return Contains(c.p0()) && Contains(c.p1()) && Contains(c.p2());
  }
  
  // Scale.
  void scale_x(double s) { bottom_.scale_x(s); top_.scale_x(s); }
  void scale_y(double s) { bottom_.scale_y(s); top_.scale_y(s); }
  void scale_z(double s) { bottom_.scale_z(s); top_.scale_z(s); }
  void scale(double s) { bottom_.scale(s); top_.scale(s); }

  void UnionWith(const Box& other);
  Box Unioned(const Box& other) const {
    Box copy = *this;
    copy.UnionWith(other);
    return copy;
  }
  void IntersectWith(const Box& other);
  Box IntersectedWith(const Box& other) const {
    Box copy = *this;
    copy.IntersectWith(other);
    return copy;
  }

  void UnionWith(const Point& point);
  Box Unioned(const Point& point) const {
    Box copy = *this;
    copy.UnionWith(point);
    return copy;
  }

  // Intersection
  bool Intersects(const Box& other) const;
  bool Intersects(const Triangle& triangle) const;
  bool Intersects(const FixedTriangle& triangle) const;
  bool IntersectsPlane(const Triangle& plane) const;
  bool IntersectsPlane(const FixedTriangle& plane) const;

  // Operators
  bool operator==(const Box& other) const {
    return bottom() == other.bottom() && top() == other.top();
  }
  bool operator!=(const Box& other) const {
    return bottom() != other.bottom() || top() != other.top();
  }
  Box operator+(const Point& offset) const {
    return Box(bottom() + offset, top() + offset);
  }
  Box operator-(const Point& offset) const {
    return Box(bottom() - offset, top() - offset);
  }

  std::string DebugString() const;

 private:
  template <typename T>
  bool IntersectsTriangle(const T& triangle) const;
  template <typename T>
  bool IntersectsPlaneInternal(const T& plane,
                               Point& normal,
                               Point& pos) const;
  Point bottom_, top_;
};

class Point2D {
 public:
  Point2D() : x_(0), y_(0) {}
  Point2D(double x, double y) : x_(x), y_(y) {}
  ~Point2D() {}

  // Operators.
  Point2D operator+(const Point2D& o) const {
    return Point2D(x_ + o.x_, y_ + o.y_);
  }
  Point2D operator-(const Point2D& o) const {
    return Point2D(x_ - o.x_, y_ - o.y_);
  }
  Point2D operator*(double d) const {
    return Point2D(x_ * d, y_ * d);
  }
  Point2D operator/(double d) const {
    return Point2D(x_ / d, y_ / d);
  }
  bool operator<(const Point2D& other) const {
    return x_ != other.x_ ? x_ < other.x_ : y_ < other.y_;
  }
  bool operator==(const Point2D& other) const {
    return x_ == other.x_ && y_ == other.y_;
  }
  bool operator!=(const Point2D& other) const {
    return x_ != other.x_ || y_ != other.y_;
  }

  // Accessors
  double x() const { return x_; }
  double y() const { return y_; }
  double magnitude() const { return sqrt(magnitude2()); }
  double magnitude2() const { return x_ * x_ + y_ * y_; }
  Point2D Normalized() const {
    return *this / magnitude();
  }
  double z_cross(const Point2D& o) const {
    return (o == *this ? 0 : x_ * o.y_ - y_ * o.x_);
  }
  bool LeftOf(const Point2D& o) const { return x_ < o.x_; }
  bool RightOf(const Point2D& o) const { return x_ > o.x_; }
  bool Below(const Point2D& o) const { return y_ < o.y_; }
  bool Above(const Point2D& o) const { return y_ > o.y_; }

  // Mutators
  void set_x(double x) { x_ = x; }
  void set_y(double y) { y_ = y; }
  void Normalize() { *this = Normalized(); }

  std::string DebugString() const;
  std::string FullString() const;

 private:
  double x_, y_;
};

class Edge2D {
 public:
  Edge2D() {}
  Edge2D(const Point2D& p0, const Point2D& p1) : p0_(p0), p1_(p1) {}
  ~Edge2D() {}

  Edge2D flipped() const { return Edge2D(p1_, p0_); }
  Point2D direction() const { return p1_ - p0_; }
  Edge2D Deduped() const { return p0_ < p1_ ? *this : flipped(); }
  const Point2D& p0() const { return p0_; }
  const Point2D& p1() const { return p1_; }
  Range x_range() const { return Range::FromValues(p0_.x(), p1_.x()); }
  Range y_range() const { return Range::FromValues(p0_.y(), p1_.y()); }
  double length() const { return direction().magnitude(); }
  bool is_single_point() const { return p0_ == p1_; }

  void set_p0(const Point2D& p) { p0_ = p; }
  void set_p1(const Point2D& p) { p1_ = p; }

  bool operator==(const Edge2D& other) const {
    return p0_ == other.p0_ && p1_ == other.p1_;
  }
  bool operator!=(const Edge2D& other) const {
    return p0_ != other.p0_ && p1_ != other.p1_;
  }
  void operator=(const Edge2D& other) { p0_ = other.p0_; p1_ = other.p1_; }
  bool operator<(const Edge2D& other) const {
    return (p0_ != other.p0_ ? p0_ < other.p0_ : p1_ < other.p1_);
  }

  std::string DebugString() const;

 private:
  Point2D p0_, p1_;
};

class Triangle2D {
 public:
  Triangle2D(const Point2D a, const Point2D& b, const Point2D& c)
      : p0_(a), p1_(b), p2_(c) {
  }
  ~Triangle2D() {}

  static Triangle2D FromAxes(const Triangle& input,
                             const Point& u,
                             const Point& v,
                             double epsilon);
  static Triangle2D FromAxes(const Triangle& input,
                             const Point& u,
                             const Point& v) {
    return FromAxes(input, u, v, Triangle::kDefaultIntersectEpsilon);
  }

  Triangle ToAxes(const Point& u, const Point& v) const;

  const Point2D& p0() const { return p0_; }
  const Point2D& p1() const { return p1_; }
  const Point2D& p2() const { return p2_; }
  bool CounterClockwise() const { return (p1_ - p0_).z_cross(p2_ - p0_) > 0; }
  Triangle2D Flipped() const { return Triangle2D(p0_, p2_, p1_); }
  Triangle2D Normalized() const;
  std::string DebugString() const;

  void set_p0(const Point2D& p0) { p0_ = p0; }
  void set_p1(const Point2D& p1) { p1_ = p1; }
  void set_p2(const Point2D& p2) { p2_ = p2; }

  bool operator==(const Triangle2D& other) const {
    return p0_ == other.p0_ && p1_ == other.p1_ && p2_ == other.p2_;
  }
  bool operator<(const Triangle2D& other) const;

 private:
  Point2D p0_, p1_, p2_;
};

class FixedTriangle : public Triangle {
 public:
  FixedTriangle() {
    ReCompute();
  }
  FixedTriangle(const Triangle& t) : Triangle(t) {
    ReCompute();
  }
  FixedTriangle(const Point& p0, const Point& p1, const Point& p2)
      : Triangle(p0, p1, p2) {
    ReCompute();
  }

  void ReCompute();

  const Point& normal() const { return normal_; }
  const Box& BoundingBox() const { return box_; }
  size_t hash() const { return hash_; }

  bool Intersects(const FixedTriangle& other) const {
    return IntersectsEpsilon(other, kDefaultIntersectEpsilon);
  }
  bool IntersectsNotCoincident(const FixedTriangle& other) const {
    return IntersectsNotCoincidentEpsilon(other, kDefaultIntersectEpsilon);
  }

  bool IntersectsEpsilon(const FixedTriangle& other,
                         double epsilon) const;
  bool IntersectsNotCoincidentEpsilon(const FixedTriangle& other,
                                      double epsilon) const;

 private:
  Point normal_;
  Box box_;
  size_t hash_;
};

class Sphere {
 public:
  Sphere() : radius_(0) {}
  Sphere(const Point& center, double r) : center_(center), radius_(r) {}
  ~Sphere() {}

  const Point& center() const { return center_; }
  double radius() const { return radius_; }
  bool Contains(const Point& point) const {
    return point.DistanceFrom(center_) <= radius_;
  }
  Box BoundingBox() const {
    return Box(center_ + Point(radius_, radius_, radius_),
               center_ - Point(radius_, radius_, radius_));
  }

  void set_center(const Point& c) { center_ = c; }
  void set_radius(double r) { radius_ = r; }

 private:
  Point center_;
  double radius_;
};

class GeometryTransform {
 public:
  GeometryTransform() {}
  virtual ~GeometryTransform() {}
  virtual Point Transform(const Point& input) = 0;
  virtual Point InverseTransform(const Point& input) = 0;
  virtual bool IsLinear() const { return true; }
};

class ScaleTransform : public GeometryTransform {
 public:
  explicit ScaleTransform(const Point& scale)
      : scale_(scale),
        inv_scale_(Point(1.0/scale.x(), 1.0/scale.y(), 1.0/scale.z())) {
  }
  explicit ScaleTransform(const double& s)
      : scale_(Point(s, s, s)),
        inv_scale_(Point(1.0/s, 1.0/s, 1.0/s)) {
  }
  virtual ~ScaleTransform() {}
  virtual Point Transform(const Point& input) {
    Point copy = input;
    copy.scale_x(scale_.x());
    copy.scale_x(scale_.y());
    copy.scale_x(scale_.z());
    return copy;
  }
  virtual Point InverseTransform(const Point& input) {
    Point copy = input;
    copy.scale_x(inv_scale_.x());
    copy.scale_x(inv_scale_.y());
    copy.scale_x(inv_scale_.z());
    return copy;
  }

 private:
  Point scale_, inv_scale_;
};

class TranslateTransform : public GeometryTransform {
 public:
  explicit TranslateTransform(const Point& translate) : translate_(translate) {}
  virtual ~TranslateTransform() {}
  virtual Point Transform(const Point& input) {
    Point copy = input;
    copy.add(translate_);
    return copy;
  }
  virtual Point InverseTransform(const Point& input) {
    Point copy = input;
    copy.sub(translate_);
    return copy;
  }

 private:
  Point translate_;
};

class RotateTransform : public GeometryTransform {
 public:
  RotateTransform(const Point& vec, double radians)
      : origin_(Point(0, 0, 0)), vector_(vec), radians_(radians) {
  }
  RotateTransform(const Point& center, const Point& vec, double radians)
      : origin_(center), vector_(vec), radians_(radians) {
  }
  virtual ~RotateTransform() {}
  virtual Point Transform(const Point& input) {
    Point copy = input;
    copy.sub(origin_);
    copy.RotateAround(vector_, radians_);
    copy.add(origin_);
    return copy;
  }
  virtual Point InverseTransform(const Point& input) {
    Point copy = input;
    copy.sub(origin_);
    copy.RotateAround(vector_, -radians_);
    copy.add(origin_);
    return copy;
  }

 private:
  Point origin_, vector_;
  double radians_;
};

class SetTransform : public GeometryTransform {
 public:
  SetTransform() {}
  virtual ~SetTransform() {
    for (GeometryTransform* t : transforms_) {
      delete t;
    }
  }
  void Add(GeometryTransform* t) { transforms_.push_back(t); }

  virtual Point Transform(const Point& input) {
    Point copy = input;
    for (GeometryTransform* t : transforms_) {
      copy = t->Transform(copy);
    }
    return copy;
  }
  virtual Point InverseTransform(const Point& input) {
    Point copy = input;
    for (int i = transforms_.size() - 1; i >=0; --i) {
      copy = transforms_[i]->InverseTransform(copy);
    }
    return copy;
  }

 private:
  std::vector<GeometryTransform*> transforms_;
};

inline Point Point::operator+(const Point& other) const {
  Point copy = *this;
  copy.add(other);
  return copy;
}

inline Point Point::operator-(const Point& other) const {
  Point copy = *this;
  copy.sub(other);
  return copy;
}

inline Point Point::operator*(double s) const {
  return Point(x() * s, y() * s, z() * s);
}

inline Point Point::operator/(double s) const {
  return Point(x() / s, y() / s, z() / s);
}

// Dot product
inline double Point::operator*(const Point& other) const {
  return x() * other.x() + y() * other.y() + z() * other.z();
}

inline Point Point::Cross(const Point& other) const {
  return Point(y()*other.z() - z()*other.y(),
               z()*other.x() - x() * other.z(),
               x()*other.y() - y() * other.x());
}

inline bool Point::operator<(const Point& other) const {
  return (x() != other.x() ? x() < other.x() :
          (y() != other.y() ? y() < other.y() :
           (z() < other.z())));
}

inline bool Point::operator<=(const Point& other) const {
  return (x() != other.x() ? x() < other.x() :
          (y() != other.y() ? y() < other.y() :
           (z() <= other.z())));
}

inline bool Point::operator>=(const Point& other) const {
  return (x() != other.x() ? x() > other.x() :
          (y() != other.y() ? y() > other.y() :
           (z() >= other.z())));
}

inline bool Point::operator==(const Point& other) const {
  return x() == other.x() && y() == other.y() && z() == other.z();
}
 
inline bool Point::operator!=(const Point& other) const {
  return x() != other.x() || y() != other.y() || z() != other.z();
}

inline bool Point::ApproxEqual(const Point& other, double epsilon) const {
  return (fabs(x() - other.x()) < epsilon &&
          fabs(y() - other.y()) < epsilon &&
          fabs(z() - other.z()) < epsilon);
}

inline double Point::magnitude2() const {
  return x() * x() + y() * y() + z() * z();
}

inline double Point::magnitude() const {
  return sqrt(magnitude2());
}

inline double Point::DistanceFrom(const Point& other) const {
  return (*this - other).magnitude();
}

inline Point Point::Normalized() const {
  Point copy = *this;
  copy.Normalize();
  return copy;
}

inline void Point::Normalize() {
  double m = magnitude();
  if (m != 0) {
    *this = *this / m;
  }
}

inline void Point::add(const Point& other) {
  x_ += other.x_; y_ += other.y_; z_ += other.z_;
}

inline void Point::sub(const Point& other) {
  x_ -= other.x_; y_ -= other.y_; z_ -= other.z_;
}

inline void Point::RotateAround(const Point& vec, double angle_radians) {
  Point u = vec.Normalized();
  double c = cos(angle_radians);
  double i_c = 1 - c;
  double s = sin(angle_radians);
  double u_xy = u.x() * u.y(), u_xz = u.x() * u.z(), u_yz = u.y() * u.z();
  double u_xx = u.x() * u.x(), u_yy = u.y() * u.y(), u_zz = u.z() * u.z();
  *this = Point((c          + u_xx * i_c) * x_ +
                (u_xy * i_c - u.z() * s)  * y_ +
                (u_xz * i_c + u.y() * s)  * z_,
                (u_xy * i_c + u.z() * s)  * x_ +
                (c          + u_yy * i_c) *  y_ +
                (u_yz * i_c - u.x() * s)  * z_,
                (u_xz * i_c - u.y() * s)  * x_ +
                (u_yz * i_c + u.x() * s)  * y_ +
                (c          + u_zz * i_c) * z_);
}

inline Box Triangle::BoundingBox() const {
  Range x = x_range(), y = y_range(), z = z_range();
  return Box(Point(x.start(), y.start(), z.start()),
             Point(x.end(), y.end(), z.end()));
}

// Modeled on stl tuple hash function.
template <typename T>
static inline void geometry_hash_stupid_combine(size_t& seed, const T& a) {
  seed ^= std::hash<T>()(a) + 0x9e3779b9 + (seed<<6) + (seed>>2);
}
template <typename T>
static inline size_t geometry_hash_stupid(T a, T b, T c) {
  size_t seed = 0;
  geometry_hash_stupid_combine(seed, a);
  geometry_hash_stupid_combine(seed, b);
  geometry_hash_stupid_combine(seed, c);
  return seed;
}

struct HashPoint {
  size_t operator()(const Point& point) const {
    return geometry_hash_stupid(point.x(), point.y(), point.z());
  }
};

struct HashTriangle {
  size_t operator()(const Triangle& t) const {
    size_t p0 = HashPoint()(t.p0());
    geometry_hash_stupid_combine(p0, HashPoint()(t.p1()));
    geometry_hash_stupid_combine(p0, HashPoint()(t.p2()));
    return p0;
  }
};

struct HashFixedTriangle {
  size_t operator()(const FixedTriangle& t) const {
    return t.hash();
  }
};

struct HashEdge {
  size_t operator()(const Edge& e) const {
    size_t p0 = HashPoint()(e.p0());
    geometry_hash_stupid_combine(p0, HashPoint()(e.p1()));
    return p0;
  }
};

inline void FixedTriangle::ReCompute() {
  normal_ = Triangle::normal();
  box_ = Triangle::BoundingBox();
  hash_ = HashTriangle()(*this);
}

inline bool Box::Intersects(const Box& other) const {
  return (bottom_.x() <= other.top_.x() && top_.x() >= other.bottom_.x() &&
          bottom_.y() <= other.top_.y() && top_.y() >= other.bottom_.y() &&
          bottom_.z() <= other.top_.z() && top_.z() >= other.bottom_.z());
}

inline bool Box::Intersects(const Triangle& t) const {
  return IntersectsTriangle<Triangle>(t);
}

inline bool Box::Intersects(const FixedTriangle& t) const {
  return IntersectsTriangle<FixedTriangle>(t);
}

inline bool axis_point_test(double p0, double p1, double rad) {
  double min = (p0 < p1 ? p0 : p1);
  double max = (p0 < p1 ? p1 : p0);
  return min <= rad &&  max >= -rad;
}

template <int point>
inline bool axistest_X(double a, double b, double fa, double fb,
                       const Triangle& v,
                       const Point& box_half_size) {
  const Point& v0 = v.p0();
  const Point& v1 = (point == 1 ? v.p2() : v.p1());
  double p0 = a * v0.y() - b * v0.z();
  double p1 = a * v1.y() - b * v1.z();
  double rad = fa * box_half_size.y() + fb * box_half_size.z();
  return axis_point_test(p0, p1, rad);
}

template <int point>
inline bool axistest_Y(double a, double b, double fa, double fb,
                       const Triangle& v,
                       const Point& box_half_size) {
  const Point& v0 = v.p0();
  const Point& v1 = (point == 1 ? v.p2() : v.p1());
  double p0 = - a * v0.x() + b * v0.z();
  double p1 = - a * v1.x() + b * v1.z();
  double rad = fa * box_half_size.x() + fb * box_half_size.z();
  return axis_point_test(p0, p1, rad);
}

template <int point>
inline bool axistest_Z(double a, double b, double fa, double fb,
                       const Triangle& v,
                       const Point& box_half_size) {
  const Point& v0 = (point == 0 ? v.p1() : v.p0());
  const Point& v1 = (point == 0 ? v.p2() : v.p1());
  double p0 = a * v0.x() - b * v0.y();
  double p1 = a * v1.x() - b * v1.y();
  double rad = fa * box_half_size.x() + fb * box_half_size.y();
  return axis_point_test(p0, p1, rad);
}

template <int edge>
inline bool axistest(const Triangle& v,
                     const Point& e,
                     const Point& box_half_size) {
  const Point& b = box_half_size;
  double fex = fabs(e.x());
  double fey = fabs(e.y());
  double fez = fabs(e.z());

  // 0 => X01, Y02, Z12
  // 1 => X01, Y02, Z0
  // 2 => X2, Y1, Z12
  if (((edge != 2) && !axistest_X<1>(e.z(), e.y(), fez, fey, v, b)) ||
      ((edge != 2) && !axistest_Y<1>(e.z(), e.x(), fez, fex, v, b)) ||
      ((edge != 1) && !axistest_Z<0>(e.y(), e.x(), fey, fex, v, b)) ||
      ((edge == 1) && !axistest_Z<2>(e.y(), e.x(), fey, fex, v, b)) ||
      ((edge == 2) && !axistest_X<2>(e.z(), e.y(), fez, fey, v, b)) ||
      ((edge == 2) && !axistest_Y<2>(e.z(), e.x(), fez, fex, v, b))) {
    return false;
  }
  return true;
}

template <typename T>
inline bool Box::IntersectsTriangle(const T& t) const {
  // Test bounding box intersection
  if (!Intersects(t.BoundingBox())) {
    return false;
  }

  // Test to see if box encapsulates a vertex.
  if (Contains(t.p0()) || Contains(t.p1()) || Contains(t.p2())) {
    return true;
  }

  // Test plane intersection
  Point normal, pos;
  if (!IntersectsPlaneInternal<T>(t, normal, pos)) {
    return false;
  }

  // http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/tribox3.txt
  Point center = this->center();
  Point halfsize = top() - center;
  Triangle v = t - center;
  Point e0 = t.p1() - t.p0();
  Point e1 = t.p2() - t.p1();
  Point e2 = t.p0() - t.p2();
  return (axistest<0>(v, e0, halfsize) &&
          axistest<1>(v, e1, halfsize) && 
          axistest<2>(v, e2, halfsize));
}

inline bool Box::IntersectsPlane(const Triangle& plane) const {
  Point normal, pos;
  return IntersectsPlaneInternal<Triangle>(plane, normal, pos);
}

inline bool Box::IntersectsPlane(const FixedTriangle& plane) const {
  Point normal, pos;
  return IntersectsPlaneInternal<FixedTriangle>(plane, normal, pos);
}

template <typename T>
inline bool Box::IntersectsPlaneInternal(const T& plane,
                                         Point& normal,
                                         Point& pos) const {
  normal = plane.normal();
  pos = bottom();
  Point neg = top();

  if (normal.x() >= 0) {
    pos.set_x(top().x());
    neg.set_x(bottom().x());
  }
  if (normal.y() >= 0) {
    pos.set_y(top().y());
    neg.set_y(bottom().y());
  }
  if (normal.z() >= 0) {
    pos.set_z(top().z());
    neg.set_z(bottom().z());
  }
  double pos_dot = (pos - plane.p0()) * normal;
  double neg_dot = (neg - plane.p0()) * normal;
  return ((pos_dot >= 0) != (neg_dot >= 0));
}

inline void Box::UnionWith(const Box& other) {
  bottom_.set(std::min(bottom_.x(), other.bottom().x()),
              std::min(bottom_.y(), other.bottom().y()),
              std::min(bottom_.z(), other.bottom().z()));
  top_.set(std::max(top_.x(), other.top().x()),
           std::max(top_.y(), other.top().y()),
           std::max(top_.z(), other.top().z()));
}

inline void Box::IntersectWith(const Box& other) {
  if (!Intersects(other)) {
    bottom_ = top_ = Point(0, 0, 0);
  } else {
    bottom_.set(std::max(bottom_.x(), other.bottom().x()),
                std::max(bottom_.y(), other.bottom().y()),
                std::max(bottom_.z(), other.bottom().z()));
    top_.set(std::min(top_.x(), other.top().x()),
             std::min(top_.y(), other.top().y()),
             std::min(top_.z(), other.top().z()));
  }
}

inline void Box::UnionWith(const Point& point) {
  bottom_.set(std::min(bottom_.x(), point.x()),
              std::min(bottom_.y(), point.y()),
              std::min(bottom_.z(), point.z()));
  top_.set(std::max(top_.x(), point.x()),
           std::max(top_.y(), point.y()),
           std::max(top_.z(), point.z()));
}

inline bool Triangle::get_z_value(double x, double y, double* z) const {
  // 1) Check x/y range.
  if (!x_range().contains_closed(x) || !y_range().contains_closed(y)) {
    return false;
  }

  // TODO: Caching/precompute?

  // 2) Get z value (ax + by + cz = d)
  Point n = normal();
  if (n.z() == 0) {
    return false;
  }
  *z = (n * p0() - n.x() * x - n.y() * y) / n.z();

  // 3) Check point x,y,z actually in triangle.
  // TODO: double error?
  Point p(x, y, *z);
  return ((p1() - p0()).Cross(p - p0()) * n >= 0 &&
          (p2() - p1()).Cross(p - p1()) * n >= 0 &&
          (p0() - p2()).Cross(p - p2()) * n >= 0);
}

inline Triangle Triangle::Normalized() const {
  // p0->p1->p2 OR p1->p2->p0 OR p2->p0->p1
  if (p0() < p1()) {
    return (p0() < p2() ? Triangle(p0(), p1(), p2()) :
            Triangle(p2(), p0(), p1()));
  }
  return (p1() < p2() ? Triangle(p1(), p2(), p0()) :
          Triangle(p2(), p0(), p1()));
}

inline Triangle Triangle::Shuffled() const {
  return Triangle(p1(), p2(), p0());
}

inline Triangle Triangle::Flipped() const {
  return Triangle(p0(), p2(), p1());
}

inline bool Triangle::operator<(const Triangle& other) const {
  return (p0_ != other.p0_ ? p0_ < other.p0_ :
          (p1_ != other.p1_ ? p1_ < other.p1_ :
           (p2_ < other.p2_)));
}

inline bool Triangle::operator==(const Triangle& other) const {
  return (p0_ == other.p0_ && p1_ == other.p1_ && p2_ == other.p2_);
}

inline bool Triangle::operator!=(const Triangle& other) const {
  return (p0_ != other.p0_ || p1_ != other.p1_ || p2_ != other.p2_);
}

inline void Triangle::operator=(const Triangle& other) {
  p0_ = other.p0_; p1_ = other.p1_; p2_ = other.p2_;
}

inline Triangle Triangle::operator-(const Point& offset) const {
  return Triangle(p0() - offset, p1() - offset, p2() - offset);
}

inline Triangle Triangle::operator+(const Point& offset) const {
  return Triangle(p0() + offset, p1() + offset, p2() + offset);
}

inline bool Triangle::PlaneIntersects(const Triangle& other,
                                      double epsilon,
                                      PlaneIntersection& results) const {
  results.normal = normal();
  results.p0_dist = (other.p0() - p0()) * results.normal;
  results.p1_dist = (other.p1() - p0()) * results.normal;
  results.p2_dist = (other.p2() - p0()) * results.normal;
  if (results.p0_dist != 0 && fabs(results.p0_dist) < epsilon &&
      (other.p0() == p1() || other.p0() == p2())) {
    results.p0_dist = 0;
  }
  if (results.p1_dist != 0 && fabs(results.p1_dist) < epsilon &&
      (other.p1() == p1() || other.p1() == p2())) {
    results.p1_dist = 0;
  }
  if (results.p2_dist != 0 && fabs(results.p2_dist) < epsilon &&
      (other.p2() == p1() || other.p2() == p2())) {
    results.p2_dist = 0;
  }
  return ((results.p0_dist > 0) != (results.p1_dist > 0) ||
          (results.p0_dist > 0) != (results.p2_dist > 0) ||
          // Point/edge intersection:
          (results.p0_dist == 0 ||
           results.p1_dist == 0 ||
           results.p2_dist == 0));
}

// static
template <bool coincident, typename T>
inline bool Triangle::IntersectsComplicated(const T& t,
                                            const T& other,
                                            double epsilon) {
  // Project other onto our plane, see if it intersects.
  PlaneIntersection other_triangle_on_our_plane;
  if (!t.PlaneIntersects(other, epsilon, other_triangle_on_our_plane)) {
    return false;
  }

  // Project us onto other plane, see if it intersects.
  PlaneIntersection our_triangle_on_other_plane;
  if (!other.PlaneIntersects(t, epsilon, our_triangle_on_other_plane)) {
    return false;
  }

  // Check coplanar.
  if (fabs(other_triangle_on_our_plane.p0_dist) < epsilon &&
      fabs(other_triangle_on_our_plane.p1_dist) < epsilon &&
      fabs(other_triangle_on_our_plane.p2_dist) < epsilon) {
    return IntersectsTriangle2D(coincident, epsilon, t, other);
  }

  // Get segment s that intersects our triangle plane.
  return IntersectsTriangle3D(coincident, epsilon, t, other,
                              our_triangle_on_other_plane,
                              other_triangle_on_our_plane);
}

// static
template <bool coincident, typename T>
inline bool Triangle::IntersectsInternal(const T& t,
                                         const T& other,
                                         double epsilon) {
  // First, check bounding box.
  if (!other.BoundingBox().Intersects(t.BoundingBox())) {
    return false;
  }
  return IntersectsComplicated<coincident, T>(t, other, epsilon);
}

inline bool Triangle::IntersectsEpsilon(const Triangle& other,
                                        double epsilon) const {
  return IntersectsInternal<true, Triangle>(*this, other, epsilon);
}

inline bool Triangle::IntersectsNotCoincidentEpsilon(const Triangle& other,
                                                     double epsilon) const {
  return IntersectsInternal<false, Triangle>(*this, other, epsilon);
}

inline bool FixedTriangle::IntersectsEpsilon(
    const FixedTriangle& other,
    double epsilon) const {
  return IntersectsInternal<true, FixedTriangle>(*this, other, epsilon);
}

inline bool FixedTriangle::IntersectsNotCoincidentEpsilon(
    const FixedTriangle& other,
    double epsilon) const {
  return IntersectsInternal<false, FixedTriangle>(*this, other, epsilon);
}

inline bool Triangle2D::operator<(const Triangle2D& other) const {
  return (p0_ != other.p0_ ? p0_ < other.p0_ :
          (p1_ != other.p1_ ? p1_ < other.p1_ :
           (p2_ < other.p2_)));
}

inline Triangle2D Triangle2D::Normalized() const {
  // p0->p1->p2 OR p1->p2->p0 OR p2->p0->p1
  if (p0() < p1()) {
    return (p0() < p2() ? Triangle2D(p0(), p1(), p2()) :
            Triangle2D(p2(), p0(), p1()));
  }
  return (p1() < p2() ? Triangle2D(p1(), p2(), p0()) :
          Triangle2D(p2(), p0(), p1()));
}
}  // namespace printer

#endif  // _PRINTER_BASE_GEOMETRY_H__
