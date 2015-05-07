// Copyright 2015
// Author: Christopher Van Arsdale
//
// This file should not be taken as an example of good geometry code.
// (I winged it)

#include <float.h>
#include <string>
#include <sstream>
#include "printer/base/geometry.h"
#include "common/base/flags.h"
#include "common/log/log.h"
#include "common/strings/strutil.h"

#ifndef DECIMAL_DIG
#define DECIMAL_DIG 21
#endif

DEFINE_bool(always_print_full_point, false,
            "If true, forces DebugString value to be full precision.");

using std::string;

namespace printer {
namespace {
template <typename GetA, typename GetB>
static Point2D GetPoint(const Point& input,
                        const Point& u,
                        const Point& v,
                        double cross) {
  // TODO(cvanarsdale): Because we do our calculation in our transformed
  // space, we run into double numeric errors causing incorrect results. This
  // fixes *some* of of them, but honestly it's not perfect.
  return (input == Point(0, 0, 0) ? Point2D(0, 0) :
          (input == u ? Point2D(1, 0) :
           (input == v ? Point2D(0, 1) :
            Point2D((GetA()(input) * GetB()(v) -
                     GetB()(input) * GetA()(v)) / cross,
                    (GetB()(input) * GetA()(u) -
                     GetA()(input) * GetB()(u)) / cross))));
}

template <typename GetA, typename GetB>
static Triangle2D BuildAB(const Triangle& input,
                          const Point& u,
                          const Point& v,
                          double cross) {
  return Triangle2D(GetPoint<GetA, GetB>(input.p0(), u, v, cross),
                    GetPoint<GetA, GetB>(input.p1(), u, v, cross),
                    GetPoint<GetA, GetB>(input.p2(), u, v, cross));
}

template <int xyz>
struct GetVal {
  double operator()(const Point& p) const {
    return (xyz == 0 ? p.x() : (xyz == 1 ? p.y() : p.z()));
  }
};

void GetIntercepts(const Point2D& a, const Point2D& b,
                   double* x, int* next_x,
                   double* y, int* next_y) {
  double dy = b.y() - a.y();
  double dx = b.x() - a.x();
  double xmin = std::min(a.x(), b.x());
  double xmax = std::max(a.x(), b.x());
  double ymin = std::min(a.y(), b.y());
  double ymax = std::max(a.y(), b.y());

  if (b.y() == 0) {
    x[(*next_x)++] = b.x();
  } else if (dy != 0 && ymin <= 0 && ymax >= 0) {
    x[(*next_x)++] = -a.y()*dx/dy + a.x();
  }

  if (b.x() == 0) {
    y[(*next_y)++] = b.y();
  } else if (dx != 0 && xmin <= 0 && xmax >= 0) {
    y[(*next_y)++] = -a.x()*dy/dx + a.y();
  }
}

bool HalfEdgeX(const Triangle2D& t) {
  // Half edge is ok if it is 1 -> 0, based on how we did our projection.
  return ((t.p0().x() == 1 && t.p1().x() == 0 &&
           t.p0().y() == 0 && t.p1().y() == 0) ||
          (t.p1().x() == 1 && t.p2().x() == 0 &&
           t.p1().y() == 0 && t.p2().y() == 0) ||
          (t.p2().x() == 1 && t.p0().x() == 0 &&
           t.p2().y() == 0 && t.p0().y() == 0));
}

bool HalfEdgeY(const Triangle2D& t) {
  // Half edge is ok if it is 0 -> 1, based on how we did our projection.
  return ((t.p0().y() == 0 && t.p1().y() == 1 &&
           t.p0().x() == 0 && t.p1().x() == 0) ||
          (t.p1().y() == 0 && t.p2().y() == 1 &&
           t.p1().x() == 0 && t.p2().x() == 0) ||
          (t.p2().y() == 0 && t.p0().y() == 1 &&
           t.p2().x() == 0 && t.p0().x() == 0));
}

template <bool coincident_intersects>
bool IntersectsHalfUnit(const Triangle2D& t) {
  // We are checking if t fits into (0,0) -> (0,1) -> (1,0)
  // If any points in t are in the half-unit-triangle, return true.
  if (coincident_intersects) {
    if ((t.p0().x() >= 0 && t.p0().y() >= 0 &&
         (t.p0().x() + t.p0().y() <= 1)) ||
        (t.p1().x() >= 0 && t.p1().y() >= 0 &&
         (t.p1().x() + t.p1().y() <= 1)) ||
        (t.p2().x() >= 0 && t.p2().y() >= 0 &&
         (t.p2().x() + t.p2().y() <= 1))) {
      return true;
    }
  } else {
    if ((t.p0().x() > 0 && t.p0().y() > 0 && (t.p0().x() + t.p0().y() < 1)) ||
        (t.p1().x() > 0 && t.p1().y() > 0 && (t.p1().x() + t.p1().y() < 1)) ||
        (t.p2().x() > 0 && t.p2().y() > 0 && (t.p2().x() + t.p2().y() < 1))) {
      return true;
    }
  }

  // If we are in a negative region only (e.g. below x axis), we don't
  // intersect.
  if ((t.p0().x() < 0 && t.p1().x() < 0 && t.p2().x() < 0) ||
      (t.p0().y() < 0 && t.p1().y() < 0 && t.p2().y() < 0) ||
      (t.p0().x() > 1 && t.p1().x() > 1 && t.p2().x() > 1) ||
      (t.p0().y() > 1 && t.p1().y() > 1 && t.p2().y() > 1) ||
      ((t.p0().x() + t.p0().y()) < 0 &&
       (t.p1().x() + t.p1().y()) < 0 &&
       (t.p2().x() + t.p2().y()) < 0) ||
      ((t.p0().x() + t.p0().y()) > 1 &&
       (t.p1().x() + t.p1().y()) > 1 &&
       (t.p2().x() + t.p2().y()) > 1)) {
     return false;
  }

  // Test any line intersects covers half area.
  int next_x = 0, next_y = 0;
  double x[3], y[3];
  GetIntercepts(t.p1(), t.p0(), x, &next_x, y, &next_y);
  GetIntercepts(t.p2(), t.p1(), x, &next_x, y, &next_y);
  GetIntercepts(t.p0(), t.p2(), x, &next_x, y, &next_y);

  // If we cross the x axis:
  if (next_x > 0) {
    double x_min = std::min(x[0], std::min(x[next_x - 1], x[next_x >> 1]));
    double x_max = std::max(x[0], std::max(x[next_x - 1], x[next_x >> 1]));
    if ((coincident_intersects && x_min <= 1 && x_max >= 0) ||
        (!coincident_intersects && x_min < 1 && x_max > 0 && x_min != x_max &&
         !HalfEdgeX(t))) {
      return true;
    }
  }

  // If we cross the y axis:
  if (next_y > 0) {
    double y_min = std::min(y[0], std::min(y[next_y - 1], y[next_y >> 1]));
    double y_max = std::max(y[0], std::max(y[next_y - 1], y[next_y >> 1]));
    if ((coincident_intersects && y_min <= 1 && y_max >= 0) ||
        (!coincident_intersects && y_min < 1 && y_max > 0 && y_min != y_max &&
         !HalfEdgeY(t))) {
      return true;
    }
  }
  return false;
}

}  // anonymous namespace

// This interpolates "input" based on a new set of basis vectors u and v.
// Basically this maps something in the uv plane to the xy plane.
// static
Triangle2D Triangle2D::FromAxes(const Triangle& input,
                                const Point& u,
                                const Point& v,
                                double epsilon) {
  // x * u + y * v == input.
  // => solve for x, y for each point in input.
  // x * u_x + y * v_x == i_x
  // x * u_y + y * v_y == i_y
  // x * u_z + y * v_z == i_z.
  // Overconstrained (obviously, since we are on a plane).
  double vxy = u.x() * v.y() - u.y() * v.x();
  if (fabs(vxy) > epsilon) {
    return BuildAB<GetVal<0>, GetVal<1>>(input, u, v, vxy);
  }

  double vzy = u.z() * v.y() - u.y() * v.z();
  if (fabs(vzy) > epsilon) {
    return BuildAB<GetVal<2>, GetVal<1>>(input, u, v, vzy);
  }

  double vxz = v.z() * u.x() - v.x() * u.z();
  return BuildAB<GetVal<0>, GetVal<2>>(input, u, v, vxz);
}

Triangle Triangle2D::ToAxes(const Point& u, const Point& v) const {
  return Triangle(u * p0().x() + v * p0().y(),
                  u * p1().x() + v * p1().y(),
                  u * p2().x() + v * p2().y());
}

// static
const double Triangle::kDefaultIntersectEpsilon = 1e-12;

string Point::DebugString() const {
  if (FLAGS_always_print_full_point) {
    return FullString();
  }
  std::stringstream out;
  out << x() << "," << y() << "," << z();
  return out.str();
}

string Point::FullString() const {
  return strings::StringPrintf("%.*e,%.*e,%.*e",
                               DECIMAL_DIG, x(),
                               DECIMAL_DIG, y(),
                               DECIMAL_DIG, z());
}
string Point2D::DebugString() const {
  if (FLAGS_always_print_full_point) {
    return FullString();
  }
  std::stringstream out;
  out << x() << "," << y();
  return out.str();
}

string Point2D::FullString() const {
  return strings::StringPrintf("%.*e,%.*e",
                               DECIMAL_DIG, x(),
                               DECIMAL_DIG, y());
}

string Edge2D::DebugString() const {
  std::stringstream out;
  out << "(" << p0().DebugString() << ") -> (" << p1().DebugString() << ")";
  return out.str();
}

string Box::DebugString() const {
  std::stringstream out;
  out << "(" << bottom_.DebugString() << "),(" << top_.DebugString() << ")";
  return out.str();
}

string Range::DebugString() const {
  std::stringstream out;
  out << start_ << "," << end_;
  return out.str();
}

string Edge::DebugString() const {
  std::stringstream out;
  out << "(" << p0().DebugString() << ") -> (" << p1().DebugString() << ")";
  return out.str();
}

string Triangle::DebugString() const {
  std::stringstream out;
  out << "(" << p0().DebugString() << "),(" << p1().DebugString()
      << "),(" << p2().DebugString() << ")";
  return out.str();
}

string Triangle2D::DebugString() const {
  std::stringstream out;
  out << "(" << p0().DebugString() << "),(" << p1().DebugString()
      << "),(" << p2().DebugString() << ")";
  return out.str();
}

namespace {
Point InterpolateForIntersect(const Point& a, const Point& b,
                              double a_weight, double b_weight) {
  if (b_weight == 0) { return b; }
  if (a_weight == 0) { return a; }
  return (b - a) * fabs(a_weight) / (fabs(a_weight) + fabs(b_weight)) + a;
}
void SortPointsByDist(const Point& p0, const Point& p1, const Point& p2,
                      double p0_dist, double p1_dist, double p2_dist,
                      Point& a, Point& b, Point& c,
                      double& a_dist, double& b_dist, double& c_dist) {
  if (p0_dist < p1_dist) {
    if (p1_dist < p2_dist) {
      a = p0; b = p1; c = p2;
      a_dist = p0_dist; b_dist = p1_dist; c_dist = p2_dist;
    } else if (p0_dist < p2_dist) {
      a = p0; b = p2; c = p1;
      a_dist = p0_dist; b_dist = p2_dist; c_dist = p1_dist;
    } else {
      a = p2; b = p0; c = p1;
      a_dist = p2_dist; b_dist = p0_dist; c_dist = p1_dist;
    }
  } else {
    if (p0_dist < p2_dist) {
      a = p1; b = p0; c = p2;
      a_dist = p1_dist; b_dist = p0_dist; c_dist = p2_dist;
    } else if (p1_dist < p2_dist) {
      a = p1; b = p2; c = p0;
      a_dist = p1_dist; b_dist = p2_dist; c_dist = p0_dist;
    } else {
      a = p2; b = p1; c = p0;
      a_dist = p2_dist; b_dist = p1_dist; c_dist = p0_dist;
    }
  }
}
}  // anonymous namesspace

// static
Edge Triangle::GetPlaneSegment(const Triangle& t,
                               const PlaneIntersection& sect) {
  Point a, b, c;
  double a_dist, b_dist, c_dist;
  SortPointsByDist(t.p0(), t.p1(), t.p2(),
                   sect.p0_dist, sect.p1_dist, sect.p2_dist,
                   a, b, c, a_dist, b_dist, c_dist);
  Point s1 = InterpolateForIntersect(a, c, a_dist, c_dist);
  Point s2 =  ((a_dist <= 0) == (b_dist <= 0) ?
               InterpolateForIntersect(c, b, c_dist, b_dist) :
               InterpolateForIntersect(a, b, a_dist, b_dist));
  return s1 < s2 ? Edge(s1, s2) : Edge(s2, s1);
}

bool Triangle::SharesOpposingHalfedge(const Triangle& o) const {
  if (p0() == o.p0()) {
    return p1() == o.p2() || p2() == o.p1();
  }
  if (p0() == o.p1()) {
    return p1() == o.p0() || p2() == o.p2();
  }
  if (p0() == o.p2()) {
    return p1() == o.p1() || p2() == o.p0();
  }
  if (p1() == o.p0()) {
    return p2() == o.p2();
  }
  if (p1() == o.p1()) {
    return p2() == o.p0();
  }
  if (p1() == o.p2()) {
    return p2() == o.p1();
  }
  return false;
}

template <bool coincident_intersects, typename T>
bool Triangle::IntersectsTriangle2D(double epsilon,
                                    const T& t,
                                    const T& other) {
  VLOG(7) << "IntersectsTriangle2D";
  Point u = t.p1() - t.p0();
  Point v = t.p2() - t.p0();
  Triangle2D other_2d = Triangle2D::FromAxes(other - t.p0(), u, v, epsilon);
  VLOG(8) << "Normalized: " << other_2d.DebugString();

  // We've used p0->p2 and p0->p1 as axes, and p0 as our new origin.
  // Thus, "t" translates to (0,0) -> (1,0) -> (0, 1).
  if (coincident_intersects) {
    return IntersectsHalfUnit<true>(other_2d);
  }
  return IntersectsHalfUnit<false>(other_2d);
}


// static
bool Triangle::IntersectsTriangle2D(bool coincident_intersects,
                                    double epsilon,
                                    const Triangle& t,
                                    const Triangle& other) {
  if (coincident_intersects) {
    return IntersectsTriangle2D<true, Triangle>(epsilon, t, other);
  }
  return  IntersectsTriangle2D<false, Triangle>(epsilon, t, other);
}

// static
bool Triangle::IntersectsTriangle2D(bool coincident_intersects,
                                    double epsilon,
                                    const FixedTriangle& t,
                                    const FixedTriangle& other) {
  if (coincident_intersects) {
    return IntersectsTriangle2D<true, FixedTriangle>(epsilon, t, other);
  }
  return  IntersectsTriangle2D<false, FixedTriangle>(epsilon, t, other);
}

template <bool coincident_intersects, typename T>
bool Triangle::IntersectsTriangle3D(double epsilon,
                                    const T& t,
                                    const T& other,
                                    const PlaneIntersection& t_on_other_plane,
                                    const PlaneIntersection& other_on_t_plane) {
  VLOG(7) << "IntersectsTriangle3D";
  Edge other_s = GetPlaneSegment(other, other_on_t_plane);
  Edge t_s = GetPlaneSegment(t, t_on_other_plane);
  if (other_s.p1() < t_s.p0() || t_s.p1() < other_s.p0()) {
    return false;
  }
  if (coincident_intersects) {
    return true;
  }

  // Compute parameter 't', which is along intersection line.
  Point dir = t_s.direction();
  Range t_range(0, dir.magnitude2());
  Range o_range = Range::FromValues((other_s.p0() - t_s.p0()) * dir,
                                    (other_s.p1() - t_s.p0()) * dir);
  double start_int = std::max(t_range.start(), o_range.start());
  double end_int = std::min(t_range.end(), o_range.end());
  if (start_int >= end_int - epsilon) {
    return false;
  }

  // Ok, we do overlap on the line. Make sure we don't share a half-edge
  return !t.SharesOpposingHalfedge(other);
}

// static
bool Triangle::IntersectsTriangle3D(bool coincident_intersects,
                                    double epsilon,
                                    const Triangle& t,
                                    const Triangle& other,
                                    const PlaneIntersection& t_on_other_plane,
                                    const PlaneIntersection& other_on_t_plane) {
  if (coincident_intersects) {
    return IntersectsTriangle3D<true, Triangle>(
        epsilon, t, other, t_on_other_plane, other_on_t_plane);
  }
  return IntersectsTriangle3D<false, Triangle>(
      epsilon, t, other, t_on_other_plane, other_on_t_plane);
}

// static
bool Triangle::IntersectsTriangle3D(bool coincident_intersects,
                                    double epsilon,
                                    const FixedTriangle& t,
                                    const FixedTriangle& other,
                                    const PlaneIntersection& t_on_other_plane,
                                    const PlaneIntersection& other_on_t_plane) {
  if (coincident_intersects) {
    return IntersectsTriangle3D<true, FixedTriangle>(
        epsilon, t, other, t_on_other_plane, other_on_t_plane);
  }
  return IntersectsTriangle3D<false, FixedTriangle>(
      epsilon, t, other, t_on_other_plane, other_on_t_plane);
}

}  // namespace printer
