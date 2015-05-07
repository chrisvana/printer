// Copyright 2015
// Author: Christopher Van Arsdale

#include <map>
#include <unordered_set>
#include <vector>
#include "common/log/log.h"
#include "printer/base/geometry.h"
#include "printer/simplify/sweep_triangulation.h"
#include "printer/simplify/triangulation.h"

using std::vector;
using std::map;
using std::unordered_set;

namespace printer {
namespace {
class Point2DGenerator {
 public:
  Point2DGenerator(const Point& u, const Point& v,
                   const Triangulation::PolygonLoop& outer_loop)
      : base_input_(outer_loop[0]),
        u_(u),
        v_(v),
        center_2d_(Point2D(0, 0)),
        scale_2d_(Point2D(1, 1)) {
    Point2D min_p, max_p;
    for (int i = 0; i < outer_loop.size(); ++i) {
      Point2D tmp = GetPoint2D(outer_loop[i]);
      if (i == 0) {
        min_p = max_p = tmp;
      } else {
        min_p.set_x(std::min(min_p.x(), tmp.x()));
        min_p.set_y(std::min(min_p.y(), tmp.y()));
        max_p.set_x(std::max(max_p.x(), tmp.x()));
        max_p.set_y(std::max(max_p.y(), tmp.y()));
      }
    }
    center_2d_ = (min_p + max_p) / 2;
    scale_2d_ = (max_p - min_p) / 2;
  }

  void ProcessLoop(const vector<Point>& loop, vector<Point2D>* out) {
    CHECK_GE(loop.size(), 3);
    Point2D prev = GetPoint2D(loop.back());
    Point2D next = GetPoint2D(loop.front());
    for (int i = 0; i < loop.size(); ++i) {
      Point2D point = next;
      next = GetPoint2D(loop[i == loop.size() - 1 ? 0 : i + 1]);
      out->push_back(AdjustPoint(prev, point, next));
      back_map_[out->back()] = loop[i];
      VLOG(5) << loop[i].DebugString() << " became "
              << out->back().DebugString();
      prev = point;
    }
  }

  Point GetPoint(const Point2D& point) const {
    Point out;
    auto it = back_map_.find(point);
    if (it != back_map_.end()) {
      VLOG(5) << "Found inverse point: " << it->second.DebugString();
      out = it->second;
    } else {
      // This is unexpected. We don't expect the triangulation code to produce
      // novel points at the moment. This is here just in case.
#if 0
      Point2D scaled = Point2D(point.x() * scale_2d_.x(),
                               point.y() * scale_2d_.y());
      scaled = scaled + center_2d_;
      out = u_ * scaled.x() + v_ * scaled.y() + base_input_;
#else
      LOG(FATAL) << "Unexpected point produced by triangulation code, "
                 << "not in our known point set: " << point.DebugString();
#endif
    }
    VLOG(5) << point.DebugString() << " became " << out.DebugString();
    return out;
  }

 private:
  typedef map<Point2D, Point> BackMap;

  Point2D GetPoint2D(const Point& input) const {
    Point2D tmp = Triangle2D::FromAxes(    // Temp hack.
        Triangle(input - base_input_, Point(), Point()), u_, v_).p0();
    tmp = tmp - center_2d_;
    return Point2D(tmp.x() / scale_2d_.x(), tmp.y() / scale_2d_.y());
  }

  Point2D AdjustPoint(const Point2D& prev,
                      const Point2D& point,
                      const Point2D& next) const {
    Point2D adjust = ((prev - point).Normalized() +
                      (next - point).Normalized()) / 1e6;
    return point + adjust;
  }

  Point base_input_;
  Point u_, v_;
  Point2D center_2d_, scale_2d_;
  BackMap back_map_;
};
}  // anonymous namespace

bool Triangulation::Triangulate(std::vector<Triangle>* output) const {
  if (loop_ == NULL || loop_->size() < 3) {
    return false;
  }

  Point2DGenerator generator(u_, v_, *loop_);

  // Build up 2d points.
  SweepTriangulation sweep;
  for (int i = 0; i < num_loops(); ++i) {
    vector<Point2D> points;
    generator.ProcessLoop(loop(i), &points);
    sweep.AddLoop(points);
  }

  // Run sweep.
  vector<Triangle2D> triangles;
  if (!sweep.Triangulate(&triangles)) {
    return false;
  }

  // Back to 3D.
  unordered_set<Point, HashPoint> seen_points;
  for (int i = 0; i < triangles.size(); ++i) {
    output->push_back(Triangle(generator.GetPoint(triangles[i].p0()),
                               generator.GetPoint(triangles[i].p1()),
                               generator.GetPoint(triangles[i].p2())));
    if (!output->back().WellFormed()) {
      // This triangle is spanning a intersection point, e.g. when
      // the polygon touches itself:
      //   ____
      //  / _  |
      // | | | |
      // |_><__|    ... we separate the two points that touch when
      // we do the 2d triangulation, and bring them back together with GetPoint.
      // Thus, we may get empty triangles that we can drop.
      output->pop_back();
    } else {
      seen_points.insert(output->back().p0());
      seen_points.insert(output->back().p1());
      seen_points.insert(output->back().p2());
    }
  }

  // Sanity check
  for (int i = 0; i < num_loops(); ++i) {
    for (const Point& p : loop(i)) {
      if (seen_points.find(p) == seen_points.end()) {
        LOG(ERROR) << "Internal error: Triangulation missing critical point: "
                   << p.DebugString();
        return false;
      }
    }
  }

  return true;
}

}  // namespace printer
