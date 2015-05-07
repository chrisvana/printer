// Copyright 2015
// Author: Christopher Van Arsdale

#include "common/test/test.h"
#include "common/base/flags.h"
#include "common/log/log.h"
#include "printer/base/geometry.h"

namespace printer {
namespace {

class GeometryTest : public testing::Test {
};

TEST_F(GeometryTest, CrossProduct) {
  Point a(0, 0, 1), b(0, 0, -1), c(1, 0, 0);
  Point a_c = a.Cross(c);
  Point c_a = c.Cross(a);
  Point a_b = a.Cross(b);
  EXPECT_TRUE(a_c == Point(0, 1, 0));
  EXPECT_TRUE(c_a == Point(0, -1, 0));
  EXPECT_TRUE(a_b == Point(0, 0, 0));
}

TEST_F(GeometryTest, DotProduct) {
  Point a(0, 0, 1), b(0, 0, -1), c(1, 0, 0);
  EXPECT_EQ(0, a*c);
  EXPECT_EQ(-1, a*b);
  EXPECT_EQ(1, a*a);
}

TEST_F(GeometryTest, Rotate) {
  const double kPi = 3.14159265357989;
  {  // Around same axis as vector
    Point a(1, 0, 0);
    a.RotateAround(Point(1, 0, 0), kPi/2);
    EXPECT_TRUE(a == Point(1, 0, 0));
  }

  {  // Around another axis
    Point a(1, 0, 0);
    a.RotateAround(Point(0, 0, 1), kPi/2);
    EXPECT_TRUE(a.ApproxEqual(Point(0, 1, 0), 0.0000001));
  }

  {  // Around orthogonal vector
    Point a(1, 1, 1);
    a.RotateAround(Point(1, 0, -1), kPi);
    EXPECT_TRUE(a.ApproxEqual(Point(-1, -1, -1), 0.0000001)) << a.DebugString();
  }
}

TEST_F(GeometryTest, RotateTransform) {
  const double kPi = 3.14159265357989;
  Box dimensions(Point(0, 0, 0), Point(10, 10, 20));

  // 2 pi, same position.
  {
    RotateTransform transform(
        dimensions.center(),
        Point(1, 0, 0),
        2 * kPi);
    Point test = Point(5, 5, 20);
    Point p = transform.Transform(test);
    ASSERT_TRUE(p.ApproxEqual(test, 0.0000001)) << p.DebugString();
    p = transform.InverseTransform(p);
    ASSERT_TRUE(p.ApproxEqual(test, 0.0000001)) << p.DebugString();
  }

  // pi, reversed
  {
    RotateTransform transform(
        dimensions.center(),
        Point(1, 0, 0),
        kPi);
    Point test = Point(5, 5, 20);
    Point p = transform.Transform(test);
    ASSERT_TRUE(p.ApproxEqual(Point(5, 5, 0), 0.0000001)) << p.DebugString();
    p = transform.InverseTransform(p);
    ASSERT_TRUE(p.ApproxEqual(test, 0.0000001)) << p.DebugString();
  }

  // pi/2, sideways.
  {
    RotateTransform transform(
        dimensions.center(),
        Point(1, 0, 0),
        kPi / 2);
    Point test = Point(5, 5, 20);
    Point p = transform.Transform(test);
    ASSERT_TRUE(p.ApproxEqual(Point(5, -5, 10), 0.0000001)) << p.DebugString();
    p = transform.InverseTransform(p);
    ASSERT_TRUE(p.ApproxEqual(test, 0.0000001)) << p.DebugString();
  }
}

TEST_F(GeometryTest, Normal) {
  Triangle triangle(Point(0, 0, 1),
                    Point(1, 0, 1),
                    Point(1, 1, 1));
  EXPECT_TRUE(Point(0, 0, 1) == triangle.normal())
      << triangle.normal().DebugString();
}

TEST_F(GeometryTest, SharesOpposingHalfedge) {
  Triangle t1(Point(0, 0, 1),
              Point(1, 0, 1),
              Point(1, 1, 1));
  EXPECT_FALSE(t1.SharesOpposingHalfedge(t1));
  EXPECT_FALSE(t1.SharesOpposingHalfedge(Triangle(Point(0, 0, 1),
                                                  Point(1, 0, 1),
                                                  Point(2, 2, 2))));
  EXPECT_FALSE(t1.SharesOpposingHalfedge(Triangle(Point(2, 2, 2),
                                                  Point(0, 0, 1),
                                                  Point(1, 0, 1))));
  EXPECT_FALSE(t1.SharesOpposingHalfedge(Triangle(Point(1, 0, 1),
                                                  Point(2, 2, 2),
                                                  Point(0, 0, 1))));
  EXPECT_TRUE(t1.SharesOpposingHalfedge(Triangle(Point(0, 0, 1),
                                                 Point(1, 1, 1),
                                                 Point(2, 2, 2))));
  EXPECT_TRUE(t1.SharesOpposingHalfedge(Triangle(Point(1, 1, 1),
                                                 Point(2, 2, 2),
                                                 Point(0, 0, 1))));
  EXPECT_TRUE(t1.SharesOpposingHalfedge(Triangle(Point(2, 2, 2),
                                                 Point(0, 0, 1),
                                                 Point(1, 1, 1))));
}

TEST_F(GeometryTest, ZIntercept) {
  Triangle triangle(Point(0, 0, 1),
                    Point(1, 0, 1),
                    Point(1, 1, 1));
  double z_val = -1;
  EXPECT_TRUE(triangle.get_z_value(0.5, 0.5, &z_val));
  EXPECT_EQ(1, z_val);

  z_val = -1;
  EXPECT_FALSE(triangle.get_z_value(-0.5, 0.5, &z_val));

  z_val = -1;
  Triangle t2(Point(-9.99781,-0.209424,0),
              Point(-9.9989,0.104712,0),
              Point(-10,-1.4855e-10,1));
  EXPECT_FALSE(triangle.get_z_value(-10, -0.2, &z_val))
      << z_val;
}

TEST_F(GeometryTest, ZInterceptReg) {
  // Old
  Triangle t0(Point(-10,0,0),
              Point(-9.99781,0.209424,0),
              Point(-9.99781,0.209424,1));
  Triangle t1(Point(-10,0,0),
              Point(-9.99781,0.209424,1),
              Point(-10,0,1));
  Triangle t2(Point(-9.99781,0.209424,0),
              Point(-9.99123,0.418757,0),
              Point(-9.99123,0.418757,1));
  Triangle t3(Point(-9.99781,0.209424,0),
              Point(-9.99123,0.418757,1),
              Point(-9.99781,0.209424,1));
  Triangle t4(Point(-9.99781,-0.209424,0),
              Point(-10,0,0),
              Point(-10,0,1));
  Triangle t5(Point(0,0,0),
              Point(-10,0,0),
              Point(-9.99781,-0.209424,0));
  Triangle t6(Point(0,0,0),
              Point(-9.99781,0.209424,0),
              Point(-10,0,0));
  Triangle t7(Point(0,0,0),
              Point(-9.99123,0.418757,0),
              Point(-9.99781,0.209424,0));

  // New
  Triangle t8(Point(-9.9989,0.104712,0),
              Point(-9.99781,0.209424,1),
              Point(-10,0,1));
  Triangle t9(Point(-9.9989,0.104712,0),
              Point(-9.99123,0.418757,0),
              Point(-9.99123,0.418757,1));
  Triangle t10(Point(-9.9989,0.104712,0),
               Point(-9.99123,0.418757,1),
               Point(-9.99781,0.209424,1));
  Triangle t11(Point(-9.99781,-0.209424,0),
               Point(-9.9989,0.104712,0),
               Point(-10,0,1));
  Triangle t12(Point(0,0,0),
               Point(-9.9989,0.104712,0),
               Point(-9.99781,-0.209424,0));  // MAYBE?
  Triangle t13(Point(0,0,0),
               Point(-9.99123,0.418757,0),
               Point(-9.9989,0.104712,0));

  double z_val = -1;
  EXPECT_FALSE(t0.get_z_value(-10, -0.2, &z_val)) << z_val;
  EXPECT_FALSE(t1.get_z_value(-10, -0.2, &z_val)) << z_val;
  EXPECT_FALSE(t2.get_z_value(-10, -0.2, &z_val)) << z_val;
  EXPECT_FALSE(t3.get_z_value(-10, -0.2, &z_val)) << z_val;
  EXPECT_FALSE(t4.get_z_value(-10, -0.2, &z_val)) << z_val;
  EXPECT_FALSE(t5.get_z_value(-10, -0.2, &z_val)) << z_val;
  EXPECT_FALSE(t6.get_z_value(-10, -0.2, &z_val)) << z_val;
  EXPECT_FALSE(t7.get_z_value(-10, -0.2, &z_val)) << z_val;

  EXPECT_FALSE(t8.get_z_value(-10, -0.2, &z_val)) << z_val;
  EXPECT_FALSE(t9.get_z_value(-10, -0.2, &z_val)) << z_val;
  EXPECT_FALSE(t10.get_z_value(-10, -0.2, &z_val)) << z_val;
  EXPECT_FALSE(t11.get_z_value(-10, -0.2, &z_val)) << z_val;
  EXPECT_FALSE(t12.get_z_value(-10, -0.2, &z_val)) << z_val;
  EXPECT_FALSE(t13.get_z_value(-10, -0.2, &z_val)) << z_val;
}

TEST_F(GeometryTest, TriangleBoxInside) {
  Triangle t(Point(0.5, 0.5, 0.5),
             Point(0.6, 0.5, 0.5),
             Point(0.55, 0.6, 0.6));
  Box box(Point(0, 0, 0), Point(1, 1, 1));
  EXPECT_TRUE(box.Intersects(t));
}

TEST_F(GeometryTest, TriangleBoxOneInside) {
  Triangle t(Point(0.5, 0.5, 0.5),
             Point(0.5, 0.5, 2),
             Point(0.55, 0.6, 2));
  Box box(Point(0, 0, 0), Point(1, 1, 1));
  EXPECT_TRUE(box.Intersects(t));
}

TEST_F(GeometryTest, TriangleBoxFullyOutside) {
  Triangle t(Point(0.5, 0.5, 0.5),
             Point(0.6, 0.5, 0.5),
             Point(0.55, 0.6, 0.6));
  Box box(Point(0, 0, 0), Point(0.4, 1, 1));
  EXPECT_FALSE(box.Intersects(t));
}

TEST_F(GeometryTest, TriangleBoxPlaneButNoIntersect) {
  Triangle t(Point(0.5, 2, 0.5),
             Point(2, 0.5, 0.5),
             Point(2, 2, 0.5));
  Box box(Point(0, 0, 0), Point(1, 1, 1));
  EXPECT_FALSE(box.Intersects(t));
}

TEST_F(GeometryTest, TriangleBoxPlaneAndIntersect) {
  Triangle t(Point(0.5, 1.5, 0.5),
             Point(1.5, 0.5, 0.5),
             Point(1.5, 1.5, 0.5));
  Box box(Point(0, 0, 0), Point(1, 1, 1));
  EXPECT_TRUE(box.Intersects(t));
}

TEST_F(GeometryTest, TriangleBoxCorner) {
  Triangle t(Point(0.5, 1.5, 0.5),
             Point(1.5, 0.5, 0.5),
             Point(0.5, 0.5, 1.5));
  Box box(Point(0, 0, 0), Point(1, 1, 1));
  EXPECT_TRUE(box.Intersects(t));

  // Inverse
  Triangle t2(Point(1.5, 0.5, 0.5),
              Point(0.5, 1.5, 0.5),
              Point(0.5, 0.5, 1.5));
  EXPECT_TRUE(box.Intersects(t2));
}

TEST_F(GeometryTest, TriangleBoxCornerOutside) {
  Triangle t(Point(0.8, 1.5, 0.8),
             Point(1.5, 0.8, 0.8),
             Point(0.8, 0.8, 1.5));
  Box box(Point(0, 0, 0), Point(1, 1, 1));
  EXPECT_FALSE(box.Intersects(t));
}

TEST_F(GeometryTest, Triangle2D_NoBoxOverlap) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(2, 2, 0),
              Point(2, 2, 0),
              Point(0, 2, 0));
  EXPECT_FALSE(t1.Intersects(t2));
  EXPECT_FALSE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_NoOverlap) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(2, 0, 0),
              Point(0.75, 1, 0),
              Point(0.75, 0.5, 0));
  EXPECT_FALSE(t1.Intersects(t2));
  EXPECT_FALSE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_NoOverlapHarder) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(2, 0, 0),
              Point(0.75, 1, 0),
              Point(1.5, -0.5, 0));
  EXPECT_FALSE(t1.Intersects(t2));
  EXPECT_FALSE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_NoOverlapStraight) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(2, 0.9, 0),
              Point(0.9, 0.9, 0),
              Point(1.5, -0.5, 0));
  EXPECT_FALSE(t1.Intersects(t2));
  EXPECT_FALSE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_NoOverlapAlmostStraight) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(2, 1, 0),
              Point(0.9, 0.9, 0),
              Point(1.5, -0.5, 0));
  EXPECT_FALSE(t1.Intersects(t2));
  EXPECT_FALSE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_PointContained) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(2, 0, 0),
              Point(0.5, 1, 0),
              Point(0.5, 0.4, 0));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_TRUE(t1.IntersectsNotCoincident(t2));
  EXPECT_TRUE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_Surrounds) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(3, 0, 0),
              Point(0, 3, 0),
              Point(-1, -1, 0));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_TRUE(t1.IntersectsNotCoincident(t2));
  EXPECT_TRUE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_Surrounded) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(0.2, 0.2, 0),
              Point(0.5, 0.2, 0),
              Point(0.2, 0.5, 0));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_TRUE(t1.IntersectsNotCoincident(t2));
  EXPECT_TRUE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_Wierd) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(1, 1, 0),
              Point(-0.5, 0, 0),
              Point(-0.5, -0.5, 0));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_TRUE(t1.IntersectsNotCoincident(t2));
  EXPECT_TRUE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_YPlane) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 0, 1));
  Triangle t2(Point(1, 0, 1),
              Point(-0.5, 0, 0),
              Point(-0.5, 0, -0.5));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_TRUE(t1.IntersectsNotCoincident(t2));
  EXPECT_TRUE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_XPlane) {
  Triangle t1(Point(0, 0, 0),
              Point(0, 1, 0),
              Point(0, 0, 1));
  Triangle t2(Point(0, 1, 1),
              Point(0, -0.5, 0),
              Point(0, -0.5, -0.5));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_TRUE(t1.IntersectsNotCoincident(t2));
  EXPECT_TRUE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_PointPoint) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(2, 0, 0),
              Point(1, 0, 0),
              Point(1, 1, 0));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_PointPoint2) {
  Triangle t1(Point(0,0,1),
              Point(2.4869,9.68583,1),
              Point(1.25333,9.92115,1));
  Triangle t2(Point(0,0,1),
              Point(3.68125,9.29776,1),
              Point(2.4869,9.68583,1));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_PointLine) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(0.5, 0, 0),
              Point(0.5, -0.5, 0),
              Point(0, -0.5, 0));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_Halfedge) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(1, 0, 0),
              Point(0, 0, 0),
              Point(0, -1, 0));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_Halfedge_SameDir) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, -1, 0));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_TRUE(t1.IntersectsNotCoincident(t2));
  EXPECT_TRUE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_HalfedgeY) {
  Triangle t1(Point(0, 0, 0),
              Point(0, 1, 0),
              Point(1, 0, 0));
  Triangle t2(Point(0, 1, 0),
              Point(0, 0, 0),
              Point(-1, 0, 0));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_HalfedgeZ) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 1, 0),
              Point(1, 0, 0));
  Triangle t2(Point(0, 0, 0),
              Point(0, 1, 0),
              Point(1, 1, 0));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_Halfedge_SameDirY) {
  Triangle t1(Point(0, 0, 0),
              Point(0, 1, 0),
              Point(1, 0, 0));
  Triangle t2(Point(0, 0, 0),
              Point(0, 1, 0),
              Point(-1, 0, 0));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_TRUE(t1.IntersectsNotCoincident(t2));
  EXPECT_TRUE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle2D_LineLine) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(0.5, 0, 0),
              Point(1.5, 0, 0),
              Point(0.5, -1, 0));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_TRUE(t1.IntersectsNotCoincident(t2));
  EXPECT_TRUE(t2.IntersectsNotCoincident(t1));
}


TEST_F(GeometryTest, Triangle3D_NoBoxOverlap) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(2, 2, 0),
              Point(2, 2, 0),
              Point(0, 2, 0));
  EXPECT_FALSE(t1.Intersects(t2));
  EXPECT_FALSE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_NoOverlap) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(2, 0, 0),
              Point(0.75, 1, 0.2),
              Point(0.75, 0.5, 0.5));
  EXPECT_FALSE(t1.Intersects(t2));
  EXPECT_FALSE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_NoOverlapHarder) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(2, 0, 0),
              Point(0.75, 1, 1),
              Point(1.5, -0.5, -1));
  EXPECT_FALSE(t1.Intersects(t2));
  EXPECT_FALSE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_NoOverlapStraight) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(2, 0.9, 0),
              Point(0.9, 0.9, 1),
              Point(1.5, -0.5, -1));
  EXPECT_FALSE(t1.Intersects(t2));
  EXPECT_FALSE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_Wierd) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(1, 1, 1),
              Point(-0.5, 0, -1),
              Point(-0.5, -0.5, -1));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_TRUE(t1.IntersectsNotCoincident(t2));
  EXPECT_TRUE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_NoOverlapAlmostStraight) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(2, 1, 0),
              Point(0.9, 0.9, 1),
              Point(1.5, -0.5, -1));
  EXPECT_FALSE(t1.Intersects(t2));
  EXPECT_FALSE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_PointMissed) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(2, 0, 0),
              Point(0.5, 1, 1),
              Point(0.5, 0.4, -1));
  EXPECT_FALSE(t1.Intersects(t2));
  EXPECT_FALSE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_PointContained) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(2, 0, 0),
              Point(0.5, 0.5, 1),
              Point(0.5, 0, -1));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_TRUE(t1.IntersectsNotCoincident(t2));
  EXPECT_TRUE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_Surrounds) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(3, 0, 1),
              Point(0, 3, 1),
              Point(-1, -1, -1));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_TRUE(t1.IntersectsNotCoincident(t2));
  EXPECT_TRUE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_Surrounded) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(0.2, 0.2, 0.2),
              Point(0.5, 0.2, -0.2),
              Point(0.2, 0.5, -0.2));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_TRUE(t1.IntersectsNotCoincident(t2));
  EXPECT_TRUE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_SinglePoint) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(1, 1, 1),
              Point(0.5, 0.2, 0),
              Point(0, 0, 1));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_LineSegment) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(0.4, 0.1, 0),
              Point(0.5, 0.2, 0),
              Point(0, 0, 1));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_TRUE(t1.IntersectsNotCoincident(t2));
  EXPECT_TRUE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_PointPoint) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(1, 1, 1),
              Point(1, 0, 0),
              Point(1, 0, 1));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_PointPoint2) {
  Triangle t1(Point(0,0,1),
              Point(3.68125,9.29776,1),
              Point(2.4869,9.68583,1));
  Triangle t2(Point(1.25333,9.92115,1),
              Point(2.4869,9.68583,1),
              Point(2.4869,9.68583,1.95));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_PointLine) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(1, 1, 1),
              Point(0.5, 0, 0),
              Point(1, 0, 1));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_Halfedge) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(1, 0, 0),
              Point(0, 0, 0),
              Point(1, 0, 1));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_Halfedge_SameDir) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(1, 0, 1));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_TRUE(t1.IntersectsNotCoincident(t2));
  EXPECT_TRUE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_LineLine) {
  Triangle t1(Point(0, 0, 0),
              Point(1, 0, 0),
              Point(0, 1, 0));
  Triangle t2(Point(0.5, 0, 0),
              Point(1.5, 0, 0),
              Point(1, 0, 1));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_TRUE(t1.IntersectsNotCoincident(t2));
  EXPECT_TRUE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_Corner) {
  Triangle t1(Point(0, 0, 0),
              Point(0, 0, 1),
              Point(0, 1, 1));
  Triangle t2(Point(0, 0, 0),
              Point(0, 1, 0),
              Point(1, 1, 0));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}


TEST_F(GeometryTest, Triangle3D_Halfedge2) {
  Triangle t1(Point(0, 1, 1),
              Point(1, 1, 1),
              Point(1, 1, 0));
  Triangle t2(Point(1, 0, 1),
              Point(1, 1, 0),
              Point(1, 1, 1));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, Triangle3D_SharesOpposingHalfedge) {
  Triangle t1(Point(0, 1, 1),
              Point(1, 1, 1),
              Point(1, 1, 0));
  Triangle t2(Point(1, 0, 1),
              Point(1, 1, 0),
              Point(1, 1, 1));
  for (int i = 0; i < 3; ++i) {
    t1 = t1.Shuffled();
    for (int j = 0; j < 3; ++j) {
      t2 = t2.Shuffled();
      EXPECT_TRUE(t1.SharesOpposingHalfedge(t2));
      EXPECT_TRUE(t2.SharesOpposingHalfedge(t1));
    }
  }

  t1 = t1.Flipped();
  for (int i = 0; i < 3; ++i) {
    t1 = t1.Shuffled();
    for (int j = 0; j < 3; ++j) {
      t2 = t2.Shuffled();
      EXPECT_FALSE(t1.SharesOpposingHalfedge(t2));
      EXPECT_FALSE(t2.SharesOpposingHalfedge(t1));
    }
  }

  EXPECT_FALSE(t1.SharesOpposingHalfedge(t1));
  EXPECT_TRUE(t1.SharesOpposingHalfedge(t1.Flipped()));
}

TEST_F(GeometryTest, MoreTests1) {
  Triangle t1(Point(9.822872507297278232841e+00,
                    -1.873813145802768476145e+00,
                    1.000000000000000000000e+00),
              Point(9.822872507297278232841e+00,
                    -1.873813145802768476145e+00,
                    1.949999999999999955591e+00),
              Point(9.980267284285949003220e+00,
                    -6.279051952417391557759e-01,
                    1.000000000000000000000e+00));
  Triangle t2(Point(9.822872507297278232841e+00,
                    -1.873813145802768476145e+00,
                    1.949999999999999955591e+00),
              Point(9.980267284285949003220e+00,
                    -6.279051952417391557759e-01,
                    1.949999999999999955591e+00),
              Point(9.980267284285949003220e+00,
                    -6.279051952417391557759e-01,
                    1.000000000000000000000e+00));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, MoreTests2) {
  Triangle t1(Point(-2.408768370458241214749e+00,
                    -4.381533400246989451432e+00,
                    1.000000000000000000000e+00),
              Point(9.048270524687181293189e+00,
                    -4.257792915593379134975e+00,
                    1.000000000000000000000e+00),
              Point(9.510565162969896846334e+00,
                    -3.090169943692962384318e+00,
                    1.000000000000000000000e+00));
  Triangle t2(Point(-5.877852522828588277548e+00,
                    -8.090169943819326192624e+00,
                    1.000000000000000000000e+00),
              Point(-5.877852522828588277548e+00,
                    -8.090169943819326192624e+00,
                    1.949999999999999955591e+00),
              Point(-2.408768370458241214749e+00,
                    -4.381533400246989451432e+00,
                    1.000000000000000000000e+00));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, MoreTests3) {
  Triangle t1(Point(-9.744795671172296280815e+00,
                    -2.177902345468328171307e+00,
                    1.667499999999999715783e+01),
              Point(-9.510565162908692471433e+00,
                    -3.090169943881334368996e+00,
                    1.619999999999999928946e+01),
              Point(-9.666718835084427752236e+00,
                    -2.481991544939330385233e+00,
                    1.525000000000000000000e+01));
  Triangle t2(Point(-9.822872507260164809395e+00,
                    -1.873813145997326401471e+00,
                    1.382499999999999928946e+01),
              Point(-9.666718835084427752236e+00,
                    -2.481991544939330385233e+00,
                    1.525000000000000000000e+01),
              Point(-9.510565162908692471433e+00,
                    -3.090169943881334368996e+00,
                    1.429999999999999893419e+01));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, MoreTests4) {
  Triangle t1(Point(-5.877852522828588277548e+00,
                    -8.090169943819326192624e+00,
                    1.905000000000000071054e+01),
              Point(-2.202397135253424842460e+00,
                    -9.685181081498246413730e+00,
                    1.758789062500000000000e+01),
              Point(-1.331948685243626195884e+00,
                    -9.557044431191727085206e+00,
                    6.125732421875000000000e+00));
  Triangle t2(Point(-7.229738023208197361669e+00,
                    -6.754106728883098931249e+00,
                    1.067626953125000000000e+01),
              Point(-1.331948685243626195884e+00,
                    -9.557044431191727085206e+00,
                    6.125732421875000000000e+00),
              Point(-6.939434084271201008676e+00,
                    -7.160885138809919503444e+00,
                    2.899999999999999911182e+00));

  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, MoreTests5) {
  Triangle t1(Point(-9.500000000000000000000e+00,
                    9.408146252119495293486e-11,
                    1.769999999999999928946e+01),
              Point(-9.500000000000000000000e+00,
                    9.408146252119495293486e-11,
                    1.860000000000000142109e+01),
              Point(-9.496999798189005304039e+00,
                    2.387359068050048660492e-01,
                    1.769999999999999928946e+01));
  Triangle t2(Point(-9.500000000000000000000e+00,
                    9.408146252119495293486e-11,
                    1.860000000000000142109e+01),
              Point(-9.496999798189005304039e+00,
                    2.387359068050048660492e-01,
                    1.860000000000000142109e+01),
              Point(-9.496999798189005304039e+00,
                    2.387359068050048660492e-01,
                    1.769999999999999928946e+01));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, MoreTests6) {
  Triangle t1(Point(-8.09,5.877,10.5),
              Point(-4.58211,8.421,10.5),
              Point(-4.42122,8.46783,1));
  Box b1(Point(-7.823,5.389,5.5), Point(-4.887,9.136,6.9));
  Box b2(Point(-6.589,6.842,5.5), Point(-4.887,9.136,6.9));
  Box b3 = b1 + Point(0, 0, 10);
  EXPECT_TRUE(b1.Intersects(t1));
  EXPECT_TRUE(b2.Intersects(t1));
  EXPECT_FALSE(b3.Intersects(t1));
  EXPECT_TRUE(b1.FullyContains(b2));
}

TEST_F(GeometryTest, MoreTests7) {
  Triangle t1(Point(4.000000000000000000000e+01,
                    3.198999999761581491953e+01,
                    4.590000000000000568434e+01),
              Point(4.010000000000000142109e+01,
                    3.198999999761581491953e+01,
                    4.590000000000000568434e+01),
              Point(4.000000000000000000000e+01,
                    3.198999999761581491953e+01,
                    4.600000000000000000000e+01));
  Triangle t2(Point(3.990000000000000568434e+01,
                    3.198999999761581491953e+01,
                    4.590000000000000568434e+01),
              Point(4.050000000000000000000e+01,
                    3.178999999761581563007e+01,
                    4.570000000000000284217e+01),
              Point(4.010000000000000142109e+01,
                    3.198999999761581491953e+01,
                    4.590000000000000568434e+01));
  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

TEST_F(GeometryTest, MoreTests8) {
  // (5.9,0.1,-0.01),(9.9,4.1,-0.01),(6,0.2,-0.01) against
  // (5.9,0.09,0),   (6,0.2,-0.01),  (6,0.19,0)
  Triangle t1(Point(5.900000000000000355271e+00,
                    1.000000000000000055511e-01,
                    -1.000000238418579170951e-02),
              Point(9.900000000000000355271e+00,
                    4.100000000000000532907e+00,
                    -1.000000238418579170951e-02),
              Point(6.000000000000000000000e+00,
                    2.000000000000000111022e-01,
                    -1.000000238418579170951e-02));
  Triangle t2(Point(5.900000000000000355271e+00,
                    8.999999761581421731105e-02,
                    0.000000000000000000000e+00),
              Point(6.000000000000000000000e+00,
                    2.000000000000000111022e-01,
                    -1.000000238418579170951e-02),
              Point(6.000000000000000000000e+00,
                    1.899999976158142089844e-01,
                    0.000000000000000000000e+00));
  double area1 = (t1.p1() - t1.p0()).Cross(t1.p2() - t1.p0()).magnitude() / 2;
  double area2 = (t2.p1() - t2.p0()).Cross(t2.p2() - t2.p0()).magnitude() / 2;
  LG << area1 << ", " << area2;

  EXPECT_TRUE(t1.Intersects(t2));
  EXPECT_TRUE(t2.Intersects(t1));
  EXPECT_FALSE(t1.IntersectsNotCoincident(t2));
  EXPECT_FALSE(t2.IntersectsNotCoincident(t1));
}

}  // anonymous namespace
}  // namespace printer
