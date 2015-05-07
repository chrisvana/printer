// Copyright 2015
// Author: Christopher Van Arsdale

#include "common/test/test.h"
#include "printer/base/geometry.h"
#include "printer/base/octree.h"

namespace printer {
namespace {

class OctreeTest : public testing::Test {
 public:
  OctreeTest() {}  
};

TEST_F(OctreeTest, TestSimple) {
  Octree a(Box(Point(0, 0, 0), Point(10, 10, 10)));
  ASSERT_TRUE(a.AddTriangle(Triangle(Point(0, 0, 0),
                                     Point(0, 0, 1),
                                     Point(0, 1, 1))));

  Octree b(Box(Point(0, 0, 0), Point(10, 10, 10)));
  ASSERT_TRUE(b.AddTriangle(Triangle(Point(0, 0, 0),
                                     Point(0, 0, 1),
                                     Point(0, 1, 1))));

  EXPECT_TRUE(a.Intersects(b));
  EXPECT_TRUE(b.Intersects(a));
}

TEST_F(OctreeTest, TestDeeper) {
  Octree a(Box(Point(0, 0, 0), Point(10, 10, 10)));
  a.AddTriangle(Triangle(Point(0, 0, 0),
                         Point(0, 0, 1),
                         Point(0, 1, 1)));
  a.AddTriangle(Triangle(Point(1, 0, 0),
                         Point(0, 0, 4),
                         Point(0, 4, 4)));
  a.AddTriangle(Triangle(Point(1, 1, 1),
                         Point(7, 7, 7),
                         Point(3, 7, 0)));

  Octree b(Box(Point(0, 0, 0), Point(10, 10, 10)));
  b.AddTriangle(Triangle(Point(0, 0, 0),
                         Point(0, 0, 1),
                         Point(0, 1, 1)));

  EXPECT_TRUE(a.Intersects(b));
  EXPECT_TRUE(b.Intersects(a));
}
}  // anonymous namespace
}  // namespace printer
