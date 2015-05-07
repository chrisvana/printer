// Copyright 2015
// Author: Christopher Van Arsdale

#include "common/test/test.h"
#include "printer/base/geometry.h"
#include "printer/base/aabb_tree.h"

namespace printer {
namespace {

class AABBTreeTest : public testing::Test {
 public:
  AABBTreeTest() {}
  
};

TEST_F(AABBTreeTest, TestInsert) {
  AABBTree a;
  ASSERT_TRUE(a.AddTriangle(Triangle(Point(0, 0, 0),
                                     Point(0, 0, 1),
                                     Point(0, 1, 1))));
  ASSERT_FALSE(a.AddTriangle(Triangle(Point(0, 0, 0),
                                      Point(0, 0, 1),
                                      Point(0, 1, 1))));

  AABBTree::Iterator iter = a.iterator();
  ASSERT_FALSE(iter.Done());
  ASSERT_TRUE(iter.triangle() == Triangle(Point(0, 0, 0),
                                          Point(0, 0, 1),
                                          Point(0, 1, 1)));
  iter.Next();
  ASSERT_TRUE(iter.Done());
}

TEST_F(AABBTreeTest, TestSimple) {
  AABBTree a, b;
  ASSERT_TRUE(a.AddTriangle(Triangle(Point(0, 0, 0),
                                     Point(0, 0, 1),
                                     Point(0, 1, 1))));
  ASSERT_TRUE(b.AddTriangle(Triangle(Point(0, 0, 0),
                                     Point(0, 0, 1),
                                     Point(0, 1, 1))));
  EXPECT_TRUE(a.Intersects(a));
  EXPECT_TRUE(a.Intersects(b));
  EXPECT_TRUE(b.Intersects(a));
  EXPECT_TRUE(b.Intersects(b));
}

TEST_F(AABBTreeTest, TestDeeper) {
  AABBTree a, b;
  ASSERT_TRUE(a.AddTriangle(Triangle(Point(0, 0, 0),
                                     Point(0, 0, 1),
                                     Point(0, 1, 1))));
  ASSERT_TRUE(a.AddTriangle(Triangle(Point(1, 0, 0),
                                     Point(0, 0, 4),
                                     Point(0, 4, 4))));
  ASSERT_TRUE(a.AddTriangle(Triangle(Point(1, 1, 1),
                                     Point(7, 7, 7),
                                     Point(3, 7, 0))));

  // B is empty.
  EXPECT_FALSE(a.Intersects(b));
  EXPECT_FALSE(b.Intersects(a));
  EXPECT_FALSE(b.Intersects(b));
  EXPECT_TRUE(a.Intersects(a));


  // B intersects
  ASSERT_TRUE(b.AddTriangle(Triangle(Point(0, 0, 0),
                                     Point(0, 0, 1),
                                     Point(0, 1, 1))));
  EXPECT_TRUE(a.Intersects(b));
  EXPECT_TRUE(b.Intersects(a));
  EXPECT_TRUE(b.Intersects(b));

  // B is empty again.
  ASSERT_TRUE(b.RemoveTriangle(Triangle(Point(0, 0, 0),
                                        Point(0, 0, 1),
                                        Point(0, 1, 1))));
  EXPECT_FALSE(a.Intersects(b));
  EXPECT_FALSE(b.Intersects(a));
  EXPECT_FALSE(b.Intersects(b));

  // B is non-intersecting
  ASSERT_TRUE(b.AddTriangle(Triangle(Point(15, 15, 5),
                                     Point(15, 15, 7),
                                     Point(15, 14, 6))));
  EXPECT_FALSE(a.Intersects(b));
  EXPECT_FALSE(b.Intersects(a));
  EXPECT_TRUE(b.Intersects(b));

  // B is intersecting again
  ASSERT_TRUE(b.AddTriangle(Triangle(Point(0, 0, 0),
                                     Point(0, 0, 1),
                                     Point(0, 1, 1))));
  EXPECT_TRUE(a.Intersects(b));
  EXPECT_TRUE(b.Intersects(a));
  EXPECT_TRUE(b.Intersects(b));
}
}  // anonymous namespace
}  // namespace printer
