// Copyright 2015
// Author: Christopher Van Arsdale

#include <vector>
#include "common/test/test.h"
#include "common/log/log.h"
#include "printer/base/geometry.h"
#include "printer/simplify/triangulation.h"

using std::vector;

namespace printer {
namespace {
class TriangulationTest : public testing::Test {
 public:
  TriangulationTest() {}
};

TEST_F(TriangulationTest, SimpleSquare) {
  Triangulation::PolygonLoop loop;
  loop.push_back(Point(1, 0, 0));
  loop.push_back(Point(1, 1, 0));
  loop.push_back(Point(0, 1, 0));
  loop.push_back(Point(0, 0, 0));

  Triangulation triangulate(Point(1, 0, 0), Point(0, 1, 0));
  triangulate.SetOuterLoop(&loop);

  vector<Triangle> output;
  ASSERT_TRUE(triangulate.Triangulate(&output));
  EXPECT_EQ(2, output.size());
}
}  // anonymous namespace
}  // namespace printer
