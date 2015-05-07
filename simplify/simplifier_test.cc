// Copyright 2015
// Author: Christopher Van Arsdale

#include <vector>
#include "common/test/test.h"
#include "common/base/profiler.h"
#include "common/log/log.h"
#include "printer/base/geometry.h"
#include "printer/base/mesh.h"
#include "printer/base/octree.h"
#include "printer/simplify/simplifier.h"
#include "printer/simplify/test_helper.h"

using std::vector;

namespace printer {
namespace {
class SimplifierTest : public testing::Test {
 public:
  SimplifierTest() {}
};

TEST_F(SimplifierTest, BuildMesh) {
  // Self-test.
  TriangleMesh mesh;
  TestHelper::GetCylinderDefault(1, 20, 10, &mesh);
  ASSERT_TRUE(mesh.CheckManifold());
  HeapProfilerDump("BuildMeshHeap");
}

TEST_F(SimplifierTest, Simplify) {
  // Set up mesh
  TriangleMesh mesh;
  TestHelper::GetCylinderDefault(1, 20, 10, &mesh);
  ASSERT_TRUE(mesh.InitializeOctree(Box(Point(-20, -20, 0),
                                        Point(20, 20, 25))));
  ASSERT_TRUE(mesh.CheckManifold());

  // Set up inner boundary
  TriangleMesh inner_boundary;
  TestHelper::GetCylinder(1.5, 19.5, 20, 9.5, &inner_boundary);
  ASSERT_TRUE(inner_boundary.InitializeOctree(
      Box(Point(-20, -20, 0), Point(20, 20, 25))));
  ASSERT_TRUE(inner_boundary.CheckManifold());
  ASSERT_FALSE(mesh.IntersectsWithOctree(inner_boundary));

  // Set up outer boundary
  TriangleMesh outer_boundary;
  TestHelper::GetCylinder(0.5, 20.5, 20, 10.5, &outer_boundary);
  ASSERT_TRUE(outer_boundary.InitializeOctree(Box(Point(-20, -20, 0),
                                                  Point(20, 20, 25))));
  ASSERT_TRUE(outer_boundary.CheckManifold());
  ASSERT_FALSE(mesh.IntersectsWithOctree(outer_boundary));

  // Merge boundaries, flipping inner one to point inwards.
  for (TriangleMesh::TriangleIterator iter(inner_boundary);
       !iter.Done(); iter.Next()) {
    ASSERT_TRUE(outer_boundary.AddTriangle(iter.triangle().Flipped()));
  }
  ASSERT_TRUE(outer_boundary.CheckManifold());
  LOG(INFO) << "Total octree nodes in boundary: "
            << outer_boundary.octree_node_count();
  ASSERT_FALSE(mesh.IntersectsWithOctree(outer_boundary));

  // Set up simplifier
  Simplifier::Input input;
  Simplifier simplifier(input);

  // Simplify
  LOG(INFO) << "Size before simplification: " << mesh.size();
  simplifier.Execute(outer_boundary, &mesh);
  LOG(INFO) << "Size after simplification: " << mesh.size();

  // Sanity check
  EXPECT_FALSE(mesh.IntersectsWithOctree(outer_boundary));
  EXPECT_FALSE(outer_boundary.IntersectsWithOctree(mesh));
  EXPECT_LT(mesh.size(), 200);

  TestHelper::MaybeDump(mesh);
}

}  // anonymous namespace
}  // namespace printer
