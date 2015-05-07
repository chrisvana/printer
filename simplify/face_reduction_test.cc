// Copyright 2015
// Author: Christopher Van Arsdale

#include <string>
#include <vector>
#include "common/log/log.h"
#include "common/strings/strutil.h"
#include "common/test/test.h"
#include "printer/base/geometry.h"
#include "printer/base/mesh.h"
#include "printer/simplify/face_reduction.h"
#include "printer/simplify/test_helper.h"

namespace printer {
namespace {
class FaceReductionTest : public testing::Test {
 public:
  FaceReductionTest() {}
  static Point ConvertPoint(const std::vector<StringPiece>& pieces, int start) {
    return Point(std::stod(pieces[start].as_string()),
                 std::stod(pieces[start+1].as_string()),
                 std::stod(pieces[start+2].as_string()));
  }
  static Triangle ConvertTriangle(const std::vector<StringPiece>& pieces,
                                  int start) {
    return Triangle(ConvertPoint(pieces, start),
                    ConvertPoint(pieces, start+3),
                    ConvertPoint(pieces, start+6));
  }
};

TEST_F(FaceReductionTest, Simplify) {
  // Set up meshes
  TriangleMesh mesh;
  TestHelper::GetCylinderDefault(1, 20, 10, &mesh);
  TriangleMesh mesh2;
  mesh2.CopyFrom(mesh);
  LOG(INFO) << "Input size: " << mesh.size();
  int original_size = mesh.size();

  // Normal reduce.
  FaceReduction::ReduceFaces(&mesh);

  LOG(INFO) << "Output size: " << mesh.size();
  EXPECT_LT(mesh.size(), original_size / 2);
  ASSERT_TRUE(mesh.CheckManifold());
  ASSERT_TRUE(mesh.InitializeOctree(mesh.MinimalBoundingBox()));
  TestHelper::MaybeDump(mesh);

  // Parallel reduce.
  FaceReduction::ReduceFacesParallel(&mesh2);
  EXPECT_EQ(mesh.size(), mesh2.size());
}

}  // anonymous namespace
}  // namespace printer
