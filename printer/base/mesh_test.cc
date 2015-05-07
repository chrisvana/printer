// Copyright 2015
// Author: Christopher Van Arsdale

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "common/test/test.h"
#include "common/base/flags.h"
#include "common/base/profiler.h"
#include "common/log/log.h"
#include "printer/base/mesh.h"
#include "printer/stl/format.h"

using std::vector;
using std::map;
using std::string;

DEFINE_string(dump_stl, "",
              "If non-empty, object to dump (cube, simplified)");

DEFINE_bool(run_triangle_benchmarks, false,
            "If true, run the benchmarks. Should probably be paired with "
            "--gtest_filter to specify which benchmark to run.");

namespace printer {
namespace {

class MeshTest : public testing::Test {
 public:
  MeshTest() {
    test_points_.push_back(Point(0, 0, 0));
    test_points_.push_back(Point(1, 0, 0));
    test_points_.push_back(Point(0, 1, 0));
    test_points_.push_back(Point(1, 1, 0));

    test_points_.push_back(Point(0, 0, 1));
    test_points_.push_back(Point(1, 0, 1));
    test_points_.push_back(Point(0, 1, 1));
    test_points_.push_back(Point(1, 1, 1));

    name_[Bottom1()] = "bottom1";
    name_[Bottom2()] = "bottom2";
    name_[Top1()] = "top1";
    name_[Top2()] = "top2";
    name_[Front1()] = "front1";
    name_[Front2()] = "front2";
    name_[Back1()] = "back1";
    name_[Back2()] = "back2";
    name_[Left1()] = "left1";
    name_[Left2()] = "left2";
    name_[Right1()] = "right1";
    name_[Right2()] = "right2";
  }

  Triangle TestTriangle(int p0, int p1, int p2) {
    return Triangle(TestPoint(p0), TestPoint(p1), TestPoint(p2));
  }

  Point front_left_bottom() { return TestPoint(0); }
  Point front_right_bottom() { return TestPoint(1); }
  Point back_left_bottom() { return TestPoint(2); }
  Point back_right_bottom() { return TestPoint(3); }

  Point front_left_top() { return TestPoint(4); }
  Point front_right_top() { return TestPoint(5); }
  Point back_left_top() { return TestPoint(6); }
  Point back_right_top() { return TestPoint(7); }

  Triangle Bottom1() {
    return Triangle(front_left_bottom(),
                    back_left_bottom(),
                    back_right_bottom()).Normalized();
  }

  Triangle Bottom2() {
    return Triangle(front_right_bottom(),
                    front_left_bottom(),
                    back_right_bottom()).Normalized();
  }

  Triangle Left1() {
    return Triangle(front_left_bottom(),
                    front_left_top(),
                    back_left_top()).Normalized();
  }

  Triangle Left2() {
    return Triangle(back_left_top(),
                    back_left_bottom(),
                    front_left_bottom()).Normalized();
  }

  Triangle Right1() {
    return Triangle(back_right_bottom(),
                    back_right_top(),
                    front_right_top()).Normalized();
  }

  Triangle Right2() {
    return Triangle(front_right_top(),
                    front_right_bottom(),
                    back_right_bottom()).Normalized();
  }

  Triangle Front1() {
    return Triangle(front_right_bottom(),
                    front_right_top(),
                    front_left_top()).Normalized();
  }

  Triangle Front2() {
    return Triangle(front_left_top(),
                    front_left_bottom(),
                    front_right_bottom()).Normalized();
  }

  Triangle Back1() {
    return Triangle(back_right_bottom(),
                    back_left_bottom(),
                    back_left_top()).Normalized();
  }

  Triangle Back2() {
    return Triangle(back_left_top(),
                    back_right_top(),
                    back_right_bottom()).Normalized();
  }

  Triangle Top1() {
    return Triangle(back_right_top(),
                    back_left_top(),
                    front_left_top()).Normalized();
  }

  Triangle Top2() {
    return Triangle(front_left_top(),
                    front_right_top(),
                    back_right_top()).Normalized();
  }

  Point TestPoint(int p) {
    CHECK_GE(p, 0);
    CHECK_LT(p, test_points_.size());
    return test_points_[p];
  }

  void AddCube(TriangleMesh* mesh) {
    CHECK(mesh->AddTriangle(Bottom1()));
    CHECK(mesh->AddTriangle(Bottom2()));
    CHECK(mesh->AddTriangle(Top1()));
    CHECK(mesh->AddTriangle(Top2()));
    CHECK(mesh->AddTriangle(Left1()));
    CHECK(mesh->AddTriangle(Left2()));
    CHECK(mesh->AddTriangle(Right1()));
    CHECK(mesh->AddTriangle(Right2()));
    CHECK(mesh->AddTriangle(Front1()));
    CHECK(mesh->AddTriangle(Front2()));
    CHECK(mesh->AddTriangle(Back1()));
    CHECK(mesh->AddTriangle(Back2()));
  }

 protected:
  ::testing::AssertionResult AssertSetEquals(
       const char* m_expr,
       const char* n_expr,
       const TriangleMesh::TriangleList& a_list,
       const TriangleMesh::TriangleList& b_list) {
    TriangleMesh::TriangleSet a(a_list.begin(), a_list.end());
    TriangleMesh::TriangleSet b(b_list.begin(), b_list.end());
    if (a_list.size() != a.size()) {
      testing::AssertionResult r = ::testing::AssertionFailure();
      r << m_expr << " contains duplicate entries." << std::endl;
    }
    if (b_list.size() != b.size()) {
      testing::AssertionResult r = ::testing::AssertionFailure();
      r << n_expr << " contains duplicate entries." << std::endl;
    }

    if (a != b) {
      testing::AssertionResult r = ::testing::AssertionFailure();
      r << m_expr << " != " << n_expr << ":" << std::endl;
      r << m_expr << "{" << std::endl;
      for (const Triangle& a_t : a) {
        r << "  " << (name_[a_t] != "" ? name_[a_t] : a_t.DebugString())
          << (b.find(a_t) == b.end() ? "        (mismatch)" : "")
          << std::endl;
      }
      r << "}" << std::endl;
      r << n_expr << "{" << std::endl;
      for (const Triangle& b_t : b) {
        r << "  " << (name_[b_t] != "" ? name_[b_t] : b_t.DebugString())
          << (a.find(b_t) == a.end() ? "        (mismatch)" : "")
          << std::endl;
      }
      r << "}" << std::endl;
      return r;
    }

    return testing::AssertionSuccess();
  }

  vector<Point> test_points_;
  map<Triangle, string> name_;
};

TEST_F(MeshTest, CubeRight) {
  EXPECT_EQ(Point(-1, 0, 0), Left1().normal())
      << Left1().normal().DebugString();
  EXPECT_EQ(Point(-1, 0, 0), Left2().normal())
      << Left2().normal().DebugString();

  EXPECT_EQ(Point(1, 0, 0), Right1().normal())
      << Right1().normal().DebugString();
  EXPECT_EQ(Point(1, 0, 0), Right2().normal())
      << Right2().normal().DebugString();

  EXPECT_EQ(Point(0, 0, 1), Top1().normal())
      << Top1().normal().DebugString();
  EXPECT_EQ(Point(0, 0, 1), Top2().normal())
      << Top2().normal().DebugString();

  EXPECT_EQ(Point(0, 0, -1), Bottom1().normal())
      << Bottom1().normal().DebugString();
  EXPECT_EQ(Point(0, 0, -1), Bottom2().normal())
      << Bottom2().normal().DebugString();

  EXPECT_EQ(Point(0, -1, 0), Front1().normal())
      << Front1().normal().DebugString();
  EXPECT_EQ(Point(0, -1, 0), Front2().normal())
      << Front2().normal().DebugString();


  EXPECT_EQ(Point(0, 1, 0), Back1().normal())
      << Back1().normal().DebugString();
  EXPECT_EQ(Point(0, 1, 0), Back2().normal())
      << Back2().normal().DebugString();
}

TEST_F(MeshTest, TestManifoldEmpty) {
  TriangleMesh mesh;
  ASSERT_FALSE(mesh.CheckManifold());
}

TEST_F(MeshTest, TestAddsTriangles) {
  TriangleMesh mesh;
  ASSERT_TRUE(mesh.AddTriangle(Bottom1()));
  ASSERT_TRUE(mesh.AddTriangle(Bottom2()));
  ASSERT_EQ(2, mesh.size());
  for (TriangleMesh::TriangleIterator iter(mesh); !iter.Done(); iter.Next()) {
    EXPECT_TRUE(iter.triangle() == Bottom1() ||
                iter.triangle() == Bottom2())
        << iter.triangle().DebugString();
  }
}

TEST_F(MeshTest, TestManifoldBroken) {
  TriangleMesh mesh;
  ASSERT_TRUE(mesh.AddTriangle(Bottom1()));
  ASSERT_TRUE(mesh.AddTriangle(Bottom2()));
  ASSERT_TRUE(mesh.AddTriangle(Top1()));
  ASSERT_TRUE(mesh.AddTriangle(Top2()));
  ASSERT_TRUE(mesh.AddTriangle(Left1()));
  ASSERT_TRUE(mesh.AddTriangle(Left2()));
  ASSERT_TRUE(mesh.AddTriangle(Right1()));
  ASSERT_TRUE(mesh.AddTriangle(Right2()));
  ASSERT_TRUE(mesh.AddTriangle(Front1()));
  ASSERT_TRUE(mesh.AddTriangle(Front2()));
  ASSERT_TRUE(mesh.AddTriangle(Back1()));

  ASSERT_FALSE(mesh.CheckManifold());

  // Complete the cube.
  ASSERT_TRUE(mesh.AddTriangle(Back2()));

  ASSERT_TRUE(mesh.CheckManifold());
}


TEST_F(MeshTest, TestEdgeRemoval) {
  VLOG(2) << "bottom1: " <<  Bottom1().DebugString();
  VLOG(2) << "bottom2: " << Bottom2().DebugString();
  VLOG(2) << "top1: " << Top1().DebugString();
  VLOG(2) << "top2: " << Top2().DebugString();
  VLOG(2) << "left1: " << Left1().DebugString();
  VLOG(2) << "left2: " << Left2().DebugString();
  VLOG(2) << "right1: " << Right1().DebugString();
  VLOG(2) << "right2: " << Right2().DebugString();
  VLOG(2) << "front1: " << Front1().DebugString();
  VLOG(2) << "front2: " << Front2().DebugString();
  VLOG(2) << "back1: " << Back1().DebugString();
  VLOG(2) << "back2: " << Back2().DebugString();

  std::ostream* out = &std::cout;
  STLFormatWriter::Input input(out);
  STLFormatWriter writer(input);

  TriangleMesh mesh;
  mesh.InitializeOctree(Box(Point(-1, -1, -1), Point(1, 1, 1)));
  AddCube(&mesh);
  if (FLAGS_dump_stl == "cube") {
    writer.WriteMesh("cube", mesh);
  }
  ASSERT_TRUE(mesh.CheckManifold());

  TriangleMesh::TriangleList old_t, new_t;
  ASSERT_TRUE(mesh.TestEdgeCollapse(
      Edge(front_left_top(), front_left_bottom()),
      front_left_bottom(),
      &old_t, &new_t));

  TriangleMesh::TriangleList old_expected;
  old_expected.push_back(Bottom1());
  old_expected.push_back(Bottom2());
  old_expected.push_back(Top1());
  old_expected.push_back(Top2());
  old_expected.push_back(Front1());
  old_expected.push_back(Front2());
  old_expected.push_back(Left1());
  old_expected.push_back(Left2());
  EXPECT_PRED_FORMAT2(AssertSetEquals, old_expected, old_t);
  EXPECT_EQ(6, new_t.size());
  ASSERT_TRUE(mesh.ApplyUpdate(old_t, new_t, true));
  ASSERT_TRUE(mesh.CheckManifold());

  if (FLAGS_dump_stl == "simplified") {
    writer.WriteMesh("simplified", mesh);
  }
}

TEST_F(MeshTest, TestAddTriangleBenchmark) {
  if (!FLAGS_run_triangle_benchmarks) {
    return;
  }

  TriangleMesh mesh;
  for (size_t i = 0; i < (1 << 21); ++i) {
    Triangle t(Point(i, i, i),
               Point(i + 1, i + 1, i + 1),
               Point(i, i + 1, i + 1));
    mesh.AddTriangle(t);
  }
}

TEST_F(MeshTest, TestMergeTriangleBenchmark) {
  if (!FLAGS_run_triangle_benchmarks) {
    return;
  }

  TriangleMesh mesh1, mesh2;
  size_t i = 0;
  for (; i < (1 << 20); ++i) {
    Triangle t(Point(i, i, i),
               Point(i + 1, i + 1, i + 1),
               Point(i, i + 1, i + 1));
    mesh1.AddTriangle(t);
  }
  for (; i < (1 << 21); ++i) {
    Triangle t(Point(i, i, i),
               Point(i + 1, i + 1, i + 1),
               Point(i, i + 1, i + 1));
    mesh2.AddTriangle(t);
  }

  LOG(INFO) << "Running merge.";
  mesh1.Merge(mesh2);
  LOG(INFO) << "Merge done.";
}

}  // anonymous namespace
}  // namespace printer
