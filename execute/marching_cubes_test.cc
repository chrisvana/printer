// Copyright 2015
// Author: Christopher Van Arsdale

#include <fstream>
#include <vector>
#include "common/test/test.h"
#include "common/log/log.h"
#include "common/thread/threadpool.h"
#include "printer/base/geometry.h"
#include "printer/base/mesh.h"
#include "printer/execute/print_box.h"
#include "printer/execute/marching_cubes.h"

using std::vector;

namespace printer {
namespace {

class MarchingCubesTest : public testing::Test {
 public:
  MarchingCubesTest() {}
};

TEST_F(MarchingCubesTest, Empty) {
  TriangleMesh mesh;
  BinaryPrintBox box(10, 10, 10, 1, 1);
  MarchingCubes cubes;
  cubes.Execute(box, &mesh);
  EXPECT_EQ(0, mesh.size());
}

TEST_F(MarchingCubesTest, Box) {
  TriangleMesh mesh;
  BinaryPrintBox box(10, 10, 10, 1, 1);
  box.FillRegion(3, 5, 3, 5, 3, 5, 1);
  MarchingCubes cubes;
  cubes.Execute(box, &mesh);
  EXPECT_EQ(44, mesh.size());
}

TEST_F(MarchingCubesTest, BoxFloat) {
  TriangleMesh mesh;
  FloatPrintBox box(10, 10, 10, 1, 1);
  box.FillRegion(3, 5, 3, 5, 3, 5, 0.8);
  MarchingCubes cubes;
  cubes.Execute(box, &mesh);
  EXPECT_EQ(44, mesh.size());
}

TEST_F(MarchingCubesTest, BoxRotated) {
  TriangleMesh mesh;
  BinaryPrintBox box(100, 100, 100, 10, 10);
  int set_voxels[31][3] = {
    { 0,0,0 },
    { 1,0,0 },
    { 2,0,0 },
    { 0,1,0 },
    { 1,1,0 },
    { 2,1,0 },
    { 0,2,0 },
    { 1,2,0 },
    { 2,2,0 },
    { 0,0,1 },
    { 1,0,1 },
    { 2,0,1 },
    { 0,1,1 },
    { 1,1,1 },
    { 2,1,1 },
    { 0,2,1 },
    { 1,2,1 },
    { 2,2,1 },
    { 0,0,2 },
    { 1,0,2 },
    { 2,0,2 },
    { 0,1,2 },
    { 1,1,2 },
    { 2,1,2 },
    { 0,2,2 },
    { 1,2,2 },
    { 2,2,2 },
    { 1,0,3 },
    { 0,1,3 },
    { 1,1,3 },
    { 2,1,3 },
  };
  for (int i = 0; i < 31; ++i) {
    box.SetValue(set_voxels[i][0],
                 set_voxels[i][1],
                 set_voxels[i][2],
                 1);
  }

  MarchingCubes cubes;
  cubes.Execute(box, &mesh);
  EXPECT_EQ(124, mesh.size());
}

TEST_F(MarchingCubesTest, Threaded) {
  TriangleMesh mesh;
  BinaryPrintBox box(10, 10, 10, 1, 1);
  box.FillRegion(3, 5, 3, 5, 3, 5, 1);
  MarchingCubes::Input input;

  thread::ThreadPool pool(4);
  pool.StartWorkers();
  input.set_pool(&pool);
  MarchingCubes cubes(input);
  cubes.Execute(box, &mesh);
  EXPECT_EQ(44, mesh.size());
}

}  // anonymous namespace
}  // namespace printer
