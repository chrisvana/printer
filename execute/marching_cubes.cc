// Copyright 2015
// Author: Christopher Van Arsdale

#include <algorithm>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>
#include "common/log/log.h"
#include "common/base/callback.h"
#include "common/base/flags.h"
#include "common/thread/threadpool.h"
#include "common/thread/counter.h"
#include "printer/base/mesh.h"
#include "printer/execute/print_box.h"
#include "printer/execute/marching_cubes.h"
#include "printer/execute/marching_cubes_tables.h"

DEFINE_int32(marching_cubes_min_island_size, 0,
             "We trim any holes/islands smaller than this size from our "
             "output.");

using std::queue;
using std::vector;
using std::unordered_map;

namespace printer {
namespace {
int CubeIndex(const float grid[8], float iso) {
  int index = 0;
  if (grid[0] < iso) { index |= 1; }
  if (grid[1] < iso) { index |= 2; }
  if (grid[2] < iso) { index |= 4; }
  if (grid[3] < iso) { index |= 8; }
  if (grid[4] < iso) { index |= 16; }
  if (grid[5] < iso) { index |= 32; }
  if (grid[6] < iso) { index |= 64; }
  if (grid[7] < iso) { index |= 128; }
  return index;
}
Point Interpolate(float iso_level,
                  const Point& p1, const Point& p2,
                  float iso_1, float iso_2) {
  float distance = 0.5;
  if (iso_1 != iso_2) {
    distance = (iso_level - iso_1)/(iso_2 - iso_1);
  }
  distance = std::min(std::max(distance, 0.01f), 0.99f);
  return (p2 - p1) * distance + p1;
}

Point GetVertex(float iso_level,
                const Point& p1, const Point& p2,
                float iso_1, float iso_2) {
  // Linearly interpolate. We always try and do it in the same direction
  // (small to large by point) to avoid double issues.
  if (p2 < p1) {
    return Interpolate(iso_level, p1, p2, iso_1, iso_2);
  }
  return Interpolate(iso_level, p2, p1, iso_2, iso_1);
}

void FillVertices(float iso_cutoff,
                  int edges,
                  float iso[8],
                  Point points[8],
                  Point vertices[12]) {
  const int kPoint1[12] = { 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3 };
  const int kPoint2[12] = { 1, 2, 3, 0, 5, 6, 7, 4, 4, 5, 6, 7 };
  for (int i = 0; i < 12; ++i) {
    if (edges & (1 << i)) {
      int p1 = kPoint1[i], p2 = kPoint2[i];
      vertices[i] = GetVertex(iso_cutoff,
                              points[p1], points[p2],
                              iso[p1], iso[p2]);
    }
  }
}


bool MaybeAddNeigbhor(const PrintBox& input,
                      float iso_cutoff,
                      float our_iso,
                      int x1, int y1, int z1,
                      int x2, int y2, int z2,
                      PrintBox* shell) {
  float neighbor_iso = input.iso_value(x2, y2, z2);
  if ((neighbor_iso >= iso_cutoff) != (our_iso >= iso_cutoff)) {
    shell->SetValue(x2 + 1, y2 + 1, z2 + 1, true);
    return true;
  }
  return false;
}

BinaryPrintBox* FillShell(float iso_cutoff, const PrintBox& input) {
  BinaryPrintBox* shell = new BinaryPrintBox(
      input.size_x() + 2, input.size_y() + 2, input.size_z() + 2,
      input.horizontal_resolution(), input.vertical_resolution());

  PrintBox::Iterator iter = input.NewIterator();
  int num_set = 0;
  while (iter.NextSet()) {
    int x = iter.x(), y = iter.y(), z = iter.z();
    float our_iso = iter.iso_value();
    num_set += (our_iso > 0 ? 1 : 0);

    // Search neighbors, add any that are not the same iso value to our
    // output shell.
    const int kNeighbors[7][3] = {
      { 1, 0, 0 },
      { 0, 1, 0 },
      { 1, 1, 0 },
      { 0, 0, 1 },
      { 1, 0, 1 },
      { 0, 1, 1 },
      { 1, 1, 1 }
    };
    bool added = false;
    for (int i = 0; i < 7; ++i) {
      added |= MaybeAddNeigbhor(input, iso_cutoff, our_iso,
                                x, y, z,
                                x + kNeighbors[i][0],
                                y + kNeighbors[i][1],
                                z + kNeighbors[i][2],
                                shell);
      added |= MaybeAddNeigbhor(input, iso_cutoff, our_iso,
                                x, y, z,
                                x - kNeighbors[i][0],
                                y - kNeighbors[i][1],
                                z - kNeighbors[i][2],
                                shell);
    }
    if (added) {
      shell->SetValue(x + 1, y + 1, z + 1, true);
    }
  }

  LOG(INFO) << "Filled shell, total set voxels: " << num_set;
  return shell;
}


struct PointInt {
  PointInt(int xin, int yin, int zin) : x(xin), y(yin), z(zin) {}
  ~PointInt() {}

  bool operator<(const PointInt& other) const {
    return (x != other.x ? x < other.x :
            (y != other.y ? y < other.y : z < other.z));
  }
  bool operator==(const PointInt& other) const {
    return x == other.x && y == other.y && z == other.z;
  }

  int x, y, z;
};

struct HashPointInt {
  size_t operator()(const PointInt& point) const {
    size_t hash = std::hash<int>()(point.x);
    geometry_hash_stupid_combine(hash, std::hash<int>()(point.y));
    geometry_hash_stupid_combine(hash, std::hash<int>()(point.z));
    return hash;
  }
};
typedef unordered_map<PointInt, int, HashPointInt> PointIntIdMap;

void FindShellNeighbors(const BinaryPrintBox& shell,
                       PointInt start,
                       int id,
                       PointIntIdMap* id_map) {
  std::queue<PointInt> points;
  points.push(start);

  while (!points.empty()) {
    PointInt point = points.front();
    points.pop();

    const int kDirectNeighbors[6][3] = {
      { 1, 0, 0 },
      { 0, 1, 0 },
      { 0, 0, 1 },
      { -1, 0, 0 },
      { 0, -1, 0 },
      { 0, 0, -1 },
    };
    for (int i = 0; i < 6; ++i) {
      PointInt neighbor = PointInt(point.x + kDirectNeighbors[i][0],
                                   point.y + kDirectNeighbors[i][1],
                                   point.z + kDirectNeighbors[i][2]);
      if (shell.is_set(neighbor.x, neighbor.y, neighbor.z) &&
          id_map->insert(std::make_pair(neighbor, id)).second) {
        points.push(neighbor);
      }
    }
  }
}

}  // anonymous namespace


MarchingCubes_Input::MarchingCubes_Input()
    : pool_(NULL),
      min_island_size_(FLAGS_marching_cubes_min_island_size) {
}

MarchingCubes::MarchingCubes() {
}

MarchingCubes::MarchingCubes(const Input& input)
    : input_(input) {
}

MarchingCubes::~MarchingCubes() {
}

template <bool has_output, bool has_boundary, bool sharded>
void MarchingCubes::ExecuteInternalSingle(const PrintBox* box,
                                          const BinaryPrintBox* shell,
                                          float iso_cutoff,
                                          int total_shards,
                                          int shard,
                                          TriangleMesh* output,
                                          TriangleMesh* boundary) const {
  PrintBox::Iterator shell_iter = shell->NewIterator();
  float iso[8];
  Point points[8];
  Point vertices[12], inner[12], outer[12];
  int total = 0;
  while (shell_iter.NextSet()) {
    if (sharded && (++total) % total_shards != shard) {
      continue;
    }

    // The shell is offset by 1 up in each direction.
    int x = shell_iter.x() - 1;
    int y = shell_iter.y() - 1;
    int z = shell_iter.z() - 1;

    VLOG(5) << "Shard " << shard << " processing point: "
            << x << ", " << y << ", " << z;

    // Check on which cube we are.
    box->FillISOBox(x, y, z, iso);
    int index = CubeIndex(iso, iso_cutoff);
    int edges = MarchingCubesTables::kEdgeTable[index];
    if (edges == 0) {
      continue;
    }

    // Figure out xyz coordinates of triangles.
    box->FillPointBox(x, y, z, points);
    if (has_output) {
      FillVertices(iso_cutoff, edges, iso, points, vertices);
    }
    if (has_boundary) {
      FillVertices(iso_cutoff / 3, edges, iso, points, inner);
      FillVertices((iso_cutoff + 2) / 3, edges, iso, points, outer);
    }

    // Write triangles to output.
    const int* triangles = MarchingCubesTables::kTriangleTable[index];
    if (has_output) {
      for (int i = 0; triangles[i] != -1; i += 3) {
        CHECK(output->AddTriangle(Triangle(vertices[triangles[i]],
                                           vertices[triangles[i+1]],
                                           vertices[triangles[i+2]])));
      }
    }
    if (has_boundary) {
      for (int i = 0; triangles[i] != -1; i += 3) {
        CHECK(boundary->AddTriangle(Triangle(outer[triangles[i]],
                                                   outer[triangles[i+1]],
                                                   outer[triangles[i+2]])));
        CHECK(boundary->AddTriangle(Triangle(inner[triangles[i+2]],
                                                   inner[triangles[i+1]],
                                                   inner[triangles[i]])));
      }
    }
  }
}

template <bool has_output, bool has_boundary, bool sharded>
void MarchingCubes::ExecuteInternalShard(const PrintBox* box,
                                         const BinaryPrintBox* shell,
                                         float iso_cutoff,
                                         ShardInfo shard) const {
  VLOG(1) << "Starting shard: " << shard.shard;
  TriangleMesh* output = has_output ? new TriangleMesh : NULL;
  TriangleMesh* boundary = has_boundary ? new TriangleMesh : NULL;
  ExecuteInternalSingle<has_output, has_boundary, sharded>(
      box, shell, iso_cutoff,
      shard.total_shards, shard.shard, output, boundary);
  if (has_output) {
    shard.output_merger->Add(output);
  }
  if (has_boundary) {
    shard.boundary_merger->Add(boundary);
  }
  VLOG(1) << "Finishing shard: " << shard.shard;
  shard.done->Decrement();
}

template <bool has_output, bool has_boundary>
void MarchingCubes::ExecuteInternal(const PrintBox& box,
                                    const BinaryPrintBox& shell,
                                    float iso_cutoff,
                                    TriangleMesh* output,
                                    TriangleMesh* boundary) const {
  CHECK(!has_output || output);
  CHECK(!has_boundary || boundary);

  if (input_.pool() == NULL || input_.pool()->NumWorkers() < 2) {
    ExecuteInternalSingle<has_output, has_boundary, false>(
        &box, &shell, iso_cutoff, 1, 0, output, boundary);
    return;
  }

  TriangleMeshMerger output_merger(output);
  TriangleMeshMerger boundary_merger(boundary);

  // Ok, use the pool
  int num_workers = input_.pool()->NumWorkers();
  thread::BlockingCounter counter(num_workers);
  for (int i = 0; i < num_workers; ++i) {
    input_.pool()->Add(NewCallback(
        this,
        &MarchingCubes::ExecuteInternalShard<has_output, has_boundary, true>,
        &box, &shell, iso_cutoff,
        ShardInfo(num_workers, i, &output_merger, &boundary_merger,
                  &counter)));
  }
  counter.Wait();
  output_merger.Wait();
  boundary_merger.Wait();
}

void MarchingCubes::PruneIslands(BinaryPrintBox* shell,
                                 size_t min_island_size) const {
  PrintBox::Iterator iter = shell->NewIterator();
  int next_id = 0;
  PointIntIdMap id_map;
  vector<int> id_map_size;
  while (iter.NextSet()) {
    // There has to be a corner for each island.
    if (shell->is_set(iter.x() - 1, iter.y(), iter.z()) ||
        shell->is_set(iter.x(), iter.y() - 1, iter.z()) ||
        shell->is_set(iter.x(), iter.y(), iter.z() - 1)) {
      continue;
    }
    PointInt next = PointInt(iter.x(), iter.y(), iter.z());
    auto it = std::make_pair(next, next_id);
    if (id_map.insert(it).second) {
      int start_size = id_map.size() - 1;
      FindShellNeighbors(*shell, next, next_id++, &id_map);
      int end_size = id_map.size();
      id_map_size.push_back(end_size - start_size);
    }
  }

  // Remove any shell points not part of a large island.
  for (auto it : id_map) {
    const PointInt& point = it.first;
    int id = it.second;
    if (id_map_size[id] < min_island_size) {
      shell->SetValue(point.x, point.y, point.z, 0);
    }
  }
}

void MarchingCubes::ExecuteWithBoundary(const PrintBox& box,
                                        float iso_cutoff,
                                        TriangleMesh* output,
                                        TriangleMesh* boundary) const {
  // Step 1, figure out shell from input. This is the set of nodes that are
  // not fully empty or fully enclosed.
  LOG(INFO) << "Building shell.";
  std::unique_ptr<BinaryPrintBox> shell(FillShell(iso_cutoff, box));
  if (input_.min_island_size() > 0) {
    LOG(INFO) << "Pruning shell bubbles/islands.";
    PruneIslands(shell.get(), input_.min_island_size());
  }

  // Step 2, march.
  LOG(INFO) << "Marching.";
  bool has_output = (output != NULL);
  bool has_boundary = (boundary != NULL);
  switch ((has_output ? 2 : 0) + (has_boundary ? 1 : 0)) {
    case 0:
      ExecuteInternal<false, false>(box, *shell, iso_cutoff, output, boundary);
      break;
    case 1:
      ExecuteInternal<false, true>(box, *shell, iso_cutoff, output, boundary);
      break;
    case 2:
      ExecuteInternal<true, false>(box, *shell, iso_cutoff, output, boundary);
      break;
    case 3:
      ExecuteInternal<true, true>(box, *shell, iso_cutoff, output, boundary);
      break;
  }

  if (has_output) {
    CHECK(output->size() == 0 || output->CheckManifold());
  }
}

} // namespace printer
