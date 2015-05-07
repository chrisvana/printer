// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_EXECUTE_MARCHING_CUBES_H__
#define _PRINTER_EXECUTE_MARCHING_CUBES_H__

namespace thread {
class ThreadPool;
class BlockingCounter;
}

namespace printer {

class BinaryPrintBox;
class PrintBox;
class TriangleMesh;

class MarchingCubes_Input {
 public:
  MarchingCubes_Input();
  ~MarchingCubes_Input() {}

  thread::ThreadPool* pool() const { return pool_; }
  size_t min_island_size() const { return min_island_size_; }

  void set_pool(thread::ThreadPool* pool) { pool_ = pool; }
  void set_min_island_size(size_t s) { min_island_size_ = s; }

 private:
  thread::ThreadPool* pool_;
  size_t min_island_size_;
};

class MarchingCubes {
 public:
  typedef MarchingCubes_Input Input;
  MarchingCubes();
  explicit MarchingCubes(const Input& input);
  ~MarchingCubes();

  void Execute(const PrintBox& box,
               TriangleMesh* output) const {
    ExecuteWithISO(box, 0.5, output);
  }
  void ExecuteWithISO(const PrintBox& box,
                      float iso_cutoff,
                      TriangleMesh* output) const {
    return ExecuteWithBoundary(box, iso_cutoff, output, NULL);
  }
  void ExecuteWithBoundary(const PrintBox& box,
                           float iso_cutoff,
                           TriangleMesh* output,
                           TriangleMesh* boundary) const;

 private:
  struct ShardInfo {
    ShardInfo(int t, int s, TriangleMeshMerger* o, TriangleMeshMerger* b,
              thread::BlockingCounter* c)
        : total_shards(t), shard(s), output_merger(o),
          boundary_merger(b), done(c) {
    }
    int total_shards;
    int shard;
    TriangleMeshMerger* output_merger;
    TriangleMeshMerger* boundary_merger;
    thread::BlockingCounter* done;
  };

  template <bool has_output, bool has_boundary, bool sharded>
  void ExecuteInternalSingle(const PrintBox* box,
                             const BinaryPrintBox* shell,
                             float iso_cutoff,
                             int total_shards,
                             int shard,
                             TriangleMesh* output,
                             TriangleMesh* boundary) const;
  template <bool has_output, bool has_boundary, bool sharded>
  void ExecuteInternalShard(const PrintBox* box,
                            const BinaryPrintBox* shell,
                            float iso_cutoff,
                            ShardInfo info) const;
  template <bool has_output, bool has_boundary>
  void ExecuteInternal(const PrintBox& box,
                       const BinaryPrintBox& shell,
                       float iso_cutoff,
                       TriangleMesh* output,
                       TriangleMesh* boundary) const;

  void PruneIslands(BinaryPrintBox* shell,
                    size_t min_island_size) const;

  Input input_;  
};

}  // namespace printer

#endif  // _PRINTER_EXECUTE_MARCHING_CUBES_H__
