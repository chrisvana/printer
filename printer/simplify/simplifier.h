// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_SIMPLIFY_SIMPLIFIER_H__
#define _PRINTER_SIMPLIFY_SIMPLIFIER_H__

#include <map>
#include <mutex>
#include <vector>
#include "common/base/macros.h"
#include "printer/base/mesh.h"
#include "printer/simplify/simplifier_heap.h"

namespace thread {
class ThreadPool;
class BlockingCounter;
}
class Closure;

namespace printer {

class Edge;
class Octree;
class TriangleMesh;

class Simplifier_Input {
 public:
  Simplifier_Input();
  ~Simplifier_Input();

  // Accessors.
  int max_simplifications() const { return max_simplifications_; }
  bool check_intersection() const { return check_intersection_; }
  bool check_boundary() const { return check_boundary_; }
  Box simplification_box() const { return simplification_box_; }

  // Mutators
  void set_max_simplifications(int s) { max_simplifications_ = s; }
  void set_check_intersection(bool c) { check_intersection_ = c; }
  void set_check_boundary(bool c) { check_boundary_ = c; }
  void set_simplification_box(const Box& b) { simplification_box_ = b; }

 private:
  int max_simplifications_;
  bool check_intersection_, check_boundary_;
  Box simplification_box_;
};

class Simplifier {
 public:
  typedef Simplifier_Input Input;
  class SimplifierState {
   public:
    SimplifierState(size_t starting_triangles,
                    int num_simplifiers,
                    int n);
    ~SimplifierState() {}

    void MaybeLog();
    void Register(TriangleMesh* mesh, SimplifierHeap* heap);
    void Deregister(TriangleMesh* mesh);

    size_t num_simplifiers() const {
      return num_simplifiers_;
    }

   private:
    void TryLog();
    static void ActuallyLog(size_t triangles,
                            size_t pending_simp,
                            size_t num_active,
                            size_t num_pending,
                            size_t num_done);

    typedef std::mutex Mutex;
    typedef std::unique_lock<Mutex> MutexLock;
    Mutex lock_;

    // Logging info.
    int n_, last_;

    // Starting info.
    size_t num_simplifiers_, num_pending_, num_done_;
    size_t static_triangles_;

    // Pending simplifiers.
    std::map<TriangleMesh*, SimplifierHeap*> simplifiers_;
  };

  explicit Simplifier(const Input& input);
  ~Simplifier();

  bool RequiresBoundary() const { return input_.check_boundary(); }
  void Execute(const Octree* boundary, TriangleMesh* mesh) const;
  void Execute(const TriangleMesh& boundary, TriangleMesh* mesh) const {
    Execute(boundary.octree(), mesh);
  }

  // Mostly for internal use.
  void ExecuteWithState(const Octree* boundary,
                        TriangleMesh* mesh,
                        SimplifierState* state) const;

 private:
  DISALLOW_COPY_AND_ASSIGN(Simplifier);

  void EnqueueEdge(const Edge& edge,
                   const TriangleMesh& mesh,
                   SimplifierHeap* heap) const;
  void RebuildEdges(const TriangleMesh::TriangleList& triangles,
                    const TriangleMesh& mesh,
                    SimplifierHeap* heap) const;
  void BuildEdges(const TriangleMesh& full_mesh,
                  SimplifierHeap* heap) const;

  static double ScoreSimplification(const Edge& edge,
                                    const Point& new_vertex,
                                    const TriangleMesh::TriangleList& old_t,
                                    const TriangleMesh::TriangleList& new_t);
  static bool TestBoundary(const TriangleMesh::TriangleList& triangles,
                           const Octree& octree);

  Input input_;
};

class ParallelSimplifier_Input : public Simplifier_Input {
 public:
  ParallelSimplifier_Input();
  ~ParallelSimplifier_Input() {}

  thread::ThreadPool* pool() const { return pool_; }
  void set_pool(thread::ThreadPool* pool) { pool_ = pool; }

  int min_to_split() const { return min_to_split_; }
  void set_min_to_split(int splits) { min_to_split_ = splits; }

  bool enable_parallel() const { return enable_parallel_; }
  void set_enable_parallel(bool enable) { enable_parallel_ = enable; }

 private:
  thread::ThreadPool* pool_;
  int min_to_split_;
  bool enable_parallel_;
};

class ParallelSimplifier {
 public:
  typedef ParallelSimplifier_Input Input;
  explicit ParallelSimplifier(const Input& input);
  ~ParallelSimplifier();

  bool RequiresBoundary() const { return input_.check_boundary(); }
  void Execute(const Octree* boundary, TriangleMesh* mesh) const;
  void Execute(const TriangleMesh& boundary, TriangleMesh* mesh) const {
    Execute(boundary.octree(), mesh);
  }

 private:
  typedef std::mutex Mutex;
  typedef std::unique_lock<Mutex> MutexLock;

  void ComputeShards(const Box& box,
                     TriangleMesh* input,
                     std::vector<Box>* ranges,
                     std::vector<TriangleMesh*>* outputs) const;
  void Split(TriangleMesh* input,
             const std::vector<Box>& ranges,
             std::vector<TriangleMesh*>* outputs) const;
  void SingleSplit(Box range,
                   TriangleMesh* input,
                   TriangleMesh* output,
                   thread::BlockingCounter* done) const;

  struct WorkUnit {
    WorkUnit(const Box& b, const Octree* bound,
             TriangleMesh* i, TriangleMeshMerger* m,
             Simplifier::SimplifierState* s, Closure* d)
        : box(b), boundary(bound), input(i), merger(m), state(s), done(d) {
    }

    Box box;
    const Octree* boundary;
    TriangleMesh* input;
    TriangleMeshMerger* merger;
    Simplifier::SimplifierState* state;
    Closure* done;
  };
  void ExecuteSingleBox(WorkUnit work) const;

  Input input_;
};

inline void Simplifier::SimplifierState::TryLog() {
  MutexLock l(lock_, std::try_to_lock);
  if (!l.owns_lock() || last_ < n_) {
    return;
  }

  last_ = 0;
  size_t total_triangles = static_triangles_;
  size_t total_simplifications = 0;
  size_t num = simplifiers_.size();
  size_t num_pending = num_pending_;
  size_t num_done = num_done_;
  for (auto it : simplifiers_) {
    // These are scary, called outside of a mutex (could be updated).
    total_triangles += it.first->size();
    total_simplifications += it.second->size();
  }
  l.unlock();
  ActuallyLog(total_triangles,
              total_simplifications,
              num,
              num_pending,
              num_done);
}

inline void Simplifier::SimplifierState::MaybeLog() {
  if (++last_ > n_) {
    TryLog();
  }
}

}  // namespace printer

#endif  // _PRINTER_SIMPLIFY_SIMPLIFIER_H__
