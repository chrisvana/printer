// Copyright 2015
// Author: Christopher Van Arsdale

#include <mutex>
#include <cmath>
#include <memory>
#include <vector>
#include <queue>
#include "common/base/callback.h"
#include "common/base/flags.h"
#include "common/log/log.h"
#include "common/thread/counter.h"
#include "common/thread/threadpool.h"
#include "common/util/stl.h"
#include "printer/base/geometry.h"
#include "printer/base/mesh.h"
#include "printer/base/octree.h"
#include "printer/simplify/simplifier.h"

DEFINE_int32(simplifier_max_simplifications, -1,
             "If >= 0, max number of simplifications to run by default.");
DEFINE_bool(simplifier_check_internal_intersection, true,
            "If false, we disable internal intersection checks, which makes "
            "everything a lot faster, but may result in non-manifold "
            "meshes.");
DEFINE_bool(simplifier_check_boundary, true,
            "If false, we disable inner/outer boundary checking on what "
            "constitutes an acceptable simplification.");
DEFINE_bool(simplifier_enable_parallel_simplifier, true,
            "If false, parallel simplifier just delegates to the normal "
            "simplifier for all input.");

using std::unique_ptr;
using std::vector;

namespace printer {

Simplifier_Input::Simplifier_Input()
    : max_simplifications_(FLAGS_simplifier_max_simplifications),
      check_intersection_(FLAGS_simplifier_check_internal_intersection),
      check_boundary_(FLAGS_simplifier_check_boundary) {
}

Simplifier_Input::~Simplifier_Input() {}

Simplifier::Simplifier(const Input& input)
    : input_(input) {
}

Simplifier::~Simplifier() {
}

void Simplifier::Execute(const Octree* boundary, TriangleMesh* mesh) const {
  SimplifierState state(mesh->size(), 1, 1000);
  ExecuteWithState(boundary, mesh, &state);
}

void Simplifier::ExecuteWithState(const Octree* boundary,
                                  TriangleMesh* mesh,
                                  SimplifierState* state) const {
  if (input_.max_simplifications() == 0) {
    return;
  }

  SimplifierHeap heap;

  VLOG(1) << "Building initial edges.";
  BuildEdges(*mesh, &heap);

  state->Register(mesh, &heap);

  VLOG(1) << "Starting simplification.";
  int accepted_simplifications = 0;
  TriangleMesh::TriangleList old_triangles, new_triangles;
  for (int i = 0; (input_.max_simplifications() < 0 ||
                   accepted_simplifications < input_.max_simplifications());
       ++i) {
    state->MaybeLog();
    unique_ptr<SimplifierHeap::Simplification> simp(heap.PopBest());
    if (simp.get() == NULL) {
      break;
    }
    VLOG(3) << "Pop: " << simp->score;

    // 1) Reconstruct triangles by testing the edge again.
    old_triangles.clear();
    new_triangles.clear();
    if (!mesh->TestEdgeCollapse(
            simp->edge, simp->new_vertex,
            &old_triangles, &new_triangles)) {
      continue;
    }

    // 2) Test that this is a valid simplification.
    if (input_.check_boundary() && boundary != NULL &&
        !TestBoundary(new_triangles, *boundary)) {
      continue;
    }

    // 2) Apply the update.
    if (!mesh->ApplyUpdate(old_triangles, new_triangles,
                           input_.check_intersection())) {
      continue;
    }

    ++accepted_simplifications;
    heap.RemoveOverlapping(old_triangles);
    RebuildEdges(new_triangles, *mesh, &heap);
  }

  state->Deregister(mesh);
}

void Simplifier::RebuildEdges(const TriangleMesh::TriangleList& triangles,
                            const TriangleMesh& mesh,
                            SimplifierHeap* heap) const {
  std::unordered_set<Edge, HashEdge> seen;
  for (const Triangle& t : triangles) {
    if (seen.insert(t.e0().Deduped()).second) {
      EnqueueEdge(t.e0(), mesh, heap);
    }
    if (seen.insert(t.e1().Deduped()).second) {
      EnqueueEdge(t.e1(), mesh, heap);
    }
    if (seen.insert(t.e2().Deduped()).second) {
      EnqueueEdge(t.e2(), mesh, heap);
    }
  }
}

void Simplifier::BuildEdges(const TriangleMesh& mesh,
                            SimplifierHeap* heap) const {
  std::unordered_set<Edge, HashEdge> seen;
  for (TriangleMesh::EdgeIterator iter = mesh.edge_iterator();
       !iter.Done(); iter.Next()) {
    const Edge& e = iter.edge();
    if (seen.insert(e.Deduped()).second) {
      EnqueueEdge(e, mesh, heap);
    }
  }
}

void Simplifier::EnqueueEdge(const Edge& edge,
                             const TriangleMesh& mesh,
                             SimplifierHeap* heap) const {
  // TODO, minimize error?
  Point new_vertex = (edge.p0() + edge.p1()) / 2;

  // Try and collapse the edge.
  // TODO:thread_local
  TriangleMesh::TriangleList old_triangles, new_triangles;
  old_triangles.clear();
  new_triangles.clear();
  if (!mesh.TestEdgeCollapse(
          edge, new_vertex,
          &old_triangles, &new_triangles)) {
    VLOG(3) << "Failed edge collapse.";
    return;
  }

  // Check if we are leaving our box with any of the triangles (either the
  // ones we are removing, or the ones we are adding).
  if (input_.simplification_box().bottom() !=
      input_.simplification_box().top()) {
    for (const Triangle& t : old_triangles) {
      if (!input_.simplification_box().FullyContains(t)) {
        VLOG(3) << "Old triangle crosses simplification box boundary, "
                << "cannot simplify.";
        return;
      }
    }
    for (const Triangle& t : new_triangles) {
      if (!input_.simplification_box().FullyContains(t)) {
        VLOG(3) << "New triangle crosses simplification box boundary, "
                << "cannot add.";
        return;
      }
    }
  }

  // Set up the simplification
  // TODO: freelist?
  unique_ptr<SimplifierHeap::Simplification> simp(
      new SimplifierHeap::Simplification);
  simp->edge = edge;
  simp->new_vertex = new_vertex;
  simp->score = ScoreSimplification(
      edge, new_vertex, old_triangles, new_triangles);
  if (simp->score >= 0) {
    VLOG(4) << "Accepted simplification: " << simp->score;
    heap->AddSimplification(simp.release());
  }
}

namespace {
double Squash(double x, double cap) {
  return x > 1 ? x * cap / (cap + x) : x;
}
//double Unsquash(double x, double cap) {
//  return x > 1 ? x * cap / (cap - x) : x;
//}
double InverseSquash(double x, double cap) {
  return x == 0 ? cap : cap / Squash(x, cap);
}
double TriangleCountScore(const TriangleMesh::TriangleList& old_t,
                          const TriangleMesh::TriangleList& new_t) {
  // We always remove 2, and we would like to reduce the vertex count into
  // an individual triangle, so we prefer simplifications that result in fewer
  // triangles in "new_t".
  return InverseSquash(new_t.size(), 5);
}
double AverageRatio(const TriangleMesh::TriangleList& triangles) {
  double average_ratio = 0;
  for (const Triangle& t : triangles) {
    double e0_size = t.e0().length();
    double e1_size = t.e1().length();
    double e2_size = t.e2().length();
    double max_length = std::max(std::max(e0_size, e1_size), e2_size);
    double min_length = std::min(std::min(e0_size, e1_size), e2_size);
    average_ratio += max_length / min_length;
  }
  return average_ratio / triangles.size();
}
double RatioScore(const TriangleMesh::TriangleList& old_t,
                  const TriangleMesh::TriangleList& new_t) {
  double old_r = AverageRatio(old_t);
  double new_r = AverageRatio(new_t);
  return old_r / new_r;
}
double MaxSize(const TriangleMesh::TriangleList& triangles) {
  double max_size = 0;
  for (const Triangle& t : triangles) {
    Point normal = t.normal();
    max_size = std::max(max_size, sqrt(normal*normal));
  }
  return max_size;
}
double MaxSizeScore(const TriangleMesh::TriangleList& old_t,
                    const TriangleMesh::TriangleList& new_t) {
  return MaxSize(old_t) / MaxSize(new_t);
}
double NormalScore(const Edge& edge,
                   const TriangleMesh::TriangleList& old_t,
                   const TriangleMesh::TriangleList& new_t) {
  // Compute the average normal at edge.p0 and edge.p1
  double p0_total = 0, p1_total = 0;
  Point p0_normal, p1_normal;
  for (const Triangle& t : old_t) {
    Point normal = t.normal().Normalized();
    if (t.p0() == edge.p0() || t.p1() == edge.p0() || t.p2() == edge.p0()) {
      ++p0_total;
      p0_normal = p0_normal + normal;
    }
    if (t.p0() == edge.p1() || t.p1() == edge.p1() || t.p2() == edge.p1()) {
      ++p1_total;
      p1_normal = p1_normal + normal;
    }
  }

  // Return the dot product of the 2 normals as the score.
  return (p0_normal / p0_total).Normalized() *
      (p1_normal / p1_total).Normalized();
}
}  // anonymous namespace

// static
double Simplifier::ScoreSimplification(
    const Edge& edge,
    const Point& new_vertex,
    const TriangleMesh::TriangleList& old_t,
    const TriangleMesh::TriangleList& new_t) {
  // Score
  double count_score = TriangleCountScore(old_t, new_t);
  double ratio_score = RatioScore(old_t, new_t);
  double size_score = MaxSizeScore(old_t, new_t);
  double normal_score = NormalScore(edge, old_t, new_t);
  VLOG(4) << "Old size: " << old_t.size();
  VLOG(4) << "New size: " << new_t.size();
  VLOG(4) << "Count score: " << count_score;
  VLOG(4) << "Ratio score: " << ratio_score;
  VLOG(4) << "Size score: " << size_score;
  VLOG(4) << "Normal score: " << size_score;
  double score = (Squash(count_score, 5) +
                  Squash(3 * ratio_score, 5) +
                  Squash(3 * size_score, 5) +
                  Squash(5 * normal_score, 5));
  VLOG(4) << "Final score: " << score;
  return score;
}

// static
bool Simplifier::TestBoundary(const TriangleMesh::TriangleList& triangles,
                              const Octree& boundary) {
  Octree octree(boundary.range());
  for (const Triangle& t : triangles) {
    if (!octree.AddTriangle(t)) {
      return false;
    }
  }
  return !octree.Intersects(boundary);
}

ParallelSimplifier_Input::ParallelSimplifier_Input()
    : pool_(NULL),
      min_to_split_(2000),
      enable_parallel_(FLAGS_simplifier_enable_parallel_simplifier) {
}

ParallelSimplifier::ParallelSimplifier(const Input& input)
    : input_(input) {
}

ParallelSimplifier::~ParallelSimplifier() {
}

namespace {
Box BoxForChild(const Box& input, int index) {
  Point center = input.center();
  Range x = (index & 1 ?
             Range(center.x(), input.top().x()) :
             Range(input.bottom().x(), center.x()));
  Range y = (index & 2 ?
             Range(center.y(), input.top().y()) :
             Range(input.bottom().y(), center.y()));
  Range z = (index & 4 ?
             Range(center.z(), input.top().z()) :
             Range(input.bottom().z(), center.z()));
  return Box(Point(x.start(), y.start(), z.start()),
             Point(x.end(), y.end(), z.end()));
}
}  // anonymous namespace

void ParallelSimplifier::Execute(const Octree* boundary,
                                 TriangleMesh* mesh) const {
  int starting_size = mesh->size();

  if (input_.enable_parallel() &&
      input_.pool() != NULL &&
      input_.pool()->NumWorkers() > 1 &&
      mesh->size() > input_.min_to_split()) {
    Box simplification_box;
    if (boundary != NULL) {
      simplification_box = boundary->range();
    } else if (mesh->octree() != NULL) {
      simplification_box = mesh->octree()->range();
    } else {
      simplification_box = mesh->MinimalBoundingBox();
    }

    // Build up shards.
    vector<Box> ranges;
    vector<TriangleMesh*> shards;  // will get passed to merger to delete.

    size_t input_total = mesh->size();
    LOG(INFO) << "Sharding input.";
    ComputeShards(simplification_box, mesh, &ranges, &shards);
    size_t total = 0;
    for (const TriangleMesh* m : shards) {
      total += m->size();
    }

    if (VLOG_IS_ON(3)) {
      VLOG(3) << "Total input (" << input_total << ") split to " << total
              << " amongst " << shards.size() << " shards.";
    }

    mesh->Clear();
    TriangleMeshMerger merger(mesh);

    Simplifier::SimplifierState state(total, shards.size(), 10000);

    // Execute.
    LOG(INFO) << "Running " << shards.size() << " simplifiers.";
    thread::BlockingCounter counter(shards.size());
    for (int i = 0; i < shards.size(); ++i) {
      input_.pool()->Add(NewCallback(
          this, &ParallelSimplifier::ExecuteSingleBox,
          WorkUnit(
              ranges[i], boundary, shards[i], &merger, &state,
              NewCallback(&counter, &thread::BlockingCounter::Decrement))));
    }
    counter.Wait();
    merger.Wait();

    // We erased this.
    mesh->InitializeOctreeUnsafe(simplification_box);
  }

  // Now we perform one final pass (or the first pass, if we haven't done any
  // simplification yet).
  Simplifier::SimplifierState state(mesh->size(), 1, 1000);
  Simplifier::Input input = input_;
  if (input.max_simplifications() > 0) {
    // We used up some of our simplification budget. Compare the current size
    // of the mesh to the original size.
    int mesh_size_delta = starting_size - mesh->size();
    input.set_max_simplifications(
        input.max_simplifications() - mesh_size_delta);
  }
  Simplifier simplifier(input);
  simplifier.ExecuteWithState(boundary, mesh, &state);
}

namespace {
struct QueueElem {
  QueueElem() : mesh(NULL) {}
  QueueElem(TriangleMesh* m, Box b) : mesh(m), box(b) {}

  TriangleMesh* mesh;
  Box box;

  bool operator<(const QueueElem& other) const {
    return mesh->size() < other.mesh->size();
  }
};
}  // anonymous namespace

void ParallelSimplifier::ComputeShards(const Box& box,
                                       TriangleMesh* input,
                                       vector<Box>* ranges,
                                       vector<TriangleMesh*>* outputs) const {
  int num_workers = input_.pool()->NumWorkers();
  int max_shards = 1.5 * num_workers;

  std::priority_queue<QueueElem> next;
  next.push(QueueElem(input, box));

  while (!next.empty()) {
    QueueElem top = next.top();
    next.pop();

    // When we fill up our allocated shard count.
    if (next.size() + outputs->size() >= max_shards - 7) {
      ranges->push_back(top.box);
      outputs->push_back(top.mesh);
      continue;
    }

    // Compute sub-triangles.
    vector<TriangleMesh*> temp_shards(8);
    vector<Box> temp_ranges(temp_shards.size());
    for (int i = 0; i < temp_shards.size(); ++i) {
      temp_shards[i] = new TriangleMesh();
      temp_ranges[i] = BoxForChild(top.box, i);
    }

    // If we jump our triangle count by over 1.5, give up and use this top level
    // shard, and don't use any of the sub-elements.
    Split(top.mesh, temp_ranges, &temp_shards);
    size_t total = 0;
    for (const TriangleMesh* m : temp_shards) {
      total += m->size();
    }
    if (top.mesh != input && total > 1.5 * top.mesh->size()) {
      ranges->push_back(top.box);
      outputs->push_back(top.mesh);
      DeleteElements(&temp_shards);
      continue;
    }

    // Ok, we are going to keep the sub-shards.
    top.mesh->Clear();
    for (int i = 0; i < temp_shards.size(); ++i) {
      TriangleMesh* shard = temp_shards[i];
      const Box& range = temp_ranges[i];
      if (shard->size() == 0) {
        delete shard;
        continue;
      }

      if (shard->size() < input_.min_to_split()) {
        ranges->push_back(range);
        outputs->push_back(shard);
      } else {
        next.push(QueueElem(shard, range));
      }
    }
  }

  for (TriangleMesh* m : *outputs) {
    m->InitializeOctreeUnsafe(box);
  }
}


void ParallelSimplifier::Split(TriangleMesh* input,
                               const vector<Box>& ranges,
                               vector<TriangleMesh*>* outputs) const {
  thread::BlockingCounter counter(ranges.size());
  for (int i = 0; i < ranges.size(); ++i) {
    input_.pool()->Add(NewCallback(
        this, &ParallelSimplifier::SingleSplit,
        ranges[i], input, (*outputs)[i], &counter));
  }
  counter.Wait();
}

void ParallelSimplifier::SingleSplit(Box range,
                                     TriangleMesh* input,
                                     TriangleMesh* output,
                                     thread::BlockingCounter* done) const {
  for (TriangleMesh::TriangleIterator iter = input->iterator();
       !iter.Done(); iter.Next()) {
    if (range.Intersects(iter.triangle())) {
      output->AddTriangle(iter.triangle());
    }
  }
  done->Decrement();
}

void ParallelSimplifier::ExecuteSingleBox(WorkUnit work) const {
  // This is the base case, non-parallel.
  Simplifier::Input input = input_;
  input.set_simplification_box(work.box);
  if (input_.max_simplifications() > 0) {
    input.set_max_simplifications(
        input_.max_simplifications() / work.state->num_simplifiers());
  }
  Simplifier simplifier(input);
  simplifier.ExecuteWithState(work.boundary, work.input, work.state);
  work.merger->Add(work.input);  // owns it now.
  work.done->Run();
}

Simplifier::SimplifierState::SimplifierState(size_t starting_triangles,
                                             int num_simplifiers,
                                             int n)
    : n_(n),
      last_(0),
      num_simplifiers_(num_simplifiers),
      num_pending_(num_simplifiers),
      num_done_(0),
      static_triangles_(starting_triangles) {
}

void Simplifier::SimplifierState::Register(TriangleMesh* mesh,
                                           SimplifierHeap* heap) {
  MutexLock l(lock_);
  static_triangles_ -= mesh->size();
  num_pending_--;
  simplifiers_[mesh] = heap;
}

void Simplifier::SimplifierState::Deregister(TriangleMesh* mesh) {
  MutexLock l(lock_);
  simplifiers_.erase(mesh);
  num_done_++;
  static_triangles_ += mesh->size();
}

// static
void Simplifier::SimplifierState::ActuallyLog(size_t triangles,
                                              size_t pending_simp,
                                              size_t num_active,
                                              size_t num_pending,
                                              size_t num_seen) {
  LOG(INFO) << "Simplification step: "
            << "triangles=" << triangles
            << ", changes=" << pending_simp
            << ".  Simplififiers=(active=" << num_active
            << ", pending=" << num_pending
            << ", done=" << num_seen << ")";
}

}  // namespace printer
