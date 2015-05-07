// Copyright 2015
// Author: Christopher Van Arsdale

#include <cmath>
#include <map>
#include <mutex>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "common/base/callback.h"
#include "common/base/flags.h"
#include "common/log/log.h"
#include "common/thread/threadpool.h"
#include "common/thread/counter.h"
#include "printer/base/geometry.h"
#include "printer/base/mesh.h"
#include "printer/base/octree.h"
#include "printer/simplify/face_reduction.h"
#include "printer/simplify/triangulation.h"

DEFINE_double(face_reduction_min_face_ratio, 3,
              "If our triangle to face ratio is below this number, we do not "
              "bother with running the face reduction.");

using std::map;
using std::set;
using std::unordered_map;
using std::unordered_set;
using std::vector;

namespace printer {
namespace {
typedef std::mutex Mutex;
typedef std::unique_lock<Mutex> MutexLock;
typedef unordered_map<int, unordered_set<int> > FaceMap;
typedef unordered_map<int, int> CanonicalIdMap;
typedef unordered_set<Point, HashPoint> CriticalPointSet;
typedef unordered_map<Point, set<Point>, HashPoint> CriticalEdgeMap;
typedef unordered_set<int> BrokenFaceSet;

class TriangleIdMap {
 public:
  TriangleIdMap() : next_id_(1) {}

  int GetId(const Triangle& t) const {
    auto it = triangle_id_.find(t);
    CHECK(it != triangle_id_.end());
    return it->second;
  }

  int GetOrInsert(const Triangle& t) {
    int* id = &triangle_id_[t];
    if (*id == 0) {
      *id = next_id_++;
      id_to_triangle_[*id] = t;
      edges_[t.e0()] = *id;
      edges_[t.e1()] = *id;
      edges_[t.e2()] = *id;
    }
    return *id;
  }

  Triangle GetTriangle(int id) const {
    auto it = id_to_triangle_.find(id);
    CHECK(it != id_to_triangle_.end());
    return it->second;
  }

  void GetNeighbors(const Triangle& t, vector<int>* neighbors) const {
    GetTriangle(t.e0().flipped(), neighbors);
    GetTriangle(t.e1().flipped(), neighbors);
    GetTriangle(t.e2().flipped(), neighbors);
  }

  void GetTriangle(const Edge& e, vector<int>* t) const {
    auto it = edges_.find(e);
    if (it != edges_.end()) {
      t->push_back(it->second);
    }
  }

  bool ContainsTriangle(const Triangle& t) const {
    return triangle_id_.find(t) != triangle_id_.end();
  }

 private:
  int next_id_;
  unordered_map<Triangle, int, HashTriangle> triangle_id_;
  unordered_map<int, Triangle> id_to_triangle_;
  unordered_map<Edge, int, HashEdge> edges_;
};

// ProcessTriangle
//   Given a triangle, find all neighbors with the same normal and add them to
//   to the canonical map. This uses a queue to expand the triangles outward
//   from our initial input.
void ProcessTriangle(const Triangle& triangle,
                     const TriangleIdMap& id_map,
                     CanonicalIdMap* canonical) {
  int id = id_map.GetId(triangle);
  if (canonical->find(id) != canonical->end()) {
    return;
  }
  (*canonical)[id] = id;

  Point normal = triangle.normal().Normalized();

  // Get neighbors with same normal.
  vector<int> to_process;
  unordered_set<int> seen;
  to_process.push_back(id);
  seen.insert(id);
  while (!to_process.empty()) {
    // Get the last id.
    int next_id = to_process.back();
    to_process.resize(to_process.size() - 1);

    // Figure out which triangle we are looking at.
    Triangle next = id_map.GetTriangle(next_id);
    vector<int> neighbors;
    id_map.GetNeighbors(next, &neighbors);
    for (int neighbor_id : neighbors) {
      if (!seen.insert(neighbor_id).second) {
        // Already processed.
        continue;
      }
      Triangle neighbor = id_map.GetTriangle(neighbor_id);
      Point neighbor_normal = neighbor.normal().Normalized();
      if (neighbor_normal == normal ||
          (neighbor_normal * normal) >
          (1 - Triangle::kDefaultIntersectEpsilon)) {
        (*canonical)[neighbor_id] = id;
        to_process.push_back(neighbor_id);
      }
    }
  }
}

void ComputeIdMap(const TriangleMesh& mesh,
                  TriangleIdMap* id_map) {
  for (TriangleMesh::TriangleIterator iter(mesh.iterator());
       !iter.Done(); iter.Next()) {
    const Triangle& triangle = iter.triangle();
    id_map->GetOrInsert(triangle.Normalized());
  }
}

void ComputeCanonicals(const TriangleMesh& mesh,
                       const TriangleIdMap& id_map,
                       CanonicalIdMap* canonicals,
                       FaceMap* faces) {
  for (TriangleMesh::TriangleIterator iter(mesh.iterator());
       !iter.Done(); iter.Next()) {
    const Triangle& triangle = iter.triangle();
    ProcessTriangle(triangle.Normalized(), id_map, canonicals);
  }

  for (auto it : *canonicals) {
    (*faces)[it.second].insert(it.first);
  }
}

bool ComputeCriticalPoints(
    int canonical_id,
    const CanonicalIdMap& canonicals,
    const BrokenFaceSet& broken_faces,
    const unordered_set<int>& triangles,
    const TriangleIdMap& id_map,
    const TriangleMesh& mesh,
    CriticalPointSet* critical_points,
    CriticalEdgeMap* critical_edges) {
  if (triangles.size() <= 2) {
    // We cannot do any better.
    return false;
  }

  for (int id : triangles) {
    Triangle triangle = id_map.GetTriangle(id);
    Point normal = triangle.normal().Normalized();

    // For each point, compute if it is a critical point in the face.
    for (int i = 0; i < 3; ++i) {
      const Point& p = (i == 0 ? triangle.p0() :
                        (i == 1 ? triangle.p1() : triangle.p2()));
      const Point& dest = (i == 0 ? triangle.p1() :
                           (i == 1 ? triangle.p2() : triangle.p0()));
      TriangleMesh::TriangleList point_neighbors;
      mesh.GetAdjascentToPoint(p, &point_neighbors);

      // Critical points are ones with 3 or more bordering normals.
      set<int> normals;
      bool found_critical_point = false, found_critical_edge = false;
      for (const Triangle& neighbor : point_neighbors) {
        int id = id_map.GetId(neighbor);
        int canonical = canonicals.find(id)->second;
        if (canonical != canonical_id) {
          // Check for critical point (normals.size() >= 3 or the face is
          // one of our broken faces).
          if ((normals.size() < 2 &&
               normals.insert(canonical).second &&
               normals.size() == 2 &&
               critical_points->insert(p).second) ||
              broken_faces.find(canonical) != broken_faces.end()) {
            found_critical_point = true;
          }

          // Check for critical edge.
          if ((neighbor.p0() == dest ||
               neighbor.p1() == dest ||
               neighbor.p2() == dest) &&
              (*critical_edges)[p].insert(dest).second) {
            found_critical_edge = true;
          }

          if (found_critical_edge && found_critical_point) {
            break;
          }
        }
      }
    }
  }
  CHECK_GE(critical_points->size(), 3);
  CHECK_GE(critical_edges->size(), critical_points->size());
  VLOG(3) << "Computed: " << critical_points->size() << " points.";
  VLOG(3) << "Computed: " << critical_edges->size() << " edge origins.";
  return true;
}

Point GetBranchPoint(const Point& prev,
                     const Point& current,
                     const Point& normal,
                     set<Point>& options) {
  // Ok, we have a branching point. Pick the next point "to the left",
  // meaning prev -> current -> next forms the smallest angle possible
  // to the left as looking down on the mesh.
  Point next = *options.begin();
  double min_angle = 2;
  if (prev != current) {
    Point last_dir = (current - prev);
    for (const Point& p : options) {
      Point next_dir = (next - current);
      double dot = next_dir * last_dir;
      double cross = next_dir.Cross(last_dir) * normal;
      double angle = cross > 0 ? (1 - dot) : (dot - 1);
      if (angle < min_angle) {
        next = p;
      }
    }
  }
  return next;
}

void ComputeSingleLoop(const Point& normal,
                       Point start,
                       const BrokenFaceSet& broken_faces,
                       CriticalPointSet* critical_points,
                       CriticalEdgeMap* critical_edges,
                       vector<Point>* loop) {
  Point current = start, prev = start;
  while (true) {
    // Get the next point in the list.
    auto it = critical_edges->find(current);
    if (it == critical_edges->end()) {
      // We've looped around, and have nothing left.
      if (current == start) {
        break;
      }

      // Some programming error.
      LOG(FATAL) << "Programming error, could not find point: "
                 << current.DebugString();
    }

    // Figure out the next point.
    bool done_with_point = false;
    Point next = GetBranchPoint(prev, current, normal, it->second);
    CHECK_EQ(1, it->second.erase(next));
    if (it->second.empty()) {
      critical_edges->erase(it);
      done_with_point = true;
    }

    // Add our current point to the loop, if it is a critical point.
    auto it2 = critical_points->find(current);
    if (it2 != critical_points->end()) {
      loop->push_back(current);
      if (done_with_point) {
        critical_points->erase(it2);
      }
    }

    // Move forward, stop if we loop around to the beginning.
    if (next == start) {
      break;
    }
    prev = current;
    current = next;
  }
  CHECK_GE(loop->size(), 3);
}

void ComputeLoops(const Point& normal,
                  const BrokenFaceSet& broken_faces,
                  CriticalPointSet* critical_points,
                  CriticalEdgeMap* critical_edges,
                  vector<vector<Point> >* loops) {
  // Find the minimum point, which has to be outside of a hole.
  Point minimum = *critical_points->begin();
  for (const Point& p : *critical_points) {
    if (p < minimum) {
      minimum = p;
    }
  }

  // Find the loop, this is the outside.
  CHECK_GE(critical_points->size(), 3);
  loops->resize(1);
  ComputeSingleLoop(normal, minimum, broken_faces, critical_points,
                    critical_edges, &loops->back());

  // Find the holes.
  while (!critical_points->empty()) {
    CHECK_GE(critical_points->size(), 3);
    loops->resize(loops->size() + 1);
    ComputeSingleLoop(normal, *critical_points->begin(), broken_faces,
                      critical_points, critical_edges, &loops->back());
  }
}

bool ComputeTriangles(const Triangle& base,
                      const vector<vector<Point> >& loops,
                      vector<Triangle>* output) {
  Triangulation triangulate(base.p1() - base.p0(),
                            base.p2() - base.p0());
  triangulate.SetOuterLoop(&loops[0]);
  for (int i = 1; i < loops.size(); ++i) {
    triangulate.AddHole(&loops[i]);
  }
  return triangulate.Triangulate(output);
}

bool ComputeSingleFace(int canonical_id,
                       const BrokenFaceSet& broken_faces,
                       const CanonicalIdMap& canonicals,
                       const unordered_set<int>& triangles,
                       const TriangleIdMap& id_map,
                       const TriangleMesh& mesh,
                       Mutex* lock,
                       TriangleMesh* out_mesh) {
  CriticalPointSet critical_points;
  CriticalEdgeMap critical_edges;

  // Compute critical points.
  VLOG(3) << "Computing critical points.";
  if (!ComputeCriticalPoints(canonical_id, canonicals, broken_faces,
                             triangles, id_map, mesh,
                             &critical_points, &critical_edges)) {
    VLOG(3) << "Insufficient triangles to re-triangulate, keeping original.";
    MutexLock l(*lock);
    for (int id : triangles) {
      out_mesh->AddTriangle(id_map.GetTriangle(id));
    }
    return true;
  }

  // Compute loops.
  VLOG(3) << "Computing loops.";
  vector<vector<Point> > loops;
  Triangle base = id_map.GetTriangle(*triangles.begin());
  ComputeLoops(base.normal().Normalized(),
               broken_faces,
               &critical_points,
               &critical_edges,
               &loops);

  // Triangulate loops.
  VLOG(3) << "Running triangulation (on " << loops.size() << " loops).";
  vector<Triangle> output;
  if (!ComputeTriangles(base, loops, &output)) {
    VLOG(3) << "Could not triangulate, keeping original (which will break "
            << "tight mesh.";
    MutexLock l(*lock);
    for (int id : triangles) {
      out_mesh->AddTriangle(id_map.GetTriangle(id));
    }
    return false;
  }

  VLOG(2) << "Converted face from " << triangles.size()
          << " to " << output.size();
  MutexLock l(*lock);
  for (const Triangle& t : output) {
    out_mesh->AddTriangle(t);
  }
  return true;
}

struct WorkUnit {
  WorkUnit(int t, int w, TriangleMesh* o, Mutex* l,
           thread::BlockingCounter* c, BrokenFaceSet* b)
      : total_units(t), work_unit(w), out_mesh(o), lock(l), done(c),
        broken(b) {
  }
  int total_units;
  int work_unit;
  TriangleMesh* out_mesh;
  Mutex* lock;
  thread::BlockingCounter* done;
  BrokenFaceSet* broken;
};

void ProcessWorkUnit(const FaceMap* face_map,
                     const BrokenFaceSet* broken_faces,
                     const CanonicalIdMap* canonicals,
                     const TriangleIdMap* id_map,
                     const TriangleMesh* mesh,
                     WorkUnit unit) {
  int count = 0;
  for (auto it : *face_map) {
    if ((count++ % unit.total_units) != unit.work_unit) {
      continue;
    }

    VLOG(2) << "Computing face # " << count
            << " (of " << face_map->size() << ").";
    if (!ComputeSingleFace(it.first,
                           *broken_faces,
                           *canonicals,
                           it.second,
                           *id_map, *mesh, unit.lock, unit.out_mesh)) {
      MutexLock l(*unit.lock);
      unit.broken->insert(it.first);
    }
  }
  if (unit.done != NULL) {
    unit.done->Decrement();
  }
}

}  // anonymous namespace

FaceReduction_Input::FaceReduction_Input()
    : pool_(NULL),
      allow_broken_faces_(false),
      minimum_triangle_face_ratio_(FLAGS_face_reduction_min_face_ratio) {
}

// static
void FaceReduction::ReduceFacesParallel(TriangleMesh* mesh) {
  FaceReduction::Input input;
  thread::ThreadPool workers;
  workers.StartWorkers();
  input.set_pool(&workers);
  FaceReduction red(input);
  red.Execute(mesh);
}

void FaceReduction::ReduceFaces(TriangleMesh* mesh) { 
  FaceReduction::Input input;
  FaceReduction red(input);
  red.Execute(mesh);
}

bool FaceReduction::Execute(TriangleMesh* mesh) const {
  if (mesh->size() == 0) {
    return true;
  }

  // Step 1, initial data structure.
  VLOG(1) << "Building initial structure.";
  TriangleIdMap id_map;
  ComputeIdMap(*mesh, &id_map);

  // Step 2, compute canonical set for each triangle.
  VLOG(1) << "Computing canonicals.";
  CanonicalIdMap canonicals;
  FaceMap faces;
  ComputeCanonicals(*mesh, id_map, &canonicals, &faces);

  double ratio = static_cast<double>(canonicals.size()) / faces.size();
  if (ratio < input_.minimum_triangle_face_ratio()) {
    VLOG(1) << "Giving up on face reduction, triangle to face ratio too high.";
    return true;
  }

  BrokenFaceSet broken_faces, next_broken_faces;

  // Step 3, for each face, compute new triangulation, and write to mesh.
  VLOG(1) << "Computing new triangulation.";
  TriangleMesh out_mesh;
  Mutex lock;
  if (input_.pool() == NULL) {
    ProcessWorkUnit(&faces, &broken_faces, &canonicals, &id_map, mesh,
                    WorkUnit(1, 0, &out_mesh, &lock, NULL, &next_broken_faces));
  } else {
    // Use thread pool.
    int workers = std::min(faces.size(), input_.pool()->NumWorkers());
    VLOG(1) << "Using threadpool and " << workers << " batches.";
    thread::BlockingCounter counter(workers);

    for (int i = 0; i < workers; ++i) {
      Closure* callback = NewCallback(
          &ProcessWorkUnit, &faces, &broken_faces,
          &canonicals, &id_map, mesh,
          WorkUnit(workers, i, &out_mesh, &lock, &counter, &next_broken_faces));
      input_.pool()->Add(callback);
    }

    // Wait for everything to finish.
    counter.Wait();
  }

  // TODO: re-evaluate broken faces and their neighbors. We would retry
  // here with broken_faces set to next_broken_faces for just the
  // faces that are broken and their neighbors.
  if (!next_broken_faces.empty() && !input_.allow_broken_faces()) {
    LOG(ERROR) << "Unable to perform face reduction (TODO).";
    return false;
  }

  // Step 5, rewrite input mesh.
  VLOG(1) << "Final mesh size: " << out_mesh.size();
  if (mesh->octree() != NULL) {
    Box octree_range = mesh->octree()->range();
    mesh->CopyFrom(out_mesh);
    mesh->InitializeOctreeUnsafe(octree_range);
  } else {
    mesh->CopyFrom(out_mesh);
  }
  return true;
}

}  // namespace printer
