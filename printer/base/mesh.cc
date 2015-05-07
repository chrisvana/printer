// Copyright 2015
// Author: Christopher Van Arsdale

#include <algorithm>
#include <string>
#include <map>
#include <set>
#include <unordered_map>
#include "printer/base/mesh.h"
#include "printer/base/octree.h"
#include "common/log/log.h"

#include "common/base/flags.h"

using std::string;
using std::set;
using std::map;
using std::unordered_map;
using std::unique_ptr;

namespace printer {

TriangleMesh::TriangleMesh()
    : next_id_(1) {
}

TriangleMesh::~TriangleMesh() {
}

size_t TriangleMesh::octree_node_count() const {
  return octree_.get() == NULL ? 0 : octree_->total_nodes();
}

bool TriangleMesh::InitializeOctree(const Box& boundary) {
  octree_.reset(NewOctree(boundary, true));
  return octree_.get() != NULL;
}

Octree* TriangleMesh::NewOctree(const Box& boundary, bool safe) const {
  unique_ptr<Octree> octree(new Octree(boundary));
  for (TriangleIterator iter(*this); !iter.Done(); iter.Next()) {
    Triangle t = iter.triangle();
    if (safe && octree->IntersectsNotCoincident(t)) {
      VLOG(1) << "Cannot build octree, intersecting triangles.";
      return NULL;
    }
    octree->AddTriangle(t);
  }
  return octree.release();
}

bool TriangleMesh::InitializeOctreeFrom(const TriangleMesh& parent) {
  return (parent.octree_.get() != NULL &&
          InitializeOctree(parent.octree_->range()));
}

void TriangleMesh::InitializeOctreeUnsafe(const Box& boundary) {
  octree_.reset(new Octree(boundary));
  for (TriangleIterator iter(*this); !iter.Done(); iter.Next()) {
    octree_->AddTriangle(iter.triangle());
  }
}

Box TriangleMesh::MinimalBoundingBox() const {
  Box box;
  bool first = true;
  for (TriangleIterator iter(*this); !iter.Done(); iter.Next()) {
    if (first) {
      first = false;
      box = iter.triangle().BoundingBox();
    } else {
      box.UnionWith(iter.triangle().BoundingBox());
    }
  }
  return box;
}

bool TriangleMesh::IntersectsWithOctree(const TriangleMesh& other) const {
  if (octree_.get() == NULL || other.octree_.get() == NULL) {
    LOG(FATAL) << "Must call InitializeOctree.";
  }
  return octree_->Intersects(*other.octree_);
}

bool TriangleMesh::AddTriangle(const Triangle& t_in) {
  Triangle t = t_in.Normalized();
  VLOG(3) << t.DebugString();

  if (!t.WellFormed()) {
    LOG(ERROR) << "Not well formed: " << t.DebugString();
    return false;
  }

  if (octree_.get() != NULL && octree_->IntersectsNotCoincident(t)) {
    VLOG(2) << "Triangle intersects mesh.";
    return false;
  }

  return AddNoChecks(t);
}

void TriangleMesh::Clear() {
  TriangleMesh empty;
  CopyFrom(empty);
}

bool TriangleMesh::AddNoChecks(const Triangle& triangle) {
  InternalTriangle internal;
  FillPoints(triangle, &internal);
  if (!triangles_.insert(internal).second) {
    VLOG(3) << "Already have triangle.";
    return false;
  }
  if (octree_.get() != NULL) {
    octree_->AddTriangle(triangle);
  }
  FillAdjascent(internal);
  return true;
}

void TriangleMesh::GetAdjascentToPoint(const Point& point,
                                       TriangleList* out) const {
  int id = MaybeGetPoint(point);
  if (id <= 0) {
    return;
  }

  const PointInfo& info = point_to_triangles_.find(id)->second;
  for (const InternalTriangle& t : info.triangles) {
    out->push_back(ConvertTriangle(t));
  }
}

void TriangleMesh::CopyFrom(const TriangleMesh& other) {
  // Deep copy all of the members (none of them have pointers).
  triangles_ = other.triangles_;
  point_to_triangles_ = other.point_to_triangles_;
  point_to_id_ = other.point_to_id_;

  octree_.reset();
  if (other.octree() != NULL) {
    InitializeOctreeUnsafe(other.octree()->range());
  }
}

bool TriangleMesh::CheckManifold() const {
  if (triangles_.empty()) {
    return false;
  }

  // TODO: self intersection test?
  std::map<InternalEdge, int> edge_count;
  for (const InternalTriangle& t : triangles_) {
    // Normal
    edge_count[InternalEdge(t.p0, t.p1)]++;
    edge_count[InternalEdge(t.p1, t.p2)]++;
    edge_count[InternalEdge(t.p2, t.p0)]++;

    // Flipped
    edge_count[InternalEdge(t.p1, t.p0)]++;
    edge_count[InternalEdge(t.p2, t.p1)]++;
    edge_count[InternalEdge(t.p0, t.p2)]++;
  }
  for (auto it : edge_count) {
    if (it.second != 2) {
      VLOG(4) << "Edge ids: " << it.first.p0 << " -> " << it.first.p1;
      VLOG(1) << "Incorrect shared edges: "
              << Edge(ConvertPoint(it.first.p0),
                      ConvertPoint(it.first.p1)).DebugString()
              << " (have " << it.second << " edges, expected 2).";
      return false;
    }
  }
  return true;
}

bool TriangleMesh::TestEdgeCollapse(const Edge& edge,
                                    const Point& new_vertex,
                                    TriangleList* old_triangles,
                                    TriangleList* new_triangles) const {
  int p0, p1;
  if ((p0 = MaybeGetPoint(edge.p0())) <= 0 ||
      (p1 = MaybeGetPoint(edge.p1())) <= 0) {
    VLOG(2) << "Unknown edge: " << edge.DebugString();
    return false;
  }

  // Get adjascent triangles.
  const PointInfo& primary = point_to_triangles_.find(p0)->second;
  const PointInfo& secondary = point_to_triangles_.find(p1)->second;
  if (!TwoPointOverlap(primary,  secondary)) {
    VLOG(2) << "Cannot collapse edge, incorrect shared points.";
    return false;
  }

  // Calculate all of the new triangles.
  for (int i = 0; i < 2; ++i) {
    const InternalTriangleSet& triangles =
        (i == 0 ? primary.triangles : secondary.triangles);
    for (const InternalTriangle& old_triangle : triangles) {
      // Compute the new value (if any points were part of the collapsed edge,
      // replace them with new_vertex).
      Triangle old_value = ConvertTriangle(old_triangle);
      Triangle new_value = old_value;
      int shared_points = 0;
      if (old_triangle.p0 == p0 || old_triangle.p0 == p1) {
        ++shared_points;
        new_value.set_p0(new_vertex);
      }
      if (old_triangle.p1 == p0 || old_triangle.p1 == p1) {
        ++shared_points;
        new_value.set_p1(new_vertex);
      }
      if (old_triangle.p2 == p0 || old_triangle.p2 == p1) {
        ++shared_points;
        new_value.set_p2(new_vertex);
      }

      // Check the validity of the new triangle. We ignore ones we are
      // eliminating (not well formed, since 2 vetices share the same point
      // after we collapsed the edge).
      if (new_value.WellFormed()) {
        old_triangles->push_back(old_value);

        // We give up if we flipped the normal orientation (rotated more than
        // 90 degrees).
        if ((new_value.normal() * old_value.normal()) < 0) {
          VLOG(2) << "Cannot collapse edge, normal shifted: "
                  << old_value.normal().DebugString() << " to "
                  << new_value.normal().DebugString();
          return false;
        }

        // Record the new triangle.
        new_triangles->push_back(new_value);
      } else if (i == 0) {
        // We need to record the old triangle regardless, but we don't want
        // to double count them.
        old_triangles->push_back(old_value);
      }
    }
  }

  return true;
}

bool TriangleMesh::ApplyUpdate(const TriangleList& old_triangles,
                               const TriangleList& new_triangles,
                               bool check_intersection) {
  if (VLOG_IS_ON(3)) {
    VLOG(3) << "Update: " << old_triangles.size()
            << " -> " << new_triangles.size();
    for (const Triangle& t : old_triangles) {
      VLOG(3) << "Old triangle: " << t.DebugString();
    }
    for (const Triangle& t : new_triangles) {
      VLOG(3) << "New triangle: " << t.DebugString();
    }
  }

  // TODO: check edges.

  // Check that we have all of our old triangles
  InternalTriangleSet old_triangles_internal;
  for (const Triangle& t : old_triangles) {
    InternalTriangle old_t(MaybeGetPoint(t.p0()),
                           MaybeGetPoint(t.p1()),
                           MaybeGetPoint(t.p2()));
    if (triangles_.find(old_t) == triangles_.end()) {
      LOG(ERROR) << "No triangle in mesh: " << t.DebugString();
      return false;
    }
  }

  // Erase old triangles
  for (const Triangle& t : old_triangles) {
    RemoveTriangle(t);
  }

  if (ApplyUpdateInternal(new_triangles, old_triangles, check_intersection)) {
    return true;
  }

  // Recovery:
  // Re-add the old ones, ignore safety checks.
  for (const Triangle& t : old_triangles) {
    AddNoChecks(t);
  }
  return false;
}

bool TriangleMesh::ApplyUpdateInternal(const TriangleList& new_triangles,
                                       const TriangleList& old_triangles,
                                       bool check_intersection) {
  // Check triangle intersection
  if (check_intersection) {
    CHECK(octree_.get() != NULL)
        << "Must call InitializeOctree before ApplyUpdate with "
        << "\"check_intersection\", this:" << this;
    for (const Triangle& t : old_triangles) {
      CHECK(octree_->RemoveTriangle(t));
    }

    Octree tmp(octree_->range());
    for (const Triangle& t : new_triangles) {
      tmp.AddTriangle(t);
    }

    if (tmp.IntersectsNotCoincident(*octree_)) {
      VLOG(2) << "Invalid update, has invalid mesh intersection.";
      return false;
    }
  }

  // Add new triangles.
  for (const Triangle& t : new_triangles) {
    AddNoChecks(t);
  }

  VLOG(2) << "After update, total octree nodes: " << octree_->total_nodes();
  return true;
}

void TriangleMesh::RemoveTriangle(const Triangle& t) {
  InternalTriangle internal(MaybeGetPoint(t.p0()),
                            MaybeGetPoint(t.p1()),
                            MaybeGetPoint(t.p2()));

  VLOG(2) << "Removing: " << t.DebugString();
  if (triangles_.erase(internal)) {
    // p0
    auto it = point_to_triangles_.find(internal.p0);
    CHECK(it != point_to_triangles_.end());
    {
      PointInfo& info = it->second;
      CHECK_EQ(1, info.triangles.erase(internal));
      if (info.triangles.empty()) {
        point_to_id_.erase(info.point);
        point_to_triangles_.erase(it);
      }
    }

    // p1
    it = point_to_triangles_.find(internal.p1);
    CHECK(it != point_to_triangles_.end());
    {
      PointInfo& info = it->second;
      CHECK_EQ(1, info.triangles.erase(internal));
      if (info.triangles.empty()) {
        point_to_id_.erase(info.point);
        point_to_triangles_.erase(it);
      }
    }

    // p2
    it = point_to_triangles_.find(internal.p2);
    CHECK(it != point_to_triangles_.end());
    {
      PointInfo& info = it->second;
      CHECK_EQ(1, info.triangles.erase(internal));
      if (info.triangles.empty()) {
        point_to_id_.erase(info.point);
        point_to_triangles_.erase(it);
      }
    }
  }
}

bool TriangleMesh::ApplyEdgeCollapse(const Edge& edge,
                                     const Point& new_vertex) {
  TriangleList old_t, new_t;
  if (!TestEdgeCollapse(edge, new_vertex, &old_t, &new_t)) {
    return false;
  }
  return ApplyUpdate(old_t, new_t, true);
}

bool TriangleMesh::TwoPointOverlap(const PointInfo& primary,
                                   const PointInfo& secondary) {
  const InternalTriangleSet& big =
      (primary.triangles.size() >
       secondary.triangles.size() ?
       primary.triangles : secondary.triangles);
  const InternalTriangleSet& small =
      (primary.triangles.size() >
       secondary.triangles.size() ?
       secondary.triangles : primary.triangles);

  thread_local unordered_map<int, bool> points;
  points.clear();
  for (const InternalTriangle& t : small) {
    points[t.p0] = false;
    points[t.p1] = false;
    points[t.p2] = false;
  }

  int in_common = 0;
  for (const InternalTriangle& t : big) {
    {  // p0
      auto it = points.find(t.p0);
      if (it != points.end() && !it->second) {
        it->second = true;
        ++in_common;
      }
    }
    {  // p1
      auto it = points.find(t.p1);
      if (it != points.end() && !it->second) {
        it->second = true;
        ++in_common;
      }
    }
    {  // p2
      auto it = points.find(t.p2);
      if (it != points.end() && !it->second) {
        it->second = true;
        ++in_common;
      }
    }

    if (in_common > 4) {
      return false;
    }
  }
  return true;
}

bool TriangleMesh::Merge(const TriangleMesh& mesh) {
  // Reserve memory.
  point_to_id_.reserve(point_to_id_.size() + mesh.point_to_id_.size());
  triangles_.reserve(triangles_.size() + mesh.triangles_.size());
  point_to_triangles_.reserve(point_to_triangles_.size() +
                              mesh.point_to_triangles_.size());

  bool retval = true;
  for (TriangleIterator iter = mesh.iterator(); !iter.Done(); iter.Next()) {
    retval |= AddTriangle(iter.triangle());    
  }
  return retval;
}

TriangleMeshMerger::TriangleMeshMerger(TriangleMesh* output)
    : is_running_(false),
      output_(output) {
}

TriangleMeshMerger::~TriangleMeshMerger() {
  Wait();
}

void TriangleMeshMerger::Add(TriangleMesh* mesh){
  int pending = 0;
  size_t pending_triangles = 0;
  {
    MutexLock l(lock_);
    pending_.push_back(mesh);

    while (!is_running_ && !pending_.empty()) {
      std::vector<TriangleMesh*> copy = pending_;
      pending_.clear();
      is_running_ = true;
      l.unlock();

      // Process copy outside of lock.
      size_t input_size = 0;
      for (TriangleMesh* m : copy) {
        input_size += m->size();
        output_->Merge(*m);
        delete m;
      }
      VLOG(1) << "Merged in " << input_size << " triangles, output size now: "
              << output_->size();

      l.lock();
      is_running_ = false;
    }

    if (!is_running_ && pending_.empty()) {
      done_.notify_all();
    }

    pending = pending_.size();
    for (const TriangleMesh* mesh : pending_) {
      pending_triangles += mesh->size();
    }
  }
  VLOG(1) << "Triangle merger, pending merges: " << pending
          << " with pending triangles: " << pending_triangles;
}

void TriangleMeshMerger::Wait() {
  MutexLock l(lock_);
  while (is_running_ || !pending_.empty()) {
    done_.wait(l);
  }
}

}  // namespace printer
