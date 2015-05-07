// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_BASE_MESH_H__
#define _PRINTER_BASE_MESH_H__

#include <stdint.h>
#include <map>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <map>
#include <string>
#include "printer/base/geometry.h"

namespace printer {

class Octree;

class TriangleMesh {
 public:
  typedef std::unordered_set<Triangle, HashTriangle> TriangleSet;
  typedef std::vector<Triangle> TriangleList;

  class TriangleIterator;
  class EdgeIterator;

 public:
  TriangleMesh();
  ~TriangleMesh();

  // Triangle building
  void Clear();
  bool AddTriangle(const Triangle& t);
  bool Merge(const TriangleMesh& mesh);  // false if any conflicts.

  // Octree handling.
  size_t octree_node_count() const;
  bool InitializeOctree(const Box& boundary);
  bool InitializeOctreeFrom(const TriangleMesh& parent);
  void InitializeOctreeUnsafe(const Box& boundary);
  bool IntersectsWithOctree(const TriangleMesh& other) const;
  Octree* NewOctree(const Box& boundary, bool safe) const;
  Octree* NewOctreeFromMinimalBounds() const {
    return NewOctree(MinimalBoundingBox(), true);
  }
  const Octree* octree() const { return octree_.get(); }

  bool CheckManifold() const;
  Box MinimalBoundingBox() const;

  size_t size() const { return triangles_.size(); }
  TriangleIterator iterator() const;
  EdgeIterator edge_iterator() const;
  void GetAdjascentToPoint(const Point& point, TriangleList* out) const;

  void CopyFrom(const TriangleMesh& other);

  // Modifying the mesh (returns false if unable to make changes, e.g.
  // no longer a manifold if check_intersection is set).
  bool ApplyUpdate(const TriangleList& old_triangles,
                   const TriangleList& new_triangles,
                   bool check_intersection);

  // Edge collapse (false if not able to do it).
  bool TestEdgeCollapse(const Edge& edge,
                        const Point& new_vertex,
                        TriangleList* old_triangles,
                        TriangleList* new_triangles) const;
  bool ApplyEdgeCollapse(const Edge& edge,
                         const Point& new_vertex);


 private:
  struct InternalTriangle {
    InternalTriangle() {}
    InternalTriangle(int p0i, int p1i, int p2i) : p0(p0i), p1(p1i), p2(p2i) {}
    int p0, p1, p2;
    bool operator==(const InternalTriangle& other) const {
      return p0 == other.p0 && p1 == other.p1 && p2 == other.p2;
    }
    bool operator<(const InternalTriangle& other) const {
      return p0 != other.p0 ? p0 < other.p0 :
          (p1 != other.p1 ? p1 < other.p1 : p2 < other.p2);
    }
    int get_not(int p0_in, int p1_in) const {
      return (p0 != p0_in && p0 != p1_in ? p0 :
              p1 != p0_in && p1 != p1_in ? p1 : p2);
    }
    int contains(int p) const { return p0 == p || p1 == p || p2 == p; }
  };
  struct InternalEdge {
    InternalEdge() {}
    InternalEdge(int p0i, int p1i) : p0(p0i), p1(p1i) {}
    int p0, p1;
    bool operator<(const InternalEdge& other) const {
      return p0 != other.p0 ? p0 < other.p0 : p1 < other.p1;
    }
  };
  struct HashInternalTriangle {
    size_t operator()(const InternalTriangle& t) const {
      return geometry_hash_stupid(t.p0, t.p1, t.p2);
    }
  };
  typedef std::unordered_map<Point, int, HashPoint> PointToIdMap;
  typedef std::unordered_set<InternalTriangle, HashInternalTriangle>
      InternalTriangleSet;
  struct PointInfo {
    Point point;
    InternalTriangleSet triangles;
  };
  typedef std::unordered_map<int, PointInfo> PointMap;

  // Internal update functions.
  bool ApplyUpdateInternal(const TriangleList& new_triangles,
                           const TriangleList& old_triangles,
                           bool check_intersection);
  bool AddNoChecks(const Triangle& triangle);
  void RemoveTriangle(const Triangle& t);
  int GetOrInsertPoint(const Point& p);
  void FillPoints(const Triangle& triangle, InternalTriangle* internal);
  void FillAdjascent(const InternalTriangle& internal);

  // others.
  Point ConvertPoint(int p) const;
  Triangle ConvertTriangle(const InternalTriangle& t) const;
  int MaybeGetPoint(const Point& p) const;
  static bool TwoPointOverlap(const PointInfo& primary,
                              const PointInfo& secondary);

  std::unique_ptr<Octree> octree_;

  // Point <-> ID
  int next_id_;
  PointToIdMap point_to_id_;

  // Triangle set
  InternalTriangleSet triangles_;

  // Point -> Triangle
  PointMap point_to_triangles_;
};

class TriangleMesh::TriangleIterator {
 public:
  explicit TriangleIterator(const TriangleMesh& parent)
      : iter_(parent.triangles_.begin()),
        parent_(parent) {
    if (!Done()) {
      InitTriangle();
    }
  }
  ~TriangleIterator() {}

  bool Done() const { return iter_ == parent_.triangles_.end(); }
  void Next() { ++iter_; InitTriangle(); }
  const Triangle& triangle() const { return triangle_; }

 private:
  void InitTriangle() {
    if (!Done()) {
      triangle_ = parent_.ConvertTriangle(*iter_);
    }
  }
  InternalTriangleSet::const_iterator iter_;
  const TriangleMesh& parent_;
  Triangle triangle_;
};

class TriangleMesh::EdgeIterator {
 public:
  explicit EdgeIterator(const TriangleMesh& parent)
    : iter_(TriangleIterator(parent)),
      edge_(0) {
  }
  ~EdgeIterator() {}

  bool Done() const { return iter_.Done(); }
  void Next() { if (++edge_ > 2) { edge_ = 0; iter_.Next(); } }
  const Triangle& triangle() const { return iter_.triangle(); }
  Edge edge() const {
    switch (edge_) {
      case 0: return triangle().e0();
      case 1: return triangle().e1();
      default: return triangle().e2();
    }
  }

 private:
  TriangleMesh::TriangleIterator iter_;
  int edge_;
};

class TriangleMeshMerger {
 public:
  explicit TriangleMeshMerger(TriangleMesh* output);
  ~TriangleMeshMerger();

  void Add(TriangleMesh* mesh);  // may block, but only 1 thread will block.
  void Wait();

 private:
  typedef std::mutex Mutex;
  typedef std::unique_lock<Mutex> MutexLock;
  typedef std::condition_variable CondVar;

  Mutex lock_;
  bool is_running_;
  CondVar done_;
  std::vector<TriangleMesh*> pending_;
  TriangleMesh* output_;
};

inline Point TriangleMesh::ConvertPoint(int id) const {
  auto it = point_to_triangles_.find(id);
  if (it == point_to_triangles_.end()) {
    return Point();
  }
  return it->second.point;
}

inline Triangle TriangleMesh::ConvertTriangle(
    const TriangleMesh::InternalTriangle& t) const {
  return Triangle(ConvertPoint(t.p0), ConvertPoint(t.p1), ConvertPoint(t.p2));
}

inline int TriangleMesh::MaybeGetPoint(const Point& p) const {
  auto it = point_to_id_.find(p);
  return (it == point_to_id_.end() ? -1 : it->second);
}

inline int TriangleMesh::GetOrInsertPoint(const Point& p) {
  int& id = point_to_id_[p];
  if (id == 0) {
    id = next_id_++;
    point_to_triangles_[id].point = p;
  }
  return id;
}

inline void TriangleMesh::FillPoints(const Triangle& triangle,
                                     InternalTriangle* internal) {
  internal->p0 = GetOrInsertPoint(triangle.p0());
  internal->p1 = GetOrInsertPoint(triangle.p1());
  internal->p2 = GetOrInsertPoint(triangle.p2());
}

inline void TriangleMesh::FillAdjascent(const InternalTriangle& internal) {
  PointInfo* info = &point_to_triangles_[internal.p0];
  info->triangles.insert(internal);
  info = &point_to_triangles_[internal.p1];
  info->triangles.insert(internal);
  info = &point_to_triangles_[internal.p2];
  info->triangles.insert(internal);
}

inline TriangleMesh::TriangleIterator TriangleMesh::iterator() const {
  return TriangleIterator(*this);
}

inline TriangleMesh::EdgeIterator TriangleMesh::edge_iterator() const {
  return EdgeIterator(*this);
}

}  // namespace printer

#endif  // _PRINTER_BASE_MESH_H__
