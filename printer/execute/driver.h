// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_EXECUTE_DRIVER_H__
#define _PRINTER_EXECUTE_DRIVER_H__

#include <memory>
#include <vector>
#include "common/base/macros.h"
#include "printer/simplify/simplifier.h"
#include "printer/simplify/face_reduction.h"
#include "printer/execute/marching_cubes.h"
#include "printer/execute/voxel_fill.h"

namespace thread {
class ThreadPool;
}

namespace printer {

class PrintBox;
class PrintObject;
class TriangleMesh;

class Driver_Input {
 public:
  Driver_Input();
  ~Driver_Input() {}

  // Accessors
  bool run_face_reduction() const { return run_face_reduction_; }
  bool run_simplifier() const { return run_simplifier_; }
  int min_triangles_to_simplify() const { return min_triangles_to_simplify_; }
  double iso_level() const { return iso_level_; }
  double outer_boundary_iso_level() const { return outer_boundary_iso_level_; }
  double inner_boundary_iso_level() const { return inner_boundary_iso_level_; }
  bool boundary_is_single_voxel() const { return boundary_is_single_voxel_; }
  const VoxelFill::Input& voxel_input() const { return voxel_input_; }
  const ParallelSimplifier::Input& simplifier_input() const {
    return simplifier_input_;
  }
  const FaceReduction::Input& face_reduction_input() const {
    return face_reduction_input_;
  }
  const MarchingCubes::Input& marching_cubes_input() const {
    return marching_cubes_input_;
  }
  thread::ThreadPool* pool() const { return pool_; }
  int parallelism() const { return parallelism_; }
  bool owns_objects() const { return owns_objects_; }

  // Mutators
  void set_run_face_reduction(bool run) { run_face_reduction_ = run; }
  void set_run_simplifier(bool run) { run_simplifier_ = run; }
  void set_min_triangles_to_simplify(int m) { min_triangles_to_simplify_ = m; }
  void set_iso_level(double iso) { iso_level_ = iso; }
  void set_outer_boundary_iso_level(double iso) {
    outer_boundary_iso_level_ = iso;
  }
  void set_inner_boundary_iso_level(double iso) {
    inner_boundary_iso_level_ = iso;
  }
  void set_boundary_is_single_voxel(bool s) { boundary_is_single_voxel_ = s; }
  VoxelFill::Input* mutable_voxel_input() { return &voxel_input_; }
  ParallelSimplifier::Input* mutable_simplifier_input() {
    return &simplifier_input_;
  }
  FaceReduction::Input* mutable_face_reduction_input() {
    return &face_reduction_input_;
  }
  MarchingCubes::Input* mutable_marching_cubes_input() {
    return &marching_cubes_input_;
  }
  void set_pool(thread::ThreadPool* pool) { pool_ = pool; }
  void set_parallelism(int p) { parallelism_ = p; }
  void set_owns_objects(bool owns) { owns_objects_ = owns; }

 private:
  bool run_face_reduction_;
  bool run_simplifier_;
  int min_triangles_to_simplify_;
  double iso_level_;
  double outer_boundary_iso_level_;
  double inner_boundary_iso_level_;
  bool boundary_is_single_voxel_;
  VoxelFill::Input voxel_input_;
  ParallelSimplifier::Input simplifier_input_;
  FaceReduction::Input face_reduction_input_;
  MarchingCubes::Input marching_cubes_input_;
  thread::ThreadPool* pool_;
  int parallelism_;
  bool owns_objects_;
};

class Driver {
 public:
  typedef Driver_Input Input;
  explicit Driver(const Input& input);
  ~Driver();

  void Execute(const std::vector<PrintObject*>& objects,
               TriangleMesh* output) const;
  void ExecuteWithBox(const PrintBox& print_box, TriangleMesh* output) const;

 private:
  struct PrintBoxTracker;
  void ExecuteInternal(PrintBoxTracker* print_box,
                       TriangleMesh* mesh) const;
  bool MightSimplify(const TriangleMesh& mesh) const;
  void RunSimplifier(PrintBoxTracker* box,
                     TriangleMesh* mesh) const;
  MarchingCubes* GetCubes(bool for_boundary) const;
  Octree* ComputeBoundary(const PrintBox& box) const;
  void RunReduction(bool allow_broken, TriangleMesh* mesh) const;
  DISALLOW_COPY_AND_ASSIGN(Driver);

  Input input_;
  thread::ThreadPool* pool_;
  std::unique_ptr<thread::ThreadPool> owned_pool_;
  std::unique_ptr<FaceReduction> face_reduction_;
};

}  // namespace printer

#endif  // _PRINTER_EXECUTE_DRIVER_H__
