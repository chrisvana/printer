// Copyright 2015
// Author: Christopher Van Arsdale

#include <algorithm>
#include <iostream>
#include <memory>
#include <vector>
#include "common/log/log.h"
#include "common/thread/threadpool.h"
#include "printer/base/mesh.h"
#include "printer/base/octree.h"
#include "printer/execute/driver.h"
#include "printer/execute/marching_cubes.h"
#include "printer/execute/print_box.h"
#include "printer/execute/voxel_fill.h"
#include "printer/simplify/face_reduction.h"
#include "printer/simplify/simplifier.h"
#include "printer/objects/object.h"

DEFINE_bool(driver_run_face_reduction, true,
            "If false, disable face reduction.");

DEFINE_bool(driver_run_simplifier, true,
            "If false, disable simplification.");

DEFINE_int32(driver_min_triangles_to_simplify, -1,
             "If >= 0, the maximum number of triangles to take before we run "
             "the simplification process.");

DEFINE_double(driver_iso_level, 0.5,
              "ISO value to use when generating cubes.");

DEFINE_double(driver_outer_boundary_iso_level, 0.1,
              "ISO value of outer boundary to use when simplifying. You will "
              "need to set --driver_boundary_is_single_voxel=false for this "
              "to be used.");

DEFINE_double(driver_inner_boundary_iso_level, 0.9,
              "ISO value of inner boundary to use when simplifying. You will "
              "need to set --driver_boundary_is_single_voxel=false for this "
              "to be used.");

DEFINE_bool(driver_boundary_is_single_voxel, true,
            "Overrides inner/outer boundary to be the voxel that contains "
            "our initial points, rather than an iso surface defined by "
            "driver_*_boundary_iso_level.");

DEFINE_int32(driver_default_parallelism, -1,
             "Specifies the number of parallel threads we use for some "
             "operations.");

using std::unique_ptr;
using std::vector;

namespace printer {
struct Driver::PrintBoxTracker {
  PrintBoxTracker(const PrintBox* b, bool can)
      : box(b), can_delete(can), deleted(false) {
  }

  const PrintBox* box;
  bool can_delete, deleted;

  void MaybeDelete() {
    if (can_delete && !deleted) {
      VLOG(1) << "Cleaning up voxels to save memory.";
      delete box;
      deleted = true;
    }
  }
  bool CanUse() const { return !deleted; }
};

Driver_Input::Driver_Input()
    : run_face_reduction_(FLAGS_driver_run_face_reduction),
      run_simplifier_(FLAGS_driver_run_simplifier),
      min_triangles_to_simplify_(FLAGS_driver_min_triangles_to_simplify),
      iso_level_(FLAGS_driver_iso_level),
      outer_boundary_iso_level_(FLAGS_driver_outer_boundary_iso_level),
      inner_boundary_iso_level_(FLAGS_driver_inner_boundary_iso_level),
      boundary_is_single_voxel_(FLAGS_driver_boundary_is_single_voxel),
      pool_(NULL),
      parallelism_(FLAGS_driver_default_parallelism),
      owns_objects_(false) {
}

Driver::Driver(const Input& input)
    : input_(input),
      pool_(NULL) {
  if (input.pool() != NULL) {
    pool_ = input.pool();
  } else if (input.parallelism() > 1 || input.parallelism() < 0) {
    owned_pool_.reset(new thread::ThreadPool(input.parallelism()));
    owned_pool_->StartWorkers();
    pool_ = owned_pool_.get();
  }
}

Driver::~Driver() {
}

void Driver::Execute(const vector<PrintObject*>& objects,
                     TriangleMesh* mesh) const {
  LOG(INFO) << "Computing voxels =========================================== ";

  // Compute our voxels.
  VoxelFill::Input input = input_.voxel_input();
  if (pool_ != NULL) {
    input.set_pool(pool_);
  }
  VoxelFill fill(input);
  unique_ptr<PrintBox> box(fill.Execute(objects));

  // Maybe free some memory.
  if (input_.owns_objects()) {
    VLOG(1) << "Cleaning up objects to free memory.";
    for (int i = 0; i < objects.size(); ++i) {
      delete objects[i];
    }
  }

  // Some logging.
  if (VLOG_IS_ON(2)) {
    double counts[256];
    for (int i = 0; i < 256; ++i) { counts[i] = 0; }
    double total = 0;
    for (PrintBox::Iterator iter = box->NewIterator(); iter.Next(); ++total) {
      size_t bucket = std::min<size_t>(
          255, static_cast<size_t>(iter.iso_value() * 256));
      counts[bucket]++;
    }
    std::cerr << "ISO Distribution (cumulative):" << std::endl;
    float prev = 0;
    for (int i = 0; i < 256; ++i) {
      prev += counts[i];
      std::cerr << static_cast<float>(i) / 256 << ": "
                << prev / total * 100 << "%" << std::endl;
    }
  }

  // Generate triangle mesh from voxel map.
  PrintBoxTracker tracker(box.release(), true);
  ExecuteInternal(&tracker, mesh);
  tracker.MaybeDelete();
}

void Driver::ExecuteWithBox(const PrintBox& box, TriangleMesh* mesh) const {
  PrintBoxTracker tracker(&box, false);
  ExecuteInternal(&tracker, mesh);
}


void Driver::ExecuteInternal(PrintBoxTracker* box,
                             TriangleMesh* mesh) const {
  LOG(INFO) << "Building initial mesh. ====================================== ";
  unique_ptr<MarchingCubes> cubes(GetCubes(false));
  cubes->ExecuteWithISO(*box->box, input_.iso_level(), mesh);

  // Maybe reduce our memory if we don't need the PrintBox anymore.
  if (!MightSimplify(*mesh)) {
    box->MaybeDelete();
  }

  LOG(INFO) << "Initial triangle count: " << mesh->size();
  RunReduction(false, mesh);

  if (MightSimplify(*mesh) && box->CanUse()) {
    LOG(INFO) << "Simplifying mesh, initial triangle count: " << mesh->size();
    RunSimplifier(box, mesh);
  }

  LOG(INFO) << "Final triangle count: " << mesh->size();
  VLOG(2) << "Final bounding box: "
          << mesh->MinimalBoundingBox().DebugString();
}

bool Driver::MightSimplify(const TriangleMesh& mesh) const {
   return input_.run_simplifier() &&
       (input_.min_triangles_to_simplify() < 0 ||
        mesh.size() >= input_.min_triangles_to_simplify());
}

void Driver::RunSimplifier(PrintBoxTracker* box,
                           TriangleMesh* mesh) const {
  ParallelSimplifier::Input simplifier_input = input_.simplifier_input();
  if (simplifier_input.pool() == NULL) {
    simplifier_input.set_pool(pool_);
  }
  ParallelSimplifier simplifier(simplifier_input);

  unique_ptr<Octree> octree;
  if (simplifier.RequiresBoundary()) {
    LOG(INFO) << "Computing boundary. ======================================= ";
    octree.reset(ComputeBoundary(*mesh, *box->box));
    if (octree.get() == NULL) {
      LOG(FATAL) << "Simplifier requires boundary, but no boundary information "
                 << "supplied to Driver.";
    }
  }

  box->MaybeDelete();

  VLOG(1) << "Initializing mesh octree.";
  mesh->InitializeOctreeUnsafe(octree != NULL ? octree->range() :
                               mesh->MinimalBoundingBox());

  LOG(INFO) << "Running simplification ====================================== ";
  simplifier.Execute(octree.get(), mesh);
}

Octree* Driver::ComputeBoundary(const TriangleMesh& base,
                                const PrintBox& box) const {
  unique_ptr<MarchingCubes> cubes(GetCubes(true /* for boundary */));

  if (input_.boundary_is_single_voxel()) {
    VLOG(1) << "Computing single voxel boundary.";
    TriangleMesh boundary;
    cubes->ExecuteWithBoundary(box, input_.iso_level(), NULL, &boundary);
    VLOG(1) << "Finished boundary.";
    RunReduction(true, &boundary);
    return boundary.NewOctree(GetBoundaryBox(base, boundary, box), false);
  }

  unique_ptr<Octree> octree;
  if (input_.outer_boundary_iso_level() >= 0) {
    VLOG(1) << "Computing outer boundary.";
    TriangleMesh outer;
    cubes->ExecuteWithISO(box, input_.outer_boundary_iso_level(), &outer);
    VLOG(1) << "Finished outer boundary.";
    RunReduction(true, &outer);
    VLOG(1) << "Building final octree.";
    octree.reset(outer.NewOctree(GetBoundaryBox(base, outer, box), false));
    CHECK(octree.get());
  }

  if (input_.inner_boundary_iso_level() >= 0) {
    VLOG(1) << "Computing inner boundary.";
    TriangleMesh inner;
    cubes->ExecuteWithISO(box, input_.inner_boundary_iso_level(), &inner);
    VLOG(1) << "Finished inner boundary.";
    RunReduction(true, &inner);
    VLOG(1) << "Building final octree.";
    if (octree.get() == NULL) {
      VLOG(3) << "No outer boundary found.";
      octree.reset(inner.NewOctree(GetBoundaryBox(base, inner, box), false));
    } else {
      VLOG(3) << "Merging inner/outer octrees.";
      for (TriangleMesh::TriangleIterator iter = inner.iterator();
           !iter.Done(); iter.Next()) {
        octree->AddTriangle(iter.triangle().Flipped());
      }
    }
    CHECK(octree.get());
  }

  VLOG(1) << "Finished boundary.";
  return octree.release();
}

MarchingCubes* Driver::GetCubes(bool for_boundary) const {
  MarchingCubes::Input marching_input = input_.marching_cubes_input();
  if (marching_input.pool() == NULL) {
    marching_input.set_pool(pool_);
  }
  if (for_boundary) {
    marching_input.set_min_island_size(0);
  }
  return new MarchingCubes(marching_input);
}

void Driver::RunReduction(bool allow_broken, TriangleMesh* mesh) const {
  if (!input_.run_face_reduction()) {
    return;
  }

  LOG(INFO) << "Reducing faces to minimal triangles (staring with "
            << mesh->size() << ").";
  FaceReduction::Input face_input = input_.face_reduction_input();
  if (face_input.pool() == NULL) {
    face_input.set_pool(pool_);
  }
  face_input.set_allow_broken_faces(allow_broken);
  FaceReduction red(face_input);
  red.Execute(mesh);
}

Box Driver::GetBoundaryBox(const TriangleMesh& base,
                           const TriangleMesh& test,
                           const PrintBox& box) const {
  Box test_box = test.MinimalBoundingBox();
  Box base_box = base.MinimalBoundingBox();
  Box region = box.BoundingBox();
  if (test_box.Unioned(base_box) == test_box) {
    return test_box;
  }
  return region;
}

}  // namespace printer
