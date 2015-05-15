// Copyright 2015
// Author: Christopher Van Arsdale

#include <string>
#include <cstddef>
#include <memory>
#include <vector>
#include <mutex>
#include "common/base/callback.h"
#include "common/base/flags.h"
#include "common/log/log.h"
#include "common/thread/counter.h"
#include "common/thread/threadpool.h"
#include "printer/execute/voxel_fill.h"
#include "printer/execute/print_box.h"
#include "printer/objects/object.h"

DEFINE_string(voxel_print_box_name, "BinaryPrintBox",
              "Other options include FloatPrintBox, etc.");

DEFINE_double(horizontal_resolution, 0.1,
              "Horizontal resolution, in mm.");

DEFINE_double(vertical_resolution, 0.1,
              "Vertical resolution, in mm.");

DEFINE_double(print_region_size, 100,
              "Print region size, in mm.");

DEFINE_int32(recursive_box_split_size, 1000,
             "Minimum size to split down to before we go to linear scan.");

using std::unique_ptr;
using std::vector;
using thread::Semaphore;

namespace printer {
namespace {
typedef std::mutex Mutex;
typedef std::unique_lock<Mutex> MutexLock;
}
class VoxelFill::ObjectSet {
 public:
  explicit ObjectSet(const vector<PrintObject*>& objects)
      : objects_(objects) {
  }
  ~ObjectSet() {}

  size_t num_objects() const { return objects_.size(); }
  PrintObject* object(int idx) const { return objects_[idx]; }

  // ContainsPoint
  float PointISO(const Point& point) const {
    float max_val = 0;
    for (int i = 0; max_val < 1 && i < objects_.size(); ++i) {
      max_val = std::max(max_val, objects_[i]->ISOValue(point));
      if (max_val >= 1) {
        return 1;
      }
    }
    return std::max(0.0f, max_val);
  }

  // FullyContains
  bool FullyContains(const Box& region) const {
    for (int i = 0; i < objects_.size(); ++i) {
      if (objects_[i]->FullyContains(region)) {
        return true;
      }
    }
    return false;
  }

  // Generate new ObjectSet for region.
  ObjectSet GetRegion(const Box& region) const {
    vector<PrintObject*> objects;
    for (int i = 0; i < objects_.size(); ++i) {
      if (objects_[i]->MayIntersectRegion(region)) {
        objects.push_back(objects_[i]);
      }
    }
    return ObjectSet(objects);
  }

  bool ThreadSafe() const {
    for (int i = 0; i < objects_.size(); ++i) {
      if (!objects_[i]->ThreadSafe()) {
        return false;
      }
    }
    return true;
  }

 private:
  vector<PrintObject*> objects_;
};

VoxelFill_Input::VoxelFill_Input()
    : print_box_name_(FLAGS_voxel_print_box_name),
      horizontal_resolution_(FLAGS_horizontal_resolution),
      vertical_resolution_(FLAGS_vertical_resolution),
      print_region_(Box(Point(0, 0, 0),
                        Point(FLAGS_print_region_size,
                              FLAGS_print_region_size,
                              FLAGS_print_region_size))),
      pool_(NULL) {
}

VoxelFill::VoxelFill(const Input& input)
    : input_(input) {
}

VoxelFill::~VoxelFill() {
}

PrintBox* VoxelFill::Execute(const vector<PrintObject*>& objects) const {
  Box print_region = DetermineRegion(objects);
  int size_x = print_region.size_x() / input_.horizontal_resolution();
  int size_y = print_region.size_y() / input_.horizontal_resolution();
  int size_z = print_region.size_z() / input_.vertical_resolution();

  ObjectSet object_set(objects);
  unique_ptr<PrintBox> output;
  output.reset(PrintBox::FromName(input_.print_box_name(),
                                  size_x, size_y, size_z,
                                  input_.horizontal_resolution(),
                                  input_.vertical_resolution()));
  output->SetOrigin(print_region.bottom());
  LOG(INFO) << "Computing voxels for region: "
            << print_region.DebugString()
            << " with resolution h=" << input_.horizontal_resolution()
            << ", v=" << input_.vertical_resolution();

  Semaphore sem;
  Mutex lock;
  ProcessSplit(print_region,
               object_set,
               RangeBox(Range(0, size_x),
                        Range(0, size_y),
                        Range(0, size_z)),
               output.get(),
               &lock,
               &sem);
  sem.Wait();
  return output.release();
}

void VoxelFill::ProcessSplit(const Box& print_region,
                             const ObjectSet& objects,
                             const RangeBox& range,
                             PrintBox* output,
                             Mutex* lock,
                             Semaphore* sem) const {
  // Figure out which objects are in our range.
  Box region = GetBox(print_region, range);
  ObjectSet object_subset = objects.GetRegion(region);

  // If the whole region is obviously uniform, handle that now.
  if (object_subset.num_objects() == 0) {
    MutexLock l(*lock);
    output->FillRegion(range.x_range.start, range.x_range.end,
                       range.y_range.start, range.y_range.end,
                       range.z_range.start, range.z_range.end,
                       false);
  } else if (object_subset.FullyContains(region)) {
    MutexLock l(*lock);
    output->FillRegion(range.x_range.start, range.x_range.end,
                       range.y_range.start, range.y_range.end,
                       range.z_range.start, range.z_range.end,
                       true);
  } else if (range.size() < FLAGS_recursive_box_split_size) {
    if (input_.pool() != NULL && object_subset.ThreadSafe()) {
      sem->Increment();
      input_.pool()->Add(NewCallback(
          this, &VoxelFill::ProcessRangeParallel,
          print_region, object_subset, range, output, lock, sem));
    } else {
      ProcessRange(print_region, object_subset, range, output, lock);
    }
  } else {
    RangeBox r1, r2, r3, r4, r5, r6, r7, r8;
    range.Split(&r1, &r2, &r3, &r4, &r5, &r6, &r7, &r8);
    ProcessSplit(print_region, object_subset, r1, output, lock, sem);
    ProcessSplit(print_region, object_subset, r2, output, lock, sem);
    ProcessSplit(print_region, object_subset, r3, output, lock, sem);
    ProcessSplit(print_region, object_subset, r4, output, lock, sem);
    ProcessSplit(print_region, object_subset, r5, output, lock, sem);
    ProcessSplit(print_region, object_subset, r6, output, lock, sem);
    ProcessSplit(print_region, object_subset, r7, output, lock, sem);
    ProcessSplit(print_region, object_subset, r8, output, lock, sem);
  }
  LogProgress(output, lock);
}

void VoxelFill::LogProgress(PrintBox* output, Mutex* lock) const {
  // TODO: mutex?
  LOG_EVERY_N(INFO, 10000)
      << "Processed: "
      << static_cast<double>(output->fill_calls()) / output->size() * 100
      << "%";
}

void VoxelFill::ProcessRange(const Box& print_region,
                             const ObjectSet& objects,
                             const RangeBox& range,
                             PrintBox* output,
                             Mutex* lock) const {
  double z = (print_region.bottom().z() +
              input_.vertical_resolution() * range.z_range.start);

  // Fill contents into temporary buffer.
  vector<bool> tmp_contents(range.size());
  vector<float> tmp_iso_contents;
  int next_idx = 0;
  for (int z_idx = range.z_range.start; z_idx < range.z_range.end;
       ++z_idx, z += input_.vertical_resolution()) {
    double y = (print_region.bottom().y() +
                input_.horizontal_resolution() * range.y_range.start);
    for (int y_idx = range.y_range.start; y_idx < range.y_range.end;
         ++y_idx, y += input_.horizontal_resolution()) {
      double x = (print_region.bottom().x() +
                  input_.horizontal_resolution() * range.x_range.start);
      for (int x_idx = range.x_range.start; x_idx < range.x_range.end;
           ++x_idx, x += input_.horizontal_resolution()) {
        float iso = objects.PointISO(Point(x, y, z));
        bool contains = iso > 0;
        tmp_contents[next_idx++] = contains;
        if (contains) {
          tmp_iso_contents.push_back(iso);
        }
      }
    }
  }

  // Now dump into output with lock
  next_idx = 0;
  int next_iso = 0;
  MutexLock l(*lock);
  for (int z = range.z_range.start; z < range.z_range.end; ++z) {
    for (int y = range.y_range.start; y < range.y_range.end; ++y) {
      for (int x = range.x_range.start; x < range.x_range.end; ++x) {
        float val = tmp_contents[next_idx++] ? tmp_iso_contents[next_iso++] : 0;
        /*
          VLOG(4) << "XYZ (" << x << "," << y << "," << z << "): " << val;
        */
        output->SetValue(x, y, z, val);
      }
    }
  }
}

void VoxelFill::ProcessRangeParallel(Box print_region,
                                     ObjectSet objects,
                                     RangeBox range,
                                     PrintBox* output,
                                     Mutex* lock,
                                     Semaphore* sem) const {
  ProcessRange(print_region, objects, range, output, lock);
  sem->Decrement();
  LogProgress(output, lock);
}

Box VoxelFill::GetBox(const Box& print_region, const RangeBox& range) const {
  return Box(Point(print_region.bottom().x() +
                   input_.horizontal_resolution() * range.x_range.start,
                   print_region.bottom().y() +
                   input_.horizontal_resolution() * range.y_range.start,
                   print_region.bottom().z() +
                   input_.vertical_resolution() * range.z_range.start),
             Point(print_region.bottom().x() +
                   input_.horizontal_resolution() * (range.x_range.end - 1),
                   print_region.bottom().y() +
                   input_.horizontal_resolution() * (range.y_range.end - 1),
                   print_region.bottom().z() +
                   input_.vertical_resolution() * (range.z_range.end - 1)));
}

Box VoxelFill::DetermineRegion(const std::vector<PrintObject*>& objects) const {
  Box unioned;
  if (objects.empty() || !objects[0]->BoundingBox(&unioned)) {
    return input_.print_region();
  }
  for (int i = 1; i < objects.size(); ++i) {
    Box next;
    if (!objects[i]->BoundingBox(&next)) {
      return input_.print_region();
    }
    unioned.UnionWith(next);
  }

  // Make sure xyz axis all meet or minimum size.
  if (unioned.size_x() < input_.horizontal_resolution()) {
    unioned.UnionWith(unioned.top() +
                      Point(input_.horizontal_resolution(), 0, 0));
  }
  if (unioned.size_y() < input_.horizontal_resolution()) {
    unioned.UnionWith(unioned.top() +
                      Point(0, input_.horizontal_resolution(), 0));
  }
  if (unioned.size_z() < input_.vertical_resolution()) {
    unioned.UnionWith(unioned.top() +
                      Point(0, 0, input_.vertical_resolution()));
  }

  return input_.print_region().IntersectedWith(unioned);
}

}  // namespace printer
