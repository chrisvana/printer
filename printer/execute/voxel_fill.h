// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_EXECUTE_VOXEL_FILL_H__
#define _PRINTER_EXECUTE_VOXEL_FILL_H__

#include <string>
#include <memory>
#include <mutex>
#include "common/base/macros.h"
#include "printer/base/geometry.h"

namespace thread {
class Semaphore;
class ThreadPool;
}

namespace printer {

class PrintBox;
class PrintObject;

class VoxelFill_Input {
 public:
  VoxelFill_Input();
  ~VoxelFill_Input() {}

  // Raster inputs
  const std::string& print_box_name() const { return print_box_name_; }
  void set_print_box_name(const std::string& name) { print_box_name_ = name; }

  const Box& print_region() const { return print_region_; }
  Box* mutable_print_region() { return &print_region_; }
  void set_print_region(const Box& box) { print_region_ = box; }
  thread::ThreadPool* pool() const { return pool_; }

  double horizontal_resolution() const { return horizontal_resolution_; }
  double vertical_resolution() const { return vertical_resolution_; }
  void set_horizontal_resolution(double h) { horizontal_resolution_ = h; }
  void set_vertical_resolution(double v) { vertical_resolution_ = v; }
  void set_pool(thread::ThreadPool* pool) { pool_ = pool; }

 private:
  std::string print_box_name_;
  double horizontal_resolution_, vertical_resolution_;
  Box print_region_;
  thread::ThreadPool* pool_;
};

class VoxelFill {
 public:
  typedef VoxelFill_Input Input;
  explicit VoxelFill(const Input& input);
  ~VoxelFill();

  PrintBox* Execute(const std::vector<PrintObject*>& objects) const;

 private:
  DISALLOW_COPY_AND_ASSIGN(VoxelFill);

  class ObjectSet;

  // Linear [start, end) range.
  struct Range {
    Range() : start(0), end(0) {}
    Range(int s, int e) : start(s), end(e) {}
    bool Empty() const { return start >= end; }
    int size() const { return end - start; }
    void Split(Range* out1, Range* out2) const {
      int split = (end + start) / 2;
      *out1 = Range(start, split);
      *out2 = Range(split, end);
    }
    int start, end;
  };

  // xyz set of [start, end)^3.
  struct RangeBox {
    RangeBox() {}
    RangeBox(const Range& x, const Range& y, const Range& z)
        : x_range(x), y_range(y), z_range(z) {
    }

    int size() const {
      return x_range.size() * y_range.size() * z_range.size();
    }
    void Split(RangeBox* range1,
               RangeBox* range2,
               RangeBox* range3,
               RangeBox* range4,
               RangeBox* range5,
               RangeBox* range6,
               RangeBox* range7,
               RangeBox* range8) const {
      Range x_split1, x_split2, y_split1, y_split2, z_split1, z_split2;
      x_range.Split(&x_split1, &x_split2);
      y_range.Split(&y_split1, &y_split2);
      z_range.Split(&z_split1, &z_split2);
      *range1 = RangeBox(x_split1, y_split1, z_split1);
      *range2 = RangeBox(x_split1, y_split1, z_split2);
      *range3 = RangeBox(x_split1, y_split2, z_split1);
      *range4 = RangeBox(x_split1, y_split2, z_split2);
      *range5 = RangeBox(x_split2, y_split1, z_split1);
      *range6 = RangeBox(x_split2, y_split1, z_split2);
      *range7 = RangeBox(x_split2, y_split2, z_split1);
      *range8 = RangeBox(x_split2, y_split2, z_split2);
    }
    Range x_range, y_range, z_range;
  };

  void ProcessSplit(const Box& print_region,
                    const ObjectSet& objects,
                    const RangeBox& range,
                    PrintBox* output,
                    std::mutex* lock,
                    thread::Semaphore* sem) const;
  void ProcessRange(const Box& print_region,
                    const ObjectSet& objects,
                    const RangeBox& range,
                    PrintBox* output,
                    std::mutex* lock) const;
  void ProcessRangeParallel(Box print_region,
                            ObjectSet objects,
                            RangeBox range,
                            PrintBox* output,
                            std::mutex* lock,
                            thread::Semaphore* sem) const;
  void LogProgress(PrintBox* output, std::mutex* lock) const;
  Box GetBox(const Box& print_region, const RangeBox& range) const;
  Box DetermineRegion(const std::vector<PrintObject*>& objects) const;

  Input input_;
};

}  // namespace printer

#endif  // _PRINTER_EXECUTE_VOXEL_FILL_H__
