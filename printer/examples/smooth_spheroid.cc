// Copyright 2015
// Author: Christopher Van Arsdale
//
// This prints a smooth sphere if FloatPrintBox is enabled.

#include "common/base/flags.h"
#include "common/base/init.h"
#include "printer/stl/driver.h"
#include "printer/objects/primitives.h"

DEFINE_string(output_stl_file, "",
              "If set, where to write output stl.");

namespace printer {
namespace {
class IsoSpheroid : public PrintObject {
 public:
  explicit IsoSpheroid(Box region)
      : region_(region),
        center_(region.center()),
        dx2_(region.size_x() * region.size_x() / 4),
        dy2_(region.size_y() * region.size_y() / 4),
        dz2_(region.size_z() * region.size_z() / 4) {
  }

  virtual float ISOValue(const Point& p) {
    Point input = (p - region_.center());
    double x2 = input.x() * input.x() / dx2_;
    double y2 = input.y() * input.y() / dy2_;
    double z2 = input.z() * input.z() / dz2_;
    return std::max(0.0, 1.0 - (x2 + y2 + z2));
  }
  virtual bool BoundingBox(Box* b) {
    *b = region_;
    return true;
  }
  virtual bool FullyContains(const Box& box) { return false; }
  virtual bool ThreadSafe() { return true; }

 private:
  Box region_;
  Point center_;
  double dx2_, dy2_, dz2_;
};

PrintObject* GetObject() {
  return new IsoSpheroid(Box(Point(0, 0, 0),
                             Point(30, 30, 20)));
}

}  // anonymous namespace
}  // namespace printer

int main(int argc, char** argv) {
  InitProgram(&argc, &argv);

  // Make sure we have a specified output file.
  if (FLAGS_output_stl_file.empty()) {
    std::cerr << "Please specify --output_stl_file." << std::endl;
    return 1;
  }

  // Set up driver.
  printer::STLDriver::Input input;
  input.mutable_driver_input()->mutable_voxel_input()->set_print_box_name(
      "FloatPrintBox");
  printer::STLDriver driver(input);
  driver.AddOwnedObject(printer::GetObject());

  // Perform construction.
  driver.ExecuteAndWriteToFile(FLAGS_output_stl_file);

  return 0;
}
