// Copyright 2015
// Author: Christopher Van Arsdale
//
// This prints a sort of fractal-like object (honestly it looks like a big
// blob with the current settings).
//
// This demonstrates how you can do custom voxel filling by specifying each
// xyz point's value individually.

// (This produces a large object, so I have the default resolution reduced--
// feel free to play with it. It also produces a lot of sweep/triangulation
// errors due to double precision issues that I have yet to track down)
//
// Example:
/*
./mandel --output_stl_file=/tmp/mandel.stl --logtostderr \
         --horizontal_resolution=0.3 --vertical_resolution=0.3
*/

#include "common/base/flags.h"
#include "common/base/init.h"
#include "printer/stl/driver.h"
#include "printer/objects/primitives.h"

DEFINE_string(output_stl_file, "",
              "If set, where to write output stl.");

namespace printer {
namespace {
class MandelBulb : public PrintObject {
 public:
  explicit MandelBulb(Box region)
      : region_(region) {
  }

  virtual float ISOValue(const Point& p) {
    Point c(0.1, 0.2, -0.3);
    Point input = ScalePoint(p);
    double r = input.magnitude();
    double phi = atan(input.y() / input.x());
    double theta = atan(sqrt(input.x()*input.x() + input.y() * input.y()) /
                        input.z());
    int iter = 8;
    double np = iter * phi;
    double nt = iter * theta;
    double nr = pow(r, iter);
    Point out = (Point(sin(nt) * cos(np), sin(nt) * sin(np), cos(nt)) * nr) +
                c;
    return (out.magnitude() < 2) ? (1 - out.magnitude() / 2) : 0;
  }
  virtual bool BoundingBox(Box* b) { return false; }
  virtual bool MayIntersectRegion(const Box& box) { return true; }
  virtual bool FullyContains(const Box& box) { return false; }
  virtual bool ThreadSafe() { return true; }

 private:
  Point ScalePoint(const Point& p) const {
    return Point(ScaleX(p.x()), ScaleY(p.y()), ScaleZ(p.z()));
  }
  double ScaleX(double x) const {
    return (x - region_.bottom().x()) / region_.size_x() * 3.5 - 2.5;
  }
  double ScaleY(double y) const {
    return (y - region_.bottom().y()) / region_.size_y() * 2 - 1;
  }
  double ScaleZ(double z) const {
    return (z - region_.bottom().z()) / region_.size_z() * 2 - 1;
  }
  Box region_;
};

PrintObject* GetObject() {
  return new MandelBulb(Box(Point(0, 0, 0),
                            Point(100, 100, 100)));
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

  // Set up input objects.
  printer::STLDriver driver;
  driver.AddOwnedObject(printer::GetObject());

  // Perform construction.
  driver.ExecuteAndWriteToFile(FLAGS_output_stl_file);

  return 0;
}
