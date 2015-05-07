// Copyright 2015
// Author: Christopher Van Arsdale
//
// This prints a large ring by merging together 200 spheres. It can use a lot
// of memory, so the example command line below reduces the print resolution
// to compensate. Resolution is in millimeters, and defaults to 0.1.
//
// (This takes a while to run, and can produce some errors (mostly due to
// floating point precision) which I have been having trouble tracking down. It
// should recover from the errors and work anyway.)
//
// Example:
/*
./merged --output_stl_file=/tmp/merged.stl --logtostderr \
         --horizontal_resolution=0.3 --vertical_resolution=0.3
*/


#include <vector>
#include "common/base/flags.h"
#include "common/base/init.h"
#include "printer/stl/driver.h"
#include "printer/objects/primitives.h"

DEFINE_string(output_stl_file, "",
              "If set, where to write output stl.");

namespace printer {

std::vector<PrintObject*> GetObjects() {
  const double kPi = 3.14159265357989;

  std::vector<PrintObject*> output;

  double total_mag = 0;
  for (int i = 0; i < 200; ++i) {
    double phase = static_cast<double>(i) / 100.0 * 2 * kPi;
    double magnitude = fabs(cos(phase) * 3 + 10);
    Point center(50 + cos(phase) * 30,
                 50 + sin(phase) * 30,
                 sin(phase/2) * 30);
    PrintObject* obj = new SphereObject(Sphere(center, magnitude));
    output.push_back(obj);
    total_mag += magnitude;
  }
  return output;
}

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
  driver.AddOwnedObjects(printer::GetObjects());

  // Perform construction.
  driver.ExecuteAndWriteToFile(FLAGS_output_stl_file);

  return 0;
}
