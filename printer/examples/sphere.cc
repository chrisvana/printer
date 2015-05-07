// Copyright 2015
// Author: Christopher Van Arsdale
//
// This prints a simple sphere.
//
// Example:
// ./sphere --output_stl_file=/tmp/sphere.stl --logtostderr

#include "common/base/flags.h"
#include "common/base/init.h"
#include "printer/stl/driver.h"
#include "printer/objects/primitives.h"

DEFINE_string(output_stl_file, "",
              "If set, where to write output stl.");

namespace printer {

PrintObject* GetObject() {
  return new SphereObject(Sphere(Point(40, 40, 40), 10));
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
  driver.AddOwnedObject(printer::GetObject());

  // Perform construction.
  driver.ExecuteAndWriteToFile(FLAGS_output_stl_file);

  return 0;
}
