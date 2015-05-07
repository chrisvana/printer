// Copyright 2015
// Author: Christopher Van Arsdale
//
// This prints the intersection of two cylinders.
//
// Example:
// ./cylinder --output_stl_file=/tmp/cylinder.stl --logtostderr

#include "common/base/flags.h"
#include "common/base/init.h"
#include "printer/stl/driver.h"
#include "printer/objects/primitives.h"
#include "printer/objects/transform.h"

DEFINE_string(output_stl_file, "",
              "If set, where to write output stl.");

namespace printer {

PrintObject* GetObject() {
  const double kPi = 3.14159265357989;

  Box dimensions(Point(0, 0, 0), Point(10, 10, 20));

  // First cylinder, vertical:
  PrintObject* a = new CylinderObject(dimensions);

  // Second cylinder, horizontal (rotated around x axis).
  PrintObject* b = new TransformObject(
      new CylinderObject(dimensions),
      new RotateTransform(dimensions.center(),
                          Point(1, 0, 0),
                          kPi / 2));

  return new IntersectObject(a, b);
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
