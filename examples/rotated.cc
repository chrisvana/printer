// Copyright 2015
// Author: Christopher Van Arsdale
//
// This demonstrates simple rotation transformation. It prints one cube
// with its corner sitting atop another cube.
//
// Example:
/*
./rotated --output_stl_file=/tmp/cubes.stl --logtostderr \
         --horizontal_resolution=0.2 --vertical_resolution=0.2
*/

#include <vector>
#include "common/base/flags.h"
#include "common/base/init.h"
#include "printer/stl/driver.h"
#include "printer/objects/primitives.h"
#include "printer/objects/transform.h"

DEFINE_string(output_stl_file, "",
              "If set, where to write output stl.");

namespace printer {

std::vector<PrintObject*> GetObjects() {
  const double kPi = 3.14159265357989;

  Box dimensions(Point(0, 0, 0), Point(20, 20, 20));

  std::vector<PrintObject*> output;
  output.push_back(new RectangleObject(dimensions));
  output.push_back(new TransformObject(
      new TransformObject(
          new RectangleObject(dimensions),
          new RotateTransform(Point(10, 10, 10), Point(1, 1, 1),
                              kPi / 4)),
      new TranslateTransform(Point(0, 0, 15))));
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
