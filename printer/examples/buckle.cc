// Copyright 2015
// Author: Christopher Van Arsdale
//
// This prints a curved buckle (for straps).
//
// Example:
// ./buckle --output_stl_file=/tmp/buckle.stl --logtostderr

#include "common/base/flags.h"
#include "common/base/init.h"
#include "printer/stl/driver.h"
#include "printer/objects/primitives.h"
#include "printer/objects/transform.h"

DEFINE_string(output_stl_file, "",
              "If set, where to write output stl.");

namespace printer {

PrintObject* GetObject() {
  // Outside Cylinder
  PrintObject* outside = new CylinderObject(Box(Point(0, 0, 0),
                                                Point(80, 80, 50)));

  // Minus Inside Cylinder
  outside = new RemoveObject(outside,
                             new CylinderObject(Box(Point(8, 8, 0),
                                                    Point(72, 72, 50))));
  
  // Minus slits
  outside = new RemoveObject(outside,
                             new RectangleObject(Box(Point(0, 25, 5),
                                                     Point(100, 35, 45))));
  outside = new RemoveObject(outside,
                             new RectangleObject(Box(Point(0, 45, 5),
                                                     Point(100, 55, 45))));

  // Minus inverted cylinders for smoothness.
  PrintObject* inverted_shell = new RemoveObject(
      new CylinderObject(Box(Point(-70, 0, 5), Point(10, 80, 45))),
      new CylinderObject(Box(Point(-65, 5, 5), Point(5, 75, 45))));
  outside = new RemoveObject(outside, inverted_shell);
  inverted_shell = new RemoveObject(
      new CylinderObject(Box(Point(-5, -5, 5), Point(85, 85, 45))),
      new CylinderObject(Box(Point(0, 8, 5), Point(80, 72, 45))));
  outside = new RemoveObject(outside, inverted_shell);

  // Only for box area
  outside = new IntersectObject(outside,
                                new RectangleObject(Box(Point(0, 15, 0),
                                                        Point(50, 65, 50))));

  return outside;
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
