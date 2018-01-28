// Copyright 2015
// Author: Christopher Van Arsdale
//
// Example:
// ./barrel --output_stl_file=/tmp/out.stl --logtostderr

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

  double outer_radius = 15.0;  // 1.5 cm
  double height = 50;
  Box dimensions(Point(-outer_radius, -outer_radius, 0),
                 Point(outer_radius, outer_radius, height));

  // First cylinder, vertical:
  PrintObject* a = new CylinderObject(dimensions);

  // Center shaft:
  double center_shaft = 6.35;
  double size = center_shaft*2.4;
  PrintObject* shaft1 = new CylinderObject(Box(Point(-size/2, -size/2, 0),
                                               Point(size/2, size/2, height)));
  PrintObject* b = new RemoveObject(a, shaft1);

  // Shaft 2:
  size = center_shaft*1.4;
  PrintObject* shaft2 = new CylinderObject(Box(Point(-size/2, -size/2, 0),
                                               Point(size/2, size/2, height)));
  size = center_shaft*1.1;
  PrintObject* shaft3 = new CylinderObject(Box(Point(-size/2, -size/2, 0),
                                               Point(size/2, size/2, height)));
  PrintObject* c = new RemoveObject(shaft2, shaft3);

  PrintObject* d = new UnionObject(b, c);
  for (int i = 0; i < 10; ++i) {
    double thickness = 1.0;  // mm
    PrintObject* support = new RectangleObject(Box(Point(center_shaft*1.2/2, -thickness/2, 0),
                                                   Point(center_shaft*2.5/2, thickness/2, height)));
    support = new TransformObject(support, new RotateTransform(Point(0, 0, 1), 2*kPi*i/10));
    d = new UnionObject(d, support);
  }

  PrintObject* out = new TransformObject(
      d,
      new TranslateTransform(Point(outer_radius, outer_radius, 0)));
  return out;
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
