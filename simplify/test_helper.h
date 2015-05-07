// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_SIMPLIFY_TEST_HELPER_H__
#define _PRINTER_SIMPLIFY_TEST_HELPER_H__

namespace printer {

class TestHelper {
 public:
  static void GetCylinder(double min_z,
                          double max_z,
                          int step_count,
                          double radius,
                          TriangleMesh* mesh);
  static void GetCylinderDefault(double min_z,
                                 double max_z,
                                 double radius,
                                 TriangleMesh* mesh);
  static void MaybeDump(const TriangleMesh& out);
};

#endif  // _PRINTER_SIMPLIFY_TEST_HELPER_H__

}  // namespace printer
