// Copyright 2015
// Author: Christopher Van Arsdale

#include <cmath>
#include <fstream>
#include "common/base/flags.h"
#include "common/log/log.h"
#include "printer/base/mesh.h"
#include "printer/simplify/test_helper.h"
#include "printer/stl/format.h"

DEFINE_string(output_stl_file, "",
              "If non-empty, file to dump.");

DEFINE_int32(build_mesh_z_steps, 4,
             "Number of z steps to use in building mesh for self test.");

DEFINE_int32(build_mesh_xy_steps, 100,
             "Number of z steps to use in building mesh for self test.");

namespace printer {
namespace {
Point Clipped(double x, double y, double z) {
  return Point(static_cast<double>(static_cast<int>(x * 1000)) / 1000,
               static_cast<double>(static_cast<int>(y * 1000)) / 1000,
               static_cast<double>(static_cast<int>(z * 1000)) / 1000);
}
}  // anonymous namespace

// static
void TestHelper::GetCylinderDefault(double min_z,
                                    double max_z,
                                    double radius,
                                    TriangleMesh* mesh) {
  GetCylinder(min_z, max_z, FLAGS_build_mesh_z_steps, radius, mesh);
}

// static
void TestHelper::GetCylinder(double min_z,
                             double max_z,
                             int step_count,
                             double radius,
                             TriangleMesh* mesh) {
  const double kPi = 3.14159265357989;
  const int points = FLAGS_build_mesh_xy_steps;
  for (int i = 0; i < step_count; ++i) {
    double z = ((max_z - min_z) / step_count) * i + min_z;
    double next_z = ((max_z - min_z) / step_count) * (i + 1) + min_z;
    for (int j = 0; j < points; ++j) {
      double rad = (2 * kPi * j) / points;
      double next_rad = (2 * kPi * (j + 1)) / points;
      if (j == points - 1) {
        next_rad = 0;  // for double rounding issues.
      }
      double x = cos(rad) * radius;
      double y = sin(rad) * radius;
      double next_x = cos(next_rad) * radius;
      double next_y = sin(next_rad) * radius;
      CHECK(mesh->AddTriangle(
          Triangle(Clipped(x, y, z),
                   Clipped(next_x, next_y, z),
                   Clipped(next_x, next_y, next_z))));
      CHECK(mesh->AddTriangle(
          Triangle(Clipped(x, y, z),
                   Clipped(next_x, next_y, next_z),
                   Clipped(x, y, next_z))));
      if (i == step_count - 1) {
        // Top
        CHECK(mesh->AddTriangle(
            Triangle(Clipped(0, 0, next_z),
                     Clipped(x, y, next_z),
                     Clipped(next_x, next_y, next_z))));
      }

      // Bottom
      if (i == 0) {
        CHECK(mesh->AddTriangle(
            Triangle(Clipped(0, 0, z),
                     Clipped(next_x, next_y, z),
                     Clipped(x, y, z))));
      }
    }
  }
}

// static
void TestHelper::MaybeDump(const TriangleMesh& out) {
  if (!FLAGS_output_stl_file.empty()) {
    std::ofstream output_file(FLAGS_output_stl_file.c_str());
    CHECK(output_file.good()) << "Unable to open: " << FLAGS_output_stl_file;
    STLFormatWriter_Input input(&output_file);
    STLFormatWriter writer(input);
    LOG(INFO) << "Writing to " << FLAGS_output_stl_file << " with "
              << out.size() << " triangles.";
    writer.WriteMesh("test_helper", out);
  }
}

}  // namespace printer
