// Copyright 2015
// Author: Christopher Van Arsdale
//
// This demonstrates how to print DICOM medical files. I recommend trying your
// first print with downsampled resolution (see --dicom_resolution_scale) to
// see how it looks. If you get no triangles, try modifying the iso levels.
//
// This is very memory/CPU intensive.
//
// (This pulls in a third_party library, dcmtk)
//
// You can find some example dicom files at:
// http://www.insight-journal.org/midas/collection/view/194
//
// Example (downsampled with --dicom_resolution_scale=4)
/*
nice ./print_dicom --input_dicom_files=path/to/Downloads/Tomato/IM_* \
                   --logtostderr \
                   --output_stl_file=/tmp/dicom.stl \
                   --marching_cubes_min_island_size=300 \
                   --driver_run_face_reduction=false \
                   --driver_boundary_is_single_voxel=false \
                   --driver_outer_boundary_iso_level=0.05 \
                   --driver_inner_boundary_iso_level=0.1 \
                   --driver_iso_level=0.15 \
                   --dicom_resolution_scale=4
*/

#include <memory>
#include <string>
#include <vector>
#include "common/base/flags.h"
#include "common/base/init.h"
#include "common/base/types.h"
#include "common/file/fileutil.h"
#include "common/log/log.h"
#include "common/strings/strutil.h"
#include "printer/base/mesh.h"
#include "printer/stl/format.h"
#include "printer/dicom/dicom_set.h"
#include "printer/execute/driver.h"
#include "printer/execute/print_box.h"
#include "printer/objects/object.h"
#include "printer/objects/primitives.h"
#include "printer/objects/transform.h"

using std::string;
using std::vector;
using std::unique_ptr;

using namespace printer;

DEFINE_string(input_dicom_files, "",
              "Glob/list of dicom files.");

DEFINE_string(output_stl_file, "",
              "If set, where to write output stl.");

DEFINE_double(dicom_resolution_scale, 1,
              "Scaling value on resolution relative to DICOM native.");

// Optional way to restrict to a specific subregion.
DEFINE_double(dicom_min_x_value, -1,
              "If >=0, we add a bounding box for this region.");
DEFINE_double(dicom_min_y_value, -1,
              "If >=0, we add a bounding box for this region.");
DEFINE_double(dicom_min_z_value, -1,
              "If >=0, we add a bounding box for this region.");
DEFINE_double(dicom_max_x_value, -1,
              "If >=0, we add a bounding box for this region.");
DEFINE_double(dicom_max_y_value, -1,
              "If >=0, we add a bounding box for this region.");
DEFINE_double(dicom_max_z_value, -1,
              "If >=0, we add a bounding box for this region.");

namespace {
vector<string> GetFileName(const string& pattern) {
  vector<string> splits = strings::SplitString(pattern, ",");
  vector<string> files;
  for (const string& s : splits) {
    size_t start = files.size();
    if (!file::Glob(s, &files) || files.size() == start) {
      LOG(FATAL) << "Could not glob pattern: " << s;
    }
  }
  return files;
}

PrintObject* MaybeAddIntersect(PrintObject* input) {
  if (FLAGS_dicom_min_x_value >= 0 ||
      FLAGS_dicom_min_y_value >= 0 ||
      FLAGS_dicom_min_z_value >= 0 ||
      FLAGS_dicom_max_x_value >= 0 ||
      FLAGS_dicom_max_y_value >= 0 ||
      FLAGS_dicom_min_z_value >= 0) {
    Box bounds(Point(FLAGS_dicom_min_x_value,
                     FLAGS_dicom_min_y_value,
                     FLAGS_dicom_min_z_value),
               Point(FLAGS_dicom_max_x_value,
                     FLAGS_dicom_max_y_value,
                     FLAGS_dicom_max_z_value));
    return new printer::IntersectObject(new printer::RectangleObject(bounds),
                                        input);
  }
  return input;
}

void FillDriverInput(const DicomPrintObject::PrintInfo& info,
                     Driver::Input* input) {
  input->mutable_voxel_input()->set_print_box_name("FloatPrintBox");

  // Compute print region.
  info.print_object->BoundingBox(
      input->mutable_voxel_input()->mutable_print_region());

  // Resolution.
  input->mutable_voxel_input()->set_horizontal_resolution(
      info.horizontal_res * FLAGS_dicom_resolution_scale);
  input->mutable_voxel_input()->set_vertical_resolution(
      info.vertical_res * FLAGS_dicom_resolution_scale);
}

}  // anonymous namespace

int main(int argc, char** argv) {
  InitProgram(&argc, &argv);

  // Set up input DICOM.
  unique_ptr<DicomPrintObject::PrintInfo> input_dicom(
      DicomPrintObject::LoadFromFilesParallel(
          GetFileName(FLAGS_input_dicom_files)));
  if (input_dicom.get() == NULL) {
    LOG(ERROR) << "Could not initialize voxel map from files.";
    return 1;
  }

  // Set up driver.
  Driver::Input input;
  input.set_owns_objects(true);
  FillDriverInput(*input_dicom, &input);
  Driver driver(input);

  // Execute (and delete objects):
  TriangleMesh output;
  {
    vector<PrintObject*> objects;
    objects.push_back(MaybeAddIntersect(input_dicom->print_object.release()));
    driver.Execute(objects, &output);
  }

  // Output.
  if (!FLAGS_output_stl_file.empty()) {
    STLFormatWriter::WriteOrDie(FLAGS_output_stl_file, output);
  }

  return 0;
}
