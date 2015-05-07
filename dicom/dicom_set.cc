// Copyright 2015
// Author: Christopher Van Arsdale

#include <memory>
#include <string>
#include <vector>
#include "common/base/callback.h"
#include "common/base/flags.h"
#include "common/log/log.h"
#include "common/thread/counter.h"
#include "common/thread/threadpool.h"
#include "common/util/stl.h"
#include "printer/base/geometry.h"
#include "printer/base/point_octree.h"
#include "printer/dicom/dicom_file.h"
#include "printer/dicom/dicom_set.h"
#include "printer/objects/point_octree_object.h"

DEFINE_string(dicom_set_point_mode, "INTERPOLATE",
             "Mode to use when reconstructing voxel values. See "
             "PointList::FromString for values.");

using std::string;
using std::vector;
using std::unique_ptr;

namespace printer {
namespace {

bool GetCoordinateSystem(const vector<DicomFileInfo*>& infos,
                         Point* x, Point* y, Point* z,
                         double* horizontal_res,
                         double* vertical_res) {
  const DicomFileInfo* primary = infos[0];
  DicomFrameInfo frame;
  if (!primary->GetFrameInfo(0, &frame)) {
    LOG(ERROR) << "Cannot get frame 0 info.";
    return false;
  }

  // Coordinate system (somewhat arbitrarily chosen, but hopefully corresponds
  // well to our images).
  Point z_dir = *z = frame.z_dir();
  *x = frame.x_dir();
  *y = frame.y_dir();

  // Horizontal resolution is based on the pixel spacing, which we have to have.
  *horizontal_res = std::min(frame.x_res(), frame.y_res());

  // Vertical resolution is based on the slice spacing, and we fall back on
  // looking at the total number of slices in the voxel space.
  if (!primary->GetSliceThickness(vertical_res)) {
    // Vertical resolution is based on the slice spacing. We compute the z
    // range of our slices, and then determine the average slice distance.
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::min();
    int total_slices = 0;
    for (const DicomFileInfo* info : infos) {
      total_slices += info->num_frames();
      for (int i = 0; i < info->num_frames(); ++i) {
        DicomFrameInfo frame;
        if (!info->GetFrameInfo(i, &frame)) {
          LOG(ERROR) << "Unable to load frame: " << i;
          return false;
        }

        min_z = std::min(min_z, z_dir * frame.top_left());
        min_z = std::min(min_z, z_dir * frame.top_right());
        min_z = std::min(min_z, z_dir * frame.bottom_right());
        min_z = std::min(min_z, z_dir * frame.bottom_left());
        max_z = std::max(max_z, z_dir * frame.top_left());
        max_z = std::max(max_z, z_dir * frame.top_right());
        max_z = std::max(max_z, z_dir * frame.bottom_right());
        max_z = std::max(max_z, z_dir * frame.bottom_left());
      }
    }
    *vertical_res = (max_z - min_z) / total_slices;
  }

  return true;
}

Point TranslatePoint(const Point& x,
                     const Point& y,
                     const Point& z,
                     const Point& input) {
  // Thankfully xyz are orthogonal:
  return Point(x * input, y * input, z * input);
}

Box GetRange(const vector<DicomFileInfo*>& infos,
             const Point& x,
             const Point& y,
             const Point& z) {
  Box range;
  for (int i = 0; i < infos.size(); ++i) {
    for (int j = 0; j < infos[i]->num_frames(); ++j) {
      DicomFrameInfo frame;
      CHECK(infos[i]->GetFrameInfo(j, &frame));
      Box test = Box(TranslatePoint(x, y, z, frame.top_left()),
                     TranslatePoint(x, y, z, frame.bottom_right()));
      test.UnionWith(Box(TranslatePoint(x, y, z, frame.top_right()),
                         TranslatePoint(x, y, z, frame.bottom_left())));
      if (i == 0 && j == 0) {
        range = test;
      } else {
        range.UnionWith(test);
      }
    }
  }
  return range;
}

}  // anonymous namespace

DicomPrintObject_Input::DicomPrintObject_Input()
    : pool_(NULL),
      point_mode_(PointOctreeObject::FromString(FLAGS_dicom_set_point_mode)) {
}

DicomPrintObject::PrintInfo::PrintInfo() {
}

DicomPrintObject::PrintInfo::~PrintInfo() {
}

// static
DicomPrintObject::PrintInfo* DicomPrintObject::LoadFromFiles(
    const vector<string>& filenames) {
  DicomPrintObject::Input input;
  DicomPrintObject printer(input);
  return printer.Execute(filenames);
}

// static
DicomPrintObject::PrintInfo* DicomPrintObject::LoadFromFilesParallel(
    const vector<string>& filenames) {
  thread::ThreadPool pool;
  pool.StartWorkers();
  DicomPrintObject::Input input;
  input.set_pool(&pool);
  DicomPrintObject printer(input);
  return printer.Execute(filenames);
}


DicomPrintObject::DicomPrintObject(const Input& input)
    : input_(input) {
}

DicomPrintObject::~DicomPrintObject() {}

DicomPrintObject::PrintInfo* DicomPrintObject::Execute(
    const vector<string>& filenames) {
  if (filenames.empty()) {
    LOG(ERROR) << "No filenames specified.";
    return NULL;
  }

  LOG(INFO) << "Loading " << filenames.size() << " DICOM files.";
  vector<DicomFileInfo*> infos;
  ElementDeleter del(&infos);  // deletes when we go out of scope.
  if (!FillFiles(filenames, &infos)) {
    return NULL;
  }

  // Decide on xyz direction:
  Point x_dir, y_dir, z_dir;
  double horizontal_resolution, vertical_resolution;
  if (!GetCoordinateSystem(infos, &x_dir, &y_dir, &z_dir,
                           &horizontal_resolution, &vertical_resolution)) {
    return NULL;
  }
  VLOG(1) << "X-dir: " << x_dir.DebugString();
  VLOG(1) << "Y-dir: " << y_dir.DebugString();
  VLOG(1) << "Z-dir: " << z_dir.DebugString();
  VLOG(1) << "horizontal_resolution: " << horizontal_resolution;
  VLOG(1) << "vertical_resolution: " << vertical_resolution;

  //  Find some offset that puts everything into a positive print region (find
  //  extreme xyz from original points).
  Box range = GetRange(infos, x_dir, y_dir, z_dir);
  VLOG(1) << "Original range: " << range.DebugString();
  Point origin = range.bottom();
  range = range - origin;

  // Build octree map so we can easily look up values in a region.
  LOG(INFO) << "Building voxel map.";
  VLOG(1) << "Shifted range: " << range.DebugString();
  // TODO: parallelize and merge trees.
  unique_ptr<PointOctree> octree(BuildOctree(infos, x_dir, y_dir, z_dir,
                                             origin, range));
  if (octree == NULL) {
    return NULL;
  }
  VLOG(1) << "Size of points: " << octree->size();

  unique_ptr<PrintInfo> output(new PrintInfo);
  output->horizontal_res = horizontal_resolution/2;
  output->vertical_res = vertical_resolution/2;
  output->print_object.reset(new PointOctreeObject(
      horizontal_resolution,
      vertical_resolution,
      input_.point_mode(),
      octree.release()));
  return output.release();
}

bool DicomPrintObject::FillFiles(const std::vector<std::string>& files,
                                 std::vector<DicomFileInfo*>* infos) {
  infos->resize(files.size(), NULL);
  if (input_.pool() == NULL) {
    for (int i = 0; i < files.size(); ++i) {
      FillFile(files[i], &(*infos)[i], NULL);
      if ((*infos)[i] == NULL) {
        return false;
      }
    }
    return true;
  }

  // Parallel version.
  thread::BlockingCounter counter(files.size());
  for (int i = 0; i < files.size(); ++i) {
    input_.pool()->Add(NewCallback(
        this, &DicomPrintObject::FillFile,
        files[i], &(*infos)[i],
        NewCallback(&counter, &thread::BlockingCounter::Decrement)));
  }
  counter.Wait();
  for (const DicomFileInfo* info : *infos) {
    if (info == NULL) {
      return false;
    }
  }

  return true;
}

void DicomPrintObject::FillFile(string file,
                                DicomFileInfo** output,
                                Closure* done) {
  AutoClosureRunner finished(done);

  VLOG(1) << "Opening file: " << file;
  unique_ptr<DicomFileInfo> info(DicomFileInfo::FromFile(file));
  if (info.get() == NULL) {
    return;
  }

  if (!info->CheckFrames()) {
    LOG(ERROR) << "Unable to load some frame for file: " << file;
    return;
  }

  *output = info.release();
}

PointOctree* DicomPrintObject::BuildOctree(
    const vector<DicomFileInfo*>& infos,
    const Point& x_dir,
    const Point& y_dir,
    const Point& z_dir,
    const Point& origin,
    const Box& range) {
  unique_ptr<PointOctree> octree(new PointOctree(range));

  VLOG(1) << "Building output octrees.";
  if (input_.pool() == NULL) {
    for (DicomFileInfo* info : infos) {
      for (int i = 0; i < info->num_frames(); ++i){ 
        BuildSingleOctree(info,
                          BuildOctreeParams(x_dir, y_dir, z_dir, origin,
                                            octree->range()),
                          i, &octree, NULL);
        if (octree == NULL) {
          return NULL;
        }
      }
    }
    return octree.release();
  }

  // Parallel version.
  int total_slices = 0;
  for (const DicomFileInfo* info : infos) {
    total_slices += info->num_frames();
  }

  thread::BlockingCounter counter(total_slices);
  vector<unique_ptr<PointOctree>> octrees(total_slices);
  int next = 0;
  for (DicomFileInfo* info : infos) {
    for (int i = 0; i < info->num_frames(); ++i) {
      unique_ptr<PointOctree>* out = &octrees[next++];
      input_.pool()->Add(NewCallback(
          this, &DicomPrintObject::BuildSingleOctree,
          info,
          BuildOctreeParams(x_dir, y_dir, z_dir, origin,
                            octree->range()),
          i, out,
          NewCallback(&counter, &thread::BlockingCounter::Decrement)));
    }
  }
  counter.Wait();

  // Merge and return
  VLOG(1) << "Merging output octrees.";
  for (int i = 0; i < octrees.size(); ++i) {
    CHECK(octree->Merge(octrees[i].release()));
  }
  return octree.release();
}

void DicomPrintObject::BuildSingleOctree(DicomFileInfo* info,
                                         BuildOctreeParams params,
                                         int frame_id,
                                         unique_ptr<PointOctree>* output,
                                         Closure* done) {
  AutoClosureRunner finished(done);

  unique_ptr<PointOctree> octree(output->release());
  if (octree == NULL) {
    octree.reset(new PointOctree(params.range));
  }

  unique_ptr<DicomPixelData> pixels(info->NewPixelDataForFrame(frame_id));
  if (pixels.get() == NULL) {
    LOG(ERROR) << "Unable to load pixel data, frame: " << frame_id;
    return;
  }

  float val;
  while (pixels->Next(&val)) {
    Point p = TranslatePoint(params.x_dir,
                             params.y_dir,
                             params.z_dir, pixels->point()) - params.origin;
    octree->SetPoint(p, val);
  }

  output->reset(octree.release());
}

}  // namespace printer
