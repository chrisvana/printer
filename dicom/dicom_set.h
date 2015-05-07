// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_DICOM_DICOM_SET_H__
#define _PRINTER_DICOM_DICOM_SET_H__

#include <string>
#include <vector>
#include "printer/objects/point_octree_object.h"

namespace thread {
class ThreadPool;
}
class Closure;

namespace printer {
class PointOctree;
class PrintObject;
class DicomFileInfo;

class DicomPrintObject_Input {
 public:
  DicomPrintObject_Input();
  ~DicomPrintObject_Input() {}

  thread::ThreadPool* pool() const { return pool_; }
  void set_pool(thread::ThreadPool* pool) { pool_ = pool; }

  PointOctreeObject::PointMode point_mode() const { return point_mode_; }
  void set_point_mode(const PointOctreeObject::PointMode m) { point_mode_ = m; }

 private:
  thread::ThreadPool* pool_;
  PointOctreeObject::PointMode point_mode_;
};

class DicomPrintObject {
 public:
  struct PrintInfo {
    PrintInfo();
    ~PrintInfo();

    std::unique_ptr<PrintObject> print_object;
    double horizontal_res, vertical_res;
  };

  // Helpers to simplify calling.
  static PrintInfo* LoadFromFiles(
      const std::vector<std::string>& files);
  static PrintInfo* LoadFromFilesParallel(
      const std::vector<std::string>& files);

  typedef DicomPrintObject_Input Input;
  explicit DicomPrintObject(const Input& input);
  ~DicomPrintObject();

  PrintInfo* Execute(const std::vector<std::string>& files);

 private:
  bool FillFiles(const std::vector<std::string>& files,
                 std::vector<DicomFileInfo*>* infos);
  void FillFile(std::string file /* copy intentional */,
                DicomFileInfo** out,
                Closure* done);

  PointOctree* BuildOctree(const std::vector<DicomFileInfo*>& infos,
                           const Point& x_dir,
                           const Point& y_dir,
                           const Point& z_dir,
                           const Point& origin,
                           const Box& range);
  
  struct BuildOctreeParams {
    BuildOctreeParams(Point x, Point y, Point z, Point o, Box r)
        : x_dir(x), y_dir(y), z_dir(z), origin(o), range(r) {
    }

    Point x_dir;
    Point y_dir;
    Point z_dir;
    Point origin;
    Box range;
  };
  void BuildSingleOctree(DicomFileInfo* infos,
                         BuildOctreeParams params,
                         int frame_id,
                         std::unique_ptr<PointOctree>* octree,
                         Closure* done);

  Input input_;
};

#endif  // _PRINTER_DICOM_DICOM_SET_H__

}  // namespace printer
