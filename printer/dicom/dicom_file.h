// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_DICOM_DICOM_FILE_H__
#define _PRINTER_DICOM_DICOM_FILE_H__

#include <memory>
#include <mutex>
#include <string>
#include "common/base/types.h"
#include "printer/base/geometry.h"

class DicomImage;
class DcmDataset;
class DcmFileFormat;
class DcmTagKey;

namespace printer {

class DicomFrameInfo {
 public:
  static bool Fill(DcmDataset* dataset,
                   DicomImage* image,
                   int frame,
                   DicomFrameInfo* info);

  DicomFrameInfo() {}
  ~DicomFrameInfo() {}

  // Accessors
  int frame_index() const { return frame_index_; }
  int width() const { return width_; }
  int height() const { return height_; }
  const Point& orientation_x() const { return orientation_x_; }
  const Point& orientation_y() const { return orientation_y_; }
  const Point& pos() const { return pos_; }
  double x_res() const { return x_res_; }
  double y_res() const { return y_res_; }
  const Box& range() const { return range_; }

  // Coordinate system.
  Point x_dir() const { return orientation_y_.Cross(z_dir()).Normalized(); }
  Point y_dir() const { return z_dir().Cross(x_dir()).Normalized(); }
  Point z_dir() const {
    return orientation_x_.Cross(orientation_y_).Normalized();
  }

  // Corners
  Point top_left() const { return PointFromPixels(0, 0); }
  Point top_right() const { return PointFromPixels(width_, 0); }
  Point bottom_left() const { return PointFromPixels(0, height_); }
  Point bottom_right() const { return PointFromPixels(width_, height_); }

  // Translation
  Point PointFromPixels(int pixel_x, int pixel_y) const {
    return (orientation_x_ * pixel_x +
            orientation_y_ * pixel_y +
            pos_);
  }

  std::string DebugString() const;

 private:
  int frame_index_;
  int width_, height_;
  Point orientation_x_, orientation_y_, pos_;
  double x_res_, y_res_;
  Box range_;
};

class DicomPixelData {
 public:
  static DicomPixelData* FromFrame(const DicomFrameInfo& frame,
                                   DicomImage* image);
  ~DicomPixelData() {
    delete [] pixel_data_;
  }

  // Iterating through values.
  void ResetIterator() { x_ = -1; y_ = index_ = 0; }
  size_t x() const { return x_; }
  size_t y() const { return y_; }
  Point point() const { return frame_.PointFromPixels(x_, y_); }
  bool Next(float* val) {
    for (; y_ < frame_.height(); ++y_, x_ = -1) {
      if (++x_ < frame_.width()) {
        *val = Rescale<8>(pixel_data_[index_++]);
        return true;
      }
    }
    return false;
  }

 private:
  DicomPixelData(const DicomFrameInfo& frame)
      : frame_(frame),
        pixel_data_(NULL) {
    ResetIterator();
  }

  template <int bits, typename Input>
  float Rescale(const Input& input) {
    const float max = (1LL << bits) - 1;
    return static_cast<float>(input) / max;
  }

  DicomFrameInfo frame_;
  uint8* pixel_data_;
  size_t x_, y_, index_;
};

class DicomFileInfo {
 public:
  static DicomFileInfo* FromFile(const std::string& file);
  ~DicomFileInfo();

  // Image accessor
  DicomImage* image() const { return image_.get(); }

  // Global info.
  bool GetSliceThickness(double* thick) const;
  int num_frames() const { return total_frames_; }
  bool CheckFrames();  // make sure we can load all of them.

  // Info for particular frames.
  bool GetFrameInfo(int frame_index, DicomFrameInfo* info) const;
  DicomPixelData* NewPixelDataForFrame(int frame_id) const;

 private:
  DicomFileInfo();
  bool GetFrameInfoLocked(int frame_index, DicomFrameInfo* info) const;

  // Syncronization
  typedef std::mutex Mutex;
  typedef std::unique_lock<Mutex> UniqueMutexLock;
  mutable Mutex lock_;

  // Top level info.
  std::unique_ptr<DicomImage> image_;
  std::unique_ptr<DcmFileFormat> format_;
  DcmDataset* dataset_;
  int total_frames_;

  // Frame info
  int current_frame_;
  DicomFrameInfo frame_;
};

}  // namespace printer

#endif  // _PRINTER_DICOM_DICOM_FILE_H__
