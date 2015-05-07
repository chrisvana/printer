// Copyright 2015
// Author: Christopher Van Arsdale

#include <mutex>
#include <memory>
#include <sstream>
#include <string>
#include "common/log/log.h"
#include "dcmtk/dcmimgle/dcmimage.h"
#include "dcmtk/config/osconfig.h" 
#include "dcmtk/dcmdata/dctk.h" 
#include "printer/dicom/dicom_file.h"

using std::string;
using std::unique_ptr;

namespace printer {
namespace {

bool GetNumeric(DcmItem* item,
                const DcmTagKey& key,
                int n, double* out) {
  OFString tmp;
  if (!item->findAndGetOFString(key, tmp, n).good()) {
    return false;
  }

  // TODO: error checking.
  *out = atof(tmp.c_str());
  return true;
}

bool GetFrameNumeric(DcmDataset* dataset,
                     int frame, int total_frames,
                     const DcmTagKey& key,
                     const DcmTagKey& frame_subsequence_key,
                     int n, double* out) {
  if (total_frames == 1 || frame == -1) {
    if (GetNumeric(dataset, key, n, out)) {
      return true;
    }
    frame = 0;
  }

  DcmItem* item = NULL;
  if (!dataset->findAndGetSequenceItem(DCM_PerFrameFunctionalGroupsSequence,
                                       item, frame).good()) {
    LOG(ERROR) << "Could not find frame sequence " << frame;
    return false;
  }
  if (item != NULL && GetNumeric(item, key, n, out)) {
    return true;
  }

  if (!item->findAndGetSequenceItem(frame_subsequence_key,
                                    item, 0).good()) {
    LOG(ERROR) << "Could not find frame subsequence " << frame;
    return false;
  }
  if (item != NULL && GetNumeric(item, key, n, out)) {
    return true;
  }

  // Could not find it anywhere.
  return false;
}

bool GetImagePosition(DcmDataset* dataset, int frame, int total_frames,
                      Point* position) {
  double x, y, z;
  if (GetFrameNumeric(dataset, frame, total_frames,
                      DCM_ImagePositionPatient,
                      DCM_PlanePositionSequence,
                      0, &x) &&
      GetFrameNumeric(dataset, frame, total_frames,
                      DCM_ImagePositionPatient,
                      DCM_PlanePositionSequence,
                      1, &y) &&
      GetFrameNumeric(dataset, frame, total_frames,
                      DCM_ImagePositionPatient,
                      DCM_PlanePositionSequence,
                      2, &z)) {
    *position = Point(x, y, z);
    return true;
  }
  return false;
}

bool GetImageOrientation(DcmDataset* dataset,
                         int frame, int total_frames,
                         Point* x, Point* y) {
  double xx, xy, xz, yx, yy, yz;
  if (GetFrameNumeric(dataset, frame, total_frames,
                      DCM_ImageOrientationPatient,
                      DCM_PlaneOrientationSequence,
                      0, &xx) &&
      GetFrameNumeric(dataset, frame, total_frames,
                      DCM_ImageOrientationPatient,
                      DCM_PlaneOrientationSequence,
                      1, &xy) &&
      GetFrameNumeric(dataset, frame, total_frames,
                      DCM_ImageOrientationPatient,
                      DCM_PlaneOrientationSequence,
                      2, &xz) &&
      GetFrameNumeric(dataset, frame, total_frames,
                      DCM_ImageOrientationPatient,
                      DCM_PlaneOrientationSequence,
                      3, &yx) &&
      GetFrameNumeric(dataset, frame, total_frames,
                      DCM_ImageOrientationPatient,
                      DCM_PlaneOrientationSequence,
                      4, &yy) &&
      GetFrameNumeric(dataset, frame, total_frames,
                      DCM_ImageOrientationPatient,
                      DCM_PlaneOrientationSequence,
                      5, &yz)) {
    *x = Point(xx, xy, xz);
    *y = Point(yx, yy, yz);
    return true;
  }
  return false;
}

bool GetSpacing(DcmDataset* dataset, int frame, int total_frames,
                double* x, double* y) {
  if (GetFrameNumeric(dataset, frame, total_frames,
                      DCM_PixelSpacing, DCM_PixelMeasuresSequence,
                      0, x) &&
      GetFrameNumeric(dataset, frame, total_frames,
                      DCM_PixelSpacing, DCM_PixelMeasuresSequence,
                      1, y)) {
    return true;
  }
  return false;
}

}  // anonymous namespace

// static
bool DicomFrameInfo::Fill(DcmDataset* dataset,
                          DicomImage* image,
                          int frame,
                          DicomFrameInfo* info) {
  int total_frames = image->getFrameCount();
  info->frame_index_ = frame;

  if (!GetImagePosition(dataset, frame, total_frames, &info->pos_)) {
    LOG(ERROR) << "Unable to load Image Position (Patient).";
    return false;
  }

  if (!GetImageOrientation(dataset, frame, total_frames,
                           &info->orientation_x_,
                           &info->orientation_y_)) {
    LOG(ERROR) << "Unable to load Image Orientation (Patient).";
    return false;
  }

  if (!GetSpacing(dataset, frame, total_frames,
                  &info->x_res_,
                  &info->y_res_)) {
    LOG(ERROR) << "Unable to load Image Spacing.";
    return false;
  }

  // Rescale orientation.
  info->orientation_x_ = info->orientation_x_ * info->x_res_;
  info->orientation_y_ = info->orientation_y_ * info->y_res_;

  // TODO: can these vary per frame?
  info->width_ = image->getWidth();
  info->height_ = image->getHeight();
  Box b = Box(info->top_left(), info->bottom_right());
  Box a = Box(info->top_right(), info->bottom_left());
  info->range_ = a.Unioned(b);
  return true;
}

// static
DicomPixelData* DicomPixelData::FromFrame(const DicomFrameInfo& frame,
                                          DicomImage* image) {
  unique_ptr<DicomPixelData> output(new DicomPixelData(frame));
  size_t buffer_size = image->getOutputDataSize(8);
  output->pixel_data_ = new uint8[image->getOutputDataSize(8)];
  if (!image->getOutputData(output->pixel_data_, buffer_size, 8,
                            frame.frame_index())) {
    LOG(ERROR) << "Unable to load pixel data.";
    return NULL;
  }
  return output.release();
}

// static
DicomFileInfo* DicomFileInfo::FromFile(const std::string& file) {
  // Open the file.
  unique_ptr<DicomFileInfo> output(new DicomFileInfo());
  output->image_.reset(new DicomImage(file.c_str()));
  if (output->image_->getStatus() != EIS_Normal) {
    LOG(ERROR) << "Error with file " << file << ": "
               << DicomImage::getString(output->image_->getStatus());
    return NULL;
  }
  output->image_->setMinMaxWindow();  // TODO: necessary?

  // TODO, handle non-monochrome:
  if (!output->image_->isMonochrome()) {
    LOG(ERROR) << "Non-monochrome unsupported.";
    return NULL;
  }

  // Load attributes about image.
  output->format_.reset(new DcmFileFormat());
  if (!output->format_->loadFile(file.c_str()).good()) {
    LOG(ERROR) << "Error with loading file format info.";
    return NULL;
  }
  output->dataset_ = output->format_->getDataset();

  output->total_frames_ = output->image_->getFrameCount();
  output->current_frame_ = -1;
  return output.release();
}

string DicomFrameInfo::DebugString() const {
  std::stringstream out;
  out << "index: " << frame_index_ << std::endl;
  out << "dim: (" << width_ << "," << height_ << ")" << std::endl;
  out << "pos: " << pos_.DebugString() << std::endl;
  out << "x_orient: " << orientation_x_.DebugString() << std::endl;
  out << "y_orient: " << orientation_y_.DebugString() << std::endl;
  out << "xy_res: (" << x_res_ << "," << y_res_ << ")" << std::endl;
  out << "range: " << range_.DebugString() << std::endl;
  return out.str();
}

DicomFileInfo::DicomFileInfo() {
}

DicomFileInfo::~DicomFileInfo() {
}

bool DicomFileInfo::GetFrameInfo(int frame_index, DicomFrameInfo* info) const {
  UniqueMutexLock l(lock_);
  return GetFrameInfoLocked(frame_index, info);
}

bool DicomFileInfo::GetFrameInfoLocked(int frame_index,
                                       DicomFrameInfo* info) const {
  
  return (frame_index < total_frames_ &&
          DicomFrameInfo::Fill(dataset_,
                               image_.get(),
                               frame_index,
                               info));
}

DicomPixelData* DicomFileInfo::NewPixelDataForFrame(int frame_id) const {
  UniqueMutexLock l(lock_);
  DicomFrameInfo frame;
  if (!GetFrameInfoLocked(frame_id, &frame)) {
    return NULL;
  }
  return DicomPixelData::FromFrame(frame, image_.get());
}

bool DicomFileInfo::CheckFrames() {
  UniqueMutexLock l(lock_);
  for (int i = 0; i < total_frames_; ++i) {
    DicomFrameInfo info;
    if (!GetFrameInfoLocked(i, &info)) {
      LOG(ERROR) << "Unable to load frame: " << i;
      return false;
    }
    VLOG(3) << "Loaded frame: " << info.DebugString();
  }
  return true;
}

bool DicomFileInfo::GetSliceThickness(double* thick) const {
  UniqueMutexLock l(lock_);
  return GetFrameNumeric(dataset_,
                         -1, total_frames_,
                         DCM_SliceThickness, DCM_PixelMeasuresSequence,
                         0, thick);
}

}  // namespace printer
