// Copyright 2015
// Author: Christopher Van Arsdale

#include <string>
#include <vector>
#include "common/log/log.h"
#include "printer/execute/print_box.h"

namespace printer {

// static
PrintBox* PrintBox::FromName(
    const std::string& name,
    int x, int y, int z, double horizontal, double vertical) {
  if (name == "BinaryPrintBox") {
    return new BinaryPrintBox(x, y, z, horizontal, vertical);
  }
  if (name == "FloatPrintBox") {
    return new FloatPrintBox(x, y, z, horizontal, vertical);
  }
  if (name == "Uint8PrintBox") {
    return new Uint8PrintBox(x, y, z, horizontal, vertical);
  }
  LOG(FATAL) << "Unknown print box name: " << name;
  return NULL;
}

PrintBox::PrintBox(int x, int y, int z, double horizontal, double vertical)
    : size_x_(x),
      size_y_(y),
      size_z_(z),
      horizontal_(horizontal),
      vertical_(vertical) {
}

PrintBox::Iterator::Iterator(const PrintBox* parent)
    : iso_value_(0),
      x_(-1),
      y_(0),
      z_(0),
      parent_(parent) {
}

bool PrintBox::Iterator::NextSet() {
  ++x_;
  for (; z_ < parent_->size_z(); ++z_) {
    for (; y_ < parent_->size_y(); ++y_) {
      for (; x_ < parent_->size_x(); ++x_) {
        iso_value_ = parent_->iso_value(x_, y_, z_);
        if (iso_value_ > 0) {
          return true;
        }
      }
      x_ = 0;
    }
    y_ = 0;
  }
  return false;
}

bool PrintBox::Iterator::Next() {
  if (++x_ >= parent_->size_x()) {
    x_ = 0;
    if (++y_ >= parent_->size_y()) {
      y_ = 0;
      ++z_;
    }
  }
  if (z_ < parent_->size_z()) {
    iso_value_ = parent_->iso_value(x_, y_, z_);
    return true;
  }
  return false;
}

bool PrintBox::Iterator::Seek(int x, int y, int z) {
  if (x < 0 || y < 0 || z < 0 ||
      x >= parent_->size_x() ||
      y >= parent_->size_y() ||
      z >= parent_->size_z()) {
    return false;
  }
  x_ = x;
  y_ = y;
  z_ = z;
  iso_value_ = parent_->iso_value(x_, y_, z_);
  return true;
}

BinaryPrintBox::BinaryPrintBox(int x, int y, int z,
                               double horizontal, double vertical)
    : PrintBox(x, y, z, horizontal, vertical),
      fill_calls_(0) {
  size_t total_size = (static_cast<size_t>(x) *
                       static_cast<size_t>(y) *
                       static_cast<size_t>(z));
  // TODO, naive non-encoding is stupid, use an interval set?
  bits_.resize(total_size);
}

FloatPrintBox::FloatPrintBox(int x, int y, int z,
                             double horizontal, double vertical)
    : PrintBox(x, y, z, horizontal, vertical),
      fill_calls_(0) {
  size_t total_size = (static_cast<size_t>(x) *
                       static_cast<size_t>(y) *
                       static_cast<size_t>(z));
  // TODO, naive non-encoding is stupid, use an interval set?
  values_.resize(total_size);
}

Uint8PrintBox::Uint8PrintBox(int x, int y, int z,
                             double horizontal, double vertical)
    : PrintBox(x, y, z, horizontal, vertical),
      fill_calls_(0) {
  size_t total_size = (static_cast<size_t>(x) *
                       static_cast<size_t>(y) *
                       static_cast<size_t>(z));
  // TODO, naive non-encoding is stupid, use an interval set?
  values_.resize(total_size);
}

}  // namespace printer
