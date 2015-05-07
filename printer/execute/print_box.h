// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_EXECUTE_PRINT_BOX_H__
#define _PRINTER_EXECUTE_PRINT_BOX_H__

#include <string>
#include <vector>
#include <cstddef>  // size_t
#include "common/base/types.h"
#include "common/base/macros.h"
#include "printer/base/geometry.h"

namespace printer {

class PrintBox {
 public:
  static PrintBox* FromName(
      const std::string& name,
      int x, int y, int z, double horizontal, double vertical);

  PrintBox(int x, int y, int z, double horizontal, double vertical);
  virtual ~PrintBox() {}

  void SetOrigin(const Point& p) { origin_ = p; }

  // Dimension information.
  int size_x() const { return size_x_; }
  int size_y() const { return size_y_; }
  int size_z() const { return size_z_; }
  size_t size() const { return size_x_ * size_y_ * size_z_; }
  double horizontal_resolution() const { return horizontal_; }
  double vertical_resolution() const { return vertical_; }
  Box BoundingBox() const {
    return Box(coord_of(-1, -1, -1),
               coord_of(size_x() + 1, size_y() + 1, size_z() + 1));
  }

  // Integer poisition <=> coordinate position helpers.
  Point coord_of(int x, int y, int z) const {
    return Point(x_coord_of(x), y_coord_of(y), z_coord_of(z));
  }
  double x_coord_of(int x) const { return x * horizontal_ + origin_.x(); }
  double y_coord_of(int y) const { return y * horizontal_ + origin_.y(); }
  double z_coord_of(int z) const { return z * vertical_ + origin_.z(); }

  int box_x_val(double point_x) const {
    return (point_x - origin_.x()) / horizontal_;
  }
  int box_y_val(double point_y) const {
    return (point_y - origin_.y()) / horizontal_;
  }
  int box_z_val(double point_z) const {
    return (point_z - origin_.z()) / vertical_;
  }

  virtual void FillISOBox(int x, int y, int z, float iso[8]) const {
    iso[0] = iso_value(x, y, z);
    iso[1] = iso_value(x + 1, y, z);
    iso[2] = iso_value(x + 1, y + 1, z);
    iso[3] = iso_value(x, y + 1, z);
    iso[4] = iso_value(x, y, z + 1);
    iso[5] = iso_value(x + 1, y, z + 1);
    iso[6] = iso_value(x + 1, y + 1, z + 1);
    iso[7] = iso_value(x, y + 1, z + 1);
  }

  void FillPointBox(int x, int y, int z, Point points[8]) const {
    double x0 = x_coord_of(x), x1 = x_coord_of(x + 1);
    double y0 = y_coord_of(y), y1 = y_coord_of(y + 1);
    double z0 = z_coord_of(z), z1 = z_coord_of(z + 1);
    points[0] = Point(x0, y0, z0);
    points[1] = Point(x1, y0, z0);
    points[2] = Point(x1, y1, z0);
    points[3] = Point(x0, y1, z0);
    points[4] = Point(x0, y0, z1);
    points[5] = Point(x1, y0, z1);
    points[6] = Point(x1, y1, z1);
    points[7] = Point(x0, y1, z1);
  }

  // Get.
  virtual size_t fill_calls() const = 0;
  bool is_set(int x, int y, int z) const { return iso_value(x, y, z) > 0; }
  virtual float iso_value(int x, int y, int z) const = 0;

  // Set.
  virtual void SetValue(int x, int y, int z, float val) = 0;
  virtual void FillRegion(int x_start, int x_end,
                          int y_start, int y_end,
                          int z_start, int z_end,
                          float value) {
    for (int z = z_start; z < z_end; ++z) {
      for (int y = y_start; y < y_end; ++y) {
        for (int x = x_start; x < x_end; ++x) {
          SetValue(x, y, z, value);
        }
      }
    }
  }

  class Iterator {
   public:
    Iterator(const PrintBox* parent);
    ~Iterator() {}

    bool Next();
    bool NextSet();
    bool Seek(int x, int y, int z);
    bool SeekAndSet(int x, int y, int z) {
      return Seek(x, y, z) && is_set();
    }

    bool is_set() const { return iso_value_ > 0; }
    float iso_value() const { return iso_value_; }
    int x() const { return x_; }
    int y() const { return y_; }
    int z() const { return z_; }

   private:
    float iso_value_;
    int x_, y_, z_;

    const PrintBox* parent_;
  };
  virtual Iterator NewIterator() const {
    return Iterator(this);
  }

 protected:
  bool is_valid_pos(int x, int y, int z) const {
    return (x < size_x() && y < size_y() && z < size_z() &&
            x >= 0 && y >= 0 && z >= 0);
  }
  int position_to_index(int x, int y, int z) const {
    return is_valid_pos(x, y, z) ? x + size_x() * (y + size_y() * z) : -1;
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(PrintBox);

  int size_x_, size_y_, size_z_;
  double horizontal_, vertical_;
  Point origin_;
};

class BinaryPrintBox : public PrintBox {
 public:
  BinaryPrintBox(int x, int y, int z, double horizontal, double vertical);
  ~BinaryPrintBox() {}

  size_t fill_calls() const { return fill_calls_; }
  float iso_value(int x, int y, int z) const {
    int index = position_to_index(x, y, z);
    return (index >= 0 && bits_[index] ? 1 : 0);
  }

  // TODO(cvanarsdale): Switch to tree of blocks, use octree style compression?
  void SetValue(int x, int y, int z, float value) {
    int index = position_to_index(x, y, z);
    if (index >= 0) {
      bits_[position_to_index(x, y, z)] = (value > 0);
      ++fill_calls_;
    }
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(BinaryPrintBox);

  std::vector<bool> bits_;
  size_t fill_calls_;
};

class FloatPrintBox : public PrintBox {
 public:
  FloatPrintBox(int x, int y, int z, double horizontal, double vertical);
  ~FloatPrintBox() {}

  size_t fill_calls() const { return fill_calls_; }
  float iso_value(int x, int y, int z) const {
    int index = position_to_index(x, y, z);
    return (index >= 0 ? values_[index] : 0);
  }

  // TODO(cvanarsdale): Switch to tree of blocks, use octree style compression?
  void SetValue(int x, int y, int z, float value) {
    int index = position_to_index(x, y, z);
    if (index >= 0) {
      values_[index] = value;
      ++fill_calls_;
    }
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(FloatPrintBox);

  std::vector<float> values_;
  size_t fill_calls_;
};

class Uint8PrintBox : public PrintBox {
 public:
  Uint8PrintBox(int x, int y, int z, double horizontal, double vertical);
  ~Uint8PrintBox() {}

  size_t fill_calls() const { return fill_calls_; }
  float iso_value(int x, int y, int z) const {
    int index = position_to_index(x, y, z);
    return (index >= 0 ? FromUint(values_[index]) : 0);
  }

  // TODO(cvanarsdale): Switch to tree of blocks, use octree style compression?
  void SetValue(int x, int y, int z, float value) {
    int index = position_to_index(x, y, z);
    if (index >= 0) {
      values_[index] = ToUint(value);
      ++fill_calls_;
    }
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(Uint8PrintBox);

  static float FromUint(uint8 input) {
    const float max = (1 << 8);
    return static_cast<float>(input) / max;
  }
  static uint8 ToUint(float input) {
    return static_cast<uint8>(input * 256);
  }

  std::vector<uint8> values_;
  size_t fill_calls_;
};

}  // namespace printer

#endif  // _PRINTER_EXECUTE_PRINT_BOX_H__
