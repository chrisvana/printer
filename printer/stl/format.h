// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_STL_FORMAT_H__
#define _PRINTER_STL_FORMAT_H__

#include <iostream>
#include <string>
#include <vector>
#include "common/base/macros.h"

namespace printer {

class PrintBox;
class TriangleMesh;

class STLFormatWriter_Input {
 public:
  explicit STLFormatWriter_Input(std::ostream* output);
  ~STLFormatWriter_Input() {}

  // Output.
  std::ostream* output() const { return output_; }
  void set_output(std::ostream* output) { output_ = output; }

  // Parameters
  bool binary_format() const { return binary_format_; }
  void set_binary_format(bool binary) { binary_format_ = binary; }

 private:
  std::ostream* output_;
  bool binary_format_;
};

class STLFormatWriter {
 public:
  static void WriteOrDie(const std::string& filename, const TriangleMesh& mesh);

  typedef STLFormatWriter_Input Input;
  explicit STLFormatWriter(const Input& input);
  ~STLFormatWriter();

  void WriteMesh(const std::string& obj_name, const TriangleMesh& mesh);

 private:
  DISALLOW_COPY_AND_ASSIGN(STLFormatWriter);

  void StartObject(const std::string& name, int triangle_count);
  void EndObject(const std::string& name);
  void PrintTriangle(const Triangle& triangle);

  Input input_;
};

class STLFormatReader_Input {
 public:
  explicit STLFormatReader_Input(std::istream* input);
  ~STLFormatReader_Input() {}

  // Input.
  std::istream* input() const { return input_; }
  void set_input(std::istream* input) { input_ = input; }

 private:
  std::istream* input_;
};

class STLFormatReader {
 public:
  typedef STLFormatReader_Input Input;
  explicit STLFormatReader(const Input& input);
  ~STLFormatReader();

  bool ReadMesh(TriangleMesh* mesh);

 private:
  DISALLOW_COPY_AND_ASSIGN(STLFormatReader);

  Input input_;
};

}  // namespace printer

#endif  // _PRINTER_STL_FORMAT_H__
