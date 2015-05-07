// Copyright 2015
// Author: Christopher Van Arsdale

#include <iostream>
#include <string>
#include <vector>
#include "printer/execute/driver.h"
#include "printer/stl/format.h"

namespace printer {

class PrintObject;

class STLDriver_Input {
 public:
  STLDriver_Input();
  ~STLDriver_Input() {}

  // Accessors
  const Driver::Input& driver_input() const { return driver_input_; }
  const STLFormatWriter::Input& stl_format_input() const {
    return stl_format_input_;
  }

  // Mutators
  Driver::Input* mutable_driver_input() { return &driver_input_; }
  STLFormatWriter::Input* mutable_stl_format_input() {
    return &stl_format_input_;
  }

 private:
  Driver::Input driver_input_;
  STLFormatWriter::Input stl_format_input_;
};

class STLDriver {
 public:
  typedef STLDriver_Input Input;

  STLDriver();
  explicit STLDriver(const Input& input);
  ~STLDriver();

  // Primary interface:
  void AddOwnedObject(PrintObject* object /* takes ownership */);
  void AddOwnedObjects(const std::vector<PrintObject*>& object);
  void ClearObjects();
  void Execute(std::ostream* out);
  void ExecuteAndWriteToFile(const std::string& filename);

 private:
  Input input_;
  std::vector<PrintObject*> objects_;  // owned.
};

}  // namespace printer
