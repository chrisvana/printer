// Copyright 2015
// Author: Christopher Van Arsdale

#include <iostream>
#include <fstream>
#include <vector>
#include "common/log/log.h"
#include "common/util/stl.h"
#include "printer/base/mesh.h"
#include "printer/execute/driver.h"
#include "printer/stl/driver.h"
#include "printer/stl/format.h"
#include "printer/objects/object.h"

using std::string;
using std::vector;
using std::unique_ptr;
using std::ostream;

namespace printer {

STLDriver_Input::STLDriver_Input()
    : stl_format_input_(NULL) {
}

STLDriver::STLDriver()
    : input_(STLDriver_Input()) {
}

STLDriver::STLDriver(const Input& input)
    : input_(input) {
}

STLDriver::~STLDriver() {
  ClearObjects();
}

void STLDriver::AddOwnedObject(PrintObject* object) {
  objects_.push_back(object);
}

void STLDriver::AddOwnedObjects(const std::vector<PrintObject*>& object) {
  objects_.insert(objects_.end(), object.begin(), object.end());
}

void STLDriver::ClearObjects() {
  DeleteElements(&objects_);
}

void STLDriver::Execute(ostream* output) {
  Driver driver(input_.driver_input());

  TriangleMesh mesh;
  driver.Execute(objects_, &mesh);
  
  if (output != NULL) {
    LOG(INFO) << "Writing STL output."; 
    STLFormatWriter::Input writer_input = input_.stl_format_input();
    writer_input.set_output(output);
    STLFormatWriter writer(writer_input);
    writer.WriteMesh("generated", mesh);
  }
}

void STLDriver::ExecuteAndWriteToFile(const string& filename) {
  std::ofstream out(filename.c_str());
  CHECK(out.good()) << "Unable to open: " << filename;
  Execute(&out);
  out.close();
}

}  // namespace printer
