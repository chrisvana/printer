// Copyright 2015
// Author: Christopher Van Arsdale

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "common/base/flags.h"
#include "common/base/types.h"
#include "common/log/log.h"
#include "common/strings/strutil.h"
#include "printer/base/mesh.h"
#include "printer/stl/format.h"

DEFINE_bool(stl_format_binary, true,
            "If true, we print in binary format.");

using std::string;
using std::vector;
using strings::StringPrintf;

namespace printer {
namespace {
template <typename T>
void put_little_endian(const T& t, std::ostream* out) {
  for (int i = 0; i < sizeof(t); ++i) {
    out->put(t >> (8 * i) & 0xFF);
  }
}

template <>
void put_little_endian(const float& t, std::ostream* out) {
  put_little_endian(reinterpret_cast<uint32 const&>(t), out);
}
}  // anonymous namespace

STLFormatWriter_Input::STLFormatWriter_Input(std::ostream* output)
    : output_(output),
      binary_format_(FLAGS_stl_format_binary) {
}

// static
void STLFormatWriter::WriteOrDie(const std::string& filename,
                                 const TriangleMesh& mesh) {
  std::ofstream out(filename.c_str());
  CHECK(out.good()) << "Unable to open: " << filename;

  STLFormatWriter::Input writer_input(&out);
  STLFormatWriter writer(writer_input);
  writer.WriteMesh("obj", mesh);

  out.close();
}

STLFormatWriter::STLFormatWriter(const Input& input)
    : input_(input) {
}

STLFormatWriter::~STLFormatWriter() {
}

void STLFormatWriter::WriteMesh(const string& name, const TriangleMesh& mesh) {
  VLOG(3) << "STLFormatWriter::WriteMesh";
  StartObject(name, mesh.size());
  for (TriangleMesh::TriangleIterator iter = mesh.iterator();
       !iter.Done(); iter.Next()) {
    PrintTriangle(iter.triangle());
  }
  EndObject(name);
}

void STLFormatWriter::StartObject(const string& name, int triangle_count) {
  VLOG(3) << "STLFormatWriter::StartObject";
  if (!FLAGS_stl_format_binary) {
    *input_.output() << "solid " << name << "\n";
  } else {
    // UINT8[80] – Header
    // UINT32 – Number of triangles
    for (int i = 0; i < 80; ++i) {
      put_little_endian<uint8>(0, input_.output());
    }
    put_little_endian<uint32>(triangle_count, input_.output());
  }
}

void STLFormatWriter::EndObject(const string& name) {
  if (!FLAGS_stl_format_binary) {
    *input_.output() << "endsolid " << name << "\n";
  }
}

void STLFormatWriter::PrintTriangle(const Triangle& triangle) {
  VLOG(5) << "STLFormatWriter::PrintTriangle";
  std::ostream* out = input_.output();
  if (!input_.binary_format()) {
    Point normal = triangle.normal();
    *out << StringPrintf("facet normal %lf %lf %lf\n",
                         normal.x(), normal.y(), normal.z());
    *out << "    outer loop\n";
    *out << StringPrintf("        vertex %lf %lf %lf\n",
                         triangle.p0().x(), triangle.p0().y(),
                         triangle.p0().z());
    *out << StringPrintf("        vertex %lf %lf %lf\n",
                         triangle.p1().x(), triangle.p1().y(),
                         triangle.p1().z());
    *out << StringPrintf("        vertex %lf %lf %lf\n",
                         triangle.p2().x(), triangle.p2().y(),
                         triangle.p2().z());
    *out << "    end loop\n";
    *out << "endfacet\n";
  } else {
    // foreach triangle
    //REAL32[3] – Normal vector
    //REAL32[3] – Vertex 1
    //REAL32[3] – Vertex 2
    //REAL32[3] – Vertex 3
    //UINT16 – Attribute byte count
    //end

    // Normal vector
    Point normal = triangle.normal();
    put_little_endian<float>(normal.x(), out);
    put_little_endian<float>(normal.y(), out);
    put_little_endian<float>(normal.z(), out);

    // Vertex 1
    put_little_endian<float>(triangle.p0().x(), out);
    put_little_endian<float>(triangle.p0().y(), out);
    put_little_endian<float>(triangle.p0().z(), out);

    // Vertex 2
    put_little_endian<float>(triangle.p1().x(), out);
    put_little_endian<float>(triangle.p1().y(), out);
    put_little_endian<float>(triangle.p1().z(), out);

    // Vertex 3
    put_little_endian<float>(triangle.p2().x(), out);
    put_little_endian<float>(triangle.p2().y(), out);
    put_little_endian<float>(triangle.p2().z(), out);

    // Attribute byte
    put_little_endian<uint16>(0, out);
  }
}


STLFormatReader_Input::STLFormatReader_Input(std::istream* input)
    : input_(input) {
}

STLFormatReader::STLFormatReader(const Input& input)
    : input_(input) {
}

STLFormatReader::~STLFormatReader() {
}

bool STLFormatReader::ReadMesh(TriangleMesh* mesh) {
  // TODO.
  LOG(FATAL) << "NOT IMPLEMENTED.";
  return false;
}

}  // namespace printer
