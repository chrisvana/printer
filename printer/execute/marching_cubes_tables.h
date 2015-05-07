// Copyright 2015
// Original author: Paul Bourke via http://paulbourke.net/geometry/polygonise/
// Port: Christopher Van Arsdale

#ifndef _PRINTER_EXECUTE_MARCHING_CUBES_TABLE_H__
#define _PRINTER_EXECUTE_MARCHING_CUBES_TABLE_H__

namespace printer {

class MarchingCubesTables {
 public:
  static const int kEdgeTable[256];
  static const int kTriangleTable[256][16];
};

}  // namespace printer

#endif  // _PRINTER_EXECUTE_MARCHING_CUBES_TABLE_H__
