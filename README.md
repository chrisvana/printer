Voxel based 3D geometry and printing, C++
==========

## Background
When I got a 3D printer, I thought it would be fun to procedurally generate objects. While there are a few existing options for procedural generation, it sounded fullfilling to write my own. I worked on this as a side project for a while, and had a lot of fun printing everything from simple buckles to a CAT scan of my skull.

I wanted a library that started with "objects", where each object only had to specify whether given a point x,y,z was inside or outside of the object, and that's it. For example, you could write a sphere object as:
```
class SphereObject : public PrintObject {
  public:
    bool ContainsPoint(const Point& p) { return p.x()*p.x() + p.y()*p.y() + p.z()*p.z() < 100; }
};
```
... and magically we would get a sphere printed out with radius 10cm.

## Summary
This library is written in C++, and available under BSD-3.

##### Building:
```
git clone https://github.com/chrisvana/printer.git
cd printer

# Builds binaries in printer/examples by default.
./make

# Example, generate a sphere:
./sphere --logtostderr --output_stl_file=/tmp/sphere.stl
```

###### High Level Flow
The code works by:
  1. Taking a specified set of PrintObjects
  2. Filling a set of voxels based on the PrintObjects
  3. Uses a Marching Cubes algorithm to triangulate the voxels into a mesh
  4. Performs simplification
    1. It first tries to reduce each face of the mesh to a minimal number of triangles (e.g. a large cube may have a million triangles, but only 12 are required to represent the 6 faces).
    2. It then computes a boundary that the mesh cannot cross using Marching Cubes again, but with different ISO cutoffs (e.g. 0.4 and 0.6).
    3. Using the boundary mesh to test validity of each simplification, we performs decimation on the original mesh.
  1. Finally, it writes the resulting mesh out to an STL file.

I should note that the final output is a .stl file (not the .x3g file used by most 3D printers), so you will need to slice the STL using something like ReplicatorG (or Makerware, in my case).

Simplification can be quite slow (though helps later with slicing faster), and diabled via:
--driver_run_simplifier=false
--driver_run_face_reduction=false
See "Configuration flags" below for more options.

## Examples:
The examples live in [printer/examples](https://github.com/chrisvana/printer/tree/master/printer/examples):
- [sphere](https://github.com/chrisvana/printer/blob/master/printer/examples/sphere.cc): Produces a sphere.
- [buckle](https://github.com/chrisvana/printer/blob/master/printer/examples/buckle.cc): Produces a curved buckle.
- [cylinder](https://github.com/chrisvana/printer/blob/master/printer/examples/cylinder.cc): Intersects two cylinders.
- [merged](https://github.com/chrisvana/printer/blob/master/printer/examples/merged.cc): Produces a large ring from 200 offset spheres.
- [rotated](https://github.com/chrisvana/printer/blob/master/printer/examples/rotated.cc): Demonstrates some translation/rotation on the PrintObjects.
- [mandel](https://github.com/chrisvana/printer/blob/master/printer/examples/mandel.cc): My (mostly failed) attempt at a 3D fractal.
- [print_dicom](https://github.com/chrisvana/printer/blob/master/printer/examples/print_dicom.cc): Prints out a DICOM file (or files), used by medical imaging.

## DICOM Support
[printer/dicom/dicom_set](https://github.com/chrisvana/printer/blob/master/printer/dicom/dicom_set.cc) generates a PrintObject from a set of DICOM files.

It does so by taking all frames from all input DICOM files, mapping the pixels in them to points, and then interpolates values in our voxel map based on the pixel values.

The [print_dicom](https://github.com/chrisvana/printer/blob/master/printer/examples/print_dicom.cc) example can be used to actually print a DICOM file, see the file for instructions.

## Configuration flags
(Not all are included)

###### Threading
- _--driver_default_parallelism_ (Specifies the number of parallel threads we
   use for some operations.) type: int32 default: -1

###### Voxel Region
- _--horizontal_resolution_ (Horizontal resolution, in mm.) type: double
    default: 0.10000000000000001
- _--vertical_resolution_ (Vertical resolution, in mm.) type: double
    default: 0.10000000000000001
- _--print_region_size_ (Print region size, in mm.) type: double default: 100

###### Marching
- _--driver_iso_level_ (ISO value to use when generating cubes.) type: double
    default: 0.5
- _--marching_cubes_min_island_size_ (We trim any holes/islands smaller than
   this size from our output.) type: int32 default: 0

###### Simplification
- _--driver_run_face_reduction_ (If false, disable face reduction.) type: bool
    default: true
- _--driver_run_simplifier_ (If false, disable simplification.) type: bool
    default: true
- _--driver_min_triangles_to_simplify_ (If >= 0, the maximum number of triangles
    to take before we run the simplification process.) type: int32
    default: -1

- _--driver_boundary_is_single_voxel_ (Overrides inner/outer boundary to be the
    voxel that contains our initial points, rather than an iso surface
    defined by driver_*_boundary_iso_level.) type: bool default: true
- _--driver_inner_boundary_iso_level_ (ISO value of inner boundary to use when
    simplifying. You will need to set --driver_boundary_is_single_voxel=false
    for this to be used.) type: double default: 0.90000000000000002
- _--driver_outer_boundary_iso_level_ (ISO value of outer boundary to use when
    simplifying. You will need to set --driver_boundary_is_single_voxel=false
    for this to be used.) type: double default: 0.10000000000000001

###### Dicom
- _--dicom_set_point_mode_ (Mode to use when reconstructing voxel values. See
    PointList::FromString for values.) type: string default: "INTERPOLATE"

## Code Layout
The library's major components are split into subdirectories:
- Base geometry library: [printer/base](https://github.com/chrisvana/printer/tree/master/printer/base)
   - Contains classes for manipulating geometry.
   - Points, triangles, meshes, octrees, etc.
- Procedural objects: [printer/objects](https://github.com/chrisvana/printer/tree/master/printer/objects)
   - This contains our abstract PrintObject, along with some useful objects (cubes, spheres, intersection, etc).
- Voxel manipulation: [printer/execute](https://github.com/chrisvana/printer/tree/master/printer/execute)
   - Drives procedural generation of voxel map.
   - Takes voxel map and runs Marching Cube triangulation
   - Calls simplifiers.
- Simplification: [printer/simplify](https://github.com/chrisvana/printer/tree/master/printer/simplify)
   - Does 2D (single face) and 3D (full mesh) simplification
- STL Output: [printer/stl](https://github.com/chrisvana/printer/tree/master/printer/stl)
   - Writes STL files given a triangle mesh.
- DICOM: [printer/dicom](https://github.com/chrisvana/printer/tree/master/printer/dicom)
   - Loads a set of dicom files into a PrintObject to be used by the rest of the library.
