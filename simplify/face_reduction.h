// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_SIMPLIFY_FACE_REDUCTION_H__
#define _PRINTER_SIMPLIFY_FACE_REDUCTION_H__

namespace thread {
class ThreadPool;
}

namespace printer {

class TriangleMesh;

class FaceReduction_Input {
 public:
  FaceReduction_Input();
  ~FaceReduction_Input() {}

  thread::ThreadPool* pool() const { return pool_; }
  void set_pool(thread::ThreadPool* pool) { pool_ = pool; }

  bool allow_broken_faces() const { return allow_broken_faces_; }
  void set_allow_broken_faces(bool allow) { allow_broken_faces_ = allow; }

  double minimum_triangle_face_ratio() const {
    return minimum_triangle_face_ratio_;
  }
  void set_minimum_triangle_face_ratio(double m) {
    minimum_triangle_face_ratio_ = m;
  }

 private:
  thread::ThreadPool* pool_;
  bool allow_broken_faces_;
  double minimum_triangle_face_ratio_;
};

class FaceReduction {
 public:
  typedef FaceReduction_Input Input;

  // Helpers to make it easier.
  static void ReduceFaces(TriangleMesh* mesh);
  static void ReduceFacesParallel(TriangleMesh* mesh);

  explicit FaceReduction(const Input& input) : input_(input) {}
  ~FaceReduction() {}
  bool Execute(TriangleMesh* mesh) const;

 private:
  Input input_;
};

}  // namespace printer

#endif  // _PRINTER_SIMPLIFY_FACE_REDUCTION_H__
