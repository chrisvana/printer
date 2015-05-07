// Copyright 2015
// Author: Christopher Van Arsdale

#ifndef _PRINTER_SIMPLIFY_SWEEP_TRIANGULATION_H__
#define _PRINTER_SIMPLIFY_SWEEP_TRIANGULATION_H__

#include <string>
#include <map>
#include <set>
#include <vector>
#include "printer/base/geometry.h"

namespace printer {

class SweepTriangulation {
 public:
  SweepTriangulation();
  ~SweepTriangulation();

  void AddLoop(const std::vector<Point2D>& loop);
  bool Triangulate(std::vector<Triangle2D>* triangles);

 private:
  struct Elem {
    explicit Elem(const Point2D& p) : point(p), prev(this), next(this) {}
    Elem(const Point2D& p, Elem* pr, Elem* n) : point(p), prev(pr), next(n) {}
    Point2D point;
    Elem* prev;
    Elem* next;


    void AddBack(const Point2D& p);
    void Delete();
    std::string DebugString() const;
    Elem* ShallowCopy();
    Edge2D NextEdge() const { return Edge2D(point, next->point); }
    Edge2D PrevEdge() const { return Edge2D(prev->point, point); }
  };
  template <bool top_to_bottom> struct SortPoints;
  template <typename SortOrder> class EdgeMap;

  void GetAllElements(std::vector<Elem*>* all);

  // Sweep
  template <typename SortOrder> static bool PolygonToSide(const Elem& elem);
  template <typename SortOrder> static bool IsPointVertex(const Elem& elem);
  template <typename SortOrder> static bool SingleSweep(
      std::vector<Elem*>* all_elements,
      std::vector<std::pair<Elem*, Elem*> >* edges);
  template <typename SortOrder> static bool SweepPoints(
      const std::vector<Elem*>& all_elements,
      int start, int end,
      EdgeMap<SortOrder>* edge_helpers,
      std::vector<std::pair<Elem*, Elem*> >* edges);

  // Splicing
  static void Splice(
      const std::vector<std::pair<Elem*, Elem*> >& edges,
      std::vector<Elem*>* all_elements);
  static Elem* GetNewPoints(
      Elem* p0_base,
      Elem* p1_base,
      const std::map<Elem*, std::set<Elem*> >& replacements);

  // Triangulation
  static bool TriangulateLoop(Elem* top, std::vector<Triangle2D>* triangles);
  struct Vertex;
  static bool ProcessStack(const Vertex& next,
                           std::vector<Vertex>* stack,
                           std::vector<Triangle2D>* triangles);
  static Vertex GetNext(Elem** left, Elem** right);

  std::vector<Elem*> loops_;

 protected:
  friend class SweepTriangulationTest;
  static bool SortPointsTest(const Point2D& a, const Point2D& b);
  static bool SortEdgesTest(bool top_to_bottom, const Edge2D& a, const Edge2D& b);
};

}  // namespace printer

#endif  // _PRINTER_SIMPLIFY_SWEEP_TRIANGULATION_H__
