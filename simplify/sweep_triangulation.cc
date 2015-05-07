// Copyright 2015
// Author: Christopher Van Arsdale

#include <string>
#include <sstream>
#include <algorithm>
#include <set>
#include <map>
#include <vector>
#include "common/log/log.h"
#include "printer/base/geometry.h"
#include "printer/simplify/sweep_triangulation.h"

#if 1
// tmp tmp
#include <iostream>
#endif

using std::map;
using std::pair;
using std::set;
using std::string;
using std::vector;

namespace printer {
string SweepTriangulation::Elem::DebugString() const {
  std::stringstream out;
  out << "Point: " << point.DebugString() << std::endl;
  out << "Next: " << next->point.DebugString() << std::endl;
  out << "Prev: " << prev->point.DebugString() << std::endl;
  return out.str();
}

void SweepTriangulation::Elem::Delete() {
  Elem* n = this->next;
  while (n != this) {
    Elem* tmp = n;
    n = n->next;
    delete tmp;
  }
  delete this;
}

SweepTriangulation::Elem* SweepTriangulation::Elem::ShallowCopy() {
  return new Elem(point, prev, next);
}

void SweepTriangulation::Elem::AddBack(const Point2D& point) {
  Elem* elem = new Elem(point);

  // Set new pointers.
  elem->next = this;
  elem->prev = this->prev;

  // Fix up existing pointers.
  this->prev->next = elem;
  this->prev = elem;
}

template <bool top_to_bottom>
struct SweepTriangulation::SortPoints {
  static bool PrimarySort(const Elem* a, const Elem* b) {
    return (top_to_bottom ?
            a->point.y() > b->point.y() :
            a->point.y() < b->point.y());
  }

  static bool SecondarySort(const Elem* a, const Elem* b) {
    return (top_to_bottom ?
            a->point.x() > b->point.x() :
            a->point.x() < b->point.x());
  }

  static bool ApproxEquals(double a, double b) {
    return fabs(a - b) < 10 * Triangle::kDefaultIntersectEpsilon;
  }
  static bool ApproxEqualsPrimary(const Point2D& a, const Point2D& b) {
    return ApproxEquals(a.y(), b.y());
  }
  static bool Sort(const Point2D& a, const Point2D& b) {
    double ay = a.y();
    double by = b.y();
    if (ApproxEquals(ay, by)) {
      return SortSecondary(b, a);
    }
    return (top_to_bottom ? by <= ay : ay <= by);
  }
  static bool SortExtreme(const Point2D& a, const Point2D& b) {
    double ay = a.y();
    double by = b.y();
    if (ApproxEquals(ay, by)) {
      return SortSecondary(a, b);
    }
    return (top_to_bottom ? by <= ay : ay <= by);
  }
  static bool SortPrimaryLE(const Point2D& a, const Point2D& b) {
    double ay = a.y();
    double by = b.y();
    return ApproxEquals(ay, by) || (top_to_bottom ? by <= ay : ay <= by);
  }
  static bool SortPrimary(const Point2D& a, const Point2D& b) {
    double ay = a.y();
    double by = b.y();
    return !ApproxEquals(ay, by) && (top_to_bottom ? by < ay : ay < by);
  }
  static bool SortSecondary(const Point2D& a, const Point2D& b) {
    return SortSecondaryVal(a.x(), b.x());
  }
  static bool SortSecondaryVal(double a, double b) {
    return (top_to_bottom ? a < b : b < a);
  }
  static Point2D DiffSecondary(const Point2D& a, const Point2D& b) {
    return Point2D(a.x() - b.x(), 0);
  }
};

template <typename SortOrder>
class SweepTriangulation::EdgeMap {
 public:
  struct SortEdge {
    bool operator()(const Edge2D& a, const Edge2D& b) const {
      // Cross products to determine which "side" of each other the two
      // edges fall onto.
      // We don't care about which direction (e.g. up/down) the edges are
      // pointing in. We want to normalize that to pointing up, and then sort
      // to the left.
      Edge2D a_norm = SortOrder::Sort(a.p0(), a.p1()) ?
                      Edge2D(a.p1(), a.p0()) : a;
      Edge2D b_norm = SortOrder::Sort(b.p0(), b.p1()) ?
                      Edge2D(b.p1(), b.p0()) : b;
      if (a_norm == b_norm) {
        return false;
      }
      bool select_a =  (b_norm.p0() == b_norm.p1()) ||
                       (a_norm.p0() != a_norm.p1() &&
                        SortOrder::SortPrimary(b_norm.p0(), a_norm.p0()));
      const Edge2D& bottom = (select_a ? a_norm : b_norm);
      const Edge2D& top = (select_a ? b_norm : a_norm);
      const Point2D& comp = top.p0();
      double z_cross = bottom.direction().z_cross(comp - bottom.p0());
      if (z_cross == 0) {
        return SortOrder::SortSecondaryVal(
            std::min(a_norm.p0().x(), a_norm.p1().x()),
            std::max(b_norm.p0().x(), b_norm.p1().x()));
      }
      return (select_a ? z_cross < 0 : z_cross > 0);
    }
  };

 public:
  EdgeMap() {}
  ~EdgeMap() {}

  void Insert(const Edge2D& edge, Elem* elem) {
    VLOG(3) << "Insert: " << edge.DebugString();
    edge_map_[edge] = elem;
    /*
    LG << "MAP:";
    for (auto it : edge_map_) {
      LG << "EDGE: " << it.first.DebugString();
    }
    */
  }

  void Remove(const Edge2D& edge) {
    VLOG(3) << "Remove: " << edge.DebugString();
    edge_map_.erase(edge);
  }

  Elem* Replace(Elem* elem) {
    // Get left edge
    bool error = false;
    typename Map::iterator it = GetInternal(elem, &error);
    if (error) {
      return NULL;
    }

    // Replace it.
    VLOG(3) << "Replaced edge " << it->first.DebugString()
            << " with helper " << elem->point.DebugString();
    Elem* old = it->second;
    it->second = elem;
    return old;
  }

  Elem* Get(Elem* elem) {
    // Get left edge
    bool error = false;
    auto it = GetInternal(elem, &error);
    if (error) {
      return NULL;
    }
    return it->second;
  }

 private:
  typedef map<Edge2D, Elem*, SortEdge> Map;

  typename Map::iterator GetInternal(Elem* elem, bool* error) {
    if (edge_map_.empty()) {
      LOG(ERROR) << "Internal sweep error.";
      *error = true;
      return edge_map_.end();
    }

    // 1) Find left.
    Edge2D fake_edge(elem->point, elem->point);
    typename Map::iterator it = edge_map_.lower_bound(fake_edge);
    if (it == edge_map_.begin()) {
      LOG(ERROR) << "Internal error: " << fake_edge.DebugString()
                 << " VS " << it->first.DebugString();
      *error = true;
      return it;
    }
    --it;
    return it;
  }

  Map edge_map_;
};

// static
bool SweepTriangulation::SortPointsTest(const Point2D& a, const Point2D& b) {
  return SortPoints<true>::Sort(a, b);
}

// static
bool SweepTriangulation::SortEdgesTest(bool top_to_bottom,
                                       const Edge2D& a, const Edge2D& b) {
  if (top_to_bottom) {
    return EdgeMap<SortPoints<true> >::SortEdge()(a, b);
  }
  return EdgeMap<SortPoints<false> >::SortEdge()(a, b);
}

SweepTriangulation::SweepTriangulation() {
}

SweepTriangulation::~SweepTriangulation() {
  for (int i = 0; i < loops_.size(); ++i) {
    loops_[i]->Delete();
  }
}

void SweepTriangulation::AddLoop(const vector<Point2D>& input) {
  Elem* head = new Elem(input[0]);
  VLOG(5) << "New Loop.";
  VLOG(5) << "Point: " << input[0].DebugString();
  for (int i = 1; i < input.size(); ++i) {
  VLOG(5) << "Point: " << input[i].DebugString();
    head->AddBack(input[i]);
  }
  VLOG(5) << "Loop End";
  loops_.push_back(head);
}

bool SweepTriangulation::Triangulate(vector<Triangle2D>* triangles) {
  // 1) Sort by Y axis, +/- epsilon are the same value.
  vector<Elem*> all_elements;
  GetAllElements(&all_elements);
  std::sort(all_elements.begin(), all_elements.end(),
            SortPoints<true>::PrimarySort);

  // 2) Sweep down Y, collect edges.
  vector<pair<Elem*, Elem*> > edges;
  if (!SingleSweep<SortPoints<true> >(&all_elements, &edges)) {
    return false;
  }

  // 3) Sweep up Y, collect edges.
  std::reverse(all_elements.begin(), all_elements.end());
  if (!SingleSweep<SortPoints<false> >(&all_elements, &edges)) {
    return false;
  }

  // 4) Construct new polygons.
  Splice(edges, &all_elements);
  set<Elem*> seen;
  vector<Elem*> loops;
  for (int i = 0; i < all_elements.size(); ++i) {
    Elem* next = all_elements[i];
    int size_of_loop = 1;
    if (seen.insert(next).second) {
      VLOG(3) << "Loop:";
      VLOG(3) << "Point: " << next->point.DebugString();
      Elem* best = next;
      for (next = next->next; seen.insert(next).second; next = next->next) {
        ++size_of_loop;
        VLOG(3) << "Point: " << next->point.DebugString();
        if (SortPoints<true>::SortExtreme(next->point, best->point)) {
          best = next;
        }
      }
      VLOG(3) << "End: " << size_of_loop;
      VLOG(3) << "Selected: " << best->point.DebugString();
      if (next != all_elements[i]) {
        LOG(ERROR) << "Splice failed, created loop with multiple splices.";
        return false;
      }
      if (size_of_loop < 3) {
        LOG(ERROR) << "Invalid loop.";
        return  false;
      }
      loops.push_back(best);
    }
  }

  // 5) Construct triangles for each polygon.
  bool retval = true;
  for (Elem* top : loops) {
    retval &= TriangulateLoop(top, triangles);
    top->Delete();
  }
  loops_.clear();
  return retval;
}

// static
SweepTriangulation::Elem* SweepTriangulation::GetNewPoints(
    Elem* p0_base,
    Elem* p1_base,
    const map<Elem*, set<Elem*> >& replacements) {
  VLOG(4) << "Trying to cut diagonal: "
          << Edge2D(p0_base->point, p1_base->point).DebugString();
  auto it = replacements.find(p0_base);
  if (it != replacements.end()) {
    // TODO: sorted map so we don't have to check each one.
    // Find first where we are inside of its angle.
    Point2D dir = p1_base->point - p0_base->point;
    for (Elem* test : it->second) {
      Point2D prev = (test->prev->point - p0_base->point);
      Point2D next = (test->next->point - p0_base->point);
      if (prev == dir || next == dir) {
        // Duplicate diagonal.
        VLOG(4) << "Duplicate diagonal: "
                << Edge2D(p0_base->point, p1_base->point).DebugString();
        return NULL;
      }

      bool prev_cross = prev.z_cross(dir) > 0;
      bool next_cross = next.z_cross(dir) > 0;
      bool our_cross = next.z_cross(prev) > 0;
      VLOG(5) << "Testing loop to bisect: " << test->DebugString();
      if ((our_cross && next_cross && !prev_cross) ||
          (!our_cross && (next_cross || !prev_cross))) {
        VLOG(5) << "Selected: " << test->DebugString();
        return test;
      }
    }
  }
  return p0_base;
}

// static
void SweepTriangulation::Splice(
    const vector<pair<Elem*, Elem*> >& edges,
    vector<Elem*>* all_elements) {
  map<Elem*, set<Elem*> > replacements;

  // 4) Reconstruct new polygons.
  for (auto it : edges) {
    // Figure out which elements we are splicing into. This is easy for the
    // base case (no existing splices). This is harder for when we want to
    // splice multiple diagonals into the same point.
    Elem* p0 = GetNewPoints(it.first, it.second, replacements);
    Elem* p1 = GetNewPoints(it.second, it.first, replacements);
    if (p0 == NULL || p1 == NULL) {
      // Duplicate diagonal.
      continue;
    }

    // Splice in new elements.
    Elem* new_p0 = p0->ShallowCopy();
    Elem* new_p1 = p1->ShallowCopy();

    // Splice into p0.
    p0->next = new_p1;
    new_p1->prev = p0;
    new_p1->next->prev = new_p1;

    // Splice into p1
    p1->next = new_p0;
    new_p0->prev = p1;
    new_p0->next->prev = new_p0;

    // Record new values.
    all_elements->push_back(new_p0);
    all_elements->push_back(new_p1);
    replacements[it.first].insert(new_p0);
    replacements[it.second].insert(new_p1);
  }
}

namespace {
template <typename SortOrder>
bool OpenAngle(const Point2D& point, const Point2D& prev, const Point2D& next) {
  bool prev_eq = SortOrder::ApproxEqualsPrimary(point, prev);
  bool next_eq = SortOrder::ApproxEqualsPrimary(point, next);
  Point2D next_dir = next_eq ? SortOrder::DiffSecondary(next, point) : next - point;
  Point2D prev_dir = prev_eq ? SortOrder::DiffSecondary(prev, point) : prev - point;
  return next_dir.z_cross(prev_dir) < 0;
}
}

// static
template <typename SortOrder>
bool SweepTriangulation::PolygonToSide(const Elem& elem) {
  const Point2D& point = elem.point;
  const Point2D& prev = elem.prev->point;
  const Point2D& next = elem.next->point;
  if (SortOrder::SortPrimary(point, prev)) {
    return (SortOrder::SortPrimary(next, point) ||
            OpenAngle<SortOrder>(point, prev, next));
  }
  return (SortOrder::SortPrimary(next, point) &&
          OpenAngle<SortOrder>(point, prev, next));
}

// static
template <typename SortOrder>
bool SweepTriangulation::IsPointVertex(const Elem& elem) {
  const Point2D& point = elem.point;
  const Point2D& prev = elem.prev->point;
  const Point2D& next = elem.next->point;
  bool prev_eq = SortOrder::ApproxEqualsPrimary(point, prev);
  bool next_eq = SortOrder::ApproxEqualsPrimary(point, next);
  if (prev_eq) {
    return (SortOrder::SortPrimary(point, next) &&
            OpenAngle<SortOrder>(point, prev, next));
  } else if (next_eq) {
    return (SortOrder::SortPrimary(point, prev) &&
            OpenAngle<SortOrder>(point, prev, next));
  }
  return (SortOrder::SortPrimary(point, prev) &&
          SortOrder::SortPrimary(point, next) &&
          OpenAngle<SortOrder>(point, prev, next));
}

// static
template <typename SortOrder>
bool SweepTriangulation::SingleSweep(
    vector<Elem*>* all_elements,
    vector<pair<Elem*, Elem*> >* edges) {
  EdgeMap<SortOrder> edge_helpers;

  // 1) Sweep down (e.g. iterate through sorted list).
  VLOG(1) << "Single sweep =================================================== ";
  for (int i = 0; i < all_elements->size();) {
    int j = i + 1;
    for (; j < all_elements->size(); ++j) {
      if (!SortOrder::ApproxEqualsPrimary((*all_elements)[j]->point,
                                          (*all_elements)[j-1]->point)) {
        break;
      }
    }
    std::sort(all_elements->begin() + i, all_elements->begin() + j,
              SortOrder::SecondarySort);
    if (!SweepPoints<SortOrder>(*all_elements, i, j, &edge_helpers, edges)) {
      return false;
    }
    i = j;
  }
  return true;
}

// static
template <typename SortOrder>
bool SweepTriangulation::SweepPoints(
    const vector<Elem*>& all_elements,
    int start, int end,
    EdgeMap<SortOrder>* edge_helpers,
    vector<pair<Elem*, Elem*> >* edges) {
  // Order of operations.
  // 1) Foreach: We check for split point, in which case we need to add a diagonal.
  // 2) Foreach: 
  //   a) We replace left if polygon is to the left of our point.
  //   b) We insert next if polygon is to the right of our point.
  //   c) We remove prev if it is above (by sweep order) of our current point.

#if 0
    // TMP TMP
    std::cin.ignore(10000, '\n');
#endif

  VLOG(1) << "Processing [" << start << ", " << end << ") -----------------";

  // First, check for split points.
  for (int i = start; i < end; ++i) {
    Elem* current = all_elements[i];
    VLOG(1) << "Checking for split: " << current->point.DebugString();
    // Check for split vertex, which requires a new diagonal.
    if (IsPointVertex<SortOrder>(*current)) {
      Elem* old = edge_helpers->Get(current);
      if (old == NULL) {
        return false;
      }
      VLOG(2) << "Is split point, inserting edge: "
              << Edge2D(current->point, old->point).DebugString();
      edges->push_back(std::make_pair(current, old));
    }
  }

  // Second, update our edge helpers.
  for (int i = start; i < end; ++i) {
    Elem* current = all_elements[i];
    const Point2D& point = current->point;
    const Point2D& prev = current->prev->point;
    const Point2D& next = current->next->point;
    VLOG(1) << "Updating: " << point.DebugString();

    // Check for previous edge to remove.
    if (SortOrder::SortPrimary(prev, point)) {
      VLOG(2) << "Has previous edge.";
      edge_helpers->Remove(Edge2D(prev, point));
    }

    // Check to update helper point (e.g. there is a line to the left).
    if (PolygonToSide<SortOrder>(*current)) {
      VLOG(2) << "Polygon to side.";
      if (!edge_helpers->Replace(current)) {
        return false;
      }
    }

    // Check for next edge to add.
    if (SortOrder::SortPrimary(point, next)) {
      VLOG(2) << "Has next edge.";
      edge_helpers->Insert(Edge2D(point, next), current);
    }
  }
  return true;
}

void SweepTriangulation::GetAllElements(vector<Elem*>* all) {
  for (Elem* start : loops_) {
    Elem* iter = start;
    do {
      all->push_back(iter);
      iter = iter->next;
    } while (iter != start);
  }
}

struct SweepTriangulation::Vertex {
  enum LeftRight {
    NONE = 0,
    LEFT = 1,
    RIGHT = 2,
    BOTH = 3,
  };

  Vertex() {}
  Vertex(Point2D p, LeftRight lr) : point(p), left_right(lr) {}

  Point2D point;
  LeftRight left_right;
};

// static
bool SweepTriangulation::TriangulateLoop(Elem* top,
                                         vector<Triangle2D>* triangles) {
  if (VLOG_IS_ON(3)) {
    VLOG(3) << "Single Loop";
    Elem* next = top;
    do {
      VLOG(3) << "Point: " << next->point.DebugString();
      next = next->next;
    } while (next != top);
    VLOG(3) << "End.";
  }

  // Simple algorithm:
  // 1) Sort points (can be merge sorted by merging 2 polygon chains) into Y
  //    order.
  Elem* left = top->next;
  Elem* right = top->prev;

  // 2) Take top 2 points, push onto stack.
  vector<Vertex> stack;
  stack.push_back(Vertex(top->point, Vertex::BOTH));
  stack.push_back(GetNext(&left, &right));

  // 3) Take next point in sorted points, try and triangulate with top of
  //    of stack. If successful, pop stack and repeat. If unsuccessful, push
  //    the point onto the stack. Will eventually run out of points.
  while (true) {
    Vertex next = GetNext(&left, &right);
    if (!ProcessStack(next, &stack, triangles)) {
      return false;
    }
    if (next.left_right == Vertex::BOTH) {
      break;
    }
    stack.push_back(next);
  }

  return true;
}

// static
bool SweepTriangulation::ProcessStack(const Vertex& next,
                                      vector<Vertex>* stack,
                                      vector<Triangle2D>* triangles) {
  Vertex top = stack->back();

  // if top is on the same chain as next, any triangle we make will cover
  // "top" and nothing should see it again.
  while (stack->size() >= 2) {
    //  What is the "try" mean?
    //    => Cannot make triangle with previous points on chain. Cannot make
    //       triangle where angle is less than previous angle created (e.g.
    //       would end up intersecting the edge of the polygon). Fortunately
    //       we can check this by just doing a simple winding order check.

    // Try to triangulate.
    Vertex back = (*stack)[stack->size() - 1];
    Vertex second = (*stack)[stack->size() - 2];

    // Winding order: next, back, second.
    //              Next 
    //           R       L
    // Back R   nbs     nbs
    //      L   nsb     nsb

    Triangle2D triangle =
        (back.left_right & Vertex::LEFT) ?
        Triangle2D(next.point, second.point, back.point) :
        Triangle2D(next.point, back.point, second.point);
    if (triangle.CounterClockwise() &&
        (!(back.left_right & next.left_right & second.left_right) ||
         !SortPoints<true>::ApproxEqualsPrimary(next.point, back.point) ||
         !SortPoints<true>::ApproxEqualsPrimary(back.point, second.point))) {
      triangles->push_back(triangle);
      stack->pop_back();
    } else {
      break;
    }
  }
  // If top is on a different chain, we need to push it back on, otherwise we
  // covered it up with our first triangle.
  if (!(top.left_right & next.left_right)) {
    if (stack->size() != 1) {
      LOG(ERROR) << "Internal 2D sweep error, trying to recover.";
      return false;
    }
    stack->clear();
    stack->push_back(top);
  }

  // TODO:
  //  This will create as many long-skinny triangles as it can. It would be
  //  nicer if it created fat polygons. We should do a flip test on all
  //  triangle pairs.
  return true;
}

// static
SweepTriangulation::Vertex SweepTriangulation::GetNext(
    Elem** left,
    Elem** right) {
  // For when they crossed.
  if (*left == *right) {
    return Vertex((*right)->point, Vertex::BOTH);
  }

  if (SortPoints<true>::Sort((*left)->point, (*right)->point)) {
    Elem* out = *left;
    *left = (*left)->next;
    return Vertex(out->point, Vertex::LEFT);
  }

  Elem* out = *right;
  *right = (*right)->prev;
  return Vertex(out->point, Vertex::RIGHT);
}

}  // namespace printer

