// Copyright 2015
// Author: Christopher Van Arsdale

#include <set>
#include <vector>
#include "common/test/test.h"
#include "common/log/log.h"
#include "printer/base/geometry.h"
#include "printer/simplify/sweep_triangulation.h"

using std::vector;
using std::set;

namespace printer {
class SweepTriangulationTest : public testing::Test {
 public:
  SweepTriangulationTest() {}

 protected:
  typedef vector<Triangle2D> TriangleList;
  ::testing::AssertionResult AssertTrianglesEqual(
       const char* m_expr,
       const char* n_expr,
       const TriangleList& a_list,
       const TriangleList& b_list) {
    set<Triangle2D> a_set, b_set;
    for (const Triangle2D& a : a_list) {
      a_set.insert(a.Normalized());
    }
    for (const Triangle2D& b : b_list) {
      b_set.insert(b.Normalized());
    }

    if (a_list.size() != a_set.size()) {
      testing::AssertionResult r = ::testing::AssertionFailure();
      r << m_expr << " contains duplicate entries." << std::endl;
    }
    if (b_list.size() != b_set.size()) {
      testing::AssertionResult r = ::testing::AssertionFailure();
      r << n_expr << " contains duplicate entries." << std::endl;
    }

    for (const Triangle2D& t : a_set) {
      if (b_set.find(t) == b_set.end()) {
        testing::AssertionResult r = ::testing::AssertionFailure();
        r << m_expr << " != " << n_expr << ":" << std::endl;
        r << m_expr << "{" << std::endl;
        for (const Triangle2D& a_t : a_set) {
          r << "  " << a_t.DebugString()
            << (b_set.find(a_t) == b_set.end() ? "        (mismatch)" : "")
            << std::endl;
        }
        r << "}" << std::endl;
        r << n_expr << "{" << std::endl;
        for (const Triangle2D& b_t : b_set) {
          r << "  " << b_t.DebugString()
            << (b_set.find(b_t) == b_set.end() ? "        (mismatch)" : "")
            << std::endl;
        }
        r << "}" << std::endl;
        return r; 
      }
    }

    return testing::AssertionSuccess();
  }

  static bool SortPointsTest(const Point2D& a, const Point2D& b) {
    return SweepTriangulation::SortPointsTest(a, b);
  }

  static bool SortEdgesTest(bool top_to_bottom, const Edge2D& a, const Edge2D& b) {
    return SweepTriangulation::SortEdgesTest(top_to_bottom, a, b);
  }
};

namespace {

TEST_F(SweepTriangulationTest, SimpleSquare) {
  vector<Point2D> loop;
  loop.push_back(Point2D(1, 0));
  loop.push_back(Point2D(1, 1));
  loop.push_back(Point2D(0, 1));
  loop.push_back(Point2D(0, 0));

  SweepTriangulation triangulate;
  triangulate.AddLoop(loop);

  vector<Triangle2D> output;
  triangulate.Triangulate(&output);

  vector<Triangle2D> expected = {
    Triangle2D(Point2D(0, 0), Point2D(1, 0), Point2D(0, 1)),
    Triangle2D(Point2D(0, 1), Point2D(1, 0), Point2D(1, 1))
  };
  EXPECT_PRED_FORMAT2(AssertTrianglesEqual, expected, output);
}

TEST_F(SweepTriangulationTest, LShape) {
  vector<Point2D> loop;
  loop.push_back(Point2D(0, 0));
  loop.push_back(Point2D(2, 0));
  loop.push_back(Point2D(2, 1));
  loop.push_back(Point2D(1, 1));
  loop.push_back(Point2D(1, 2));
  loop.push_back(Point2D(0, 2));

  SweepTriangulation triangulate;
  triangulate.AddLoop(loop);

  vector<Triangle2D> output;
  triangulate.Triangulate(&output);

  vector<Triangle2D> expected = {
    Triangle2D(Point2D(0, 0), Point2D(1, 1), Point2D(0, 2)),
    Triangle2D(Point2D(0, 0), Point2D(2, 0), Point2D(1, 1)),
    Triangle2D(Point2D(0, 2), Point2D(1, 1), Point2D(1, 2)),
    Triangle2D(Point2D(1, 1), Point2D(2, 0), Point2D(2, 1))
  };
  EXPECT_PRED_FORMAT2(AssertTrianglesEqual, expected, output);
}

TEST_F(SweepTriangulationTest, RotLShape) {
  vector<Point2D> loop;
  loop.push_back(Point2D(0, 0));
  loop.push_back(Point2D(-2, 0));
  loop.push_back(Point2D(-2, -1));
  loop.push_back(Point2D(-1, -1));
  loop.push_back(Point2D(-1, -2));
  loop.push_back(Point2D(0, -2));

  SweepTriangulation triangulate;
  triangulate.AddLoop(loop);

  vector<Triangle2D> output;
  triangulate.Triangulate(&output);

  vector<Triangle2D> expected = {
    Triangle2D(Point2D(0, 0), Point2D(-1, -1), Point2D(0, -2)),
    Triangle2D(Point2D(0, 0), Point2D(-2, 0), Point2D(-1, -1)),
    Triangle2D(Point2D(0, -2), Point2D(-1, -1), Point2D(-1, -2)),
    Triangle2D(Point2D(-1, -1), Point2D(-2, 0), Point2D(-2, -1))
  };
  EXPECT_PRED_FORMAT2(AssertTrianglesEqual, expected, output);
}

TEST_F(SweepTriangulationTest, WithHole) {
  SweepTriangulation triangulate;

  vector<Point2D> loop;
  loop.push_back(Point2D(0, 0));
  loop.push_back(Point2D(10, 0));
  loop.push_back(Point2D(10, 10));
  loop.push_back(Point2D(0, 10));
  triangulate.AddLoop(loop);

  loop.clear();
  loop.push_back(Point2D(4, 4));
  loop.push_back(Point2D(4, 6));
  loop.push_back(Point2D(6, 6));
  loop.push_back(Point2D(6, 4));
  triangulate.AddLoop(loop);


  vector<Triangle2D> output;
  triangulate.Triangulate(&output);

  vector<Triangle2D> expected = {
    Triangle2D(Point2D(0, 0), Point2D(4, 4), Point2D(0, 10)),
    Triangle2D(Point2D(0, 0), Point2D(10, 0), Point2D(4, 4)),
    Triangle2D(Point2D(0, 10), Point2D(4, 4), Point2D(4, 6)),
    Triangle2D(Point2D(0, 10), Point2D(4, 6), Point2D(6, 6)),
    Triangle2D(Point2D(0, 10), Point2D(6, 6), Point2D(10, 10)),
    Triangle2D(Point2D(4, 4), Point2D(10, 0), Point2D(6, 4)),
    Triangle2D(Point2D(6, 4), Point2D(10, 0), Point2D(10, 10)),
    Triangle2D(Point2D(6, 4), Point2D(10, 10), Point2D(6, 6)),
  };
  EXPECT_PRED_FORMAT2(AssertTrianglesEqual, expected, output);
}

TEST_F(SweepTriangulationTest, MultipleHoles) {
  SweepTriangulation triangulate;

  vector<Point2D> loop;
  loop.push_back(Point2D(0, 0));
  loop.push_back(Point2D(10, 0));
  loop.push_back(Point2D(10, 10));
  loop.push_back(Point2D(0, 10));
  triangulate.AddLoop(loop);

  loop.clear();
  loop.push_back(Point2D(2, 2));
  loop.push_back(Point2D(2, 6));
  loop.push_back(Point2D(4, 6));
  loop.push_back(Point2D(4, 2));
  triangulate.AddLoop(loop);

  loop.clear();
  loop.push_back(Point2D(6, 2));
  loop.push_back(Point2D(6, 8));
  loop.push_back(Point2D(8, 8));
  loop.push_back(Point2D(8, 2));
  triangulate.AddLoop(loop);


  vector<Triangle2D> output;
  triangulate.Triangulate(&output);

  vector<Triangle2D> expected = {
    Triangle2D(Point2D(0, 0), Point2D(2, 2), Point2D(0, 10)), 
    Triangle2D(Point2D(0, 0), Point2D(10, 0), Point2D(2, 2)), 
    Triangle2D(Point2D(0, 10), Point2D(2, 2), Point2D(2, 6)), 
    Triangle2D(Point2D(0, 10), Point2D(2, 6), Point2D(6, 8)), 
    Triangle2D(Point2D(0, 10), Point2D(6, 8), Point2D(8, 8)), 
    Triangle2D(Point2D(0, 10), Point2D(8, 8), Point2D(10, 10)), 
    Triangle2D(Point2D(2, 2), Point2D(10, 0), Point2D(4, 2)), 
    Triangle2D(Point2D(2, 6), Point2D(4, 6), Point2D(6, 8)), 
    Triangle2D(Point2D(4, 2), Point2D(6, 2), Point2D(4, 6)), 
    Triangle2D(Point2D(4, 2), Point2D(10, 0), Point2D(6, 2)), 
    Triangle2D(Point2D(4, 6), Point2D(6, 2), Point2D(6, 8)), 
    Triangle2D(Point2D(8, 2), Point2D(10, 0), Point2D(10, 10)), 
    Triangle2D(Point2D(8, 2), Point2D(10, 10), Point2D(8, 8)),
  };
  EXPECT_PRED_FORMAT2(AssertTrianglesEqual, expected, output);
}

TEST_F(SweepTriangulationTest, MultipleDiagonals) {
  SweepTriangulation triangulate;

  vector<Point2D> loop;
  loop.push_back(Point2D(0, 0));
  loop.push_back(Point2D(4, 0));
  loop.push_back(Point2D(5, 3));
  loop.push_back(Point2D(6, 0));
  loop.push_back(Point2D(10, 0));
  loop.push_back(Point2D(10, 10));
  loop.push_back(Point2D(8, 7));
  loop.push_back(Point2D(5, 10));
  loop.push_back(Point2D(2, 7));
  loop.push_back(Point2D(0, 10));
  triangulate.AddLoop(loop);

  vector<Triangle2D> output;
  triangulate.Triangulate(&output);

  vector<Triangle2D> expected = {
    Triangle2D(Point2D(0, 0), Point2D(4, 0), Point2D(0, 10)),
    Triangle2D(Point2D(0, 10), Point2D(4, 0), Point2D(2, 7)),
    Triangle2D(Point2D(2, 7), Point2D(4, 0), Point2D(5, 3)),
    Triangle2D(Point2D(2, 7), Point2D(5, 3), Point2D(8, 7)),
    Triangle2D(Point2D(2, 7), Point2D(8, 7), Point2D(5, 10)),
    Triangle2D(Point2D(5, 3), Point2D(6, 0), Point2D(10, 0)),
    Triangle2D(Point2D(5, 3), Point2D(10, 0), Point2D(8, 7)),
    Triangle2D(Point2D(8, 7), Point2D(10, 0), Point2D(10, 10)),
  };
  EXPECT_PRED_FORMAT2(AssertTrianglesEqual, expected, output);
}

TEST_F(SweepTriangulationTest, SortTest) {
  Point2D a(0.000000000000000000000e+00,-5.000000000000006394885e+01);
  Point2D b(1.000000000000000000000e+00,-5.000000000000006394884e+01);
  ASSERT_FALSE(SortPointsTest(a, b));
  ASSERT_TRUE(SortPointsTest(b, a));
}

TEST_F(SweepTriangulationTest, SortEdges) {
  Edge2D a(Point2D(3,-1), Point2D(2,1e-09));
  Edge2D b(Point2D(0,0), Point2D(0,10));
  ASSERT_FALSE(SortEdgesTest(true, a, b));
  ASSERT_TRUE(SortEdgesTest(true, b, a));

  ASSERT_TRUE(SortEdgesTest(false, a, b));
  ASSERT_FALSE(SortEdgesTest(false, b, a));
}

TEST_F(SweepTriangulationTest, SortEdgesSinglePoint) {
  Point2D top(-3.597122599785507190973e-14, 3.500000000000177635684e+00);
  Point2D bottom(-3.597122599785507190973e-14,1.500000000000035527137e+00);

  {
    Edge2D a(top, bottom);
    Edge2D b(bottom, bottom);
    ASSERT_FALSE(SortEdgesTest(true, a, b));
    ASSERT_FALSE(SortEdgesTest(true, b, a));
  }

  {
    Edge2D a(bottom, top);
    Edge2D b(bottom, bottom);
    ASSERT_FALSE(SortEdgesTest(true, a, b));
    ASSERT_FALSE(SortEdgesTest(true, b, a));
  }

  {
    Edge2D a(top, bottom);
    Edge2D b(top, top);
    ASSERT_FALSE(SortEdgesTest(true, a, b));
    ASSERT_FALSE(SortEdgesTest(true, b, a));
  }

  {
    Edge2D a(bottom, top);
    Edge2D b(top, top);
    ASSERT_FALSE(SortEdgesTest(true, a, b));
    ASSERT_FALSE(SortEdgesTest(true, b, a));
  }
}

/* TODO:
TEST_F(SweepTriangulationTest, TouchPoint) {
  vector<Point2D> loop;
  loop.push_back(Point2D(0, 0));
  loop.push_back(Point2D(10, 0));
  loop.push_back(Point2D(10, 2));
  loop.push_back(Point2D(5, 2));
  loop.push_back(Point2D(10, 10));
  loop.push_back(Point2D(0, 10));
  loop.push_back(Point2D(5, 2));
  loop.push_back(Point2D(0, 2));
  SweepTriangulation triangulate;
  triangulate.AddLoop(loop);

  vector<Triangle2D> output;
  triangulate.Triangulate(&output);
}
*/

TEST_F(SweepTriangulationTest, SortEdgesReg) {
  Point2D a(-9.499999999999857891453e+00,4.999999999998583355421e-01);
  Point2D b(-9.499999999999857891453e+00,1.499999999999859223720e+00);
  Point2D c(5.499999999999928945726e+00,1.500000000000924149646e+00);
  Point2D d(8.499999999999857891453e+00,5.000000000011364242880e-01);
  Point2D e(7.499999999999928945726e+00,1.500000000001065370014e+00);

  Edge2D e1(a, b);
  Edge2D e2(d, e);
  Edge2D p0(c, c);
  ASSERT_TRUE(SortEdgesTest(false, e2, e1));
  ASSERT_TRUE(SortEdgesTest(false, e2, p0));
  ASSERT_TRUE(SortEdgesTest(false, p0, e1));
  ASSERT_FALSE(SortEdgesTest(false, e1, e2));
  ASSERT_FALSE(SortEdgesTest(false, p0, e2));
  ASSERT_FALSE(SortEdgesTest(false, e1, p0));
}

TEST_F(SweepTriangulationTest, CloseSort) {
  vector<Point2D> loop;
  loop.push_back(Point2D(0, 0));
  loop.push_back(Point2D(1, 0.000000001));
  loop.push_back(Point2D(2, 0.000000001));
  loop.push_back(Point2D(3, -1));
  loop.push_back(Point2D(3, 10));
  loop.push_back(Point2D(0, 10));

  SweepTriangulation triangulate;
  triangulate.AddLoop(loop);

  vector<Triangle2D> output;
  triangulate.Triangulate(&output);

  vector<Triangle2D> expected = {
    Triangle2D(Point2D(0, 0), Point2D(1, 0.000000001), Point2D(0, 10)),
    Triangle2D(Point2D(0, 10), Point2D(1, 0.000000001), Point2D(2, 0.000000001)),
    Triangle2D(Point2D(0, 10), Point2D(2, 0.000000001), Point2D(3, 10)),
    Triangle2D(Point2D(2, 0.000000001), Point2D(3, -1), Point2D(3, 10)),
  };
  EXPECT_PRED_FORMAT2(AssertTrianglesEqual, expected, output);
}

TEST_F(SweepTriangulationTest, FlatEdge) {
  vector<Point2D> loop;
  loop.push_back(Point2D(-3.100000000000383337806e+01,1.650000000000312638804e+01));
  loop.push_back(Point2D(-3.300000000000426325641e+01,1.650000000000341415785e+01));
  loop.push_back(Point2D(-3.100000000000412470058e+01,1.450000000000326849658e+01));
  loop.push_back(Point2D(-1.700000000000113686838e+01,1.650000000000113686838e+01));

  SweepTriangulation triangulate;
  triangulate.AddLoop(loop);

  vector<Triangle2D> output;
  triangulate.Triangulate(&output);
  EXPECT_EQ(2, output.size());
}

TEST_F(SweepTriangulationTest, Oval) {
  vector<Point2D> loop;
  loop.push_back(Point2D(0.000000000000000000000e+00,-5.000000000000006394885e+01));
  loop.push_back(Point2D(1.000000000000000000000e+00,-5.000000000000006394885e+01));
  loop.push_back(Point2D(8.999999999999955591079e+00,-5.800000000000001421085e+01));
  loop.push_back(Point2D(9.999999999999946709295e+00,-5.800000000000000710543e+01));
  loop.push_back(Point2D(1.399999999999992894573e+01,-6.199999999999999289457e+01));
  loop.push_back(Point2D(1.499999999999992006394e+01,-6.199999999999997868372e+01));
  loop.push_back(Point2D(1.699999999999991118216e+01,-6.399999999999997157829e+01));
  loop.push_back(Point2D(1.799999999999991118216e+01,-6.399999999999997157829e+01));
  loop.push_back(Point2D(1.899999999999990052402e+01,-6.499999999999995736744e+01));
  loop.push_back(Point2D(1.999999999999989341859e+01,-6.499999999999994315658e+01));
  loop.push_back(Point2D(2.099999999999989341859e+01,-6.599999999999994315658e+01));
  loop.push_back(Point2D(2.199999999999988276045e+01,-6.599999999999994315658e+01));
  loop.push_back(Point2D(2.299999999999988276045e+01,-6.699999999999994315658e+01));
  loop.push_back(Point2D(2.399999999999987565502e+01,-6.699999999999992894573e+01));
  loop.push_back(Point2D(2.499999999999986499688e+01,-6.799999999999991473487e+01));
  loop.push_back(Point2D(2.599999999999986854959e+01,-6.799999999999991473487e+01));
  loop.push_back(Point2D(2.699999999999986144417e+01,-6.899999999999991473487e+01));
  loop.push_back(Point2D(2.899999999999984723331e+01,-6.899999999999990052402e+01));
  loop.push_back(Point2D(2.999999999999984012788e+01,-6.999999999999988631316e+01));
  loop.push_back(Point2D(3.999999999999978683718e+01,-6.999999999999982946974e+01));
  loop.push_back(Point2D(3.999999999999978683718e+01,-6.899999999999982946974e+01));
  loop.push_back(Point2D(4.199999999999978683718e+01,-6.899999999999981525889e+01));
  loop.push_back(Point2D(4.199999999999978683718e+01,-6.799999999999981525889e+01));
  loop.push_back(Point2D(4.299999999999977973175e+01,-6.799999999999980104803e+01));
  loop.push_back(Point2D(4.299999999999977973175e+01,-6.699999999999980104803e+01));
  loop.push_back(Point2D(4.399999999999976552090e+01,-6.699999999999980104803e+01));
  loop.push_back(Point2D(4.399999999999976552090e+01,-6.599999999999978683718e+01));
  loop.push_back(Point2D(4.499999999999975841547e+01,-6.599999999999978683718e+01));
  loop.push_back(Point2D(4.499999999999975841547e+01,-6.499999999999978683718e+01));
  loop.push_back(Point2D(4.599999999999976552090e+01,-6.499999999999978683718e+01));
  loop.push_back(Point2D(4.599999999999976552090e+01,-6.399999999999978683718e+01));
  loop.push_back(Point2D(4.699999999999975841547e+01,-6.399999999999977262632e+01));
  loop.push_back(Point2D(4.699999999999975841547e+01,-6.199999999999977262632e+01));
  loop.push_back(Point2D(4.799999999999975131004e+01,-6.199999999999976552090e+01));
  loop.push_back(Point2D(4.799999999999975131004e+01,-5.799999999999975841547e+01));
  loop.push_back(Point2D(4.899999999999974420462e+01,-5.799999999999974420462e+01));
  loop.push_back(Point2D(4.899999999999974420462e+01,-4.999999999999973709919e+01));
  loop.push_back(Point2D(4.999999999999972999376e+01,-4.999999999999972999376e+01));
  loop.push_back(Point2D(4.899999999999974420462e+01,-4.799999999999972999376e+01));
  loop.push_back(Point2D(4.899999999999974420462e+01,-3.999999999999972288833e+01));
  loop.push_back(Point2D(4.799999999999975131004e+01,-3.799999999999973709919e+01));
  loop.push_back(Point2D(4.799999999999975131004e+01,-3.399999999999972999376e+01));
  loop.push_back(Point2D(4.699999999999975841547e+01,-3.199999999999973709919e+01));
  loop.push_back(Point2D(4.699999999999975841547e+01,-2.999999999999973354647e+01));
  loop.push_back(Point2D(4.599999999999976552090e+01,-2.799999999999973354647e+01));
  loop.push_back(Point2D(4.599999999999976552090e+01,-2.699999999999973709919e+01));
  loop.push_back(Point2D(4.499999999999975841547e+01,-2.499999999999972999376e+01));
  loop.push_back(Point2D(4.499999999999975841547e+01,-2.399999999999972288833e+01));
  loop.push_back(Point2D(4.399999999999976552090e+01,-2.199999999999973354647e+01));
  loop.push_back(Point2D(4.399999999999976552090e+01,-2.099999999999972644105e+01));
  loop.push_back(Point2D(4.299999999999977973175e+01,-1.899999999999973709919e+01));
  loop.push_back(Point2D(4.299999999999977973175e+01,-1.799999999999974420462e+01));
  loop.push_back(Point2D(4.199999999999978683718e+01,-1.599999999999974420462e+01));
  loop.push_back(Point2D(4.199999999999978683718e+01,-1.499999999999974420462e+01));
  loop.push_back(Point2D(3.999999999999978683718e+01,-1.099999999999974420462e+01));
  loop.push_back(Point2D(3.999999999999978683718e+01,-9.999999999999744204615e+00));
  loop.push_back(Point2D(2.999999999999984012788e+01,1.000000000000021316282e+01));
  loop.push_back(Point2D(2.899999999999984723331e+01,1.100000000000021316282e+01));
  loop.push_back(Point2D(2.699999999999986144417e+01,1.500000000000020605739e+01));
  loop.push_back(Point2D(2.599999999999986854959e+01,1.600000000000019184654e+01));
  loop.push_back(Point2D(2.499999999999986499688e+01,1.800000000000020605739e+01));
  loop.push_back(Point2D(2.399999999999987565502e+01,1.900000000000019184654e+01));
  loop.push_back(Point2D(2.299999999999988276045e+01,2.100000000000017763568e+01));
  loop.push_back(Point2D(2.199999999999988276045e+01,2.200000000000017763568e+01));
  loop.push_back(Point2D(2.099999999999989341859e+01,2.400000000000017763568e+01));
  loop.push_back(Point2D(1.999999999999989341859e+01,2.500000000000016342483e+01));
  loop.push_back(Point2D(1.899999999999990052402e+01,2.700000000000017763568e+01));
  loop.push_back(Point2D(1.799999999999991118216e+01,2.800000000000016342483e+01));
  loop.push_back(Point2D(1.699999999999991118216e+01,3.000000000000016342483e+01));
  loop.push_back(Point2D(1.499999999999992006394e+01,3.200000000000016342483e+01));
  loop.push_back(Point2D(1.399999999999992894573e+01,3.400000000000013500312e+01));
  loop.push_back(Point2D(9.999999999999946709295e+00,3.800000000000012079227e+01));
  loop.push_back(Point2D(8.999999999999955591079e+00,4.000000000000012079227e+01));
  loop.push_back(Point2D(1.000000000000000000000e+00,4.800000000000007815970e+01));
  loop.push_back(Point2D(-0.000000000000000000000e+00,5.000000000000006394885e+01));
  loop.push_back(Point2D(-9.999999999999911182158e-01,5.000000000000006394885e+01));
  loop.push_back(Point2D(-8.999999999999946709295e+00,5.800000000000002131628e+01));
  loop.push_back(Point2D(-9.999999999999946709295e+00,5.800000000000002131628e+01));
  loop.push_back(Point2D(-1.399999999999992361666e+01,6.199999999999999289457e+01));
  loop.push_back(Point2D(-1.499999999999992006394e+01,6.199999999999999289457e+01));
  loop.push_back(Point2D(-1.699999999999990407673e+01,6.399999999999997868372e+01));
  loop.push_back(Point2D(-1.799999999999990052402e+01,6.399999999999997868372e+01));
  loop.push_back(Point2D(-1.899999999999989697130e+01,6.499999999999997157829e+01));
  loop.push_back(Point2D(-1.999999999999989341859e+01,6.499999999999994315658e+01));
  loop.push_back(Point2D(-2.099999999999988276045e+01,6.599999999999994315658e+01));
  loop.push_back(Point2D(-2.199999999999987920773e+01,6.599999999999994315658e+01));
  loop.push_back(Point2D(-2.299999999999987565502e+01,6.699999999999994315658e+01));
  loop.push_back(Point2D(-2.399999999999986854959e+01,6.699999999999994315658e+01));
  loop.push_back(Point2D(-2.499999999999986499688e+01,6.799999999999991473487e+01));
  loop.push_back(Point2D(-2.599999999999985789145e+01,6.799999999999991473487e+01));
  loop.push_back(Point2D(-2.699999999999985433874e+01,6.899999999999991473487e+01));
  loop.push_back(Point2D(-2.899999999999984368060e+01,6.899999999999988631316e+01));
  loop.push_back(Point2D(-2.999999999999984012788e+01,6.999999999999988631316e+01));
  loop.push_back(Point2D(-3.999999999999978683718e+01,6.999999999999982946974e+01));
  loop.push_back(Point2D(-3.999999999999978683718e+01,6.899999999999982946974e+01));
  loop.push_back(Point2D(-4.199999999999977973175e+01,6.899999999999980104803e+01));
  loop.push_back(Point2D(-4.199999999999977973175e+01,6.799999999999982946974e+01));
  loop.push_back(Point2D(-4.299999999999976552090e+01,6.799999999999980104803e+01));
  loop.push_back(Point2D(-4.299999999999976552090e+01,6.699999999999980104803e+01));
  loop.push_back(Point2D(-4.399999999999976552090e+01,6.699999999999980104803e+01));
  loop.push_back(Point2D(-4.399999999999976552090e+01,6.599999999999980104803e+01));
  loop.push_back(Point2D(-4.499999999999975841547e+01,6.599999999999980104803e+01));
  loop.push_back(Point2D(-4.499999999999975841547e+01,6.499999999999977262632e+01));
  loop.push_back(Point2D(-4.599999999999975131004e+01,6.499999999999977262632e+01));
  loop.push_back(Point2D(-4.599999999999975131004e+01,6.399999999999977973175e+01));
  loop.push_back(Point2D(-4.699999999999975131004e+01,6.399999999999977973175e+01));
  loop.push_back(Point2D(-4.699999999999975131004e+01,6.199999999999976552090e+01));
  loop.push_back(Point2D(-4.799999999999973709919e+01,6.199999999999976552090e+01));
  loop.push_back(Point2D(-4.799999999999973709919e+01,5.799999999999975131004e+01));
  loop.push_back(Point2D(-4.899999999999974420462e+01,5.799999999999975131004e+01));
  loop.push_back(Point2D(-4.899999999999974420462e+01,4.999999999999975131004e+01));
  loop.push_back(Point2D(-4.999999999999972999376e+01,4.999999999999973709919e+01));
  loop.push_back(Point2D(-4.899999999999974420462e+01,4.799999999999973709919e+01));
  loop.push_back(Point2D(-4.899999999999974420462e+01,3.999999999999973709919e+01));
  loop.push_back(Point2D(-4.799999999999973709919e+01,3.799999999999972288833e+01));
  loop.push_back(Point2D(-4.799999999999973709919e+01,3.399999999999972288833e+01));
  loop.push_back(Point2D(-4.699999999999975131004e+01,3.199999999999973709919e+01));
  loop.push_back(Point2D(-4.699999999999975131004e+01,2.999999999999973709919e+01));
  loop.push_back(Point2D(-4.599999999999975131004e+01,2.799999999999973709919e+01));
  loop.push_back(Point2D(-4.599999999999975131004e+01,2.699999999999972288833e+01));
  loop.push_back(Point2D(-4.499999999999975841547e+01,2.499999999999973709919e+01));
  loop.push_back(Point2D(-4.499999999999975841547e+01,2.399999999999973709919e+01));
  loop.push_back(Point2D(-4.399999999999976552090e+01,2.199999999999975131004e+01));
  loop.push_back(Point2D(-4.399999999999976552090e+01,2.099999999999973709919e+01));
  loop.push_back(Point2D(-4.299999999999976552090e+01,1.899999999999973709919e+01));
  loop.push_back(Point2D(-4.299999999999976552090e+01,1.799999999999973709919e+01));
  loop.push_back(Point2D(-4.199999999999977973175e+01,1.599999999999975131004e+01));
  loop.push_back(Point2D(-4.199999999999977973175e+01,1.499999999999975131004e+01));
  loop.push_back(Point2D(-3.999999999999978683718e+01,1.099999999999974420462e+01));
  loop.push_back(Point2D(-3.999999999999978683718e+01,9.999999999999751310042e+00));
  loop.push_back(Point2D(-2.999999999999984012788e+01,-1.000000000000021316282e+01));
  loop.push_back(Point2D(-2.899999999999984368060e+01,-1.100000000000020605739e+01));
  loop.push_back(Point2D(-2.699999999999985433874e+01,-1.500000000000019895197e+01));
  loop.push_back(Point2D(-2.599999999999985789145e+01,-1.600000000000019184654e+01));
  loop.push_back(Point2D(-2.499999999999986499688e+01,-1.800000000000018474111e+01));
  loop.push_back(Point2D(-2.399999999999986854959e+01,-1.900000000000018118840e+01));
  loop.push_back(Point2D(-2.299999999999987565502e+01,-2.100000000000017763568e+01));
  loop.push_back(Point2D(-2.199999999999987920773e+01,-2.200000000000017408297e+01));
  loop.push_back(Point2D(-2.099999999999988276045e+01,-2.400000000000017408297e+01));
  loop.push_back(Point2D(-1.999999999999989341859e+01,-2.500000000000016342483e+01));
  loop.push_back(Point2D(-1.899999999999989697130e+01,-2.700000000000016342483e+01));
  loop.push_back(Point2D(-1.799999999999990052402e+01,-2.800000000000015631940e+01));
  loop.push_back(Point2D(-1.699999999999990407673e+01,-3.000000000000015276669e+01));
  loop.push_back(Point2D(-1.499999999999992006394e+01,-3.200000000000014210855e+01));
  loop.push_back(Point2D(-1.399999999999992361666e+01,-3.400000000000014210855e+01));
  loop.push_back(Point2D(-9.999999999999946709295e+00,-3.800000000000011368684e+01));
  loop.push_back(Point2D(-8.999999999999946709295e+00,-4.000000000000011368684e+01));
  loop.push_back(Point2D(-9.999999999999911182158e-01,-4.800000000000007105427e+01));

  SweepTriangulation triangulate;
  triangulate.AddLoop(loop);

  vector<Triangle2D> output;
  triangulate.Triangulate(&output);
  EXPECT_EQ(loop.size() - 2, output.size());
}

TEST_F(SweepTriangulationTest, SubtleEdge) {
  vector<Point2D> loop;
  loop.push_back(Point2D(2.400000000000227728947e+01,-1.850000000000185096383e+01));
  loop.push_back(Point2D(3.300000000000412825329e+01,-1.850000000000312994075e+01));
  loop.push_back(Point2D(3.100000000000398614475e+01,-1.650000000000298783220e+01));
  loop.push_back(Point2D(3.300000000000412114787e+01,-1.650000000000312994075e+01));
  loop.push_back(Point2D(3.100000000000398259203e+01,-1.450000000000298960856e+01));
  loop.push_back(Point2D(3.300000000000454747351e+01,-1.450000000000341415785e+01));
  loop.push_back(Point2D(2.100000000000383693077e+01,-2.500000000002701838753e+00));

  SweepTriangulation triangulate;
  triangulate.AddLoop(loop);

  vector<Triangle2D> output;
  triangulate.Triangulate(&output);
  EXPECT_EQ(loop.size() - 2, output.size());
};

TEST_F(SweepTriangulationTest, SubtleEdge2) {
  vector<Point2D> loop;
  loop.push_back(Point2D(-8.999999999999712230192e+00,1.450000000000014210855e+01));
  loop.push_back(Point2D(-1.100000000000014210855e+01,1.450000000000042632564e+01));
  loop.push_back(Point2D(-1.500000000000071054274e+01,1.650000000000084909857e+01));
  loop.push_back(Point2D(-1.700000000000113686838e+01,1.650000000000113686838e+01));
  loop.push_back(Point2D(-2.100000000000142463819e+01,1.850000000000142463819e+01));
  loop.push_back(Point2D(-3.300000000000383693077e+01,1.850000000000312994075e+01));
  loop.push_back(Point2D(-3.100000000000383337806e+01,1.650000000000312638804e+01));
  loop.push_back(Point2D(-3.300000000000426325641e+01,1.650000000000341415785e+01));
  loop.push_back(Point2D(-3.100000000000412470058e+01,1.450000000000326849658e+01));
  loop.push_back(Point2D(-3.300000000000454747351e+01,1.450000000000354916097e+01));
  loop.push_back(Point2D(-2.400000000000398259203e+01,5.500000000002980726777e+00));

  SweepTriangulation triangulate;
  triangulate.AddLoop(loop);

  vector<Triangle2D> output;
  triangulate.Triangulate(&output);
  EXPECT_EQ(loop.size() - 2, output.size());
};

TEST_F(SweepTriangulationTest, BigHole) {
  SweepTriangulation triangulate;

  vector<Point2D> loop;
  loop.push_back(Point2D(1.200000000000042987836e+01,-1.450000000000042987836e+01));
  loop.push_back(Point2D(1.400000000000057198690e+01,-1.450000000000057198690e+01));
  loop.push_back(Point2D(1.800000000000114042109e+01,-1.650000000000099831254e+01));
  loop.push_back(Point2D(2.000000000000170885528e+01,-1.650000000000142463819e+01));
  loop.push_back(Point2D(2.400000000000227728947e+01,-1.850000000000185096383e+01));
  loop.push_back(Point2D(3.300000000000412825329e+01,-1.850000000000312994075e+01));
  loop.push_back(Point2D(3.100000000000398614475e+01,-1.650000000000298783220e+01));
  loop.push_back(Point2D(3.300000000000412114787e+01,-1.650000000000312994075e+01));
  loop.push_back(Point2D(3.100000000000398259203e+01,-1.450000000000298960856e+01));
  loop.push_back(Point2D(3.300000000000454747351e+01,-1.450000000000341415785e+01));
  loop.push_back(Point2D(2.100000000000383693077e+01,-2.500000000002701838753e+00));
  loop.push_back(Point2D(1.700000000000326849658e+01,-5.000000000022737367544e-01));
  loop.push_back(Point2D(1.500000000000312816439e+01,1.499999999997866595436e+00));
  loop.push_back(Point2D(1.100000000000284572366e+01,3.499999999998149036173e+00));
  loop.push_back(Point2D(9.000000000002705391466e+00,5.499999999998291144720e+00));
  loop.push_back(Point2D(-8.999999999999712230192e+00,1.450000000000014210855e+01));
  loop.push_back(Point2D(-1.100000000000014210855e+01,1.450000000000042632564e+01));
  loop.push_back(Point2D(-1.500000000000071054274e+01,1.650000000000084909857e+01));
  loop.push_back(Point2D(-1.700000000000113686838e+01,1.650000000000113686838e+01));
  loop.push_back(Point2D(-2.100000000000142463819e+01,1.850000000000142463819e+01));
  loop.push_back(Point2D(-3.300000000000383693077e+01,1.850000000000312994075e+01));
  loop.push_back(Point2D(-3.100000000000383337806e+01,1.650000000000312638804e+01));
  loop.push_back(Point2D(-3.300000000000426325641e+01,1.650000000000341415785e+01));
  loop.push_back(Point2D(-3.100000000000412470058e+01,1.450000000000326849658e+01));
  loop.push_back(Point2D(-3.300000000000454747351e+01,1.450000000000354916097e+01));
  loop.push_back(Point2D(-2.400000000000398259203e+01,5.500000000002980726777e+00));
  loop.push_back(Point2D(-2.000000000000341415785e+01,3.500000000002554401135e+00));
  loop.push_back(Point2D(-1.800000000000341060513e+01,1.500000000002557953849e+00));
  loop.push_back(Point2D(-1.400000000000283861823e+01,-4.999999999978701481496e-01));
  loop.push_back(Point2D(-1.200000000000270006240e+01,-2.499999999998012256697e+00));
  triangulate.AddLoop(loop);

  loop.clear();
  loop.push_back(Point2D(1.300000000000156674673e+01,-6.500000000001138644734e+00));
  loop.push_back(Point2D(1.500000000000170885528e+01,-8.500000000001282529638e+00));
  loop.push_back(Point2D(1.200000000000114042109e+01,-8.500000000000856203997e+00));
  loop.push_back(Point2D(8.000000000000571986902e+00,-6.500000000000429878355e+00));
  loop.push_back(Point2D(6.000000000000144773082e+00,-6.500000000000145661261e+00));
  loop.push_back(Point2D(-6.000000000001278976924e+00,-4.999999999990070165268e-01));
  loop.push_back(Point2D(-8.000000000001421085472e+00,1.500000000001133315664e+00));
  loop.push_back(Point2D(-1.200000000000198951966e+01,3.500000000001563194019e+00));
  loop.push_back(Point2D(-1.500000000000213162821e+01,6.500000000001701749852e+00));
  loop.push_back(Point2D(-1.300000000000170530257e+01,6.500000000001421085472e+00));
  loop.push_back(Point2D(-1.500000000000184385840e+01,8.500000000001559641305e+00));
  loop.push_back(Point2D(-9.000000000000710542736e+00,8.500000000000706990022e+00));
  loop.push_back(Point2D(-5.000000000000138555833e+00,6.500000000000280664381e+00));
  loop.push_back(Point2D(-2.999999999999714006549e+00,6.499999999999996447286e+00));
  loop.push_back(Point2D(3.000000000001000088901e+00,3.499999999999428013098e+00));
  loop.push_back(Point2D(5.000000000001139532912e+00,1.499999999999287680907e+00));
  loop.push_back(Point2D(9.000000000001708855280e+00,-5.000000000011386447341e-01));
  loop.push_back(Point2D(1.500000000000199484873e+01,-6.500000000001424638185e+00));
  triangulate.AddLoop(loop);

  vector<Triangle2D> output;
  triangulate.Triangulate(&output);
  EXPECT_EQ(48, output.size());
}


TEST_F(SweepTriangulationTest, DiagonalHarder) {
  vector<Point2D> loop;
  loop.push_back(Point2D(-2.000000000000106581410e+00,-2.500000000000177635684e+00));
  loop.push_back(Point2D(-1.000000000000106581410e+00,-3.500000000000177635684e+00));
  loop.push_back(Point2D(-1.000000000000106581410e+00,-1.500000000000177635684e+00));
  loop.push_back(Point2D(-3.597122599785507190973e-14,-2.500000000000248245868e+00));
  loop.push_back(Point2D(-3.597122599785507190973e-14,-5.000000000001070254996e-01));
  loop.push_back(Point2D(1.000000000000035527137e+00,-1.500000000000178079773e+00));
  loop.push_back(Point2D(1.000000000000035527137e+00,4.999999999999644728632e-01));
  loop.push_back(Point2D(2.000000000000106581410e+00,-5.000000000001070254996e-01));
  loop.push_back(Point2D(2.000000000000106581410e+00,1.500000000000035527137e+00));
  loop.push_back(Point2D(-3.597122599785507190973e-14,3.500000000000177635684e+00));
  loop.push_back(Point2D(-3.597122599785507190973e-14,1.500000000000035527137e+00));
  loop.push_back(Point2D(-1.000000000000106581410e+00,2.500000000000106581410e+00));
  loop.push_back(Point2D(-1.000000000000106581410e+00,4.999999999999640287740e-01));
  loop.push_back(Point2D(-2.000000000000106581410e+00,1.499999999999963584685e+00));

  SweepTriangulation triangulate;
  triangulate.AddLoop(loop);

  vector<Triangle2D> output;
  triangulate.Triangulate(&output);
  EXPECT_EQ(loop.size() - 2, output.size());
}

TEST_F(SweepTriangulationTest, DiagonalHarder2) {
  vector<Point2D> loop;
  loop.push_back(Point2D(-5.499999999999930722083e+00,-2.999999999999928945726e+00));
  loop.push_back(Point2D(-4.499999999999928945726e+00,-3.999999999999928945726e+00));
  loop.push_back(Point2D(-2.499999999999928945726e+00,-4.000000000000001776357e+00));
  loop.push_back(Point2D(-1.500000000000000000000e+00,-4.999999999999928945726e+00));
  loop.push_back(Point2D(-5.000000000000000000000e-01,-4.999999999999928945726e+00));
  loop.push_back(Point2D(5.000000000000000000000e-01,-5.999999999999928945726e+00));
  loop.push_back(Point2D(4.499999999999928945726e+00,-5.999999999999928945726e+00));
  loop.push_back(Point2D(5.499999999999928945726e+00,-6.999999999999928945726e+00));
  loop.push_back(Point2D(8.499999999999854338739e+00,-6.999999999999857891453e+00));
  loop.push_back(Point2D(8.499999999999854338739e+00,-5.999999999999928945726e+00));
  loop.push_back(Point2D(1.249999999999985789145e+01,-5.999999999999928945726e+00));
  loop.push_back(Point2D(1.249999999999985789145e+01,-5.000000000000001776357e+00));
  loop.push_back(Point2D(1.349999999999978683718e+01,-4.999999999999928945726e+00));
  loop.push_back(Point2D(1.349999999999978683718e+01,-3.999999999999928945726e+00));
  loop.push_back(Point2D(1.549999999999978683718e+01,-3.999999999999928945726e+00));
  loop.push_back(Point2D(1.549999999999978683718e+01,-3.000000000000001776357e+00));
  loop.push_back(Point2D(1.349999999999978683718e+01,-2.999999999999928945726e+00));
  loop.push_back(Point2D(1.249999999999985789145e+01,-3.000000000000001776357e+00));
  loop.push_back(Point2D(1.249999999999985789145e+01,-4.000000000000001776357e+00));
  loop.push_back(Point2D(9.499999999999857891453e+00,-3.999999999999928945726e+00));
  loop.push_back(Point2D(9.499999999999857891453e+00,-4.999999999999928945726e+00));
  loop.push_back(Point2D(2.500000000000000000000e+00,-5.000000000000001776357e+00));
  loop.push_back(Point2D(1.500000000000000000000e+00,-4.000000000000001776357e+00));
  loop.push_back(Point2D(-1.500000000000000000000e+00,-3.999999999999928945726e+00));
  loop.push_back(Point2D(-2.499999999999928945726e+00,-3.000000000000001776357e+00));
  loop.push_back(Point2D(-3.499999999999928945726e+00,-3.000000000000001776357e+00));

  SweepTriangulation triangulate;
  triangulate.AddLoop(loop);

  vector<Triangle2D> output;
  triangulate.Triangulate(&output);
  EXPECT_EQ(loop.size() - 2, output.size());
}

TEST_F(SweepTriangulationTest, DiagonalHarder3) {
  vector<Point2D> loop;
  loop.push_back(Point2D(-1.549999999999978683718e+01,-3.500000000000781152920e+00));
  loop.push_back(Point2D(-1.449999999999978683718e+01,-3.500000000000781152920e+00));
  loop.push_back(Point2D(-1.449999999999978683718e+01,-2.500000000000710098647e+00));
  loop.push_back(Point2D(-1.349999999999978683718e+01,-2.500000000000639044373e+00));
  loop.push_back(Point2D(-1.349999999999978683718e+01,-1.500000000000567990099e+00));
  loop.push_back(Point2D(-1.249999999999978683718e+01,-1.500000000000496935826e+00));
  loop.push_back(Point2D(-1.249999999999978683718e+01,-5.000000000004973799150e-01));
  loop.push_back(Point2D(-1.149999999999985789145e+01,-5.000000000003557154571e-01));
  loop.push_back(Point2D(-1.149999999999985789145e+01,4.999999999997153388165e-01));
  loop.push_back(Point2D(-9.499999999999857891453e+00,4.999999999998583355421e-01));
  loop.push_back(Point2D(-9.499999999999857891453e+00,1.499999999999859223720e+00));
  loop.push_back(Point2D(-7.499999999999857891453e+00,1.499999999999999555911e+00));
  loop.push_back(Point2D(-7.499999999999857891453e+00,2.500000000000071498363e+00));
  loop.push_back(Point2D(1.500000000000000000000e+00,2.500000000000710098647e+00));
  loop.push_back(Point2D(2.500000000000000000000e+00,1.500000000000710098647e+00));
  loop.push_back(Point2D(4.499999999999928945726e+00,1.500000000000924149646e+00));
  loop.push_back(Point2D(5.499999999999928945726e+00,5.000000000009250378241e-01));
  loop.push_back(Point2D(7.499999999999928945726e+00,5.000000000009943157409e-01));
  loop.push_back(Point2D(8.499999999999857891453e+00,-4.999999999989350740748e-01));
  loop.push_back(Point2D(9.499999999999857891453e+00,-4.999999999988631316228e-01));
  loop.push_back(Point2D(1.049999999999985789145e+01,-1.499999999998862243444e+00));
  loop.push_back(Point2D(1.149999999999985789145e+01,-1.499999999998789856903e+00));
  loop.push_back(Point2D(1.249999999999985433874e+01,-2.499999999998789412814e+00));
  loop.push_back(Point2D(1.349999999999978683718e+01,-2.499999999998650412891e+00));
  loop.push_back(Point2D(1.449999999999978683718e+01,-3.499999999998649524713e+00));
  loop.push_back(Point2D(1.549999999999978683718e+01,-3.499999999998649524713e+00));
  loop.push_back(Point2D(1.349999999999978683718e+01,-1.499999999998651301070e+00));
  loop.push_back(Point2D(1.249999999999985433874e+01,-1.499999999998717914451e+00));
  loop.push_back(Point2D(1.149999999999985789145e+01,-4.999999999987188026296e-01));
  loop.push_back(Point2D(1.049999999999985789145e+01,-4.999999999987907450816e-01));
  loop.push_back(Point2D(9.499999999999857891453e+00,5.000000000012083667400e-01));
  loop.push_back(Point2D(8.499999999999857891453e+00,5.000000000011364242880e-01));
  loop.push_back(Point2D(7.499999999999928945726e+00,1.500000000001065370014e+00));
  loop.push_back(Point2D(5.499999999999928945726e+00,1.500000000000924149646e+00));
  loop.push_back(Point2D(4.499999999999928945726e+00,2.500000000000923261467e+00));
  loop.push_back(Point2D(2.500000000000000000000e+00,2.500000000000782041099e+00));
  loop.push_back(Point2D(1.500000000000000000000e+00,3.500000000000781152920e+00));
  loop.push_back(Point2D(-8.499999999999857891453e+00,3.500000000000071498363e+00));
  loop.push_back(Point2D(-8.499999999999857891453e+00,2.500000000000000444089e+00));
  loop.push_back(Point2D(-1.049999999999985789145e+01,2.499999999999859223720e+00));
  loop.push_back(Point2D(-1.049999999999985789145e+01,1.499999999999858335542e+00));
  loop.push_back(Point2D(-1.249999999999978683718e+01,1.499999999999644284543e+00));
  loop.push_back(Point2D(-1.249999999999978683718e+01,4.999999999995732302693e-01));
  loop.push_back(Point2D(-1.349999999999978683718e+01,4.999999999995030641742e-01));
  loop.push_back(Point2D(-1.349999999999978683718e+01,-5.000000000004969358258e-01));
  loop.push_back(Point2D(-1.449999999999978683718e+01,-5.000000000005679900994e-01));
  loop.push_back(Point2D(-1.449999999999978683718e+01,-1.500000000000639044373e+00));
  loop.push_back(Point2D(-1.549999999999978683718e+01,-1.500000000000710098647e+00));

  SweepTriangulation triangulate;
  triangulate.AddLoop(loop);

  vector<Triangle2D> output;
  triangulate.Triangulate(&output);
  EXPECT_EQ(loop.size() - 2, output.size());
}

TEST_F(SweepTriangulationTest, GiantHole) {
  SweepTriangulation triangulate;

  vector<Point2D> loop;
  loop.push_back(Point2D(-9.499999999999218402991e+00,-2.850000000000298427949e+01));
  loop.push_back(Point2D(-7.499999999999360511538e+00,-2.850000000000270006240e+01));
  loop.push_back(Point2D(-3.499999999999786837179e+00,-3.250000000000227373675e+01));
  loop.push_back(Point2D(-1.499999999999928945726e+00,-3.250000000000198951966e+01));
  loop.push_back(Point2D(4.999999999999289457264e-01,-3.450000000000184741111e+01));
  loop.push_back(Point2D(2.499999999999785060822e+00,-3.450000000000170530257e+01));
  loop.push_back(Point2D(4.499999999999500843728e+00,-3.650000000000142108547e+01));
  loop.push_back(Point2D(8.499999999999214850277e+00,-3.650000000000085265128e+01));
  loop.push_back(Point2D(1.049999999999907629444e+01,-3.850000000000071054274e+01));
  loop.push_back(Point2D(2.849999999999751310042e+01,-3.849999999999815258889e+01));
  loop.push_back(Point2D(2.849999999999751310042e+01,-3.649999999999801048034e+01));
  loop.push_back(Point2D(3.249999999999722177790e+01,-3.649999999999744204615e+01));
  loop.push_back(Point2D(3.249999999999722177790e+01,-3.449999999999744204615e+01));
  loop.push_back(Point2D(3.449999999999694466624e+01,-3.449999999999701572051e+01));
  loop.push_back(Point2D(3.449999999999694466624e+01,-3.249999999999687361196e+01));
  loop.push_back(Point2D(3.649999999999679545226e+01,-3.249999999999658228944e+01));
  loop.push_back(Point2D(3.649999999999679545226e+01,-2.849999999999644373361e+01));
  loop.push_back(Point2D(3.849999999999666044914e+01,-2.849999999999616662194e+01));
  loop.push_back(Point2D(3.849999999999666044914e+01,-9.499999999995171862111e+00));
  loop.push_back(Point2D(3.649999999999679545226e+01,-7.499999999995310417944e+00));
  loop.push_back(Point2D(3.649999999999679545226e+01,-3.499999999995029753563e+00));
  loop.push_back(Point2D(3.449999999999694466624e+01,-1.499999999995175414824e+00));
  loop.push_back(Point2D(3.449999999999694466624e+01,5.000000000051159076975e-01));
  loop.push_back(Point2D(3.249999999999722177790e+01,2.500000000004831690603e+00));
  loop.push_back(Point2D(3.249999999999722177790e+01,4.500000000004831690603e+00));
  loop.push_back(Point2D(2.849999999999751310042e+01,8.500000000004540368082e+00));
  loop.push_back(Point2D(2.849999999999751310042e+01,1.050000000000454747351e+01));
  loop.push_back(Point2D(1.049999999999907629444e+01,2.850000000000298427949e+01));
  loop.push_back(Point2D(8.499999999999214850277e+00,2.850000000000283506552e+01));
  loop.push_back(Point2D(4.499999999999500843728e+00,3.250000000000255084842e+01));
  loop.push_back(Point2D(2.499999999999785060822e+00,3.250000000000227373675e+01));
  loop.push_back(Point2D(4.999999999999289457264e-01,3.450000000000213162821e+01));
  loop.push_back(Point2D(-1.499999999999928945726e+00,3.450000000000169819714e+01));
  loop.push_back(Point2D(-3.499999999999786837179e+00,3.650000000000154898316e+01));
  loop.push_back(Point2D(-7.499999999999360511538e+00,3.650000000000085265128e+01));
  loop.push_back(Point2D(-9.499999999999218402991e+00,3.850000000000071054274e+01));
  loop.push_back(Point2D(-2.849999999999751310042e+01,3.849999999999801048034e+01));
  loop.push_back(Point2D(-2.849999999999751310042e+01,3.649999999999786837179e+01));
  loop.push_back(Point2D(-3.249999999999722888333e+01,3.649999999999744204615e+01));
  loop.push_back(Point2D(-3.249999999999722888333e+01,3.449999999999729993760e+01));
  loop.push_back(Point2D(-3.449999999999722888333e+01,3.449999999999715782906e+01));
  loop.push_back(Point2D(-3.449999999999722888333e+01,3.249999999999700861508e+01));
  loop.push_back(Point2D(-3.649999999999694466624e+01,3.249999999999673150342e+01));
  loop.push_back(Point2D(-3.649999999999694466624e+01,2.849999999999644018089e+01));
  loop.push_back(Point2D(-3.849999999999666044914e+01,2.849999999999602096068e+01));
  loop.push_back(Point2D(-3.849999999999666044914e+01,1.049999999999502620085e+01));
  loop.push_back(Point2D(-3.649999999999694466624e+01,8.499999999995303312517e+00));
  loop.push_back(Point2D(-3.649999999999694466624e+01,4.499999999995026200850e+00));
  loop.push_back(Point2D(-3.449999999999722888333e+01,2.499999999995306865230e+00));
  loop.push_back(Point2D(-3.449999999999722888333e+01,4.999999999953068652303e-01));
  loop.push_back(Point2D(-3.249999999999722888333e+01,-1.500000000004693134770e+00));
  loop.push_back(Point2D(-3.249999999999722888333e+01,-3.500000000004835243317e+00));
  loop.push_back(Point2D(-2.849999999999751310042e+01,-7.500000000004547473509e+00));
  loop.push_back(Point2D(-2.849999999999751310042e+01,-9.500000000004689582056e+00));
  triangulate.AddLoop(loop);

  loop.clear();
  loop.push_back(Point2D(-9.499999999999218402991e+00,-2.450000000000270006240e+01));
  loop.push_back(Point2D(-1.149999999999907629444e+01,-2.450000000000298427949e+01));
  loop.push_back(Point2D(-2.449999999999808153461e+01,-1.150000000000397903932e+01));
  loop.push_back(Point2D(-2.449999999999808153461e+01,-9.500000000003836930773e+00));
  loop.push_back(Point2D(-2.849999999999751310042e+01,-5.500000000004405364962e+00));
  loop.push_back(Point2D(-2.849999999999751310042e+01,-3.500000000004405364962e+00));
  loop.push_back(Point2D(-3.049999999999751310042e+01,-1.500000000004405364962e+00));
  loop.push_back(Point2D(-3.049999999999751310042e+01,4.999999999957367435854e-01));
  loop.push_back(Point2D(-3.249999999999722888333e+01,2.499999999995448973777e+00));
  loop.push_back(Point2D(-3.249999999999722888333e+01,4.499999999995594635038e+00));
  loop.push_back(Point2D(-3.449999999999722888333e+01,6.499999999995594635038e+00));
  loop.push_back(Point2D(-3.449999999999722888333e+01,1.249999999999587174671e+01));
  loop.push_back(Point2D(-3.649999999999694466624e+01,1.449999999999558752961e+01));
  loop.push_back(Point2D(-3.649999999999694466624e+01,2.249999999999616306923e+01));
  loop.push_back(Point2D(-3.449999999999722888333e+01,2.249999999999644018089e+01));
  loop.push_back(Point2D(-3.449999999999722888333e+01,2.849999999999687361196e+01));
  loop.push_back(Point2D(-3.249999999999722888333e+01,2.849999999999700861508e+01));
  loop.push_back(Point2D(-3.249999999999722888333e+01,3.049999999999700861508e+01));
  loop.push_back(Point2D(-3.049999999999751310042e+01,3.049999999999742783530e+01));
  loop.push_back(Point2D(-3.049999999999751310042e+01,3.249999999999757704927e+01));
  loop.push_back(Point2D(-2.849999999999751310042e+01,3.249999999999773336867e+01));
  loop.push_back(Point2D(-2.849999999999751310042e+01,3.449999999999786837179e+01));
  loop.push_back(Point2D(-2.449999999999808153461e+01,3.449999999999857891453e+01));
  loop.push_back(Point2D(-2.449999999999808153461e+01,3.649999999999872102308e+01));
  loop.push_back(Point2D(-1.149999999999907629444e+01,3.650000000000056843419e+01));
  loop.push_back(Point2D(-9.499999999999218402991e+00,3.450000000000071054274e+01));
  loop.push_back(Point2D(-5.499999999999644728632e+00,3.450000000000113686838e+01));
  loop.push_back(Point2D(-3.499999999999786837179e+00,3.250000000000127187150e+01));
  loop.push_back(Point2D(-1.499999999999928945726e+00,3.250000000000170530257e+01));
  loop.push_back(Point2D(4.999999999999289457264e-01,3.050000000000184741111e+01));
  loop.push_back(Point2D(2.499999999999785060822e+00,3.050000000000198241423e+01));
  loop.push_back(Point2D(4.499999999999500843728e+00,2.850000000000225952590e+01));
  loop.push_back(Point2D(6.499999999999360511538e+00,2.850000000000269295697e+01));
  loop.push_back(Point2D(1.249999999999893418590e+01,2.250000000000312638804e+01));
  loop.push_back(Point2D(1.449999999999864996880e+01,2.250000000000340349970e+01));
  loop.push_back(Point2D(2.249999999999807798190e+01,1.450000000000397903932e+01));
  loop.push_back(Point2D(2.249999999999807798190e+01,1.250000000000397903932e+01));
  loop.push_back(Point2D(2.849999999999751310042e+01,6.500000000004540368082e+00));
  loop.push_back(Point2D(2.849999999999751310042e+01,4.500000000004256150987e+00));
  loop.push_back(Point2D(3.049999999999737099188e+01,2.500000000004398259534e+00));
  loop.push_back(Point2D(3.049999999999737099188e+01,5.000000000044089176754e-01));
  loop.push_back(Point2D(3.249999999999722177790e+01,-1.499999999995445421064e+00));
  loop.push_back(Point2D(3.249999999999722177790e+01,-3.499999999995736743585e+00));
  loop.push_back(Point2D(3.449999999999694466624e+01,-5.499999999995452526491e+00));
  loop.push_back(Point2D(3.449999999999694466624e+01,-9.499999999995452526491e+00));
  loop.push_back(Point2D(3.649999999999679545226e+01,-1.149999999999531041794e+01));
  loop.push_back(Point2D(3.649999999999679545226e+01,-2.449999999999616306923e+01));
  loop.push_back(Point2D(3.449999999999694466624e+01,-2.449999999999645083903e+01));
  loop.push_back(Point2D(3.449999999999694466624e+01,-2.849999999999658939487e+01));
  loop.push_back(Point2D(3.249999999999722177790e+01,-2.849999999999701216780e+01));
  loop.push_back(Point2D(3.249999999999722177790e+01,-3.049999999999715782906e+01));
  loop.push_back(Point2D(3.049999999999737099188e+01,-3.049999999999744559886e+01));
  loop.push_back(Point2D(3.049999999999737099188e+01,-3.249999999999758415470e+01));
  loop.push_back(Point2D(2.849999999999751310042e+01,-3.249999999999772626325e+01));
  loop.push_back(Point2D(2.849999999999751310042e+01,-3.449999999999786837179e+01));
  loop.push_back(Point2D(2.249999999999807798190e+01,-3.449999999999886313162e+01));
  loop.push_back(Point2D(2.249999999999807798190e+01,-3.649999999999886313162e+01));
  loop.push_back(Point2D(1.449999999999864996880e+01,-3.650000000000000000000e+01));
  loop.push_back(Point2D(1.249999999999893418590e+01,-3.450000000000028421709e+01));
  loop.push_back(Point2D(6.499999999999360511538e+00,-3.450000000000099475983e+01));
  loop.push_back(Point2D(4.499999999999500843728e+00,-3.250000000000113686838e+01));
  loop.push_back(Point2D(2.499999999999785060822e+00,-3.250000000000156319402e+01));
  loop.push_back(Point2D(4.999999999999289457264e-01,-3.050000000000170530257e+01));
  loop.push_back(Point2D(-1.499999999999928945726e+00,-3.050000000000198951966e+01));
  loop.push_back(Point2D(-3.499999999999786837179e+00,-2.850000000000213162821e+01));
  loop.push_back(Point2D(-5.499999999999644728632e+00,-2.850000000000227373675e+01));
  triangulate.AddLoop(loop);

  vector<Triangle2D> output;
  triangulate.Triangulate(&output);
  EXPECT_EQ(120, output.size());
}

}  // anonymous namespace
}  // namespace printer
