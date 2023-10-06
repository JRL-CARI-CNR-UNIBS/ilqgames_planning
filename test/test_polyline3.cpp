///////////////////////////////////////////////////////////////////////////////
//
// Tests for Polyline3.
//
///////////////////////////////////////////////////////////////////////////////

#include <ilqgames_planning/polyline3.h>
#include <ilqgames_planning/types.h>

#include <gtest/gtest.h>
#include <math.h>

using namespace ilqgames_planning;

// Check that we find the correct closest point.
TEST(Polyline3Test, ClosestPointWorks) {
  const Point3 p1(0.0, -1.0, 1.0);
  const Point3 p2(0.0, 1.0, -2.0);
  const Point3 p3(2.0, 1.0, 0.0);
  const Polyline3 polyline({p1, p2, p3});
  float squared_distance;
  LineSegment3 segment(p1, p2);
  bool is_vertex;

  Point3 query(1.0, -2.0, 1.0);
  Point3 closest = polyline.ClosestPoint(query, &is_vertex, &segment,
                                         &squared_distance);
  EXPECT_TRUE(is_vertex);
  EXPECT_TRUE(closest.isApprox(p1));
  EXPECT_EQ(segment, polyline.Segments().at(0));
  EXPECT_NEAR(squared_distance, 2.0, ilqgames::constants::kSmallNumber);

  query << 0.5, 0.0, -0.5;
  closest = polyline.ClosestPoint(query, &is_vertex, &segment,
                                  &squared_distance);
  EXPECT_FALSE(is_vertex);
  EXPECT_EQ(segment, polyline.Segments().at(0));
  EXPECT_NEAR(squared_distance, 0.5 * 0.5, ilqgames::constants::kSmallNumber);

  query << 1.5, 0.0, -0.5;
  closest = polyline.ClosestPoint(query, &is_vertex, &segment,
                                  &squared_distance);
  EXPECT_FALSE(is_vertex);
  EXPECT_TRUE(closest.isApprox(Point3(1.5, 1.0, -0.5)));
  EXPECT_EQ(segment, polyline.Segments().at(1));
  EXPECT_NEAR(squared_distance, 1.0, ilqgames::constants::kSmallNumber);

  query << 3.0, 0.0, 0.0;
  closest = polyline.ClosestPoint(query, &is_vertex, &segment,
                                  &squared_distance);
  EXPECT_TRUE(is_vertex);
  EXPECT_TRUE(closest.isApprox(p3));
  EXPECT_EQ(segment, polyline.Segments().at(1));
  EXPECT_NEAR(squared_distance, 2.0, ilqgames::constants::kSmallNumber);

  query << 0.5, 2.0, -1.5;
  closest = polyline.ClosestPoint(query, &is_vertex, &segment,
                                  &squared_distance);
  EXPECT_FALSE(is_vertex);
  EXPECT_TRUE(closest.isApprox(Point3(0.5, 1.0, -1.5)));
  EXPECT_EQ(segment, polyline.Segments().at(1));
  EXPECT_NEAR(squared_distance, 1.0, ilqgames::constants::kSmallNumber);

}
