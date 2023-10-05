/*
 * Copyright (c) 2019, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Tests for LineSegment3.
//
///////////////////////////////////////////////////////////////////////////////

#include <ilqgames_planning/line_segment3.h>
#include <ilqgames_planning/types.h>

#include <gtest/gtest.h>
#include <math.h>

using namespace ilqgames_planning;

double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// Check that we error out on construction if the line segment is degenerate.
TEST(LineSegment3Test, DiesIfDegenerate) {
    ASSERT_DEATH(LineSegment3(Point3::Zero(), Point3::Zero()), "Check failed");
}

// Test if the norm and the versor of the segment are computed correctly
TEST(LineSegment3Test, LengthAndDirectionWorks) {
    const Point3 lower = Point3(0.0, -1.0, 1.0);
    const Point3 upper = Point3(0.0, 1.0, -2.0);
    const LineSegment3 segment(lower, upper);

    const float length = segment.Length();
    const Point3 unit_direction = segment.UnitDirection();
    const Point3 expected_unit_direction = Point3(0.0, 0.5547002, -0.8320503);

    EXPECT_NEAR(length, 3.6055513, ilqgames::constants::kSmallNumber);

    for (int i = 0; i < 3; i++)
        EXPECT_NEAR(unit_direction(i), expected_unit_direction(i), ilqgames::constants::kSmallNumber);
}

// Test if orientation of the segment (Z -Y Euler Angles in radians) is computed correctly
TEST(LineSegment3Test, OrientationWorks) {
    const Point3 lower = Point3(0.0, -1.0, 1.0);
    const Point3 upper = Point3(0.0, 1.0, -2.0);
    const LineSegment3 segment(lower, upper);

    std::vector<float> heading = segment.Heading();
    EXPECT_NEAR(heading.at(0), deg2rad(90.0), ilqgames::constants::kSmallNumber); // theta_x
    EXPECT_NEAR(heading.at(1), 0.9827937, ilqgames::constants::kSmallNumber); // theta_y
}

// Check that we find the correct closest point.
TEST(LineSegment3Test, ClosestPointWorks) {
    const Point3 lower = Point3(0.0, -1.0, 1.0);
    const Point3 upper = Point3(0.0, 1.0, -2.0);
    const LineSegment3 segment(lower, upper);
    float squared_distance;
    bool is_endpoint;

    // Pick points and check closest points/distances.
    Point3 query(1.0, -2.0, 3.0);
    Point3 closest = segment.ClosestPoint(query, &is_endpoint, &squared_distance);
    EXPECT_TRUE(is_endpoint);
    EXPECT_TRUE(closest.isApprox(lower));
    EXPECT_NEAR(squared_distance, 6.0, ilqgames::constants::kSmallNumber);

    query << 1.0, 2.0, -2.0;
    closest = segment.ClosestPoint(query, &is_endpoint, &squared_distance);
    EXPECT_TRUE(is_endpoint);
    EXPECT_TRUE(closest.isApprox(upper));
    EXPECT_NEAR(squared_distance, 2.0, ilqgames::constants::kSmallNumber);

    // The query point is closer to an interior point than to segment extrema (see MATLAB file test_line_segment3.mlx)
    query << 0.0, 1.0, 2.0 / 3.0;
    closest = segment.ClosestPoint(query, &is_endpoint, &squared_distance);
    EXPECT_FALSE(is_endpoint);
    EXPECT_LT((closest-lower).squaredNorm(), (closest-upper).squaredNorm());
    EXPECT_NEAR(closest.x(), 0.0, ilqgames::constants::kSmallNumber);
    EXPECT_NEAR(closest.y(), -0.230770000000000, ilqgames::constants::kSmallNumber);
    EXPECT_NEAR(closest.z(), -0.153845000000000, ilqgames::constants::kSmallNumber);
    EXPECT_NEAR(closest.squaredNorm(), 0.0769231, ilqgames::constants::kSmallNumber);
    EXPECT_NEAR(squared_distance, 2.1880342, ilqgames::constants::kSmallNumber);
}
