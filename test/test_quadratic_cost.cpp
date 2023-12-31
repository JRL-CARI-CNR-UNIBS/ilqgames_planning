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
// Tests for QuadraticCost.
//
///////////////////////////////////////////////////////////////////////////////

#include <ilqgames/cost/quadratic_cost.h>
#include <ilqgames_planning/types.h>
#include <ilqgames_planning/polyline3.h>
#include <ilqgames_planning/quadratic_polyline3_cost.h>

#include <gtest/gtest.h>
#include <math.h>

using namespace ilqgames;

// Check that the quadratic cost applies in the correct dimension.
TEST(QuadraticCostTest, EvaluatesInCorrectDimension) {
  constexpr float kCostWeight = 5.0;
  constexpr Dimension kDimension = 3;
  const QuadraticCost cost(kCostWeight, kDimension);

  // Try vectors of different lengths.
  ASSERT_DEATH(cost.Evaluate(VectorXf::Random(2)), "Check failed");

  VectorXf input = VectorXf::Random(5);
  EXPECT_NEAR(cost.Evaluate(input),
              kCostWeight * 0.5 * input(kDimension) * input(kDimension),
              constants::kSmallNumber);

  input = VectorXf::Random(10);
  EXPECT_NEAR(cost.Evaluate(input),
              kCostWeight * 0.5 * input(kDimension) * input(kDimension),
              constants::kSmallNumber);
}

// Check that the quadratic cost applies in all dimensions when constructed with dimension < 0.
TEST(QuadraticCostTest, EvaluatesInAllDimensions) {
  constexpr float kCostWeight = 5.0;
  constexpr Dimension kDimension = -1;
  const QuadraticCost cost(kCostWeight, kDimension);

  // Try vectors of different lengths.
  VectorXf input = VectorXf::Random(5);
  EXPECT_NEAR(cost.Evaluate(input), kCostWeight * 0.5 * input.squaredNorm(),
              constants::kSmallNumber);

  input = VectorXf::Random(10);
  EXPECT_NEAR(cost.Evaluate(input), kCostWeight * 0.5 * input.squaredNorm(),
              constants::kSmallNumber);
}

// Check that we quadraticize correctly when dimension >= 0.
TEST(QuadraticCostTest, QuadraticizeInSingleDimension) {
  constexpr float kCostWeight = 5.0;
  constexpr Dimension kCostDimension = 3;
  constexpr Dimension kVectorDimension = 5;
  const QuadraticCost cost(kCostWeight, kCostDimension);

  MatrixXf hess = MatrixXf::Zero(kVectorDimension, kVectorDimension);
  VectorXf grad = VectorXf::Zero(kVectorDimension);

  // Make sure we error out if the dimension is incorrect.
  ASSERT_DEATH(cost.Quadraticize(VectorXf::Random(2), &hess, &grad),
               "Check failed");

  // Compute gradient and Hessian at a couple points.
  VectorXf input = VectorXf::Random(kVectorDimension);
  const float deriv1 = kCostWeight * input(kCostDimension);
  cost.Quadraticize(input, &hess, &grad);
  EXPECT_NEAR(hess(kCostDimension, kCostDimension), kCostWeight,
              constants::kSmallNumber);
  EXPECT_NEAR(hess.norm(), std::abs(kCostWeight), constants::kSmallNumber);
  EXPECT_NEAR(grad(kCostDimension), deriv1, constants::kSmallNumber);
  EXPECT_NEAR(grad.norm(), std::abs(deriv1), constants::kSmallNumber);

  input = VectorXf::Random(kVectorDimension);
  const float deriv2 = kCostWeight * input(kCostDimension);
  cost.Quadraticize(input, &hess, &grad);
  EXPECT_NEAR(hess(kCostDimension, kCostDimension), 2.0 * kCostWeight,
              constants::kSmallNumber);
  EXPECT_NEAR(hess.norm(), 2.0 * std::abs(kCostWeight),
              constants::kSmallNumber);
  EXPECT_NEAR(grad(kCostDimension), deriv1 + deriv2, constants::kSmallNumber);
  EXPECT_NEAR(grad.norm(), std::abs(deriv1 + deriv2), constants::kSmallNumber);
}

using namespace ilqgames_planning;

// Check that the polyline3 quadratic cost evaluates correctly for a 3D Point system.
TEST(QuadraticPolyline3Cost, EvaluatesCorrectlyFor3DPoint) {
  constexpr float kCostWeight = 5.0;
  constexpr std::tuple<ilqgames::Dimension, ilqgames::Dimension, ilqgames::Dimension> kDimension(0, 2, 4); // the state for a 3D point is (x, vx, y, vy, z, vz). Here we consider the 3D position, hence those indexes
  const Point3 way1(0.0, -1.0, 1.0);
  const Point3 way2(0.0, 1.0, -2.0);
  const Point3 way3(2.0, 1.0, 0.0);
  const PointList3 waypts = {way1, way2, way3};
  const Polyline3 polyline3(waypts);

  const QuadraticPolyline3Cost cost(kCostWeight, polyline3, kDimension, "polyline3_cost");

  // When the closest point is a vertex of the polyline
  VectorXf query(6);
  query << 1.0, 0.0, -2.0, 0.0, 1.0, 0.0;  // state: (x, vx, y, vy, z, vz)
  EXPECT_NEAR(cost.Evaluate(query), 5.0000000, ilqgames::constants::kSmallNumber);

  // When the closest point is NOT a vertex of the polyline
  query << 0.5, 0.0, 0.0, 0.0, -0.5, 0.0;  // state: (x, vx, y, vy, z, vz)
  EXPECT_NEAR(cost.Evaluate(query), 0.6250000, ilqgames::constants::kSmallNumber);
}
