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
// Line segment in 3D.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_PLANNING_LINE_SEGMENT3_H
#define ILQGAMES_PLANNING_LINE_SEGMENT3_H

#include <ilqgames/utils/types.h>
#include <ilqgames_planning/types.h>

#include <glog/logging.h>

namespace ilqgames_planning {

class LineSegment3 {
    public:
      ~LineSegment3() {}
      LineSegment3(const Point3& point1, const Point3& point2)
          : p1_(point1),
            p2_(point2),
            length_((point1 - point2).norm()),
            unit_direction_((point2 - point1) / length_) {
        CHECK_GT(length_, ilqgames::constants::kSmallNumber);
      }

      // Accessors.
      float Length() const { return length_; }
      const Point3& FirstPoint() const { return p1_; }
      const Point3& SecondPoint() const { return p2_; }
      const Point3& UnitDirection() const { return unit_direction_; }
      Eigen::Matrix3f Heading() const { // https://stackoverflow.com/questions/35613741/convert-2-3d-points-to-directional-vectors-to-euler-angles
        float r = length_; // r = SQRT(v_x^2+v_y^2+v_z^2)
        float phi = std::atan2(UnitDirection().z(), UnitDirection().x()); // ψ = ATAN2(v_z, v_x)
        float theta = std::atan2(UnitDirection().y(),
                                 std::sqrt((p2_(0)-p1_(0))*(p2_(0)-p1_(0)) + (p2_(0)-p1_(2))*(p2_(0)-p1_(2)))); // θ = ATAN2(v_y, SQRT(v_x^2+v_z^2))

        Eigen::Vector3f j(std::cos(phi)*std::cos(theta), std::sin(theta), std::sin(phi)*std::cos(theta)); // the along direction vector
        Eigen::Vector3f i(std::sin(phi), 0.0, -std::cos(phi)); // perpendicular vector 1
        Eigen::Vector3f k(std::cos(phi)*std::sin(theta), -std::cos(theta), std::sin(phi)*std::sin(theta)); // perpendicular vector 2

        Eigen::Matrix3f R(3,3);
        R(0,0) = i(0);    R(0,1) = j(0);    R(0,2) = k(0);
        R(1,0) = i(1);    R(1,1) = j(1);    R(1,2) = k(1);
        R(2,0) = i(2);    R(2,1) = j(2);    R(2,2) = k(2);

        return R;
      }

      // Find closest point on this line segment to a given point (and optionally
      // the squared distance and whether or not the closest point is an endpoint).
      Point3 ClosestPoint(const Point3& query, bool* is_endpoint = nullptr,
                          float* squared_distance = nullptr) const;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
      Point3 p1_;
      Point3 p2_;
      float length_;
      Point3 unit_direction_;
};  // struct LineSegment3

}  // namespace ilqgames_planning

#endif
