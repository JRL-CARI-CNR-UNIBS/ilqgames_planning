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

          p2p1_ = (p2_ - p1_);
          CHECK_LT(p2p1_.norm() - length_, ilqgames::constants::kSmallNumber);
          CHECK_GT(length_, ilqgames::constants::kSmallNumber);

      }

      // Override operator equal to check if two objects are the same
      bool operator==(const LineSegment3 &other_segment) const {
        return (this->p1_ == other_segment.p1_ && this->p1_ == other_segment.p1_);
      }

      // Accessors.
      float Length() const { return length_; }
      const Point3& FirstPoint() const { return p1_; }
      const Point3& SecondPoint() const { return p2_; }
      const Point3& UnitDirection() const { return unit_direction_; }

      // Express heading in terms of Euler Angles (Z-Y rotations in radians)
      std::vector<float> Heading() const {
        const float theta_x = std::atan2(p2p1_.y(), p2p1_.x());
        const float theta_y = -std::atan2(p2p1_.z(), std::sqrt((pow(p2p1_.x(),2) + pow(p2p1_.y(),2))));
        return {theta_x, theta_y};
      }

      // Find closest point on this line segment to a given point (and optionally
      // the squared distance and whether or not the closest point is an endpoint).
      Point3 ClosestPoint(const Point3& query, bool* is_endpoint = nullptr,
                          float* squared_distance = nullptr) const;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
      Point3 p1_;
      Point3 p2_;
      Point3 p2p1_;
      float length_;
      Point3 unit_direction_;
};  // struct LineSegment3

}  // namespace ilqgames_planning

#endif
