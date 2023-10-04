///////////////////////////////////////////////////////////////////////////////
//
// Polyline3 class for piecewise linear paths in 3D.
//
///////////////////////////////////////////////////////////////////////////////

#include <ilqgames_planning/line_segment3.h>
#include <ilqgames_planning/polyline3.h>
#include <ilqgames_planning/types.h>

#include <glog/logging.h>

namespace ilqgames_planning {

Polyline3::Polyline3(const PointList3& points) : length_(0.0) {
  CHECK_GT(points.size(), 1);
  cumulative_lengths_.push_back(length_);

  // Parse into list of line segents.
  for (size_t ii = 1; ii < points.size(); ii++) {
    segments_.emplace_back(points[ii - 1], points[ii]);
    length_ += segments_.back().Length();
    cumulative_lengths_.push_back(length_);
  }
}

void Polyline3::AddPoint(const Point3& point) {
  segments_.emplace_back(segments_.back().SecondPoint(), point);
  length_ += segments_.back().Length();
}

Point3 Polyline3::PointAt(float route_pos, bool* is_vertex,
                          LineSegment3* segment, bool* is_endpoint) const {
  auto upper = std::upper_bound(cumulative_lengths_.begin(),
                                cumulative_lengths_.end(), route_pos);
  if (upper == cumulative_lengths_.end()) {
    LOG(WARNING) << "Route position " << route_pos
                 << " was off the end of the route.";
    upper--;
  }

  // Find the index of the line segment which contains this route position.
  upper--;
  const size_t idx = std::distance(cumulative_lengths_.begin(), upper);
  if (segment) *segment = segments_[idx];

  // Walk along this line segment the remaining distance.
  const float remaining = route_pos - cumulative_lengths_[idx];
  CHECK_GE(remaining, 0.0);

  if (is_vertex) {
    *is_vertex = remaining < ilqgames::constants::kSmallNumber ||
                 remaining > segments_[idx].Length();
  }

  const Point3 return_point =
      segments_[idx].FirstPoint() + remaining * segments_[idx].UnitDirection();
  if (is_endpoint) {
    if (idx != 0 && idx != segments_.size() - 1)
      *is_endpoint = false;
    else
      *is_endpoint = (return_point == segments_.front().FirstPoint()) ||
                     (return_point == segments_.back().SecondPoint());
  }

  return return_point;
}

Point3 Polyline3::ClosestPoint(const Point3& query, bool* is_vertex,
                               LineSegment3* segment,
                               float* squared_distance,
                               bool* is_endpoint) const {
  // Walk along each line segment and remember which was closest.
  float closest_squared_distance = ilqgames::constants::kInfinity;
  Point3 closest_point;

  float current_squared_distance;
  int segment_idx = 0;
  int segment_counter = 0;
  bool is_segment_endpoint;
  for (const auto& s : segments_) {
    const Point3 current_point = s.ClosestPoint(
        query, &is_segment_endpoint, &current_squared_distance);

    if (std::abs(current_squared_distance) <
        std::abs(closest_squared_distance)) {
      // If this is an endpoint, compute which side of the polyline this is on
      // by finding which side of the line segment from the previous point to
      // the next point this is on.
      if (is_segment_endpoint &&
          (segment_counter > 0 || current_point == s.SecondPoint()) &&
          (segment_counter < segments_.size() - 1 ||
           current_point == s.FirstPoint())) {
        const LineSegment3 shortcut =
            (current_point == s.FirstPoint())
                ? LineSegment3(segments_[segment_counter - 1].FirstPoint(),
                               s.SecondPoint())
                : LineSegment3(s.FirstPoint(),
                               segments_[segment_counter + 1].SecondPoint());
        current_squared_distance *= current_squared_distance;

        CHECK(current_squared_distance >= 0.0);
      }

      closest_squared_distance = current_squared_distance;
      closest_point = current_point;

      if (is_vertex) *is_vertex = is_segment_endpoint;
      segment_idx = segment_counter;
    }

    segment_counter++;
  }

  // Maybe set segment.
  if (segment) *segment = segments_[segment_idx];

  // Maybe set squared_distance.
  if (squared_distance) *squared_distance = closest_squared_distance;

  // Check if the closest point occurs at an endpoint for the polyline.
  if (is_endpoint) {
    auto is_same_point = [](const Point3& p1, const Point3& p2) {
      return (p1 - p2).squaredNorm() < ilqgames::constants::kSmallNumber;
    };  // is_same_point

    *is_endpoint =
        is_same_point(closest_point, segments_.front().FirstPoint()) ||
        is_same_point(closest_point, segments_.back().SecondPoint());
  }

  return closest_point;
}

}  // namespace ilqgames_planning
