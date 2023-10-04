///////////////////////////////////////////////////////////////////////////////
//
// Line segment in 3D.
//
///////////////////////////////////////////////////////////////////////////////

#include <ilqgames_planning/line_segment3.h>
#include <ilqgames_planning/types.h>

namespace ilqgames_planning {

Point3 LineSegment3::ClosestPoint(const Point3& query, bool* is_endpoint,
                                  float* squared_distance) const {
  // Find query relative to p1.
  const Point3 relative_query = query - p1_;

  // Find dot and cross product.
  const float dot_product = relative_query.dot(unit_direction_);
  const Eigen::Vector3f cross_product = relative_query.cross(unit_direction_); // https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html

  // Determine closest point. This will either be an endpoint or the interior of the segment.
  if (dot_product < 0.0) {
    // Query lies behind this line segment, so closest point is p1.
    if (is_endpoint) *is_endpoint = true;

    if (squared_distance) {
      *squared_distance = relative_query.squaredNorm();
    }

    return p1_;

  } else if (dot_product > length_) {
    // Closest point is p2.
    if (is_endpoint) *is_endpoint = true;

    if (squared_distance) {
      *squared_distance = (query - p2_).squaredNorm();
    }

    return p2_;
  }

  // Closest point is in the interior of the line segment.
  if (is_endpoint) *is_endpoint = false;

  if (squared_distance)
    *squared_distance = cross_product.squaredNorm(); // https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html

  return p1_ + dot_product * unit_direction_;
}

}  // namespace ilqgames_planning
