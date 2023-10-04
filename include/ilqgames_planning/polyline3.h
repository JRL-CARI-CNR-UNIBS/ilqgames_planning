///////////////////////////////////////////////////////////////////////////////
//
// Polyline3 class for piecewise linear paths in 3D.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_PLANNING_POLYLINE3_H
#define ILQGAMES_PLANNING_POLYLINE3_H

#include <ilqgames_planning/line_segment3.h>
#include <ilqgames_planning/types.h>

#include <glog/logging.h>
#include <math.h>

namespace ilqgames_planning {

class Polyline3 {
 public:
  // Construct from a list of points. This list must contain at least 2 points!
  Polyline3(const PointList3& points);
  ~Polyline3() {}

  // Add a new point to the end of the polyline.
  void AddPoint(const Point3& point);

  // Compute length.
  float Length() const { return length_; }

  // Find closest point on this line segment to a given point, and optionally
  // the line segment that point belongs to (and flag for whether it is a
  // vertex), and the squared distance.
  Point3 ClosestPoint(const Point3& query, bool* is_vertex = nullptr,
                      LineSegment3* segment = nullptr,
                      float* squared_distance = nullptr,
                      bool* is_endpoint = nullptr) const;

  // Find the point the given distance from the start of the polyline.
  // Optionally returns whether this is a vertex and the line segment which the
  // point belongs to.
  Point3 PointAt(float route_pos, bool* is_vertex = nullptr,
                 LineSegment3* segment = nullptr,
                 bool* is_endpoint = nullptr) const;

  // Access line segments.
  const std::vector<LineSegment3>& Segments() const { return segments_; }

 private:
  std::vector<LineSegment3> segments_;
  std::vector<float> cumulative_lengths_;
  float length_;
};  // struct Polyline3

}  // namespace ilqgames_planning

#endif
