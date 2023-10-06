////////////////////////////////////////////////////////////////////////////////
//
// Quadratic penalty on distance from a given Polyline3.
//
///////////////////////////////////////////////////////////////////////////////

#include <ilqgames_planning/quadratic_polyline3_cost.h>
#include <ilqgames/cost/time_invariant_cost.h>
#include <ilqgames_planning/polyline3.h>
#include <ilqgames_planning/types.h>

#include <tuple>

namespace ilqgames_planning {

float QuadraticPolyline3Cost::Evaluate(const VectorXf& input) const {
  CHECK_LT(xidx_, input.size());
  CHECK_LT(yidx_, input.size());
  CHECK_LT(zidx_, input.size());

  // Compute squared distance by finding closest point.
  float squared_distance;
  bool is_endpoint;
  polyline_.ClosestPoint(Point3(input(xidx_), input(yidx_), input(zidx_)), nullptr, nullptr,
                         &squared_distance, &is_endpoint);

//  if (is_endpoint) {
//    // If the is_endpoint flag is raised, we set the squared_distance to
//    // 0.0.
//    squared_distance = 0.0;
//  }

  return 0.5 * weight_ * squared_distance;
}

void QuadraticPolyline3Cost::Quadraticize(const VectorXf& input, MatrixXf* hess,
                                          VectorXf* grad) const {
  CHECK_LT(xidx_, input.size());
  CHECK_LT(yidx_, input.size());
  CHECK_LT(zidx_, input.size());

  CHECK_NOTNULL(hess);
  CHECK_NOTNULL(grad);
  CHECK_EQ(input.size(), hess->rows());
  CHECK_EQ(input.size(), hess->cols());
  CHECK_EQ(input.size(), grad->size());

  // Unpack current position and find closest point / segment.
  const Point3 current_position(input(xidx_), input(yidx_), input(zidx_));

  bool is_vertex;
  bool is_endpoint;
  LineSegment3 segment(Point3(0.0, 0.0, 0.0), Point3(1.0, 1.0, 1.0));
  const Point3 closest_point = polyline_.ClosestPoint(
      current_position, &is_vertex, &segment, nullptr, &is_endpoint);

//  std::cout << "Closest point: " << closest_point << std::endl;

  // First check whether the closest point is a endpoint of the polyline.
  if (is_endpoint) return;

  // Handle cases separately depending on whether or not closest point is
  // a vertex of the polyline.
  float ddx = weight_;
  float ddy = weight_;
  float ddz = weight_;
  float dxdy = 0.0;
  float dxdz = 0.0;
  float dydz = 0.0;
  float dx = weight_ * (current_position.x() - closest_point.x());
  float dy = weight_ * (current_position.y() - closest_point.y());
  float dz = weight_ * (current_position.z() - closest_point.z());

  // if closest_point is a vertex, then hessian and gradient are computed
  // in the standard way. If it is not, then compute the cross product
  if (!is_vertex) {
    const Point3 relative = current_position - segment.FirstPoint();
    const Point3 unit_segment = segment.UnitDirection();
    const Point3 closest_point_relative = closest_point - segment.FirstPoint();

    const Point3 orthogonal_vector = relative - closest_point_relative;
    const Point3 orthogonal_versor = orthogonal_vector / orthogonal_vector.norm();

    // Handle Hessian first.
    ddx = weight_ * orthogonal_versor.x() * orthogonal_versor.x();
    ddy = weight_ * orthogonal_versor.y() * orthogonal_versor.y();
    ddz = weight_ * orthogonal_versor.z() * orthogonal_versor.z();
    dxdy = weight_ * orthogonal_versor.x() * orthogonal_versor.y();
    dxdz = weight_ * orthogonal_versor.x() * orthogonal_versor.z();
    dydz = weight_ * orthogonal_versor.y() * orthogonal_versor.z();

    // Handle gradient.
    dx = weight_ * orthogonal_vector.x();
    dy = weight_ * orthogonal_vector.y();
    dz = weight_ * orthogonal_vector.z();
  }

  (*grad)(xidx_) += dx;
  (*grad)(yidx_) += dy;
  (*grad)(yidx_) += dz;

  (*hess)(xidx_, xidx_) += ddx;
  (*hess)(yidx_, yidx_) += ddy;
  (*hess)(zidx_, zidx_) += ddz;

  (*hess)(xidx_, yidx_) += dxdy;
  (*hess)(yidx_, xidx_) += dxdy;

  (*hess)(xidx_, zidx_) += dxdz;
  (*hess)(zidx_, xidx_) += dxdz;

  (*hess)(yidx_, zidx_) += dydz;
  (*hess)(zidx_, yidx_) += dydz;
}

}  // namespace ilqgames_planning
