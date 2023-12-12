///////////////////////////////////////////////////////////////////////////////
//
// Quadratic penalty on distance from where we should be along a given polyline
// if we were traveling at the given nominal speed.
//
///////////////////////////////////////////////////////////////////////////////

#include <ilqgames_planning/route3_progress_cost.h>

namespace ilqgames_planning {

float Route3ProgressCost::Evaluate(Time t, const VectorXf& input) const {
    LOG(INFO) << "Evaluating route progress cost at time " << t << std::endl;

    CHECK_LT(xidx_, input.size());
    CHECK_LT(yidx_, input.size());
    CHECK_LT(zidx_, input.size());

    const float desired_route_pos =
        initial_route_pos_ + (t - initial_time_) * nominal_speed_;
    const Point3 desired = polyline_.PointAt(desired_route_pos, nullptr, nullptr);

    const float dx = input(xidx_) - desired.x();
    const float dy = input(yidx_) - desired.y();
    const float dz = input(zidx_) - desired.z();
    return 0.5 * weight_ * (dx * dx + dy * dy + dz * dz);
}

void Route3ProgressCost::Quadraticize(Time t, const VectorXf& input,
                                      MatrixXf* hess, VectorXf* grad) const {
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
    const float desired_route_pos =
        initial_route_pos_ + (t - initial_time_) * nominal_speed_;

    bool is_endpoint;
    const Point3 route_point =
        polyline_.PointAt(desired_route_pos, nullptr, nullptr, &is_endpoint);

    // Compute gradient and Hessian.
    const float diff_x = current_position.x() - route_point.x();
    const float diff_y = current_position.y() - route_point.y();
    const float diff_z = current_position.z() - route_point.z();
    const float dx = weight_ * diff_x;
    const float dy = weight_ * diff_y;
    const float dz = weight_ * diff_z;
    const float ddx = weight_;
    const float ddy = weight_;
    const float ddz = weight_;
    const float dxdy = 0.0;
    const float dydz = 0.0;
    const float dxdz = 0.0;

    (*grad)(xidx_) += dx;
    (*grad)(yidx_) += dy;
    (*grad)(zidx_) += dz;

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
