#include <ilqgames_planning/proximity_cost_3d.h>
#include <ilqgames/utils/types.h>

#include <glog/logging.h>

namespace ilqgames_planning {

float ProximityCost3D::Evaluate(const Eigen::VectorXf& input) const {
    const float dx = input(xidx1_) - input(xidx2_);
    const float dy = input(yidx1_) - input(yidx2_);
    const float dz = input(zidx1_) - input(zidx2_);
    const float delta_sq = dx * dx + dy * dy + dz * dz;

    if (delta_sq >= threshold_sq_) return 0.0;

    const float gap = threshold_ - std::sqrt(delta_sq);
    return 0.5 * weight_ * gap * gap;
}

void ProximityCost3D::Quadraticize(const Eigen::VectorXf& input, Eigen::MatrixXf* hess,
                                   Eigen::VectorXf* grad) const {
    CHECK_NOTNULL(hess);
    CHECK_NOTNULL(grad);

    // Check dimensions.
    CHECK_EQ(input.size(), hess->rows());
    CHECK_EQ(input.size(), hess->cols());
    CHECK_EQ(input.size(), grad->size());

    // Compute Hessian and gradient.
    const float dx = input(xidx1_) - input(xidx2_);
    const float dy = input(yidx1_) - input(yidx2_);
    const float dz = input(zidx1_) - input(zidx2_);
    const float delta_sq = dx * dx + dy * dy + dz * dz;

    // Catch cost not active.
    if (delta_sq >= threshold_sq_) return;

    const float delta = std::sqrt(delta_sq);
    const float gap = threshold_ - delta;
    const float weight_delta = weight_ / delta;
    const float dx_delta = dx / delta;
    const float dy_delta = dy / delta;
    const float dz_delta = dz / delta;

    const float ddx1 = -weight_delta * gap * dx;
    const float ddy1 = -weight_delta * gap * dy;
    const float ddz1 = -weight_delta * gap * dz;

    const float hess_x1x1 = weight_delta * (dx_delta * (gap * dx_delta + dx) - gap);
    const float hess_y1y1 = weight_delta * (dy_delta * (gap * dy_delta + dy) - gap);
    const float hess_z1z1 = weight_delta * (dz_delta * (gap * dz_delta + dz) - gap);

    const float hess_x1y1 = weight_delta * (dx_delta * (gap * dy_delta + dy));
    const float hess_x1z1 = weight_delta * (dx_delta * (gap * dz_delta + dz));
    const float hess_y1z1 = weight_delta * (dy_delta * (gap * dz_delta + dz));

    (*grad)(xidx1_) += ddx1;
    (*grad)(xidx2_) -= ddx1;

    (*grad)(yidx1_) += ddy1;
    (*grad)(yidx2_) -= ddy1;

    (*grad)(zidx1_) += ddz1;
    (*grad)(zidx2_) -= ddz1;

    (*hess)(xidx1_, xidx1_) += hess_x1x1;
    (*hess)(xidx1_, xidx2_) -= hess_x1x1;
    (*hess)(xidx2_, xidx1_) -= hess_x1x1;
    (*hess)(xidx2_, xidx2_) += hess_x1x1;

    (*hess)(yidx1_, yidx1_) += hess_y1y1;
    (*hess)(yidx1_, yidx2_) -= hess_y1y1;
    (*hess)(yidx2_, yidx1_) -= hess_y1y1;
    (*hess)(yidx2_, yidx2_) += hess_y1y1;

    (*hess)(zidx1_, zidx1_) += hess_z1z1;
    (*hess)(zidx1_, zidx2_) -= hess_z1z1;
    (*hess)(zidx2_, zidx1_) -= hess_z1z1;
    (*hess)(zidx2_, zidx2_) += hess_z1z1;

    (*hess)(xidx1_, yidx1_) += hess_x1y1;
    (*hess)(yidx1_, xidx1_) += hess_x1y1;

    (*hess)(xidx1_, zidx1_) += hess_x1z1;
    (*hess)(zidx1_, xidx1_) += hess_x1z1;

    (*hess)(yidx1_, zidx1_) += hess_y1z1;
    (*hess)(zidx1_, yidx1_) += hess_y1z1;

FIX HERE !!

    (*hess)(xidx1_, yidx2_) -= hess_x1y1;
    (*hess)(yidx2_, xidx1_) -= hess_x1y1;

    (*hess)(xidx2_, yidx1_) -= hess_x1y1;
    (*hess)(yidx1_, xidx2_) -= hess_x1y1;

    (*hess)(xidx2_, yidx2_) += hess_x1y1;
    (*hess)(yidx2_, xidx2_) += hess_x1y1;
}

}  // namespace ilqgames_planning
