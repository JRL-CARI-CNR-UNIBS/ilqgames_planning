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

    // Compute elements of the gradient vector
    const float ddx1 = -weight_delta * gap * dx;
    const float ddy1 = -weight_delta * gap * dy;
    const float ddz1 = -weight_delta * gap * dz;

    // Compute elements of the hessian matrix
    const float hess_x1x1 = weight_delta * (dx_delta * (gap * dx_delta + dx) - gap);
    const float hess_y1y1 = weight_delta * (dy_delta * (gap * dy_delta + dy) - gap);
    const float hess_z1z1 = weight_delta * (dz_delta * (gap * dz_delta + dz) - gap);

    const float hess_x1y1 = weight_delta * (dx_delta * (gap * dy_delta + dy));
    const float hess_x1z1 = weight_delta * (dx_delta * (gap * dz_delta + dz));
    const float hess_y1z1 = weight_delta * (dy_delta * (gap * dz_delta + dz));

    // Update gradient (+ for idx1, - for idx2)
    (*grad)(xidx1_) += ddx1;
    (*grad)(xidx2_) -= ddx1;

    (*grad)(yidx1_) += ddy1;
    (*grad)(yidx2_) -= ddy1;

    (*grad)(zidx1_) += ddz1;
    (*grad)(zidx2_) -= ddz1;

    // Update hessian matrix
    // (+ for idx1/idx1, - for idx1/idx2, - for idx2/idx1, + for idx2/idx2)
    // It must be size(input)*size(input) = 6*6 = 36 total elements

    // Second-order x-derivatives (x1x1, x2x2, x1x2, x2x1)
    (*hess)(xidx1_, xidx1_) += hess_x1x1;
    (*hess)(xidx1_, xidx2_) -= hess_x1x1;
    (*hess)(xidx2_, xidx1_) -= hess_x1x1;
    (*hess)(xidx2_, xidx2_) += hess_x1x1;

    // Second-order y-derivatives (y1y1, y2y2, y1y2, y2y1)
    (*hess)(yidx1_, yidx1_) += hess_y1y1;
    (*hess)(yidx1_, yidx2_) -= hess_y1y1;
    (*hess)(yidx2_, yidx1_) -= hess_y1y1;
    (*hess)(yidx2_, yidx2_) += hess_y1y1;

    // Second-order z-derivatives (z1z1, z2z2, z1z2, z2z1)
    (*hess)(zidx1_, zidx1_) += hess_z1z1;
    (*hess)(zidx1_, zidx2_) -= hess_z1z1;
    (*hess)(zidx2_, zidx1_) -= hess_z1z1;
    (*hess)(zidx2_, zidx2_) += hess_z1z1;


    // Second-order mixed xy-derivatives (x1y1, y1x1) for input1
    (*hess)(xidx1_, yidx1_) += hess_x1y1;
    (*hess)(yidx1_, xidx1_) += hess_x1y1;

    // Second-order mixed xz-derivatives (x1z1, z1x1) for input1
    (*hess)(xidx1_, zidx1_) += hess_x1z1;
    (*hess)(zidx1_, xidx1_) += hess_x1z1;

    // Second-order mixed yz-derivatives (y1z1, z1y1) for input1
    (*hess)(yidx1_, zidx1_) += hess_y1z1;
    (*hess)(zidx1_, yidx1_) += hess_y1z1;

    // Second-order mixed xy-derivatives (x2y2, y2x2) for input2
    (*hess)(xidx2_, yidx2_) += hess_x1y1;
    (*hess)(yidx2_, xidx2_) += hess_x1y1;

    // Second-order mixed xz-derivatives (x2z2, z2x2) for input2
    (*hess)(xidx2_, zidx2_) += hess_x1z1;
    (*hess)(zidx2_, xidx2_) += hess_x1z1;

    // Second-order mixed yz-derivatives (y2z2, z2y2) for input2
    (*hess)(yidx2_, zidx2_) += hess_y1z1;
    (*hess)(zidx2_, yidx2_) += hess_y1z1;


    // Second-order mixed xy-derivatives (x1y2, y2x1) for input1/input2
    (*hess)(xidx1_, yidx2_) -= hess_x1y1;
    (*hess)(yidx2_, xidx1_) -= hess_x1y1;

    // Second-order mixed xz-derivatives (x1z2, z2x1) for input1/input2
    (*hess)(xidx1_, zidx2_) -= hess_x1z1;
    (*hess)(zidx2_, xidx1_) -= hess_x1z1;

    // Second-order mixed yz-derivatives (y2z1, z1y2) for input1/input2
    (*hess)(yidx2_, zidx1_) -= hess_y1z1;
    (*hess)(zidx1_, yidx2_) -= hess_y1z1;

    // Second-order mixed xy-derivatives (x2y1, y1x2) for input1/input2
    (*hess)(xidx2_, yidx1_) -= hess_x1y1;
    (*hess)(yidx1_, xidx2_) -= hess_x1y1;

    // Second-order mixed xz-derivatives (x2z1, z1x2) for input1/input2
    (*hess)(xidx2_, zidx1_) -= hess_x1z1;
    (*hess)(zidx1_, xidx2_) -= hess_x1z1;

    // Second-order mixed yz-derivatives (y1z2, z2y1) for input1/input2
    (*hess)(yidx1_, zidx2_) -= hess_y1z1;
    (*hess)(zidx2_, yidx1_) -= hess_y1z1;
}

}  // namespace ilqgames_planning
