#include <ilqgames/cost/time_invariant_cost.h>

#ifndef PROXIMITY_COST_3D_H
#define PROXIMITY_COST_3D_H

namespace ilqgames_planning {

class ProximityCost3D : public ilqgames::TimeInvariantCost {

public:
ProximityCost3D(float weight,
                const std::tuple<ilqgames::Dimension, ilqgames::Dimension, ilqgames::Dimension>& position_idxs1,
                const std::tuple<ilqgames::Dimension, ilqgames::Dimension, ilqgames::Dimension>& position_idxs2,
                float threshold, const std::string& name = "") :
    ilqgames::TimeInvariantCost(weight, name),
    threshold_(threshold),
    threshold_sq_(threshold * threshold),
    xidx1_(std::get<0>(position_idxs1)),
    yidx1_(std::get<1>(position_idxs1)),
    zidx1_(std::get<2>(position_idxs1)),
    xidx2_(std::get<0>(position_idxs2)),
    yidx2_(std::get<1>(position_idxs2)),
    zidx2_(std::get<2>(position_idxs2)) {}

 // Evaluate this cost at the current input.
 float Evaluate(const Eigen::VectorXf& input) const;

 // Quadraticize this cost at the given input, and add to the running
 // sum of gradients and Hessians.
 void Quadraticize(const Eigen::VectorXf& input, Eigen::MatrixXf* hess,
                   Eigen::VectorXf* grad) const;

private:
 // Threshold for minimum squared relative distance.
 const float threshold_, threshold_sq_;

 // Position indices for two 3D points.
 const ilqgames::Dimension xidx1_, yidx1_, zidx1_;
 const ilqgames::Dimension xidx2_, yidx2_, zidx2_;
};  //\class ProximityCost3D

}   // namespace ilqgames_planning


#endif // PROXIMITY_COST_3D_H
