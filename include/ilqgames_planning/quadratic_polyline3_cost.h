///////////////////////////////////////////////////////////////////////////////
//
// Quadratic penalty on distance from a given Polyline3.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_PLANNING_QUADRATIC_POLYLINE3_COST_H
#define ILQGAMES_PLANNING_QUADRATIC_POLYLINE3_COST_H

#include <ilqgames/cost/time_invariant_cost.h>
#include <ilqgames_planning/polyline3.h>
#include <ilqgames_planning/types.h>

#include <string>
#include <tuple>

namespace ilqgames_planning {

class QuadraticPolyline3Cost : public ilqgames::TimeInvariantCost {
 public:
  // Construct from a multiplicative weight and the input dimensions
  // corresponding to (x, y, z)-position.
  QuadraticPolyline3Cost(float weight, const Polyline3& polyline,
                         const std::tuple<ilqgames::Dimension, ilqgames::Dimension, ilqgames::Dimension>& position_idxs,
                         const std::string& name = "")
      : ilqgames::TimeInvariantCost(weight, name),
        polyline_(polyline),
        xidx_(std::get<0>(position_idxs)),
        yidx_(std::get<1>(position_idxs)),
        zidx_(std::get<2>(position_idxs)) {}

  // Evaluate this cost at the current input.
  float Evaluate(const VectorXf& input) const;

  // Quadraticize this cost at the given input, and add to the running
  // sum of gradients and Hessians.
  void Quadraticize(const VectorXf& input, MatrixXf* hess,
                    VectorXf* grad) const;

 private:
  // Polyline to compute distances from.
  const Polyline3 polyline_;

  // Dimensions of input corresponding to (x, y, z)-position.
  const ilqgames::Dimension xidx_;
  const ilqgames::Dimension yidx_;
  const ilqgames::Dimension zidx_;
};  //\class QuadraticPolyline3Cost

}  // namespace ilqgames_planning

#endif
