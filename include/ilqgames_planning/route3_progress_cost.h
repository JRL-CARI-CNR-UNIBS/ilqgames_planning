///////////////////////////////////////////////////////////////////////////////
//
// Quadratic penalty on distance from where we should be along a given polyline
// if we were traveling at the given nominal speed.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef ILQGAMES_PLANNING_ROUTE3_PROGRESS_COST_H
#define ILQGAMES_PLANNING_ROUTE3_PROGRESS_COST_H

#include <ilqgames/cost/cost.h>
#include <ilqgames_planning/polyline3.h>
#include <ilqgames/utils/types.h>

#include <string>
#include <tuple>

namespace ilqgames_planning {

class Route3ProgressCost : public ilqgames::Cost {
 public:
  // Construct from a multiplicative weight and the input dimensions
  // corresponding to (x, y, z)-position.
  Route3ProgressCost(float weight, float nominal_speed,
                     const Polyline3& polyline,
                     const std::tuple<ilqgames::Dimension, ilqgames::Dimension, ilqgames::Dimension>& position_idxs,
                     const std::string& name = "", float initial_route_pos = 0.0)
      : Cost(weight, name),
        nominal_speed_(nominal_speed),
        polyline_(polyline),
        xidx_(std::get<0>(position_idxs)),
        yidx_(std::get<1>(position_idxs)),
        zidx_(std::get<2>(position_idxs)),
        initial_route_pos_(initial_route_pos) {}

  // Evaluate this cost at the current input.
  float Evaluate(Time t, const VectorXf& input) const;

  // Quadraticize this cost at the given input, and add to the running
  // sum of gradients and Hessians.
  void Quadraticize(Time t, const VectorXf& input, MatrixXf* hess,
                    VectorXf* grad) const;

 private:
  // Nominal speed.
  const float nominal_speed_;

  // Polyline to compute distances from.
  const Polyline3 polyline_;

  // Dimensions of input corresponding to (x, y, z)-position.
  const ilqgames::Dimension xidx_;
  const ilqgames::Dimension yidx_;
  const ilqgames::Dimension zidx_;

  // Initial route position and time.
  const float initial_route_pos_;

};  //\class Route3ProgressCost

}  // namespace ilqgames_planning

#endif
