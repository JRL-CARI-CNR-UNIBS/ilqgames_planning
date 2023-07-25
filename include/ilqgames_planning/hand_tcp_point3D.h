#ifndef HAND_TCP_POINT3D_H
#define HAND_TCP_POINT3D_H

#include <ilqgames/solver/top_down_renderable_problem.h>

namespace ilqgames_planning {

class HandTcpPoint3D : public ilqgames::TopDownRenderableProblem {
    public:
        ~HandTcpPoint3D() {}

        HandTcpPoint3D() : ilqgames::TopDownRenderableProblem() {}

        // Construct dynamics, initial state, and player costs.
        void ConstructDynamics();
        void ConstructInitialState();
        void ConstructPlayerCosts();

        // Unpack x, y, z (for each player, potentially) from a given state.
        std::vector<float> Xs(const Eigen::VectorXf& x) const;
        std::vector<float> Ys(const Eigen::VectorXf& x) const;
        std::vector<float> Zs(const Eigen::VectorXf& x) const;

};  // class HandTcpPoint3D

}  // namespace ilqgames_planning

#endif // HAND_TCP_POINT3D_H
