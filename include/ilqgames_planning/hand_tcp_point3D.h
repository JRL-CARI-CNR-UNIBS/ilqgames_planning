#ifndef HAND_TCP_POINT3D_H
#define HAND_TCP_POINT3D_H

#include <ilqgames/solver/top_down_renderable_problem.h>

namespace ilqgames_planning {

class HandTcpPoint3D : public ilqgames::TopDownRenderableProblem {
    public:
        ~HandTcpPoint3D() {}

        HandTcpPoint3D() : ilqgames::TopDownRenderableProblem() {}

        // Construct dynamics, initial state, and player costs
        void ConstructDynamics();
        void ConstructInitialState();
        void ConstructPlayerCosts();

        // Unpack x, y, z, vx, vy, vz (for each player, potentially) from a given state
        std::vector<float> Xs(const Eigen::VectorXf& x) const;
        std::vector<float> Ys(const Eigen::VectorXf& x) const;
        std::vector<float> Zs(const Eigen::VectorXf& x) const;

        std::vector<float> Vxs(const Eigen::VectorXf& x) const;
        std::vector<float> Vys(const Eigen::VectorXf& x) const;
        std::vector<float> Vzs(const Eigen::VectorXf& x) const;

        inline std::vector<float> Thetas(const Eigen::VectorXf& x) const { return {}; }; // just to implement the pure virtual method from the parent class (not used!)

};  // class HandTcpPoint3D

}  // namespace ilqgames_planning

#endif // HAND_TCP_POINT3D_H
