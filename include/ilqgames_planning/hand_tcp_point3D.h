#ifndef HAND_TCP_POINT3D_H
#define HAND_TCP_POINT3D_H

#include <ilqgames/solver/top_down_renderable_problem.h>
#include <ilqgames_planning/types.h>
#include <map>

namespace ilqgames_planning {

class HandTcpPoint3D : public ilqgames::TopDownRenderableProblem {
    public:
        ~HandTcpPoint3D() {}

        HandTcpPoint3D() : ilqgames::TopDownRenderableProblem() {}

        // Construct dynamics, initial state, and player costs
        void ConstructDynamics();
        void ConstructInitialState();
        void ConstructPlayerCosts();


        // Initialize this object.
        void Initialize(const ilqgames::Time t) {
          time_horizon_ = t;
          Problem::Initialize();
        }

        // Unpack x, y, z, vx, vy, vz (for each player, potentially) from a given state
        std::vector<float> Xs(const Eigen::VectorXf& x) const;
        std::vector<float> Ys(const Eigen::VectorXf& x) const;
        std::vector<float> Zs(const Eigen::VectorXf& x) const;

        std::vector<float> Vxs(const Eigen::VectorXf& x) const;
        std::vector<float> Vys(const Eigen::VectorXf& x) const;
        std::vector<float> Vzs(const Eigen::VectorXf& x) const;

        inline std::vector<float> Thetas(const Eigen::VectorXf& x) const { return {}; }; // just to implement the pure virtual method from the parent class (not used!)

        // Unpack the list of possible waypoints to pass through for each player
        inline std::map<std::string, std::vector<Point3>> GetWaypoints() const { return waypoints_; };

        inline void SetTimeHorizon(const ilqgames::Time t) { time_horizon_ = t; };
        inline ilqgames::Time GetTimeHorizon() const { return time_horizon_; };

    private:
        // Map to store list of waypoints for each agent
        std::map<std::string, std::vector<Point3>> waypoints_;

        // Time horizon
        ilqgames::Time time_horizon_ = 10.0;

};  // class HandTcpPoint3D

}  // namespace ilqgames_planning

#endif // HAND_TCP_POINT3D_H
