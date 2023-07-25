#ifndef POINT_PLAYER_3D_H
#define POINT_PLAYER_3D_H

#include <ilqgames/dynamics/single_player_dynamical_system.h>

namespace ilqgames_planning {

class PointPlayer3D: public ilqgames::SinglePlayerDynamicalSystem {
    public:
        ~PointPlayer3D() {}

        PointPlayer3D() : ilqgames::SinglePlayerDynamicalSystem(kNumXDims, kNumUDims) {}

        // Compute time derivative of state.
        Eigen::VectorXf Evaluate(ilqgames::Time t, const Eigen::VectorXf& x, const Eigen::VectorXf& u) const;

        // Compute a discrete-time Jacobian linearization.
        void Linearize(ilqgames::Time t, const Eigen::VectorXf& x, const Eigen::VectorXf& u,
        Eigen::Ref<Eigen::MatrixXf> A, Eigen::Ref<Eigen::MatrixXf> B) const;

        // Distance metric between two states.
        float DistanceBetween(const Eigen::VectorXf& x0, const Eigen::VectorXf& x1) const;

        // Position dimensions.
        std::vector<ilqgames::Dimension> PositionDimensions() const { return {kPxIdx, kPyIdx, kPzIdx}; }

        // Constexprs for state indices.
        static const ilqgames::Dimension kNumXDims;
        static const ilqgames::Dimension kPxIdx;
        static const ilqgames::Dimension kVxIdx;
        static const ilqgames::Dimension kPyIdx;
        static const ilqgames::Dimension kVyIdx;
        static const ilqgames::Dimension kPzIdx;
        static const ilqgames::Dimension kVzIdx;

        // Constexprs for control indices.
        static const ilqgames::Dimension kNumUDims;
        static const ilqgames::Dimension kAxIdx;
        static const ilqgames::Dimension kAyIdx;
        static const ilqgames::Dimension kAzIdx;

    private:

};  // class HumanPlayer3D

// ----------------------------- IMPLEMENTATION ----------------------------- //

inline Eigen::VectorXf PointPlayer3D::Evaluate(ilqgames::Time t, const Eigen::VectorXf& x,
                                               const Eigen::VectorXf& u) const {

    Eigen::VectorXf xdot(xdim_);
    xdot(kPxIdx) = x(kVxIdx);
    xdot(kVxIdx) = u(kAxIdx);
    xdot(kPyIdx) = x(kVyIdx);
    xdot(kVyIdx) = u(kAyIdx);
    xdot(kPzIdx) = x(kVzIdx);
    xdot(kVzIdx) = u(kAzIdx);

    return xdot;
}

inline void PointPlayer3D::Linearize(ilqgames::Time t,
                                     const Eigen::VectorXf& x, const Eigen::VectorXf& u,
                                     Eigen::Ref<Eigen::MatrixXf> A,
                                     Eigen::Ref<Eigen::MatrixXf> B) const {


    A.block(0,0,kNumXDims,kNumXDims) << Eigen::MatrixXf::Identity(kNumXDims, kNumXDims);
    B.block(0,0,kNumXDims,kNumUDims) << Eigen::MatrixXf::Zero(kNumXDims, kNumUDims);

    A(kPxIdx,kVxIdx) = 1;
    A(kPyIdx,kVyIdx) = 1;
    A(kPzIdx,kVzIdx) = 1;

    B(kVxIdx,kAxIdx) = 1;
    B(kVyIdx,kAyIdx) = 1;
    B(kVzIdx,kAzIdx) = 1;

    A = A * ilqgames::time::kTimeStep;
    B = B * ilqgames::time::kTimeStep;
}

inline float PointPlayer3D::DistanceBetween(const Eigen::VectorXf& x0,
                                            const Eigen::VectorXf& x1) const {

    // Squared distance in position space.
    const float dx = x0(kPxIdx) - x1(kPxIdx);
    const float dy = x0(kPyIdx) - x1(kPyIdx);
    const float dz = x0(kPzIdx) - x1(kPzIdx);
    return dx * dx + dy * dy + dz * dz;
}

}  // namespace ilqgames_planning

#endif // POINT_PLAYER_3D_H
