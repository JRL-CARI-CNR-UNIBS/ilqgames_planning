#include <ilqgames_planning/point_player_3d.h>
#include <ilqgames_planning/hand_tcp_point3D.h>
#include <ilqgames/dynamics/concatenated_dynamical_system.h>

namespace ilqgames_planning {

namespace {

// Input contraints
static constexpr float kHumanHandMaxAx = 4.0;  // m/s^2
static constexpr float kHumanHandMaxAy = 4.0;  // m/s^2
static constexpr float kHumanHandMaxAz = 4.0;  // m/s^2

static constexpr float kRobotTcpMaxAx = 3.0;  // m/s^2
static constexpr float kRobotTcpMaxAy = 3.0;  // m/s^2
static constexpr float kRobotTcpMaxAz = 3.0;  // m/s^2

// Cost weights
static constexpr float kOmegaCostWeight = 25.0;
static constexpr float kACostWeight = 15.0;
static constexpr float kNominalVCostWeight = 10.0;
static constexpr float kLaneCostWeight = 25.0;
static constexpr float kProximityCostWeight = 100.0;

// Nominal speed
static constexpr float kHumanHandNominalV = 8.0;  // m/s
static constexpr float kRobotTcpNominalV = 8.0;  // m/s

// Initial state
static constexpr float kHumanHandInitialX = 0.0;    // m
static constexpr float kHumanHandInitialY = -30.0;  // m
static constexpr float kHumanHandInitialZ = -30.0;  // m

static constexpr float kRobotTcpInitialX = -5.0;  // m
static constexpr float kRobotTcpInitialY = 30.0;  // m
static constexpr float kRobotTcpInitialZ = 30.0;  // m

static constexpr float kHumanHandInitialVx = 4.0;  // m/s
static constexpr float kHumanHandInitialVy = 4.0;  // m/s
static constexpr float kHumanHandInitialVz = 4.0;  // m/s

static constexpr float kRobotTcpInitialVx = 3.0;  // m/s
static constexpr float kRobotTcpInitialVy = 3.0;  // m/s
static constexpr float kRobotTcpInitialVz = 3.0;  // m/s

// State dimensions
using HumanHand = PointPlayer3D;
using RobotTcp = PointPlayer3D;

static const ilqgames::Dimension kHumanHandXIdx = HumanHand::kPxIdx;
static const ilqgames::Dimension kHumanHandYIdx = HumanHand::kPyIdx;
static const ilqgames::Dimension kHumanHandZIdx = HumanHand::kPzIdx;

static const ilqgames::Dimension kRobotTcpXIdx = HumanHand::kNumXDims + RobotTcp::kPxIdx;
static const ilqgames::Dimension kRobotTcpYIdx = HumanHand::kNumXDims + RobotTcp::kPyIdx;
static const ilqgames::Dimension kRobotTcpZIdx = HumanHand::kNumXDims + RobotTcp::kPzIdx;

}  // anonymous namespace

void HandTcpPoint3D::ConstructDynamics() {
    // Create dynamics. In this case, we have two points in 3D space with   decoupled dynamics (they are only coupled through the cost structure of the game). This is expressed in the ConcatenatedDynamicalSystem class. Here, each player's dynamics follow that of a double-integrator
    dynamics_.reset(new ilqgames::ConcatenatedDynamicalSystem(
                    {std::make_shared<HumanHand>,
                     std::make_shared<RobotTcp>}));
}

void HandTcpPoint3D::ConstructInitialState() {
    // Set up initial state. Initially, this is zero, but then we override
    // individual dimensions to match the desired initial conditions above.
    x0_ = Eigen::VectorXf::Zero(dynamics_->XDim());
    x0_(kHumanHandXIdx) = kHumanHandInitialX;
    x0_(kHumanHandYIdx) = kHumanHandInitialY;
    x0_(kHumanHandZIdx) = kHumanHandInitialZ;
    x0_(kHumanHandVxIdx) = kHumanHandInitialVx;
    x0_(kHumanHandVyIdx) = kHumanHandInitialVy;
    x0_(kHumanHandVzIdx) = kHumanHandInitialVz;

}


}  // namespace ilqgames_planning
