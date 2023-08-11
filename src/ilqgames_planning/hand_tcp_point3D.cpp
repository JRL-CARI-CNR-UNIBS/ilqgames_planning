#include <ilqgames_planning/point_player_3d.h>
#include <ilqgames_planning/hand_tcp_point3D.h>
#include <ilqgames/dynamics/concatenated_dynamical_system.h>
#include <ilqgames/cost/quadratic_cost.h>

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
static constexpr float kHumanHandGoalXCost = 25.0;
static constexpr float kHumanHandGoalYCost = 25.0;
static constexpr float kHumanHandGoalZCost = 25.0;

static constexpr float kRobotTcpGoalXCost = 25.0;
static constexpr float kRobotTcpGoalYCost = 25.0;
static constexpr float kRobotTcpGoalZCost = 25.0;

static constexpr float kHumanHandAccXCost = 5.0;
static constexpr float kHumanHandAccYCost = 5.0;
static constexpr float kHumanHandAccZCost =5.0;

static constexpr float kRobotTcpAccXCost = 5.0;
static constexpr float kRobotTcpAccYCost = 5.0;
static constexpr float kRobotTcpAccZCost = 5.0;

// Nominal speed
static constexpr float kHumanHandNominalV = 3.0;    // m/s
static constexpr float kRobotTcpNominalV = 4.0;     // m/s

// Initial state
static constexpr float kHumanHandInitialX = 0.0;    // m
static constexpr float kHumanHandInitialY = 0.0;    // m
static constexpr float kHumanHandInitialZ = 0.0;    // m

static constexpr float kRobotTcpInitialX = -2.0;    // m
static constexpr float kRobotTcpInitialY = 3.0;     // m
static constexpr float kRobotTcpInitialZ = 1.0;     // m

static constexpr float kHumanHandInitialVx = 0.5;   // m/s
static constexpr float kHumanHandInitialVy = 0.5;   // m/s
static constexpr float kHumanHandInitialVz = 0.5;   // m/s

static constexpr float kRobotTcpInitialVx = 0.0;    // m/s
static constexpr float kRobotTcpInitialVy = 0.0;    // m/s
static constexpr float kRobotTcpInitialVz = 0.0;    // m/s

// State dimensions
using HumanHand = PointPlayer3D;
using RobotTcp = PointPlayer3D;

static const ilqgames::Dimension kHumanHandXIdx = HumanHand::kPxIdx;
static const ilqgames::Dimension kHumanHandYIdx = HumanHand::kPyIdx;
static const ilqgames::Dimension kHumanHandZIdx = HumanHand::kPzIdx;
static const ilqgames::Dimension kHumanHandVxIdx = HumanHand::kVxIdx;
static const ilqgames::Dimension kHumanHandVyIdx = HumanHand::kVyIdx;
static const ilqgames::Dimension kHumanHandVzIdx = HumanHand::kVzIdx;

static const ilqgames::Dimension kRobotTcpXIdx = HumanHand::kNumXDims + RobotTcp::kPxIdx;
static const ilqgames::Dimension kRobotTcpYIdx = HumanHand::kNumXDims + RobotTcp::kPyIdx;
static const ilqgames::Dimension kRobotTcpZIdx = HumanHand::kNumXDims + RobotTcp::kPzIdx;
static const ilqgames::Dimension kRobotTcpVxIdx = HumanHand::kNumXDims + RobotTcp::kVxIdx;
static const ilqgames::Dimension kRobotTcpVyIdx = HumanHand::kNumXDims + RobotTcp::kVyIdx;
static const ilqgames::Dimension kRobotTcpVzIdx = HumanHand::kNumXDims + RobotTcp::kVzIdx;

}  // anonymous namespace

void HandTcpPoint3D::ConstructDynamics() {
    // Create dynamics. In this case, we have two points in 3D space with   decoupled dynamics (they are only coupled through the cost structure of the game). This is expressed in the ConcatenatedDynamicalSystem class. Here, each player's dynamics follow that of a double-integrator
    dynamics_.reset(new ilqgames::ConcatenatedDynamicalSystem(
                    {std::make_shared<HumanHand>(),
                     std::make_shared<RobotTcp>()}));
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

    x0_(kRobotTcpXIdx) = kRobotTcpInitialX;
    x0_(kRobotTcpYIdx) = kRobotTcpInitialY;
    x0_(kRobotTcpZIdx) = kRobotTcpInitialZ;
    x0_(kRobotTcpVxIdx) = kRobotTcpInitialVx;
    x0_(kRobotTcpVyIdx) = kRobotTcpInitialVy;
    x0_(kRobotTcpVzIdx) = kRobotTcpInitialVz;
}

void HandTcpPoint3D::ConstructPlayerCosts() {
    // Set up costs for all players. These are containers for holding each
    // player's constituent cost functions and constraints that hold pointwise in
    // time and can apply to either state or control (for *any* player).
    // These costs can also build in regularization on the state or the control,
    // which essentially boils down to adding a scaled identity matrix to each's
    // Hessian.
    player_costs_.emplace_back("HumanHand");
    player_costs_.emplace_back("RobotTcp");
    auto& humanHand_cost = player_costs_[0];
    auto& robotTcp_cost = player_costs_[1];

    // Quadratic state costs for target position
    const auto humanHand_goalX_cost = std::make_shared<ilqgames::QuadraticCost>(kHumanHandGoalXCost, HumanHand::kPxIdx, 0.0, "humanHand_goalX_cost");
    const auto humanHand_goalY_cost = std::make_shared<ilqgames::QuadraticCost>(kHumanHandGoalYCost, HumanHand::kPyIdx, 0.0, "humanHand_goalY_cost");
    const auto humanHand_goalZ_cost = std::make_shared<ilqgames::QuadraticCost>(kHumanHandGoalZCost, HumanHand::kPzIdx, 0.0, "humanHand_goalZ_cost");

    humanHand_cost.AddStateCost(humanHand_goalX_cost);
    humanHand_cost.AddStateCost(humanHand_goalY_cost);
    humanHand_cost.AddStateCost(humanHand_goalZ_cost);

    const auto robotTcp_goalX_cost = std::make_shared<ilqgames::QuadraticCost>(kRobotTcpGoalXCost, RobotTcp::kPxIdx, 0.0, "robotTcp_goalX_cost");
    const auto robotTcp_goalY_cost = std::make_shared<ilqgames::QuadraticCost>(kRobotTcpGoalYCost, RobotTcp::kPyIdx, 0.0, "robotTcp_goalY_cost");
    const auto robotTcp_goalZ_cost = std::make_shared<ilqgames::QuadraticCost>(kRobotTcpGoalZCost, RobotTcp::kPzIdx, 0.0, "robotTcp_goalZ_cost");

    robotTcp_cost.AddStateCost(robotTcp_goalX_cost);
    robotTcp_cost.AddStateCost(robotTcp_goalY_cost);
    robotTcp_cost.AddStateCost(robotTcp_goalZ_cost);


    // Quadratic control costs.
    const auto humanHand_accX_cost = std::make_shared<ilqgames::QuadraticCost>(
        kHumanHandAccXCost, HumanHand::kAxIdx, 0.0, "humanHand_accX_cost");
    const auto humanHand_accY_cost = std::make_shared<ilqgames::QuadraticCost>(
        kHumanHandAccYCost, HumanHand::kAyIdx, 0.0, "humanHand_accY_cost");
    const auto humanHand_accZ_cost = std::make_shared<ilqgames::QuadraticCost>(
        kHumanHandAccZCost, HumanHand::kAzIdx, 0.0, "humanHand_accZ_cost");

    humanHand_cost.AddControlCost(0, humanHand_accX_cost);
    humanHand_cost.AddControlCost(0, humanHand_accY_cost);
    humanHand_cost.AddControlCost(0, humanHand_accZ_cost);

    const auto robotTcp_accX_cost = std::make_shared<ilqgames::QuadraticCost>(
        kRobotTcpAccXCost, RobotTcp::kAxIdx, 0.0, "robotTcp_accX_cost");
    const auto robotTcp_accY_cost = std::make_shared<ilqgames::QuadraticCost>(
        kRobotTcpAccYCost, RobotTcp::kAyIdx, 0.0, "robotTcp_accY_cost");
    const auto robotTcp_accZ_cost = std::make_shared<ilqgames::QuadraticCost>(
        kRobotTcpAccZCost, RobotTcp::kAzIdx, 0.0, "robotTcp_accZ_cost");

    robotTcp_cost.AddControlCost(1, robotTcp_accX_cost);
    robotTcp_cost.AddControlCost(1, robotTcp_accY_cost);
    robotTcp_cost.AddControlCost(1, robotTcp_accZ_cost);



    // Constrain each control input to lie in an interval.
    // Step 3. Try uncommenting these blocks.
    // const auto p1_omega_max_constraint =
    //     std::make_shared<SingleDimensionConstraint>(
    //         P1::kOmegaIdx, kOmegaMax, true,  "Omega Constraint
    //         (Max)");
    // const auto p1_omega_min_constraint =
    //     std::make_shared<SingleDimensionConstraint>(
    //         P1::kOmegaIdx, -kOmegaMax, false, "Omega Constraint
    //         (Min)");
    // const auto p1_a_max_constraint =
    // std::make_shared<SingleDimensionConstraint>(
    //     P1::kAIdx, kAMax, true, "Acceleration Constraint (Max)");
    // const auto p1_a_min_constraint =
    // std::make_shared<SingleDimensionConstraint>(
    //     P1::kAIdx, -kAMax, false, "Acceleration Constraint
    //     (Min)");
    // p1_cost.AddControlConstraint(0, p1_omega_max_constraint);
    // p1_cost.AddControlConstraint(0, p1_omega_min_constraint);
    // p1_cost.AddControlConstraint(0, p1_a_max_constraint);
    // p1_cost.AddControlConstraint(0, p1_a_min_constraint);

    // const auto p2_omega_max_constraint =
    //     std::make_shared<SingleDimensionConstraint>(
    //         P2::kOmegaIdx, kOmegaMax, true, "Omega Constraint
    //         (Max)");
    // const auto p2_omega_min_constraint =
    //     std::make_shared<SingleDimensionConstraint>(
    //         P2::kOmegaIdx, -kOmegaMax, false, "Omega Constraint
    //         (Min)");
    // const auto p2_a_max_constraint =
    // std::make_shared<SingleDimensionConstraint>(
    //     P2::kAIdx, kAMax, true, "Acceleration Constraint (Max)");
    // const auto p2_a_min_constraint =
    // std::make_shared<SingleDimensionConstraint>(
    //     P2::kAIdx, -kAMax, false, "Acceleration Constraint
    //     (Min)");
    // p2_cost.AddControlConstraint(1, p2_omega_max_constraint);
    // p2_cost.AddControlConstraint(1, p2_omega_min_constraint);
    // p2_cost.AddControlConstraint(1, p2_a_max_constraint);
    // p2_cost.AddControlConstraint(1, p2_a_min_constraint);

    // Encourage each player to go a given nominal speed.
//    const auto p1_nominal_v_cost = std::make_shared<QuadraticCost>(
//      kNominalVCostWeight, kP1VIdx, kP1NominalV, "NominalV");
//    p1_cost.AddStateCost(p1_nominal_v_cost);

//    const auto p2_nominal_v_cost = std::make_shared<QuadraticCost>(
//      kNominalVCostWeight, kP2VIdx, kP2NominalV, "NominalV");
//    p2_cost.AddStateCost(p2_nominal_v_cost);

//    // Encourage each player to remain near the lane center. Could also add
//    // constraints to stay in the lane.
//    const Polyline2 lane1(
//      {Point2(kP1InitialX, -1000.0), Point2(kP1InitialX, 1000.0)});
//    const Polyline2 lane2({Point2(kP2InitialX, 1000.0), Point2(kP2InitialX, 5.0),
//                         Point2(kP2InitialX + 5.0, 0.0),
//                         Point2(kP2InitialX + 1000.0, 0.0)});

//    const std::shared_ptr<QuadraticPolyline2Cost> p1_lane_cost(
//      new QuadraticPolyline2Cost(kLaneCostWeight, lane1, {kP1XIdx, kP1YIdx},
//                                 "LaneCenter"));
//    p1_cost.AddStateCost(p1_lane_cost);

//    const std::shared_ptr<QuadraticPolyline2Cost> p2_lane_cost(
//      new QuadraticPolyline2Cost(kLaneCostWeight, lane2, {kP2XIdx, kP2YIdx},
//                                 "LaneCenter"));
//    p2_cost.AddStateCost(p2_lane_cost);

//    // Penalize proximity (could also use a constraint).
//    constexpr float kMinProximity = 6.0;  // m
//    const std::shared_ptr<ProximityCost> p1p2_proximity_cost(
//      new ProximityCost(kProximityCostWeight, {kP1XIdx, kP1YIdx},
//                        {kP2XIdx, kP2YIdx}, kMinProximity, "Proximity"));
//    p1_cost.AddStateCost(p1p2_proximity_cost);
//    p2_cost.AddStateCost(p1p2_proximity_cost);
}

inline std::vector<float> HandTcpPoint3D::Xs(const Eigen::VectorXf& x) const {
    return {x(kHumanHandXIdx), x(kRobotTcpXIdx)};
}

inline std::vector<float> HandTcpPoint3D::Ys(const Eigen::VectorXf& x) const {
    return {x(kHumanHandYIdx), x(kRobotTcpYIdx)};
}

inline std::vector<float> HandTcpPoint3D::Zs(const Eigen::VectorXf& x) const {
    return {x(kHumanHandZIdx), x(kRobotTcpZIdx)};
}

inline std::vector<float> HandTcpPoint3D::Vxs(const Eigen::VectorXf& x) const {
    return {x(kHumanHandVxIdx), x(kRobotTcpVxIdx)};
}

inline std::vector<float> HandTcpPoint3D::Vys(const Eigen::VectorXf& x) const {
    return {x(kHumanHandVyIdx), x(kRobotTcpVyIdx)};
}

inline std::vector<float> HandTcpPoint3D::Vzs(const Eigen::VectorXf& x) const {
    return {x(kHumanHandVzIdx), x(kRobotTcpVzIdx)};
}

}  // namespace ilqgames_planning
