#include <ilqgames_planning/point_player_3d.h>
#include <ilqgames_planning/hand_tcp_point3D.h>
#include <ilqgames/dynamics/concatenated_dynamical_system.h>
#include <ilqgames/cost/quadratic_cost.h>
#include <ilqgames_planning/proximity_cost_3d.h>
#include <ilqgames/constraint/single_dimension_constraint.h>

namespace ilqgames_planning {

namespace {

// TimeStep & TimeHorizon
// Edit variables kTimeStep and kTimeHorizon in file
// "/ilqgames/include/ilqgames/utils/types.h:133" and recompile the ilqgames library
// (Default values: kTimeStep = 0.1, kTimeHorizon = 10.0)

// Input contraints
//static constexpr float kHumanHandMaxAx = 10.0;  // m/s^2
//static constexpr float kHumanHandMaxAy = 10.0;  // m/s^2
//static constexpr float kHumanHandMaxAz = 10.0;  // m/s^2

//static constexpr float kRobotTcpMaxAx = 50.0;  // m/s^2
//static constexpr float kRobotTcpMaxAy = 50.0;  // m/s^2
//static constexpr float kRobotTcpMaxAz = 50.0;  // m/s^2

// Cost weights
static constexpr float kHumanHandGoalXCost = 1.0;
static constexpr float kHumanHandGoalYCost = 1.0;
static constexpr float kHumanHandGoalZCost = 1.0;

static constexpr float kRobotTcpGoalXCost = 1.0;
static constexpr float kRobotTcpGoalYCost = 1.0;
static constexpr float kRobotTcpGoalZCost = 1.0;

static constexpr float kHumanHandGoalVxCost = 1.0;
static constexpr float kHumanHandGoalVyCost = 1.0;
static constexpr float kHumanHandGoalVzCost = 1.0;

static constexpr float kRobotTcpGoalVxCost = 1.0;
static constexpr float kRobotTcpGoalVyCost = 1.0;
static constexpr float kRobotTcpGoalVzCost = 1.0;

static constexpr float kHumanHandAccYCost = 1.0;
static constexpr float kHumanHandAccZCost = 1.0;
static constexpr float kHumanHandAccXCost = 1.0;

static constexpr float kRobotTcpAccXCost = 1.0;
static constexpr float kRobotTcpAccYCost = 1.0;
static constexpr float kRobotTcpAccZCost = 1.0;

static constexpr float kProximityCostWeight = 100.0; -CHECK: NOT WORKING!

//// Nominal speed
//static constexpr float kHumanHandNominalV = 3.0;    // m/s
//static constexpr float kRobotTcpNominalV = 4.0;     // m/s

// Initial state
static constexpr float kHumanHandInitialX = 10.0;    // m
static constexpr float kHumanHandInitialY = 10.0;    // m
static constexpr float kHumanHandInitialZ = 10.0;    // m

static constexpr float kRobotTcpInitialX = -10.0;    // m
static constexpr float kRobotTcpInitialY = -10.0;    // m
static constexpr float kRobotTcpInitialZ = -10.0;    // m

static constexpr float kHumanHandInitialVx = 0.0;   // m/s
static constexpr float kHumanHandInitialVy = 0.0;   // m/s
static constexpr float kHumanHandInitialVz = 0.0;   // m/s

static constexpr float kRobotTcpInitialVx = 0.0;    // m/s
static constexpr float kRobotTcpInitialVy = 0.0;    // m/s
static constexpr float kRobotTcpInitialVz = 0.0;    // m/s

// Target state
static constexpr float kHumanHandTargetX = -10.0;    // m
static constexpr float kHumanHandTargetY = -10.0;    // m
static constexpr float kHumanHandTargetZ = -10.0;    // m

static constexpr float kRobotTcpTargetX = 5.0;      // m
static constexpr float kRobotTcpTargetY = 5.0;      // m
static constexpr float kRobotTcpTargetZ = 5.0;      // m

static constexpr float kHumanHandTargetVx = 0.0;   // m/s
static constexpr float kHumanHandTargetVy = 0.0;   // m/s
static constexpr float kHumanHandTargetVz = 0.0;   // m/s

static constexpr float kRobotTcpTargetVx = 0.0;    // m/s
static constexpr float kRobotTcpTargetVy = 0.0;    // m/s
static constexpr float kRobotTcpTargetVz = 0.0;    // m/s

// State dimensions
using HumanHand = PointPlayer3D;
using RobotTcp = PointPlayer3D;

static const ilqgames::Dimension kHumanHandXIdx = HumanHand::kPxIdx;
static const ilqgames::Dimension kHumanHandVxIdx = HumanHand::kVxIdx;
static const ilqgames::Dimension kHumanHandYIdx = HumanHand::kPyIdx;
static const ilqgames::Dimension kHumanHandVyIdx = HumanHand::kVyIdx;
static const ilqgames::Dimension kHumanHandZIdx = HumanHand::kPzIdx;
static const ilqgames::Dimension kHumanHandVzIdx = HumanHand::kVzIdx;

static const ilqgames::Dimension kRobotTcpXIdx = HumanHand::kNumXDims + RobotTcp::kPxIdx;
static const ilqgames::Dimension kRobotTcpVxIdx = HumanHand::kNumXDims + RobotTcp::kVxIdx;
static const ilqgames::Dimension kRobotTcpYIdx = HumanHand::kNumXDims + RobotTcp::kPyIdx;
static const ilqgames::Dimension kRobotTcpVyIdx = HumanHand::kNumXDims + RobotTcp::kVyIdx;
static const ilqgames::Dimension kRobotTcpZIdx = HumanHand::kNumXDims + RobotTcp::kPzIdx;
static const ilqgames::Dimension kRobotTcpVzIdx = HumanHand::kNumXDims + RobotTcp::kVzIdx;

}  // anonymous namespace

void HandTcpPoint3D::ConstructDynamics() {
    // Create dynamics. In this case, we have two points in 3D space with decoupled dynamics (they are only coupled through the cost structure of the game). This is expressed in the ConcatenatedDynamicalSystem class. Here, each player's dynamics follow that of a double-integrator.
    dynamics_.reset(new ilqgames::ConcatenatedDynamicalSystem(
                    {std::make_shared<HumanHand>(),
                     std::make_shared<RobotTcp>()}));

//    dynamics_.reset(new ilqgames::ConcatenatedDynamicalSystem(
//                    {std::make_shared<HumanHand>()}));
}

void HandTcpPoint3D::ConstructInitialState() {
    // Set up initial state. Initially, this is zero, but then we override
    // individual dimensions to match the desired initial conditions above.
    x0_ = Eigen::VectorXf::Zero(dynamics_->XDim());

    x0_(kHumanHandXIdx) = kHumanHandInitialX;
    x0_(kHumanHandVxIdx) = kHumanHandInitialVx;
    x0_(kHumanHandYIdx) = kHumanHandInitialY;
    x0_(kHumanHandVyIdx) = kHumanHandInitialVy;
    x0_(kHumanHandZIdx) = kHumanHandInitialZ;
    x0_(kHumanHandVzIdx) = kHumanHandInitialVz;

    x0_(kRobotTcpXIdx) = kRobotTcpInitialX;
    x0_(kRobotTcpVxIdx) = kRobotTcpInitialVx;
    x0_(kRobotTcpYIdx) = kRobotTcpInitialY;
    x0_(kRobotTcpVyIdx) = kRobotTcpInitialVy;
    x0_(kRobotTcpZIdx) = kRobotTcpInitialZ;
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
    const auto humanHand_goalX_cost = std::make_shared<ilqgames::QuadraticCost>(
        kHumanHandGoalXCost, kHumanHandXIdx, kHumanHandTargetX, "humanHand_goalX_cost");
    const auto humanHand_goalY_cost = std::make_shared<ilqgames::QuadraticCost>(
        kHumanHandGoalYCost, kHumanHandYIdx, kHumanHandTargetY, "humanHand_goalY_cost");
    const auto humanHand_goalZ_cost = std::make_shared<ilqgames::QuadraticCost>(
        kHumanHandGoalZCost, kHumanHandZIdx, kHumanHandTargetZ, "humanHand_goalZ_cost");

    humanHand_cost.AddStateCost(humanHand_goalX_cost);
    humanHand_cost.AddStateCost(humanHand_goalY_cost);
    humanHand_cost.AddStateCost(humanHand_goalZ_cost);

    const auto robotTcp_goalX_cost = std::make_shared<ilqgames::QuadraticCost>(
        kRobotTcpGoalXCost, kRobotTcpXIdx, kRobotTcpTargetX, "robotTcp_goalX_cost");
    const auto robotTcp_goalY_cost = std::make_shared<ilqgames::QuadraticCost>(
        kRobotTcpGoalYCost, kRobotTcpYIdx, kRobotTcpTargetY, "robotTcp_goalY_cost");
    const auto robotTcp_goalZ_cost = std::make_shared<ilqgames::QuadraticCost>(
        kRobotTcpGoalZCost, kRobotTcpZIdx, kRobotTcpTargetZ, "robotTcp_goalZ_cost");

    robotTcp_cost.AddStateCost(robotTcp_goalX_cost);
    robotTcp_cost.AddStateCost(robotTcp_goalY_cost);
    robotTcp_cost.AddStateCost(robotTcp_goalZ_cost);


    // Quadratic state costs for target velocity
    const auto humanHand_goalVx_cost = std::make_shared<ilqgames::QuadraticCost>(
        kHumanHandGoalVxCost, kHumanHandVxIdx, kHumanHandTargetVx, "humanHand_goalVx_cost");
    const auto humanHand_goalVy_cost = std::make_shared<ilqgames::QuadraticCost>(
        kHumanHandGoalVyCost, kHumanHandVyIdx, kHumanHandTargetVy, "humanHand_goalVy_cost");
    const auto humanHand_goalVz_cost = std::make_shared<ilqgames::QuadraticCost>(
        kHumanHandGoalVzCost, kHumanHandVzIdx, kHumanHandTargetVz, "humanHand_goalVz_cost");

    humanHand_cost.AddStateCost(humanHand_goalVx_cost);
    humanHand_cost.AddStateCost(humanHand_goalVy_cost);
    humanHand_cost.AddStateCost(humanHand_goalVz_cost);

    const auto robotTcp_goalVx_cost = std::make_shared<ilqgames::QuadraticCost>(
        kRobotTcpGoalVxCost, kRobotTcpVxIdx, kRobotTcpTargetVx, "robotTcp_goalVx_cost");
    const auto robotTcp_goalVy_cost = std::make_shared<ilqgames::QuadraticCost>(
        kRobotTcpGoalVyCost, kRobotTcpVyIdx, kRobotTcpTargetVy, "robotTcp_goalVy_cost");
    const auto robotTcp_goalVz_cost = std::make_shared<ilqgames::QuadraticCost>(
        kRobotTcpGoalVzCost, kRobotTcpVzIdx, kRobotTcpTargetVz, "robotTcp_goalVz_cost");

    robotTcp_cost.AddStateCost(robotTcp_goalVx_cost);
    robotTcp_cost.AddStateCost(robotTcp_goalVy_cost);
    robotTcp_cost.AddStateCost(robotTcp_goalVz_cost);


    // Quadratic control costs
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

    // Penalize proximity when below the threshold set by kMinProximity (could also use a constraint).
    constexpr float kMinProximity = 1;  // m
    const std::shared_ptr<ilqgames_planning::ProximityCost3D> humanRobot_proximity_cost(
        new ilqgames_planning::ProximityCost3D(kProximityCostWeight,
                                               {kHumanHandXIdx, kHumanHandYIdx, kHumanHandZIdx},
                                               {kRobotTcpXIdx, kRobotTcpYIdx, kRobotTcpZIdx},
                                               kMinProximity, "Proximity"));

    humanHand_cost.AddStateCost(humanRobot_proximity_cost);
    robotTcp_cost.AddStateCost(humanRobot_proximity_cost);


    // State constraints (???)

    // Constrain each control input to lie in a (symmetrical) range.
//    const auto humanHand_accX_max_constraint = std::make_shared<ilqgames::SingleDimensionConstraint>(
//        HumanHand::kAxIdx, kHumanHandMaxAx, true,  "humanHand_accX_max_constraint");
//    const auto humanHand_accX_min_constraint = std::make_shared<ilqgames::SingleDimensionConstraint>(
//        HumanHand::kAxIdx, -kHumanHandMaxAx, false, "humanHand_accX_min_constraint");
//    const auto humanHand_accY_max_constraint = std::make_shared<ilqgames::SingleDimensionConstraint>(
//        HumanHand::kAyIdx, kHumanHandMaxAy, true,  "humanHand_accY_max_constraint");
//    const auto humanHand_accY_min_constraint = std::make_shared<ilqgames::SingleDimensionConstraint>(
//        HumanHand::kAyIdx, -kHumanHandMaxAy, false, "humanHand_accY_min_constraint");
//    const auto humanHand_accZ_max_constraint = std::make_shared<ilqgames::SingleDimensionConstraint>(
//        HumanHand::kAzIdx, kHumanHandMaxAz, true,  "humanHand_accZ_max_constraint");
//    const auto humanHand_accZ_min_constraint = std::make_shared<ilqgames::SingleDimensionConstraint>(
//        HumanHand::kAzIdx, -kHumanHandMaxAz, false, "humanHand_accZ_min_constraint");

//    humanHand_cost.AddControlConstraint(0, humanHand_accX_max_constraint);
//    humanHand_cost.AddControlConstraint(0, humanHand_accX_min_constraint);
//    humanHand_cost.AddControlConstraint(0, humanHand_accY_max_constraint);
//    humanHand_cost.AddControlConstraint(0, humanHand_accY_min_constraint);
//    humanHand_cost.AddControlConstraint(0, humanHand_accZ_max_constraint);
//    humanHand_cost.AddControlConstraint(0, humanHand_accZ_min_constraint);

//    const auto robotTcp_accX_max_constraint = std::make_shared<ilqgames::SingleDimensionConstraint>(
//        RobotTcp::kAxIdx, kRobotTcpMaxAx, true,  "robotTcp_accX_max_constraint");
//    const auto robotTcp_accX_min_constraint = std::make_shared<ilqgames::SingleDimensionConstraint>(
//        RobotTcp::kAxIdx, -kRobotTcpMaxAx, false, "robotTcp_accX_min_constraint");
//    const auto robotTcp_accY_max_constraint = std::make_shared<ilqgames::SingleDimensionConstraint>(
//        RobotTcp::kAyIdx, kRobotTcpMaxAy, true,  "robotTcp_accY_max_constraint");
//    const auto robotTcp_accY_min_constraint = std::make_shared<ilqgames::SingleDimensionConstraint>(
//        RobotTcp::kAyIdx, -kRobotTcpMaxAy, false, "robotTcp_accY_min_constraint");
//    const auto robotTcp_accZ_max_constraint = std::make_shared<ilqgames::SingleDimensionConstraint>(
//        RobotTcp::kAzIdx, kRobotTcpMaxAz, true,  "robotTcp_accZ_max_constraint");
//    const auto robotTcp_accZ_min_constraint = std::make_shared<ilqgames::SingleDimensionConstraint>(
//        RobotTcp::kAzIdx, -kRobotTcpMaxAz, false, "robotTcp_accZ_min_constraint");

//    robotTcp_cost.AddControlConstraint(1, robotTcp_accX_max_constraint);
//    robotTcp_cost.AddControlConstraint(1, robotTcp_accX_min_constraint);
//    robotTcp_cost.AddControlConstraint(1, robotTcp_accY_max_constraint);
//    robotTcp_cost.AddControlConstraint(1, robotTcp_accY_min_constraint);
//    robotTcp_cost.AddControlConstraint(1, robotTcp_accZ_max_constraint);
//    robotTcp_cost.AddControlConstraint(1, robotTcp_accZ_min_constraint);





//    // Encourage each player to go a given nominal speed.
//    const auto humanHand_nominalVx_cost = std::make_shared<ilqgames::QuadraticCost>(
//      kNominalVCostWeight, kP1VIdx, kP1NominalV, "NominalV");

//    humanHand_cost.AddStateCost(p1_nominal_v_cost);

//    const auto p2_nominal_v_cost = std::make_shared<QuadraticCost>(
//      kNominalVCostWeight, kP2VIdx, kP2NominalV, "NominalV");

//    robotTcp_cost.AddStateCost(p2_nominal_v_cost);


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

//    // Cost on final time
//    constexpr float kFinalTimeWindow = 0.5;  // s
//    const auto p1_goalx_cost = std::make_shared<FinalTimeCost>(
//        std::make_shared<QuadraticCost>(kGoalCostWeight, kP1XIdx, kP1GoalX),
//        time::kTimeHorizon - kFinalTimeWindow, "GoalX");
//    const auto p1_goaly_cost = std::make_shared<FinalTimeCost>(
//        std::make_shared<QuadraticCost>(kGoalCostWeight, kP1YIdx, kP1GoalY),
//        time::kTimeHorizon - kFinalTimeWindow, "GoalY");
//    p1_cost.AddStateCost(p1_goalx_cost);
//    p1_cost.AddStateCost(p1_goaly_cost);

//    const auto p2_goalx_cost = std::make_shared<FinalTimeCost>(
//        std::make_shared<QuadraticCost>(kGoalCostWeight, kP2XIdx, kP2GoalX),
//        time::kTimeHorizon - kFinalTimeWindow, "GoalX");
//    const auto p2_goaly_cost = std::make_shared<FinalTimeCost>(
//        std::make_shared<QuadraticCost>(kGoalCostWeight, kP2YIdx, kP2GoalY),
//        time::kTimeHorizon - kFinalTimeWindow, "GoalY");
//    p2_cost.AddStateCost(p2_goalx_cost);
//    p2_cost.AddStateCost(p2_goaly_cost);

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
