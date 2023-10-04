#include <glog/logging.h>
#include <ilqgames_planning/point_player_3d.h>
#include <ilqgames_planning/hand_tcp_point3D.h>
#include <ilqgames/dynamics/concatenated_dynamical_system.h>
#include <ilqgames/cost/quadratic_cost.h>
#include <ilqgames_planning/proximity_cost_3d.h>
#include <ilqgames/cost/final_time_cost.h>
#include <ilqgames/constraint/single_dimension_constraint.h>
#include <ilqgames_planning/polyline3.h>
#include <ilqgames_planning/quadratic_polyline3_cost.h>
#include <ilqgames_planning/types.h>

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

// ===== Cost weights ===== //
// Target position cost
static constexpr float kHumanHandGoalXCost = 0.1;
static constexpr float kHumanHandGoalYCost = 0.1;
static constexpr float kHumanHandGoalZCost = 0.1;

static constexpr float kRobotTcpGoalXCost = 0.1;
static constexpr float kRobotTcpGoalYCost = 0.1;
static constexpr float kRobotTcpGoalZCost = 0.1;

// Target velocity cost
static constexpr float kHumanHandGoalVxCost = 5.0;
static constexpr float kHumanHandGoalVyCost = 5.0;
static constexpr float kHumanHandGoalVzCost = 5.0;

static constexpr float kRobotTcpGoalVxCost = 5.0;
static constexpr float kRobotTcpGoalVyCost = 5.0;
static constexpr float kRobotTcpGoalVzCost = 5.0;

// Nominal velocity cost
static constexpr float kHumanHandNominalVxCost = 10.0;
static constexpr float kHumanHandNominalVyCost = 10.0;
static constexpr float kHumanHandNominalVzCost = 10.0;

static constexpr float kRobotTcpNominalVxCost = 10.0;
static constexpr float kRobotTcpNominalVyCost = 10.0;
static constexpr float kRobotTcpNominalVzCost = 10.0;

// Control effort (acceleration) cost
static constexpr float kHumanHandAccXCost = 0.1;
static constexpr float kHumanHandAccYCost = 0.1;
static constexpr float kHumanHandAccZCost = 0.1;

static constexpr float kRobotTcpAccXCost = 0.1;
static constexpr float kRobotTcpAccYCost = 0.1;
static constexpr float kRobotTcpAccZCost = 0.1;

// Proximity cost
static constexpr float kProximityCostWeight = 150.0;
constexpr float kMinProximity = 0.0;  // m (threshold to activate proximity cost)

// Final time cost
static constexpr float kHumanHandFinalTimeXCost = 0.0;
static constexpr float kHumanHandFinalTimeYCost = 0.0;
static constexpr float kHumanHandFinalTimeZCost = 0.0;

static constexpr float kRobotTcpFinalTimeXCost = 0.0;
static constexpr float kRobotTcpFinalTimeYCost = 0.0;
static constexpr float kRobotTcpFinalTimeZCost = 0.0;

constexpr float kFinalTimeWindow = 0.5;  // s (threshold to activate final time cost)

// Reference trajectory cost
static constexpr float kLaneCostWeight = 0.0;

// ===== END Cost weights ===== //

// Nominal speed (the sign must be specified!)
static constexpr float kHumanHandNominalV = -3.0;    // m/s
static constexpr float kRobotTcpNominalV = 1.0;     // m/s

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
static constexpr float kHumanHandTargetX = -8.0;    // m
static constexpr float kHumanHandTargetY = -8.0;    // m
static constexpr float kHumanHandTargetZ = -8.0;    // m

static constexpr float kRobotTcpTargetX = 7.0;      // m
static constexpr float kRobotTcpTargetY = 9.0;      // m
static constexpr float kRobotTcpTargetZ = 8.0;      // m

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
    const std::shared_ptr<ilqgames_planning::ProximityCost3D> humanRobot_proximity_cost(
        new ilqgames_planning::ProximityCost3D(kProximityCostWeight,
                                               {kHumanHandXIdx, kHumanHandYIdx, kHumanHandZIdx},
                                               {kRobotTcpXIdx, kRobotTcpYIdx, kRobotTcpZIdx},
                                               kMinProximity, "proximity_cost"));

    humanHand_cost.AddStateCost(humanRobot_proximity_cost);
    robotTcp_cost.AddStateCost(humanRobot_proximity_cost);


    // Encourage each player to go a given nominal speed.
    const auto humanHand_nominalVx_cost = std::make_shared<ilqgames::QuadraticCost>(
      kHumanHandNominalVxCost, kHumanHandVxIdx, kHumanHandNominalV, "humanHand_nominalVx_cost");
    const auto humanHand_nominalVy_cost = std::make_shared<ilqgames::QuadraticCost>(
      kHumanHandNominalVyCost, kHumanHandVyIdx, kHumanHandNominalV, "humanHand_nominalVy_cost");
    const auto humanHand_nominalVz_cost = std::make_shared<ilqgames::QuadraticCost>(
      kHumanHandNominalVzCost, kHumanHandVzIdx, kHumanHandNominalV, "humanHand_nominalVz_cost");

    humanHand_cost.AddStateCost(humanHand_nominalVx_cost);
    humanHand_cost.AddStateCost(humanHand_nominalVy_cost);
    humanHand_cost.AddStateCost(humanHand_nominalVz_cost);

    const auto robotTcp_nominalVx_cost = std::make_shared<ilqgames::QuadraticCost>(
      kRobotTcpNominalVxCost, kRobotTcpVxIdx, kRobotTcpNominalV, "robotTcp_nominalVx_cost");
    const auto robotTcp_nominalVy_cost = std::make_shared<ilqgames::QuadraticCost>(
      kRobotTcpNominalVyCost, kRobotTcpVyIdx, kRobotTcpNominalV, "robotTcp_nominalVy_cost");
    const auto robotTcp_nominalVz_cost = std::make_shared<ilqgames::QuadraticCost>(
      kRobotTcpNominalVzCost, kRobotTcpVzIdx, kRobotTcpNominalV, "robotTcp_nominalVz_cost");

    robotTcp_cost.AddStateCost(robotTcp_nominalVx_cost);
    robotTcp_cost.AddStateCost(robotTcp_nominalVy_cost);
    robotTcp_cost.AddStateCost(robotTcp_nominalVz_cost);


//    // Cost on final time
//    std::cout << ilqgames::time::kTimeHorizon << std::endl;

//    const auto humanHand_finalTimeX_cost = std::make_shared<ilqgames::FinalTimeCost>(
//        std::make_shared<ilqgames::QuadraticCost>(kHumanHandFinalTimeXCost, kHumanHandXIdx, kHumanHandTargetX [CHE TARGET VA MESSO QUI, TEMPO O SPAZIO??]),
//        ilqgames::time::kTimeHorizon - kFinalTimeWindow, "humanHand_finalTimeX_cost");
//    const auto humanHand_finalTimeY_cost = std::make_shared<ilqgames::FinalTimeCost>(
//        std::make_shared<ilqgames::QuadraticCost>(kHumanHandFinalTimeYCost, kHumanHandYIdx, kHumanHandTargetY [CHE TARGET VA MESSO QUI, TEMPO O SPAZIO??]),
//        ilqgames::time::kTimeHorizon - kFinalTimeWindow, "humanHand_finalTimeY_cost");
//    const auto humanHand_finalTimeZ_cost = std::make_shared<ilqgames::FinalTimeCost>(
//        std::make_shared<ilqgames::QuadraticCost>(kHumanHandFinalTimeZCost, kHumanHandZIdx, kHumanHandTargetZ [CHE TARGET VA MESSO QUI, TEMPO O SPAZIO??]),
//        ilqgames::time::kTimeHorizon - kFinalTimeWindow, "humanHand_finalTimeZ_cost");

//    humanHand_cost.AddStateCost(humanHand_finalTimeX_cost);
//    humanHand_cost.AddStateCost(humanHand_finalTimeY_cost);
//    humanHand_cost.AddStateCost(humanHand_finalTimeZ_cost);

//    const auto robotTcp_finalTimeX_cost = std::make_shared<ilqgames::FinalTimeCost>(
//        std::make_shared<ilqgames::QuadraticCost>(kRobotTcpFinalTimeXCost, kRobotTcpXIdx, kRobotTcpTargetX [CHE TARGET VA MESSO QUI, TEMPO O SPAZIO??]),
//        ilqgames::time::kTimeHorizon - kFinalTimeWindow, "robotTcp_finalTimeX_cost");
//    const auto robotTcp_finalTimeY_cost = std::make_shared<ilqgames::FinalTimeCost>(
//        std::make_shared<ilqgames::QuadraticCost>(kRobotTcpFinalTimeYCost, kRobotTcpYIdx, kRobotTcpTargetY [CHE TARGET VA MESSO QUI, TEMPO O SPAZIO??]),
//        ilqgames::time::kTimeHorizon - kFinalTimeWindow, "robotTcp_finalTimeY_cost");
//    const auto robotTcp_finalTimeZ_cost = std::make_shared<ilqgames::FinalTimeCost>(
//        std::make_shared<ilqgames::QuadraticCost>(kRobotTcpFinalTimeZCost, kRobotTcpZIdx, kRobotTcpTargetZ [CHE TARGET VA MESSO QUI, TEMPO O SPAZIO??]),
//        ilqgames::time::kTimeHorizon - kFinalTimeWindow, "robotTcp_finalTimeZ_cost");

//    humanHand_cost.AddStateCost(robotTcp_finalTimeX_cost);
//    humanHand_cost.AddStateCost(robotTcp_finalTimeY_cost);
//    humanHand_cost.AddStateCost(robotTcp_finalTimeZ_cost);


    // Encourage each player to remain near the given trajectory.
    ilqgames_planning::Point3 P0_humanHand = ilqgames_planning::Point3(kHumanHandInitialX, kHumanHandInitialY, kHumanHandInitialZ);
    ilqgames_planning::Point3 P0_robotTcp = ilqgames_planning::Point3(kRobotTcpInitialX, kRobotTcpInitialY, kRobotTcpInitialZ);

    ilqgames_planning::Point3 Pf_humanHand = ilqgames_planning::Point3(kHumanHandTargetX, kHumanHandTargetY, kHumanHandTargetZ);
    ilqgames_planning::Point3 Pf_robotTcp = ilqgames_planning::Point3(kRobotTcpTargetX, kRobotTcpTargetY, kRobotTcpTargetZ);

    ilqgames_planning::Point3 PfP0_humanHand = Pf_humanHand - P0_humanHand;
    ilqgames_planning::Point3 random_vector_humanHand = ilqgames_planning::Point3(0.0, 0.0, 1.0);
    ilqgames_planning::Point3 orthogonal_vector_humanHand = PfP0_humanHand.cross3(random_vector_humanHand);

    ilqgames_planning::Point3 PfP0_robotTcp = Pf_robotTcp - P0_robotTcp;
    ilqgames_planning::Point3 random_vector_robotTcp = ilqgames_planning::Point3(0.0, 0.0, 1.0);
    ilqgames_planning::Point3 orthogonal_vector_robotTcp = PfP0_robotTcp.cross3(random_vector_robotTcp);

    ilqgames_planning::Point3 P1_humanHand = ilqgames_planning::Point3(kHumanHandInitialX + (kHumanHandTargetX - kHumanHandInitialX) / 4.0,
                                                                       kHumanHandInitialY + (kHumanHandTargetY - kHumanHandInitialY) / 4.0,
                                                                       kHumanHandInitialZ + (kHumanHandTargetZ - kHumanHandInitialZ) / 4.0) +
                                                                       orthogonal_vector_humanHand;

    ilqgames_planning::Point3 P1_robotTcp = ilqgames_planning::Point3(kRobotTcpInitialX + (kRobotTcpTargetX - kRobotTcpInitialX) / 4.0,
                                                                      kRobotTcpInitialY + (kRobotTcpTargetY - kRobotTcpInitialY) / 4.0,
                                                                      kRobotTcpInitialZ + (kRobotTcpTargetZ - kRobotTcpInitialZ) / 4.0) +
                                                                      orthogonal_vector_robotTcp;

    ilqgames_planning::Point3 P2_humanHand = ilqgames_planning::Point3(kHumanHandInitialX + (kHumanHandTargetX - kHumanHandInitialX) / 2.0,
                                                                       kHumanHandInitialY + (kHumanHandTargetY - kHumanHandInitialY) / 2.0,
                                                                       kHumanHandInitialZ + (kHumanHandTargetZ - kHumanHandInitialZ) / 2.0) +
                                                                       orthogonal_vector_humanHand;

    ilqgames_planning::Point3 P2_robotTcp = ilqgames_planning::Point3(kRobotTcpInitialX + (kRobotTcpTargetX - kRobotTcpInitialX) / 2.0,
                                                                      kRobotTcpInitialY + (kRobotTcpTargetY - kRobotTcpInitialY) / 2.0,
                                                                      kRobotTcpInitialZ + (kRobotTcpTargetZ - kRobotTcpInitialZ) / 2.0) +
                                                                      orthogonal_vector_robotTcp;

    ilqgames_planning::Point3 P3_humanHand = ilqgames_planning::Point3(kHumanHandInitialX + (kHumanHandTargetX - kHumanHandInitialX) * 3.0 / 4.0,
                                                                       kHumanHandInitialY + (kHumanHandTargetY - kHumanHandInitialY) * 3.0 / 4.0,
                                                                       kHumanHandInitialZ + (kHumanHandTargetZ - kHumanHandInitialZ) * 3.0 / 4.0) +
                                                                       orthogonal_vector_humanHand;

    ilqgames_planning::Point3 P3_robotTcp = ilqgames_planning::Point3(kRobotTcpInitialX + (kRobotTcpTargetX - kRobotTcpInitialX) * 3.0 / 4.0,
                                                                      kRobotTcpInitialY + (kRobotTcpTargetY - kRobotTcpInitialY) * 3.0 / 4.0,
                                                                      kRobotTcpInitialZ + (kRobotTcpTargetZ - kRobotTcpInitialZ) * 3.0 / 4.0) +
                                                                      orthogonal_vector_robotTcp;


    const ilqgames_planning::Polyline3 humanHand_traj({P0_humanHand, P1_humanHand, P2_humanHand, P3_humanHand, Pf_humanHand});

    const std::shared_ptr<ilqgames_planning::QuadraticPolyline3Cost> humanHand_traj_cost(
        new ilqgames_planning::QuadraticPolyline3Cost(kLaneCostWeight, humanHand_traj,
                                                      {kHumanHandXIdx, kHumanHandYIdx, kHumanHandZIdx}, "humanHand_traj_cost"));

    humanHand_cost.AddStateCost(humanHand_traj_cost);

    const ilqgames_planning::Polyline3 robotTcp_traj({P0_robotTcp, P1_robotTcp, P2_robotTcp, P3_robotTcp, Pf_robotTcp});

    const std::shared_ptr<ilqgames_planning::QuadraticPolyline3Cost> robotTcp_traj_cost(
        new ilqgames_planning::QuadraticPolyline3Cost(kLaneCostWeight, robotTcp_traj,
                                                      {kRobotTcpXIdx, kRobotTcpYIdx, kRobotTcpZIdx}, "robotTcp_traj_cost"));

    robotTcp_cost.AddStateCost(robotTcp_traj_cost);


// Lane boundaries -> Could also add constraints to stay in the lane.
//    const std::shared_ptr<QuadraticPolyline2Cost> p1_lane_cost(
//        new QuadraticPolyline2Cost(kLaneCostWeight, lane1, {kP1XIdx, kP1YIdx},
//                                   "LaneCenter"));
//    const std::shared_ptr<SemiquadraticPolyline2Cost> p1_lane_r_cost(
//        new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, lane1,
//                                       {kP1XIdx, kP1YIdx}, kLaneHalfWidth,
//                                       kOrientedRight, "LaneRightBoundary"));
//    const std::shared_ptr<SemiquadraticPolyline2Cost> p1_lane_l_cost(
//        new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, lane1,
//                                       {kP1XIdx, kP1YIdx}, -kLaneHalfWidth,
//                                       !kOrientedRight, "LaneLeftBoundary"));


//    // Max/min/nominal speed costs.
//    const std::shared_ptr<SemiquadraticNormCost> p1_min_v_cost(
//        new SemiquadraticNormCost(kMaxVCostWeight, {kP1VxIdx, kP1VyIdx}, kMinV,
//                                  !kOrientedRight, "MinV"));
//    const std::shared_ptr<SemiquadraticNormCost> p1_max_v_cost(
//        new SemiquadraticNormCost(kMaxVCostWeight, {kP1VxIdx, kP1VyIdx}, kP1MaxV,
//                                  kOrientedRight, "MaxV"));
//    const std::shared_ptr<QuadraticNormCost> p1_nominal_v_cost(
//        new QuadraticNormCost(kNominalVCostWeight, {kP1VxIdx, kP1VyIdx},
//                              kP1NominalV, "NominalV"));
//    p1_cost.AddStateCost(p1_min_v_cost);
//    p1_cost.AddStateCost(p1_max_v_cost);
//    p1_cost.AddStateCost(p1_nominal_v_cost);

//    const std::shared_ptr<SemiquadraticNormCost> p2_min_v_cost(
//        new SemiquadraticNormCost(kMaxVCostWeight, {kP2VxIdx, kP2VyIdx}, kMinV,
//                                  !kOrientedRight, "MinV"));
//    const std::shared_ptr<SemiquadraticNormCost> p2_max_v_cost(
//        new SemiquadraticNormCost(kMaxVCostWeight, {kP2VxIdx, kP2VyIdx}, kP2MaxV,
//                                  kOrientedRight, "MaxV"));
//    const std::shared_ptr<QuadraticNormCost> p2_nominal_v_cost(
//        new QuadraticNormCost(kNominalVCostWeight, {kP2VxIdx, kP2VyIdx},
//                              kP2NominalV, "NominalV"));
//    p2_cost.AddStateCost(p2_min_v_cost);
//    p2_cost.AddStateCost(p2_max_v_cost);
//    p2_cost.AddStateCost(p2_nominal_v_cost);

//    const std::shared_ptr<SemiquadraticNormCost> p3_min_v_cost(
//        new SemiquadraticNormCost(kMaxVCostWeight, {kP3VxIdx, kP3VyIdx}, kMinV,
//                                  !kOrientedRight, "MinV"));
//    const std::shared_ptr<SemiquadraticNormCost> p3_max_v_cost(
//        new SemiquadraticNormCost(kMaxVCostWeight, {kP3VxIdx, kP3VyIdx}, kP3MaxV,
//                                  kOrientedRight, "MaxV"));
//    const std::shared_ptr<QuadraticNormCost> p3_nominal_v_cost(
//        new QuadraticNormCost(kNominalVCostWeight, {kP3VxIdx, kP3VyIdx},
//                              kP3NominalV, "NominalV"));
//    p3_cost.AddStateCost(p3_min_v_cost);
//    p3_cost.AddStateCost(p3_max_v_cost);
//    p3_cost.AddStateCost(p3_nominal_v_cost);


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
