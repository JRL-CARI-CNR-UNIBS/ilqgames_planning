# Common settings

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


# hand_tcp_point3D_receding_targetPosCost_0

// Cost weights
static constexpr float kHumanHandGoalXCost = 1.0;
static constexpr float kHumanHandGoalYCost = 1.0;
static constexpr float kHumanHandGoalZCost = 1.0;

static constexpr float kRobotTcpGoalXCost = 1.0;
static constexpr float kRobotTcpGoalYCost = 1.0;
static constexpr float kRobotTcpGoalZCost = 1.0;

static constexpr float kHumanHandGoalVxCost = 0.0;
static constexpr float kHumanHandGoalVyCost = 0.0;
static constexpr float kHumanHandGoalVzCost = 0.0;

static constexpr float kRobotTcpGoalVxCost = 0.0;
static constexpr float kRobotTcpGoalVyCost = 0.0;
static constexpr float kRobotTcpGoalVzCost = 0.0;

static constexpr float kHumanHandNominalVxCost = 0.0;
static constexpr float kHumanHandNominalVyCost = 0.0;
static constexpr float kHumanHandNominalVzCost = 0.0;

static constexpr float kRobotTcpNominalVxCost = 0.0;
static constexpr float kRobotTcpNominalVyCost = 0.0;
static constexpr float kRobotTcpNominalVzCost = 0.0;

static constexpr float kHumanHandAccYCost = 0.0;
static constexpr float kHumanHandAccZCost = 0.0;
static constexpr float kHumanHandAccXCost = 0.0;

static constexpr float kRobotTcpAccXCost = 0.0;
static constexpr float kRobotTcpAccYCost = 0.0;
static constexpr float kRobotTcpAccZCost = 0.0;

static constexpr float kProximityCostWeight = 0.0;

**Agents follow a straight path, as expected.**

# hand_tcp_point3D_receding_targetPosControlCost_1

// Cost weights
static constexpr float kHumanHandGoalXCost = 1.0;
static constexpr float kHumanHandGoalYCost = 1.0;
static constexpr float kHumanHandGoalZCost = 1.0;

static constexpr float kRobotTcpGoalXCost = 1.0;
static constexpr float kRobotTcpGoalYCost = 1.0;
static constexpr float kRobotTcpGoalZCost = 1.0;

static constexpr float kHumanHandGoalVxCost = 0.0;
static constexpr float kHumanHandGoalVyCost = 0.0;
static constexpr float kHumanHandGoalVzCost = 0.0;

static constexpr float kRobotTcpGoalVxCost = 0.0;
static constexpr float kRobotTcpGoalVyCost = 0.0;
static constexpr float kRobotTcpGoalVzCost = 0.0;

static constexpr float kHumanHandNominalVxCost = 0.0;
static constexpr float kHumanHandNominalVyCost = 0.0;
static constexpr float kHumanHandNominalVzCost = 0.0;

static constexpr float kRobotTcpNominalVxCost = 0.0;
static constexpr float kRobotTcpNominalVyCost = 0.0;
static constexpr float kRobotTcpNominalVzCost = 0.0;

static constexpr float kHumanHandAccYCost = 1.0;
static constexpr float kHumanHandAccZCost = 1.0;
static constexpr float kHumanHandAccXCost = 1.0;

static constexpr float kRobotTcpAccXCost = 1.0;
static constexpr float kRobotTcpAccYCost = 1.0;
static constexpr float kRobotTcpAccZCost = 1.0;

static constexpr float kProximityCostWeight = 0.0;

**Agents follow a straight path, as expected.**

# hand_tcp_point3D_receding_proximityCost_2

// Cost weights
static constexpr float kHumanHandGoalXCost = 0.1;
static constexpr float kHumanHandGoalYCost = 0.1;
static constexpr float kHumanHandGoalZCost = 0.1;

static constexpr float kRobotTcpGoalXCost = 0.1;
static constexpr float kRobotTcpGoalYCost = 0.1;
static constexpr float kRobotTcpGoalZCost = 0.1;

static constexpr float kHumanHandGoalVxCost = 0.0;
static constexpr float kHumanHandGoalVyCost = 0.0;
static constexpr float kHumanHandGoalVzCost = 0.0;

static constexpr float kRobotTcpGoalVxCost = 0.0;
static constexpr float kRobotTcpGoalVyCost = 0.0;
static constexpr float kRobotTcpGoalVzCost = 0.0;

static constexpr float kHumanHandNominalVxCost = 0.0;
static constexpr float kHumanHandNominalVyCost = 0.0;
static constexpr float kHumanHandNominalVzCost = 0.0;

static constexpr float kRobotTcpNominalVxCost = 0.0;
static constexpr float kRobotTcpNominalVyCost = 0.0;
static constexpr float kRobotTcpNominalVzCost = 0.0;

static constexpr float kHumanHandAccYCost = 0.1;
static constexpr float kHumanHandAccZCost = 0.1;
static constexpr float kHumanHandAccXCost = 0.1;

static constexpr float kRobotTcpAccXCost = 0.1;
static constexpr float kRobotTcpAccYCost = 0.1;
static constexpr float kRobotTcpAccZCost = 0.1;

static constexpr float kProximityCostWeight = 150.0;
constexpr float kMinProximity = 1.0;  // m (threshold to activate proximity cost)

![Proximity Cost](./proximity_cost.png "Proximity Cost")

**Agents bend their future trajectories when they foresee coming too close, as expected.**

# hand_tcp_point3D_receding_targetVelCost_3

// Cost weights
static constexpr float kHumanHandGoalXCost = 0.1;
static constexpr float kHumanHandGoalYCost = 0.1;
static constexpr float kHumanHandGoalZCost = 0.1;

static constexpr float kRobotTcpGoalXCost = 0.1;
static constexpr float kRobotTcpGoalYCost = 0.1;
static constexpr float kRobotTcpGoalZCost = 0.1;

static constexpr float kHumanHandGoalVxCost = 5.0;
static constexpr float kHumanHandGoalVyCost = 5.0;
static constexpr float kHumanHandGoalVzCost = 5.0;

static constexpr float kRobotTcpGoalVxCost = 5.0;
static constexpr float kRobotTcpGoalVyCost = 5.0;
static constexpr float kRobotTcpGoalVzCost = 5.0;

static constexpr float kHumanHandNominalVxCost = 0.0;
static constexpr float kHumanHandNominalVyCost = 0.0;
static constexpr float kHumanHandNominalVzCost = 0.0;

static constexpr float kRobotTcpNominalVxCost = 0.0;
static constexpr float kRobotTcpNominalVyCost = 0.0;
static constexpr float kRobotTcpNominalVzCost = 0.0;

static constexpr float kHumanHandAccYCost = 0.1;
static constexpr float kHumanHandAccZCost = 0.1;
static constexpr float kHumanHandAccXCost = 0.1;

static constexpr float kRobotTcpAccXCost = 0.1;
static constexpr float kRobotTcpAccYCost = 0.1;
static constexpr float kRobotTcpAccZCost = 0.1;

static constexpr float kProximityCostWeight = 150.0;
constexpr float kMinProximity = 0.0;  // m (threshold to activate proximity cost)

**Agents tend to slow down immediately, traveling a shorter path, as expected.**

# hand_tcp_point3D_receding_nominalVelCost_4

// Cost weights
static constexpr float kHumanHandGoalXCost = 0.1;
static constexpr float kHumanHandGoalYCost = 0.1;
static constexpr float kHumanHandGoalZCost = 0.1;

static constexpr float kRobotTcpGoalXCost = 0.1;
static constexpr float kRobotTcpGoalYCost = 0.1;
static constexpr float kRobotTcpGoalZCost = 0.1;

static constexpr float kHumanHandGoalVxCost = 5.0;
static constexpr float kHumanHandGoalVyCost = 5.0;
static constexpr float kHumanHandGoalVzCost = 5.0;

static constexpr float kRobotTcpGoalVxCost = 5.0;
static constexpr float kRobotTcpGoalVyCost = 5.0;
static constexpr float kRobotTcpGoalVzCost = 5.0;

static constexpr float kHumanHandNominalVxCost = 10.0;
static constexpr float kHumanHandNominalVyCost = 10.0;
static constexpr float kHumanHandNominalVzCost = 10.0;

static constexpr float kRobotTcpNominalVxCost = 10.0;
static constexpr float kRobotTcpNominalVyCost = 10.0;
static constexpr float kRobotTcpNominalVzCost = 10.0;

static constexpr float kHumanHandAccYCost = 0.1;
static constexpr float kHumanHandAccZCost = 0.1;
static constexpr float kHumanHandAccXCost = 0.1;

static constexpr float kRobotTcpAccXCost = 0.1;
static constexpr float kRobotTcpAccYCost = 0.1;
static constexpr float kRobotTcpAccZCost = 0.1;

static constexpr float kProximityCostWeight = 150.0;
constexpr float kMinProximity = 0.0;  // m (threshold to activate proximity cost)

// Nominal speed
static constexpr float kHumanHandNominalV = -3.0;    // m/s
static constexpr float kRobotTcpNominalV = 1.0;     // m/s

**Agents tend to maintain the specified velocity, as expected.**
