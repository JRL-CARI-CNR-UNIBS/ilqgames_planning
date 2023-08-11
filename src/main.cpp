#include <ilqgames_planning/hand_tcp_point3D.h>
#include <ilqgames/solver/ilq_solver.h>
#include <ilqgames/solver/augmented_lagrangian_solver.h>

#include <gflags/gflags.h>
#include <glog/logging.h>


// Optional log saving and visualization.
DEFINE_bool(save, true, "Optionally save solver logs to disk.");
DEFINE_bool(viz, false, "Visualize results in a GUI.");
DEFINE_bool(last_traj, false,
            "Should the solver only dump the last trajectory?");
DEFINE_string(experiment_name, "", "Name for the experiment.");

// Regularization.
DEFINE_double(state_regularization, 1.0, "State regularization.");
DEFINE_double(control_regularization, 1.0, "Control regularization.");

// Linesearch parameters.
DEFINE_bool(linesearch, true, "Should the solver linesearch?");
DEFINE_double(initial_alpha_scaling, 0.25, "Initial step size in linesearch.");
DEFINE_double(convergence_tolerance, 0.01, "KKT squared error tolerance.");
DEFINE_double(expected_decrease, 0.1, "KKT sq err expected decrease per iter.");


int main(int argc, char **argv) {
    const std::string log_file =
        ILQGAMES_LOG_DIR + std::string("/hand_tcp_point3D.log");
    google::SetLogDestination(0, log_file.c_str());
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    FLAGS_logtostderr = true;

    // Solve for open-loop information pattern
    ilqgames::SolverParams params;
    params.max_backtracking_steps = 100;
    params.linesearch = FLAGS_linesearch;
    params.expected_decrease_fraction = FLAGS_expected_decrease;
    params.initial_alpha_scaling = FLAGS_initial_alpha_scaling;
    params.convergence_tolerance = FLAGS_convergence_tolerance;
    params.state_regularization = FLAGS_state_regularization;
    params.control_regularization = FLAGS_control_regularization;
    params.open_loop = false;

    // Solve for feedback equilibrium
    auto problem = std::make_shared<ilqgames_planning::HandTcpPoint3D>();
    problem->Initialize();
    ilqgames::AugmentedLagrangianSolver solver(problem, params);

    // Solve the game
    const auto start = std::chrono::system_clock::now();
    const std::shared_ptr<const ilqgames::SolverLog> log = solver.Solve();
    const std::vector<std::shared_ptr<const ilqgames::SolverLog>> logs = {log};
    LOG(INFO) << "Solver completed in "
              << std::chrono::duration<ilqgames::Time>(
                 std::chrono::system_clock::now() - start).count()
              << " seconds.";

    // Dump the logs and/or exit
    if (FLAGS_save) {
        if (FLAGS_experiment_name == "") {
            CHECK(log->Save(FLAGS_last_traj));
        } else {
            CHECK(log->Save(FLAGS_last_traj, FLAGS_experiment_name));
        }
    }

    return 0;
}
