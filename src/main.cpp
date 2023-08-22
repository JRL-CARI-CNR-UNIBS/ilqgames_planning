#include <ilqgames_planning/hand_tcp_point3D.h>
#include <ilqgames/solver/ilq_solver.h>
#include <ilqgames/solver/augmented_lagrangian_solver.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <stdio.h>

#include <vector>
#include <regex>
#include <filesystem>
namespace fs = std::filesystem;

// Optional log saving and visualization.
DEFINE_bool(save, true, "Optionally save solver logs to disk.");
DEFINE_bool(viz, false, "Visualize results in a GUI.");
DEFINE_bool(last_traj, false,
            "Should the solver only dump the last trajectory?");
DEFINE_string(experiment_name, "hand_tcp_point3D", "Name for the experiment.");

// Regularization.
DEFINE_double(state_regularization, 1.0, "State regularization.");
DEFINE_double(control_regularization, 1.0, "Control regularization.");

// Linesearch parameters.
DEFINE_bool(linesearch, true, "Should the solver linesearch?"); //DEFAULT: true
DEFINE_double(initial_alpha_scaling, 0.25, "Initial step size in linesearch.");
DEFINE_double(convergence_tolerance, 0.01, "KKT squared error tolerance.");
DEFINE_double(expected_decrease, 0.1, "KKT sq err expected decrease per iter.");

int find_max_idx_in_logdir(); // custom function to find the maximum index in the logging directory


int main(int argc, char **argv) {

    // Set the logging directory
    const std::string log_file =
        ILQGAMES_LOG_DIR + std::string("/") + FLAGS_experiment_name + std::string(".log");
    FLAGS_v = 2; // Set verbosity log level (useful to let the ilqgames library display more log messages)
    google::SetLogDestination(0, log_file.c_str());
    google::InitGoogleLogging(argv[0]);

    // Init Command Line Flags
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    FLAGS_logtostderr = true;

    // Set ilqgames solver parameters
    ilqgames::SolverParams params;
    params.max_backtracking_steps = 500; //DEFAULT: 100
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
    LOG(INFO) << "Is the problem constrained? " << (problem->IsConstrained() ? "YES" : "NO");
    ilqgames::AugmentedLagrangianSolver solver(problem, params);

    // Solve the game
    const auto start = std::chrono::system_clock::now();
    bool value = false;
    bool* solver_converged = &value;
    const std::shared_ptr<const ilqgames::SolverLog> log = solver.Solve(solver_converged);
    LOG(INFO) << "Did the solver converge? " << (*solver_converged ? "YES" : "NO");
    if (*solver_converged == true)
        LOG(INFO) << "Solver converged.";

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
            if (fs::is_empty(ILQGAMES_LOG_DIR))
                CHECK(log->Save(FLAGS_last_traj, FLAGS_experiment_name + "_0"));
            else {
                CHECK(log->Save(FLAGS_last_traj, FLAGS_experiment_name + "_" + std::to_string(find_max_idx_in_logdir()+1)));
            }
        }
    }

    return 0;
}


int find_max_idx_in_logdir() {
    std::regex rgx("_[^_]+$");
    std::smatch match;
    std::string idx_str;
    int max_idx = 0;
    std::vector<int> idxs = {};

    for (const auto & entry : fs::directory_iterator(ILQGAMES_LOG_DIR)) {
        const std::string filename = entry.path().c_str();
//                    std::cout << filename << std::endl;

        if (std::regex_search(filename.begin(), filename.end(), match, rgx)) {
            idx_str = match.str(0).substr(1,match.str(0).size());
//                    std::cout << "Matched substring: " << idx_str << std::endl;
            idxs.push_back(stoi(idx_str));
        }
    }
    return *max_element(idxs.begin(), idxs.end());
}
