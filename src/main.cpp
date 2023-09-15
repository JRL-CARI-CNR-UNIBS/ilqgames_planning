#include <ilqgames_planning/hand_tcp_point3D.h>
#include <ilqgames/solver/ilq_solver.h>
#include <ilqgames/solver/augmented_lagrangian_solver.h>
#include <ilqgames/utils/check_local_nash_equilibrium.h>
#include <ilqgames/utils/compute_strategy_costs.h>
#include <ilqgames_planning/receding_horizon_simulator.h>

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
DEFINE_bool(last_traj, false, "Should the solver only dump the last trajectory?");
DEFINE_string(experiment_name, "hand_tcp_point3D", "Name for the experiment.");

// Open- vs closed-loop & receding horizon
DEFINE_bool(open_loop, false, "Use open loop (vs. feedback) solver.");
DEFINE_bool(receding_horizon, false, "Solve in a receding horizon fashion.");

// Regularization.
//DEFINE_double(state_regularization, 1.0, "State regularization.");
//DEFINE_double(control_regularization, 1.0, "Control regularization.");

// Linesearch parameters.
//DEFINE_bool(linesearch, true, "Should the solver linesearch?"); //DEFAULT: true
//DEFINE_double(initial_alpha_scaling, 0.25, "Initial step size in linesearch."); //DEFAULT: 0.25
//DEFINE_double(convergence_tolerance, 0.1, "KKT squared error tolerance."); //DEFAULT: 0.1
//DEFINE_double(expected_decrease, 0.1, "KKT sq err expected decrease per iter."); //DEFAULT: 0.1

int find_max_idx_in_dir(const std::string dir); // custom function to find the maximum index in a given directory
void pack_dirs(const std::string common_string, const std::string dir);

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

    // === Set ilqgames solver parameters === //
    ilqgames::SolverParams params;

    // Consider a solution converged once max elementwise difference is below this
    // tolerance or solver has exceeded a maximum number of iterations.
    params.convergence_tolerance = 1e-5;
    params.max_solver_iters = 100;

    // Linesearch parameters. If flag is set 'true', then applied initial alpha
    // scaling to all strategies and backs off geometrically at the given rate for
    // the specified number of steps.
    params.linesearch = true;
    params.initial_alpha_scaling = 0.98;
    params.geometric_alpha_scaling = 0.01;
    params.max_backtracking_steps = 500;
    params.expected_decrease_fraction = 0.1;

    // Whether solver should shoot for an open loop or feedback Nash.
    params.open_loop = FLAGS_open_loop;

    // State and control regularization.
    params.state_regularization = 1.0;
    params.control_regularization = 1.0;

    // Augmented Lagrangian parameters.
    params.unconstrained_solver_max_iters = 100;
    params.geometric_mu_scaling = 1.1;
    params.geometric_mu_downscaling = 0.5;
    params.geometric_lambda_downscaling = 0.5;
    params.constraint_error_tolerance = 1e-1;

    // Should the solver reset problem/constraint params to their initial values.
    // NOTE: defaults to true.
    params.reset_problem = true;
    params.reset_lambdas = true;
    params.reset_mu = true;

    // === END Set ilqgames solver parameters === //

    // Solve for feedback equilibrium
    auto problem = std::make_shared<ilqgames_planning::HandTcpPoint3D>();
    problem->Initialize();
    LOG(INFO) << "Is the problem constrained? " << (problem->IsConstrained() ? "YES" : "NO");
    ilqgames::AugmentedLagrangianSolver solver(problem, params);

    if (!FLAGS_receding_horizon) {
        // Solve the game (one-step)
        const auto start = std::chrono::system_clock::now();
        bool solver_converged = false;
        const std::shared_ptr<const ilqgames::SolverLog> log = solver.Solve(&solver_converged);

        LOG(INFO) << "Did the solver converge? " << (solver_converged ? "YES" : "NO");
        LOG(INFO) << "Solver completed in "
                  << std::chrono::duration<ilqgames::Time>(
                     std::chrono::system_clock::now() - start).count()
                  << " seconds.";

        // Compute strategy costs
        problem->OverwriteSolution(log->FinalOperatingPoint(),
                                   log->FinalStrategies());
        const std::vector<float> total_costs =
            ilqgames::ComputeStrategyCosts(*problem);

        LOG(INFO) << "Total strategy costs are: ";
        for (const float c : total_costs)
            LOG(INFO) << c;

//        // Check if solution satisfies sufficient condition for being a local Nash [NOT WORKING]
//        LOG(INFO) << "Checking sufficient condition for local Nash equilibrium...";
//        const bool sufficient_cond_local_nash = ilqgames::CheckSufficientLocalNashEquilibrium(*problem);
//        if (sufficient_cond_local_nash)
//            LOG(INFO) << "Solution is a local Nash.";
//        else
//            LOG(INFO) << "Solution may not be a local Nash.";

        // Check if solution satisfies numerical condition for being a local Nash
        LOG(INFO) << "Checking numerical condition for local Nash equilibrium...";
        constexpr float kMaxPerturbation = 1e-10;
        const bool is_numerical_local_nash = NumericalCheckLocalNashEquilibrium(
            *problem, kMaxPerturbation, false);
        if (is_numerical_local_nash)
            LOG(INFO) << "Feedback solution is a local Nash.";
        else
            LOG(INFO) << "Feedback solution is not a local Nash.";

        // Create log list
        const std::vector<std::shared_ptr<const ilqgames::SolverLog>> logs = {log};

        // Dump the logs and/or exit
        if (FLAGS_save) {
            if (FLAGS_experiment_name == "") {
                CHECK(log->Save(FLAGS_last_traj));

            } else {
                if (fs::is_empty(ILQGAMES_LOG_DIR))
                    CHECK(log->Save(FLAGS_last_traj, FLAGS_experiment_name + "_0"));
                else {
                    CHECK(log->Save(FLAGS_last_traj, FLAGS_experiment_name + "_" + std::to_string(find_max_idx_in_dir(ILQGAMES_LOG_DIR)+1)));
                }
            }
        }
    }

    else {
        // Solve the game (receding horizon)
        constexpr ilqgames::Time kFinalTime = 10.0;       // DEFAULT: 10 s -> CANNOT BE CHANGED ?!
        constexpr ilqgames::Time kPlannerRuntime = 0.25;  // DEFAULT: 0.25 s
        constexpr ilqgames::Time kExtraTime = 0.25;       // DEFAULT: 0.25 s
        const std::vector<std::shared_ptr<const ilqgames::SolverLog>> logs =
            ilqgames_planning::RecedingHorizonSimulator(kFinalTime, kPlannerRuntime, kExtraTime, &solver); // using customized simulator

        // Dump the logs and/or exit
        if (FLAGS_save) {
            std::string new_dir;
            if (fs::is_empty(ILQGAMES_LOG_DIR))
                new_dir = FLAGS_experiment_name + "_receding_0";
            else
                new_dir = FLAGS_experiment_name + "_receding_" + std::to_string(find_max_idx_in_dir(ILQGAMES_LOG_DIR)+1);

            int i = 0;
            for (auto &log : logs) {
                if (FLAGS_experiment_name == "") {
                    CHECK(log->Save(FLAGS_last_traj));

                } else {
                    if (i == 0)
                        CHECK(log->Save(FLAGS_last_traj, new_dir + "_iter_0"));
                    else
                        CHECK(log->Save(FLAGS_last_traj, new_dir + "_iter_" + std::to_string(i)));
                }
                i++;
            }
            pack_dirs(new_dir, ILQGAMES_LOG_DIR);
        }
    }

    return 0;
}


int find_max_idx_in_dir(const std::string dir) {
    std::regex rgx("_[^_]+$");
    std::smatch match;
    std::string idx_str;
    int max_idx = 0;
    std::vector<int> idxs = {};

    for (const auto & entry : fs::directory_iterator(dir)) {
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


void pack_dirs(const std::string common_string, const std::string current_dir) {

    fs::create_directory(common_string);

    int i = 0;
    for (const auto & entry : fs::directory_iterator(current_dir)) {
        const std::string dir_to_move = entry.path().string();
        const std::string filename = entry.path().filename();

        if (dir_to_move.find(common_string) != std::string::npos) {
            std::string dest_dir = current_dir + fs::path::preferred_separator + common_string;
            std::string command = "mv " + dir_to_move + " " + dest_dir;
//            std::cout << command << std::endl;
            system(command.c_str());

            // Remove dir created during first iteration (written "/0" instead of full path)
            if (i == 0) {
                std::string new_dir = current_dir + fs::path::preferred_separator + common_string + fs::path::preferred_separator + filename;
                std::string old_dir = dest_dir + fs::path::preferred_separator + "0";
                fs::create_directory(new_dir);
                std::string command = "mv " + old_dir + fs::path::preferred_separator + "* " + new_dir;
//                std::cout << command << std::endl;
                system(command.c_str());

                command = "rm -r " + old_dir;
                system(command.c_str());
            }
        }
        i++;
    }
}
