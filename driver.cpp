#include <string>
#include <iostream>
#include <ctime>
#include <sys/stat.h>
#include <boost/program_options.hpp>
#include "simulation.h"

namespace po = boost::program_options;
using namespace std;

void set_parameters(MAPDConfig& config, const po::variables_map& vm)
{
    config = get_preset(vm["algo"].as<string>());

    if (vm.count("mode")) {
        string mode = vm["mode"].as<string>();
        if (mode == "ONLINE") config.mode = MODE_ONLINE;
        else if (mode == "OFFLINE") config.mode = MODE_OFFLINE;
        else if (mode == "SEMI_ONLINE") config.mode = MODE_SEMI_ONLINE;
        else
            throw invalid_argument(
                "mode must be ONLINE, OFFLINE, or SEMI_ONLINE");
    }

    // Both LKH3-based TA algorithms are defined for the offline setting: the
    // complete task set must be known when the tour is read. TA-Hybrid may
    // reassign that fixed task set later, but it does not reveal new tasks.
    if ((config.assign_method == AM_LKH3_TSP ||
         config.assign_method == AM_LKH3_TSP_REASSIGN) &&
        config.mode != MODE_OFFLINE) {
        throw invalid_argument(
            "TA-Prioritized and TA-Hybrid support OFFLINE mode only");
    }

    string sa = vm["single_agent"].as<string>();
    if (sa == "MLA") config.single_agent = SA_MLA_SEQUENCE;
    else if (sa == "MLSIPP" || sa == "SIPP") config.single_agent = SA_MLSIPP_SEQUENCE;
    else if (sa == "STA") config.single_agent = SA_STA_TASK_EP;
    string mf = vm["mapf"].as<string>();
    if (mf == "PBS") config.mapf = MAPF_PBS;
    else if (mf == "wPBS") config.mapf = MAPF_wPBS;
    else if (mf == "CBS") config.mapf = MAPF_CBS;
    else if (mf == "PP" || mf == "PP_PER_TASK") config.mapf = MAPF_PP_PER_TASK;
    else if (mf == "PP_TASK_SEQUENCE") config.mapf = MAPF_PP_TASK_SEQUENCE;
    else if (mf != "default")
        throw invalid_argument("unsupported MAPF override: " + mf);

    if (vm.count("endpoint_strategy")) {
        string strategy = vm["endpoint_strategy"].as<string>();
        if (strategy == "WAIT_OR_NEAREST_SAFE")
            config.endpoint_strategy = WAIT_OR_NEAREST_SAFE;
        else if (strategy == "RETURN_TO_HOME")
            config.endpoint_strategy = RETURN_TO_HOME;
        else if (strategy == "NEAREST_WITH_STRICT_EXCLUSIONS")
            config.endpoint_strategy = NEAREST_WITH_STRICT_EXCLUSIONS;
        else if (strategy == "PAIRWISE_TASK_THEN_HOME")
            config.endpoint_strategy = PAIRWISE_TASK_THEN_HOME;
        else if (strategy == "WAIT_OR_NEAREST_FREE_NONTASK")
            config.endpoint_strategy = WAIT_OR_NEAREST_FREE_NONTASK;
        else if (strategy == "NEAREST_AVAILABLE")
            config.endpoint_strategy = NEAREST_AVAILABLE;
        else
            throw invalid_argument(
                "unsupported endpoint strategy: " + strategy);
    }
    config.seed = vm["seed"].as<int>();
    config.runtime_limit_seconds = vm["runtime_limit"].as<int>();
    if (config.runtime_limit_seconds < 0)
        throw invalid_argument("runtime_limit must be non-negative");
    config.pathfinding_runtime_limit_seconds =
        vm["pathfinding_runtime_limit"].as<int>();
    if (config.pathfinding_runtime_limit_seconds < 0)
        throw invalid_argument(
            "pathfinding_runtime_limit must be non-negative");

    // Algorithm-specific overrides. Other algorithms retain these values but
    // never read them.
    if (vm.count("task_sequence_limit")) {
        config.task_sequence_limit = vm["task_sequence_limit"].as<int>();
        if (config.task_sequence_limit <= 0)
            throw invalid_argument("task_sequence_limit must be positive");
    }
    if (vm.count("wpbs_replan_window")) {
        config.wpbs_replan_window = vm["wpbs_replan_window"].as<int>();
        if (config.wpbs_replan_window <= 0)
            throw invalid_argument("wpbs_replan_window must be positive");
    }
    if (vm.count("lns_time"))
        config.lns_time_limit = vm["lns_time"].as<int>();
    if (vm.count("lns_no_improvement_limit")) {
        config.lns_no_improvement_limit =
            vm["lns_no_improvement_limit"].as<int>();
        if (config.lns_no_improvement_limit < 0)
            throw invalid_argument(
                "lns_no_improvement_limit must be non-negative");
    }
    if (vm.count("ecbs_focal_weight")) {
        config.ecbs_focal_weight = vm["ecbs_focal_weight"].as<double>();
        if (config.ecbs_focal_weight < 1.0)
            throw invalid_argument(
                "ecbs_focal_weight must be at least 1.0");
    }
    if (vm.count("cbs_high_level_expansion_limit")) {
        config.cbs_high_level_expansion_limit =
            vm["cbs_high_level_expansion_limit"].as<int>();
        if (config.cbs_high_level_expansion_limit <= 0)
            throw invalid_argument(
                "cbs_high_level_expansion_limit must be positive");
    }
    if (vm.count("cbs_low_level_expansion_limit")) {
        config.cbs_low_level_expansion_limit =
            vm["cbs_low_level_expansion_limit"].as<int>();
        if (config.cbs_low_level_expansion_limit <= 0)
            throw invalid_argument(
                "cbs_low_level_expansion_limit must be positive");
    }
    if (vm.count("semi_online_lookahead_batches")) {
        config.semi_online_lookahead_batches =
            vm["semi_online_lookahead_batches"].as<int>();
        if (config.semi_online_lookahead_batches < 0)
            throw invalid_argument(
                "semi_online_lookahead_batches must be non-negative");
    }
}

string basename_no_ext(const string& path) {
    size_t slash = path.find_last_of("/\\");
    string name = (slash == string::npos) ? path : path.substr(slash + 1);
    size_t dot = name.find_last_of('.');
    return (dot == string::npos) ? name : name.substr(0, dot);
}

string mapf_name(MAPFMethod m) {
    switch (m) {
    case MAPF_CBS: return "CBS";
    case MAPF_PBS: return "PBS";
    case MAPF_wPBS: return "wPBS";
    case MAPF_PP_PER_TASK: return "PP_PER_TASK";
    case MAPF_PP_TASK_SEQUENCE: return "PP_TASK_SEQUENCE";
    default: return "PP";
    }
}

string sa_name(SingleAgentMethod s) {
    switch (s) {
    case SA_MLA_SEQUENCE: return "MLA";
    case SA_MLSIPP_SEQUENCE: return "MLSIPP";
    case SA_STA_TASK_EP: return "STA";
    default: return "";
    }
}

int main(int argc, char** argv)
{
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("map,m",          po::value<string>()->required(),            "input map file")
        ("task,t",         po::value<string>()->required(),            "input task file")
        ("algo,a",         po::value<string>()->default_value("TP"),   "algorithm")
        ("mode",           po::value<string>(),                        "task-information mode: ONLINE, OFFLINE, or SEMI_ONLINE")
        ("single_agent",   po::value<string>()->default_value("default"), "low-level: STA, MLA, or MLSIPP")
        ("mapf",           po::value<string>()->default_value("default"), "MAPF override: CBS, PBS, wPBS, PP (or PP_PER_TASK), PP_TASK_SEQUENCE")
        ("endpoint_strategy", po::value<string>(),                       "endpoint override: WAIT_OR_NEAREST_SAFE, RETURN_TO_HOME, NEAREST_WITH_STRICT_EXCLUSIONS, PAIRWISE_TASK_THEN_HOME, WAIT_OR_NEAREST_FREE_NONTASK, or NEAREST_AVAILABLE")
        ("seed",           po::value<int>()->default_value(0),         "general RNG seed (>=0 deterministic, <0 = time(NULL))")
        ("runtime_limit",  po::value<int>()->default_value(1800),      "whole-run wall-clock limit in seconds; 0 disables")
        ("pathfinding_runtime_limit", po::value<int>()->default_value(600), "per planning-cycle wall-clock limit in seconds; 0 disables")
        ("tour",           po::value<string>()->default_value(""),     "LKH3 tour file")
        ("save_output",    po::bool_switch()->default_value(false),    "save output to ./output/")
        ("output_dir",     po::value<string>()->default_value("./output"), "output directory")
        ("screen,s",       po::value<int>()->default_value(1),         "screen output (0=none, 1=results)")
        // Algorithm-specific tuning options. They do not affect algorithms
        // that do not use the named component.
        ("task_sequence_limit", po::value<int>(),                       "PBS/wPBS only: maximum tasks planned per agent (default 2)")
        ("wpbs_replan_window", po::value<int>(),                       "wPBS only: executed steps between replans (default 10)")
        ("lns_time",       po::value<int>()->default_value(1),         "LNS only: assignment-improvement budget (s)")
        ("lns_no_improvement_limit", po::value<int>(),                 "LNS only: rejected moves before early stop; 0 disables (default 2000)")
        ("ecbs_focal_weight", po::value<double>(),                     "CBS/ECBS only: focal bound; 1.0=optimal CBS, >1.0=ECBS (default 1.0)")
        ("cbs_high_level_expansion_limit", po::value<int>(),           "CBS/ECBS only: maximum high-level expansions per batch (default INT_MAX)")
        ("cbs_low_level_expansion_limit", po::value<int>(),            "CBS/ECBS only: maximum expansions per low-level search (default INT_MAX)")
        ("semi_online_lookahead_batches", po::value<int>(),            "SEMI_ONLINE only: future task-release batches known in advance (default 1)")
        ("lns_imp",        po::value<int>()->default_value(0),         "LNS only: optional post-run LNS improvement rounds (0=off)")
        ("lns_imp_group",  po::value<int>()->default_value(5),         "LNS only: post-run LNS destroy-group size")
    ;

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            cout << "Unified MAPD Framework" << endl;
            cout << desc << endl;
            cout << "Examples:" << endl;
            cout << "  ./mapd -m kiva.map -t kiva.task -a TP" << endl;
            cout << "  ./mapd -m kiva.map -t kiva.task -a CENTRAL-CBS" << endl;
            cout << "  ./mapd -m kiva.map -t kiva.task -a HBH_MLA --save_output" << endl;
            cout << "  ./mapd -m kiva.map -t kiva.task -a HUNGARIAN_PBS --mapf PP --single_agent MLSIPP" << endl;
            return 0;
        }

        po::notify(vm);
    } catch (const po::error& e) {
        cerr << "Error: " << e.what() << endl;
        cerr << desc << endl;
        return 1;
    }

    string map_file  = vm["map"].as<string>();
    string task_file = vm["task"].as<string>();
    string tour_file = vm["tour"].as<string>();
    string algorithm_name = vm["algo"].as<string>();
    int screen       = vm["screen"].as<int>();
    int lns_imp_rounds = vm["lns_imp"].as<int>();
    int lns_imp_group  = vm["lns_imp_group"].as<int>();
    bool save_output = vm["save_output"].as<bool>();
    string output_dir = vm["output_dir"].as<string>();

    MAPDConfig config;
    try {
        set_parameters(config, vm);
    } catch (const invalid_argument& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }

    if (screen >= 1) {
        cout << "Map: " << map_file << endl;
        cout << "Task: " << task_file << endl;
        cout << "Algorithm: " << algorithm_name << endl;
        string sa_override = vm["single_agent"].as<string>();
        string mf_override = vm["mapf"].as<string>();
        if (sa_override != "default") cout << "Single-agent: " << sa_override << endl;
        if (mf_override != "default") cout << "MAPF: " << mf_override << endl;
        if (vm.count("endpoint_strategy"))
            cout << "Endpoint strategy: "
                 << vm["endpoint_strategy"].as<string>() << endl;
        if (lns_imp_rounds > 0)
            cout << "REALPATH_LNS_IMP: " << lns_imp_rounds << " rounds, group_size=" << lns_imp_group << endl;
        cout << endl;
    }

    clock_t t_start = clock();
    RuntimeDeadline::start_run(config.runtime_limit_seconds);

    Simulation sim;
    try {
        sim.init(map_file, task_file, config, tour_file);
        sim.run();
    } catch (const runtime_error& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }

    double elapsed_base = (double)(clock() - t_start) / CLOCKS_PER_SEC * 1000.0;

    if (lns_imp_rounds > 0) {
        if (screen >= 1) {
            cout << "--- Before REALPATH_LNS_IMP ---" << endl;
            sim.showTask();
            cout << endl;
        }

        clock_t t_lns = clock();
        int imp = sim.realpath_lns_imp(lns_imp_rounds, lns_imp_group);
        double elapsed_lns = (double)(clock() - t_lns) / CLOCKS_PER_SEC * 1000.0;

        if (screen >= 1) {
            cout << "--- After REALPATH_LNS_IMP (" << imp << "/" << lns_imp_rounds << " improving) ---" << endl;
        }
    }

    double elapsed = (double)(clock() - t_start) / CLOCKS_PER_SEC * 1000.0;

    sim.fullCollisionCheck(algorithm_name);

    if (screen >= 1) {
        cout << endl;
        sim.showTask();
        cout << "Total runtime:\t" << elapsed << " ms" << endl;
    }

    // Save output if requested
    if (save_output) {
        mkdir(output_dir.c_str(), 0755);

        // Build descriptive filename
        string map_name = basename_no_ext(map_file);
        string task_name = basename_no_ext(task_file);
        string algo = algorithm_name;
        string sa_str = vm["single_agent"].as<string>();
        string mf_str = vm["mapf"].as<string>();

        string filename = algo;
        if (sa_str != "default") filename += "_sa-" + sa_str;
        if (mf_str != "default") filename += "_mapf-" + mf_str;
        if (lns_imp_rounds > 0)
            filename += "_lnsimp-" + to_string(lns_imp_rounds) + "g" + to_string(lns_imp_group);
        filename += "_" + map_name + "_" + task_name + ".txt";

        string filepath = output_dir + "/" + filename;
        sim.saveOutput(filepath, elapsed, algorithm_name);

        if (screen >= 1)
            cout << "Output saved to: " << filepath << endl;
    }

    return 0;
}
