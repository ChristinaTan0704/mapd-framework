#include <string>
#include <iostream>
#include <ctime>
#include <fstream>
#include <sys/stat.h>
#include <boost/program_options.hpp>
#include "simulation.h"
#include "config_names.h"

namespace po = boost::program_options;
using namespace std;

void set_parameters(MAPDConfig& config, const po::variables_map& vm)
{
    // A preset only supplies default components; without one every component
    // comes from the options (or the MAPDConfig defaults).
    config = vm.count("algo") ? get_preset(vm["algo"].as<string>()) : MAPDConfig();

    // Component options; an option that is not given keeps the preset value.
    auto override_enum = [&](const char* option, auto& field) {
        if (vm.count(option))
            field = parse_enum<decay_t<decltype(field)>>(option, vm[option].as<string>());
    };
    override_enum("assign_method", config.assign_method);
    override_enum("assign_trigger", config.assign_trigger);
    if (vm.count("replan_on"))
        config.replan_on = parse_flags<ReplanEvent>("replan_on", vm["replan_on"].as<string>());
    override_enum("agent_selection", config.agent_selection);
    override_enum("mapf", config.mapf);
    override_enum("single_agent", config.single_agent);
    override_enum("endpoint_strategy", config.endpoint_strategy);
    override_enum("mode", config.mode);
    override_enum("task_type", config.task_type);
    override_enum("tie_breaking", config.tie_breaking);
    override_enum("cbs_conflict_selection", config.cbs_conflict_selection);
    if (vm.count("dummy_path"))
        config.dummy_path = vm["dummy_path"].as<int>() != 0;

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
    if (vm.count("cbs_bypass"))
        config.cbs_bypass = vm["cbs_bypass"].as<int>() != 0;
    if (vm.count("cbs_target_reasoning"))
        config.cbs_target_reasoning =
            vm["cbs_target_reasoning"].as<int>() != 0;
    if (vm.count("cbs_rectangle_reasoning"))
        config.cbs_rectangle_reasoning =
            vm["cbs_rectangle_reasoning"].as<int>() != 0;
    if (vm.count("semi_online_lookahead_batches")) {
        config.semi_online_lookahead_batches =
            vm["semi_online_lookahead_batches"].as<int>();
        if (config.semi_online_lookahead_batches < 1)
            throw invalid_argument(
                "semi_online_lookahead_batches must be at least 1 (0 would be ONLINE)");
    }
}

string basename_no_ext(const string& path) {
    size_t slash = path.find_last_of("/\\");
    string name = (slash == string::npos) ? path : path.substr(slash + 1);
    size_t dot = name.find_last_of('.');
    return (dot == string::npos) ? name : name.substr(0, dot);
}

int main(int argc, char** argv)
{
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("map,m",          po::value<string>()->required(),            "input map file")
        ("task,t",         po::value<string>()->required(),            "input task file")
        ("algo,a",         po::value<string>(),                        "algorithm preset supplying default components (optional)")
        ("config,c",       po::value<string>(),                        "file of option = value lines (e.g. configs/table1/TP.cfg); command-line options take precedence")
        // The components; each overrides the preset chosen with --algo.
        ("assign_method",  po::value<string>(), ("task assignment: " + enum_choices<AssignMethod>()).c_str())
        ("assign_trigger", po::value<string>(), ("when to assign: " + enum_choices<AssignTrigger>()).c_str())
        ("replan_on",      po::value<string>(), ("when to replan, comma-separated: " + enum_choices<ReplanEvent>() + "; WHEN_ASSIGNED = plan each agent when it is assigned, inside the assignment").c_str())
        ("agent_selection", po::value<string>(), ("agents a replan plans: " + enum_choices<AgentSelection>()).c_str())
        ("mapf",           po::value<string>(), ("MAPF method: " + enum_choices<MAPFMethod>()).c_str())
        ("single_agent",   po::value<string>(), ("single-agent search: " + enum_choices<SingleAgentMethod>()).c_str())
        ("endpoint_strategy", po::value<string>(), ("endpoint strategy: " + enum_choices<EndpointStrategy>()).c_str())
        ("dummy_path",     po::value<int>(),    "1 = append a path to the chosen endpoint after the last task, 0 = stop at the last goal")
        ("mode",           po::value<string>(), ("task-information mode: " + enum_choices<Mode>()).c_str())
        ("task_type",      po::value<string>(), ("task type, checked against the task file: " + enum_choices<TaskType>()).c_str())
        ("seed",           po::value<int>()->default_value(0),         "general RNG seed (>=0 deterministic, <0 = time(NULL))")
        ("tie_breaking",   po::value<string>(),                        "search-node ties: reference (original implementations) or random (--seed) (default reference)")
        ("runtime_limit",  po::value<int>()->default_value(1000),      "whole-run wall-clock limit in seconds; 0 disables")
        ("pathfinding_runtime_limit", po::value<int>()->default_value(600), "per planning-cycle wall-clock limit in seconds; 0 disables")
        ("tour",           po::value<string>()->default_value(""),     "LKH3 tour file")
        ("save_output",    po::bool_switch()->default_value(false),    "save output to ./output/")
        ("output_dir",     po::value<string>()->default_value("./output"), "output directory")
        ("screen,s",       po::value<int>()->default_value(1),         "screen output (0=none, 1=results)")
        // Algorithm-specific tuning options. They do not affect algorithms
        // that do not use the named component.
        ("task_sequence_limit", po::value<int>(),                       "tasks planned per agent by PBS, wPBS and CBS, and kept per agent by repeated Hungarian/LNS (default 2)")
        ("wpbs_replan_window", po::value<int>(),                       "steps between PERIODIC replans; also the wPBS planning window (default 10)")
        ("lns_time",       po::value<int>(),                           "LNS only: assignment-improvement budget in seconds (default 1)")
        ("lns_no_improvement_limit", po::value<int>(),                 "LNS only: rejected moves before early stop; 0 disables (default 2000)")
        ("ecbs_focal_weight", po::value<double>(),                     "CBS/ECBS only: focal bound; 1.0=optimal CBS, >1.0=ECBS (default 1.0)")
        ("cbs_high_level_expansion_limit", po::value<int>(),           "CBS/ECBS only: maximum high-level expansions per batch (default 10000); a search that reaches it fails, and the planner falls back")
        ("cbs_conflict_selection", po::value<string>(),                "CBS/ECBS only: earliest or cardinal (ICBS MDD prioritization) (default cardinal)")
        ("cbs_bypass",     po::value<int>(),                           "CBS/ECBS only: 1=ICBS bypass, 0=always branch (default 1)")
        ("cbs_target_reasoning", po::value<int>(),                     "CBS/ECBS only: 1=target-conflict reasoning, 0=vertex constraints (default 1)")
        ("cbs_rectangle_reasoning", po::value<int>(),                  "CBS/ECBS only: 1=rectangle (barrier) reasoning, 0=vertex constraints (default 1)")
        ("semi_online_lookahead_batches", po::value<int>(),            "SEMI_ONLINE only: future task-release batches known in advance (default 1)")
    ;

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if (vm.count("config")) {
            // Stored after the command line, so command-line options win.
            const string config_file = vm["config"].as<string>();
            ifstream input(config_file);
            if (!input) throw po::error("cannot open config file " + config_file);
            po::store(po::parse_config_file(input, desc), vm);
        }

        if (vm.count("help")) {
            cout << "Unified MAPD Framework" << endl;
            cout << desc << endl;
            cout << "Examples:" << endl;
            cout << "  ./mapd -m kiva.map -t kiva.task -a TP" << endl;
            cout << "  ./mapd -m kiva.map -t kiva.task -a CENTRAL-CBS" << endl;
            cout << "  ./mapd -m kiva.map -t kiva.task -a HBH_MLA --save_output" << endl;
            cout << "  ./mapd -m kiva.map -t kiva.task -a HUNGARIAN_PBS --mapf PP --single_agent MLSIPP" << endl;
            cout << "  ./mapd -m kiva.map -t kiva.task --config configs/table1/TP.cfg" << endl;
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
    string algorithm_name = vm.count("algo") ? vm["algo"].as<string>()
                          : vm.count("config") ? basename_no_ext(vm["config"].as<string>())
                          : "custom";
    int screen       = vm["screen"].as<int>();
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
        if (vm.count("single_agent"))
            cout << "Single-agent: " << vm["single_agent"].as<string>() << endl;
        if (vm.count("mapf")) cout << "MAPF: " << vm["mapf"].as<string>() << endl;
        if (vm.count("endpoint_strategy"))
            cout << "Endpoint strategy: "
                 << vm["endpoint_strategy"].as<string>() << endl;
        cout << endl;
    }

    clock_t t_start = clock();
    RuntimeDeadline::start_run(config.runtime_limit_seconds);

    Simulation sim;
    try {
        sim.init(map_file, task_file, config, tour_file);
        sim.run();
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }

    double elapsed = (double)(clock() - t_start) / CLOCKS_PER_SEC * 1000.0;

    // Both checks always print; a failed one makes the exit status 2.
    const bool checks_passed = sim.fullCollisionCheck(algorithm_name) &
                               sim.taskOrderCheck(algorithm_name);

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
        string filename = algo;
        if (vm.count("single_agent")) filename += "_sa-" + vm["single_agent"].as<string>();
        if (vm.count("mapf")) filename += "_mapf-" + vm["mapf"].as<string>();
        filename += "_" + map_name + "_" + task_name + ".txt";

        string filepath = output_dir + "/" + filename;
        sim.saveOutput(filepath, elapsed, algorithm_name);

        if (screen >= 1)
            cout << "Output saved to: " << filepath << endl;
    }

    return checks_passed ? 0 : 2;
}
