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

    if (vm.count("ecbs_weight"))
        config.ecbs_weight = vm["ecbs_weight"].as<double>();
    if (vm.count("replan_window"))
        config.replan_window = vm["replan_window"].as<int>();
    if (vm.count("lns_time"))
        config.lns_time_limit = vm["lns_time"].as<int>();
    string sa = vm["single_agent"].as<string>();
    if (sa == "MLA") config.single_agent = SA_MLA_SEQUENCE;
    else if (sa == "MLSIPP" || sa == "SIPP") config.single_agent = SA_MLSIPP_SEQUENCE;
    else if (sa == "STA") config.single_agent = SA_STA_TASK_EP;
    string mf = vm["mapf"].as<string>();
    if (mf == "CBS") config.mapf = MAPF_CBS;
    else if (mf == "PBS") config.mapf = MAPF_PBS;
    else if (mf == "wPBS") config.mapf = MAPF_wPBS;
    else if (mf == "PP") config.mapf = MAPF_DECOUPLED_PP;
    string mm = vm["mla_mode"].as<string>();
    if (mm == "seq") config.mla_mode = MLA_SEQ;
    else if (mm == "task") config.mla_mode = MLA_TASKWISE;
    else if (mm == "sta") config.mla_mode = MLA_SEQ_STA;
    config.use_sipp = vm["sipp"].as<bool>();
    if (config.use_sipp) config.single_agent = SA_MLSIPP_SEQUENCE;
    config.lns_seed = vm["seed"].as<int>();
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
    case MAPF_DECOUPLED_PP: return "PP";
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
        ("ecbs_weight,w",  po::value<double>()->default_value(1.0),    "ECBS suboptimality weight")
        ("replan_window",  po::value<int>()->default_value(10),        "replanning window for wPBS")
        ("lns_time",       po::value<int>()->default_value(1),         "LNS improvement time limit (s)")
        ("lns_imp",        po::value<int>()->default_value(0),         "REALPATH_LNS_IMP rounds (0=off)")
        ("lns_imp_group",  po::value<int>()->default_value(5),         "REALPATH_LNS_IMP destroy group")
        ("single_agent",   po::value<string>()->default_value("default"), "low-level: STA, MLA, or MLSIPP")
        ("mapf",           po::value<string>()->default_value("default"), "MAPF override: CBS, PBS, wPBS, PP")
        ("mla_mode",       po::value<string>()->default_value("default"), "MLA mode: seq (SeqMLA*), task (task-by-task), default")
        ("sipp",           po::bool_switch()->default_value(false),    "use SIPP instead of MLA* for PBS low-level")
        ("seed",           po::value<int>()->default_value(0),         "LNS RNG seed (>=0 deterministic, <0 = time(NULL))")
        ("tour",           po::value<string>()->default_value(""),     "LKH3 tour file")
        ("save_output",    po::bool_switch()->default_value(false),    "save output to ./output/")
        ("output_dir",     po::value<string>()->default_value("./output"), "output directory")
        ("screen,s",       po::value<int>()->default_value(1),         "screen output (0=none, 1=results)")
    ;

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            cout << "Unified MAPD Framework" << endl;
            cout << desc << endl;
            cout << "Examples:" << endl;
            cout << "  ./mapd -m kiva.map -t kiva.task -a TP" << endl;
            cout << "  ./mapd -m kiva.map -t kiva.task -a HBH_MLA --save_output" << endl;
            cout << "  ./mapd -m kiva.map -t kiva.task -a CENTRAL --mapf PBS --save_output" << endl;
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
        cout << "Algorithm: " << config.name << endl;
        string sa_override = vm["single_agent"].as<string>();
        string mf_override = vm["mapf"].as<string>();
        if (sa_override != "default") cout << "Single-agent: " << sa_override << endl;
        if (mf_override != "default") cout << "MAPF: " << mf_override << endl;
        if (config.mapf == MAPF_CBS)
            cout << "ECBS weight: " << config.ecbs_weight << endl;
        if (lns_imp_rounds > 0)
            cout << "REALPATH_LNS_IMP: " << lns_imp_rounds << " rounds, group_size=" << lns_imp_group << endl;
        string mm_str = vm["mla_mode"].as<string>();
        if (mm_str != "default") cout << "MLA mode: " << mm_str << endl;
        cout << endl;
    }

    clock_t t_start = clock();

    Simulation sim;
    sim.init(map_file, task_file, config, tour_file);
    sim.run();

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

    sim.fullCollisionCheck(config.name);

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
        string algo = config.name;
        string sa_str = vm["single_agent"].as<string>();
        string mf_str = vm["mapf"].as<string>();

        string filename = algo;
        if (sa_str != "default") filename += "_sa-" + sa_str;
        if (mf_str != "default") filename += "_mapf-" + mf_str;
        if (lns_imp_rounds > 0)
            filename += "_lnsimp-" + to_string(lns_imp_rounds) + "g" + to_string(lns_imp_group);
        filename += "_" + map_name + "_" + task_name + ".txt";

        string filepath = output_dir + "/" + filename;
        sim.saveOutput(filepath, elapsed);

        if (screen >= 1)
            cout << "Output saved to: " << filepath << endl;
    }

    return 0;
}
