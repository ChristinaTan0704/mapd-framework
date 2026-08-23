#pragma once
#include <string>
#include <stdexcept>
using namespace std;

// ============ Enums matching pseudocode Section 0 ============

enum Mode { MODE_ONLINE, MODE_OFFLINE, MODE_SEMI_ONLINE };
enum AssignType { ASSIGN_IA, ASSIGN_TA };
enum AssignMethod {
    AM_DECOUPLED_GREEDY,
    AM_CENTRALIZED_GREEDY,
    AM_CENTRAL_HUNGARIAN,
    AM_REPEATED_HUNGARIAN,
    AM_DECOUPLED_GREEDY_SWAPS,
    AM_LKH3_TSP,
    AM_LKH3_TSP_REASSIGN,
    AM_GREEDY_INSERT_LNS,
    AM_REPEATED_HUNGARIAN_LNS
};
enum AssignTrigger {
    AT_ON_FREE_WAITS,
    AT_EVERY_TIMESTEP,
    AT_ON_UNASSIGNED_TASK_OR_NEW_AVAILABLE_AGENT,
    // Legacy name retained so the archived all-methods source still compiles.
    AT_ON_UNASSIGNED_OR_FREE = AT_ON_UNASSIGNED_TASK_OR_NEW_AVAILABLE_AGENT,
    AT_ONCE
};
enum MAPFMethod {
    MAPF_DECOUPLED_PP,
    MAPF_CBS,
    MAPF_PBS,
    MAPF_wPBS,
    MAPF_TA_HYBRID_TWO_GROUP
};
enum SingleAgentMethod {
    SA_STA_TASK_EP,
    SA_STA_NONTASK_EP,
    SA_MLA_SEQUENCE,
    SA_MLSIPP_SEQUENCE,
    SA_SEQ_STA
};
enum MLAMode {
    MLA_DEFAULT,
    MLA_SEQ,
    MLA_TASKWISE,
    MLA_SEQ_STA   // Sequential single-goal A* (matches reference StateTimeA*)
};
enum CoupledMode { CM_NONE, CM_SWAPS_ONLY, CM_REASSIGN_ONLY, CM_FULLY_COUPLED };
enum EndpointStrategy { EP_TASK_ENDPOINT, EP_FIXED_PARKING, EP_FLEXIBLE_STRICT, EP_FLEXIBLE_PAIRWISE };

// ============ Config struct ============

struct MAPDConfig {
    string name;
    Mode mode;
    AssignType assign_type;
    AssignMethod assign_method;
    AssignTrigger assign_trigger;
    MAPFMethod mapf;
    SingleAgentMethod single_agent;
    // Whether to append a post-delivery path to the selected endpoint. The
    // endpoint may equal the delivery location, yielding a zero-length dummy.
    bool dummy_path;
    int replan_window;   // for wPBS
    double ecbs_weight;  // for ECBS/CBS (1.0 = optimal)
    int lns_time_limit;  // LNS improvement time limit in seconds (0 = no LNS)
    MLAMode mla_mode;
    bool use_sipp;  // legacy CLI compatibility; maps to SA_MLSIPP_SEQUENCE
    int lns_seed;   // RNG seed for the LNS assignment loop: >=0 deterministic, <0 = time(NULL)
    CoupledMode coupled;               // task-assignment/pathfinding coupling axis
    EndpointStrategy endpoint_strategy; // endpoint/parking selection axis
    bool windowed;                     // windowed replanning: the clock advances at
                                       // most replan_window steps between replans
                                       // (a planning-horizon property, true for wPBS)

    MAPDConfig() : name("Custom"), mode(MODE_ONLINE), assign_type(ASSIGN_IA),
        assign_method(AM_DECOUPLED_GREEDY), assign_trigger(AT_ON_FREE_WAITS),
        mapf(MAPF_DECOUPLED_PP), single_agent(SA_STA_TASK_EP),
        dummy_path(true), replan_window(10), ecbs_weight(1.0),
        lns_time_limit(1), mla_mode(MLA_TASKWISE), use_sipp(false), lns_seed(0),
        coupled(CM_NONE), endpoint_strategy(EP_TASK_ENDPOINT), windowed(false) {}
};

// ============ Algorithm Presets ============

inline MAPDConfig get_preset(const string& name) {
    MAPDConfig c;
    c.name = name;

    if (name == "TP") {
        c.mode = MODE_ONLINE; c.assign_type = ASSIGN_IA;
        c.assign_method = AM_DECOUPLED_GREEDY;
        c.assign_trigger = AT_ON_FREE_WAITS;
        c.mapf = MAPF_DECOUPLED_PP;
        c.single_agent = SA_STA_TASK_EP;
        c.dummy_path = true;
        c.coupled = CM_NONE; c.endpoint_strategy = EP_TASK_ENDPOINT;
    }
    else if (name == "TPTS") {
        c.mode = MODE_ONLINE; c.assign_type = ASSIGN_IA;
        c.assign_method = AM_DECOUPLED_GREEDY_SWAPS;
        c.assign_trigger = AT_ON_FREE_WAITS;
        c.mapf = MAPF_DECOUPLED_PP;
        c.single_agent = SA_STA_TASK_EP;
        c.dummy_path = true;
        c.coupled = CM_SWAPS_ONLY; c.endpoint_strategy = EP_TASK_ENDPOINT;
    }
    else if (name == "CENTRAL") {
        c.mode = MODE_ONLINE; c.assign_type = ASSIGN_IA;
        c.assign_method = AM_CENTRAL_HUNGARIAN;
        c.assign_trigger = AT_EVERY_TIMESTEP;
        c.mapf = MAPF_CBS;
        c.single_agent = SA_STA_TASK_EP;
        c.dummy_path = true;
        c.coupled = CM_NONE; c.endpoint_strategy = EP_TASK_ENDPOINT;
    }
    else if (name == "HBH_MLA") {
        c.mode = MODE_ONLINE; c.assign_type = ASSIGN_IA;
        c.assign_method = AM_CENTRALIZED_GREEDY;
        c.assign_trigger = AT_ON_FREE_WAITS;
        c.mapf = MAPF_DECOUPLED_PP;
        c.single_agent = SA_MLA_SEQUENCE;
        c.dummy_path = true;
        c.coupled = CM_NONE; c.endpoint_strategy = EP_TASK_ENDPOINT;
    }
    else if (name == "TA_PRIORITIZED") {
        c.mode = MODE_OFFLINE; c.assign_type = ASSIGN_TA;
        c.assign_method = AM_LKH3_TSP;
        c.assign_trigger = AT_ONCE;
        c.mapf = MAPF_DECOUPLED_PP;
        c.single_agent = SA_SEQ_STA;
        c.dummy_path = true;
        c.coupled = CM_NONE; c.endpoint_strategy = EP_FIXED_PARKING;
    }
    else if (name == "TA_HYBRID") {
        c.mode = MODE_OFFLINE; c.assign_type = ASSIGN_TA;
        c.assign_method = AM_LKH3_TSP_REASSIGN;
        c.assign_trigger = AT_ONCE;
        c.mapf = MAPF_TA_HYBRID_TWO_GROUP;
        c.single_agent = SA_STA_TASK_EP;
        c.dummy_path = true;
        c.coupled = CM_REASSIGN_ONLY; c.endpoint_strategy = EP_FIXED_PARKING;
    }
    else if (name == "HUNGARIAN_PBS") {
        c.mode = MODE_ONLINE; c.assign_type = ASSIGN_TA;
        c.assign_method = AM_REPEATED_HUNGARIAN;
        c.assign_trigger = AT_ON_UNASSIGNED_TASK_OR_NEW_AVAILABLE_AGENT;
        c.mapf = MAPF_PBS;
        c.single_agent = SA_MLA_SEQUENCE;
        c.dummy_path = true;
        c.coupled = CM_NONE; c.endpoint_strategy = EP_FLEXIBLE_STRICT;
    }
    else if (name == "HUNGARIAN_wPBS") {
        c.mode = MODE_ONLINE; c.assign_type = ASSIGN_TA;
        c.assign_method = AM_REPEATED_HUNGARIAN;
        c.assign_trigger = AT_ON_UNASSIGNED_TASK_OR_NEW_AVAILABLE_AGENT;
        c.mapf = MAPF_wPBS;
        c.single_agent = SA_MLA_SEQUENCE;
        c.dummy_path = true;
        c.replan_window = 15; // Match reference: --planning_window=15 --simulation_window=15
        c.windowed = true;
        c.coupled = CM_NONE; c.endpoint_strategy = EP_FLEXIBLE_PAIRWISE;
    }
    else if (name == "LNS_PBS") {
        c.mode = MODE_ONLINE; c.assign_type = ASSIGN_TA;
        c.assign_method = AM_REPEATED_HUNGARIAN_LNS;
        c.assign_trigger = AT_ON_UNASSIGNED_TASK_OR_NEW_AVAILABLE_AGENT;
        c.mapf = MAPF_PBS;
        c.single_agent = SA_MLA_SEQUENCE;
        c.dummy_path = true;
        c.lns_time_limit = 1;
        c.coupled = CM_NONE; c.endpoint_strategy = EP_FLEXIBLE_STRICT;
    }
    else if (name == "LNS_wPBS") {
        c.mode = MODE_ONLINE; c.assign_type = ASSIGN_TA;
        c.assign_method = AM_REPEATED_HUNGARIAN_LNS;
        c.assign_trigger = AT_ON_UNASSIGNED_TASK_OR_NEW_AVAILABLE_AGENT;
        c.mapf = MAPF_wPBS;
        c.single_agent = SA_MLA_SEQUENCE;
        c.dummy_path = true;
        c.replan_window = 15; // Match reference: --simulation_window=15 --planning_window=15
        c.windowed = true;
        c.lns_time_limit = 1;
        c.coupled = CM_NONE; c.endpoint_strategy = EP_FLEXIBLE_PAIRWISE;
    }
    else {
        throw invalid_argument("unsupported algorithm preset: " + name);
    }

    return c;
}
