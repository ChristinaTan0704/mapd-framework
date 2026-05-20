#pragma once
#include <string>
using namespace std;

// ============ Enums matching pseudocode Section 0 ============

enum Mode { MODE_ONLINE, MODE_OFFLINE, MODE_SEMI_ONLINE };
enum AssignType { ASSIGN_IA, ASSIGN_TA };
enum AssignMethod {
    AM_DECOUPLED_GREEDY,
    AM_CENTRALIZED_GREEDY,
    AM_HUNGARIAN,
    AM_DECOUPLED_GREEDY_SWAPS,
    AM_LKH3_TSP,
    AM_LKH3_TSP_REASSIGN,
    AM_GREEDY_INSERT_LNS,
    AM_REPEATED_HUNGARIAN_LNS
};
enum AssignTrigger {
    AT_ON_FREE_WAITS,
    AT_EVERY_TIMESTEP,
    AT_ON_NEW_TASK_OR_FREE,
    AT_ON_UNASSIGNED_OR_FREE,
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
    SA_SEQ_STA
};
enum DeadlockAvoid {
    DA_HOLDING_ENDPOINT,
    DA_DUMMY_PATH,
    DA_NO_AVOIDANCE
};
enum MLAMode {
    MLA_DEFAULT,
    MLA_SEQ,
    MLA_TASKWISE
};

// ============ Config struct ============

struct MAPDConfig {
    string name;
    Mode mode;
    AssignType assign_type;
    AssignMethod assign_method;
    AssignTrigger assign_trigger;
    MAPFMethod mapf;
    SingleAgentMethod single_agent;
    DeadlockAvoid deadlock;
    int replan_window;   // for wPBS
    double ecbs_weight;  // for ECBS/CBS (1.0 = optimal)
    int lns_time_limit;  // LNS improvement time limit in seconds (0 = no LNS)
    MLAMode mla_mode;
    bool use_sipp;  // use SIPP instead of MLA* for PBS low-level search

    MAPDConfig() : name("Custom"), mode(MODE_ONLINE), assign_type(ASSIGN_IA),
        assign_method(AM_DECOUPLED_GREEDY), assign_trigger(AT_ON_FREE_WAITS),
        mapf(MAPF_DECOUPLED_PP), single_agent(SA_STA_TASK_EP),
        deadlock(DA_HOLDING_ENDPOINT), replan_window(10), ecbs_weight(1.0),
        lns_time_limit(1), mla_mode(MLA_TASKWISE), use_sipp(false) {}
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
        c.deadlock = DA_HOLDING_ENDPOINT;
    }
    else if (name == "TPTS") {
        c.mode = MODE_ONLINE; c.assign_type = ASSIGN_IA;
        c.assign_method = AM_DECOUPLED_GREEDY_SWAPS;
        c.assign_trigger = AT_ON_FREE_WAITS;
        c.mapf = MAPF_DECOUPLED_PP;
        c.single_agent = SA_STA_TASK_EP;
        c.deadlock = DA_HOLDING_ENDPOINT;
    }
    else if (name == "CENTRAL") {
        c.mode = MODE_ONLINE; c.assign_type = ASSIGN_IA;
        c.assign_method = AM_HUNGARIAN;
        c.assign_trigger = AT_EVERY_TIMESTEP;
        c.mapf = MAPF_CBS;
        c.single_agent = SA_STA_TASK_EP;
        c.deadlock = DA_HOLDING_ENDPOINT;
    }
    else if (name == "CENTRAL_FIXED") {
        c.mode = MODE_ONLINE; c.assign_type = ASSIGN_IA;
        c.assign_method = AM_HUNGARIAN;
        c.assign_trigger = AT_ON_NEW_TASK_OR_FREE;
        c.mapf = MAPF_CBS;
        c.single_agent = SA_STA_TASK_EP;
        c.deadlock = DA_HOLDING_ENDPOINT;
    }
    else if (name == "HBH_MLA") {
        c.mode = MODE_ONLINE; c.assign_type = ASSIGN_IA;
        c.assign_method = AM_CENTRALIZED_GREEDY;
        c.assign_trigger = AT_ON_FREE_WAITS;
        c.mapf = MAPF_DECOUPLED_PP;
        c.single_agent = SA_MLA_SEQUENCE;
        c.deadlock = DA_HOLDING_ENDPOINT;
    }
    else if (name == "TA_PRIORITIZED") {
        c.mode = MODE_OFFLINE; c.assign_type = ASSIGN_TA;
        c.assign_method = AM_LKH3_TSP;
        c.assign_trigger = AT_ONCE;
        c.mapf = MAPF_DECOUPLED_PP;
        c.single_agent = SA_SEQ_STA;
        c.deadlock = DA_DUMMY_PATH;
    }
    else if (name == "TA_HYBRID") {
        c.mode = MODE_OFFLINE; c.assign_type = ASSIGN_TA;
        c.assign_method = AM_LKH3_TSP_REASSIGN;
        c.assign_trigger = AT_ONCE;
        c.mapf = MAPF_TA_HYBRID_TWO_GROUP;
        c.single_agent = SA_STA_TASK_EP;
        c.deadlock = DA_DUMMY_PATH;
    }
    else if (name == "HUNGARIAN_PBS") {
        c.mode = MODE_ONLINE; c.assign_type = ASSIGN_TA;
        c.assign_method = AM_HUNGARIAN;
        c.assign_trigger = AT_ON_UNASSIGNED_OR_FREE;
        c.mapf = MAPF_PBS;
        c.single_agent = SA_MLA_SEQUENCE;
        c.deadlock = DA_DUMMY_PATH;
    }
    else if (name == "HUNGARIAN_wPBS") {
        c.mode = MODE_ONLINE; c.assign_type = ASSIGN_TA;
        c.assign_method = AM_HUNGARIAN;
        c.assign_trigger = AT_ON_UNASSIGNED_OR_FREE;
        c.mapf = MAPF_wPBS;
        c.single_agent = SA_MLA_SEQUENCE;
        c.deadlock = DA_DUMMY_PATH;
        c.replan_window = 10;
    }
    else if (name == "LNS_PBS") {
        c.mode = MODE_ONLINE; c.assign_type = ASSIGN_TA;
        c.assign_method = AM_REPEATED_HUNGARIAN_LNS;
        c.assign_trigger = AT_ON_UNASSIGNED_OR_FREE;
        c.mapf = MAPF_PBS;
        c.single_agent = SA_MLA_SEQUENCE;
        c.deadlock = DA_DUMMY_PATH;
        c.lns_time_limit = 1;
    }
    else if (name == "LNS_wPBS") {
        c.mode = MODE_ONLINE; c.assign_type = ASSIGN_TA;
        c.assign_method = AM_REPEATED_HUNGARIAN_LNS;
        c.assign_trigger = AT_ON_UNASSIGNED_OR_FREE;
        c.mapf = MAPF_wPBS;
        c.single_agent = SA_MLA_SEQUENCE;
        c.deadlock = DA_DUMMY_PATH;
        c.replan_window = 10;
        c.lns_time_limit = 1;
    }

    return c;
}
