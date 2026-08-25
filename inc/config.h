#pragma once
#include <climits>
#include <string>
#include <stdexcept>
using namespace std;

// ============ Enums matching pseudocode Section 0 ============

enum Mode { MODE_ONLINE, MODE_OFFLINE, MODE_SEMI_ONLINE };
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
    AT_ON_NEW_TASK_OR_AGENT_BECOMES_FREE,
    AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE,
    AT_ONCE
};
enum MAPFMethod {
    MAPF_PP_PER_TASK,
    MAPF_PP_TASK_SEQUENCE,
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
enum EndpointStrategy {
    // Return the agent's own initial/home parking endpoint.
    RETURN_TO_HOME,
    // Choose the nearest task or parking endpoint after excluding new dummies,
    // unfinished-task goals, and the old parking endpoints of other agents.
    NEAREST_WITH_STRICT_EXCLUSIONS,
    // Choose a pairwise-distinct task endpoint first, then any parking/home
    // endpoint; skip the current goal and stay there only as the final fallback.
    PAIRWISE_TASK_THEN_HOME,
    // TP/TPTS Path2: wait if safe; otherwise choose the nearest task or parking
    // endpoint that avoids future paths and open-task delivery locations.
    WAIT_OR_NEAREST_SAFE,
    // HBH: wait unless the current cell blocks an open task goal; otherwise
    // move to the nearest reachable parking (non-task) endpoint that can be held.
    WAIT_OR_NEAREST_FREE_NONTASK,
    // CENTRAL: choose the nearest task or parking endpoint not already reserved
    // by a carrying-task goal, selected task, or earlier parking choice.
    NEAREST_AVAILABLE
};

// ============ Config struct ============

struct MAPDConfig {
    Mode mode;
    AssignMethod assign_method;
    AssignTrigger assign_trigger;
    MAPFMethod mapf;
    SingleAgentMethod single_agent;
    // Whether to append a post-delivery path to the selected endpoint. The
    // endpoint may equal the delivery location, yielding a zero-length dummy.
    bool dummy_path;
    // General RNG seed for randomized framework components: LNS and search-node
    // f/g tie-breaking. >=0 is deterministic and <0 is time-based.
    int seed;
    // Whole-process wall-clock limit in seconds; 0 disables the internal limit.
    int runtime_limit_seconds;
    EndpointStrategy endpoint_strategy; // endpoint/parking selection axis

    // Algorithm-specific tuning parameters. They are ignored by algorithms
    // that do not use the corresponding planning or assignment component.
    int task_sequence_limit; // PBS/wPBS only: maximum tasks planned per agent
    int wpbs_replan_window; // wPBS only: executed steps between replans
    int lns_time_limit;     // LNS only: assignment-improvement budget in seconds
    // LNS only: consecutive rejected moves before early stop; 0 disables it.
    int lns_no_improvement_limit;
    // CBS/ECBS only: focal bound; 1.0 is optimal CBS and >1.0 is ECBS.
    double ecbs_focal_weight;
    // CBS/ECBS only: maximum high-level conflict-tree nodes expanded per batch.
    int cbs_high_level_expansion_limit;
    // CBS/ECBS only: maximum nodes expanded by each low-level ECBS search.
    int cbs_low_level_expansion_limit;
    // Semi-online only: number of future release batches known in advance.
    int semi_online_lookahead_batches;

    MAPDConfig() : mode(MODE_ONLINE),
        assign_method(AM_DECOUPLED_GREEDY), assign_trigger(AT_ON_FREE_WAITS),
        mapf(MAPF_PP_PER_TASK), single_agent(SA_STA_TASK_EP),
        dummy_path(true), seed(0), runtime_limit_seconds(1800),
        endpoint_strategy(WAIT_OR_NEAREST_SAFE),
        task_sequence_limit(2), wpbs_replan_window(10), lns_time_limit(1),
        lns_no_improvement_limit(2000), ecbs_focal_weight(1.0),
        cbs_high_level_expansion_limit(INT_MAX),
        cbs_low_level_expansion_limit(INT_MAX),
        semi_online_lookahead_batches(1) {}
};

// ============ Algorithm Presets ============

inline MAPDConfig get_preset(const string& name) {
    MAPDConfig c;

    if (name == "TP") {
        c.mode = MODE_ONLINE;
        c.assign_method = AM_DECOUPLED_GREEDY;
        c.assign_trigger = AT_ON_FREE_WAITS;
        c.mapf = MAPF_PP_PER_TASK;
        c.single_agent = SA_STA_TASK_EP;
        c.dummy_path = true;
        c.endpoint_strategy = WAIT_OR_NEAREST_SAFE;
    }
    else if (name == "TPTS") {
        c.mode = MODE_ONLINE;
        c.assign_method = AM_DECOUPLED_GREEDY_SWAPS;
        c.assign_trigger = AT_ON_FREE_WAITS;
        c.mapf = MAPF_PP_PER_TASK;
        c.single_agent = SA_STA_TASK_EP;
        c.dummy_path = true;
        c.endpoint_strategy = WAIT_OR_NEAREST_SAFE;
    }
    else if (name == "CENTRAL" || name == "CENTRAL_CBS" ||
             name == "CENTRAL-CBS" || name == "CENTRAL-ECBS") {
        c.mode = MODE_ONLINE;
        c.assign_method = AM_CENTRAL_HUNGARIAN;
        c.assign_trigger = AT_EVERY_TIMESTEP;
        c.mapf = MAPF_CBS;
        c.single_agent = SA_STA_TASK_EP;
        // CENTRAL plans only to the next assigned endpoint; it does not append
        // a separate post-delivery dummy path.
        c.dummy_path = false;
        c.endpoint_strategy = NEAREST_AVAILABLE;
    }
    else if (name == "CENTRAL_FIXED" || name == "CENTRAL-FIXED" ||
             name == "CENTRAL-fixed" || name == "CENTRAL_FIXED_CBS" ||
             name == "CENTRAL-fixed-CBS" ||
             name == "CENTRAL_FIXED_ECBS" ||
             name == "CENTRAL-fixed-ECBS") {
        c.mode = MODE_ONLINE;
        c.assign_method = AM_CENTRAL_HUNGARIAN;
        c.assign_trigger = AT_ON_NEW_TASK_OR_AGENT_BECOMES_FREE;
        c.mapf = MAPF_CBS;
        c.single_agent = SA_STA_TASK_EP;
        // CENTRAL-fixed retains the endpoint-holding policy but avoids
        // recomputing the free-agent assignment on every timestep.
        c.dummy_path = false;
        c.endpoint_strategy = NEAREST_AVAILABLE;
    }
    else if (name == "HBH_MLA" || name == "HBH+MLA*" ||
             name == "HBH-MLA*") {
        c.mode = MODE_ONLINE;
        c.assign_method = AM_CENTRALIZED_GREEDY;
        c.assign_trigger = AT_ON_FREE_WAITS;
        c.mapf = MAPF_PP_PER_TASK;
        c.single_agent = SA_MLA_SEQUENCE;
        c.dummy_path = true;
        c.endpoint_strategy = WAIT_OR_NEAREST_FREE_NONTASK;
    }
    else if (name == "TA_PRIORITIZED") {
        c.mode = MODE_OFFLINE;
        c.assign_method = AM_LKH3_TSP;
        c.assign_trigger = AT_ONCE;
        c.mapf = MAPF_PP_TASK_SEQUENCE;
        c.single_agent = SA_SEQ_STA;
        c.dummy_path = true;
        c.endpoint_strategy = RETURN_TO_HOME;
    }
    else if (name == "TA_HYBRID") {
        c.mode = MODE_OFFLINE;
        c.assign_method = AM_LKH3_TSP_REASSIGN;
        // The initial offline release starts the LKH3 assignment. Later calls
        // reassign the fixed task set when an agent becomes available.
        c.assign_trigger =
            AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE;
        c.mapf = MAPF_TA_HYBRID_TWO_GROUP;
        c.single_agent = SA_STA_TASK_EP;
        c.dummy_path = true;
        c.endpoint_strategy = RETURN_TO_HOME;
    }
    else if (name == "HUNGARIAN_PBS") {
        c.mode = MODE_ONLINE;
        c.assign_method = AM_REPEATED_HUNGARIAN;
        c.assign_trigger = AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE;
        c.mapf = MAPF_PBS;
        c.single_agent = SA_MLA_SEQUENCE;
        c.dummy_path = true;
        c.endpoint_strategy = NEAREST_WITH_STRICT_EXCLUSIONS;
    }
    else if (name == "HUNGARIAN_wPBS") {
        c.mode = MODE_ONLINE;
        c.assign_method = AM_REPEATED_HUNGARIAN;
        c.assign_trigger = AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE;
        c.mapf = MAPF_wPBS;
        c.single_agent = SA_MLA_SEQUENCE;
        c.dummy_path = true;
        c.wpbs_replan_window = 10; // paper/reference planning window
        c.endpoint_strategy = PAIRWISE_TASK_THEN_HOME;
    }
    else if (name == "LNS_PBS") {
        c.mode = MODE_ONLINE;
        c.assign_method = AM_REPEATED_HUNGARIAN_LNS;
        c.assign_trigger = AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE;
        c.mapf = MAPF_PBS;
        c.single_agent = SA_MLA_SEQUENCE;
        c.dummy_path = true;
        c.lns_time_limit = 1;
        c.endpoint_strategy = NEAREST_WITH_STRICT_EXCLUSIONS;
    }
    else if (name == "LNS_wPBS") {
        c.mode = MODE_ONLINE;
        c.assign_method = AM_REPEATED_HUNGARIAN_LNS;
        c.assign_trigger = AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE;
        c.mapf = MAPF_wPBS;
        c.single_agent = SA_MLA_SEQUENCE;
        c.dummy_path = true;
        c.wpbs_replan_window = 10; // paper/reference planning window
        c.lns_time_limit = 1;
        c.endpoint_strategy = PAIRWISE_TASK_THEN_HOME;
    }
    else {
        throw invalid_argument("unsupported algorithm preset: " + name);
    }

    return c;
}
