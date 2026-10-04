// ============================================================================
//  Configuration validation
//
//  SUPPORT lists what the framework implements: each row is an (assignment
//  method, MAPF method) pair with the assignment triggers, replanning events,
//  agent selections, single-agent searches, endpoint strategies, task modes and
//  dummy-path settings it runs correctly with. A configuration is supported
//  when some row admits all of its components; anything else is rejected before
//  the simulation starts, so no combination runs with a component ignored.
//
//  Supporting a new combination means implementing it and adding or widening
//  its row.
// ============================================================================
#include "config_names.h"

#include <algorithm>
#include <set>
#include <sstream>
#include <vector>

namespace {

struct Support {
    AssignMethod assign;
    MAPFMethod mapf;
    std::vector<AssignTrigger> triggers;
    std::vector<int> replans;        // allowed sets of ReplanEvent flags
    std::vector<AgentSelection> selections;
    std::vector<SingleAgentMethod> single_agents;
    std::vector<EndpointStrategy> endpoints;
    std::vector<Mode> modes;
    std::vector<bool> dummy_paths;
};

const std::vector<Mode> ALL_MODES = {MODE_ONLINE, MODE_SEMI_ONLINE, MODE_OFFLINE};
// The single-agent searches of PBS, PP and CBS cannot reach pickups released
// far in the future (OFFLINE): PBS and PP then keep colliding waiting paths,
// CBS finds no solution.
const std::vector<Mode> NOT_OFFLINE = {MODE_ONLINE, MODE_SEMI_ONLINE};
// Planning inside assignment relocates an idle agent only when its endpoint is
// needed; strategies that always pick a new endpoint can deadlock there.
const std::vector<EndpointStrategy> RELOCATION_ENDPOINTS = {
    WAIT_OR_NEAREST_SAFE, WAIT_OR_NEAREST_FREE_NONTASK, NEAREST_AVAILABLE, RETURN_TO_HOME};
const std::vector<EndpointStrategy> FLEXIBLE_ENDPOINTS = {
    NEAREST_WITH_STRICT_EXCLUSIONS, NEAREST_AVAILABLE, RETURN_TO_HOME};
const std::vector<EndpointStrategy> WINDOWED_ENDPOINTS = {
    PAIRWISE_TASK_THEN_HOME, NEAREST_WITH_STRICT_EXCLUSIONS, NEAREST_AVAILABLE,
    RETURN_TO_HOME};
const std::vector<SingleAgentMethod> ANY_SEARCH = {
    SA_STA_TASK_EP, SA_MLA_SEQUENCE, SA_MLSIPP_SEQUENCE, SA_SIPP_SEGMENTS};
// wPBS stops every search at the end of its window; SIPP2 cannot.
const std::vector<SingleAgentMethod> WINDOWED_SEARCH = {
    SA_STA_TASK_EP, SA_MLA_SEQUENCE, SA_MLSIPP_SEQUENCE};
const std::vector<AgentSelection> CHANGED = {SELECT_CHANGED};
const std::vector<AgentSelection> ALL = {SELECT_ALL};
const std::vector<AgentSelection> EITHER_SELECTION = {SELECT_ALL, SELECT_CHANGED};
const std::vector<bool> WITH_DUMMY = {true};
const std::vector<bool> EITHER = {true, false};
const AssignTrigger ON_EVENT = AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE;

const int IN_ASSIGNMENT = 0;   // the assignment method plans while assigning
const int AFTER = REPLAN_AFTER_ASSIGNMENT;
const int SEGMENTS = REPLAN_AFTER_ASSIGNMENT | REPLAN_GOAL_REACHED;
const int WINDOWED = REPLAN_AFTER_ASSIGNMENT | REPLAN_PERIODIC;
const int ALL_EVENTS = REPLAN_AFTER_ASSIGNMENT | REPLAN_GOAL_REACHED | REPLAN_PERIODIC;
const std::vector<int> ANY_REPLAN = {AFTER, SEGMENTS, WINDOWED, ALL_EVENTS,
                                     REPLAN_EVERY_TIMESTEP};
// A plan that ends at a goal before the task's next leg (TA-Hybrid's groups,
// a CENTRAL pickup) must be followed by one when an agent gets there, and a
// windowed plan must be refreshed before its window runs out.
const std::vector<int> WITH_SEGMENTS = {SEGMENTS, ALL_EVENTS, REPLAN_EVERY_TIMESTEP};
const std::vector<int> WITH_REFRESH = {WINDOWED, ALL_EVENTS, REPLAN_EVERY_TIMESTEP};

std::vector<Support> supported_rows() {
    std::vector<Support> rows = {
        // TP, TPTS and HBH as published plan inside task assignment, one agent
        // at a time against the others' committed paths (per-task prioritized
        // planning); the WAIT_* strategies exist only for their relocation step.
        {AM_DECOUPLED_GREEDY, MAPF_PP_PER_TASK, {AT_ON_FREE_WAITS}, {IN_ASSIGNMENT},
         CHANGED, ANY_SEARCH, RELOCATION_ENDPOINTS, ALL_MODES, WITH_DUMMY},
        {AM_DECOUPLED_GREEDY_SWAPS, MAPF_PP_PER_TASK, {AT_ON_FREE_WAITS}, {IN_ASSIGNMENT},
         CHANGED, ANY_SEARCH, RELOCATION_ENDPOINTS, ALL_MODES, WITH_DUMMY},
        {AM_CENTRALIZED_GREEDY, MAPF_PP_PER_TASK, {AT_ON_FREE_WAITS}, {IN_ASSIGNMENT},
         CHANGED, ANY_SEARCH, RELOCATION_ENDPOINTS, ALL_MODES, WITH_DUMMY},
        // The LKH3 tour covers the complete task set, so it must be known upfront.
        {AM_LKH3_TSP, MAPF_PP_TASK_SEQUENCE, {AT_ONCE}, {AFTER}, ALL, {SA_SEQ_STA},
         {RETURN_TO_HOME, NEAREST_AVAILABLE}, {MODE_OFFLINE}, WITH_DUMMY},
        {AM_LKH3_TSP_REASSIGN, MAPF_TA_HYBRID_TWO_GROUP,
         {AT_EVERY_TIMESTEP, ON_EVENT}, {SEGMENTS},
         CHANGED, {SA_STA_TASK_EP}, {RETURN_TO_HOME, NEAREST_AVAILABLE}, {MODE_OFFLINE},
         EITHER},
    };
    // Methods that only assign tasks feed the same planners: repeated
    // Hungarian with or without LNS, TP, TPTS and HBH whenever planning is a
    // separate step (replan_on is set), and CENTRAL. PBS, PP and CBS plan the
    // selected agents' goal sequences with any single-agent search. CBS plans
    // the agents whose goals changed: one CBS over every agent's goal
    // sequence at every event hits exponential plateaus with 50 agents.
    for (AssignMethod method : {AM_REPEATED_HUNGARIAN, AM_REPEATED_HUNGARIAN_LNS,
                                AM_DECOUPLED_GREEDY, AM_DECOUPLED_GREEDY_SWAPS,
                                AM_CENTRALIZED_GREEDY, AM_CENTRAL_HUNGARIAN}) {
        const std::vector<AssignTrigger> triggers = {ON_EVENT, AT_EVERY_TIMESTEP};
        // CENTRAL matches released tasks only, so no search waits for a
        // pickup far in the future, and it parks idle agents itself, so CBS
        // and wPBS need no dummy path (PBS and PP cannot route around agents
        // holding their deliveries). An agent plans its matched pickup only,
        // so a replan must follow when it gets there.
        const bool central = method == AM_CENTRAL_HUNGARIAN;
        const std::vector<Mode> modes = central ? ALL_MODES : NOT_OFFLINE;
        const std::vector<int> replans = central ? WITH_SEGMENTS : ANY_REPLAN;
        const std::vector<bool> dummy_paths = central ? EITHER : WITH_DUMMY;
        for (MAPFMethod mapf : {MAPF_PBS, MAPF_PP_PER_TASK})
            rows.push_back({method, mapf, triggers, replans, EITHER_SELECTION, ANY_SEARCH,
                            FLEXIBLE_ENDPOINTS, modes, WITH_DUMMY});
        rows.push_back({method, MAPF_CBS, triggers, replans, CHANGED, ANY_SEARCH,
                        FLEXIBLE_ENDPOINTS, modes, dummy_paths});
        rows.push_back({method, MAPF_wPBS, triggers, WITH_REFRESH, ALL, WINDOWED_SEARCH,
                        WINDOWED_ENDPOINTS, ALL_MODES, dummy_paths});
    }
    // The LKH3 task sequences can be planned a few tasks at a time like any
    // other assignment; the plan is refreshed as agents work through them.
    // (Not by PBS or PP: the tour needs OFFLINE. CBS manages because its
    // tasks are all released at time 0.)
    const std::vector<AssignTrigger> tour_triggers = {AT_ONCE, ON_EVENT};
    rows.push_back({AM_LKH3_TSP, MAPF_wPBS, tour_triggers, WITH_REFRESH, ALL, WINDOWED_SEARCH,
                    WINDOWED_ENDPOINTS, {MODE_OFFLINE}, WITH_DUMMY});
    rows.push_back({AM_LKH3_TSP, MAPF_CBS, tour_triggers, ANY_REPLAN, CHANGED,
                    ANY_SEARCH, FLEXIBLE_ENDPOINTS, {MODE_OFFLINE}, WITH_DUMMY});
    return rows;
}

const std::vector<Support> SUPPORT = supported_rows();

template <typename T>
void check(std::ostringstream& reason, const char* field, T value,
           const std::vector<T>& allowed) {
    if (!reason.str().empty() ||
        std::find(allowed.begin(), allowed.end(), value) != allowed.end())
        return;
    reason << field << " " << enum_name(value) << " (supported:";
    for (T option : allowed) reason << " " << enum_name(option);
    reason << ")";
}

// Why `row` does not admit `config`; empty when it does.
std::string mismatch(const Support& row, const MAPDConfig& config) {
    std::ostringstream reason;
    check(reason, "assign_trigger", config.assign_trigger, row.triggers);
    if (reason.str().empty() &&
        std::find(row.replans.begin(), row.replans.end(), config.replan_on) ==
            row.replans.end()) {
        reason << "replan_on " << flags_name<ReplanEvent>(config.replan_on)
               << " (supported:";
        for (int flags : row.replans) reason << " " << flags_name<ReplanEvent>(flags);
        reason << ")";
    }
    check(reason, "agent_selection", config.agent_selection, row.selections);
    check(reason, "single_agent", config.single_agent, row.single_agents);
    check(reason, "endpoint_strategy", config.endpoint_strategy, row.endpoints);
    check(reason, "mode", config.mode, row.modes);
    check(reason, "dummy_path", config.dummy_path, row.dummy_paths);
    return reason.str();
}

}  // namespace

void validate_config(const MAPDConfig& config, bool has_tour) {
    // With a one-step window two agents can block each other forever.
    if (config.mapf == MAPF_wPBS && config.wpbs_replan_window < 2)
        throw std::invalid_argument("wPBS requires --wpbs_replan_window of at least 2");
    std::set<std::string> mapf_options;
    std::vector<std::string> reasons;
    for (const Support& row : SUPPORT) {
        if (row.assign != config.assign_method) continue;
        mapf_options.insert(enum_name(row.mapf));
        if (row.mapf != config.mapf) continue;
        std::string reason = mismatch(row, config);
        if (reason.empty()) {
            if ((config.assign_method == AM_LKH3_TSP ||
                 config.assign_method == AM_LKH3_TSP_REASSIGN) && !has_tour)
                throw std::invalid_argument(
                    std::string(enum_name(config.assign_method)) + " requires --tour <file>");
            return;
        }
        reasons.push_back(reason);
    }

    std::ostringstream message;
    if (mapf_options.empty()) {
        message << "assign_method " << enum_name(config.assign_method) << " is not implemented";
    } else if (reasons.empty()) {
        message << "MAPF method " << enum_name(config.mapf) << " is not supported with "
                << enum_name(config.assign_method) << "; supported:";
        for (const std::string& option : mapf_options) message << " " << option;
    } else {
        message << enum_name(config.assign_method) << " + " << enum_name(config.mapf)
                << " is not supported with ";
        for (size_t i = 0; i < reasons.size(); i++)
            message << (i ? "; or with " : "") << reasons[i];
    }
    throw std::invalid_argument(message.str());
}
