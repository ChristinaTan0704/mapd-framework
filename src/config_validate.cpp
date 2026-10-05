// ============================================================================
//  Configuration validation
//
//  SUPPORT lists what the framework implements: each row is an (assignment
//  method, MAPF method) pair with the assignment triggers, replanning events,
//  agent selections, single-agent searches, endpoint strategies, task modes and
//  dummy-path settings it runs correctly with. A configuration is supported
//  when some row admits all of its components; anything else is rejected before
//  the simulation starts.
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
const std::vector<EndpointStrategy> ALL_ENDPOINTS = {
    WAIT_OR_NEAREST_SAFE, WAIT_OR_NEAREST_FREE_NONTASK, NEAREST_AVAILABLE,
    NEAREST_WITH_STRICT_EXCLUSIONS, PAIRWISE_TASK_THEN_HOME, RETURN_TO_HOME};
const std::vector<SingleAgentMethod> ANY_SEARCH = {
    SA_STA_TASK_EP, SA_MLA_SEQUENCE, SA_MLSIPP_SEQUENCE, SA_SIPP_SEGMENTS, SA_SEQ_STA};
const std::vector<AgentSelection> CHANGED = {SELECT_CHANGED};
const std::vector<AgentSelection> ANY_SELECTION = {SELECT_ALL, SELECT_CHANGED,
                                                   SELECT_CHANGED_OR_NOT_CARRYING};
const std::vector<bool> EITHER = {true, false};
const AssignTrigger ON_EVENT = AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE;
const AssignTrigger AT_PICKUP = AT_ON_AGENT_FREE_OR_AT_PICKUP;

const int WHEN_ASSIGNED = 0;   // each agent is planned when assigned, inside the assignment
const int AFTER = REPLAN_AFTER_ASSIGNMENT;
const int SEGMENTS = REPLAN_AFTER_ASSIGNMENT | REPLAN_GOAL_REACHED;
const int WINDOWED = REPLAN_AFTER_ASSIGNMENT | REPLAN_PERIODIC;
const int ALL_EVENTS = REPLAN_AFTER_ASSIGNMENT | REPLAN_GOAL_REACHED | REPLAN_PERIODIC;
const int PERIODIC = REPLAN_PERIODIC;
const int GOAL_PERIODIC = REPLAN_GOAL_REACHED | REPLAN_PERIODIC;
const std::vector<int> ANY_REPLAN = {AFTER, SEGMENTS, WINDOWED, ALL_EVENTS, PERIODIC,
                                     GOAL_PERIODIC, REPLAN_EVERY_TIMESTEP};
// A plan that ends at a goal before the task's next leg (TA-Hybrid's groups,
// a CENTRAL pickup) must be followed by one when an agent gets there, and a
// windowed plan must be refreshed before its window runs out.
const std::vector<int> WITH_SEGMENTS = {SEGMENTS, ALL_EVENTS, GOAL_PERIODIC,
                                        REPLAN_EVERY_TIMESTEP};
const std::vector<int> WITH_REFRESH = {WINDOWED, ALL_EVENTS, PERIODIC, GOAL_PERIODIC,
                                       REPLAN_EVERY_TIMESTEP};
// CENTRAL's delivery legs: per-goal or periodic replans.
const std::vector<int> SEGMENTS_OR_REFRESH = {SEGMENTS, WINDOWED, ALL_EVENTS, PERIODIC,
                                              GOAL_PERIODIC, REPLAN_EVERY_TIMESTEP};

std::vector<Support> supported_rows() {
    std::vector<Support> rows = {
        // TP, TPTS and HBH as published plan inside task assignment, one agent
        // at a time against the others' committed paths (per-task prioritized
        // planning). Only the agent being assigned is planned (CHANGED): the
        // others' paths fix when they finish their tasks, which the
        // assignment relies on.
        {AM_DECOUPLED_GREEDY, MAPF_PP_PER_TASK, {AT_ON_FREE_WAITS}, {WHEN_ASSIGNED},
         CHANGED, ANY_SEARCH, ALL_ENDPOINTS, ALL_MODES, EITHER},
        {AM_DECOUPLED_GREEDY_SWAPS, MAPF_PP_PER_TASK, {AT_ON_FREE_WAITS}, {WHEN_ASSIGNED},
         CHANGED, ANY_SEARCH, ALL_ENDPOINTS, ALL_MODES, EITHER},
        {AM_CENTRALIZED_GREEDY, MAPF_PP_PER_TASK, {AT_ON_FREE_WAITS}, {WHEN_ASSIGNED},
         CHANGED, ANY_SEARCH, ALL_ENDPOINTS, ALL_MODES, EITHER},
        // The LKH3 tour covers the complete task set, so it must be known
        // upfront. TA-Hybrid's two groups plan each carrier's next goal and
        // route free agents to their pickups.
        {AM_LKH3_TSP_REASSIGN, MAPF_TA_HYBRID_TWO_GROUP,
         {AT_EVERY_TIMESTEP, ON_EVENT, AT_PICKUP, AT_ON_FREE_WAITS}, WITH_SEGMENTS,
         ANY_SELECTION, ANY_SEARCH, ALL_ENDPOINTS, {MODE_OFFLINE}, EITHER},
    };
    // Methods that only assign tasks feed every planner: repeated Hungarian
    // with or without LNS, and TP, TPTS and HBH whenever planning is a
    // separate step (replan_on is set). The prioritized planners, PBS and CBS
    // plan every selected agent's next tasks (PP_TASK_SEQUENCE all of them);
    // wPBS plans a window of them and must refresh it before it runs out.
    const std::vector<AssignTrigger> triggers = {ON_EVENT, AT_PICKUP, AT_EVERY_TIMESTEP,
                                                 AT_ON_FREE_WAITS};
    for (AssignMethod method : {AM_REPEATED_HUNGARIAN, AM_REPEATED_HUNGARIAN_LNS,
                                AM_DECOUPLED_GREEDY, AM_DECOUPLED_GREEDY_SWAPS,
                                AM_CENTRALIZED_GREEDY}) {
        for (MAPFMethod mapf : {MAPF_PP_PER_TASK, MAPF_PP_TASK_SEQUENCE, MAPF_PBS, MAPF_CBS})
            rows.push_back({method, mapf, triggers, ANY_REPLAN, ANY_SELECTION,
                            ANY_SEARCH, ALL_ENDPOINTS, ALL_MODES, EITHER});
        rows.push_back({method, MAPF_wPBS, triggers, WITH_REFRESH, ANY_SELECTION,
                        ANY_SEARCH, ALL_ENDPOINTS, ALL_MODES, EITHER});
    }
    // CENTRAL plans an agent to its matched pickup only, so a replan must
    // follow when it gets there (wPBS reaches it at its periodic replan).
    for (AssignMethod method : {AM_CENTRAL_HUNGARIAN, AM_CENTRAL_HUNGARIAN_FIXED}) {
        for (MAPFMethod mapf : {MAPF_PP_PER_TASK, MAPF_PP_TASK_SEQUENCE, MAPF_PBS, MAPF_CBS})
            rows.push_back({method, mapf, triggers, SEGMENTS_OR_REFRESH, ANY_SELECTION,
                            ANY_SEARCH, ALL_ENDPOINTS, ALL_MODES, EITHER});
        rows.push_back({method, MAPF_wPBS, triggers, WITH_REFRESH, ANY_SELECTION,
                        ANY_SEARCH, ALL_ENDPOINTS, ALL_MODES, EITHER});
    }
    // The LKH3 task sequences feed the same planners. A planner that plans only
    // the next tasks of a sequence must be refreshed periodically as the
    // agents work through it; PP_TASK_SEQUENCE plans all of it, and
    // replanning complete tours at every goal or period (n(n+1)/2 whole-tour
    // searches each time) does not finish in reasonable time.
    // LKH3_TSP_REASSIGN also lets an idle agent take over a task not yet
    // started; it feeds the same planners.
    const std::vector<Mode> OFFLINE = {MODE_OFFLINE};
    for (AssignMethod method : {AM_LKH3_TSP, AM_LKH3_TSP_REASSIGN}) {
        // Reassignment acts after time 0, so ONCE would make it LKH3_TSP.
        std::vector<AssignTrigger> tour_triggers = {ON_EVENT, AT_PICKUP, AT_EVERY_TIMESTEP,
                                                    AT_ON_FREE_WAITS};
        std::vector<AssignTrigger> sequence_triggers = {ON_EVENT};
        if (method == AM_LKH3_TSP) {
            tour_triggers.insert(tour_triggers.begin(), AT_ONCE);
            sequence_triggers.insert(sequence_triggers.begin(), AT_ONCE);
        }
        rows.push_back({method, MAPF_PP_TASK_SEQUENCE, sequence_triggers, {AFTER},
                        ANY_SELECTION, ANY_SEARCH, ALL_ENDPOINTS, OFFLINE, EITHER});
        for (MAPFMethod mapf : {MAPF_PP_PER_TASK, MAPF_PBS, MAPF_CBS, MAPF_wPBS})
            rows.push_back({method, mapf, tour_triggers, WITH_REFRESH, ANY_SELECTION,
                            ANY_SEARCH, ALL_ENDPOINTS, OFFLINE, EITHER});
    }
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
    // Replanning every timestep subsumes the other replan events.
    const int replan_on = (config.replan_on & REPLAN_EVERY_TIMESTEP) ? REPLAN_EVERY_TIMESTEP
                                                                     : config.replan_on;
    if (reason.str().empty() &&
        std::find(row.replans.begin(), row.replans.end(), replan_on) ==
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
