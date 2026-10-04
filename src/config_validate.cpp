// ============================================================================
//  Configuration validation
//
//  SUPPORT lists every (assignment method, MAPF method) pair the framework
//  implements, with the assignment triggers, replanning events, agent
//  selections, single-agent searches, endpoint strategies, task modes and
//  dummy-path settings that the pair runs correctly with. Any other configuration is rejected before the simulation starts, so
//  no combination can run silently with a component ignored.
//
//  Supporting a new combination means implementing it and widening its row.
// ============================================================================
#include "config_names.h"

#include <algorithm>
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
const std::vector<EndpointStrategy> FLEXIBLE_ENDPOINTS = {
    NEAREST_WITH_STRICT_EXCLUSIONS, NEAREST_AVAILABLE, RETURN_TO_HOME};
const std::vector<SingleAgentMethod> ANY_SEARCH = {
    SA_STA_TASK_EP, SA_MLA_SEQUENCE, SA_MLSIPP_SEQUENCE, SA_SIPP_SEGMENTS};
// wPBS runs its own windowed solver (native_wpbs_solve) with MLA* or MLSIPP.
const std::vector<SingleAgentMethod> MLA_OR_MLSIPP = {SA_MLA_SEQUENCE, SA_MLSIPP_SEQUENCE};
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
// Segment planners must plan each carrier's next leg, and a windowed plan
// must be refreshed before its window runs out.
const std::vector<int> WITH_SEGMENTS = {SEGMENTS, ALL_EVENTS, REPLAN_EVERY_TIMESTEP};
const std::vector<int> WITH_REFRESH = {WINDOWED, ALL_EVENTS, REPLAN_EVERY_TIMESTEP};

std::vector<Support> supported_pairs() {
    std::vector<Support> rows = {
        // TP, TPTS and HBH plan inside task assignment, one agent at a time
        // against the others' committed paths (per-task prioritized planning);
        // the WAIT_* strategies exist only for their relocation step.
        {AM_DECOUPLED_GREEDY, MAPF_PP_PER_TASK, {AT_ON_FREE_WAITS}, {IN_ASSIGNMENT},
         CHANGED, ANY_SEARCH, ALL_ENDPOINTS, ALL_MODES, WITH_DUMMY},
        {AM_DECOUPLED_GREEDY_SWAPS, MAPF_PP_PER_TASK, {AT_ON_FREE_WAITS}, {IN_ASSIGNMENT},
         CHANGED, ANY_SEARCH,
         {WAIT_OR_NEAREST_SAFE, WAIT_OR_NEAREST_FREE_NONTASK, NEAREST_AVAILABLE,
          NEAREST_WITH_STRICT_EXCLUSIONS, RETURN_TO_HOME}, ALL_MODES, WITH_DUMMY},
        {AM_CENTRALIZED_GREEDY, MAPF_PP_PER_TASK, {AT_ON_FREE_WAITS}, {IN_ASSIGNMENT},
         CHANGED, ANY_SEARCH, ALL_ENDPOINTS, ALL_MODES, WITH_DUMMY},
        // CENTRAL runs CBS with its own low-level search (src/cbs.cpp) and
        // routes free agents to parking endpoints during assignment.
        {AM_CENTRAL_HUNGARIAN, MAPF_CBS,
         {AT_EVERY_TIMESTEP, AT_ON_NEW_TASK_OR_AGENT_BECOMES_FREE},
         WITH_SEGMENTS, CHANGED, {SA_STA_TASK_EP},
         {NEAREST_AVAILABLE, NEAREST_WITH_STRICT_EXCLUSIONS, RETURN_TO_HOME}, ALL_MODES,
         {false}},
        // The LKH3 tour covers the complete task set, so it must be known upfront.
        {AM_LKH3_TSP, MAPF_PP_TASK_SEQUENCE, {AT_ONCE}, {AFTER}, ALL, {SA_SEQ_STA},
         {RETURN_TO_HOME, NEAREST_AVAILABLE}, {MODE_OFFLINE}, WITH_DUMMY},
        {AM_LKH3_TSP_REASSIGN, MAPF_TA_HYBRID_TWO_GROUP, {ON_EVENT}, {SEGMENTS},
         CHANGED, {SA_STA_TASK_EP},
         {RETURN_TO_HOME, NEAREST_AVAILABLE}, {MODE_OFFLINE}, EITHER},
    };
    // Repeated Hungarian, with or without LNS, feeds the same planners.
    for (AssignMethod hungarian : {AM_REPEATED_HUNGARIAN, AM_REPEATED_HUNGARIAN_LNS}) {
        rows.push_back({hungarian, MAPF_PBS, {ON_EVENT}, ANY_REPLAN, EITHER_SELECTION,
                        ANY_SEARCH,
                        FLEXIBLE_ENDPOINTS, ALL_MODES, WITH_DUMMY});
        rows.push_back({hungarian, MAPF_wPBS, {ON_EVENT}, WITH_REFRESH, ALL, MLA_OR_MLSIPP,
                        {PAIRWISE_TASK_THEN_HOME, NEAREST_WITH_STRICT_EXCLUSIONS,
                         NEAREST_AVAILABLE, RETURN_TO_HOME}, ALL_MODES, WITH_DUMMY});
        rows.push_back({hungarian, MAPF_PP_PER_TASK, {ON_EVENT}, ANY_REPLAN, EITHER_SELECTION,
                        ANY_SEARCH,
                        FLEXIBLE_ENDPOINTS, ALL_MODES, WITH_DUMMY});
    }
    return rows;
}

const std::vector<Support> SUPPORT = supported_pairs();

template <typename T>
void require(const char* field, T value, const std::vector<T>& allowed,
             const Support& row) {
    if (std::find(allowed.begin(), allowed.end(), value) != allowed.end()) return;
    std::ostringstream message;
    message << field << " " << enum_name(value) << " is not supported with "
            << enum_name(row.assign) << " + " << enum_name(row.mapf) << "; supported:";
    for (T option : allowed) message << " " << enum_name(option);
    throw std::invalid_argument(message.str());
}

}  // namespace

void validate_config(const MAPDConfig& config, bool has_tour) {
    const Support* row = nullptr;
    std::ostringstream pairs;
    for (const Support& support : SUPPORT) {
        if (support.assign != config.assign_method) continue;
        if (support.mapf == config.mapf) row = &support;
        pairs << " " << enum_name(support.mapf);
    }
    if (pairs.str().empty())
        throw std::invalid_argument(std::string("assign_method ") +
                                    enum_name(config.assign_method) + " is not implemented");
    if (row == nullptr) {
        std::ostringstream message;
        message << "MAPF method " << enum_name(config.mapf) << " is not supported with "
                << enum_name(config.assign_method) << "; supported:" << pairs.str();
        throw std::invalid_argument(message.str());
    }

    require("assign_trigger", config.assign_trigger, row->triggers, *row);
    if (std::find(row->replans.begin(), row->replans.end(), config.replan_on) ==
        row->replans.end()) {
        std::ostringstream message;
        message << "replan_on " << flags_name<ReplanEvent>(config.replan_on)
                << " is not supported with " << enum_name(row->assign) << " + "
                << enum_name(row->mapf) << "; supported:";
        for (int flags : row->replans) message << " " << flags_name<ReplanEvent>(flags);
        throw std::invalid_argument(message.str());
    }
    require("agent_selection", config.agent_selection, row->selections, *row);
    require("single_agent", config.single_agent, row->single_agents, *row);
    require("endpoint_strategy", config.endpoint_strategy, row->endpoints, *row);
    require("mode", config.mode, row->modes, *row);
    require("dummy_path", config.dummy_path, row->dummy_paths, *row);

    if ((config.assign_method == AM_LKH3_TSP ||
         config.assign_method == AM_LKH3_TSP_REASSIGN) && !has_tour)
        throw std::invalid_argument(
            std::string(enum_name(config.assign_method)) + " requires --tour <file>");
}
