// ============================================================================
//  Configuration validation
//
//  SUPPORT lists every (assignment method, MAPF method) pair the framework
//  implements, together with the assignment triggers, single-agent searches,
//  endpoint strategies and task modes that pair runs correctly with. Any other
//  configuration is rejected before the simulation starts, so an unsupported
//  combination can never run silently with a component ignored.
//
//  Supporting a new combination means implementing it and widening its row.
// ============================================================================
#include "config.h"

#include <algorithm>
#include <sstream>
#include <vector>

namespace {

const char* name(Mode v) {
    switch (v) {
    case MODE_ONLINE: return "ONLINE";
    case MODE_OFFLINE: return "OFFLINE";
    case MODE_SEMI_ONLINE: return "SEMI_ONLINE";
    }
    return "?";
}

const char* name(AssignMethod v) {
    switch (v) {
    case AM_DECOUPLED_GREEDY: return "DECOUPLED_GREEDY";
    case AM_CENTRALIZED_GREEDY: return "CENTRALIZED_GREEDY";
    case AM_CENTRAL_HUNGARIAN: return "CENTRAL_HUNGARIAN";
    case AM_REPEATED_HUNGARIAN: return "REPEATED_HUNGARIAN";
    case AM_DECOUPLED_GREEDY_SWAPS: return "DECOUPLED_GREEDY_SWAPS";
    case AM_LKH3_TSP: return "LKH3_TSP";
    case AM_LKH3_TSP_REASSIGN: return "LKH3_TSP_REASSIGN";
    case AM_GREEDY_INSERT_LNS: return "GREEDY_INSERT_LNS";
    case AM_REPEATED_HUNGARIAN_LNS: return "REPEATED_HUNGARIAN_LNS";
    }
    return "?";
}

const char* name(AssignTrigger v) {
    switch (v) {
    case AT_ON_FREE_WAITS: return "ON_FREE_WAITS";
    case AT_EVERY_TIMESTEP: return "EVERY_TIMESTEP";
    case AT_ON_NEW_TASK_OR_AGENT_BECOMES_FREE: return "ON_NEW_TASK_OR_AGENT_BECOMES_FREE";
    case AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE:
        return "ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE";
    case AT_ONCE: return "ONCE";
    }
    return "?";
}

const char* name(MAPFMethod v) {
    switch (v) {
    case MAPF_PP_PER_TASK: return "PP";
    case MAPF_PP_TASK_SEQUENCE: return "PP_TASK_SEQUENCE";
    case MAPF_CBS: return "CBS";
    case MAPF_PBS: return "PBS";
    case MAPF_wPBS: return "wPBS";
    case MAPF_TA_HYBRID_TWO_GROUP: return "TA_HYBRID_TWO_GROUP";
    }
    return "?";
}

const char* name(SingleAgentMethod v) {
    switch (v) {
    case SA_STA_TASK_EP: return "STA";
    case SA_STA_NONTASK_EP: return "STA_NONTASK_EP";
    case SA_MLA_SEQUENCE: return "MLA";
    case SA_MLSIPP_SEQUENCE: return "MLSIPP";
    case SA_SIPP_SEGMENTS: return "SIPP2";
    case SA_SEQ_STA: return "SEQ_STA";
    }
    return "?";
}

const char* name(EndpointStrategy v) {
    switch (v) {
    case RETURN_TO_HOME: return "RETURN_TO_HOME";
    case NEAREST_WITH_STRICT_EXCLUSIONS: return "NEAREST_WITH_STRICT_EXCLUSIONS";
    case PAIRWISE_TASK_THEN_HOME: return "PAIRWISE_TASK_THEN_HOME";
    case WAIT_OR_NEAREST_SAFE: return "WAIT_OR_NEAREST_SAFE";
    case WAIT_OR_NEAREST_FREE_NONTASK: return "WAIT_OR_NEAREST_FREE_NONTASK";
    case NEAREST_AVAILABLE: return "NEAREST_AVAILABLE";
    }
    return "?";
}

struct Support {
    AssignMethod assign;
    MAPFMethod mapf;
    std::vector<AssignTrigger> triggers;
    std::vector<SingleAgentMethod> single_agents;
    std::vector<EndpointStrategy> endpoints;
    std::vector<Mode> modes;
};

const std::vector<Mode> ALL_MODES = {MODE_ONLINE, MODE_SEMI_ONLINE, MODE_OFFLINE};
const std::vector<EndpointStrategy> ALL_ENDPOINTS = {
    WAIT_OR_NEAREST_SAFE, WAIT_OR_NEAREST_FREE_NONTASK, NEAREST_AVAILABLE,
    NEAREST_WITH_STRICT_EXCLUSIONS, PAIRWISE_TASK_THEN_HOME, RETURN_TO_HOME};
const AssignTrigger ON_EVENT = AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE;

// The decoupled methods (TP, TPTS, HBH) plan inside task assignment, one agent
// at a time against the others' committed paths, i.e. per-task prioritized
// planning; the WAIT_* strategies exist only for their relocation step.
const std::vector<Support> SUPPORT = {
    {AM_DECOUPLED_GREEDY, MAPF_PP_PER_TASK, {AT_ON_FREE_WAITS},
     {SA_STA_TASK_EP, SA_MLSIPP_SEQUENCE}, ALL_ENDPOINTS, ALL_MODES},
    {AM_DECOUPLED_GREEDY_SWAPS, MAPF_PP_PER_TASK, {AT_ON_FREE_WAITS},
     {SA_STA_TASK_EP, SA_MLSIPP_SEQUENCE},
     {WAIT_OR_NEAREST_SAFE, WAIT_OR_NEAREST_FREE_NONTASK, NEAREST_AVAILABLE,
      NEAREST_WITH_STRICT_EXCLUSIONS, RETURN_TO_HOME}, ALL_MODES},
    {AM_CENTRALIZED_GREEDY, MAPF_PP_PER_TASK, {AT_ON_FREE_WAITS},
     {SA_MLA_SEQUENCE, SA_MLSIPP_SEQUENCE, SA_SIPP_SEGMENTS}, ALL_ENDPOINTS, ALL_MODES},
    // CBS runs its own low-level search (src/cbs.cpp).
    {AM_CENTRAL_HUNGARIAN, MAPF_CBS,
     {AT_EVERY_TIMESTEP, AT_ON_NEW_TASK_OR_AGENT_BECOMES_FREE}, {SA_STA_TASK_EP},
     {NEAREST_AVAILABLE, NEAREST_WITH_STRICT_EXCLUSIONS, RETURN_TO_HOME}, ALL_MODES},
    {AM_REPEATED_HUNGARIAN, MAPF_PBS, {ON_EVENT}, {SA_MLA_SEQUENCE, SA_MLSIPP_SEQUENCE},
     {NEAREST_WITH_STRICT_EXCLUSIONS, NEAREST_AVAILABLE, RETURN_TO_HOME}, ALL_MODES},
    {AM_REPEATED_HUNGARIAN, MAPF_wPBS, {ON_EVENT}, {SA_MLA_SEQUENCE, SA_MLSIPP_SEQUENCE},
     {PAIRWISE_TASK_THEN_HOME, NEAREST_WITH_STRICT_EXCLUSIONS, NEAREST_AVAILABLE,
      RETURN_TO_HOME}, ALL_MODES},
    {AM_REPEATED_HUNGARIAN, MAPF_PP_PER_TASK, {ON_EVENT},
     {SA_MLA_SEQUENCE, SA_MLSIPP_SEQUENCE},
     {NEAREST_WITH_STRICT_EXCLUSIONS, NEAREST_AVAILABLE, RETURN_TO_HOME}, ALL_MODES},
    {AM_REPEATED_HUNGARIAN_LNS, MAPF_PBS, {ON_EVENT}, {SA_MLA_SEQUENCE, SA_MLSIPP_SEQUENCE},
     {NEAREST_WITH_STRICT_EXCLUSIONS, NEAREST_AVAILABLE, RETURN_TO_HOME}, ALL_MODES},
    {AM_REPEATED_HUNGARIAN_LNS, MAPF_wPBS, {ON_EVENT}, {SA_MLA_SEQUENCE, SA_MLSIPP_SEQUENCE},
     {PAIRWISE_TASK_THEN_HOME, NEAREST_WITH_STRICT_EXCLUSIONS, NEAREST_AVAILABLE,
      RETURN_TO_HOME}, ALL_MODES},
    {AM_REPEATED_HUNGARIAN_LNS, MAPF_PP_PER_TASK, {ON_EVENT},
     {SA_MLA_SEQUENCE, SA_MLSIPP_SEQUENCE},
     {NEAREST_WITH_STRICT_EXCLUSIONS, NEAREST_AVAILABLE, RETURN_TO_HOME}, ALL_MODES},
    // The LKH3 tour covers the complete task set, so it must be known upfront.
    {AM_LKH3_TSP, MAPF_PP_TASK_SEQUENCE, {AT_ONCE}, {SA_SEQ_STA},
     {RETURN_TO_HOME, NEAREST_AVAILABLE}, {MODE_OFFLINE}},
    {AM_LKH3_TSP_REASSIGN, MAPF_TA_HYBRID_TWO_GROUP, {ON_EVENT}, {SA_STA_TASK_EP},
     {RETURN_TO_HOME, NEAREST_AVAILABLE}, {MODE_OFFLINE}},
};

template <typename T>
void require(const char* field, T value, const std::vector<T>& allowed,
             const Support& row) {
    if (std::find(allowed.begin(), allowed.end(), value) != allowed.end()) return;
    std::ostringstream message;
    message << field << " " << name(value) << " is not supported with "
            << name(row.assign) << " + " << name(row.mapf) << "; supported:";
    for (T option : allowed) message << " " << name(option);
    throw std::invalid_argument(message.str());
}

}  // namespace

void validate_config(const MAPDConfig& config, bool has_tour) {
    const Support* row = nullptr;
    std::ostringstream pairs;
    for (const Support& support : SUPPORT) {
        if (support.assign != config.assign_method) continue;
        if (support.mapf == config.mapf) row = &support;
        pairs << " " << name(support.mapf);
    }
    if (pairs.str().empty())
        throw std::invalid_argument(std::string("assign_method ") +
                                    name(config.assign_method) + " is not implemented");
    if (row == nullptr) {
        std::ostringstream message;
        message << "MAPF method " << name(config.mapf) << " is not supported with "
                << name(config.assign_method) << "; supported:" << pairs.str();
        throw std::invalid_argument(message.str());
    }

    require("assign_trigger", config.assign_trigger, row->triggers, *row);
    require("single_agent", config.single_agent, row->single_agents, *row);
    require("endpoint_strategy", config.endpoint_strategy, row->endpoints, *row);
    require("mode", config.mode, row->modes, *row);

    if ((config.assign_method == AM_LKH3_TSP ||
         config.assign_method == AM_LKH3_TSP_REASSIGN) && !has_tour)
        throw std::invalid_argument(
            std::string(name(config.assign_method)) + " requires --tour <file>");
}
