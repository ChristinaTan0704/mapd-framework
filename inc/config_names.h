#pragma once
// ============================================================================
//  Names of the configuration enums, shared by the command line (driver.cpp)
//  and the validation messages (config_validate.cpp). The first name of a value
//  is its canonical name; later names are accepted aliases.
// ============================================================================
#include "config.h"

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

template <typename E>
using NameTable = std::vector<std::pair<E, const char*>>;

inline const NameTable<Mode>& names(Mode) {
    static const NameTable<Mode> table = {
        {MODE_ONLINE, "ONLINE"}, {MODE_SEMI_ONLINE, "SEMI_ONLINE"},
        {MODE_OFFLINE, "OFFLINE"}};
    return table;
}

inline const NameTable<AssignMethod>& names(AssignMethod) {
    static const NameTable<AssignMethod> table = {
        {AM_DECOUPLED_GREEDY, "DECOUPLED_GREEDY"},
        {AM_DECOUPLED_GREEDY_SWAPS, "DECOUPLED_GREEDY_SWAPS"},
        {AM_CENTRALIZED_GREEDY, "CENTRALIZED_GREEDY"},
        {AM_CENTRAL_HUNGARIAN, "CENTRAL_HUNGARIAN"},
        {AM_REPEATED_HUNGARIAN, "REPEATED_HUNGARIAN"},
        {AM_REPEATED_HUNGARIAN_LNS, "REPEATED_HUNGARIAN_LNS"},
        {AM_LKH3_TSP, "LKH3_TSP"},
        {AM_LKH3_TSP_REASSIGN, "LKH3_TSP_REASSIGN"},
        {AM_GREEDY_INSERT_LNS, "GREEDY_INSERT_LNS"}};
    return table;
}

inline const NameTable<AssignTrigger>& names(AssignTrigger) {
    static const NameTable<AssignTrigger> table = {
        {AT_ON_FREE_WAITS, "ON_FREE_WAITS"},
        {AT_EVERY_TIMESTEP, "EVERY_TIMESTEP"},
        {AT_ON_NEW_TASK_OR_AGENT_BECOMES_FREE, "ON_NEW_TASK_OR_AGENT_BECOMES_FREE"},
        {AT_ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE,
         "ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE"},
        {AT_ONCE, "ONCE"}};
    return table;
}

inline const NameTable<MAPFMethod>& names(MAPFMethod) {
    static const NameTable<MAPFMethod> table = {
        {MAPF_PP_PER_TASK, "PP"}, {MAPF_PP_PER_TASK, "PP_PER_TASK"},
        {MAPF_PP_TASK_SEQUENCE, "PP_TASK_SEQUENCE"},
        {MAPF_CBS, "CBS"}, {MAPF_PBS, "PBS"}, {MAPF_wPBS, "wPBS"},
        {MAPF_TA_HYBRID_TWO_GROUP, "TA_HYBRID_TWO_GROUP"}};
    return table;
}

inline const NameTable<SingleAgentMethod>& names(SingleAgentMethod) {
    static const NameTable<SingleAgentMethod> table = {
        {SA_STA_TASK_EP, "STA"},
        {SA_MLA_SEQUENCE, "MLA"},
        {SA_MLSIPP_SEQUENCE, "MLSIPP"}, {SA_MLSIPP_SEQUENCE, "SIPP"},
        {SA_SIPP_SEGMENTS, "SIPP2"}, {SA_SIPP_SEGMENTS, "2SEG_SIPP"},
        {SA_SEQ_STA, "SEQ_STA"}};
    return table;
}

inline const NameTable<EndpointStrategy>& names(EndpointStrategy) {
    static const NameTable<EndpointStrategy> table = {
        {WAIT_OR_NEAREST_SAFE, "WAIT_OR_NEAREST_SAFE"},
        {WAIT_OR_NEAREST_FREE_NONTASK, "WAIT_OR_NEAREST_FREE_NONTASK"},
        {NEAREST_AVAILABLE, "NEAREST_AVAILABLE"},
        {NEAREST_WITH_STRICT_EXCLUSIONS, "NEAREST_WITH_STRICT_EXCLUSIONS"},
        {PAIRWISE_TASK_THEN_HOME, "PAIRWISE_TASK_THEN_HOME"},
        {RETURN_TO_HOME, "RETURN_TO_HOME"}};
    return table;
}

inline const NameTable<TieBreaking>& names(TieBreaking) {
    static const NameTable<TieBreaking> table = {
        {TIE_BREAK_REFERENCE, "reference"}, {TIE_BREAK_RANDOM, "random"}};
    return table;
}

inline const NameTable<CBSConflictSelection>& names(CBSConflictSelection) {
    static const NameTable<CBSConflictSelection> table = {
        {CBS_CONFLICT_CARDINAL, "cardinal"}, {CBS_CONFLICT_EARLIEST, "earliest"}};
    return table;
}

inline const NameTable<ReplanEvent>& names(ReplanEvent) {
    static const NameTable<ReplanEvent> table = {
        {REPLAN_AFTER_ASSIGNMENT, "AFTER_ASSIGNMENT"},
        {REPLAN_EVERY_TIMESTEP, "EVERY_TIMESTEP"},
        {REPLAN_GOAL_REACHED, "GOAL_REACHED"},
        {REPLAN_PERIODIC, "PERIODIC"}};
    return table;
}

inline const NameTable<AgentSelection>& names(AgentSelection) {
    static const NameTable<AgentSelection> table = {
        {SELECT_ALL, "ALL"}, {SELECT_CHANGED, "CHANGED"}};
    return table;
}

inline const NameTable<bool>& names(bool) {
    static const NameTable<bool> table = {{false, "0"}, {true, "1"}};
    return table;
}

template <typename E>
const char* enum_name(E value) {
    for (const auto& entry : names(value))
        if (entry.first == value) return entry.second;
    return "?";
}

// Canonical names of every value, for help texts and error messages.
template <typename E>
std::string enum_choices() {
    std::string choices;
    E previous = E();
    bool first = true;
    for (const auto& entry : names(E())) {
        if (!first && entry.first == previous) continue;   // skip aliases
        choices += (first ? "" : ", ") + std::string(entry.second);
        previous = entry.first;
        first = false;
    }
    return choices;
}

// A set of flags of enum E, written "A,B"; "NONE" is the empty set.
template <typename E>
std::string flags_name(int flags) {
    std::string text;
    for (const auto& entry : names(E()))
        if (flags & entry.first) text += (text.empty() ? "" : ",") + std::string(entry.second);
    return text.empty() ? "NONE" : text;
}

template <typename E>
E parse_enum(const std::string& option, const std::string& text);

template <typename E>
int parse_flags(const std::string& option, const std::string& text) {
    if (text == "NONE") return 0;
    int flags = 0;
    std::string::size_type start = 0;
    while (start <= text.size()) {
        std::string::size_type end = text.find(',', start);
        if (end == std::string::npos) end = text.size();
        flags |= parse_enum<E>(option, text.substr(start, end - start));
        start = end + 1;
    }
    return flags;
}

template <typename E>
E parse_enum(const std::string& option, const std::string& text) {
    for (const auto& entry : names(E()))
        if (text == entry.second) return entry.first;
    throw std::invalid_argument(option + " must be one of: " + enum_choices<E>());
}
