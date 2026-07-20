#pragma once
// ============================================================================
// ref_solve.h — Env-gated adapter to the VERBATIM reference (MGMAPD/LNS-wPBS)
// integrated windowed solve: StateTimeAStar + ReservationTable + PBS high-level.
//
// This exposes a single C++ entry point.  All the reference classes live inside
// namespace refsolve in ref_solve.cpp and are NOT leaked here, so simulation.cpp
// only deals in plain std types.  The DEFAULT reimpl binary never calls this
// (it is reached only when REF_SOLVE is set in the environment).
// ============================================================================
#include <vector>
#include <utility>

// Run the reference integrated windowed PBS solve.
//   rows, cols       : padded grid dimensions (reimpl coordinate system)
//   passable         : size rows*cols, 1 = free cell, 0 = blocked
//   start_locs       : per-agent current grid location (padded coords)
//   goal_seqs        : per-agent ordered list of (goal_loc, release_time ABSOLUTE)
//                      -- the LAST entry is the (already chosen) dispersal endpoint
//   cur_time         : current absolute timestep (start.timestep for every agent;
//                      the reference operates in absolute time and re-bases paths)
//   window           : planning window (== reference plan_window; low-level cutoff
//                      is start+window, conflict window is window)
//   time_limit_ms    : per-solve budget in milliseconds
//   out_paths (out)  : per-agent location sequence indexed from 0 (relative time);
//                      caller places out_paths[i][t] at absolute (cur_time+t).
// Returns true always (falls back to hold-in-place for any agent with no plan).
bool ref_wpbs_solve(int rows, int cols,
                    const std::vector<char>& passable,
                    const std::vector<int>& start_locs,
                    const std::vector<std::vector<std::pair<int,int>>>& goal_seqs,
                    int cur_time, int window, int time_limit_ms,
                    std::vector<std::vector<int>>& out_paths);
