#!/usr/bin/env python3
"""
Revert the TA_HYBRID 40/500 deadlock fix in src/simulation.cpp.

Context
-------
The reimplementation deadlocked (infinite hang) on agents=40, freq=500 with
algorithm TA_HYBRID: after ~451/500 delivered tasks it printed
"Group1 delivery planning failed for agent 38" and hung forever.

Root cause: the fixed-priority Group1/GoHome path search (astar_with_dummy) is
weaker than the reference's ICBS and genuinely fails to route an agent under
heavy congestion. On failure the reimpl returned false / errored but the caller
ignored it, leaving the agent stuck as an immovable blocker, which then spun the
downstream Group2 dummy-replan and GoHome-retry loops forever.

The fix (4 edits) added a relaxed-constraint retry fallback (Group1 delivery and
GoHome) plus safety caps on the two Group2 retry loops. Result: terminates in
~3.35s with makespan 284 / SWT 72852 (reference 280 / 72857).

This script UNDOES those 4 edits, restoring the original (deadlocking) code.

Usage
-----
    python3 revert_ta_hybrid_deadlock_fix.py            # revert
    python3 revert_ta_hybrid_deadlock_fix.py --check    # verify fix is present, no changes

After reverting, rebuild with `make`. (You can also just restore the saved
fixed copy: `cp src/simulation.cpp.fixed.bak src/simulation.cpp`, or use git:
`git checkout -- src/simulation.cpp`.)
"""
import sys, os

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "src", "simulation.cpp")

# Each entry: (current_fixed_text, original_pre_fix_text)
REPLACEMENTS = [
    # --- Edit 1: GoHome relaxed-constraint fallback ---
    (
"""        if (result < 0) {
            // Fixed-priority GoHome failed under congestion. The reference's ICBS
            // never fails here; our weaker search can. Retry ignoring inter-agent
            // constraints so the agent still gets a valid path home (guaranteed on
            // a connected grid) rather than triggering an unbounded hold_time /
            // GoHome retry loop. Any transient conflict is smoothed by later
            // dummy-replan sweeps and is negligible for makespan/SWT.
            cerr << "GoHome Error! agent " << ags[i]->id
                 << " start=" << start_loc << " park=" << park_loc
                 << " -- retrying with relaxed constraints" << endl;
            vector<vector<int>> relaxed_cons;
            result = astar_with_dummy(*ags[i], start_loc, t,
                                      start_loc, park_loc,
                                      h_goal, h_park_vec,
                                      relaxed_cons, 0);
            if (result < 0)
                return start_loc;
        }""",
"""        if (result < 0) {
            cerr << "GoHome Error! agent " << ags[i]->id
                 << " start=" << start_loc << " park=" << park_loc << endl;
            return start_loc;
        }""",
    ),
    # --- Edit 2: Group1 delivery relaxed-constraint fallback ---
    (
"""            if (result < 0) {
                // The fixed-priority astar_with_dummy used here is strictly weaker
                // than the reference's full ICBS Group1 solver: at high agent
                // density near task exhaustion it can fail to find a delivery path
                // that the reference's reprioritizing ICBS *would* find (ICBS can
                // move the constraint agents out of the way; a fixed-priority
                // search cannot). The reference simply aborts here (ref
                // simulation.cpp line 391, exit(1)); its commented-out "Recovery"
                // block (ref lines 370-383) instead lets the robot proceed on a
                // best-effort path.
                //
                // We follow that recovery intent: retry the search ignoring the
                // inter-agent constraints so the agent still obtains a valid
                // pickup->delivery->parking path and the simulation keeps making
                // progress instead of dead-locking (a stuck delivering agent pins
                // itself on its pickup cell forever and spins the downstream
                // Group2 GoHome / dummy-replan loops). On a connected grid this
                // relaxed search always succeeds. Any residual conflict this
                // introduces is a single agent over a short window and is smoothed
                // out by the dummy-replan sweeps on subsequent timesteps; it is
                // negligible for makespan/SWT.
                cerr << "Group1 delivery planning failed for agent " << ag->id
                     << " -- retrying with relaxed constraints" << endl;
                vector<vector<int>> relaxed_cons;
                result = astar_with_dummy(*ag, start_loc, (int)hybrid_timestep_,
                                          goal_loc, park_loc,
                                          h_goal, h_park_vec,
                                          relaxed_cons, 0);
                if (result < 0) {
                    // Should not happen on a connected map. Un-assign the
                    // not-yet-planned delivery agents so they retry later, and
                    // bail out of this planning call.
                    cerr << "Group1 delivery planning still failed for agent "
                         << ag->id << " even with relaxed constraints; reverting"
                         << endl;
                    for (int k = i; k < (int)delivery_agents.size(); k++) {
                        Agent* a = delivery_agents[k];
                        a->delivering = false;
                        if (a->task_ptr) {
                            a->task_ptr->delivering = false;
                            a->task_ptr->ag = nullptr;
                        }
                        a->task_ptr = nullptr;
                    }
                    return false;
                }
            }""",
"""            if (result < 0) {
                cerr << "Group1 delivery planning failed for agent " << ag->id << endl;
                return false;
            }""",
    ),
    # --- Edits 3 & 4: Group2 replan-loop cap + GoHome-loop cap (adjacent) ---
    (
"""            // Inner loop: solve cost flow, then GoHome, handle conflicts
            // Safety cap matching the reference replan sweep cap (ref
            // simulation.cpp line 964, "replan_pass > 200"). Without this, if a
            // dummy replan can never succeed (e.g. an agent left stuck by a
            // failed Group1 delivery blocks a corridor), this loop spins forever.
            int replan_pass = 0;
            while (true) {
                if (++replan_pass > 200) {
                    cerr << "TA-Hybrid: Group2 replan loop exceeded 200 passes, "
                            "breaking with current paths" << endl;
                    break;
                }
                int gohome_pass = 0;
                while (true) {
                    // Safety cap for the GoHome retry loop. The reference relies on
                    // its full ICBS planner (which never fails) so this loop always
                    // converges; the fixed-priority astar_with_dummy used here can
                    // fail to route an agent home under heavy congestion, in which
                    // case hold_time cannot advance and the loop spins forever.
                    // Cap it: the affected agent simply holds at its current cell
                    // for this timestep and retries on the next one, after other
                    // agents have moved and freed space.
                    if (++gohome_pass > 200) {
                        cerr << "TA-Hybrid: GoHome retry loop exceeded 200 passes, "
                                "agent(s) will wait this timestep" << endl;
                        break;
                    }
                    // Build constraint paths from non-costflow agents
                    vector<vector<int>> cons_paths;""",
"""            // Inner loop: solve cost flow, then GoHome, handle conflicts
            while (true) {
                while (true) {
                    // Build constraint paths from non-costflow agents
                    vector<vector<int>> cons_paths;""",
    ),
]


def main():
    check_only = "--check" in sys.argv
    with open(SRC, "r") as f:
        text = f.read()

    missing = []
    for i, (fixed, _orig) in enumerate(REPLACEMENTS, 1):
        if fixed not in text:
            missing.append(i)
    if missing:
        print(f"ERROR: fixed block(s) {missing} not found in {SRC}.")
        print("The file may already be reverted or modified further; aborting.")
        sys.exit(1)

    if check_only:
        print("OK: all 4 fix edits are present in src/simulation.cpp.")
        return

    for fixed, orig in REPLACEMENTS:
        text = text.replace(fixed, orig, 1)

    with open(SRC, "w") as f:
        f.write(text)
    print("Reverted the TA_HYBRID deadlock fix (4 edits) in src/simulation.cpp.")
    print("Rebuild with `make`. NOTE: this restores the original DEADLOCKING behavior.")


if __name__ == "__main__":
    main()
