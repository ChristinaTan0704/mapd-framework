#!/usr/bin/env python3
"""Combination check: every component combination must either run correctly
or be rejected by validate_config() before the simulation starts.

Each algorithm configuration (configs/) is crossed with every --mapf and
--single_agent value, and separately with every --assign_trigger,
--endpoint_strategy, --mode, --dummy_path, --replan_on and --agent_selection,
and with each planner set up the way it is used on its own (assignment
trigger, replan events, endpoints), on the kiva map with 10 agents; the
endpoint and planner axes also run with 50 agents, where crowding exposes
deadlocks. CENTRAL is crossed with every planner, single-agent search,
dummy-path setting and mode, and every configuration with CBS and every
single-agent search, at 10 and 50 agents. Every planner also runs with
releases far ahead of the agents (kiva-0.2 with every task known upfront, or
1000 release batches ahead), fed by repeated Hungarian (10 agents), TP's
greedy assignment (50 agents) and the LKH3 tour (10 and 50 agents).
A run passes when it

  * is rejected immediately (exit 1, "is not supported" / "requires"), or
  * completes every task and passes the collision and task-order checks.

Anything else (crash, timeout, incomplete run, collision) fails the check.

Usage: scripts/check_combinations.py [--executable ./mapd] [--max-parallel 5]
"""
from __future__ import annotations

import argparse
import concurrent.futures
import re
import shlex
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data" / "Instances" / "small"

CONFIGS = ("table1/TP", "table1/TPTS", "table1/HBH", "table1/CENTRAL",
           "table1/CENTRAL-fixed", "variants/Hungarian-PBS", "variants/Hungarian-wPBS",
           "table1/LNS-PBS", "table1/LNS-wPBS", "table1/TA-Prioritized",
           "table1/TA-Hybrid")
OFFLINE_ONLY = {"table1/TA-Prioritized", "table1/TA-Hybrid"}
MAPF = (None, "PP", "PP_TASK_SEQUENCE", "CBS", "PBS", "wPBS")
SINGLE_AGENT = (None, "STA", "MLA", "MLSIPP", "SIPP2", "SEQ_STA")
ENDPOINTS = ("WAIT_OR_NEAREST_SAFE", "WAIT_OR_NEAREST_FREE_NONTASK",
             "NEAREST_AVAILABLE", "NEAREST_WITH_STRICT_EXCLUSIONS",
             "PAIRWISE_TASK_THEN_HOME", "RETURN_TO_HOME")
TRIGGERS = ("ON_FREE_WAITS", "EVERY_TIMESTEP",
            "ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE", "ONCE")
MODES = ("ONLINE", "SEMI_ONLINE", "OFFLINE")
REPLANS = ("WHEN_ASSIGNED", "AFTER_ASSIGNMENT", "EVERY_TIMESTEP", "AFTER_ASSIGNMENT,GOAL_REACHED",
           "AFTER_ASSIGNMENT,PERIODIC", "AFTER_ASSIGNMENT,GOAL_REACHED,PERIODIC")
REFRESH = "AFTER_ASSIGNMENT,PERIODIC"
# Each planner with the trigger, replanning events and endpoints it needs, so
# any assignment method can be paired with it as a separate planning step.
PLANNER_SETUPS = (
    ("PBS", "AFTER_ASSIGNMENT", "NEAREST_WITH_STRICT_EXCLUSIONS"),
    ("PP", "AFTER_ASSIGNMENT", "NEAREST_WITH_STRICT_EXCLUSIONS"),
    ("PP_TASK_SEQUENCE", "AFTER_ASSIGNMENT", "NEAREST_WITH_STRICT_EXCLUSIONS"),
    ("wPBS", REFRESH, "PAIRWISE_TASK_THEN_HOME"),
    ("CBS", "AFTER_ASSIGNMENT,GOAL_REACHED", "NEAREST_WITH_STRICT_EXCLUSIONS"),
)
CENTRAL = ("table1/CENTRAL", "table1/CENTRAL-fixed")
# Releases far ahead of the agents, in both modes: the planners fed by
# repeated Hungarian with 10 agents and by TP's greedy batch assignment with 50
# (Hungarian rematching ~500 known tasks at every event outlasts the time
# limit there), and by the (offline) LKH3 tour; a tour planned a few tasks at
# a time is refreshed periodically.
FUTURE_MODES = (("--mode", "OFFLINE"),
                ("--mode", "SEMI_ONLINE", "--semi_online_lookahead_batches", "1000"))
FUTURE_PLANNERS = ("PBS", "PP", "PP_TASK_SEQUENCE", "wPBS")
FUTURE_TOUR = (("PP_TASK_SEQUENCE", "AFTER_ASSIGNMENT"), ("PP", REFRESH), ("PBS", REFRESH),
               ("wPBS", REFRESH))
LOG_CAP = 2_000_000


def combinations() -> list[tuple[str, int, str, list[str]]]:
    """(configuration, agents, task frequency, overriding options) of every job."""
    jobs = []
    for config in CONFIGS:
        # The offline TA configurations run the task set of their LKH3 tours.
        base = "500" if config in OFFLINE_ONLY else "1"
        for mapf in MAPF:
            for sa in SINGLE_AGENT:
                jobs.append((config, 10, base, (["--mapf", mapf] if mapf else []) +
                                               (["--single_agent", sa] if sa else [])))
        jobs += [(config, 10, base, ["--assign_trigger", t]) for t in TRIGGERS]
        # Releases far in the future (kiva-0.2) exercise the modes.
        jobs += [(config, 10, "500" if config in OFFLINE_ONLY else "0.2", ["--mode", m])
                 for m in MODES]
        jobs += [(config, 10, base, ["--replan_on", r]) for r in REPLANS]
        jobs += [(config, 10, base, ["--agent_selection", a]) for a in ("ALL", "CHANGED")]
        # Where agents rest matters most when many agents share few free
        # endpoints (50 agents, every task open at once).
        for agents, frequency in ((10, base), (50, base), (50, "500")):
            jobs += [(config, agents, frequency, ["--endpoint_strategy", e, "--dummy_path", d])
                     for e in ENDPOINTS for d in ("0", "1")]
        for agents in (10, 50):
            for mapf, replan, endpoint in PLANNER_SETUPS:
                for selection in ("ALL", "CHANGED"):
                    # One CBS over every agent's goal sequence reaches its
                    # expansion limit at almost every replan with 50 agents
                    # (then prioritized planning takes over): too slow here.
                    if mapf == "CBS" and selection == "ALL" and agents == 50:
                        continue
                    jobs.append((config, agents, base, [
                        "--mapf", mapf, "--assign_trigger",
                        "ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE",
                        "--replan_on", replan, "--endpoint_strategy", endpoint,
                        "--agent_selection", selection]))
        # CENTRAL with every planner, and every assignment with CBS, in each
        # planner's own setup (wPBS plans every agent; CENTRAL parks agents
        # at the nearest available endpoints).
        for mapf, replan, endpoint in PLANNER_SETUPS:
            if config not in CENTRAL and mapf != "CBS":
                continue
            if config in CENTRAL and mapf != "wPBS":
                # CENTRAL plans an agent to its pickup only, so the delivery
                # leg needs a GOAL_REACHED replan.
                endpoint = "NEAREST_AVAILABLE"
                replan = "AFTER_ASSIGNMENT,GOAL_REACHED"
            setup = ["--mapf", mapf, "--assign_trigger",
                     "ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE", "--replan_on", replan,
                     "--endpoint_strategy", endpoint,
                     "--agent_selection", "ALL" if mapf == "wPBS" else "CHANGED"]
            for agents in (10, 50):
                jobs += [(config, agents, base, setup + ["--single_agent", sa])
                         for sa in SINGLE_AGENT[1:]]
                if config in CENTRAL:
                    jobs += [(config, agents, base, setup + ["--dummy_path", d])
                             for d in ("0", "1")]
            if config in CENTRAL:
                jobs += [(config, 10, "0.2", setup + ["--dummy_path", "1", "--mode", m])
                         for m in MODES]
    setups = {mapf: (replan, endpoint) for mapf, replan, endpoint in PLANNER_SETUPS}
    for mapf in FUTURE_PLANNERS:
        hungarian = "variants/Hungarian-wPBS" if mapf == "wPBS" else "variants/Hungarian-PBS"
        replan, endpoint = setups[mapf]
        greedy = ["--assign_trigger", "ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE",
                  "--replan_on", replan, "--endpoint_strategy", endpoint]
        for mode in FUTURE_MODES:
            for selection in ("ALL", "CHANGED"):
                options = ["--mapf", mapf, *mode, "--agent_selection", selection]
                jobs.append((hungarian, 10, "0.2", options))
                jobs.append(("table1/TP", 50, "0.2", options + greedy))
        for sa in SINGLE_AGENT[1:]:
            jobs.append((hungarian, 10, "0.2", ["--mapf", mapf, "--mode", "OFFLINE",
                                                "--single_agent", sa]))
    for agents in (10, 50):
        for mapf, replan in FUTURE_TOUR:
            jobs.append(("table1/TA-Prioritized", agents, "0.2",
                         ["--mapf", mapf, "--replan_on", replan]))
    unique = {(c, a, f, tuple(e)): (c, a, f, e) for c, a, f, e in jobs}
    return list(unique.values())


def run(executable: Path, config: str, agents: int, frequency: str, extra: list[str],
        timeout: int) -> str:
    instance = ["-t", str(DATA / f"kiva-{frequency}.task")]
    if config in OFFLINE_ONLY:
        instance += ["--tour", str(ROOT / "tour" / f"{agents}-500.tour")]
    command = [str(executable), "-m", str(DATA / f"kiva-{agents}-500-5.map"), *instance,
               "--config", str(ROOT / "configs" / f"{config}.cfg"), "--seed", "0", "-s", "1",
               "--runtime_limit", str(timeout), "--pathfinding_runtime_limit", str(timeout),
               *extra]
    # The wall-clock limit and the bounded, de-duplicated log keep a broken
    # combination from hanging the check or flooding the disk.
    shell = (f"timeout {timeout + 30} {shlex.join(command)} 2>&1 | uniq | head -c {LOG_CAP}; "
             "exit ${PIPESTATUS[0]}")
    completed = subprocess.run(["bash", "-c", shell], capture_output=True,
                               text=True, errors="replace")
    code, output = completed.returncode, completed.stdout
    if code == 124:
        return "FAIL timeout"
    rejected = re.search(r"^Error: .*(is not supported|requires|is not implemented)", output, re.M)
    if rejected and code == 1:
        return "rejected"
    done = re.search(r"Tasks completed:\s*(\d+)/(\d+)", output)
    if (done and done[1] == done[2] and "COLLISION CHECK PASSED" in output
            and "TASK ORDER CHECK PASSED" in output):
        return "ok"
    error = re.findall(r"^Error: (.*)$", output, re.M)
    return "FAIL " + (error[-1][:80] if error else f"exit {code}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--executable", type=Path, default=ROOT / "mapd")
    parser.add_argument("--max-parallel", type=int, default=5)
    parser.add_argument("--timeout", type=int, default=300, help="seconds per run")
    args = parser.parse_args()

    jobs = combinations()
    with concurrent.futures.ThreadPoolExecutor(args.max_parallel) as pool:
        results = list(pool.map(
            lambda job: run(args.executable.resolve(), *job, args.timeout), jobs))

    failures = [(c, a, f, " ".join(e), r)
                for (c, a, f, e), r in zip(jobs, results) if r.startswith("FAIL")]
    for config in CONFIGS:
        rows = [r for (c, _, _, _), r in zip(jobs, results) if c == config]
        print(f"{config:24s} ok {rows.count('ok'):3d}  rejected {rows.count('rejected'):3d}  "
              f"failed {sum(r.startswith('FAIL') for r in rows):3d}")
    for config, agents, frequency, extra, result in failures:
        print(f"  {config} a{agents} f{frequency} {extra}: {result}")
    print(f"{len(jobs)} combinations, {len(failures)} failures")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
