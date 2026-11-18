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
Multi-goal tasks (--task_type MG_MAPD) run every configuration with every
--mapf and --single_agent value on the kiva map's five-goal tasks
(tests/multigoal-5.task), and with every single-agent search, endpoint
strategy and dummy-path setting on the structured multi-goal benchmark with
10 agents.
Further jobs exercise what the README promises beyond those axes: TP, TPTS
and HBH planning as a separate step with every trigger, the tour assignments
with every planner, every endpoint strategy and dummy-path setting with every
planner as a separate step (MAPD and multi-goal), CBS with known future tasks,
multi-goal tasks in every mode and agent selection and with 50 agents, random
tie-breaking, the PERIODIC/wPBS window and CENTRAL-fixed (also with the paper's
T' test). Those combinations
are supported by the README, so a rejection fails the check.

A run passes when it

  * is rejected before the simulation starts (exit 1, "is not supported" /
    "requires", no result lines), unless the README says it is supported, or
  * exits 0 after completing every task, passing the collision and task-order
    checks, and its executed paths (--save_output) pass an independent check:
    moves, vertex and swap collisions, every goal of every task visited in
    order by its carrier from the pickup to the completion, no agent carrying
    two tasks at once, and the reported makespan and waiting time.

A run that stops at its own runtime limit is listed as slow; one still running
30 s past it is hung, and that, like anything else (crash, stall, incomplete
run, collision, wrong path), fails the check.

Usage: scripts/check_combinations.py [--executable ./mapd] [--max-parallel 5]
"""
from __future__ import annotations

import argparse
import concurrent.futures
import re
import shlex
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data" / "Instances" / "small"
BENCHMARK = ROOT / "benchmark_instances"
# Multi-goal instances, named in place of a kiva task frequency.
MULTIGOAL = ("mg5", "ssmg")

# CENTRAL-fixed runs the same components as CENTRAL; the thesis's fixed
# matching (CENTRAL_HUNGARIAN_FIXED) is crossed with the planners below.
# CENTRAL-ma is CENTRAL with the paper's every-timestep trigger and T' test.
CONFIGS = ("table1/TP", "table1/TPTS", "table1/HBH", "table1/CENTRAL", "variants/CENTRAL-ma",
           "variants/Hungarian-PBS", "variants/Hungarian-wPBS",
           "table1/LNS-PBS", "table1/LNS-wPBS", "table1/TA-Prioritized",
           "table1/TA-Hybrid", "variants/LNS-real-PP-MLSIPP")
# LNS judged on real paths plans its candidates: several times slower than
# Hungarian + PP, so the check stops it early.
REAL_PATH_LNS = "variants/LNS-real-PP-MLSIPP"
OFFLINE_ONLY = {"table1/TA-Prioritized", "table1/TA-Hybrid"}
# Assignments that plan as published (WHEN_ASSIGNED), one agent at a time.
PLAN_IN_ASSIGNMENT = ("table1/TP", "table1/TPTS", "table1/HBH")
MAPF = (None, "PP", "PP_TASK_SEQUENCE", "CBS", "PBS", "wPBS")
SINGLE_AGENT = (None, "STA", "MLA", "MLSIPP", "SIPP2", "SEQ_STA")
ENDPOINTS = ("WAIT_OR_NEAREST_SAFE", "WAIT_OR_NEAREST_FREE_NONTASK",
             "NEAREST_AVAILABLE", "NEAREST_WITH_STRICT_EXCLUSIONS",
             "PAIRWISE_TASK_THEN_HOME", "RETURN_TO_HOME")
TRIGGERS = ("ON_FREE_WAITS", "EVERY_TIMESTEP",
            "ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE", "ON_AGENT_FREE_OR_AT_PICKUP",
            "ONCE")
SELECTIONS = ("ALL", "CHANGED", "CHANGED_OR_NOT_CARRYING")
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
CENTRAL = ("table1/CENTRAL", "variants/CENTRAL-ma")
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
# Jobs whose combination the README calls supported: a rejection fails.
MUST_RUN: set[tuple] = set()


def combinations() -> list[tuple[str, int, str, list[str]]]:
    """(configuration, agents, task frequency, overriding options) of every job."""
    jobs = []
    for config in CONFIGS:
        # The offline TA configurations run the task set of their LKH3 tours.
        base = "500" if config in OFFLINE_ONLY else "1"
        for mapf in MAPF:
            for sa in SINGLE_AGENT:
                jobs.append((config, 10, "mg5", (["--mapf", mapf] if mapf else []) +
                                                (["--single_agent", sa] if sa else [])))
        jobs += [(config, 10, "ssmg", ["--single_agent", sa]) for sa in SINGLE_AGENT[1:]]
        jobs += [(config, 10, "ssmg", ["--endpoint_strategy", e, "--dummy_path", d])
                 for e in ENDPOINTS for d in ("0", "1")]
        for mapf in MAPF:
            for sa in SINGLE_AGENT:
                jobs.append((config, 10, base, (["--mapf", mapf] if mapf else []) +
                                               (["--single_agent", sa] if sa else [])))
        jobs += [(config, 10, base, ["--assign_trigger", t]) for t in TRIGGERS]
        # Releases far in the future (kiva-0.2) exercise the modes.
        jobs += [(config, 10, "500" if config in OFFLINE_ONLY else "0.2", ["--mode", m])
                 for m in MODES]
        jobs += [(config, 10, base, ["--replan_on", r]) for r in REPLANS]
        jobs += [(config, 10, base, ["--agent_selection", a]) for a in SELECTIONS]
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
                    jobs.append((config, agents, base,
                                 setup + ["--assign_method", "CENTRAL_HUNGARIAN_FIXED"]))
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
    jobs += readme_claims()
    unique = {(c, a, f, tuple(e)): (c, a, f, e) for c, a, f, e in jobs}
    return list(unique.values())


def readme_claims() -> list[tuple[str, int, str, list[str]]]:
    """Combinations the README's quick reference calls supported that the
    axes above do not run; each must run (MUST_RUN)."""
    claims = []
    on_event = "ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE"
    pbs = ["--mapf", "PBS", "--replan_on", "AFTER_ASSIGNMENT",
           "--endpoint_strategy", "NEAREST_WITH_STRICT_EXCLUSIONS", "--agent_selection", "CHANGED"]
    # "Every trigger with every assignment that plans as a separate step."
    for config in PLAN_IN_ASSIGNMENT:
        for trigger in ("ON_FREE_WAITS", "EVERY_TIMESTEP", "ON_AGENT_FREE_OR_AT_PICKUP"):
            claims.append((config, 10, "1", pbs[:2] + ["--assign_trigger", trigger] + pbs[2:]))
    # The tour assignments feed every planner (LKH3_TSP_REASSIGN "as LKH3_TSP").
    for config in ("table1/TA-Prioritized", "table1/TA-Hybrid"):
        claims.append((config, 10, "500", ["--mapf", "PP_TASK_SEQUENCE", "--assign_trigger", on_event,
                                           "--replan_on", "AFTER_ASSIGNMENT", "--agent_selection", "ALL"]))
        for mapf in ("PP", "PBS", "wPBS", "CBS"):
            claims.append((config, 10, "500", ["--mapf", mapf, "--assign_trigger", on_event,
                                               "--replan_on", REFRESH, "--agent_selection", "CHANGED"]))
    # "Every strategy, with either --dummy_path, for every planner": each
    # planner as a separate step after repeated Hungarian, MAPD and multi-goal.
    for frequency in ("1", "ssmg"):
        for mapf, replan, _ in PLANNER_SETUPS:
            for e in ENDPOINTS:
                for d in ("0", "1"):
                    claims.append(("variants/Hungarian-PBS", 10, frequency,
                                   ["--mapf", mapf, "--replan_on", replan, "--agent_selection",
                                    "CHANGED", "--endpoint_strategy", e, "--dummy_path", d]))
    # "Every planner runs in every mode the assignment allows": CBS too.
    setups = {mapf: replan for mapf, replan, _ in PLANNER_SETUPS}
    for mode in FUTURE_MODES:
        claims.append(("variants/Hungarian-PBS", 10, "0.2",
                       ["--mapf", "CBS", "--replan_on", setups["CBS"], "--agent_selection",
                        "CHANGED", *mode]))
    # Multi-goal tasks in every mode and agent selection, and with 50 agents.
    for config in CONFIGS:
        if config not in OFFLINE_ONLY:
            claims += [(config, 10, "ssmg", ["--mode", m]) for m in ("SEMI_ONLINE", "OFFLINE")]
        # TP/TPTS/HBH plan inside the assignment as published: other selections plan after it.
        after = ["--replan_on", "AFTER_ASSIGNMENT"] if config in PLAN_IN_ASSIGNMENT else []
        # LNS on real paths plans every agent of each candidate (ALL only).
        claims += [(config, 10, "ssmg", (after if a != "CHANGED" else []) + ["--agent_selection", a])
                   for a in (("ALL",) if config == REAL_PATH_LNS else SELECTIONS)]
        claims.append((config, 50, "ssmg", []))
        # Random tie-breaking takes other code paths (CBS repeat check, ECBS port).
        claims.append((config, 10, "500" if config in OFFLINE_ONLY else "1",
                       ["--tie_breaking", "random"]))
    # The window: PERIODIC with another interval, wPBS with its own.
    claims.append(("variants/Hungarian-PBS", 10, "1", ["--replan_on", REFRESH,
                                                       "--wpbs_replan_window", "25"]))
    claims.append(("variants/Hungarian-wPBS", 10, "1", ["--wpbs_replan_window", "2"]))
    # CENTRAL-fixed as configured.
    claims += [("table1/CENTRAL-fixed", a, f, []) for a, f in ((10, "1"), (50, "1"), (10, "ssmg"))]
    # The paper's T' test with the thesis's fixed matching too.
    claims += [("table1/CENTRAL-fixed", a, f, ["--central_check_all_goals", "1"])
               for a, f in ((10, "1"), (50, "1"), (10, "ssmg"))]
    # LNS on real paths with both prioritized planners, multi-goal and in every mode.
    claims += [(REAL_PATH_LNS, a, f, e) for a, f, e in (
        (10, "1", []), (50, "1", []), (10, "ssmg", []),
        (10, "1", ["--mapf", "PP_TASK_SEQUENCE"]), (10, "1", ["--single_agent", "MLA"]),
        (10, "0.2", ["--mode", "SEMI_ONLINE"]), (10, "0.2", ["--mode", "OFFLINE"]),
        (10, "1", ["--lns_time", "0"]),
        (10, "1", ["--real_lns_warm_start", "0", "--real_lns_destroy_size", "2"]),
        (10, "1", ["--real_lns_destroy_size", "2", "--real_lns_destroy_planned", "0"]),
        (10, "1", ["--hungarian_pickup_cost", "1"]),
        (10, "ssmg", ["--real_lns_destroy_size", "3", "--real_lns_warm_start", "1"]),
        (50, "1", ["--real_lns_warm_start", "1", "--real_lns_destroy_size", "1"]))]
    # Repeated Hungarian costed at the pickup; MLA*'s node cap scaled both ways.
    claims += [(c, a, f, ["--hungarian_pickup_cost", "1"])
               for c in ("variants/Hungarian-PBS", "variants/Hungarian-wPBS-MLSIPP", "table1/LNS-wPBS")
               for a, f in ((10, "1"), (50, "1"), (10, "ssmg"))]
    claims += [(c, a, f, ["--mla_node_cap_scale", x])
               for c in ("table1/HBH", "variants/Hungarian-PBS") for x in ("0.1", "10")
               for a, f in ((10, "1"), (50, "1"), (10, "ssmg"))]
    MUST_RUN.update((c, a, f, tuple(e)) for c, a, f, e in claims)
    return claims


def instance(config: str, agents: int, frequency: str) -> list[str]:
    """The map, task and (offline TA) tour options of a job."""
    offline = config in OFFLINE_ONLY
    if frequency == "mg5":
        return ["-m", str(DATA / f"kiva-{agents}-500-5.map"),
                "-t", str(ROOT / "tests" / "multigoal-5.task"), "--task_type", "MG_MAPD",
                *(["--tour", str(ROOT / "tests" / "multigoal-10.tour")] if offline else [])]
    if frequency == "ssmg":
        name = f"benchmark_structured_small_mg_a{agents}_f{'all' if offline else '1'}"
        return ["-m", str(BENCHMARK / "maps" / f"benchmark_structured_small_a{agents}.map"),
                "-t", str(BENCHMARK / "tasks" / f"{name}.task"), "--task_type", "MG_MAPD",
                *(["--tour", str(BENCHMARK / "lkh_tours" / f"{name}.tour")] if offline else [])]
    return ["-m", str(DATA / f"kiva-{agents}-500-5.map"),
            "-t", str(DATA / f"kiva-{frequency}.task"),
            *(["--tour", str(ROOT / "tour" / f"{agents}-500.tour")] if offline else [])]


def run(executable: Path, config: str, agents: int, frequency: str, extra: list[str],
        timeout: int) -> str:
    with tempfile.TemporaryDirectory(prefix="combo-") as workdir:
        return run_in(Path(workdir), executable, config, agents, frequency, extra, timeout)


def run_in(workdir: Path, executable: Path, config: str, agents: int, frequency: str,
           extra: list[str], timeout: int) -> str:
    files = instance(config, agents, frequency)
    # 500 structured multi-goal tasks: LNS's up to 1 s per assignment call
    # alone outlasts the time limit, so the check stops it early.
    budget = (["--lns_no_improvement_limit", "50"]
              if frequency == "ssmg" or config == REAL_PATH_LNS else [])
    command = [str(executable), *files, *budget,
               "--config", str(ROOT / "configs" / f"{config}.cfg"), "--seed", "0", "-s", "1",
               "--runtime_limit", str(timeout), "--pathfinding_runtime_limit", str(timeout),
               "--save_output", *extra]
    # The wall-clock limit and the bounded, de-duplicated log keep a broken
    # combination from hanging the check or flooding the disk.
    shell = (f"timeout {timeout + 30} {shlex.join(command)} 2>&1 | uniq | head -c {LOG_CAP}; "
             "exit ${PIPESTATUS[0]}")
    completed = subprocess.run(["bash", "-c", shell], capture_output=True, cwd=workdir,
                               text=True, errors="replace")
    code, output = completed.returncode, completed.stdout
    if code == 124:
        # The program checks its runtime limit; 30 s past it, it is stuck.
        return "FAIL hung past --runtime_limit"
    rejected = re.search(r"^Error: .*(is not supported|requires|is not implemented)", output, re.M)
    started = re.search(r"Tasks completed|Finishing Timestep|CHECK (PASSED|FAILED)", output)
    if rejected and code == 1 and not started:
        if (config, agents, frequency, tuple(extra)) in MUST_RUN:
            return "FAIL rejected, but the README supports it: " + rejected[0][7:80]
        return "rejected"
    done = re.search(r"Tasks completed:\s*(\d+)/(\d+)", output)
    if (code == 0 and done and done[1] == done[2] and "COLLISION CHECK PASSED" in output
            and "TASK ORDER CHECK PASSED" in output):
        saved = list((workdir / "output").glob("*.txt"))
        problem = (verify_paths(files[1], files[3], saved[0], "MG_MAPD" in files)
                   if len(saved) == 1 else f"{len(saved)} --save_output files")
        return "FAIL paths: " + problem[:80] if problem else "ok"
    error = re.findall(r"^Error: (.*)$", output, re.M)
    # Running out of time is slowness, not a wrong result.
    if error and error[-1].startswith("System runtime timeout"):
        return "SLOW " + error[-1][:80]
    return "FAIL " + (error[-1][:80] if error else f"exit {code}")


def verify_paths(map_file: str, task_file: str, saved: Path, multigoal: bool) -> str:
    """Independent check of the executed paths in a --save_output file; returns
    the first problem, or "" when there is none."""
    lines = open(map_file).read().split("\n")
    rows, cols = (int(x) + 2 for x in lines[0].split(","))
    free = [False] * (rows * cols)
    task_eps, homes = [], []
    for i in range(1, rows - 1):
        line = lines[3 + i] if 3 + i < len(lines) else ""
        for j in range(1, cols - 1):
            ch = line[j - 1] if j - 1 < len(line) else "."
            free[i * cols + j] = ch != "@"
            (task_eps if ch == "e" else homes if ch == "r" else []).append(i * cols + j)
    endpoints = task_eps + homes
    task_lines = open(task_file).read().split("\n")
    goals_of = []
    for line in task_lines[1:1 + int(task_lines[0])]:
        r = [int(x) for x in line.split()]
        goals_of.append((r[0], [endpoints[e] for e in (r[1:] if multigoal else r[1:3])]))
    paths, records, header, section = {}, [], {}, None
    for line in open(saved):
        line = line.rstrip("\n")
        m = re.match(r"# (Makespan|Sum of Task Waiting Time): (\d+)", line)
        if m:
            header[m[1]] = int(m[2])
        if line.startswith("# Agent paths") or line.startswith("# Tasks ("):
            section = line[2]
        elif line and not line.startswith("#") and section == "A":
            agent, rest = line.split(":", 1)
            paths[int(agent)] = [int(x) for x in rest.split()]
        elif line and not line.startswith("#") and section == "T":
            records.append([int(x) for x in line.split()])
    if sorted(paths) != list(range(len(homes))):
        return f"{len(paths)} paths for {len(homes)} agents"
    horizon = max(len(p) for p in paths.values())
    P = [paths[i] + [paths[i][-1]] * (horizon - len(paths[i])) for i in range(len(homes))]
    for i, path in enumerate(P):
        if path[0] != homes[i]:
            return f"agent {i} does not start at its home"
        for t in range(1, horizon):
            step = path[t] - path[t - 1]
            if (not free[path[t]] or step not in (0, 1, -1, cols, -cols) or
                    (abs(step) == 1 and path[t] // cols != path[t - 1] // cols)):
                return f"agent {i} moves {path[t - 1]}->{path[t]} at t={t}"
    for t in range(horizon):
        at = {}
        for i in range(len(P)):
            if P[i][t] in at:
                return f"agents {at[P[i][t]]} and {i} collide at {P[i][t]} t={t}"
            at[P[i][t]] = i
        if t:
            moves = {(P[i][t - 1], P[i][t]) for i in range(len(P)) if P[i][t] != P[i][t - 1]}
            if any((b, a) in moves for a, b in moves):
                return f"swap collision at t={t}"
    if len(records) != len(goals_of):
        return f"{len(records)} task records for {len(goals_of)} tasks"
    carrying, makespan, waiting = {}, 0, 0
    for tid, release, _, _, _, done, picked, carrier, *_ in records:
        released, goals = goals_of[tid]
        if release != released or done < 0 or picked < release or not 0 <= carrier < len(P):
            return f"task {tid}: release {release}/{released}, pickup {picked}, done {done}"
        path, t = P[carrier], picked
        if path[t] != goals[0]:
            return f"task {tid}: carrier {carrier} not at the pickup at t={picked}"
        for goal in goals[1:]:
            while t <= done and path[t] != goal:
                t += 1
            if t > done:
                return f"task {tid}: carrier {carrier} misses goal {goal} by t={done}"
        if path[done] != goals[-1]:
            return f"task {tid}: carrier {carrier} not at the last goal at t={done}"
        carrying.setdefault(carrier, []).append((picked, done))
        makespan, waiting = max(makespan, done), waiting + done - release
    for intervals in carrying.values():
        intervals.sort()
        if any(b[0] < a[1] for a, b in zip(intervals, intervals[1:])):
            return "an agent carries two tasks at once"
    if (header.get("Makespan", makespan), header.get("Sum of Task Waiting Time", waiting)) != \
            (makespan, waiting):
        return f"reported makespan/waiting {header} != {makespan}/{waiting}"
    return ""


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
    slow = [(c, a, f, " ".join(e), r)
            for (c, a, f, e), r in zip(jobs, results) if r.startswith("SLOW")]
    for config in dict.fromkeys(c for c, _, _, _ in jobs):
        rows = [r for (c, _, _, _), r in zip(jobs, results) if c == config]
        print(f"{config:24s} ok {rows.count('ok'):3d}  rejected {rows.count('rejected'):3d}  "
              f"slow {sum(r.startswith('SLOW') for r in rows):3d}  "
              f"failed {sum(r.startswith('FAIL') for r in rows):3d}")
    for title, listed in (("Slow (out of time, no wrong result):", slow), ("Failed:", failures)):
        if listed:
            print(title)
        for config, agents, frequency, extra, result in listed:
            print(f"  {config} a{agents} f{frequency} {extra}: {result}")
    print(f"{len(jobs)} combinations, {len(slow)} slow, {len(failures)} failures")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
