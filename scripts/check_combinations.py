#!/usr/bin/env python3
"""Combination check: every component combination must either run correctly
or be rejected by validate_config() before the simulation starts.

Each algorithm preset is crossed with every --mapf and --single_agent override,
and separately with every --endpoint_strategy, --mode, --dummy_path and
--replan_on, on a small instance.
A run passes when it

  * is rejected immediately (exit 1, "is not supported" / "requires"), or
  * completes every task and passes the collision check.

Anything else (crash, timeout, incomplete run, collision) fails the check.

Usage: scripts/check_combinations.py [--executable ./mapd] [--max-parallel 20]
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

PRESETS = ("TP", "TPTS", "HBH_MLA", "CENTRAL-CBS", "CENTRAL-fixed",
           "HUNGARIAN_PBS", "HUNGARIAN_wPBS", "LNS_PBS", "LNS_wPBS",
           "TA_PRIORITIZED", "TA_HYBRID")
OFFLINE_ONLY = {"TA_PRIORITIZED", "TA_HYBRID"}
MAPF = (None, "PP", "PP_TASK_SEQUENCE", "CBS", "PBS", "wPBS")
SINGLE_AGENT = (None, "STA", "MLA", "MLSIPP", "SIPP2")
ENDPOINTS = ("WAIT_OR_NEAREST_SAFE", "WAIT_OR_NEAREST_FREE_NONTASK",
             "NEAREST_AVAILABLE", "NEAREST_WITH_STRICT_EXCLUSIONS",
             "PAIRWISE_TASK_THEN_HOME", "RETURN_TO_HOME")
MODES = ("ONLINE", "SEMI_ONLINE", "OFFLINE")
REPLANS = ("NONE", "AFTER_ASSIGNMENT", "EVERY_TIMESTEP", "AFTER_ASSIGNMENT,GOAL_REACHED",
           "AFTER_ASSIGNMENT,PERIODIC", "AFTER_ASSIGNMENT,GOAL_REACHED,PERIODIC")
LOG_CAP = 2_000_000


def combinations() -> list[tuple[str, list[str]]]:
    jobs = []
    for preset in PRESETS:
        for mapf in MAPF:
            for sa in SINGLE_AGENT:
                jobs.append((preset, (["--mapf", mapf] if mapf else []) +
                                     (["--single_agent", sa] if sa else [])))
        jobs += [(preset, ["--endpoint_strategy", e]) for e in ENDPOINTS]
        jobs += [(preset, ["--mode", m]) for m in MODES]
        jobs += [(preset, ["--dummy_path", d]) for d in ("0", "1")]
        jobs += [(preset, ["--replan_on", r]) for r in REPLANS]
    return jobs


def run(executable: Path, preset: str, extra: list[str], timeout: int) -> str:
    if preset in OFFLINE_ONLY:
        instance = ["-t", str(DATA / "kiva-500.task"), "--tour", str(ROOT / "tour" / "10-500.tour")]
    else:
        instance = ["-t", str(DATA / "kiva-1.task")]
    command = [str(executable), "-m", str(DATA / "kiva-10-500-5.map"), *instance,
               "-a", preset, "--seed", "0", "-s", "1",
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
    if done and done[1] == done[2] and "COLLISION CHECK PASSED" in output:
        return "ok"
    error = re.findall(r"^Error: (.*)$", output, re.M)
    return "FAIL " + (error[-1][:80] if error else f"exit {code}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--executable", type=Path, default=ROOT / "mapd")
    parser.add_argument("--max-parallel", type=int, default=20)
    parser.add_argument("--timeout", type=int, default=300, help="seconds per run")
    args = parser.parse_args()

    jobs = combinations()
    with concurrent.futures.ThreadPoolExecutor(args.max_parallel) as pool:
        results = list(pool.map(
            lambda job: run(args.executable.resolve(), job[0], job[1], args.timeout), jobs))

    failures = [(p, " ".join(e), r) for (p, e), r in zip(jobs, results) if r.startswith("FAIL")]
    for preset in PRESETS:
        rows = [r for (p, _), r in zip(jobs, results) if p == preset]
        print(f"{preset:16s} ok {rows.count('ok'):3d}  rejected {rows.count('rejected'):3d}  "
              f"failed {sum(r.startswith('FAIL') for r in rows):3d}")
    for preset, extra, result in failures:
        print(f"  {preset} {extra}: {result}")
    print(f"{len(jobs)} combinations, {len(failures)} failures")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
