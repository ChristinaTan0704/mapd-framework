#!/usr/bin/env python3
"""Run the reproducible MAPD paper matrix on Linux (maximum five workers)."""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATA = ROOT / "data" / "Instances" / "small"
DEFAULT_AGENTS = (10, 20, 30, 40, 50)
DEFAULT_FREQUENCIES = ("0.2", "0.5", "1", "2", "5", "10", "500")


@dataclass(frozen=True)
class Method:
    """A matrix row: a component configuration (configs/<config>.cfg, every
    component set explicitly) plus optional component overrides."""
    label: str
    config: str
    extra: tuple[str, ...] = ()
    offline_only: bool = False

    @property
    def options(self) -> tuple[str, ...]:
        return ("--config", str(ROOT / "configs" / f"{self.config}.cfg")) + self.extra


MLSIPP = ("--single_agent", "MLSIPP")
TS1 = ("--task_sequence_limit", "1")
# The Multi-Goal MAPD paper's sequence length C = 2 (the configurations keep the
# original code's 1): LNS then keeps its improvements to each agent's second task,
# and Hungarian plans each agent's second task as well.
TS2 = ("--task_sequence_limit", "2")
METHODS = (
    Method("TP-STA*", "table1/TP"),
    Method("TPTS-STA*", "table1/TPTS"),
    Method("CENTRAL-CBS", "table1/CENTRAL"),
    Method("CENTRAL-fixed-CBS", "table1/CENTRAL-fixed"),
    Method("CENTRAL-ma-CBS", "variants/CENTRAL-ma"),
    Method("HBH+MLA*", "table1/HBH"),
    Method("HBH+MLSIPP", "table1/HBH", MLSIPP),
    Method("HBH+2-segment SIPP", "table1/HBH", ("--single_agent", "SIPP2")),
    Method("TA-Hybrid-STA*", "table1/TA-Hybrid", ("--tour", "{tour}"), True),
    Method("TA-Prioritized-STA*", "table1/TA-Prioritized", ("--tour", "{tour}"), True),
    Method("Hungarian+PBS-MLA*", "variants/Hungarian-PBS"),
    Method("Hungarian+wPBS-MLA*", "variants/Hungarian-wPBS"),
    Method("LNS(1s)+PBS-MLA*", "table1/LNS-PBS"),
    Method("LNS(1s)+wPBS-MLA*", "table1/LNS-wPBS"),
    Method("Hungarian+PBS-MLSIPP", "variants/Hungarian-PBS-MLSIPP"),
    Method("Hungarian+wPBS-MLSIPP", "variants/Hungarian-wPBS-MLSIPP"),
    Method("LNS(1s)+PBS-MLSIPP", "variants/LNS-PBS-MLSIPP"),
    Method("LNS(1s)+wPBS-MLSIPP", "variants/LNS-wPBS-MLSIPP"),
    Method("TP-MLSIPP", "table1/TP", MLSIPP),
    Method("TPTS-MLSIPP", "variants/TPTS-MLSIPP"),
    Method("Hungarian+PP-SIPP", "variants/Hungarian-PBS-MLSIPP", ("--mapf", "PP")),
    Method("LNS(1s)+PP-SIPP", "variants/LNS-PBS-MLSIPP", ("--mapf", "PP")),
    Method("Hungarian+PBS-MLA* (ts 1)", "variants/Hungarian-PBS", TS1),
    Method("Hungarian+wPBS-MLA* (ts 1)", "variants/Hungarian-wPBS", TS1),
    Method("LNS(1s)+PBS-MLA* (ts 1)", "table1/LNS-PBS", TS1),
    Method("LNS(1s)+wPBS-MLA* (ts 1)", "table1/LNS-wPBS", TS1),
    Method("Hungarian+PBS-MLSIPP (ts 1)", "variants/Hungarian-PBS-MLSIPP", TS1),
    Method("Hungarian+wPBS-MLSIPP (ts 1)", "variants/Hungarian-wPBS-MLSIPP", TS1),
    Method("LNS(1s)+PBS-MLSIPP (ts 1)", "variants/LNS-PBS-MLSIPP", TS1),
    Method("LNS(1s)+wPBS-MLSIPP (ts 1)", "variants/LNS-wPBS-MLSIPP", TS1),
    Method("LNS(1s)+PBS-MLA* (ts 2)", "table1/LNS-PBS", TS2),
    Method("LNS(1s)+wPBS-MLA* (ts 2)", "table1/LNS-wPBS", TS2),
    Method("LNS(1s)+PBS-MLSIPP (ts 2)", "variants/LNS-PBS-MLSIPP", TS2),
    Method("LNS(1s)+wPBS-MLSIPP (ts 2)", "variants/LNS-wPBS-MLSIPP", TS2),
    Method("Hungarian+PBS-MLA* (ts 2)", "variants/Hungarian-PBS", TS2),
    Method("Hungarian+wPBS-MLA* (ts 2)", "variants/Hungarian-wPBS", TS2),
    Method("Hungarian+PBS-MLSIPP (ts 2)", "variants/Hungarian-PBS-MLSIPP", TS2),
    Method("Hungarian+wPBS-MLSIPP (ts 2)", "variants/Hungarian-wPBS-MLSIPP", TS2),
)
BASE_METHOD_COUNT = 21

# Follow-up experiments (11-18), selectable by label but not part of "all":
#   [LAk]          semi-online, the next k release batches known in advance (MG paper Table VI)
#   [pickup cost]  repeated Hungarian costs a task by the arrival at its pickup, not its last goal
#   [cap xS]       MLA*'s per-search node cap multiplied by S
#   LNS-real       REPEATED_HUNGARIAN_REAL_PATH_LNS: LNS whose candidates are planned (PP, MLSIPP)
def _lookahead(label, config, extra, k):
    return Method(f"{label} [LA{k}]", config,
                  extra + ("--mode", "SEMI_ONLINE", "--semi_online_lookahead_batches", str(k)))


_BY_LABEL = {method.label: method for method in METHODS}
EXPERIMENT_METHODS = tuple(
    _lookahead(label, _BY_LABEL[label].config, _BY_LABEL[label].extra, k)
    for label in ("Hungarian+wPBS-MLA*", "LNS(1s)+wPBS-MLA*", "Hungarian+PBS-MLA*",
                  "LNS(1s)+PBS-MLA*", "LNS(1s)+wPBS-MLA* (ts 2)")
    for k in (1, 5, 10)
) + tuple(
    Method(f"{label} [pickup cost]", _BY_LABEL[label].config,
           _BY_LABEL[label].extra + ("--hungarian_pickup_cost", "1"))
    for label in ("Hungarian+PBS-MLA*", "Hungarian+wPBS-MLA*", "Hungarian+PBS-MLSIPP",
                  "Hungarian+wPBS-MLSIPP", "Hungarian+PP-SIPP")
) + tuple(
    Method(f"HBH+MLA* [cap x{scale}]", "table1/HBH", ("--mla_node_cap_scale", scale))
    for scale in ("0.1", "10")
) + (
    # Repeated Hungarian + LNS judged on the real paths of PP with MLSIPP (11-18).
    Method("LNS-real(1s)+PP-MLSIPP", "variants/LNS-real-PP-MLSIPP"),
)

# Seed-0 results with the default reference tie-breaking (validated on Linux,
# 2026-10-03; CENTRAL rows after the reference ECBS port and the TP/TPTS
# MLSIPP rows after its goal waiting, 2026-10-04).
# Runtime is deliberately excluded because it depends on the server.  LNS rows are reported but are not
# exact-gated: their one-second CPU budget can perform a different number of
# iterations on another processor/compiler.
SMOKE_BASELINE = {
    "TP-STA*": (2532, 19270),
    "TPTS-STA*": (2532, 14666),
    "CENTRAL-CBS": (2516, 14887),
    "CENTRAL-fixed-CBS": (2513, 14423),
    "HBH+MLA*": (2532, 14728),
    "HBH+MLSIPP": (2532, 14618),
    "HBH+2-segment SIPP": (2514, 14659),
    "TA-Hybrid-STA*": (1050, 263910),
    "TA-Prioritized-STA*": (1052, 263485),
    "Hungarian+PBS-MLA*": (2514, 13781),
    "Hungarian+wPBS-MLA*": (2513, 13781),
    "LNS(1s)+PBS-MLA*": (2513, 13553),
    "LNS(1s)+wPBS-MLA*": (2512, 13774),
    "Hungarian+PBS-MLSIPP": (2513, 13668),
    "Hungarian+wPBS-MLSIPP": (2513, 13780),
    "LNS(1s)+PBS-MLSIPP": (2512, 13817),
    "LNS(1s)+wPBS-MLSIPP": (2513, 13677),
    "TP-MLSIPP": (2540, 19336),
    "TPTS-MLSIPP": (2530, 14670),
    "Hungarian+PP-SIPP": (2513, 13863),
    "LNS(1s)+PP-SIPP": (2513, 13746),
    "Hungarian+PBS-MLA* (ts 1)": (2514, 13781),
    "Hungarian+wPBS-MLA* (ts 1)": (2513, 13781),
    "LNS(1s)+PBS-MLA* (ts 1)": (2513, 13553),
    "LNS(1s)+wPBS-MLA* (ts 1)": (2512, 13774),
    "Hungarian+PBS-MLSIPP (ts 1)": (2513, 13668),
    "Hungarian+wPBS-MLSIPP (ts 1)": (2513, 13780),
    "LNS(1s)+PBS-MLSIPP (ts 1)": (2512, 13817),
    "LNS(1s)+wPBS-MLSIPP (ts 1)": (2513, 13677),
    "LNS(1s)+PBS-MLA* (ts 2)": (2512, 13681),
    "LNS(1s)+wPBS-MLA* (ts 2)": (2513, 13637),
    "LNS(1s)+PBS-MLSIPP (ts 2)": (2515, 13759),
    "LNS(1s)+wPBS-MLSIPP (ts 2)": (2514, 13648),
    "Hungarian+PBS-MLA* (ts 2)": (2512, 13684),
    "Hungarian+wPBS-MLA* (ts 2)": (2513, 13762),
    "Hungarian+PBS-MLSIPP (ts 2)": (2512, 13701),
    "Hungarian+wPBS-MLSIPP (ts 2)": (2512, 13652),
}

MULTIGOAL_BASELINE = {
    "TP-STA*": (171, 1066),
    "TPTS-STA*": (161, 1014),
    "CENTRAL-CBS": (168, 1092),
    "CENTRAL-fixed-CBS": (159, 1029),
    "HBH+MLA*": (161, 1014),
    "HBH+MLSIPP": (161, 1014),
    "HBH+2-segment SIPP": (161, 1014),
    "TA-Hybrid-STA*": (123, 814),
    "TA-Prioritized-STA*": (123, 839),
    "Hungarian+PBS-MLA*": (158, 1029),
    "Hungarian+wPBS-MLA*": (162, 1030),
    "LNS(1s)+PBS-MLA*": (158, 1029),
    "LNS(1s)+wPBS-MLA*": (162, 1030),
    "Hungarian+PBS-MLSIPP": (158, 1028),
    "Hungarian+wPBS-MLSIPP": (163, 1031),
    "LNS(1s)+PBS-MLSIPP": (158, 1028),
    "LNS(1s)+wPBS-MLSIPP": (163, 1031),
    "TP-MLSIPP": (161, 1049),
    "TPTS-MLSIPP": (161, 1016),
    "Hungarian+PP-SIPP": (161, 1048),
    "LNS(1s)+PP-SIPP": (161, 1048),
    "Hungarian+PBS-MLA* (ts 1)": (158, 1029),
    "Hungarian+wPBS-MLA* (ts 1)": (162, 1030),
    "LNS(1s)+PBS-MLA* (ts 1)": (158, 1029),
    "LNS(1s)+wPBS-MLA* (ts 1)": (162, 1030),
    "Hungarian+PBS-MLSIPP (ts 1)": (158, 1028),
    "Hungarian+wPBS-MLSIPP (ts 1)": (163, 1031),
    "LNS(1s)+PBS-MLSIPP (ts 1)": (158, 1028),
    "LNS(1s)+wPBS-MLSIPP (ts 1)": (163, 1031),
    "LNS(1s)+PBS-MLA* (ts 2)": (158, 1029),
    "LNS(1s)+wPBS-MLA* (ts 2)": (162, 1030),
    "LNS(1s)+PBS-MLSIPP (ts 2)": (158, 1028),
    "LNS(1s)+wPBS-MLSIPP (ts 2)": (163, 1031),
    "Hungarian+PBS-MLA* (ts 2)": (158, 1029),
    "Hungarian+wPBS-MLA* (ts 2)": (162, 1030),
    "Hungarian+PBS-MLSIPP (ts 2)": (158, 1028),
    "Hungarian+wPBS-MLSIPP (ts 2)": (163, 1031),
}


def parse_list(value: str, cast):
    return tuple(cast(item.strip()) for item in value.split(",") if item.strip())


def safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9]+", "_", value).strip("_")


def resolve_template(template: str | None, fallback: Path,
                     agents: int, frequency: str) -> Path:
    if not template:
        return fallback
    path = Path(template.format(agents=agents, frequency=frequency))
    return path if path.is_absolute() else ROOT / path


def metric(pattern: str, output: str, cast):
    match = re.search(pattern, output)
    return cast(match.group(1)) if match else None


SLOT_CGROUPS = Path("/sys/fs/cgroup/mapd-bench")


def slot_oom_kills():
    """{slot: oom_kill count of its cgroup} for the benchmark slots (empty outside them)."""
    counts = {}
    for events in SLOT_CGROUPS.glob("slot*/memory.events"):
        try:
            text = events.read_text()
        except OSError:
            continue
        match = re.search(r"^oom_kill (\d+)", text, re.M)
        if match:
            counts[int(events.parent.name[4:])] = int(match.group(1))
    return counts


def build_jobs(args, selected_methods):
    jobs = []
    for method in selected_methods:
        if args.multigoal_smoke:
            frequencies = ("multigoal",)
        elif method.offline_only:
            frequencies = (args.offline_frequency,)
        else:
            frequencies = ("0.2",) if args.smoke else args.frequencies
        agents = (10,) if args.smoke or args.multigoal_smoke else args.agents
        for agents_count in agents:
            for frequency in frequencies:
                jobs.append((method, agents_count, frequency))
    return jobs


def run_one(args, output_dir: Path, job):
    method, agents, frequency = job
    stem = f"{safe_name(method.label)}_{agents}_{frequency}"
    if args.endpoint_strategy:
        stem += f"_endpoint_{safe_name(args.endpoint_strategy)}"
    cache_path = output_dir / f"{stem}.json"
    log_path = output_dir / f"{stem}.log"
    if cache_path.exists() and not args.rerun:
        result = json.loads(cache_path.read_text())
        print(f"[CACHED] {method.label} {agents}/{frequency}: {result['status']}",
              flush=True)
        return result

    map_path = resolve_template(
        args.map_template, args.data_dir / f"kiva-{agents}-500-5.map",
        agents, frequency)
    task_template = (args.offline_task_template
                     if method.offline_only and args.offline_task_template
                     else args.task_template)
    task_path = (ROOT / "tests" / "multigoal-5.task"
                 if args.multigoal_smoke else resolve_template(
                     task_template,
                     args.data_dir / f"kiva-{frequency}.task",
                     agents, frequency))
    tour_path = (ROOT / "tests" / "multigoal-10.tour"
                 if args.multigoal_smoke else resolve_template(
                     args.tour_template,
                     args.tour_dir / f"{agents}-500.tour",
                     agents, frequency))
    options = [value.format(tour=str(tour_path)) for value in method.options]
    if args.multigoal_smoke:
        options += ["--task_type", "MG_MAPD"]
    elif args.task_type:
        options += ["--task_type", args.task_type]
    command = [str(args.executable), "-m", str(map_path), "-t", str(task_path),
               *options, "--seed", str(args.seed),
               "--runtime_limit", str(args.runtime_limit),
               "--pathfinding_runtime_limit",
               str(args.pathfinding_runtime_limit), "-s", "1"]
    if args.endpoint_strategy:
        command.extend(("--endpoint_strategy", args.endpoint_strategy))

    oom_before = slot_oom_kills()
    started = time.monotonic()
    timed_out = False
    try:
        completed = subprocess.run(
            command, cwd=ROOT, capture_output=True, text=True,
            timeout=args.timeout)
        output = (completed.stdout or "") + (completed.stderr or "")
        return_code = completed.returncode
    except subprocess.TimeoutExpired as error:
        stdout = error.stdout.decode() if isinstance(error.stdout, bytes) else (error.stdout or "")
        stderr = error.stderr.decode() if isinstance(error.stderr, bytes) else (error.stderr or "")
        output = stdout + stderr + f"\nTIMEOUT after {args.timeout} seconds\n"
        return_code = 124
        timed_out = True
    wall_runtime = time.monotonic() - started
    log_path.write_text(output)

    makespan = metric(r"Finishing Timestep:\s*(\d+)", output, int)
    swt = metric(r"Sum of Task Waiting Time:\s*(\d+)", output, int)
    runtime_ms = metric(r"Total runtime:\s*([0-9.eE+-]+)\s*ms", output, float)
    tasks = re.search(r"Tasks completed:\s*(\d+)/(\d+)", output)
    completed_tasks = int(tasks.group(1)) if tasks else None
    total_tasks = int(tasks.group(2)) if tasks else None
    collision_failed = any(text in output for text in (
        "COLLISION DETECTED", "COLLISION CHECK FAILED", "VERTEX COLLISION",
        "EDGE COLLISION"))
    collision_passed = "COLLISION CHECK PASSED" in output

    # The simulator's own runtime limits end the run with an error (exit 1);
    # a SIGKILL the runner did not send is the memory limit (OOM killer).
    own_limit = re.search(r"(System|Pathfinding) runtime timeout in", output)
    # bench_exec.py's line: which slot ran the job and how long it waited for one
    # (the wall-clock --timeout runs while it waits).
    slot = metric(r"\[bench\] slot (\d+) core", output, int)
    # Since 11-17 bench_exec.py also prints the wait to the millisecond; wall_runtime_s - slot_wait_s is the
    # time the job ran in its slot. Before that only whole seconds were printed.
    slot_wait = metric(r"\[bench\] slot wait ([0-9.]+) s", output, float)
    if slot_wait is None:
        slot_wait = metric(r"\[bench\] slot \d+ core \d+ \(waited (\d+) s\)", output, int)
    slot_oom = slot is not None and slot_oom_kills().get(slot, 0) > oom_before.get(slot, 0)
    if timed_out or (return_code != 0 and own_limit):
        status = "timeout"
    elif collision_failed:   # also exits with status 2
        status = "collision"
    elif return_code == -9 and slot_oom:   # the slot's cgroup counted an OOM kill
        status = "oom"
    elif return_code != 0:
        status = f"error({return_code})"
    elif not collision_passed:
        status = "error(no collision-check result)"
    elif None in (makespan, swt, runtime_ms, completed_tasks, total_tasks):
        status = "error(parse)"
    elif completed_tasks != total_tasks:
        status = "incomplete"
    else:
        status = "ok"

    result = {
        "method": method.label,
        "config": method.config,
        "agents": agents,
        "frequency": frequency,
        "seed": args.seed,
        "runtime_limit_s": args.runtime_limit,
        "pathfinding_runtime_limit_s": args.pathfinding_runtime_limit,
        "makespan": makespan,
        "swt": swt,
        "runtime_s": runtime_ms / 1000.0 if runtime_ms is not None else None,
        "wall_runtime_s": wall_runtime,
        "tasks_completed": completed_tasks,
        "tasks_total": total_tasks,
        "status": status,
        "slot": slot,
        "slot_wait_s": slot_wait,
        "command": command,
        "log": str(log_path),
    }
    # Written whole or not at all: a batch driver may read it while this runner ends.
    partial = cache_path.with_name(cache_path.name + ".tmp")
    partial.write_text(json.dumps(result, indent=2) + "\n")
    partial.replace(cache_path)
    print(f"[DONE] {method.label} {agents}/{frequency}: ms={makespan} "
          f"swt={swt} wall={wall_runtime:.2f}s [{status}]", flush=True)
    return result


def write_summary(output_dir: Path, results):
    def frequency_order(value):
        try:
            return (0, float(value))
        except (TypeError, ValueError):
            return (1, str(value))

    ordered = sorted(results, key=lambda row: (
        row["agents"], frequency_order(row["frequency"]),
        row["method"].lower()))
    (output_dir / "results.json").write_text(json.dumps(ordered, indent=2) + "\n")
    columns = ("method", "config", "agents", "frequency", "seed",
               "runtime_limit_s", "pathfinding_runtime_limit_s",
               "makespan", "swt", "runtime_s", "wall_runtime_s",
               "tasks_completed", "tasks_total", "status", "log")
    with (output_dir / "results.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(ordered)


def validate_smoke(results):
    failures = []
    print("\nSmoke comparison (expected -> observed makespan/SWT):")
    for result in sorted(results, key=lambda row: row["method"].lower()):
        expected = SMOKE_BASELINE[result["method"]]
        observed = (result["makespan"], result["swt"])
        is_lns = result["method"].startswith("LNS(1s)")
        match = observed == expected
        policy = "informational" if is_lns else "exact"
        print(f"  {result['method']}: {expected[0]}/{expected[1]} -> "
              f"{observed[0]}/{observed[1]} "
              f"({'match' if match else 'DIFF'}, {policy})")
        if result["status"] != "ok":
            failures.append(f"{result['method']}: {result['status']}")
        elif not is_lns and not match:
            failures.append(f"{result['method']}: expected {expected}, got {observed}")
    return failures


def validate_multigoal_smoke(results, exact_metrics=True):
    failures = []
    print("\nFive-goal comparison (expected -> observed makespan/SWT):")
    for result in sorted(results, key=lambda row: row["method"].lower()):
        expected = MULTIGOAL_BASELINE[result["method"]]
        observed = (result["makespan"], result["swt"])
        match = observed == expected
        policy = "exact" if exact_metrics else "informational"
        print(f"  {result['method']}: {expected[0]}/{expected[1]} -> "
              f"{observed[0]}/{observed[1]} "
              f"({'match' if match else 'DIFF'}, {policy})")
        if result["status"] != "ok":
            failures.append(f"{result['method']}: {result['status']}")
        elif result["tasks_completed"] != 10 or result["tasks_total"] != 10:
            failures.append(
                f"{result['method']}: expected 10/10 completed tasks, got "
                f"{result['tasks_completed']}/{result['tasks_total']}")
        elif exact_metrics and not match:
            failures.append(
                f"{result['method']}: expected {expected}, got {observed}")
    return failures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--smoke", action="store_true",
                        help="run every method at 10 agents/0.2 (TA at 10/500) and validate")
    parser.add_argument(
        "--multigoal-smoke", action="store_true",
        help="run every method on the committed ten-task, five-goal scenario")
    parser.add_argument("--methods", default="all",
                        help="comma-separated displayed method labels, or all")
    parser.add_argument(
        "--base-methods", action="store_true",
        help="with --methods=all, run the 21 base methods and omit ts-1 variants")
    parser.add_argument("--agents", type=lambda value: parse_list(value, int),
                        default=DEFAULT_AGENTS)
    parser.add_argument("--frequencies", type=lambda value: parse_list(value, str),
                        default=DEFAULT_FREQUENCIES)
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--tour-dir", type=Path, default=ROOT / "tour")
    parser.add_argument(
        "--map-template",
        help="map path template with {agents} and optional {frequency}")
    parser.add_argument(
        "--task-template",
        help="task path template with {agents} and {frequency}")
    parser.add_argument(
        "--offline-task-template",
        help="optional task template used only by offline TA methods")
    parser.add_argument(
        "--tour-template",
        help="offline-tour path template with {agents} and optional {frequency}")
    parser.add_argument(
        "--offline-frequency", default="500",
        help="frequency label used for offline TA jobs (default 500; use all for _fall tasks)")
    parser.add_argument("--executable", type=Path, default=ROOT / "mapd")
    parser.add_argument("--output-dir", type=Path,
                        default=ROOT / "server_results")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--endpoint-strategy",
        choices=("WAIT_OR_NEAREST_SAFE", "RETURN_TO_HOME",
                 "NEAREST_WITH_STRICT_EXCLUSIONS",
                 "PAIRWISE_TASK_THEN_HOME",
                 "WAIT_OR_NEAREST_FREE_NONTASK", "NEAREST_AVAILABLE"),
        help="override the configured endpoint strategy for every selected method")
    parser.add_argument(
        "--task-type", choices=("MAPD", "MG_MAPD"),
        help="override the configured task type (MG_MAPD for multi-goal task files)")
    parser.add_argument("--timeout", type=int, default=1000,
                        help="wall-clock timeout per simulator process (seconds)")
    parser.add_argument(
        "--runtime-limit", type=int,
        help="internal simulator timeout; default is --timeout minus 5 seconds")
    parser.add_argument(
        "--pathfinding-runtime-limit", type=int, default=600,
        help="per planning-cycle simulator timeout in seconds (default 600)")
    parser.add_argument("--max-parallel", type=int, default=5)
    parser.add_argument("--rerun", action="store_true",
                        help="ignore cached per-job JSON files")
    args = parser.parse_args()

    if args.smoke and args.multigoal_smoke:
        parser.error("--smoke and --multigoal-smoke are mutually exclusive")
    if not 1 <= args.max_parallel <= 20:
        parser.error("--max-parallel must be between 1 and 20")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.runtime_limit is None:
        args.runtime_limit = max(1, args.timeout - 5)
    if args.runtime_limit < 0:
        parser.error("--runtime-limit must be non-negative")
    if args.pathfinding_runtime_limit < 0:
        parser.error("--pathfinding-runtime-limit must be non-negative")
    if args.seed < 0:
        parser.error("use a non-negative seed for reproducible server runs")
    if not args.executable.is_file():
        parser.error(f"executable not found: {args.executable}; run make first")

    by_label = {method.label: method for method in METHODS + EXPERIMENT_METHODS}
    if args.methods == "all":
        selected = METHODS[:BASE_METHOD_COUNT] if args.base_methods else METHODS
    else:
        if args.base_methods:
            parser.error("--base-methods cannot be combined with explicit --methods")
        requested = [item.strip() for item in args.methods.split(",")]
        unknown = [item for item in requested if item not in by_label]
        if unknown:
            parser.error("unknown method label(s): " + ", ".join(unknown))
        selected = tuple(by_label[item] for item in requested)

    jobs = build_jobs(args, selected)
    missing = []
    for method, agents, frequency in jobs:
        map_path = resolve_template(
            args.map_template, args.data_dir / f"kiva-{agents}-500-5.map",
            agents, frequency)
        task_template = (args.offline_task_template
                         if method.offline_only and args.offline_task_template
                         else args.task_template)
        task_path = (ROOT / "tests" / "multigoal-5.task"
                     if args.multigoal_smoke else resolve_template(
                         task_template,
                         args.data_dir / f"kiva-{frequency}.task",
                         agents, frequency))
        for path in (map_path, task_path):
            if not path.is_file():
                missing.append(path)
        if method.offline_only:
            path = (ROOT / "tests" / "multigoal-10.tour"
                    if args.multigoal_smoke else resolve_template(
                        args.tour_template,
                        args.tour_dir / f"{agents}-500.tour",
                        agents, frequency))
            if not path.is_file():
                missing.append(path)
    if missing:
        parser.error("missing input files:\n  " + "\n  ".join(map(str, sorted(set(missing)))))

    args.output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Running {len(jobs)} jobs with seed={args.seed}, "
          f"max_parallel={args.max_parallel}, timeout={args.timeout}s")
    results = []
    with concurrent.futures.ThreadPoolExecutor(
            max_workers=args.max_parallel) as executor:
        futures = [executor.submit(run_one, args, args.output_dir, job)
                   for job in jobs]
        for future in concurrent.futures.as_completed(futures):
            results.append(future.result())

    write_summary(args.output_dir, results)
    failures = [f"{row['method']} {row['agents']}/{row['frequency']}: {row['status']}"
                for row in results if row["status"] != "ok"]
    if args.smoke:
        failures = validate_smoke(results)
    elif args.multigoal_smoke:
        failures = validate_multigoal_smoke(
            results, exact_metrics=args.endpoint_strategy is None)

    print(f"\nResults: {args.output_dir / 'results.csv'}")
    print(f"Manifest: {args.output_dir / 'results.json'}")
    if failures:
        print("FAILED:", file=sys.stderr)
        for failure in failures:
            print(f"  {failure}", file=sys.stderr)
        return 1
    print(f"All {len(results)} jobs passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
