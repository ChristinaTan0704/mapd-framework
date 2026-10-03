#!/usr/bin/env python3
"""Shared logic for the per-map experiment-matrix scripts.

Not meant to be run directly -- see run_structured_small_matrix.py,
run_structured_medium_matrix.py, run_structured_large_matrix.py, and
run_sparse_small_to_medium_matrix.py, one per generated benchmark map family.

Each of those scripts covers all agent counts and all packaged task
frequencies for its map. The 19 online algorithms (27 method-rows once the
eight Hungarian/LNS PBS-or-wPBS MLA*/MLSIPP rows are doubled for
task_sequence_limit 1 and 2) each run once in ONLINE mode and once in
SEMI_ONLINE mode. TA-Prioritized and TA-Hybrid run offline-only, once per
agent count, against the matching packaged LKH tour.

For every job this stores makespan, SWT ("sum of cost" per this framework's
only cost metric), wall/process runtime, and full per-agent paths (via
--save_output) in a per-job directory so no two jobs can overwrite each
other's saved path dump.

Every job also receives the executable's own internal wall-clock deadlines
(--runtime_limit, --pathfinding_runtime_limit) so a hung or exploding search
aborts itself cleanly well before the external subprocess timeout -- the
external timeout is only the final hard-kill fallback, not something that
needs active monitoring.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import re
import resource
import subprocess
import sys
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_server_matrix import METHODS, Method, safe_name, metric  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
BENCH = ROOT / "benchmark_instances"
MAPS_DIR = BENCH / "maps"
TASKS_DIR = BENCH / "tasks"
TOURS_DIR = BENCH / "lkh_tours"

DEFAULT_FREQUENCIES = ("0.2", "0.5", "1", "2", "5", "10", "all")
MODES = ("online", "semi_online")

# Shared default output directory: all four per-map scripts write into the
# same tree by default (job stems encode the map family, so there is no
# collision risk) precisely so cached/completed jobs from one script are
# visible to -- and skipped by -- the others.
DEFAULT_OUTPUT_DIR = ROOT / "server_results" / "full_experiment_matrix"


@dataclass(frozen=True)
class MapFamily:
    name: str          # job-stem / grouping key
    agents: tuple[int, ...]
    timeout_s: int      # per-job system (external + internal) wall-clock limit
    endpoint_ablation: bool = False  # also run forced RETURN_TO_HOME / NEAREST_AVAILABLE sets
    map_name: str = ""  # benchmark_<map_name>_a<agents>.map; defaults to name
    task_name: str = ""  # benchmark_<task_name>_a<agents>_f<freq>.task/.tour; defaults to name
    frequencies: tuple[str, ...] = ()  # defaults to DEFAULT_FREQUENCIES if empty

    def __post_init__(self):
        if not self.map_name:
            object.__setattr__(self, "map_name", self.name)
        if not self.task_name:
            object.__setattr__(self, "task_name", self.name)
        if not self.frequencies:
            object.__setattr__(self, "frequencies", DEFAULT_FREQUENCIES)


# System (external subprocess + internal --runtime_limit) timeout per map
# family: 1000 s for SMALL-scale maps (structured_small and the equivalently
# small-scale sparse map), 2h for MEDIUM, 4h for LARGE.
#
# structured_small and sparse_small_to_medium additionally get the endpoint-
# choice ablation: besides each method's own default endpoint strategy, every
# job also runs once with --endpoint_strategy forced to RETURN_TO_HOME and
# once forced to NEAREST_AVAILABLE (3x the job count for just those two maps).
#
# structured_small_mg is the multi-goal variant: it reuses the structured_small
# .map files but reads _mg_ task/tour files (ordered multi-goal task
# sequences). No endpoint ablation for this one -- only requested by frequency.
# structured_medium and structured_large use explicit frequency lists instead
# of DEFAULT_FREQUENCIES. Their extra frequencies (20 for medium; 4, 20, 40,
# 200 for large) come from scripts/generate_missing_frequency_tasks.py.
MEDIUM_FREQUENCIES = ("2", "5", "10", "20", "50", "100", "all")
LARGE_FREQUENCIES = ("4", "10", "20", "40", "100", "200", "all")

FAMILIES = (
    MapFamily("structured_small", (10, 20, 30, 40, 50), 1000,
              endpoint_ablation=True),
    MapFamily("structured_medium", (100, 200, 300, 400, 500), 2 * 3600,
              frequencies=MEDIUM_FREQUENCIES),
    MapFamily("structured_large", (200, 400, 600, 800, 1000), 4 * 3600,
              frequencies=LARGE_FREQUENCIES),
    MapFamily("sparse_small_to_medium", (10, 20, 30, 40, 50), 1000,
              endpoint_ablation=True),
    MapFamily("structured_small_mg", (10, 20, 30, 40, 50), 1000,
              map_name="structured_small", task_name="structured_small_mg"),
)
FAMILY_BY_NAME = {family.name: family for family in FAMILIES}

# None = each method's own preset default endpoint strategy (the base set).
ENDPOINT_STRATEGY_ABLATION = (None, "RETURN_TO_HOME", "NEAREST_AVAILABLE")

ONLINE_METHODS = tuple(m for m in METHODS if not m.offline_only)   # 27 rows
TA_METHODS = tuple(m for m in METHODS if m.offline_only)           # TA-Hybrid, TA-Prioritized
assert len(ONLINE_METHODS) == 27 and len(TA_METHODS) == 2


def family_map_path(family: str, agents: int) -> Path:
    map_name = FAMILY_BY_NAME[family].map_name
    return MAPS_DIR / f"benchmark_{map_name}_a{agents}.map"


def family_task_path(family: str, agents: int, frequency: str) -> Path:
    task_name = FAMILY_BY_NAME[family].task_name
    return TASKS_DIR / f"benchmark_{task_name}_a{agents}_f{frequency}.task"


def family_tour_path(family: str, agents: int) -> Path:
    task_name = FAMILY_BY_NAME[family].task_name
    return TOURS_DIR / f"benchmark_{task_name}_a{agents}_fall.tour"


def limit_memory(max_bytes: int):
    def _apply():
        resource.setrlimit(resource.RLIMIT_AS, (max_bytes, max_bytes))
    return _apply


def mode_extra_args(mode: str, lookahead: int) -> tuple[str, ...]:
    if mode == "online":
        return ()
    return ("--mode", "SEMI_ONLINE",
            "--semi_online_lookahead_batches", str(lookahead))


def build_jobs(family: MapFamily, args) -> list[dict]:
    endpoint_strategies = (ENDPOINT_STRATEGY_ABLATION if family.endpoint_ablation
                           else (None,))
    jobs: list[dict] = []
    for agents in family.agents:
        if args.agents and agents not in args.agents:
            continue
        for frequency in family.frequencies:
            if args.frequencies and frequency not in args.frequencies:
                continue
            for method in ONLINE_METHODS:
                if args.methods and method.label not in args.methods:
                    continue
                for mode in MODES:
                    for endpoint_strategy in endpoint_strategies:
                        jobs.append(dict(
                            family=family.name, agents=agents,
                            frequency=frequency, method=method, mode=mode,
                            endpoint_strategy=endpoint_strategy))
        for method in TA_METHODS:
            if args.methods and method.label not in args.methods:
                continue
            for endpoint_strategy in endpoint_strategies:
                jobs.append(dict(
                    family=family.name, agents=agents, frequency="all",
                    method=method, mode="offline",
                    endpoint_strategy=endpoint_strategy))
    return jobs


def job_stem(job: dict) -> str:
    parts = [
        job["family"], f"a{job['agents']}", f"f{job['frequency']}",
        safe_name(job["method"].label), job["mode"],
    ]
    if job.get("endpoint_strategy"):
        parts.append(f"ep-{safe_name(job['endpoint_strategy'])}")
    return "_".join(parts)


def run_one(args, output_dir: Path, job: dict) -> dict:
    method: Method = job["method"]
    family, agents, frequency, mode = (
        job["family"], job["agents"], job["frequency"], job["mode"])
    stem = job_stem(job)
    cache_path = output_dir / "cache" / f"{stem}.json"
    log_path = output_dir / "logs" / f"{stem}.log"
    paths_dir = output_dir / "paths" / stem

    if cache_path.exists() and not args.rerun:
        try:
            result = json.loads(cache_path.read_text())
        except json.JSONDecodeError:
            result = None
        # Only a genuinely completed job counts as "already have a result" --
        # errors/timeouts/collisions/incomplete runs are retried so a crashed
        # run's partial failures don't get stuck permanently.
        if result is not None and result.get("status") == "ok":
            print(f"[CACHED] {stem}: {result['status']}", flush=True)
            return result

    system_timeout = FAMILY_BY_NAME[family].timeout_s
    internal_runtime_limit = max(1, system_timeout - 5)

    map_path = family_map_path(family, agents)
    task_path = family_task_path(family, agents, frequency)
    extra = list(method.extra)
    if method.offline_only:
        tour_path = family_tour_path(family, agents)
        extra = [value.format(tour=str(tour_path)) for value in extra]
    else:
        extra = list(extra) + list(mode_extra_args(mode, args.semi_online_lookahead))

    paths_dir.mkdir(parents=True, exist_ok=True)
    command = [str(args.executable), "-m", str(map_path), "-t", str(task_path),
               "-a", method.preset, "--seed", str(args.seed), "-s", "1",
               "--runtime_limit", str(internal_runtime_limit),
               "--pathfinding_runtime_limit", str(args.pathfinding_runtime_limit),
               "--save_output", "--output_dir", str(paths_dir)] + extra
    if job.get("endpoint_strategy"):
        command.extend(("--endpoint_strategy", job["endpoint_strategy"]))

    started = time.monotonic()
    timed_out = False
    try:
        completed = subprocess.run(
            command, cwd=ROOT, capture_output=True, text=True,
            timeout=system_timeout,
            preexec_fn=limit_memory(args.memory_limit_bytes))
        output = (completed.stdout or "") + (completed.stderr or "")
        return_code = completed.returncode
    except subprocess.TimeoutExpired as error:
        stdout = error.stdout.decode() if isinstance(error.stdout, bytes) else (error.stdout or "")
        stderr = error.stderr.decode() if isinstance(error.stderr, bytes) else (error.stderr or "")
        output = stdout + stderr + f"\nTIMEOUT after {system_timeout} seconds\n"
        return_code = 124
        timed_out = True
    wall_runtime = time.monotonic() - started
    log_path.parent.mkdir(parents=True, exist_ok=True)
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
    saved_files = sorted(str(p) for p in paths_dir.glob("*.txt"))

    if timed_out:
        status = "timeout"
    elif return_code != 0:
        status = f"error({return_code})"
    elif collision_failed:
        status = "collision"
    elif not collision_passed:
        status = "error(no collision-check result)"
    elif None in (makespan, swt, runtime_ms, completed_tasks, total_tasks):
        status = "error(parse)"
    elif completed_tasks != total_tasks:
        status = "incomplete"
    else:
        status = "ok"

    result = {
        "family": family,
        "agents": agents,
        "frequency": frequency,
        "method": method.label,
        "preset": method.preset,
        "mode": mode,
        "endpoint_strategy": job.get("endpoint_strategy"),
        "seed": args.seed,
        "makespan": makespan,
        "sum_of_cost": swt,
        "runtime_s": runtime_ms / 1000.0 if runtime_ms is not None else None,
        "wall_runtime_s": wall_runtime,
        "tasks_completed": completed_tasks,
        "tasks_total": total_tasks,
        "status": status,
        "command": command,
        "log": str(log_path),
        "paths": saved_files,
    }
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    cache_path.write_text(json.dumps(result, indent=2) + "\n")
    print(f"[DONE] {stem}: ms={makespan} soc={swt} wall={wall_runtime:.2f}s "
          f"[{status}]", flush=True)
    return result


def write_summary(output_dir: Path, results: list[dict]) -> None:
    def freq_order(value):
        try:
            return (0, float(value))
        except ValueError:
            return (1, str(value))

    ordered = sorted(results, key=lambda row: (
        row["family"], row["agents"], freq_order(row["frequency"]),
        row["method"].lower(), row["mode"],
        row.get("endpoint_strategy") or ""))
    (output_dir / "results.json").write_text(json.dumps(ordered, indent=2) + "\n")
    columns = ("family", "agents", "frequency", "method", "preset", "mode",
               "endpoint_strategy", "seed", "makespan", "sum_of_cost",
               "runtime_s", "wall_runtime_s", "tasks_completed", "tasks_total",
               "status", "log")
    with (output_dir / "results.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(ordered)


def make_arg_parser(family: MapFamily) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=f"Run the {family.name} experiment matrix "
                    f"(agents={list(family.agents)}, "
                    f"per-job system timeout={family.timeout_s}s).")
    parser.add_argument("--agents", type=lambda v: tuple(int(x) for x in v.split(",")),
                        default=(), help="subset of agent counts")
    parser.add_argument("--frequencies", type=lambda v: tuple(v.split(",")),
                        default=(), help="subset of frequency labels")
    parser.add_argument("--methods", type=lambda v: tuple(v.split(",")),
                        default=(), help="subset of method labels")
    parser.add_argument("--semi-online-lookahead", type=int, default=1,
                        dest="semi_online_lookahead")
    parser.add_argument("--executable", type=Path, default=ROOT / "mapd")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR,
                        help="shared output tree (default: same directory "
                             "used by all four per-map scripts, so cached "
                             "results carry over between them)")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--pathfinding-runtime-limit", type=int, default=600,
        dest="pathfinding_runtime_limit",
        help="internal per planning-cycle wall-clock limit passed to the "
             "executable (seconds); 0 disables it")
    parser.add_argument("--max-parallel", type=int, default=20)
    parser.add_argument(
        "--memory-limit-gb", type=float, default=10.0,
        dest="memory_limit_gb",
        help="per-process RLIMIT_AS cap in GiB; a process that exceeds this "
             "aborts (std::bad_alloc) instead of exhausting system RAM and "
             "taking the whole matrix down with it")
    parser.add_argument("--rerun", action="store_true",
                        help="ignore cached per-job JSON files (by default, "
                             "only jobs that already completed with status "
                             "'ok' are skipped -- failed/timed-out jobs from "
                             "a previous run are retried automatically)")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the job count and exit")
    return parser


def run_matrix(family: MapFamily, argv: list[str] | None = None) -> int:
    parser = make_arg_parser(family)
    args = parser.parse_args(argv)

    if not 1 <= args.max_parallel <= 20:
        parser.error("--max-parallel must be between 1 and 20")
    if args.seed < 0:
        parser.error("use a non-negative seed for reproducible server runs")
    if not args.dry_run and not args.executable.is_file():
        parser.error(f"executable not found: {args.executable}; run make first")
    args.memory_limit_bytes = int(args.memory_limit_gb * (1024 ** 3))

    jobs = build_jobs(family, args)

    if args.dry_run:
        by_freq = Counter(job["frequency"] for job in jobs)
        print(f"Total jobs for {family.name}: {len(jobs)}")
        for frequency, count in sorted(by_freq.items()):
            print(f"  f{frequency}: {count}")
        if family.endpoint_ablation:
            by_ep = Counter(job["endpoint_strategy"] or "default" for job in jobs)
            print("  endpoint-strategy ablation breakdown:")
            for strategy, count in sorted(by_ep.items()):
                print(f"    {strategy}: {count}")
        return 0

    missing = []
    for job in jobs:
        map_path = family_map_path(job["family"], job["agents"])
        task_path = family_task_path(job["family"], job["agents"], job["frequency"])
        for path in (map_path, task_path):
            if not path.is_file():
                missing.append(path)
        if job["method"].offline_only:
            tour_path = family_tour_path(job["family"], job["agents"])
            if not tour_path.is_file():
                missing.append(tour_path)
    if missing:
        parser.error("missing input files:\n  " +
                     "\n  ".join(str(p) for p in sorted(set(missing))))

    args.output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Running {len(jobs)} {family.name} jobs with seed={args.seed}, "
          f"max_parallel={args.max_parallel}, "
          f"system_timeout={family.timeout_s}s, "
          f"pathfinding_runtime_limit={args.pathfinding_runtime_limit}s, "
          f"memory_limit={args.memory_limit_gb}GiB", flush=True)
    results = []
    with concurrent.futures.ThreadPoolExecutor(
            max_workers=args.max_parallel) as executor:
        futures = [executor.submit(run_one, args, args.output_dir, job)
                   for job in jobs]
        for future in concurrent.futures.as_completed(futures):
            results.append(future.result())

    write_summary(args.output_dir, results)
    failures = [f"a{row['agents']} f{row['frequency']} "
                f"{row['method']} {row['mode']}: {row['status']}"
                for row in results if row["status"] != "ok"]

    print(f"\nResults: {args.output_dir / 'results.csv'}")
    print(f"Manifest: {args.output_dir / 'results.json'}")
    if failures:
        print(f"\n{len(failures)} non-ok job(s) in {family.name}:", file=sys.stderr)
        for failure in failures[:50]:
            print(f"  {failure}", file=sys.stderr)
        if len(failures) > 50:
            print(f"  ... and {len(failures) - 50} more", file=sys.stderr)
    print(f"{family.name}: {len(results) - len(failures)}/{len(results)} jobs ok.")
    return 0
