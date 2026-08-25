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
    label: str
    preset: str
    extra: tuple[str, ...] = ()
    offline_only: bool = False


METHODS = (
    Method("TP-STA*", "TP"),
    Method("TPTS-STA*", "TPTS"),
    Method("CENTRAL-CBS", "CENTRAL-CBS"),
    Method("CENTRAL-fixed-CBS", "CENTRAL-fixed"),
    Method("HBH+MLA*", "HBH_MLA"),
    Method("TA-Hybrid-STA*", "TA_HYBRID", ("--tour", "{tour}"), True),
    Method("TA-Prioritized-STA*", "TA_PRIORITIZED", ("--tour", "{tour}"), True),
    Method("Hungarian+PBS-MLA*", "HUNGARIAN_PBS"),
    Method("Hungarian+wPBS-MLA*", "HUNGARIAN_wPBS"),
    Method("LNS(1s)+PBS-MLA*", "LNS_PBS", ("--lns_time", "1")),
    Method("LNS(1s)+wPBS-MLA*", "LNS_wPBS", ("--lns_time", "1")),
    Method("Hungarian+PBS-MLSIPP", "HUNGARIAN_PBS",
           ("--single_agent", "MLSIPP")),
    Method("Hungarian+wPBS-MLSIPP", "HUNGARIAN_wPBS",
           ("--single_agent", "MLSIPP")),
    Method("LNS(1s)+PBS-MLSIPP", "LNS_PBS",
           ("--lns_time", "1", "--single_agent", "MLSIPP")),
    Method("LNS(1s)+wPBS-MLSIPP", "LNS_wPBS",
           ("--lns_time", "1", "--single_agent", "MLSIPP")),
    Method("TP-SIPP", "TP", ("--single_agent", "MLSIPP")),
    Method("TPTS-SIPP", "TPTS", ("--single_agent", "MLSIPP")),
    Method("Hungarian+PP-SIPP", "HUNGARIAN_PBS",
           ("--mapf", "PP", "--single_agent", "MLSIPP")),
    Method("LNS(1s)+PP-SIPP", "LNS_PBS",
           ("--mapf", "PP", "--single_agent", "MLSIPP", "--lns_time", "1")),
    Method("Hungarian+PBS-MLA* (ts 1)", "HUNGARIAN_PBS",
           ("--task_sequence_limit", "1")),
    Method("Hungarian+wPBS-MLA* (ts 1)", "HUNGARIAN_wPBS",
           ("--task_sequence_limit", "1")),
    Method("LNS(1s)+PBS-MLA* (ts 1)", "LNS_PBS",
           ("--lns_time", "1", "--task_sequence_limit", "1")),
    Method("LNS(1s)+wPBS-MLA* (ts 1)", "LNS_wPBS",
           ("--lns_time", "1", "--task_sequence_limit", "1")),
    Method("Hungarian+PBS-MLSIPP (ts 1)", "HUNGARIAN_PBS",
           ("--single_agent", "MLSIPP", "--task_sequence_limit", "1")),
    Method("Hungarian+wPBS-MLSIPP (ts 1)", "HUNGARIAN_wPBS",
           ("--single_agent", "MLSIPP", "--task_sequence_limit", "1")),
    Method("LNS(1s)+PBS-MLSIPP (ts 1)", "LNS_PBS",
           ("--lns_time", "1", "--single_agent", "MLSIPP",
            "--task_sequence_limit", "1")),
    Method("LNS(1s)+wPBS-MLSIPP (ts 1)", "LNS_wPBS",
           ("--lns_time", "1", "--single_agent", "MLSIPP",
            "--task_sequence_limit", "1")),
)

# Seed-0 results from the 2026-08-24 macOS validation.  Runtime is deliberately
# excluded because it depends on the server.  LNS rows are reported but are not
# exact-gated: their one-second CPU budget can perform a different number of
# iterations on another processor/compiler.
SMOKE_BASELINE = {
    "TP-STA*": (2532, 19417),
    "TPTS-STA*": (2532, 14666),
    "CENTRAL-CBS": (2513, 14039),
    "CENTRAL-fixed-CBS": (2514, 14054),
    "HBH+MLA*": (2532, 14728),
    "TA-Hybrid-STA*": (1053, 264939),
    "TA-Prioritized-STA*": (1052, 263486),
    "Hungarian+PBS-MLA*": (2514, 13925),
    "Hungarian+wPBS-MLA*": (2514, 13722),
    "LNS(1s)+PBS-MLA*": (2514, 13927),
    "LNS(1s)+wPBS-MLA*": (2513, 13620),
    "Hungarian+PBS-MLSIPP": (2517, 13829),
    "Hungarian+wPBS-MLSIPP": (2512, 13748),
    "LNS(1s)+PBS-MLSIPP": (2514, 13792),
    "LNS(1s)+wPBS-MLSIPP": (2513, 13708),
    "TP-SIPP": (2541, 18830),
    "TPTS-SIPP": (2514, 14241),
    "Hungarian+PP-SIPP": (2517, 13811),
    "LNS(1s)+PP-SIPP": (2514, 13778),
    "Hungarian+PBS-MLA* (ts 1)": (2514, 13937),
    "Hungarian+wPBS-MLA* (ts 1)": (2519, 13723),
    "LNS(1s)+PBS-MLA* (ts 1)": (2514, 13951),
    "LNS(1s)+wPBS-MLA* (ts 1)": (2512, 13657),
    "Hungarian+PBS-MLSIPP (ts 1)": (2517, 13861),
    "Hungarian+wPBS-MLSIPP (ts 1)": (2513, 13543),
    "LNS(1s)+PBS-MLSIPP (ts 1)": (2514, 13779),
    "LNS(1s)+wPBS-MLSIPP (ts 1)": (2512, 13636),
}


def parse_list(value: str, cast):
    return tuple(cast(item.strip()) for item in value.split(",") if item.strip())


def safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9]+", "_", value).strip("_")


def metric(pattern: str, output: str, cast):
    match = re.search(pattern, output)
    return cast(match.group(1)) if match else None


def build_jobs(args, selected_methods):
    jobs = []
    for method in selected_methods:
        if method.offline_only:
            frequencies = ("500",)
        else:
            frequencies = ("0.2",) if args.smoke else args.frequencies
        agents = (10,) if args.smoke else args.agents
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

    map_path = args.data_dir / f"kiva-{agents}-500-5.map"
    task_path = args.data_dir / f"kiva-{frequency}.task"
    tour_path = args.tour_dir / f"{agents}-500.tour"
    extra = [value.format(tour=str(tour_path)) for value in method.extra]
    command = [str(args.executable), "-m", str(map_path), "-t", str(task_path),
               "-a", method.preset, "--seed", str(args.seed), "-s", "1"] + extra
    if args.endpoint_strategy:
        command.extend(("--endpoint_strategy", args.endpoint_strategy))

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
        "method": method.label,
        "preset": method.preset,
        "agents": agents,
        "frequency": frequency,
        "seed": args.seed,
        "makespan": makespan,
        "swt": swt,
        "runtime_s": runtime_ms / 1000.0 if runtime_ms is not None else None,
        "wall_runtime_s": wall_runtime,
        "tasks_completed": completed_tasks,
        "tasks_total": total_tasks,
        "status": status,
        "command": command,
        "log": str(log_path),
    }
    cache_path.write_text(json.dumps(result, indent=2) + "\n")
    print(f"[DONE] {method.label} {agents}/{frequency}: ms={makespan} "
          f"swt={swt} wall={wall_runtime:.2f}s [{status}]", flush=True)
    return result


def write_summary(output_dir: Path, results):
    ordered = sorted(results, key=lambda row: (
        row["agents"], float(row["frequency"]), row["method"].lower()))
    (output_dir / "results.json").write_text(json.dumps(ordered, indent=2) + "\n")
    columns = ("method", "preset", "agents", "frequency", "seed",
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--smoke", action="store_true",
                        help="run every method at 10 agents/0.2 (TA at 10/500) and validate")
    parser.add_argument("--methods", default="all",
                        help="comma-separated displayed method labels, or all")
    parser.add_argument("--agents", type=lambda value: parse_list(value, int),
                        default=DEFAULT_AGENTS)
    parser.add_argument("--frequencies", type=lambda value: parse_list(value, str),
                        default=DEFAULT_FREQUENCIES)
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--tour-dir", type=Path, default=ROOT / "tour")
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
        help="override the preset endpoint strategy for every selected method")
    parser.add_argument("--timeout", type=int, default=1800,
                        help="wall-clock timeout per simulator process (seconds)")
    parser.add_argument("--max-parallel", type=int, default=5)
    parser.add_argument("--rerun", action="store_true",
                        help="ignore cached per-job JSON files")
    args = parser.parse_args()

    if not 1 <= args.max_parallel <= 5:
        parser.error("--max-parallel must be between 1 and 5")
    if args.seed < 0:
        parser.error("use a non-negative seed for reproducible server runs")
    if not args.executable.is_file():
        parser.error(f"executable not found: {args.executable}; run make first")

    by_label = {method.label: method for method in METHODS}
    if args.methods == "all":
        selected = METHODS
    else:
        requested = [item.strip() for item in args.methods.split(",")]
        unknown = [item for item in requested if item not in by_label]
        if unknown:
            parser.error("unknown method label(s): " + ", ".join(unknown))
        selected = tuple(by_label[item] for item in requested)

    jobs = build_jobs(args, selected)
    missing = []
    for method, agents, frequency in jobs:
        for path in (args.data_dir / f"kiva-{agents}-500-5.map",
                     args.data_dir / f"kiva-{frequency}.task"):
            if not path.is_file():
                missing.append(path)
        if method.offline_only:
            path = args.tour_dir / f"{agents}-500.tour"
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
