#!/usr/bin/env python3
"""Generate LKH3 task-assignment tours for packaged MAPD benchmarks."""

from __future__ import annotations

import argparse
import csv
import hashlib
import re
import shutil
import subprocess
import tempfile
from array import array
from collections import deque
from dataclasses import dataclass
from pathlib import Path

from generate_benchmark_task_matrix import BENCHMARKS


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "benchmark_instances"
MAP_DIR = PACKAGE / "maps"
TASK_DIR = PACKAGE / "tasks"
TOUR_DIR = PACKAGE / "lkh_tours"
REFERENCE_ROOT = ROOT.parent / "reference_code"
INF = 1_000_000


@dataclass(frozen=True)
class Case:
    name: str
    map_path: Path
    task_path: Path
    tour_path: Path


def cases() -> list[Case]:
    result = []
    for benchmark in BENCHMARKS:
        for agents in benchmark.agent_counts:
            stem = f"benchmark_{benchmark.name}_a{agents}_fall"
            result.append(Case(
                stem,
                MAP_DIR / f"benchmark_{benchmark.name}_a{agents}.map",
                TASK_DIR / f"{stem}.task",
                TOUR_DIR / f"{stem}.tour",
            ))

    for agents in (10, 20, 30, 40, 50):
        stem = f"benchmark_structured_small_mg_a{agents}_fall"
        result.append(Case(
            stem,
            MAP_DIR / f"benchmark_structured_small_a{agents}.map",
            TASK_DIR / f"{stem}.task",
            TOUR_DIR / f"{stem}.tour",
        ))

    return result


def build_native_lkh(build_root: Path) -> Path:
    candidates = (
        REFERENCE_ROOT / "TA-Prioritized/LKH3",
        REFERENCE_ROOT / "TA-Hybrid/LKH3",
    )
    source = next((path for path in candidates if path.is_dir()), None)
    if source is None:
        raise FileNotFoundError("could not find TA-Prioritized or TA-Hybrid LKH3")
    destination = build_root / "LKH3"
    shutil.copytree(source, destination)
    subprocess.run(["make", "clean"], cwd=destination,
                   check=True, stdout=subprocess.DEVNULL)
    subprocess.run(["make", "-j4"], cwd=destination,
                   check=True, stdout=subprocess.DEVNULL)
    binary = destination / "LKH"
    if not binary.is_file():
        raise FileNotFoundError(f"native LKH build did not produce {binary}")
    return binary


def read_map(path: Path):
    lines = path.read_text().splitlines()
    rows, cols = map(int, lines[0].split(","))
    maxtime = int(lines[3])
    grid = lines[4:4 + rows]
    task_endpoints = []
    homes = []
    for y, row in enumerate(grid):
        for x, value in enumerate(row):
            location = y * cols + x
            if value == "e":
                task_endpoints.append(location)
            elif value == "r":
                homes.append(location)
    return grid, rows, cols, task_endpoints, homes, maxtime


def read_tasks(path: Path, multigoal: bool):
    lines = path.read_text().splitlines()
    expected = int(lines[0])
    tasks = []
    for line_number, line in enumerate(lines[1:], start=2):
        values = list(map(int, line.split()))
        if multigoal:
            if len(values) < 2:
                raise ValueError(f"{path}:{line_number}: missing goals")
            release, goals = values[0], values[1:]
        else:
            if len(values) != 5:
                raise ValueError(f"{path}:{line_number}: expected five fields")
            release, pickup, delivery, _, _ = values
            goals = [pickup, delivery]
        tasks.append((release, goals))
    if len(tasks) != expected:
        raise ValueError(f"{path}: expected {expected} tasks, found {len(tasks)}")
    return tasks


class Distances:
    def __init__(self, grid: list[str], rows: int, cols: int,
                 target_locations: list[int]):
        self.grid = grid
        self.rows = rows
        self.cols = cols
        self.targets = target_locations
        self.target_index = {
            location: index for index, location in enumerate(target_locations)}
        self.cache: dict[int, array] = {}

    def row(self, source: int) -> array:
        if source in self.cache:
            return self.cache[source]
        distances = array("i", [-1]) * (self.rows * self.cols)
        distances[source] = 0
        queue = deque([source])
        while queue:
            location = queue.popleft()
            y, x = divmod(location, self.cols)
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                y2, x2 = y + dy, x + dx
                if not (0 <= y2 < self.rows and 0 <= x2 < self.cols):
                    continue
                next_location = y2 * self.cols + x2
                if self.grid[y2][x2] == "@" or distances[next_location] >= 0:
                    continue
                distances[next_location] = distances[location] + 1
                queue.append(next_location)
        result = array("H")
        for target in self.targets:
            distance = distances[target]
            if distance < 0 or distance > 65535:
                raise ValueError(f"unreachable or oversized distance {source}->{target}")
            result.append(distance)
        self.cache[source] = result
        return result

    def get(self, source: int, target: int) -> int:
        return self.row(source)[self.target_index[target]]


def write_problem(case: Case, output: Path):
    grid, rows, cols, endpoints, homes, maxtime = read_map(case.map_path)
    raw_tasks = read_tasks(case.task_path, "_mg_" in case.task_path.name)
    tasks = []
    for release, goal_ids in raw_tasks:
        if any(not 0 <= goal < len(endpoints) for goal in goal_ids):
            raise ValueError(f"{case.task_path}: endpoint index out of range")
        tasks.append((release, [endpoints[goal] for goal in goal_ids]))

    target_locations = sorted({
        goal for _, goals in tasks for goal in goals[:-1]
    } | {goals[0] for _, goals in tasks})
    distances = Distances(grid, rows, cols, target_locations)
    services = []
    for _, goals in tasks:
        services.append(sum(
            distances.get(goals[index + 1], goals[index])
            for index in range(len(goals) - 1)))

    agent_count = len(homes)
    task_count = len(tasks)
    dimension = agent_count + task_count
    horizon = max(maxtime, max(release for release, _ in tasks) + 10000)

    with output.open("w") as stream:
        stream.write("NAME: MAPD\n")
        stream.write("TYPE: TSPTW\n")
        stream.write(f"DIMENSION: {dimension}\n")
        stream.write("EDGE_WEIGHT_TYPE: EXPLICIT\n")
        stream.write("DISPLAY_DATA_TYPE: NO_DISPLAY\n")
        stream.write("EDGE_WEIGHT_FORMAT: FULL_MATRIX\n")
        stream.write("EDGE_WEIGHT_SECTION\n")

        pickup_indices = [distances.target_index[goals[0]]
                          for _, goals in tasks]
        for home in homes:
            home_distances = distances.row(home)
            values = [str(INF)] * agent_count
            values.extend(str(max(home_distances[pickup_indices[index]], release))
                          for index, (release, _) in enumerate(tasks))
            stream.write(" ".join(values) + "\n")

        for task_id, (_, goals) in enumerate(tasks):
            source_distances = distances.row(goals[-1])
            service = services[task_id]
            values = [str(service)] * agent_count
            for next_task in range(task_count):
                if next_task == task_id:
                    values.append(str(INF))
                else:
                    values.append(str(
                        service + source_distances[pickup_indices[next_task]]))
            stream.write(" ".join(values) + "\n")

        stream.write("TIME_WINDOW_SECTION\n")
        for node in range(1, agent_count + 1):
            stream.write(f"{node} -1 {horizon} 0 0 0\n")
        for task_id, (release, goals) in enumerate(tasks):
            node = agent_count + task_id + 1
            stream.write(
                f"{node} {release} {horizon} {services[task_id]} "
                f"{goals[0]} {goals[-1]}\n")
    return agent_count, task_count


def validate_tour(path: Path, agent_count: int, task_count: int) -> str:
    lines = path.read_text().splitlines()
    try:
        section = lines.index("TOUR_SECTION")
    except ValueError as error:
        raise ValueError(f"{path}: missing TOUR_SECTION") from error
    nodes = []
    for line in lines[section + 1:]:
        value = int(line.strip())
        if value < 0:
            break
        nodes.append(value)
    dimension = agent_count + task_count
    if len(nodes) != dimension or set(nodes) != set(range(1, dimension + 1)):
        raise ValueError(f"{path}: tour does not cover all {dimension} nodes")
    if nodes[0] != 1:
        raise ValueError(f"{path}: tour must begin with agent node 1")
    cost_line = next((line for line in lines if line.startswith("COMMENT : Cost")), "")
    match = re.search(r"Cost = (.+)$", cost_line)
    return match.group(1) if match else "unknown"


def run_case(binary: Path, case: Case, work: Path,
             time_limit: float) -> tuple[int, int, str]:
    problem = work / f"{case.name}.tsptw"
    parameter = work / f"{case.name}.par"
    initial_tour = work / f"{case.name}.initial.tour"
    raw_tour = work / f"{case.name}.tour"
    agent_count, task_count = write_problem(case, problem)
    nodes = []
    for agent in range(agent_count):
        nodes.append(agent + 1)
        nodes.extend(
            agent_count + task + 1
            for task in range(agent, task_count, agent_count))
    initial_tour.write_text("\n".join([
        f"NAME : {case.name}.initial",
        "TYPE : TOUR",
        f"DIMENSION : {agent_count + task_count}",
        "TOUR_SECTION",
        *(str(node) for node in nodes),
        "-1",
        "EOF",
    ]) + "\n")
    parameter.write_text("\n".join([
        "SPECIAL",
        f"PROBLEM_FILE = {problem}",
        "MAX_TRIALS = 1",
        f"TIME_LIMIT = {time_limit}",
        "RUNS = 1",
        "TRACE_LEVEL = 0",
        f"TOUR_FILE = {raw_tour}",
        f"INITIAL_TOUR_FILE = {initial_tour}",
        "MAKESPAN = YES",
        "SEED = 1",
        "ASCENT_CANDIDATES = 3",
        "MAX_CANDIDATES = 3",
        "MOVE_TYPE = 2",
        "PATCHING_A = 0",
        "PATCHING_C = 0",
        "POPULATION_SIZE = 1",
        "SUBGRADIENT = NO",
    ]) + "\n")
    subprocess.run([str(binary), str(parameter)], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    cost = validate_tour(raw_tour, agent_count, task_count)
    case.tour_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(raw_tour, case.tour_path)
    return agent_count, task_count, cost


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--time-limit", type=float, default=1.0)
    parser.add_argument("--only", default="",
                        help="generate only cases whose names contain this text")
    parser.add_argument("--skip-existing", action="store_true")
    args = parser.parse_args()
    if args.time_limit <= 0:
        parser.error("--time-limit must be positive")

    selected = [case for case in cases() if args.only in case.name]
    TOUR_DIR.mkdir(parents=True, exist_ok=True)
    records = []
    with tempfile.TemporaryDirectory(prefix="mapd-lkh3-") as temp_name:
        temp = Path(temp_name)
        binary = build_native_lkh(temp)
        work = temp / "work"
        work.mkdir()
        for index, case in enumerate(selected, start=1):
            print(f"[{index}/{len(selected)}] {case.name}", flush=True)
            if args.skip_existing and case.tour_path.is_file():
                _, _, _, _, homes, _ = read_map(case.map_path)
                tasks_data = read_tasks(
                    case.task_path, "_mg_" in case.task_path.name)
                agents, tasks = len(homes), len(tasks_data)
                cost = validate_tour(case.tour_path, agents, tasks)
            else:
                agents, tasks, cost = run_case(
                    binary, case, work, args.time_limit)
            records.append((case.name, agents, tasks,
                            case.map_path.relative_to(PACKAGE),
                            case.task_path.relative_to(PACKAGE),
                            case.tour_path.relative_to(PACKAGE), cost))
            for temporary in work.iterdir():
                temporary.unlink()

    manifest = TOUR_DIR / "manifest.csv"
    with manifest.open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("case", "agents", "tasks", "map", "task", "tour",
                         "lkh_cost"))
        writer.writerows(records)

    checksums = []
    for path in sorted(TOUR_DIR.glob("*.tour")):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        checksums.append(f"{digest}  {path.name}")
    (TOUR_DIR / "SHA256SUMS").write_text("\n".join(checksums) + "\n")
    print(f"generated_tours={len(records)}")


if __name__ == "__main__":
    main()
