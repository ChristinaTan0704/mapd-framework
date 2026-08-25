#!/usr/bin/env python3
"""Generate agent-count and release-frequency variants for all benchmarks.

Each task file is generated against the exact endpoint count/order of its
matching map variant. The canonical unsuffixed files remain aliases for each
benchmark's default agent count and release frequency.
"""

from __future__ import annotations

import hashlib
import random
from collections import deque
from dataclasses import dataclass, replace
from fractions import Fraction
from pathlib import Path

import generate_paper_large_instance as large_generator
import generate_sparse_medium_grid_instance as sparse_generator
import generate_structured_small_medium as structured_generator


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "benchmark_instances"
MAPS_DIR = PACKAGE / "maps"
TASKS_DIR = PACKAGE / "tasks"


@dataclass(frozen=True)
class Benchmark:
    name: str
    agent_counts: tuple[int, ...]
    task_count: int
    default_agents: int
    default_frequency: str


BENCHMARKS = (
    Benchmark("structured_small", (10, 20, 30, 40, 50), 500, 10, "5"),
    Benchmark("structured_medium", (100, 200, 300, 400, 500), 1000, 100, "50"),
    Benchmark("structured_large", (200, 400, 600, 800, 1000), 4000, 1000, "100"),
    Benchmark("sparse_small_to_medium", (10, 20, 30, 40, 50), 500, 10, "5"),
)

# Original paper sweep, generated MEDIUM/LARGE defaults, and an explicit
# offline/all-at-once workload.
FREQUENCIES: tuple[tuple[str, Fraction | None], ...] = (
    ("0.2", Fraction(1, 5)),
    ("0.5", Fraction(1, 2)),
    ("1", Fraction(1, 1)),
    ("2", Fraction(2, 1)),
    ("5", Fraction(5, 1)),
    ("10", Fraction(10, 1)),
    ("50", Fraction(50, 1)),
    ("100", Fraction(100, 1)),
    ("500", Fraction(500, 1)),
    ("all", None),
)
PAPER_LARGE_TASK_COUNTS = (1000, 2000, 3000, 4000, 5000)
PAPER_MG_FREQUENCIES: tuple[tuple[str, Fraction | None], ...] = (
    ("0.2", Fraction(1, 5)),
    ("0.5", Fraction(1, 2)),
    ("1", Fraction(1, 1)),
    ("2", Fraction(2, 1)),
    ("5", Fraction(5, 1)),
    ("10", Fraction(10, 1)),
    ("500", Fraction(500, 1)),
    ("all", None),
)


def write_map(path: Path, grid: list[list[str]], endpoint_count: int,
              agent_count: int, maxtime: int) -> None:
    lines = [
        f"{len(grid)},{len(grid[0])}",
        str(endpoint_count),
        str(agent_count),
        str(maxtime),
        *("".join(row) for row in grid),
    ]
    path.write_text("\n".join(lines) + "\n")


def build_structured(name: str, agent_count: int) -> tuple[list[list[str]], int, int]:
    base_spec = next(
        spec for spec in structured_generator.SPECS if spec.name == name)
    spec = replace(base_spec, agents=agent_count)
    grid, endpoint_count = structured_generator.build_map(spec, seed=0)
    structured_generator.validate(spec, grid, endpoint_count)
    return grid, endpoint_count, spec.maxtime


def build_large(agent_count: int) -> tuple[list[list[str]], int, int]:
    grid, endpoint_count = large_generator.build_map(agent_count, seed=0)
    large_generator.validate(grid, endpoint_count, agent_count)
    return grid, endpoint_count, large_generator.MAXTIME


def build_sparse(agent_count: int) -> tuple[list[list[str]], int, int]:
    grid, endpoint_count = sparse_generator.build_map(agent_count, seed=0)
    sparse_generator.validate(grid, endpoint_count, agent_count)
    return grid, endpoint_count, sparse_generator.MAXTIME


def validate_well_formed(grid: list[list[str]], label: str) -> None:
    """Check the paper's endpoint-avoiding connectivity condition."""
    rows = len(grid)
    cols = len(grid[0])
    component = [[-1 for _ in range(cols)] for _ in range(rows)]
    component_id = 0
    for y in range(rows):
        for x in range(cols):
            if grid[y][x] != "." or component[y][x] != -1:
                continue
            queue = deque([(y, x)])
            component[y][x] = component_id
            while queue:
                cy, cx = queue.popleft()
                for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    ny, nx = cy + dy, cx + dx
                    if not (0 <= ny < rows and 0 <= nx < cols):
                        continue
                    if grid[ny][nx] != "." or component[ny][nx] != -1:
                        continue
                    component[ny][nx] = component_id
                    queue.append((ny, nx))
            component_id += 1

    common_components: set[int] | None = None
    for y in range(rows):
        for x in range(cols):
            if grid[y][x] not in {"e", "r"}:
                continue
            adjacent = {
                component[y + dy][x + dx]
                for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1))
                if 0 <= y + dy < rows
                and 0 <= x + dx < cols
                and component[y + dy][x + dx] >= 0
            }
            if not adjacent:
                raise ValueError(
                    f"{label}: endpoint ({y}, {x}) has no non-endpoint exit")
            common_components = (
                adjacent if common_components is None
                else common_components & adjacent)
            if not common_components:
                raise ValueError(
                    f"{label}: endpoints do not share an endpoint-free aisle component")


def build_map(benchmark: Benchmark, agent_count: int) -> tuple[list[list[str]], int, int]:
    if benchmark.name == "structured_small":
        return build_structured("small", agent_count)
    if benchmark.name == "structured_medium":
        return build_structured("medium", agent_count)
    if benchmark.name == "structured_large":
        return build_large(agent_count)
    if benchmark.name == "sparse_small_to_medium":
        return build_sparse(agent_count)
    raise ValueError(f"unknown benchmark: {benchmark.name}")


def generate_task_pairs(endpoint_count: int, task_count: int,
                        seed: int = 0) -> list[tuple[int, int, int, int]]:
    rng = random.Random(seed + 1)
    tasks = []
    for _ in range(task_count):
        pickup = rng.randrange(endpoint_count)
        delivery = rng.randrange(endpoint_count - 1)
        if delivery >= pickup:
            delivery += 1
        tasks.append((pickup, delivery, 0, 0))
    return tasks


def release_time(task_id: int, frequency: Fraction | None) -> int:
    if frequency is None:
        return 0
    return task_id * frequency.denominator // frequency.numerator


def write_tasks(path: Path, tasks: list[tuple[int, int, int, int]],
                frequency: Fraction | None) -> None:
    lines = [str(len(tasks))]
    for task_id, (pickup, delivery, start_wait, goal_wait) in enumerate(tasks):
        lines.append(
            f"{release_time(task_id, frequency)} {pickup} {delivery} "
            f"{start_wait} {goal_wait}")
    path.write_text("\n".join(lines) + "\n")


def generate_multigoal_tasks(endpoint_count: int, task_count: int,
                             seed: int = 0) -> list[list[int]]:
    rng = random.Random(seed + 2)
    tasks = []
    for _ in range(task_count):
        goal_count = rng.randint(1, 5)
        tasks.append(rng.sample(range(endpoint_count), goal_count))
    return tasks


def write_multigoal_tasks(path: Path, tasks: list[list[int]],
                          frequency: Fraction | None) -> None:
    lines = [str(len(tasks))]
    for task_id, goals in enumerate(tasks):
        values = [str(release_time(task_id, frequency)),
                  *(str(goal) for goal in goals)]
        lines.append(" ".join(values))
    path.write_text("\n".join(lines) + "\n")


def validate_tasks(tasks: list[tuple[int, int, int, int]],
                   endpoint_count: int, label: str) -> None:
    for task_id, (pickup, delivery, _, _) in enumerate(tasks):
        if not (0 <= pickup < endpoint_count):
            raise ValueError(f"{label}: task {task_id} pickup {pickup} is invalid")
        if not (0 <= delivery < endpoint_count):
            raise ValueError(
                f"{label}: task {task_id} delivery {delivery} is invalid")
        if pickup == delivery:
            raise ValueError(f"{label}: task {task_id} has identical endpoints")


def write_checksums() -> None:
    paths = sorted(MAPS_DIR.glob("*.map")) + sorted(TASKS_DIR.glob("*.task"))
    lines = []
    for path in paths:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        lines.append(f"{digest}  {path.relative_to(PACKAGE)}")
    (PACKAGE / "SHA256SUMS").write_text("\n".join(lines) + "\n")


def main() -> None:
    generated_maps = 0
    generated_tasks = 0
    endpoint_counts: dict[tuple[str, int], int] = {}
    for benchmark in BENCHMARKS:
        for agent_count in benchmark.agent_counts:
            grid, endpoint_count, maxtime = build_map(benchmark, agent_count)
            map_path = MAPS_DIR / f"benchmark_{benchmark.name}_a{agent_count}.map"
            validate_well_formed(grid, map_path.name)
            write_map(map_path, grid, endpoint_count, agent_count, maxtime)
            endpoint_counts[(benchmark.name, agent_count)] = endpoint_count
            generated_maps += 1

            tasks = generate_task_pairs(
                endpoint_count, benchmark.task_count, seed=0)
            validate_tasks(tasks, endpoint_count, map_path.name)

            for frequency_label, frequency in FREQUENCIES:
                task_path = TASKS_DIR / (
                    f"benchmark_{benchmark.name}_a{agent_count}_"
                    f"f{frequency_label}.task")
                write_tasks(task_path, tasks, frequency)
                generated_tasks += 1

            if agent_count == benchmark.default_agents:
                canonical_map = MAPS_DIR / f"benchmark_{benchmark.name}.map"
                canonical_task = TASKS_DIR / f"benchmark_{benchmark.name}.task"
                if canonical_map.read_bytes() != map_path.read_bytes():
                    raise ValueError(
                        f"default map variant differs from {canonical_map}")
                default_task = TASKS_DIR / (
                    f"benchmark_{benchmark.name}_a{agent_count}_"
                    f"f{benchmark.default_frequency}.task")
                if canonical_task.read_bytes() != default_task.read_bytes():
                    raise ValueError(
                        f"default task variant differs from {canonical_task}")

    # TABLE IV: LARGE uses 1,000 agents, f=100, and 1,000--5,000 tasks.
    large_endpoints = endpoint_counts[("structured_large", 1000)]
    for task_count in PAPER_LARGE_TASK_COUNTS:
        tasks = generate_task_pairs(large_endpoints, task_count, seed=0)
        path = TASKS_DIR / (
            f"benchmark_structured_large_a1000_t{task_count}_f100.task")
        write_tasks(path, tasks, Fraction(100, 1))
        generated_tasks += 1

    # TABLE V: MG-MAPD uses SMALL, 500 tasks with 1--5 goals, and f=2,5,10.
    small = next(item for item in BENCHMARKS
                 if item.name == "structured_small")
    for agent_count in small.agent_counts:
        endpoint_count = endpoint_counts[(small.name, agent_count)]
        tasks = generate_multigoal_tasks(endpoint_count, small.task_count)
        if not all(1 <= len(goals) <= 5 for goals in tasks):
            raise ValueError("MG-MAPD task goal count is outside 1..5")
        for label, frequency in PAPER_MG_FREQUENCIES:
            path = TASKS_DIR / (
                f"benchmark_structured_small_mg_a{agent_count}_f{label}.task")
            write_multigoal_tasks(path, tasks, frequency)
            generated_tasks += 1

    write_checksums()
    print(f"generated_map_files={generated_maps}")
    print(f"generated_task_files={generated_tasks}")
    print(f"benchmarks={len(BENCHMARKS)}")
    print(f"agent_counts_per_benchmark={len(BENCHMARKS[0].agent_counts)}")
    print(f"frequencies_per_map={len(FREQUENCIES)}")


if __name__ == "__main__":
    main()
