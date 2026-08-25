#!/usr/bin/env python3
"""Validate packaged benchmarks against the MG-MAPD paper requirements."""

from __future__ import annotations

from pathlib import Path

import generate_benchmark_task_matrix as matrix
import generate_benchmark_lkh_tours as lkh


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "benchmark_instances"

LAYOUT_EXPECTATIONS = {
    "structured_small": (21, 35, 100),
    "structured_medium": (81, 101, 1600),
    "structured_large": (153, 187, 5700),
    "sparse_small_to_medium": (81, 101, 100),
}


def read_map(path: Path) -> tuple[list[list[str]], int, int]:
    lines = path.read_text().splitlines()
    rows, cols = map(int, lines[0].split(","))
    endpoint_count = int(lines[1])
    agent_count = int(lines[2])
    grid = [list(row) for row in lines[4:4 + rows]]
    if len(grid) != rows or any(len(row) != cols for row in grid):
        raise ValueError(f"{path}: invalid dimensions")
    if sum(row.count("e") for row in grid) != endpoint_count:
        raise ValueError(f"{path}: task-endpoint header mismatch")
    if sum(row.count("r") for row in grid) != agent_count:
        raise ValueError(f"{path}: agent header mismatch")
    return grid, endpoint_count, agent_count


def validate_standard_tasks(path: Path, endpoint_count: int, task_count: int,
                            frequency) -> None:
    lines = path.read_text().splitlines()
    if int(lines[0]) != task_count or len(lines) != task_count + 1:
        raise ValueError(f"{path}: task-count mismatch")
    for task_id, line in enumerate(lines[1:]):
        fields = list(map(int, line.split()))
        if len(fields) != 5:
            raise ValueError(f"{path}: task {task_id} is not standard MAPD")
        release, pickup, delivery, _, _ = fields
        if release != matrix.release_time(task_id, frequency):
            raise ValueError(f"{path}: task {task_id} has wrong release time")
        if not (0 <= pickup < endpoint_count and
                0 <= delivery < endpoint_count and pickup != delivery):
            raise ValueError(f"{path}: task {task_id} has invalid endpoints")


def validate_multigoal_tasks(path: Path, endpoint_count: int,
                             frequency) -> None:
    lines = path.read_text().splitlines()
    if int(lines[0]) != 500 or len(lines) != 501:
        raise ValueError(f"{path}: MG-MAPD task-count mismatch")
    observed_lengths = set()
    for task_id, line in enumerate(lines[1:]):
        values = list(map(int, line.split()))
        release, goals = values[0], values[1:]
        if release != matrix.release_time(task_id, frequency):
            raise ValueError(f"{path}: task {task_id} has wrong release time")
        if not 1 <= len(goals) <= 5:
            raise ValueError(f"{path}: task {task_id} has {len(goals)} goals")
        if len(set(goals)) != len(goals):
            raise ValueError(f"{path}: task {task_id} repeats a goal")
        if any(not 0 <= goal < endpoint_count for goal in goals):
            raise ValueError(f"{path}: task {task_id} has an invalid goal")
        observed_lengths.add(len(goals))
    if observed_lengths != {1, 2, 3, 4, 5}:
        raise ValueError(f"{path}: not all goal counts 1..5 are represented")


def main() -> None:
    maps_checked = 0
    tasks_checked = 0
    endpoint_counts: dict[tuple[str, int], int] = {}

    for benchmark in matrix.BENCHMARKS:
        expected_rows, expected_cols, expected_obstacles = (
            LAYOUT_EXPECTATIONS[benchmark.name])
        for agents in benchmark.agent_counts:
            map_path = PACKAGE / "maps" / (
                f"benchmark_{benchmark.name}_a{agents}.map")
            grid, endpoint_count, actual_agents = read_map(map_path)
            if actual_agents != agents:
                raise ValueError(f"{map_path}: expected {agents} agents")
            if (len(grid), len(grid[0])) != (expected_rows, expected_cols):
                raise ValueError(f"{map_path}: unexpected layout dimensions")
            if sum(row.count("@") for row in grid) != expected_obstacles:
                raise ValueError(f"{map_path}: unexpected shelf-cell count")
            matrix.validate_well_formed(grid, map_path.name)
            endpoint_counts[(benchmark.name, agents)] = endpoint_count
            maps_checked += 1

            for label, frequency in matrix.FREQUENCIES:
                task_path = PACKAGE / "tasks" / (
                    f"benchmark_{benchmark.name}_a{agents}_f{label}.task")
                validate_standard_tasks(
                    task_path, endpoint_count, benchmark.task_count, frequency)
                tasks_checked += 1

        canonical_map = PACKAGE / "maps" / f"benchmark_{benchmark.name}.map"
        default_map = PACKAGE / "maps" / (
            f"benchmark_{benchmark.name}_a{benchmark.default_agents}.map")
        if canonical_map.read_bytes() != default_map.read_bytes():
            raise ValueError(f"{canonical_map}: canonical map alias mismatch")
        canonical_task = (
            PACKAGE / "tasks" / f"benchmark_{benchmark.name}.task")
        default_task = PACKAGE / "tasks" / (
            f"benchmark_{benchmark.name}_a{benchmark.default_agents}_"
            f"f{benchmark.default_frequency}.task")
        if canonical_task.read_bytes() != default_task.read_bytes():
            raise ValueError(f"{canonical_task}: canonical task alias mismatch")

    large_endpoints = endpoint_counts[("structured_large", 1000)]
    for task_count in matrix.PAPER_LARGE_TASK_COUNTS:
        path = PACKAGE / "tasks" / (
            f"benchmark_structured_large_a1000_t{task_count}_f100.task")
        validate_standard_tasks(
            path, large_endpoints, task_count, matrix.Fraction(100, 1))
        tasks_checked += 1

    for agents in (10, 20, 30, 40, 50):
        endpoint_count = endpoint_counts[("structured_small", agents)]
        for label, frequency in matrix.PAPER_MG_FREQUENCIES:
            path = PACKAGE / "tasks" / (
                f"benchmark_structured_small_mg_a{agents}_f{label}.task")
            validate_multigoal_tasks(path, endpoint_count, frequency)
            tasks_checked += 1

    tours_checked = 0
    for case in lkh.cases():
        _, _, _, _, homes, _ = lkh.read_map(case.map_path)
        tasks = lkh.read_tasks(case.task_path, "_mg_" in case.task_path.name)
        lkh.validate_tour(case.tour_path, len(homes), len(tasks))
        tours_checked += 1

    print("Paper-completeness audit passed.")
    print(f"well_formed_maps={maps_checked}")
    print(f"validated_task_files={tasks_checked}")
    print(f"validated_lkh_tours={tours_checked}")
    print("paper_small_mapd=complete")
    print("paper_medium_mapd=complete")
    print("paper_large_mapd=complete")
    print("paper_small_mg_mapd=complete")


if __name__ == "__main__":
    main()
