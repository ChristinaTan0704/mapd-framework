#!/usr/bin/env python3
"""Generate SMALL and MEDIUM warehouses with rectangular side endpoint banks.

The center shelf geometry follows the existing Kiva benchmarks. Unlike the
original local assets, every cell in four endpoint columns on each side is an
endpoint, matching the structured side banks visible in Figures 2 and 4 of the
warehouse papers. Agent homes replace selected side task endpoints.
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class InstanceSpec:
    name: str
    rows: int
    cols: int
    shelf_columns: int
    shelf_rows: int
    agents: int
    tasks: int
    frequency: int
    maxtime: int


SPECS = [
    InstanceSpec("small", 21, 35, 2, 5, 10, 500, 5, 5000),
    InstanceSpec("medium", 81, 101, 8, 20, 100, 1000, 50, 5000),
]

SHELF_WIDTH = 10
SHELF_X0 = 7
SHELF_X_STEP = 11
SHELF_Y0 = 2
SHELF_Y_STEP = 4


def build_map(spec: InstanceSpec, seed: int) -> tuple[list[list[str]], int]:
    grid = [["." for _ in range(spec.cols)] for _ in range(spec.rows)]

    for shelf_row in range(spec.shelf_rows):
        y = SHELF_Y0 + shelf_row * SHELF_Y_STEP
        for shelf_column in range(spec.shelf_columns):
            x0 = SHELF_X0 + shelf_column * SHELF_X_STEP
            for x in range(x0, x0 + SHELF_WIDTH):
                grid[y][x] = "@"
                grid[y - 1][x] = "e"
                grid[y + 1][x] = "e"

    # Four solid endpoint columns on each side form two rectangular banks.
    side_columns = [
        1, 2, 4, 5,
        spec.cols - 6, spec.cols - 5, spec.cols - 3, spec.cols - 2,
    ]
    side_endpoints = [(y, x) for y in range(1, spec.rows - 1)
                      for x in side_columns]
    for y, x in side_endpoints:
        grid[y][x] = "e"

    rng = random.Random(seed)
    for y, x in rng.sample(side_endpoints, spec.agents):
        grid[y][x] = "r"

    task_endpoint_count = sum(row.count("e") for row in grid)
    return grid, task_endpoint_count


def write_map(path: Path, spec: InstanceSpec, grid: list[list[str]],
              task_endpoint_count: int) -> None:
    lines = [
        f"{spec.rows},{spec.cols}",
        str(task_endpoint_count),
        str(spec.agents),
        str(spec.maxtime),
        *("".join(row) for row in grid),
    ]
    path.write_text("\n".join(lines) + "\n")


def write_tasks(path: Path, spec: InstanceSpec, task_endpoint_count: int,
                seed: int) -> None:
    rng = random.Random(seed + 1)
    lines = [str(spec.tasks)]
    for task_id in range(spec.tasks):
        pickup = rng.randrange(task_endpoint_count)
        delivery = rng.randrange(task_endpoint_count - 1)
        if delivery >= pickup:
            delivery += 1
        release_time = task_id // spec.frequency
        lines.append(f"{release_time} {pickup} {delivery} 0 0")
    path.write_text("\n".join(lines) + "\n")


def validate(spec: InstanceSpec, grid: list[list[str]],
             task_endpoint_count: int) -> None:
    assert len(grid) == spec.rows
    assert all(len(row) == spec.cols for row in grid)
    assert sum(row.count("@") for row in grid) == (
        spec.shelf_columns * spec.shelf_rows * SHELF_WIDTH)
    assert sum(row.count("e") for row in grid) == task_endpoint_count
    assert sum(row.count("r") for row in grid) == spec.agents


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    output_root = root / "data/Instances/structured"
    seed = 0

    for spec in SPECS:
        output_dir = output_root / spec.name
        output_dir.mkdir(parents=True, exist_ok=True)
        grid, task_endpoint_count = build_map(spec, seed)
        validate(spec, grid, task_endpoint_count)

        map_path = output_dir / f"kiva-structured-{spec.name}-{spec.agents}.map"
        task_path = output_dir / (
            f"kiva-structured-{spec.name}-{spec.tasks}-f{spec.frequency}.task")
        write_map(map_path, spec, grid, task_endpoint_count)
        write_tasks(task_path, spec, task_endpoint_count, seed)

        print(f"{spec.name}_map={map_path}")
        print(f"{spec.name}_tasks={task_path}")
        print(
            f"{spec.name}: {spec.cols}x{spec.rows}, "
            f"task_endpoints={task_endpoint_count}, agents={spec.agents}, "
            f"tasks={spec.tasks}, frequency={spec.frequency}")


if __name__ == "__main__":
    main()
