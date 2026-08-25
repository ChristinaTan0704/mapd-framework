#!/usr/bin/env python3
"""Generate a sparse MEDIUM-grid instance from the SMALL setup.

The grid uses the MEDIUM dimensions but retains the SMALL warehouse's two shelf
columns, five shelf rows, 302 task endpoints, 10 homes, and 500-task/f=5
workload. Side endpoints are distributed across two separated columns per side.
"""

from __future__ import annotations

import argparse
import random
from pathlib import Path


ROWS = 81
COLS = 101
SHELF_COLUMNS = 2
SHELF_ROWS = 5
SHELF_WIDTH = 10
SHELF_X0 = 29
SHELF_X_STEP = 32
SHELF_Y0 = 8
SHELF_Y_STEP = 16
MAXTIME = 5000
TASK_ENDPOINTS = 302
SIDE_ROWS_PER_COLUMN = 28


def build_map(agent_count: int, seed: int) -> tuple[list[list[str]], int]:
    grid = [["." for _ in range(COLS)] for _ in range(ROWS)]

    # SMALL has two columns by five rows of ten-cell shelf blocks. On the
    # MEDIUM grid, their horizontal pitch grows from 11 to 32 columns and their
    # vertical pitch from 4 to 16 rows.
    for shelf_row in range(SHELF_ROWS):
        y = SHELF_Y0 + shelf_row * SHELF_Y_STEP
        for shelf_column in range(SHELF_COLUMNS):
            x0 = SHELF_X0 + shelf_column * SHELF_X_STEP
            for x in range(x0, x0 + SHELF_WIDTH):
                grid[y][x] = "@"
                grid[y - 1][x] = "e"
                grid[y + 1][x] = "e"

    # Use two widely separated columns on each side. Select 28 evenly spaced
    # rows per column, producing 112 side stations. With ten homes replacing
    # ten of those task endpoints, the 200 shelf endpoints plus 102 side task
    # endpoints reproduce the SMALL map's total of 302.
    left_columns = [4, 16]
    right_columns = [COLS - 17, COLS - 5]
    side_rows = [
        round(1 + index * (ROWS - 3) / (SIDE_ROWS_PER_COLUMN - 1))
        for index in range(SIDE_ROWS_PER_COLUMN)
    ]
    if len(set(side_rows)) != SIDE_ROWS_PER_COLUMN:
        raise ValueError("side row spacing produced duplicate rows")

    left_endpoints = [(y, x) for y in side_rows
                      for x in left_columns]
    right_endpoints = [(y, x) for y in side_rows
                       for x in right_columns]
    if agent_count % 2 != 0:
        raise ValueError("agent_count must be even for a balanced side split")
    homes_per_side = agent_count // 2
    if homes_per_side > len(left_endpoints):
        raise ValueError(
            f"need {homes_per_side} homes per side but only "
            f"{len(left_endpoints)} side endpoints fit")

    for y, x in left_endpoints + right_endpoints:
        grid[y][x] = "e"

    rng = random.Random(seed)
    homes = (rng.sample(left_endpoints, homes_per_side) +
             rng.sample(right_endpoints, homes_per_side))
    for y, x in homes:
        grid[y][x] = "r"

    task_endpoint_count = sum(row.count("e") for row in grid)
    expected_task_endpoints = TASK_ENDPOINTS + 10 - agent_count
    if task_endpoint_count != expected_task_endpoints:
        raise ValueError(
            f"expected {expected_task_endpoints} task endpoints, got "
            f"{task_endpoint_count}")
    return grid, task_endpoint_count


def write_map(path: Path, grid: list[list[str]], task_endpoint_count: int,
              agent_count: int) -> None:
    lines = [
        f"{ROWS},{COLS}",
        str(task_endpoint_count),
        str(agent_count),
        str(MAXTIME),
        *("".join(row) for row in grid),
    ]
    path.write_text("\n".join(lines) + "\n")


def write_tasks(path: Path, task_endpoint_count: int, task_count: int,
                frequency: int, seed: int) -> None:
    rng = random.Random(seed + 1)
    lines = [str(task_count)]
    for task_id in range(task_count):
        pickup = rng.randrange(task_endpoint_count)
        delivery = rng.randrange(task_endpoint_count - 1)
        if delivery >= pickup:
            delivery += 1
        release_time = task_id // frequency
        lines.append(f"{release_time} {pickup} {delivery} 0 0")
    path.write_text("\n".join(lines) + "\n")


def validate(grid: list[list[str]], task_endpoint_count: int,
             agent_count: int) -> None:
    assert len(grid) == ROWS
    assert all(len(row) == COLS for row in grid)
    assert sum(row.count("@") for row in grid) == 100
    assert sum(row.count("e") for row in grid) == task_endpoint_count
    assert sum(row.count("r") for row in grid) == agent_count


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path,
                        default=Path("data/Instances/structured/sparse"))
    parser.add_argument("--agents", type=int, default=10)
    parser.add_argument("--tasks", type=int, default=500)
    parser.add_argument("--frequency", type=int, default=5)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    if args.agents <= 0 or args.tasks <= 0 or args.frequency <= 0:
        parser.error("agents, tasks, and frequency must be positive")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    grid, task_endpoint_count = build_map(args.agents, args.seed)
    validate(grid, task_endpoint_count, args.agents)

    map_path = args.output_dir / "kiva-sparse-medium-grid-small-load-10.map"
    task_path = args.output_dir / "kiva-sparse-medium-grid-small-load-500-f5.task"
    write_map(map_path, grid, task_endpoint_count, args.agents)
    write_tasks(task_path, task_endpoint_count, args.tasks,
                args.frequency, args.seed)

    print(f"map={map_path}")
    print(f"tasks={task_path}")
    print(f"size={COLS}x{ROWS}")
    print(f"task_endpoints={task_endpoint_count}")
    print(f"agent_homes={args.agents}")
    print(f"task_count={args.tasks}")
    print(f"release_frequency={args.frequency}")


if __name__ == "__main__":
    main()
