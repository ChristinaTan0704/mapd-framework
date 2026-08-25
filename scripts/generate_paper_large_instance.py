#!/usr/bin/env python3
"""Generate a deterministic paper-style LARGE warehouse MAPD instance.

The original 187x153 LARGE asset referenced by Xu et al. (IROS 2022) is not
present in the local reference package. This generator reconstructs the paper's
stated geometry: 15 shelf blocks across and 76 five-cell shelf strips (38 rows
of ten-cell shelf blocks), with 1,000 non-task endpoints and an online workload
released at 100 tasks per timestep.
"""

from __future__ import annotations

import argparse
import random
from pathlib import Path


ROWS = 153
COLS = 187
SHELF_COLUMNS = 15
SHELF_ROWS = 38
SHELF_WIDTH = 10
# The 15 ten-cell blocks plus fourteen one-cell aisles occupy 164 columns.
# Starting at column 11 centers them in the 187-column grid, leaving margins
# of 11 cells on the left and 12 cells on the right.
SHELF_X0 = 11
SHELF_X_STEP = 11
SHELF_Y0 = 2
SHELF_Y_STEP = 4
MAXTIME = 50000


def build_map(agent_count: int, seed: int) -> tuple[list[list[str]], int]:
    grid = [["." for _ in range(COLS)] for _ in range(ROWS)]

    # Each ten-cell block represents two adjacent five-cell shelf strips. Thus
    # 38 rows of blocks produce the paper's 76 shelf strips vertically.
    for shelf_row in range(SHELF_ROWS):
        y = SHELF_Y0 + shelf_row * SHELF_Y_STEP
        for shelf_column in range(SHELF_COLUMNS):
            x0 = SHELF_X0 + shelf_column * SHELF_X_STEP
            for x in range(x0, x0 + SHELF_WIDTH):
                grid[y][x] = "@"
                grid[y - 1][x] = "e"
                grid[y + 1][x] = "e"

    # Four endpoint columns on each side, following the SMALL warehouse style.
    side_columns = [1, 2, 4, 5, COLS - 6, COLS - 5, COLS - 3, COLS - 2]
    side_endpoints = [(y, x) for y in range(1, ROWS - 1)
                      for x in side_columns]
    if agent_count > len(side_endpoints):
        raise ValueError(
            f"agent_count={agent_count} exceeds {len(side_endpoints)} "
            "available side endpoints")

    for y, x in side_endpoints:
        grid[y][x] = "e"

    rng = random.Random(seed)
    for y, x in rng.sample(side_endpoints, agent_count):
        grid[y][x] = "r"

    task_endpoint_count = sum(row.count("e") for row in grid)
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
    assert sum(row.count("@") for row in grid) == 5700
    assert sum(row.count("e") for row in grid) == task_endpoint_count
    assert sum(row.count("r") for row in grid) == agent_count


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path,
                        default=Path("data/Instances/large"))
    parser.add_argument("--agents", type=int, default=1000)
    parser.add_argument("--tasks", type=int, default=4000)
    parser.add_argument("--frequency", type=int, default=100)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    if args.agents <= 0 or args.tasks <= 0 or args.frequency <= 0:
        parser.error("agents, tasks, and frequency must be positive")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    grid, task_endpoint_count = build_map(args.agents, args.seed)
    validate(grid, task_endpoint_count, args.agents)

    map_path = args.output_dir / f"kiva-large-{args.agents}-paper-style.map"
    task_path = args.output_dir / (
        f"kiva-large-{args.tasks}-f{args.frequency}.task")
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
