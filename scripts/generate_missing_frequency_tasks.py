#!/usr/bin/env python3
"""Generate task files for frequencies not covered by the original packaged
sweep in generate_benchmark_task_matrix.py.

Reuses that script's exact task-generation logic (generate_task_pairs,
release_time, write_tasks, validate_tasks) with the same seed=0, so the
pickup/delivery sequence for a given (benchmark, agent_count) is identical to
every other packaged frequency variant of that same map -- only the release
schedule differs. Endpoint counts are read from the already-published .map
files rather than regenerated, so existing maps are never touched.

Only writes the specific (benchmark, agent_count, frequency) combinations
listed in MISSING below; does not regenerate or modify any existing file.
"""

from __future__ import annotations

import sys
from fractions import Fraction
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate_benchmark_task_matrix import (  # noqa: E402
    BENCHMARKS, MAPS_DIR, TASKS_DIR, generate_task_pairs, validate_tasks,
    write_tasks, write_checksums,
)

# (benchmark_name, [agent_counts], {frequency_label: Fraction|None})
MISSING = (
    ("structured_medium", (100, 200, 300, 400, 500),
     {"20": Fraction(20, 1)}),
    ("structured_large", (200, 400, 600, 800, 1000),
     {"4": Fraction(4, 1), "20": Fraction(20, 1), "40": Fraction(40, 1),
      "200": Fraction(200, 1)}),
)


def read_endpoint_count(map_path: Path) -> int:
    lines = map_path.read_text().splitlines()
    return int(lines[1])


def main() -> None:
    benchmark_by_name = {b.name: b for b in BENCHMARKS}
    generated = 0
    for benchmark_name, agent_counts, frequencies in MISSING:
        benchmark = benchmark_by_name[benchmark_name]
        for agent_count in agent_counts:
            map_path = MAPS_DIR / f"benchmark_{benchmark_name}_a{agent_count}.map"
            endpoint_count = read_endpoint_count(map_path)
            tasks = generate_task_pairs(
                endpoint_count, benchmark.task_count, seed=0)
            validate_tasks(tasks, endpoint_count, map_path.name)

            for label, frequency in frequencies.items():
                task_path = TASKS_DIR / (
                    f"benchmark_{benchmark_name}_a{agent_count}_f{label}.task")
                if task_path.exists():
                    print(f"skip (already exists): {task_path.name}")
                    continue
                write_tasks(task_path, tasks, frequency)
                print(f"wrote {task_path.name}")
                generated += 1

    write_checksums()
    print(f"generated_task_files={generated}")


if __name__ == "__main__":
    main()
