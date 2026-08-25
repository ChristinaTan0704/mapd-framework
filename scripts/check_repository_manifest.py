#!/usr/bin/env python3
"""Reject missing runtime files or unrelated tracked artifacts."""

from __future__ import annotations

import subprocess
import sys
import zipfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

REQUIRED = {
    ".gitignore",
    ".github/workflows/linux-smoke.yml",
    "Makefile",
    "README.md",
    "all_paper_algorithms_comparison_2026-08-24.xlsx",
    "output/all_paper_algorithms_comparison_2026-08-24.xlsx",
    "driver.cpp",
    "history/2026-08-24_linux_server_handover.md",
    "inc/cbs.h",
    "inc/config.h",
    "inc/map_loader.h",
    "inc/path_planners.h",
    "inc/simulation.h",
    "inc/types.h",
    "scripts/check_repository_manifest.py",
    "scripts/run_server_matrix.py",
    "src/cbs.cpp",
    "src/map_loader.cpp",
    "src/path_planners.cpp",
    "src/simulation.cpp",
    "tests/multigoal-5.task",
    "tests/multigoal-10.tour",
}

BENCHMARKS = {
    "structured_small": (10, 20, 30, 40, 50),
    "structured_medium": (100, 200, 300, 400, 500),
    "structured_large": (200, 400, 600, 800, 1000),
    "sparse_small_to_medium": (10, 20, 30, 40, 50),
}
BENCHMARK_FREQUENCIES = (
    "0.2", "0.5", "1", "2", "5", "10", "50", "100", "500", "all",
)
EXTRA_BENCHMARK_FREQUENCIES = {
    "structured_medium": ("20",),
    "structured_large": ("4", "20", "40", "200"),
}

REQUIRED.update({
    "benchmark_instances/README.md",
    "benchmark_instances/PAPER_COMPLETENESS_AUDIT.md",
    "benchmark_instances/lkh_tours/README.md",
    "benchmark_instances/lkh_tours/SHA256SUMS",
    "benchmark_instances/lkh_tours/manifest.csv",
    "benchmark_instances/visualizations/benchmark_all_layouts_comparison.png",
    "benchmark_instances/visualizations/benchmark_delivery_heatmaps_comparison.png",
    "benchmark_instances/visualizations/benchmark_pickup_heatmaps_comparison.png",
    "scripts/generate_benchmark_task_matrix.py",
    "scripts/generate_benchmark_lkh_tours.py",
    "scripts/generate_paper_large_instance.py",
    "scripts/generate_sparse_medium_grid_instance.py",
    "scripts/generate_structured_small_medium.py",
    "scripts/render_pickup_heatmaps.py",
    "scripts/render_delivery_heatmaps.py",
    "scripts/render_warehouse_comparison.py",
    "scripts/validate_benchmark_completeness.py",
})

for benchmark, agent_counts in BENCHMARKS.items():
    REQUIRED.add(f"benchmark_instances/maps/benchmark_{benchmark}.map")
    REQUIRED.add(f"benchmark_instances/tasks/benchmark_{benchmark}.task")
    REQUIRED.add(
        f"benchmark_instances/visualizations/benchmark_{benchmark}.png")
    for agents in agent_counts:
        REQUIRED.add(
            f"benchmark_instances/maps/benchmark_{benchmark}_a{agents}.map")
        REQUIRED.add(
            "benchmark_instances/visualizations/pickup_heatmaps/"
            f"benchmark_{benchmark}_a{agents}_pickup_heatmap.png")
        REQUIRED.add(
            "benchmark_instances/visualizations/delivery_heatmaps/"
            f"benchmark_{benchmark}_a{agents}_delivery_heatmap.png")
        REQUIRED.add(
            "benchmark_instances/lkh_tours/"
            f"benchmark_{benchmark}_a{agents}_fall.tour")
        frequencies = (BENCHMARK_FREQUENCIES +
                       EXTRA_BENCHMARK_FREQUENCIES.get(benchmark, ()))
        for frequency in frequencies:
            REQUIRED.add(
                "benchmark_instances/tasks/"
                f"benchmark_{benchmark}_a{agents}_f{frequency}.task")

for task_count in (1000, 2000, 3000, 4000, 5000):
    REQUIRED.add(
        "benchmark_instances/tasks/"
        f"benchmark_structured_large_a1000_t{task_count}_f100.task")

for agents in (10, 20, 30, 40, 50):
    REQUIRED.add(
        "benchmark_instances/lkh_tours/"
        f"benchmark_structured_small_mg_a{agents}_fall.tour")
    for frequency in ("0.2", "0.5", "1", "2", "5", "10", "500", "all"):
        REQUIRED.add(
            "benchmark_instances/tasks/"
            f"benchmark_structured_small_mg_a{agents}_f{frequency}.task")

for agents in (10, 20, 30, 40, 50):
    REQUIRED.add(f"data/Instances/small/kiva-{agents}-500-5.map")
    REQUIRED.add(f"tour/{agents}-500.tour")
for frequency in ("0.2", "0.5", "1", "2", "5", "10", "500"):
    REQUIRED.add(f"data/Instances/small/kiva-{frequency}.task")


def main() -> int:
    tracked = set(subprocess.check_output(
        ["git", "ls-files"], cwd=ROOT, text=True).splitlines())
    missing = sorted(REQUIRED - tracked)
    unexpected = sorted(tracked - REQUIRED)

    if missing:
        print("Missing required tracked files:", file=sys.stderr)
        for path in missing:
            print(f"  {path}", file=sys.stderr)
    if unexpected:
        print("Unexpected tracked files:", file=sys.stderr)
        for path in unexpected:
            print(f"  {path}", file=sys.stderr)

    workbooks = (
        ROOT / "all_paper_algorithms_comparison_2026-08-24.xlsx",
        ROOT / "output" / "all_paper_algorithms_comparison_2026-08-24.xlsx",
    )
    invalid_workbooks = [
        workbook for workbook in workbooks
        if not workbook.is_file() or not zipfile.is_zipfile(workbook)
    ]
    for workbook in invalid_workbooks:
        print(f"Invalid or missing workbook: {workbook}", file=sys.stderr)

    if missing or unexpected or invalid_workbooks:
        return 1

    print(f"Repository manifest passed: {len(tracked)} approved files.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
