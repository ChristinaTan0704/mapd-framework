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

    workbook = ROOT / "all_paper_algorithms_comparison_2026-08-24.xlsx"
    workbook_valid = workbook.is_file() and zipfile.is_zipfile(workbook)
    if not workbook_valid:
        print(f"Invalid or missing workbook: {workbook}", file=sys.stderr)

    if missing or unexpected or not workbook_valid:
        return 1

    print(f"Repository manifest passed: {len(tracked)} approved files.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
