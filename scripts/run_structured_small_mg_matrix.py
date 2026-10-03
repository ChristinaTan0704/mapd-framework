#!/usr/bin/env python3
"""Run the structured SMALL multi-goal (MG) experiment matrix.

Reuses the structured_small .map files but reads the packaged _mg_ task/tour
files (ordered multi-goal task sequences). 10/20/30/40/50 agents, frequencies
0.2, 0.5, 1, 2, 5, 10, all, all 19 algorithms (25 method-rows with the
ts-1/ts-2 Hungarian/LNS PBS-or-wPBS split), online and semi-online, plus the
offline TA methods against their packaged MG LKH tours. 1000-second per-job
system timeout. No endpoint-strategy ablation for this one.

See experiment_matrix_lib.py for shared logic and run_*.py --help for options.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from experiment_matrix_lib import FAMILY_BY_NAME, run_matrix  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(run_matrix(FAMILY_BY_NAME["structured_small_mg"]))
