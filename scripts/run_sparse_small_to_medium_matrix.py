#!/usr/bin/env python3
"""Run the sparse SMALL-to-MEDIUM map experiment matrix.

10/20/30/40/50 agents, frequencies 0.2, 0.5, 1, 2, 5, 10, all, all 19 algorithms (25
method-rows with the ts-1/ts-2 Hungarian/LNS PBS-or-wPBS split), online and
semi-online, plus the offline TA methods against their packaged LKH tours.
1000-second per-job system timeout.

Also runs the endpoint-choice ablation: every job additionally runs once with
--endpoint_strategy forced to RETURN_TO_HOME and once forced to
NEAREST_AVAILABLE, alongside each method's own default strategy.

See experiment_matrix_lib.py for shared logic and run_*.py --help for options.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from experiment_matrix_lib import FAMILY_BY_NAME, run_matrix  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(run_matrix(FAMILY_BY_NAME["sparse_small_to_medium"]))
