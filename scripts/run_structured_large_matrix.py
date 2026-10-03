#!/usr/bin/env python3
"""Run the structured LARGE map experiment matrix.

200/400/600/800/1000 agents, frequencies 4, 10, 20, 40, 100, 200, all, all 19 algorithms
(25 method-rows with the ts-1/ts-2 Hungarian/LNS PBS-or-wPBS split), online
and semi-online, plus the offline TA methods against their packaged LKH
tours. 4-hour per-job system timeout.

See experiment_matrix_lib.py for shared logic and run_*.py --help for options.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from experiment_matrix_lib import FAMILY_BY_NAME, run_matrix  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(run_matrix(FAMILY_BY_NAME["structured_large"]))
