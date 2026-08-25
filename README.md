# Unified MAPD framework

This repository contains the current implementations and benchmark inputs for
TP/TPTS, CENTRAL, HBH, TA, Hungarian, and LNS-based MAPD algorithms.

The reviewed result workbook is
`all_paper_algorithms_comparison_2026-08-24.xlsx`. Other generated spreadsheets
and CSV files are intentionally excluded by `.gitignore`.

## Linux server setup

The recommended environment is Ubuntu 22.04 or 24.04 with a C++14 compiler.

```bash
sudo apt-get update
sudo apt-get install -y build-essential g++ make \
    libboost-program-options-dev libdlib-dev python3

cd MAPD_framework_imp
make clean
make -j"$(nproc)"
./mapd --help
```

The Makefile uses system Boost/dlib installations on Linux. For custom
installations, pass `BOOST_ROOT`, `BOOST_LIB_DIR`, and `DLIB_ROOT` to `make`.

The complete small benchmark is already under `data/Instances/small/`, and TA
tour files are under `tour/`. Keep both directories when copying the repository
to a server.

This branch uses an explicit `.gitignore` allow-list. To stage changes, use:

```bash
git add -A
python3 scripts/check_repository_manifest.py
```

Prefer `git add -A` or `git add .` over `git add *`: the shell expansion `*`
does not include hidden paths such as `.gitignore` and `.github/`. The manifest
check also rejects any unrelated file that is force-added despite the ignore
rules.

## Reproducibility gate

Always use seed `0` when comparing machines. A negative seed selects the
current time and is intentionally non-reproducible. First run the smoke matrix:

```bash
python3 scripts/run_server_matrix.py \
    --smoke \
    --seed 0 \
    --max-parallel 5 \
    --timeout 1800 \
    --output-dir server_results/smoke
```

This runs every configured method with 10 agents and frequency 0.2; the two
offline TA methods instead use 10 agents and the offline frequency-500 task
set. It checks:

- process success and 500/500 completed tasks;
- collision-check success;
- exact makespan/SWT agreement for non-LNS methods;
- informational makespan/SWT agreement for LNS methods.

LNS uses a one-CPU-second improvement budget, so another CPU/compiler may
execute a different number of LNS iterations. Its seed remains 0, but a small
LNS metric difference alone is not a failed port. Runtime is never expected to
match across machines.

Representative seed-0 baselines are:

| Method | Setup | Makespan | SWT |
|---|---:|---:|---:|
| TP-STA* | 10 / 0.2 | 2532 | 19417 |
| TPTS-STA* | 10 / 0.2 | 2532 | 14666 |
| CENTRAL-CBS | 10 / 0.2 | 2513 | 14039 |
| HBH+MLA* | 10 / 0.2 | 2532 | 14728 |
| Hungarian+PBS-MLA* | 10 / 0.2 | 2514 | 13925 |
| Hungarian+wPBS-MLA* | 10 / 0.2 | 2514 | 13722 |
| LNS(1s)+PBS-MLA* | 10 / 0.2 | 2514 | 13927 |
| LNS(1s)+wPBS-MLA* | 10 / 0.2 | 2513 | 13620 |
| TA-Prioritized-STA* | 10 / offline | 1052 | 263486 |
| TA-Hybrid-STA* | 10 / offline | 1053 | 264939 |

The runner contains the complete 27-method smoke baseline, including the eight
`task_sequence_limit=1` variants.

GitHub Actions runs the same repository-manifest, Linux build, and 27-method
smoke checks on every push and pull request. Its smoke logs are uploaded as the
`mapd-validation-results` workflow artifact.

It also runs every method on ten committed tasks containing five ordered goals
each:

```bash
python3 scripts/run_server_matrix.py \
    --multigoal-smoke --seed 0 --max-parallel 5 --timeout 1800 \
    --output-dir server_results/multigoal
```

The regression uses `tests/multigoal-5.task` and, for offline TA methods,
`tests/multigoal-10.tour`. Success requires completion of all ten complete
ordered goal sequences and a passing collision check.

## Multi-goal task support

Every supported algorithm consumes the same ordered `Task::goals` sequence.
The first goal has pickup/release-time semantics, intermediate goals must be
visited in order, and the final goal determines task completion and SWT. A
dummy or parking endpoint, when enabled, is selected only after the final task
goal through `choose_dummy_endpoint()`.

| Family | Multi-goal implementation |
|---|---|
| TP/TPTS STA* | Sequential STA* legs visit every ordered goal; task swapping compares arrival at the first goal and completion at the final goal. |
| TP/TPTS SIPP | One ordered SIPP request contains every goal. |
| HBH+MLA* | MLA* receives the complete goal sequence and completion is recorded at its final goal. |
| CENTRAL-CBS | CBS plans one segment at a time; `current_goal_index` triggers the next segment until the final goal is reached. |
| TA-Prioritized | The offline sequence planner iterates every goal of every assigned task before selecting parking. |
| TA-Hybrid | Group 2 plans to the first goal; subsequent Group-1 CBS calls advance through every remaining goal. |
| Hungarian/LNS with PP/PBS/wPBS | Assignment estimates, LNS costs, goal construction, and low-level MLA*/MLSIPP planning all iterate the full ordered goal list. |

Variable-length task input uses one line per task:

```text
release_time goal_endpoint_1 goal_endpoint_2 ... goal_endpoint_N
```

The committed regression contains ten tasks with five goals each. Across those
tasks, the ordered inter-goal legs total 724 steps, whereas travelling directly
from each first goal to its final goal totals only 204. This makes skipped
intermediate goals visible in the expected makespan/SWT baselines.

## Full paper-result matrix

After the smoke test passes, run all methods on agents 10, 20, 30, 40, and 50
and frequencies 0.2, 0.5, 1, 2, 5, 10, and 500:

```bash
python3 scripts/run_server_matrix.py \
    --seed 0 \
    --max-parallel 5 \
    --timeout 1800 \
    --output-dir server_results/full
```

The script enforces a maximum of five processes. It runs 885 jobs: every online
method on all 35 `(agents, frequency)` settings, and the two offline TA methods
on the five agent counts at frequency 500. LNS is explicitly configured with
`--lns_time 1`. Existing per-job JSON files are reused, so rerunning the same
command resumes an interrupted matrix. Add `--rerun` to discard the cache.

Return these files from the server:

- `server_results/full/results.csv`
- `server_results/full/results.json`
- all `server_results/full/*.log` files for audit/debugging

To run only the eight task-sequence-limit-1 variants, use:

```bash
python3 scripts/run_server_matrix.py \
    --methods 'Hungarian+PBS-MLA* (ts 1),Hungarian+wPBS-MLA* (ts 1),LNS(1s)+PBS-MLA* (ts 1),LNS(1s)+wPBS-MLA* (ts 1),Hungarian+PBS-MLSIPP (ts 1),Hungarian+wPBS-MLSIPP (ts 1),LNS(1s)+PBS-MLSIPP (ts 1),LNS(1s)+wPBS-MLSIPP (ts 1)' \
    --seed 0 --max-parallel 5 --timeout 1800 \
    --output-dir server_results/ts1
```

## Algorithm setup

All rows below also receive `--seed 0`. Preset defaults are
`task_sequence_limit=2`, `wpbs_replan_window=10`, `lns_time=1`,
`lns_no_improvement_limit=2000`, and CBS focal weight 1.0.

| Displayed method | Required command options | Mode |
|---|---|---|
| TP-STA* | `-a TP` | Online |
| TP-SIPP | `-a TP --single_agent MLSIPP` | Online extension |
| TPTS-STA* | `-a TPTS` | Online |
| TPTS-SIPP | `-a TPTS --single_agent MLSIPP` | Online extension |
| CENTRAL-CBS | `-a CENTRAL-CBS` | Online, every timestep |
| CENTRAL-fixed-CBS | `-a CENTRAL-fixed` | Online, event-triggered |
| HBH+MLA* | `-a HBH_MLA` | Online |
| TA-Prioritized-STA* | `-a TA_PRIORITIZED --tour tour/<agents>-500.tour` | Offline only |
| TA-Hybrid-STA* | `-a TA_HYBRID --tour tour/<agents>-500.tour` | Offline only |
| Hungarian+PBS-MLA* | `-a HUNGARIAN_PBS` | Online |
| Hungarian+wPBS-MLA* | `-a HUNGARIAN_wPBS` | Online |
| Hungarian+PBS-MLSIPP | `-a HUNGARIAN_PBS --single_agent MLSIPP` | Online |
| Hungarian+wPBS-MLSIPP | `-a HUNGARIAN_wPBS --single_agent MLSIPP` | Online |
| Hungarian+PP-SIPP | `-a HUNGARIAN_PBS --mapf PP --single_agent MLSIPP` | Online extension |
| LNS(1s)+PBS-MLA* | `-a LNS_PBS --lns_time 1` | Online |
| LNS(1s)+wPBS-MLA* | `-a LNS_wPBS --lns_time 1` | Online |
| LNS(1s)+PBS-MLSIPP | `-a LNS_PBS --lns_time 1 --single_agent MLSIPP` | Online extension |
| LNS(1s)+wPBS-MLSIPP | `-a LNS_wPBS --lns_time 1 --single_agent MLSIPP` | Online extension |
| LNS(1s)+PP-SIPP | `-a LNS_PBS --lns_time 1 --mapf PP --single_agent MLSIPP` | Online extension |

Append `--task_sequence_limit 1` to the eight Hungarian/LNS PBS or wPBS
MLA*/MLSIPP rows to produce their `(ts 1)` variants. Do not apply that option
to TP, TPTS, CENTRAL, HBH, TA, or PP; those paths do not use the PBS/wPBS task
sequence limit.

## Parking endpoint support

Map character `r` denotes a parking/home endpoint and `e` denotes a task
endpoint. All algorithm families can use parking endpoints, but they do so at
the point prescribed by their algorithm. Endpoint selection is dispatched by
the preset's `endpoint_strategy` and implemented centrally by
`Simulation::choose_dummy_endpoint()`.

| Methods | Strategy | How parking locations are used |
|---|---|---|
| TP, TPTS | `WAIT_OR_NEAREST_SAFE` | Wait at the current endpoint when safe; otherwise Path2 selects the nearest reachable safe task or parking endpoint. |
| HBH+MLA* | `WAIT_OR_NEAREST_FREE_NONTASK` | An idle blocking agent relocates to the nearest reachable safe parking endpoint. |
| CENTRAL-CBS, CENTRAL-fixed-CBS | `NEAREST_AVAILABLE` | Hungarian assignment may select any unreserved task or parking endpoint for a free agent. |
| TA-Prioritized, TA-Hybrid | `RETURN_TO_HOME` | Each agent returns to its own home parking endpoint after its offline sequence. |
| Hungarian/LNS with PBS or PP | `NEAREST_WITH_STRICT_EXCLUSIONS` | Dummy selection considers both task and parking endpoints after excluding reserved and unfinished-task locations. |
| Hungarian/LNS with wPBS | `PAIRWISE_TASK_THEN_HOME` | Select a pairwise-distinct task endpoint first, then any available parking endpoint, then remain in place as the final fallback. |

Parking support does not mean every agent is forced to park after each task.
For example, TP/TPTS terminally hold a safe delivery endpoint and invoke Path2
only when waiting there would block another task; this preserves the original
algorithm instead of adding a Hungarian/LNS-style dummy path.

Every method can also be run with the common `NEAREST_AVAILABLE` policy:

```bash
./mapd -m <map> -t <tasks> -a <preset> \
  --endpoint_strategy NEAREST_AVAILABLE --seed 0
```

For non-CENTRAL methods, `NEAREST_AVAILABLE` applies the method's existing task
goal protections and excludes endpoints reserved earlier in the same planning
batch or permanently held by another committed path. The requesting agent may
retain its current delivery endpoint when it is otherwise available. CENTRAL
continues to use the reservation set constructed by its Hungarian assignment
because its free-agent paths are planned jointly.

For the two offline TA methods, every unplanned agent's home is represented as
a permanent reservation during prioritized planning. Consequently, the
requesting agent's own home is its only guaranteed available parking endpoint;
the override preserves that invariant instead of allowing unsafe home swaps.

The matrix runner exposes the same override as
`--endpoint-strategy NEAREST_AVAILABLE`. Preset behavior remains unchanged
when this option is omitted.

Example single run:

```bash
./mapd \
  -m data/Instances/small/kiva-10-500-5.map \
  -t data/Instances/small/kiva-0.2.task \
  -a HUNGARIAN_PBS \
  --single_agent MLSIPP \
  --seed 0 \
  --task_sequence_limit 1 \
  -s 1
```

Use `--mode SEMI_ONLINE --semi_online_lookahead_batches N` only for online
presets when evaluating semi-online information. TA-Prioritized and TA-Hybrid
reject non-offline modes by design.

## Result acceptance checklist

A server result is valid only when every row has:

1. exit status `ok`;
2. `Tasks completed: 500/500`;
3. no collision error;
4. the intended seed and options recorded in `results.json`.

Compare the smoke makespan/SWT values with the baselines embedded in the
runner. Compare runtime only between runs made on the same server under similar
load.
