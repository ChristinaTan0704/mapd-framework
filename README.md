# Unified MAPD framework

This repository contains the current implementations and benchmark inputs for
TP/TPTS, CENTRAL, HBH, TA, Hungarian, and LNS-based MAPD algorithms.

The reviewed result workbook is
`all_paper_algorithms_comparison_2026-08-24.xlsx`. A second tracked copy is
available at `output/all_paper_algorithms_comparison_2026-08-24.xlsx` for
output collection. Other generated spreadsheets and CSV files are
intentionally excluded by `.gitignore`.

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

## Generated paper benchmark package

The finalized generated benchmark suite is under [`benchmark_instances/`](benchmark_instances/README.md).
It is separate from the legacy `data/Instances/small/` inputs used by
`scripts/run_server_matrix.py`.

| Layout family | Grid | Agent-count variants | Standard tasks |
|---|---:|---|---:|
| Structured SMALL | 35 x 21 | 10, 20, 30, 40, 50 | 500 |
| Structured MEDIUM | 101 x 81 | 100, 200, 300, 400, 500 | 1,000 |
| Structured LARGE | 187 x 153 | 200, 400, 600, 800, 1,000 | 4,000 |
| Sparse SMALL-to-MEDIUM | 101 x 81 | 10, 20, 30, 40, 50 | 500 |

The package contains:

- 20 agent-count-specific maps plus four canonical aliases;
- 245 validated standard and multi-goal task files, including the complete
  release-frequency matrix (`0.2`, `0.5`, `1`, `2`, `5`, `10`, `50`, `100`,
  `500`, and `all` where applicable);
- the paper LARGE workloads with 1,000--5,000 tasks at `f=100`;
- structured-SMALL multi-goal workloads for all five SMALL agent counts;
- layout, pickup-density, and delivery-density visualizations; and
- 25 offline LKH3 tours: one for each of the 20 standard map/agent variants
  and five structured-SMALL multi-goal variants.

In a filename such as `benchmark_structured_small_a10_fall.task`, `fall`
means frequency `all`: every task is released at timestep zero. The offline TA
algorithms require one LKH tour per agent count because agent nodes occupy
`1..A` and task nodes begin at `A+1`. Separate online-frequency tours are not
packaged.

LKH tours were generated from the bundled
`reference_code/TA-Prioritized/LKH3` source by
`scripts/generate_benchmark_lkh_tours.py`. All 25 tours cover every expected
agent/task node exactly once and pass their checksum manifest. TA-Prioritized
completed the minimum-agent offline structured SMALL (500/500 tasks) and
MEDIUM (1,000/1,000 tasks) cases with collision checking passed. The SPARSE
minimum-agent tour loaded successfully, but its subsequent prioritized path
planning failed at task 460; this is not a tour-format or node-coverage error.
The long-running LARGE path-planning smoke was stopped manually after the tour
had loaded successfully.

Validate the complete package with:

```bash
python3 scripts/validate_benchmark_completeness.py
python3 scripts/check_repository_manifest.py
(cd benchmark_instances/lkh_tours && shasum -a 256 -c SHA256SUMS)
```

See [`benchmark_instances/PAPER_COMPLETENESS_AUDIT.md`](benchmark_instances/PAPER_COMPLETENESS_AUDIT.md)
for the paper-coverage audit and reconstruction/provenance limitations.

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

Exact ties between equally good search nodes follow `--tie_breaking`:

- `reference` (default) uses the comparisons of the original reference
  implementations (non-strict `f`/`g` tests, remaining ties left to the heap)
  and reproduces their results;
- `random` gives each search node one immutable pseudo-random tie key from
  `--seed`. Comparators first apply the algorithm's normal criteria and use the
  key only for an exact tie, so different seeds explore different equal-cost
  alternatives. This was the behavior from `e584564` until this option existed.

The rule applies to STA*, assignment-cost A*, dummy-path A*, MLA*, MLSIPP and
CBS/ECBS. The seed also controls LNS in both modes.

```bash
python3 scripts/run_server_matrix.py \
    --smoke \
    --seed 0 \
    --max-parallel 5 \
    --timeout 1000 \
    --output-dir server_results/smoke
```

This runs every configured method with 10 agents and frequency 0.2; the two
offline TA methods instead use 10 agents and the offline frequency-500 task
set. It checks:

- process success and 500/500 completed tasks;
- collision-check success;
- the executable also prints a task-order check (no pickup before release, no
  completion before pickup), which `scripts/check_combinations.py` requires;
- exact makespan/SWT agreement for non-LNS methods;
- informational makespan/SWT agreement for LNS methods.

LNS uses a one-CPU-second improvement budget, so another CPU/compiler may
execute a different number of LNS iterations. Its seed remains 0, but a small
LNS metric difference alone is not a failed port. Runtime is never expected to
match across machines.

Representative seed-0 baselines are:

| Method | Setup | Makespan | SWT |
|---|---:|---:|---:|
| TP-STA* | 10 / 0.2 | 2541 | 18884 |
| TPTS-STA* | 10 / 0.2 | 2516 | 14305 |
| CENTRAL-CBS | 10 / 0.2 | 2513 | 14122 |
| HBH+MLA* | 10 / 0.2 | 2532 | 14728 |
| Hungarian+PBS-MLA* | 10 / 0.2 | 2512 | 13684 |
| Hungarian+wPBS-MLA* | 10 / 0.2 | 2513 | 13762 |
| LNS(1s)+PBS-MLA* | 10 / 0.2 | 2512 | 13681 |
| LNS(1s)+wPBS-MLA* | 10 / 0.2 | 2513 | 13637 |
| TA-Prioritized-STA* | 10 / offline | 1052 | 263485 |
| TA-Hybrid-STA* | 10 / offline | 1050 | 263910 |

The runner contains the complete 29-method smoke baseline, including the eight
`task_sequence_limit=1` variants.

The structured-small 10-agent, frequency-0.2 validation for the added
low-level combinations produced the following seed-0 results. Every row
completed 500/500 tasks and passed the collision check.

| Method | Makespan | SWT |
|---|---:|---:|
| HBH+MLA* (comparison) | 2546 | 14849 |
| HBH+MLSIPP | 2546 | 14978 |
| HBH+2-segment SIPP | 2535 | 15349 |
| TP+MLSIPP | 2543 | 19207 |
| TPTS+MLSIPP | 2535 | 14265 |

The table above is the seed-0 randomized-tie validation (`--tie_breaking
random`). It was run twice, and every method reproduced the same makespan and
SWT on the second run. With `--tie_breaking random`, use a different
non-negative `--seed` to obtain a different but repeatable equal-cost search
ordering. The exact smoke baselines in `scripts/run_server_matrix.py` use the
default reference tie-breaking.

GitHub Actions runs the same repository-manifest, Linux build, and 29-method
smoke checks on every push and pull request. Its smoke logs are uploaded as the
`mapd-validation-results` workflow artifact.

Component combinations are validated before a run starts: `validate_config()`
(`src/config_validate.cpp`) lists every supported pairing of assignment and
MAPF method with its triggers, single-agent searches, endpoint strategies and
modes, and rejects anything else with the supported values. The combination
check runs every preset against every override and fails if a combination is
neither rejected nor completed collision-free:

```bash
python3 scripts/check_combinations.py --max-parallel 20
```

It also runs every method on ten committed tasks containing five ordered goals
each:

```bash
python3 scripts/run_server_matrix.py \
    --multigoal-smoke --seed 0 --max-parallel 5 --timeout 1000 \
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
| TP/TPTS MLSIPP | One ordered MLSIPP request contains every goal. |
| HBH+MLA* or HBH+MLSIPP | The selected low-level planner receives the complete goal sequence and completion is recorded at its final goal. |
| HBH+2-segment SIPP | Each ordered goal leg is planned by a separate SIPP request; only the final leg reserves its endpoint through the planning horizon. |
| CENTRAL-CBS | An agent is planned to its matched pickup; once it carries the task, every remaining goal is planned (`GOAL_REACHED`). |
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
    --timeout 1000 \
    --output-dir server_results/full
```

The script enforces a maximum of five processes. It runs 885 jobs: every online
method on all 35 `(agents, frequency)` settings, and the two offline TA methods
on the five agent counts at frequency 500. LNS is explicitly configured with
`--lns_time 1`. Existing per-job JSON files are reused, so rerunning the same
command resumes an interrupted matrix. Add `--rerun` to discard the cache.

Every executable receives two internal wall-clock limits. By default, the
runner sets the whole-run `--runtime_limit` to five seconds less than its
external `--timeout` (995 versus 1000 seconds) and sets
`--pathfinding_runtime_limit` to 600 seconds. The pathfinding deadline resets
for every assignment/path-planning cycle. CBS, TA, PBS/wPBS, STA*, MLA*,
MLSIPP, CENTRAL assignment search, endpoint Path2, and LNS immediately raise
`Pathfinding runtime timeout` or `System runtime timeout` errors when the
respective deadline expires. The external timeout remains the final safeguard
for code that cannot check the deadline. Runner options
`--runtime-limit 0` and `--pathfinding-runtime-limit 0` disable the respective
internal limits.

Return these files from the server:

- `server_results/full/results.csv`
- `server_results/full/results.json`
- all `server_results/full/*.log` files for audit/debugging

To run only the eight task-sequence-limit-1 variants, use:

```bash
python3 scripts/run_server_matrix.py \
    --methods 'Hungarian+PBS-MLA* (ts 1),Hungarian+wPBS-MLA* (ts 1),LNS(1s)+PBS-MLA* (ts 1),LNS(1s)+wPBS-MLA* (ts 1),Hungarian+PBS-MLSIPP (ts 1),Hungarian+wPBS-MLSIPP (ts 1),LNS(1s)+PBS-MLSIPP (ts 1),LNS(1s)+wPBS-MLSIPP (ts 1)' \
    --seed 0 --max-parallel 5 --timeout 1000 \
    --output-dir server_results/ts1
```

## Nineteen-method generated-benchmark experiments

`--base-methods` selects the 19 method rows listed in the next section and
omits the eight additional `(ts 1)` variants. For these commands, the 17 online
methods run every value supplied through `--frequencies`; TA-Prioritized and
TA-Hybrid instead run once per agent count using `--offline-frequency all` and
the matching `_fall` LKH tour.

Run the structured SMALL paper matrix with:

```bash
python3 scripts/run_server_matrix.py --base-methods \
    --agents 10,20,30,40,50 --frequencies 0.2,0.5,1,2,5,10,all \
    --offline-frequency all \
    --map-template 'benchmark_instances/maps/benchmark_structured_small_a{agents}.map' \
    --task-template 'benchmark_instances/tasks/benchmark_structured_small_a{agents}_f{frequency}.task' \
    --tour-template 'benchmark_instances/lkh_tours/benchmark_structured_small_a{agents}_fall.tour' \
    --seed 0 --max-parallel 5 --timeout 1000 \
    --output-dir server_results/benchmark_structured_small
```

Run the structured MEDIUM paper matrix with:

```bash
python3 scripts/run_server_matrix.py --base-methods \
    --agents 100,200,300,400,500 --frequencies 50 \
    --offline-frequency all \
    --map-template 'benchmark_instances/maps/benchmark_structured_medium_a{agents}.map' \
    --task-template 'benchmark_instances/tasks/benchmark_structured_medium_a{agents}_f{frequency}.task' \
    --tour-template 'benchmark_instances/lkh_tours/benchmark_structured_medium_a{agents}_fall.tour' \
    --seed 0 --max-parallel 5 --timeout 1000 \
    --output-dir server_results/benchmark_structured_medium
```

Run the structured LARGE paper workloads with:

```bash
python3 scripts/run_server_matrix.py --base-methods \
    --agents 1000 \
    --frequencies t1000_f100,t2000_f100,t3000_f100,t4000_f100,t5000_f100 \
    --offline-frequency all \
    --map-template 'benchmark_instances/maps/benchmark_structured_large_a{agents}.map' \
    --task-template 'benchmark_instances/tasks/benchmark_structured_large_a{agents}_{frequency}.task' \
    --offline-task-template 'benchmark_instances/tasks/benchmark_structured_large_a{agents}_fall.task' \
    --tour-template 'benchmark_instances/lkh_tours/benchmark_structured_large_a{agents}_fall.tour' \
    --seed 0 --max-parallel 5 --timeout 1000 \
    --output-dir server_results/benchmark_structured_large
```

The LARGE command runs each online method on all five task-count workloads and
runs each offline TA method once on the packaged 4,000-task `fall` workload.

Run the sparse SMALL-to-MEDIUM matrix with:

```bash
python3 scripts/run_server_matrix.py --base-methods \
    --agents 10,20,30,40,50 --frequencies 0.2,0.5,1,2,5,10,all \
    --offline-frequency all \
    --map-template 'benchmark_instances/maps/benchmark_sparse_small_to_medium_a{agents}.map' \
    --task-template 'benchmark_instances/tasks/benchmark_sparse_small_to_medium_a{agents}_f{frequency}.task' \
    --tour-template 'benchmark_instances/lkh_tours/benchmark_sparse_small_to_medium_a{agents}_fall.tour' \
    --seed 0 --max-parallel 5 --timeout 1000 \
    --output-dir server_results/benchmark_sparse_small_to_medium
```

The sparse maps use a 10,000-step horizon: with 10 agents the paths are about
three times longer than on the SMALL grid, and offline TA-Prioritized needs a
makespan of about 6,400.

Known failure: **TA-Hybrid-STA\* with 50 agents on the sparse `fall` workload
exceeds the 1000-second system limit** (`System runtime timeout in TA-Hybrid
min-cost-flow shortest path`) for every endpoint strategy. Its Group-2 step
solves an exact time-expanded min-cost flow over all 8,549 cells; even with the
provably exact window it needs about 50 shortest-path searches per assignment
over hundreds of layers. Record these rows as timeouts. With 10 agents the same
workload completes in about 650 seconds. Agent counts 20-40 and the other
sparse frequencies have not yet been verified for this method.

Run the structured-SMALL multi-goal paper matrix with:

```bash
python3 scripts/run_server_matrix.py --base-methods \
    --agents 10,20,30,40,50 --frequencies 2,5,10 \
    --offline-frequency all \
    --map-template 'benchmark_instances/maps/benchmark_structured_small_a{agents}.map' \
    --task-template 'benchmark_instances/tasks/benchmark_structured_small_mg_a{agents}_f{frequency}.task' \
    --tour-template 'benchmark_instances/lkh_tours/benchmark_structured_small_mg_a{agents}_fall.tour' \
    --seed 0 --max-parallel 5 --timeout 1000 \
    --output-dir server_results/benchmark_structured_small_mg
```

These runs are resumable: existing per-job JSON files are reused unless
`--rerun` is supplied. Review `results.csv`, `results.json`, and every `.log`
file in the selected output directory before accepting an experiment matrix.

## Component switches

An algorithm is a choice of its nine components. Each is one option; a
`--config` file sets them as `option = value` lines, and command-line options
take precedence. `configs/table1/` holds the Table 1 algorithms (all except
RMCA) and `configs/variants/` the extra combinations, every component written
out:

```bash
./mapd -m data/Instances/small/kiva-10-500-5.map -t data/Instances/small/kiva-1.task \
    --config configs/table1/TPTS.cfg --single_agent MLSIPP
```

`--algo` names a preset instead; its components are the defaults that the
options override. The experiment scripts run the configuration files.

### Quick reference

| # | Component | (1) How to set | (2) Values | (3) Pay attention to |
|---|---|---|---|---|
| 1 | When to call task assignment | `--assign_trigger` | `ON_FREE_WAITS`<br>`EVERY_TIMESTEP`<br>`ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE` (alias `ON_NEW_TASK_OR_AGENT_BECOMES_FREE`)<br>`ONCE` | `ON_FREE_WAITS` only with `--replan_on NONE` (TP/TPTS/HBH as published).<br>`ONCE` only for LKH3, and needs `OFFLINE`.<br>`EVERY_TIMESTEP` assigns and advances every step, which is slow on large maps. |
| 2 | Task assignment | `--assign_method` | `DECOUPLED_GREEDY` (TP)<br>`DECOUPLED_GREEDY_SWAPS` (TPTS)<br>`CENTRALIZED_GREEDY` (HBH)<br>`CENTRAL_HUNGARIAN`<br>`REPEATED_HUNGARIAN`<br>`REPEATED_HUNGARIAN_LNS`<br>`LKH3_TSP`<br>`LKH3_TSP_REASSIGN` | `LKH3_*` need `--tour <file>` and `OFFLINE`.<br>`CENTRAL_HUNGARIAN` with `CBS` or `wPBS` (not `PBS`/`PP`: they deadlock with 50 agents); with `CBS` it needs `GOAL_REACHED` or `EVERY_TIMESTEP` replans.<br>LNS budget is `--lns_time` (wall-clock seconds), so its results vary slightly between runs. |
| 3 | When to plan paths | `--replan_on` | `NONE`, or a comma-separated set of `AFTER_ASSIGNMENT`, `EVERY_TIMESTEP`, `GOAL_REACHED`, `PERIODIC` (e.g. `AFTER_ASSIGNMENT,PERIODIC`) | `NONE` = plan while assigning; only TP/TPTS/HBH with `ON_FREE_WAITS`.<br>`wPBS` needs `PERIODIC` or `EVERY_TIMESTEP`.<br>`CENTRAL_HUNGARIAN` and TA-Hybrid need `GOAL_REACHED` or `EVERY_TIMESTEP`. |
| 4 | Which agents to plan | `--agent_selection` | `ALL`<br>`CHANGED` | `wPBS` and `PP_TASK_SEQUENCE`: `ALL` only.<br>`CBS`, TA-Hybrid and TP/TPTS/HBH as published: `CHANGED` only (one CBS over every agent's goal sequence does not scale to 50 agents).<br>`PBS` and `PP`: either. |
| 5 | High-level planner | `--mapf` | `PP` (alias `PP_PER_TASK`)<br>`PP_TASK_SEQUENCE`<br>`CBS`<br>`PBS`<br>`wPBS`<br>`TA_HYBRID_TWO_GROUP` | `PBS`/`PP`/`CBS` not with `OFFLINE` (`CBS` after `CENTRAL_HUNGARIAN` is fine); with a very large semi-online lookahead `PBS`/`PP` collide (exit status 2).<br>`PBS`/`wPBS`/`CBS` plan the first `--task_sequence_limit` tasks (default 2; the Excel originals used 1).<br>`PP_TASK_SEQUENCE` only with `LKH3_TSP` + `ONCE` + `OFFLINE`.<br>`TA_HYBRID_TWO_GROUP` only with `LKH3_TSP_REASSIGN`. |
| 5.1 | wPBS window | `--wpbs_replan_window` | integer, default 10 | At least 2 for `wPBS`.<br>Also the `PERIODIC` replan interval for any planner. |
| 6 | Low-level planner | `--single_agent` | `STA`<br>`MLA`<br>`MLSIPP` (alias `SIPP`)<br>`SIPP2` (alias `2SEG_SIPP`)<br>`SEQ_STA` | TA-Hybrid accepts only `STA`, but uses its own searches (the value is a label).<br>`PP_TASK_SEQUENCE` only `SEQ_STA`.<br>`wPBS` cannot use `SIPP2`. |
| 7 | Task endpoint | `--endpoint_strategy`<br>`--dummy_path 0/1` | `WAIT_OR_NEAREST_SAFE`<br>`WAIT_OR_NEAREST_FREE_NONTASK`<br>`NEAREST_AVAILABLE`<br>`NEAREST_WITH_STRICT_EXCLUSIONS`<br>`PAIRWISE_TASK_THEN_HOME`<br>`RETURN_TO_HOME` | `WAIT_*` only for TP/TPTS/HBH as published, which cannot use `STRICT` or `PAIRWISE` (deadlock).<br>`PBS`/`PP`/`CBS`: `STRICT`, `NEAREST_AVAILABLE` or `HOME`.<br>`PAIRWISE` only with `wPBS`.<br>`CENTRAL` parks idle agents with the strategy: `NEAREST_AVAILABLE` or `HOME` (and `PAIRWISE` with `wPBS`).<br>`--dummy_path 0` only for `CENTRAL` and TA-Hybrid; everything else needs 1. |
| 8 | Task setup | `--mode` | `ONLINE`<br>`SEMI_ONLINE` (with `--semi_online_lookahead_batches k`, default 1)<br>`OFFLINE` | `LKH3_*` only `OFFLINE`.<br>`PBS`/`PP`/`CBS` not `OFFLINE` (except after `CENTRAL_HUNGARIAN`, which matches released tasks only).<br>TP/TPTS/HBH as published can run `OFFLINE`, but do not use the advance knowledge and perform poorly. |
| 9 | Task type | `--task_type` | `MAPD`<br>`MG_MAPD` (default) | `MAPD` checks that every task in the task file has exactly 2 goals, otherwise errors.<br>Multi-goal task files need `MG_MAPD`. |

General:
- `--config configs/table1/TP.cfg` sets all nine at once; options given on the
  command line override the file, so you can change just one or two. `-a` is
  optional.
- Unsupported combinations stop before the run with an error that lists the
  values that work; the full rules are in `src/config_validate.cpp`.
- `mapd` exits with status 2 when the collision or task-order check fails;
  check the exit status in scripts.
- `./mapd --help` lists every option and its values.

### Values in detail

`./mapd --help` lists every option with its values. The values of the nine
components, and the Table 1 configurations (`configs/table1/`) that use them:

**1. When to call task assignment** — `--assign_trigger`

| Value | Meaning | Used by |
|---|---|---|
| `ON_FREE_WAITS` | an agent is free and waiting (one agent per call) | TP, TPTS, HBH |
| `EVERY_TIMESTEP` | every timestep | CENTRAL, TA-Hybrid |
| `ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE` | a task is released, an assignment method deferred a task, or an agent became free (alias `ON_NEW_TASK_OR_AGENT_BECOMES_FREE`) | CENTRAL-fixed, LNS-PBS, LNS-wPBS |
| `ONCE` | once, at time 0 | TA-Prioritized |

**2. Task assignment** — `--assign_method`

| Value | Meaning | Used by |
|---|---|---|
| `DECOUPLED_GREEDY` | each free agent takes its nearest task | TP |
| `DECOUPLED_GREEDY_SWAPS` | as above; may take over a task another agent would reach later | TPTS |
| `CENTRALIZED_GREEDY` | the globally nearest (agent, task) pairs first | HBH |
| `CENTRAL_HUNGARIAN` | Hungarian matching of agents to pickups or parking endpoints (chosen by the endpoint strategy); an agent owns a task once it picks it up, until then it is planned to the pickup and rematched at every assignment | CENTRAL, CENTRAL-fixed |
| `REPEATED_HUNGARIAN` | repeated Hungarian matching of agents to task sequences | Hungarian-PBS, Hungarian-wPBS (variants) |
| `REPEATED_HUNGARIAN_LNS` | as above, improved by LNS (`--lns_time`, default 1 s) | LNS-PBS, LNS-wPBS |
| `LKH3_TSP` | task sequences from an LKH3 tour (`--tour`) | TA-Prioritized |
| `LKH3_TSP_REASSIGN` | as above; free agents may take over unstarted tasks | TA-Hybrid |

**3. When to plan paths** — `--replan_on` (comma-separated set)

| Value | Meaning | Used by |
|---|---|---|
| `NONE` | TP, TPTS and HBH plan while assigning | TP, TPTS, HBH |
| `AFTER_ASSIGNMENT` | after every task assignment | LNS-PBS, TA-Prioritized; with others below |
| `EVERY_TIMESTEP` | every timestep | CENTRAL |
| `GOAL_REACHED` | an agent picked up a task or reached a later goal with goals left (plans its next leg) | CENTRAL-fixed, TA-Hybrid (`AFTER_ASSIGNMENT,GOAL_REACHED`) |
| `PERIODIC` | every `--wpbs_replan_window` steps | LNS-wPBS (`AFTER_ASSIGNMENT,PERIODIC`) |

**4. Which agents to plan** — `--agent_selection`

| Value | Meaning | Used by |
|---|---|---|
| `ALL` | every agent | LNS-PBS, LNS-wPBS, TA-Prioritized |
| `CHANGED` | agents whose task goals changed, or whose final cell another task needs; the others keep their paths | TP, TPTS, HBH, CENTRAL, CENTRAL-fixed, TA-Hybrid |

**5. High-level planner** — `--mapf`

| Value | Meaning | Used by |
|---|---|---|
| `PP` (alias `PP_PER_TASK`) | prioritized planning, one task at a time | TP, TPTS, HBH |
| `PP_TASK_SEQUENCE` | prioritized planning of each agent's whole task sequence | TA-Prioritized |
| `CBS` | conflict-based search (ICBS) over the selected agents' goal sequences, first `--task_sequence_limit` tasks | CENTRAL, CENTRAL-fixed |
| `PBS` | PBS over each agent's first `--task_sequence_limit` tasks (default 2) | LNS-PBS |
| `wPBS` | PBS with a planning window (5.1) | LNS-wPBS |
| `TA_HYBRID_TWO_GROUP` | TA-Hybrid's two-group planning | TA-Hybrid |

**5.1 Planning window** — `--wpbs_replan_window w` (default 10, at least 2 for
`wPBS`): wPBS plans and resolves conflicts within w steps; also the interval of
`PERIODIC` replans.

**6. Low-level planner** — `--single_agent`

| Value | Meaning | Used by |
|---|---|---|
| `STA` | space-time A* (MLA* with one goal per group) | TP, TPTS, CENTRAL, CENTRAL-fixed |
| `MLA` | multi-label A*, one task per group | HBH, LNS-PBS, LNS-wPBS |
| `MLSIPP` (alias `SIPP`) | multi-label SIPP over the whole goal sequence | the MLSIPP variants |
| `SIPP2` (alias `2SEG_SIPP`) | SIPP, one segment per goal | — |
| `SEQ_STA` | sequential STA* (`PP_TASK_SEQUENCE` only) | TA-Prioritized |

`TA_HYBRID_TWO_GROUP` and `PP_TASK_SEQUENCE` use their own searches. Under
`CBS` the configured search also obeys the high-level constraints and breaks
ties between equally short paths toward fewer conflicts with the other CBS
agents' paths.

**7. Task endpoint** — `--endpoint_strategy` and `--dummy_path`

| Value | Meaning | Used by |
|---|---|---|
| `WAIT_OR_NEAREST_SAFE` | wait; move to the nearest safe endpoint when waiting is unsafe | TP, TPTS |
| `WAIT_OR_NEAREST_FREE_NONTASK` | wait; move to the nearest free parking endpoint when blocking | HBH |
| `NEAREST_AVAILABLE` | the nearest endpoint not reserved by another agent | CENTRAL, CENTRAL-fixed |
| `NEAREST_WITH_STRICT_EXCLUSIONS` | the nearest task endpoint, excluding other agents' task goals and old endpoints | LNS-PBS |
| `PAIRWISE_TASK_THEN_HOME` | pairwise-distinct task endpoints first, then parking | LNS-wPBS |
| `RETURN_TO_HOME` | the agent's own home endpoint | TA-Prioritized, TA-Hybrid |

`--dummy_path 1` plans a path to the chosen endpoint after the last task (all
but CENTRAL); `0` stops at the last task goal (CENTRAL, CENTRAL-fixed). An
agent CENTRAL parks, or matched to a pickup it does not own yet, gets no
dummy endpoint: that endpoint is its final goal. See
[Parking endpoint support](#parking-endpoint-support).

**8. Task setup** — `--mode`

| Value | Meaning | Used by |
|---|---|---|
| `ONLINE` | tasks become known at their release times | the online methods |
| `SEMI_ONLINE` | the next `--semi_online_lookahead_batches` (default 1) release batches are known in advance | — |
| `OFFLINE` | every task is known at time 0 | TA-Prioritized, TA-Hybrid |

**9. Task type** — `--task_type`

| Value | Meaning |
|---|---|
| `MAPD` | every task has a pickup and a delivery goal (checked against the task file) |
| `MG_MAPD` | every task has an ordered list of goals (default) |

Searches break exact ties with `--tie_breaking reference` (default; the
original implementations) or `random` (seeded by `--seed`).

Unsupported combinations are rejected before the run starts, with the
supported values (`src/config_validate.cpp`). Which planners each assignment
method feeds:

| Task assignment | High-level planners |
|---|---|
| `DECOUPLED_GREEDY`, `DECOUPLED_GREEDY_SWAPS`, `CENTRALIZED_GREEDY` | `PP` inside the assignment (`--replan_on NONE`, `ON_FREE_WAITS`); as a separate step `PBS`, `PP`, `wPBS`, `CBS` |
| `REPEATED_HUNGARIAN`, `REPEATED_HUNGARIAN_LNS` | `PBS`, `PP`, `wPBS`, `CBS` |
| `CENTRAL_HUNGARIAN` | `CBS`, `wPBS` |
| `LKH3_TSP` (offline) | `PP_TASK_SEQUENCE`, `wPBS` |
| `LKH3_TSP_REASSIGN` (offline) | `TA_HYBRID_TWO_GROUP` |

Planner requirements: `wPBS` needs a `PERIODIC` (or `EVERY_TIMESTEP`) refresh,
`ALL` and `STA`/`MLA`/`MLSIPP`; `CBS` plans the agents whose goals changed
(`CHANGED`): a CBS over every agent's goal sequence at every event hits
exponential plateaus with 50 agents. `PBS`, `PP` and `CBS` take any
single-agent search. They are not run `OFFLINE`: the searches fail for pickups
released far in the future (the same happens with a large
`--semi_online_lookahead_batches`; `PBS`/`PP` then fail their collision check).
CENTRAL matches released tasks only, so it runs in every mode, but it plans an
agent to its pickup only and needs a `GOAL_REACHED` (or `EVERY_TIMESTEP`)
replan to plan the delivery (`wPBS` reaches it at its periodic replan). Its
`PBS` and `PP` runs deadlock with 50 agents, and the strict endpoint policy
crowds its idle agents onto task endpoints, so neither is supported. `CBS` does
not plan the LKH3 tour either: every agent starts a sequence at time 0, and a
CBS over 50 agents' sequences stalls. `wPBS` needs a window of at least 2.
When the collision or task-order check fails, `mapd` exits with status 2.

## Algorithm setup

All rows below also receive `--seed 0`. Defaults are
`task_sequence_limit=2`, `wpbs_replan_window=10`, `lns_time=1`,
`lns_no_improvement_limit=2000`, CBS focal weight 1.0, and the CBS high-level
expansion limit `INT_MAX`. CBS uses ICBS conflict prioritization
(MDD-based cardinal, then semi-cardinal, then non-cardinal conflicts), bypass,
target reasoning (a conflict with an agent parked at its goal branches on
"arrive later" versus "the other agent never enters that goal from then on"),
and rectangle reasoning (two agents crossing the same rectangle branch on
barrier constraints along its far edges); all four keep CBS optimal for the
paths its single-agent search returns.
`--cbs_conflict_selection earliest`, `--cbs_bypass 0`,
`--cbs_target_reasoning 0`, and `--cbs_rectangle_reasoning 0` restore plain
earliest-conflict CBS. The standalone executable's whole-run
wall-clock limit defaults to 1000 seconds, while each planning cycle defaults
to 600 seconds. Use `--runtime_limit 0` or
`--pathfinding_runtime_limit 0` to disable the respective limit.

| Displayed method | Required command options | Mode |
|---|---|---|
| TP-STA* | `--config configs/table1/TP.cfg` | Online |
| TP-MLSIPP | `--config configs/table1/TP.cfg --single_agent MLSIPP` | Online extension |
| TPTS-STA* | `--config configs/table1/TPTS.cfg` | Online |
| TPTS-MLSIPP | `--config configs/variants/TPTS-MLSIPP.cfg` | Online extension |
| CENTRAL-CBS | `--config configs/table1/CENTRAL.cfg` | Online, every timestep |
| CENTRAL-fixed-CBS | `--config configs/table1/CENTRAL-fixed.cfg` | Online, event-triggered |
| HBH+MLA* | `--config configs/table1/HBH.cfg` | Online |
| HBH+MLSIPP | `--config configs/table1/HBH.cfg --single_agent MLSIPP` | Online extension |
| HBH+2-segment SIPP | `--config configs/table1/HBH.cfg --single_agent SIPP2` | Online extension |
| TA-Prioritized-STA* | `--config configs/table1/TA-Prioritized.cfg --tour tour/<agents>-500.tour` | Offline only |
| TA-Hybrid-STA* | `--config configs/table1/TA-Hybrid.cfg --tour tour/<agents>-500.tour` | Offline only |
| Hungarian+PBS-MLA* | `--config configs/variants/Hungarian-PBS.cfg` | Online |
| Hungarian+wPBS-MLA* | `--config configs/variants/Hungarian-wPBS.cfg` | Online |
| Hungarian+PBS-MLSIPP | `--config configs/variants/Hungarian-PBS-MLSIPP.cfg` | Online |
| Hungarian+wPBS-MLSIPP | `--config configs/variants/Hungarian-wPBS-MLSIPP.cfg` | Online |
| Hungarian+PP-SIPP | `--config configs/variants/Hungarian-PBS-MLSIPP.cfg --mapf PP` | Online extension |
| LNS(1s)+PBS-MLA* | `--config configs/table1/LNS-PBS.cfg` | Online |
| LNS(1s)+wPBS-MLA* | `--config configs/table1/LNS-wPBS.cfg` | Online |
| LNS(1s)+PBS-MLSIPP | `--config configs/variants/LNS-PBS-MLSIPP.cfg` | Online extension |
| LNS(1s)+wPBS-MLSIPP | `--config configs/variants/LNS-wPBS-MLSIPP.cfg` | Online extension |
| LNS(1s)+PP-SIPP | `--config configs/variants/LNS-PBS-MLSIPP.cfg --mapf PP` | Online extension |

Append `--task_sequence_limit 1` to the eight Hungarian/LNS PBS or wPBS
MLA*/MLSIPP rows to produce their `(ts 1)` variants. Do not apply that option
to TP, TPTS, CENTRAL, HBH, TA, or PP; those paths do not use the PBS/wPBS task
sequence limit.

## Parking endpoint support

Map character `r` denotes a parking/home endpoint and `e` denotes a task
endpoint. All algorithm families can use parking endpoints, but they do so at
the point prescribed by their algorithm. Endpoint selection is dispatched by
the configured `endpoint_strategy` and implemented centrally by
`Simulation::choose_dummy_endpoint()`.

| Methods | Strategy | How parking locations are used |
|---|---|---|
| TP, TPTS | `WAIT_OR_NEAREST_SAFE` | Wait at the current endpoint when safe; otherwise Path2 selects the nearest reachable safe task or parking endpoint. |
| HBH+MLA* | `WAIT_OR_NEAREST_FREE_NONTASK` | An idle blocking agent relocates to the nearest reachable safe parking endpoint. |
| CENTRAL-CBS, CENTRAL-fixed-CBS | `NEAREST_AVAILABLE` | Hungarian assignment may select any unreserved task or parking endpoint for a free agent. |
| TA-Prioritized, TA-Hybrid | `RETURN_TO_HOME` | Each agent returns to its own home parking endpoint after its offline sequence. |
| Hungarian/LNS with PBS or PP | `NEAREST_WITH_STRICT_EXCLUSIONS` | Select the nearest task endpoint that no other new dummy, unfinished task or previous parking location uses (the agent's own included, so an idle agent moves on at each replan); the home endpoint is the fallback. |
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
