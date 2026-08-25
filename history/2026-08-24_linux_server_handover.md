# Linux server handover — 2026-08-24

The authoritative server instructions are in `README.md`. The portable runner
is `scripts/run_server_matrix.py`.

## Required server sequence

```bash
sudo apt-get update
sudo apt-get install -y build-essential g++ make \
    libboost-program-options-dev libdlib-dev python3
make clean && make -j"$(nproc)"

python3 scripts/run_server_matrix.py \
    --smoke --seed 0 --max-parallel 5 --timeout 1800 \
    --output-dir server_results/smoke

python3 scripts/run_server_matrix.py \
    --seed 0 --max-parallel 5 --timeout 1800 \
    --output-dir server_results/full
```

Do not begin the full matrix unless the smoke command exits successfully. The
full command covers the 27 configured method labels, including all eight
`(ts 1)` variants. Offline TA methods run only against frequency 500.

## Reproducibility policy

- Seed: `0` for every comparison run. It controls both LNS and randomized
  search-node tie-breaking.
- Maximum concurrent simulator processes: `5`.
- Per-process wall timeout: `1800` seconds.
- Internal simulator runtime limit: `1795` seconds when launched by the runner;
  the standalone executable defaults to `1800` seconds.
- LNS internal budget: `1` CPU second.
- Standard PBS/wPBS sequence limit: `2`.
- `(ts 1)` variants: sequence limit `1`.
- wPBS replan window: `10`.
- CBS focal weight: `1.0` (optimal CBS).
- CBS high- and low-level expansion limits: `INT_MAX` unless explicitly
  overridden.

The internal runtime limit is a shared `steady_clock` deadline checked inside
CBS, CENTRAL assignment A*, TA-Hybrid, TA-Prioritized, PBS/wPBS, STA*, Path2,
MLA*, MLSIPP, and LNS search loops. Expiration throws `runtime_error`, so the
algorithm prints the timeout location and stops immediately. The runner's
external timeout remains five seconds later as a hard-kill fallback. Pass
`--runtime-limit 0` to the runner (or `--runtime_limit 0` directly to `mapd`)
only when intentionally disabling the internal deadline.

Makespan/SWT should match exactly for non-LNS smoke rows. LNS metric matching
is informational because its CPU-time budget can permit a different number of
iterations on different hardware. All algorithms must nevertheless complete
500/500 tasks and pass collision checking. Runtime is server-dependent.

## Randomized search-node ties

Commit `e584564` replaced deterministic or implicit exact ties in the search
queues with stable pseudo-random tie keys. Each node receives its key once at
creation. A comparator first applies the algorithm's existing criteria—for a
normal A* queue, minimum `f` and then maximum `g`—and consults the key only
when those criteria are equal. Randomness is never sampled from inside a heap
comparator, because doing so would violate strict weak ordering.

The common `--seed` option initializes this stream in `Simulation::init()`:

- the same non-negative seed reproduces the same search order;
- another non-negative seed explores a different, repeatable ordering;
- a negative seed selects a time-based, intentionally non-reproducible order.

The change covers STA*, assignment-cost A*, dummy-path A*, MLA*, MLSIPP,
CBS/ECBS low- and high-level queues, and wPBS low- and high-level selection.
No new configuration field was added. `RandomTieBreaker` and per-node
`tie_breaker` values are implementation state only.

The seed-0 27-method smoke matrix was run twice with at most five processes.
Both runs produced identical makespan/SWT values for every method, completed
all 500 tasks, and passed collision checks. Compared with the preceding
deterministic-tie baseline, makespan improved in 7 rows, was unchanged in 16,
and worsened in 4; SWT improved in 16 rows and worsened in 11. Runtime should
not be compared across machines or concurrent runs because the randomized
ordering changes the number of expanded nodes.

Latest seed-0 smoke results after randomized tie-breaking:

| Method | Setup | Makespan | SWT | Runtime (s) |
|---|---:|---:|---:|---:|
| TP-STA* | 10 / 0.2 | 2541 | 19680 | 0.085 |
| TPTS-STA* | 10 / 0.2 | 2530 | 14454 | 1.108 |
| TP-SIPP | 10 / 0.2 | 2541 | 19032 | 0.116 |
| TPTS-SIPP | 10 / 0.2 | 2513 | 14115 | 0.841 |
| CENTRAL-CBS | 10 / 0.2 | 2514 | 14056 | 1.348 |
| CENTRAL-fixed-CBS | 10 / 0.2 | 2514 | 14156 | 0.464 |
| HBH+MLA* | 10 / 0.2 | 2532 | 14611 | 2.539 |
| TA-Hybrid-STA* | 10 / offline | 1055 | 263846 | 0.975 |
| TA-Prioritized-STA* | 10 / offline | 1049 | 263420 | 0.279 |
| Hungarian+PBS-MLA* | 10 / 0.2 | 2514 | 13814 | 0.307 |
| Hungarian+wPBS-MLA* | 10 / 0.2 | 2512 | 13743 | 0.225 |
| Hungarian+PBS-MLSIPP | 10 / 0.2 | 2517 | 13854 | 0.802 |
| Hungarian+wPBS-MLSIPP | 10 / 0.2 | 2512 | 13678 | 0.214 |
| Hungarian+PP-SIPP | 10 / 0.2 | 2517 | 13796 | 0.435 |
| LNS(1s)+PBS-MLA* | 10 / 0.2 | 2514 | 13738 | 7.170 |
| LNS(1s)+wPBS-MLA* | 10 / 0.2 | 2513 | 13622 | 6.523 |
| LNS(1s)+PBS-MLSIPP | 10 / 0.2 | 2514 | 13770 | 7.412 |
| LNS(1s)+wPBS-MLSIPP | 10 / 0.2 | 2513 | 13585 | 6.657 |
| LNS(1s)+PP-SIPP | 10 / 0.2 | 2514 | 13840 | 7.463 |
| Hungarian+PBS-MLA* (ts 1) | 10 / 0.2 | 2517 | 13775 | 0.284 |
| Hungarian+wPBS-MLA* (ts 1) | 10 / 0.2 | 2512 | 13568 | 0.190 |
| Hungarian+PBS-MLSIPP (ts 1) | 10 / 0.2 | 2514 | 13868 | 0.693 |
| Hungarian+wPBS-MLSIPP (ts 1) | 10 / 0.2 | 2512 | 13626 | 0.203 |
| LNS(1s)+PBS-MLA* (ts 1) | 10 / 0.2 | 2514 | 13790 | 6.829 |
| LNS(1s)+wPBS-MLA* (ts 1) | 10 / 0.2 | 2512 | 13596 | 6.282 |
| LNS(1s)+PBS-MLSIPP (ts 1) | 10 / 0.2 | 2514 | 13814 | 6.709 |
| LNS(1s)+wPBS-MLSIPP (ts 1) | 10 / 0.2 | 2512 | 13611 | 5.993 |

The offline TA rows use the frequency-500 task set because those algorithms
assign the complete task set once. Raw local results are under
`server_results/random_fg_ties_splitmix_seed0/`; server result directories are
ignored and are not part of the Git repository.

## Files to return

Return `server_results/full/results.csv`, `results.json`, and the per-run logs.
The JSON records the complete command for every row, so the matrix is auditable
without reconstructing CLI options from the displayed method name.

The branch includes the reviewed reference workbook
`all_paper_algorithms_comparison_2026-08-24.xlsx`. It is the only Excel file
explicitly allowed by `.gitignore`.

The branch is protected from accidental file additions in two layers:

1. `.gitignore` is an explicit allow-list of the runtime package.
2. `scripts/check_repository_manifest.py` fails locally and in GitHub Actions
   if a required file is missing or an unrelated file becomes tracked.

Use `git add -A`, not `git add *`, because the latter omits hidden paths such as
`.gitignore` and `.github/` before Git processes ignore rules.

## Multi-goal regression

`tests/multigoal-5.task` contains ten tasks with five ordered goals each, and
`tests/multigoal-10.tour` assigns all ten for the offline TA methods. Run every
algorithm against them with:

```bash
python3 scripts/run_server_matrix.py \
    --multigoal-smoke --seed 0 --max-parallel 5 --timeout 1800 \
    --output-dir server_results/multigoal
```

GitHub Actions runs this check after the standard 27-method smoke matrix.

## Generated benchmark package added on 2026-08-25

The repository now includes the finalized generated suite under
`benchmark_instances/`. This package is additional to the legacy
`data/Instances/small/` suite used by the current `run_server_matrix.py`
commands above.

Final inventory:

- four layout families: structured SMALL, MEDIUM, LARGE, and sparse
  SMALL-to-MEDIUM;
- 20 agent-count-specific maps plus four canonical map aliases;
- 245 validated task files covering the paper configurations, the full
  standard frequency matrix, and structured-SMALL multi-goal tasks;
- 20 pickup heatmaps, 20 delivery heatmaps, four layout previews, and three
  padded combined comparison images; and
- 25 offline LKH3 tours: 20 standard `fall` tours and five multi-goal `fall`
  tours.

`fall` means all tasks are released at timestep zero. There is one tour for
each agent count because the LKH node numbering changes with the number of
agent depots. The five LARGE `f100` tours and four canonical online-frequency
aliases that were generated during development were intentionally removed;
TA-Prioritized and TA-Hybrid use the packaged tours only for offline runs.

The tour generator builds a native LKH binary from
`../reference_code/TA-Prioritized/LKH3`, falling back to the TA-Hybrid copy.
It seeds LKH with a complete round-robin assignment before the short
improvement run. Regenerate and validate with:

```bash
python3 scripts/generate_benchmark_lkh_tours.py --time-limit 1
python3 scripts/validate_benchmark_completeness.py
(cd benchmark_instances/lkh_tours && shasum -a 256 -c SHA256SUMS)
python3 scripts/check_repository_manifest.py
```

Validation status at handoff:

- all 20 maps satisfy the well-formedness checks;
- all 245 task files pass format, endpoint-range, count, and frequency checks;
- all 25 LKH tours have complete, unique agent/task-node coverage and valid
  checksums;
- TA-Prioritized completed structured SMALL a10 offline (500/500) and
  structured MEDIUM a100 offline (1,000/1,000), with collision checks passed;
- sparse a10 accepted its LKH tour but later failed prioritized path planning
  at task 460, goal 1; and
- the structured LARGE a200 smoke was stopped manually while it was still
  compute-bound. No malformed-tour error occurred.

The package README is `benchmark_instances/README.md`; LKH pairings and costs
are in `benchmark_instances/lkh_tours/manifest.csv`; provenance limitations are
recorded in `benchmark_instances/PAPER_COMPLETENESS_AUDIT.md`.

## Nineteen-method generated-benchmark run setup

`scripts/run_server_matrix.py` now supports `--base-methods` plus explicit
`--map-template`, `--task-template`, `--offline-task-template`, and
`--tour-template` arguments. `--base-methods` runs the 19 base algorithm rows
and excludes the eight `(ts 1)` variants. The 17 online methods use the listed
frequencies; TA-Prioritized and TA-Hybrid run once per agent count on the
matching `fall` task/tour through `--offline-frequency all`.

The authoritative copy-paste commands for structured SMALL, MEDIUM, LARGE,
sparse SMALL-to-MEDIUM, and structured-SMALL multi-goal experiments are in the
root `README.md` under **Nineteen-method generated-benchmark experiments**.
Run them only after the standard smoke matrix succeeds. Keep
`--max-parallel 5`, `--timeout 1800`, and `--seed 0`; each output directory is
resumable and contains `results.csv`, `results.json`, and one log per job.

For LARGE, online methods run the five `t1000_f100` through `t5000_f100`
workloads. The two offline TA methods run once on the 4,000-task `fall`
workload using `--offline-task-template`, because no task-count-specific
offline series is packaged.
