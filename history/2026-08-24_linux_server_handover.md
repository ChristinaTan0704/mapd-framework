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
- LNS internal budget: `1` CPU second.
- Standard PBS/wPBS sequence limit: `2`.
- `(ts 1)` variants: sequence limit `1`.
- wPBS replan window: `10`.
- CBS focal weight: `1.0` (optimal CBS).
- CBS high-level expansion limit: `INT_MAX` unless explicitly overridden.

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
