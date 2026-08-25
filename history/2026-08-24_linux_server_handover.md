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

- Seed: `0` for every run.
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

## Files to return

Return `server_results/full/results.csv`, `results.json`, and the per-run logs.
The JSON records the complete command for every row, so the matrix is auditable
without reconstructing CLI options from the displayed method name.
