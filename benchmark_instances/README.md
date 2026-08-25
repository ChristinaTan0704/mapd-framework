# Final warehouse benchmark instances

This directory contains the finalized generated warehouse maps, compatible task
files, and preview images. Every deliverable uses the `benchmark_` prefix and is
explicitly unignored so it can be committed to GitHub.

See [`PAPER_COMPLETENESS_AUDIT.md`](PAPER_COMPLETENESS_AUDIT.md) for the
paper-by-paper validation scope and provenance limits.

| Benchmark | Grid | Task endpoints | Agent homes | Tasks | Default release rate |
|---|---:|---:|---:|---:|---:|
| `benchmark_structured_small` | 35 x 21 | 342 | 10 | 500 | 5/timestep |
| `benchmark_structured_medium` | 101 x 81 | 3,732 | 100 | 1,000 | 50/timestep |
| `benchmark_structured_large` | 187 x 153 | 11,608 | 1,000 | 4,000 | 100/timestep |
| `benchmark_sparse_small_to_medium` | 101 x 81 | 302 | 10 | 500 | 5/timestep |

## Layout definitions

- **Structured SMALL/MEDIUM/LARGE:** regular rectangular endpoint banks flank
  the shelf region.
- **Sparse SMALL to MEDIUM:** the SMALL setup and counts are spread across the
  MEDIUM grid; side endpoints occupy two separated columns per side.

The generated LARGE layout is a reconstruction from the paper description, not
the missing original author-provided map asset.

## Files

Each benchmark name has three canonical matching artifacts:

```text
maps/benchmark_<name>.map
tasks/benchmark_<name>.task
visualizations/benchmark_<name>.png
```

Generated LKH3 task-assignment solutions are stored in `lkh_tours/`. See
[`lkh_tours/README.md`](lkh_tours/README.md) for supported workloads, solver
settings, validation status, and usage.

## Agent-count and task-frequency matrix

Each layout has five agent-count variants:

| Layout | Agent counts |
|---|---|
| Structured SMALL | 10, 20, 30, 40, 50 |
| Structured MEDIUM | 100, 200, 300, 400, 500 |
| Structured LARGE | 200, 400, 600, 800, 1,000 |
| Sparse SMALL-to-MEDIUM | 10, 20, 30, 40, 50 |

Every agent-count map has task files for all repository benchmark frequencies:

```text
0.2, 0.5, 1, 2, 5, 10, 50, 100, 500, all
```

The frequency is the number of tasks released per timestep. Fractional values
therefore space task releases apart; for example, `f0.2` releases one task
every five timesteps. `fall` makes every task available at timestep zero.

Matrix files explicitly record the matching agent count and frequency:

```text
maps/benchmark_<name>_a<agents>.map
tasks/benchmark_<name>_a<agents>_f<frequency>.task
```

For example, `benchmark_structured_medium_a100_f2.task` is compatible with
`benchmark_structured_medium_a100.map` and releases two tasks per timestep.
The agent count is physically defined by the `r` home cells in the `.map`.
Task files must be paired with the same `_a<agents>` map because structured
variants can have different endpoint counts and endpoint indices.

## Paper experiment coverage

The package contains the complete configurations reported in the paper:

- **TABLE II / SMALL MAPD:** 500 tasks; 10, 20, 30, 40, or 50 agents;
  `f=0.2, 0.5, 1, 2, 5, 10`, plus offline (`fall`, equivalently `f500`
  for a 500-task file).
- **TABLE III / MEDIUM MAPD:** 1,000 tasks; 100, 200, 300, 400, or 500
  agents; `f=50`.
- **TABLE IV / LARGE MAPD:** 1,000 agents; `f=100`; and 1,000, 2,000,
  3,000, 4,000, or 5,000 tasks. These files include `_t<tasks>`.
- **TABLE V / SMALL MG-MAPD:** 500 tasks whose ordered goal-sequence lengths
  are sampled from 1 through 5; 10, 20, 30, 40, or 50 agents; `f=2, 5, 10`.
  These files include `_mg_`.

For completeness outside the paper's TABLE V subset, the SMALL MG-MAPD files
are also provided at `f=0.2, 0.5, 1, 500`, and `fall`. For 500 tasks, `f500`
and `fall` both release the complete workload at timestep zero.

TABLE VI reuses the SMALL MG-MAPD `f2` workloads; look-ahead horizon is a
runtime setting and therefore does not require different task files.

All generated layouts are checked against the paper's well-formedness
condition: agent homes and task endpoints are distinct, and every endpoint
can reach the common aisle component without traversing another endpoint.

The canonical task file without `_a..._f...` remains as a convenient alias for
the default rate listed in the table above.

The visualization directory also contains:

- `benchmark_all_layouts_comparison.png`
- `benchmark_pickup_heatmaps_comparison.png`
- `benchmark_delivery_heatmaps_comparison.png`
- `pickup_heatmaps/benchmark_<name>_a<agents>_pickup_heatmap.png` for every
  agent-count map variant.
- `delivery_heatmaps/benchmark_<name>_a<agents>_delivery_heatmap.png` for every
  agent-count map variant.

Pickup heatmaps use pale endpoint cells for zero pickups and increasingly dark
red cells for higher pickup counts. Frequency variants share the same pickup
sequence, so one heatmap covers every frequency for a given map/agent count.
Delivery heatmaps use the same convention with a pale-to-dark green scale.

Example:

```bash
./mapd \
  -m benchmark_instances/maps/benchmark_structured_small.map \
  -t benchmark_instances/tasks/benchmark_structured_small.task \
  -a HUNGARIAN_wPBS
```

The source generators and renderers are kept in `scripts/` and explicitly
included by `.gitignore` exceptions.

Regenerate the full agent-count/task-frequency matrix and checksum manifest with:

```bash
python3 scripts/generate_benchmark_task_matrix.py
python3 scripts/render_pickup_heatmaps.py
python3 scripts/render_delivery_heatmaps.py
```
