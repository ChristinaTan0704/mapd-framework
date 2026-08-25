# MG-MAPD paper completeness audit

Reference: *Multi-Goal Multi-Agent Pickup and Delivery*, especially the
problem definition in Section III and the experiments in Section VI.

## Formal well-formedness

All 20 agent-count map variants pass the paper's well-formedness conditions:

1. agent start locations (`r`) are distinct from task endpoints (`e`); and
2. every endpoint has access to one common connected aisle component without
   traversing another endpoint.

## Experimental configurations

| Paper experiment | Required configurations | Package status |
|---|---|---|
| TABLE II, SMALL MAPD | 500 tasks; M=10,20,30,40,50; f=0.2,0.5,1,2,5,10,offline | Complete |
| TABLE III, MEDIUM MAPD | 1,000 tasks; M=100,200,300,400,500; f=50 | Complete |
| TABLE IV, LARGE MAPD | M=1,000; f=100; 1,000--5,000 tasks | Complete |
| TABLE V, SMALL MG-MAPD | 500 tasks; M=10,20,30,40,50; f=2,5,10; 1--5 goals/task | Complete |
| TABLE VI, look-ahead | SMALL MG-MAPD at f=2; LA=0,1,5,10 | Task data complete; LA is a runtime option |

The package also supplies additional agent/frequency combinations for
cross-scale experiments. SMALL MG-MAPD additionally includes f=0.2,0.5,1 and
offline (`f500` and `fall`) variants for every agent count.

## Important provenance limit

The package is complete with respect to the paper's reported dimensions,
shelf-strip counts, workload sizes, agent counts, release rates, task format,
and well-formedness condition. It is not a byte-for-byte copy of every input
used by the authors:

- the structured side banks are generated reconstructions;
- the LARGE map is reconstructed from the paper's description because the
  original LARGE asset was not present in the available reference code; and
- task locations are deterministic seed-0 random samples, not claimed to be
  the authors' unpublished random samples.

Run the non-mutating audit with:

```bash
python3 scripts/validate_benchmark_completeness.py
```

The audit also verifies complete, unique agent/task-node coverage for all 25
packaged offline LKH3 tours. LKH tour generation and its end-to-end limitations
are documented in [`lkh_tours/README.md`](lkh_tours/README.md).
