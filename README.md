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

LNS uses a one-CPU-second improvement budget (`std::clock()`, as the
reference), so another CPU/compiler may execute a different number of LNS
iterations. Unlike the reference, which always spends the whole budget, a call
also stops after 2000 rejected moves in a row (`--lns_no_improvement_limit`,
0 disables it); this mainly shortens the runtime, and the results stay close
(structured small multi-goal f2: a20-a50 gave the same SWT without the stop,
a10 within 0.5%). Its seed remains 0, but a small
LNS metric difference alone is not a failed port. Runtime is never expected to
match across machines.

Representative seed-0 baselines are:

| Method | Setup | Makespan | SWT |
|---|---:|---:|---:|
| TP-STA* | 10 / 0.2 | 2532 | 19270 |
| TPTS-STA* | 10 / 0.2 | 2532 | 14666 |
| CENTRAL-CBS | 10 / 0.2 | 2516 | 14887 |
| HBH+MLA* | 10 / 0.2 | 2532 | 14728 |
| Hungarian+PBS-MLA* | 10 / 0.2 | 2514 | 13781 |
| Hungarian+wPBS-MLA* | 10 / 0.2 | 2513 | 13781 |
| LNS(1s)+PBS-MLA* | 10 / 0.2 | 2513 | 13553 |
| LNS(1s)+wPBS-MLA* | 10 / 0.2 | 2512 | 13774 |
| TA-Prioritized-STA* | 10 / offline | 1052 | 263485 |
| TA-Hybrid-STA* | 10 / offline | 1050 | 263910 |

The runner contains the complete 37-method smoke baseline, including the
`task_sequence_limit` 1 and 2 variants of the Hungarian and LNS methods.

The structured-small 10-agent, frequency-0.2 validation for the added
low-level combinations produced the following seed-0 results (the TP and
TPTS rows before TP/TPTS planned a task goal by goal). Every row completed
500/500 tasks and passed the collision check.

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

GitHub Actions runs the same repository-manifest, Linux build, and 37-method
smoke checks on every push and pull request. Its smoke logs are uploaded as the
`mapd-validation-results` workflow artifact.

Component combinations are validated before a run starts: `validate_config()`
(`src/config_validate.cpp`) lists every supported pairing of assignment and
MAPF method with its triggers, single-agent searches, endpoint strategies and
modes, and rejects anything else with the supported values. The combination
check runs every preset against every override, plus the combinations this
README calls supported that those axes miss, and fails if a combination is
neither rejected before the simulation starts nor completed (exit 0) with
executed paths that pass its own check (moves, collisions, every goal visited
in order by the carrier, the reported makespan and waiting time); a supported
combination that is rejected, or a run still going 30 s past its runtime
limit, also fails:

```bash
python3 scripts/check_combinations.py --max-parallel 5
```

It also runs every planner with releases far ahead of the agents (every task
known upfront, or 1000 release batches ahead, on `kiva-0.2`), fed by repeated
Hungarian (10 agents), TP's greedy assignment (50 agents) and the LKH3 tour
(10 and 50 agents).

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
| TP/TPTS STA* | As the reference token passing, two searches: one to the first goal, then one through the remaining ordered goals to the final goal (sequential STA* legs); each search ends where its goal can be held for good, the goals in between are passed through. Task swapping compares arrival at the first goal and completion at the final goal. As the reference's pickup/delivery rule, extended to every goal: a task is open to an agent only while none of its goals ends another agent's path, and a resting agent moves away when it rests on a later (not the first) goal of an open task. Tasks that share goal cells are therefore served one at a time, which makes TP/TPTS slow on the multi-goal maps with many agents. |
| TP/TPTS MLSIPP | The same two searches; the second is one ordered MLSIPP request over the remaining goals. |
| HBH+MLA* or HBH+MLSIPP | The selected low-level planner receives the complete goal sequence and completion is recorded at its final goal. |
| HBH+2-segment SIPP | Each ordered goal leg is planned by a separate SIPP request; only the final leg reserves its endpoint through the planning horizon. |
| CENTRAL-CBS | An agent is planned to its matched pickup; once it carries the task, every remaining goal is planned (`GOAL_REACHED`). As the reference's delivery rule, extended to every later goal: an agent on a pickup takes the task only when none of its later goals ends any agent's path or is a remaining goal of a carried task, so on multi-goal tasks agents can wait on the pickup for a while. |
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

## Kiva small results (Table 1 and the seven variants)

The sixteen methods on `data/Instances/small` with 10 to 50 agents and the
seven task frequencies (`kiva-<f>.task`), run with the final build of
2026-10-05 (Hungarian/LNS rows after the 64-bit Hungarian): seed 0, reference tie-breaking, a 1000 s limit per run. Every run
completed all 500 tasks and passed the collision and task-order checks. The
numbers are in `output/kiva_small_results_2026-10-04.csv` and `.xlsx` (with
the Excel's original values, `ori_*`; the workbook also has makespan and SWT
pivots and the best method per setting); SWT is the sum over tasks of completion minus release time.

Against the original results (per row, at most 2% worse; where the reference
code run on Linux is itself more than 2% off them, against that run): TP,
TPTS, TA-Prioritized, TA-Hybrid and LNS-wPBS pass every row; CENTRAL every row
but a50 f500 (makespan +2.6%; the reference code needs more than 1000 s
there); Hungarian+PBS fails one row, LNS-PBS and Hungarian+wPBS two each; the
MLSIPP variants are judged by random-tie seed means against their A* versions
on the same seeds (at most one row over 2% each). CENTRAL-fixed follows Ma's
thesis (Sec 6.5.3) and is worse than the original results, which are the
reference CENTRAL's, on 17 rows. HBH has no original results.

### All results

Seed 0, reference tie-breaking, 1000 s per run; each cell is makespan / SWT. Bold: the best makespan or SWT of that column. TA methods are offline (LKH3 tours of the whole task set) and run only the frequency-500 task set.

**10 agents**

| Method | f 0.2 | f 0.5 | f 1 | f 2 | f 5 | f 10 | f 500 |
|---|---|---|---|---|---|---|---|
| TP-STA* | 2532 / 19270 | 1309 / 66393 | 1198 / 155890 | 1167 / 203809 | 1162 / 236892 | 1163 / 247964 | 1136 / 256428 |
| TPTS-STA* | 2532 / 14666 | 1274 / 65574 | 1182 / 150514 | 1168 / 203618 | 1165 / 236590 | 1172 / 252631 | 1105 / 254750 |
| HBH+MLA* | 2532 / 14728 | 1240 / 57913 | **1142** / 143521 | 1131 / 197124 | 1132 / 227889 | 1126 / 242785 | 1095 / 248776 |
| CENTRAL-CBS | 2516 / 14887 | **1230** / 54856 | 1155 / 142875 | **1129** / 194109 | **1117** / 227584 | 1130 / 239088 | 1101 / 250555 |
| CENTRAL-fixed-CBS | 2513 / 14423 | 1250 / 58776 | 1158 / 146013 | **1129** / 195465 | 1145 / 233707 | **1106** / 242936 | 1136 / 251460 |
| TA-Prioritized-STA* | — | — | — | — | — | — | 1052 / 263485 |
| TA-Hybrid-STA* | — | — | — | — | — | — | **1050** / 263910 |
| LNS(1s)+PBS-MLA* | 2513 / **13553** | 1267 / **50689** | 1206 / 126485 | 1170 / **164885** | 1144 / 193113 | 1145 / **202873** | 1130 / 210965 |
| LNS(1s)+wPBS-MLA* | **2512** / 13774 | 1265 / 51427 | 1206 / 124793 | 1172 / 166049 | 1138 / **192075** | 1148 / 203875 | 1141 / **210551** |
| TPTS-MLSIPP | 2530 / 14670 | 1264 / 66634 | 1183 / 147775 | 1158 / 202137 | 1162 / 236213 | 1153 / 246314 | 1144 / 257971 |
| Hungarian+PBS-MLA* | 2514 / 13781 | 1291 / 53452 | 1189 / 124940 | 1155 / 164991 | 1149 / 193386 | 1139 / 203115 | 1131 / 211886 |
| Hungarian+wPBS-MLA* | 2513 / 13781 | 1288 / 53704 | 1193 / 122341 | 1146 / 165676 | 1142 / 193931 | 1144 / 203279 | 1150 / 210594 |
| Hungarian+PBS-MLSIPP | 2513 / 13668 | 1275 / 52741 | 1206 / 124757 | 1167 / 165147 | 1156 / 193624 | 1152 / 203229 | 1136 / 212537 |
| Hungarian+wPBS-MLSIPP | 2513 / 13780 | 1282 / 52636 | 1187 / **121338** | 1151 / 166359 | 1154 / 192868 | 1144 / 203279 | 1127 / 210636 |
| LNS(1s)+PBS-MLSIPP | **2512** / 13817 | 1271 / 51967 | 1200 / 125569 | 1167 / 165411 | 1149 / 193511 | 1138 / 204716 | 1142 / 211559 |
| LNS(1s)+wPBS-MLSIPP | 2513 / 13677 | 1283 / 51899 | 1201 / 124974 | 1174 / 165844 | 1147 / 193446 | 1139 / 202915 | 1136 / 211189 |

**20 agents**

| Method | f 0.2 | f 0.5 | f 1 | f 2 | f 5 | f 10 | f 500 |
|---|---|---|---|---|---|---|---|
| TP-STA* | 2540 / 19887 | 1094 / 21345 | 757 / 47992 | 683 / 95380 | 655 / 123540 | 643 / 137618 | 634 / 141526 |
| TPTS-STA* | 2520 / 12682 | 1038 / 15369 | 706 / 44126 | 667 / 90517 | 645 / 119012 | 645 / 129182 | 640 / 143386 |
| HBH+MLA* | 2532 / 12843 | 1062 / 15297 | 664 / 38285 | 631 / 81648 | 608 / 111633 | 594 / 123656 | 583 / 131243 |
| CENTRAL-CBS | 2515 / 13350 | 1034 / 13996 | 661 / 37566 | 613 / 81001 | 603 / 114776 | 589 / 121091 | 580 / 131779 |
| CENTRAL-fixed-CBS | 2520 / 12518 | 1033 / 14706 | 654 / 39713 | 619 / 82060 | 603 / 113282 | 587 / 123342 | 596 / 131554 |
| TA-Prioritized-STA* | — | — | — | — | — | — | **539** / 138466 |
| TA-Hybrid-STA* | — | — | — | — | — | — | **539** / 137780 |
| LNS(1s)+PBS-MLA* | 2513 / 12424 | 1027 / 13187 | 645 / 31323 | 614 / 66260 | 593 / 93468 | **581** / **100405** | 587 / 110183 |
| LNS(1s)+wPBS-MLA* | 2513 / **12368** | 1032 / 13152 | 639 / **29644** | 608 / **64615** | 594 / 91562 | 598 / 101722 | 567 / **108277** |
| TPTS-MLSIPP | 2515 / 12510 | 1043 / 15414 | 700 / 45926 | 660 / 89273 | 660 / 121688 | 648 / 133458 | 620 / 141218 |
| Hungarian+PBS-MLA* | 2513 / 12424 | 1027 / 13185 | 647 / 32395 | 609 / 67132 | 596 / 92658 | 601 / 101786 | 592 / 111386 |
| Hungarian+wPBS-MLA* | **2511** / 12408 | 1031 / 13160 | 640 / 30771 | 607 / 66339 | 598 / **91152** | 593 / 100537 | 570 / 108944 |
| Hungarian+PBS-MLSIPP | 2512 / 12382 | **1022** / 13186 | 649 / 31686 | 611 / 66708 | 592 / 92673 | 595 / 101517 | 601 / 110423 |
| Hungarian+wPBS-MLSIPP | 2512 / 12426 | 1032 / 13153 | **638** / 31582 | 616 / 66365 | 589 / 91213 | 585 / 101302 | 584 / 109768 |
| LNS(1s)+PBS-MLSIPP | 2512 / 12382 | **1022** / 13139 | 646 / 31512 | **604** / 65834 | 596 / 93131 | 591 / 101438 | 574 / 110218 |
| LNS(1s)+wPBS-MLSIPP | 2513 / 12369 | 1027 / **13125** | 650 / 30351 | 606 / 64693 | **584** / 91397 | 590 / 101739 | 581 / 108669 |

**30 agents**

| Method | f 0.2 | f 0.5 | f 1 | f 2 | f 5 | f 10 | f 500 |
|---|---|---|---|---|---|---|---|
| TP-STA* | 2546 / 19354 | 1069 / 21983 | 607 / 26902 | 529 / 57197 | 478 / 85391 | 526 / 96007 | 432 / 96677 |
| TPTS-STA* | 2527 / 11942 | 1035 / 13568 | 561 / 21421 | 496 / 51346 | 474 / 83829 | 491 / 99152 | 444 / 97516 |
| HBH+MLA* | 2532 / 12399 | 1066 / 13843 | 600 / 19426 | 503 / 45769 | 451 / 74893 | 469 / 84981 | 409 / 91144 |
| CENTRAL-CBS | 2520 / 12780 | 1034 / 13117 | 553 / 15705 | 449 / 42945 | 424 / 73880 | 422 / 82565 | 401 / 92877 |
| CENTRAL-fixed-CBS | 2520 / **11782** | 1032 / 13172 | 576 / 17476 | 452 / 45889 | 440 / 72869 | 433 / 85035 | 425 / 94619 |
| TA-Prioritized-STA* | — | — | — | — | — | — | 372 / 93627 |
| TA-Hybrid-STA* | — | — | — | — | — | — | **365** / 92532 |
| LNS(1s)+PBS-MLA* | 2513 / 11925 | 1029 / 12062 | 534 / 13846 | 430 / 35742 | 408 / 59171 | 406 / 68830 | 409 / 77695 |
| LNS(1s)+wPBS-MLA* | **2512** / 12049 | 1027 / 12255 | 532 / **13444** | 422 / 35135 | 405 / 57969 | **395** / 67055 | 393 / **75594** |
| TPTS-MLSIPP | 2524 / 11974 | 1036 / 13541 | 570 / 20640 | 494 / 53945 | 471 / 82998 | 475 / 95709 | 478 / 100045 |
| Hungarian+PBS-MLA* | 2513 / 11925 | **1026** / **12029** | **530** / 13804 | 437 / 36650 | 418 / 59385 | 405 / 69175 | 405 / 77297 |
| Hungarian+wPBS-MLA* | **2512** / 12049 | 1030 / 12361 | 531 / 13453 | 426 / 34712 | 410 / 57980 | 397 / **66945** | 406 / 75809 |
| Hungarian+PBS-MLSIPP | 2513 / 11886 | 1029 / 12145 | 535 / 13752 | 424 / 36506 | 410 / 59338 | 407 / 68129 | 405 / 76986 |
| Hungarian+wPBS-MLSIPP | **2512** / 12020 | 1027 / 12248 | 533 / 13543 | **421** / **34498** | 411 / **57502** | 406 / 67430 | 410 / 76006 |
| LNS(1s)+PBS-MLSIPP | 2513 / 11886 | 1029 / 12145 | 534 / 13635 | 422 / 35250 | 410 / 58666 | 407 / 67580 | 402 / 78050 |
| LNS(1s)+wPBS-MLSIPP | **2512** / 12020 | 1030 / 12335 | 531 / 13561 | 429 / 34986 | **404** / 57722 | 414 / 66972 | 404 / **75594** |

**40 agents**

| Method | f 0.2 | f 0.5 | f 1 | f 2 | f 5 | f 10 | f 500 |
|---|---|---|---|---|---|---|---|
| TP-STA* | 2540 / 19439 | 1090 / 21505 | 624 / 24399 | 464 / 47660 | 418 / 77663 | 407 / 77316 | 364 / 78819 |
| TPTS-STA* | 2524 / 11749 | 1038 / 12991 | 563 / 15996 | 425 / 36294 | 396 / 65681 | 389 / 76244 | 370 / 78691 |
| HBH+MLA* | 2532 / 12202 | 1062 / 13266 | 598 / 15742 | 433 / 29443 | 357 / 54864 | 395 / 65477 | 352 / 72132 |
| CENTRAL-CBS | 2524 / 12731 | 1032 / 12698 | 555 / 14169 | 366 / 28769 | 332 / 54143 | 344 / 64198 | 330 / 73829 |
| CENTRAL-fixed-CBS | 2517 / 11615 | 1032 / 12654 | 560 / 14262 | 400 / 29851 | 373 / 54901 | 352 / 65224 | 329 / 73963 |
| TA-Prioritized-STA* | — | — | — | — | — | — | 284 / 73256 |
| TA-Hybrid-STA* | — | — | — | — | — | — | **280** / 72717 |
| LNS(1s)+PBS-MLA* | 2513 / 11508 | 1028 / 11801 | **526** / 12432 | 336 / 21838 | 320 / 42747 | 320 / 51384 | 320 / 59863 |
| LNS(1s)+wPBS-MLA* | 2512 / 11842 | 1032 / 12090 | 532 / 12484 | 338 / 21076 | 328 / 42148 | 312 / 50370 | 307 / **59000** |
| TPTS-MLSIPP | 2520 / 11840 | 1035 / 12962 | 554 / 16512 | 422 / 37679 | 391 / 65829 | 410 / 77292 | 387 / 83197 |
| Hungarian+PBS-MLA* | 2513 / **11507** | 1028 / 11801 | **526** / 12432 | 345 / 22445 | 324 / 43527 | 313 / 51596 | 317 / 60738 |
| Hungarian+wPBS-MLA* | 2512 / 11875 | 1032 / 12090 | 532 / 12469 | 333 / 21489 | **313** / **42087** | 312 / 50470 | 312 / 59325 |
| Hungarian+PBS-MLSIPP | **2511** / 11523 | **1022** / **11770** | 528 / 12394 | 339 / 21793 | 319 / 43245 | 316 / 52128 | 307 / 60122 |
| Hungarian+wPBS-MLSIPP | 2513 / 11843 | 1029 / 12003 | 527 / 12581 | 337 / 21620 | 323 / 42533 | **304** / 50374 | 316 / 59427 |
| LNS(1s)+PBS-MLSIPP | **2511** / 11523 | **1022** / **11770** | 527 / **12328** | 342 / 21710 | 318 / 42665 | 314 / 51839 | 306 / 59856 |
| LNS(1s)+wPBS-MLSIPP | 2512 / 11796 | 1027 / 12001 | 532 / 12503 | **329** / **20872** | 317 / 42535 | 312 / **50330** | 319 / 59188 |

**50 agents**

| Method | f 0.2 | f 0.5 | f 1 | f 2 | f 5 | f 10 | f 500 |
|---|---|---|---|---|---|---|---|
| TP-STA* | 2540 / 20016 | 1083 / 21832 | 597 / 24570 | 432 / 37817 | 395 / 62294 | 333 / 65711 | 336 / 68302 |
| TPTS-STA* | 2524 / 11553 | 1036 / 12609 | 554 / 15134 | 383 / 29030 | 343 / 52428 | 319 / 63478 | 338 / 68049 |
| HBH+MLA* | 2532 / 12136 | 1062 / 13131 | 593 / 14947 | 377 / 21817 | 310 / 44208 | 313 / 54768 | 319 / 60624 |
| CENTRAL-CBS | 2524 / 12528 | 1035 / 12471 | 553 / 13692 | 335 / 20718 | 313 / 43452 | 304 / 53351 | 280 / 62413 |
| CENTRAL-fixed-CBS | 2518 / **11366** | 1034 / 12294 | 553 / 13617 | 331 / 21775 | 301 / 43554 | 320 / 54258 | 285 / 62729 |
| TA-Prioritized-STA* | — | — | — | — | — | — | 233 / 60403 |
| TA-Hybrid-STA* | — | — | — | — | — | — | **230** / 59431 |
| LNS(1s)+PBS-MLA* | 2514 / 11390 | 1029 / 11622 | **527** / 11979 | 296 / 14968 | 269 / 33560 | 260 / 41684 | 261 / 50554 |
| LNS(1s)+wPBS-MLA* | **2511** / 11743 | 1030 / 12018 | 528 / 12339 | 290 / 14250 | **256** / 33099 | **250** / 40975 | 259 / 49689 |
| TPTS-MLSIPP | 2524 / 11490 | 1034 / 12717 | 558 / 15398 | 356 / 29909 | 322 / 51809 | 353 / 63837 | 324 / 70632 |
| Hungarian+PBS-MLA* | 2514 / 11390 | 1028 / 11644 | 528 / 12032 | 292 / 15111 | 263 / 33823 | 257 / 41777 | 256 / 50363 |
| Hungarian+wPBS-MLA* | 2512 / 11724 | 1027 / 11980 | 532 / 12228 | 297 / **14097** | 271 / 33131 | 260 / 41066 | 252 / 49419 |
| Hungarian+PBS-MLSIPP | 2512 / 11441 | **1022** / **11555** | **527** / **11863** | 293 / 15083 | 266 / 33864 | 258 / 42047 | 258 / 49831 |
| Hungarian+wPBS-MLSIPP | 2512 / 11779 | 1029 / 11883 | 528 / 12001 | **289** / 14235 | 260 / **32321** | 265 / 41275 | 246 / **49125** |
| LNS(1s)+PBS-MLSIPP | 2512 / 11441 | **1022** / **11555** | **527** / **11863** | 295 / 14953 | 267 / 33647 | 261 / 42197 | 259 / 50148 |
| LNS(1s)+wPBS-MLSIPP | 2512 / 11743 | 1027 / 11850 | 531 / 12065 | 293 / 14232 | 271 / 32427 | 259 / **40920** | 251 / 49254 |

### Best method per setting

| Agents | Frequency | Best makespan | Best SWT |
|---:|---:|---|---|
| 10 | 0.2 | LNS(1s)+wPBS-MLA*, LNS(1s)+PBS-MLSIPP (2512) | LNS(1s)+PBS-MLA* (13553) |
| 10 | 0.5 | CENTRAL-CBS (1230) | LNS(1s)+PBS-MLA* (50689) |
| 10 | 1 | HBH+MLA* (1142) | Hungarian+wPBS-MLSIPP (121338) |
| 10 | 2 | CENTRAL-CBS, CENTRAL-fixed-CBS (1129) | LNS(1s)+PBS-MLA* (164885) |
| 10 | 5 | CENTRAL-CBS (1117) | LNS(1s)+wPBS-MLA* (192075) |
| 10 | 10 | CENTRAL-fixed-CBS (1106) | LNS(1s)+PBS-MLA* (202873) |
| 10 | 500 | TA-Hybrid-STA* (1050) | LNS(1s)+wPBS-MLA* (210551) |
| 20 | 0.2 | Hungarian+wPBS-MLA* (2511) | LNS(1s)+wPBS-MLA* (12368) |
| 20 | 0.5 | Hungarian+PBS-MLSIPP, LNS(1s)+PBS-MLSIPP (1022) | LNS(1s)+wPBS-MLSIPP (13125) |
| 20 | 1 | Hungarian+wPBS-MLSIPP (638) | LNS(1s)+wPBS-MLA* (29644) |
| 20 | 2 | LNS(1s)+PBS-MLSIPP (604) | LNS(1s)+wPBS-MLA* (64615) |
| 20 | 5 | LNS(1s)+wPBS-MLSIPP (584) | Hungarian+wPBS-MLA* (91152) |
| 20 | 10 | LNS(1s)+PBS-MLA* (581) | LNS(1s)+PBS-MLA* (100405) |
| 20 | 500 | TA-Prioritized-STA*, TA-Hybrid-STA* (539) | LNS(1s)+wPBS-MLA* (108277) |
| 30 | 0.2 | LNS(1s)+wPBS-MLA*, Hungarian+wPBS-MLA*, Hungarian+wPBS-MLSIPP, LNS(1s)+wPBS-MLSIPP (2512) | CENTRAL-fixed-CBS (11782) |
| 30 | 0.5 | Hungarian+PBS-MLA* (1026) | Hungarian+PBS-MLA* (12029) |
| 30 | 1 | Hungarian+PBS-MLA* (530) | LNS(1s)+wPBS-MLA* (13444) |
| 30 | 2 | Hungarian+wPBS-MLSIPP (421) | Hungarian+wPBS-MLSIPP (34498) |
| 30 | 5 | LNS(1s)+wPBS-MLSIPP (404) | Hungarian+wPBS-MLSIPP (57502) |
| 30 | 10 | LNS(1s)+wPBS-MLA* (395) | Hungarian+wPBS-MLA* (66945) |
| 30 | 500 | TA-Hybrid-STA* (365) | LNS(1s)+wPBS-MLA*, LNS(1s)+wPBS-MLSIPP (75594) |
| 40 | 0.2 | Hungarian+PBS-MLSIPP, LNS(1s)+PBS-MLSIPP (2511) | Hungarian+PBS-MLA* (11507) |
| 40 | 0.5 | Hungarian+PBS-MLSIPP, LNS(1s)+PBS-MLSIPP (1022) | Hungarian+PBS-MLSIPP, LNS(1s)+PBS-MLSIPP (11770) |
| 40 | 1 | LNS(1s)+PBS-MLA*, Hungarian+PBS-MLA* (526) | LNS(1s)+PBS-MLSIPP (12328) |
| 40 | 2 | LNS(1s)+wPBS-MLSIPP (329) | LNS(1s)+wPBS-MLSIPP (20872) |
| 40 | 5 | Hungarian+wPBS-MLA* (313) | Hungarian+wPBS-MLA* (42087) |
| 40 | 10 | Hungarian+wPBS-MLSIPP (304) | LNS(1s)+wPBS-MLSIPP (50330) |
| 40 | 500 | TA-Hybrid-STA* (280) | LNS(1s)+wPBS-MLA* (59000) |
| 50 | 0.2 | LNS(1s)+wPBS-MLA* (2511) | CENTRAL-fixed-CBS (11366) |
| 50 | 0.5 | Hungarian+PBS-MLSIPP, LNS(1s)+PBS-MLSIPP (1022) | Hungarian+PBS-MLSIPP, LNS(1s)+PBS-MLSIPP (11555) |
| 50 | 1 | LNS(1s)+PBS-MLA*, Hungarian+PBS-MLSIPP, LNS(1s)+PBS-MLSIPP (527) | Hungarian+PBS-MLSIPP, LNS(1s)+PBS-MLSIPP (11863) |
| 50 | 2 | Hungarian+wPBS-MLSIPP (289) | Hungarian+wPBS-MLA* (14097) |
| 50 | 5 | LNS(1s)+wPBS-MLA* (256) | Hungarian+wPBS-MLSIPP (32321) |
| 50 | 10 | LNS(1s)+wPBS-MLA* (250) | LNS(1s)+wPBS-MLSIPP (40920) |
| 50 | 500 | TA-Hybrid-STA* (230) | Hungarian+wPBS-MLSIPP (49125) |

Times best (ties count for each):

| Method | Best makespan | Best SWT |
|---|---:|---:|
| HBH+MLA* | 1 | 0 |
| CENTRAL-CBS | 3 | 0 |
| CENTRAL-fixed-CBS | 2 | 2 |
| TA-Prioritized-STA* | 1 | 0 |
| TA-Hybrid-STA* | 5 | 0 |
| LNS(1s)+PBS-MLA* | 3 | 5 |
| LNS(1s)+wPBS-MLA* | 6 | 9 |
| Hungarian+PBS-MLA* | 3 | 2 |
| Hungarian+wPBS-MLA* | 3 | 4 |
| Hungarian+PBS-MLSIPP | 5 | 3 |
| Hungarian+wPBS-MLSIPP | 5 | 5 |
| LNS(1s)+PBS-MLSIPP | 7 | 4 |
| LNS(1s)+wPBS-MLSIPP | 4 | 5 |

## Generated-benchmark results (Table 1 and the seven variants)

The same sixteen methods on every map family of `benchmark_instances/`, with
the commands of the next sections (seed 0, reference tie-breaking, 1000 s per
run; TA-Prioritized and TA-Hybrid run once per agent count on the offline
`all` task set, so in the multi-goal and MEDIUM tables the `all` column holds
only them). Builds of 2026-10-05 (`6022b51`; the CENTRAL rows `8ae6c7a`,
whose differences only concern Hungarian/LNS and TA-Hybrid). All numbers,
with the reason of every run without a result, are in
`output/benchmark_results_2026-10-05.csv` and `.xlsx` (one sheet per family,
makespan and SWT pivots and the best method per setting).

On the LARGE map (1,000 agents, 1,000 to 5,000 tasks) no method finishes
within 1000 s: the searches run out of time, and the PBS family also needs
about 20 GB (these runs were capped at 14 GB; with 60 GB
Hungarian+PBS-MLSIPP still timed out after 995 s).

### Results by map family

#### Structured SMALL (35 x 21, 500 tasks)

500 runs, all completed every task.

**10 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 0.2 | f 0.5 | f 1 | f 2 | f 5 | f 10 | f all |
|---|---|---|---|---|---|---|---|
| TP-STA* | 2546 / 20031 | 1251 / 71392 | 1190 / 164966 | 1166 / 216018 | 1146 / 249304 | 1141 / 260916 | 1135 / 266757 |
| TPTS-STA* | 2543 / 14970 | 1258 / 70075 | 1160 / 167868 | 1140 / 217923 | 1152 / 249201 | 1137 / 264442 | 1125 / 266881 |
| HBH+MLA* | 2546 / 15139 | 1225 / 56628 | 1139 / 150600 | 1112 / 206905 | 1117 / 238187 | 1101 / 253783 | **1076** / 259053 |
| CENTRAL-CBS | **2535** / 15263 | **1223** / 61155 | **1131** / 153544 | **1097** / 208279 | **1106** / 242168 | 1100 / 254705 | 1103 / 265176 |
| CENTRAL-fixed-CBS | **2535** / 14960 | 1224 / 59312 | 1133 / 153352 | 1118 / 212078 | 1117 / 244892 | **1095** / 253065 | 1104 / 265522 |
| TA-Prioritized-STA* | — | — | — | — | — | — | 2042 / 495443 |
| TA-Hybrid-STA* | — | — | — | — | — | — | 1618 / 407234 |
| LNS(1s)+PBS-MLA* | 2540 / 14368 | 1265 / 54603 | 1179 / 129687 | 1153 / 173049 | 1121 / **195096** | 1126 / 206591 | 1140 / 214717 |
| LNS(1s)+wPBS-MLA* | 2543 / 14328 | 1282 / 54912 | 1175 / **127518** | 1153 / **169735** | 1140 / 198370 | 1128 / **204290** | 1118 / **212542** |
| TPTS-MLSIPP | **2535** / 14934 | 1265 / 68879 | 1172 / 164167 | 1132 / 212712 | 1159 / 251207 | 1130 / 261211 | 1121 / 268511 |
| Hungarian+PBS-MLA* | 2542 / 14410 | 1281 / 56532 | 1175 / 130231 | 1149 / 170501 | 1143 / 196714 | 1142 / 208281 | 1129 / 214289 |
| Hungarian+wPBS-MLA* | 2543 / 14371 | 1279 / 56629 | 1175 / 128379 | 1152 / 171527 | 1124 / 195887 | 1128 / 206346 | 1116 / 213819 |
| Hungarian+PBS-MLSIPP | 2541 / 14451 | 1261 / 55752 | 1183 / 129684 | 1154 / 170405 | 1143 / 196701 | 1128 / 207861 | 1133 / 214361 |
| Hungarian+wPBS-MLSIPP | 2542 / **14289** | 1287 / 56379 | 1185 / 128241 | 1148 / 171872 | 1144 / 195888 | 1128 / 206346 | 1116 / 213819 |
| LNS(1s)+PBS-MLSIPP | 2540 / 14404 | 1272 / 55751 | 1181 / 132136 | 1158 / 172921 | 1132 / 196566 | 1124 / 205636 | 1109 / 213767 |
| LNS(1s)+wPBS-MLSIPP | 2542 / 14416 | 1284 / **54534** | 1191 / 129589 | 1159 / 169748 | 1140 / 197532 | 1120 / 204295 | 1123 / 213202 |

**20 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 0.2 | f 0.5 | f 1 | f 2 | f 5 | f 10 | f all |
|---|---|---|---|---|---|---|---|
| TP-STA* | 2554 / 20517 | 1052 / 22209 | 716 / 60626 | 685 / 106036 | 668 / 138967 | 681 / 148308 | 650 / 149167 |
| TPTS-STA* | 2538 / 13509 | 1060 / 16522 | 707 / 54940 | 692 / 104245 | 662 / 137640 | 653 / 144002 | 623 / 149370 |
| HBH+MLA* | 2535 / 13224 | 1047 / 15704 | 657 / 41490 | 644 / 88538 | 617 / 122735 | 610 / 133365 | 658 / 143856 |
| CENTRAL-CBS | 2533 / 13907 | 1041 / 14579 | 672 / 41013 | 636 / 89814 | 632 / 122005 | 641 / 134601 | 613 / 141969 |
| CENTRAL-fixed-CBS | 2535 / **12969** | 1046 / 15007 | 654 / 41828 | 645 / 91556 | 633 / 122808 | 623 / 134476 | 622 / 143018 |
| TA-Prioritized-STA* | — | — | — | — | — | — | 1127 / 259370 |
| TA-Hybrid-STA* | — | — | — | — | — | — | 772 / 195449 |
| LNS(1s)+PBS-MLA* | 2528 / 13550 | 1042 / 14124 | 674 / 34830 | 630 / 72669 | 612 / 97270 | **602** / 105976 | 602 / 113991 |
| LNS(1s)+wPBS-MLA* | 2538 / 13900 | 1050 / 14197 | 664 / 34005 | 635 / 71982 | 615 / 97223 | 614 / 106078 | 610 / 113668 |
| TPTS-MLSIPP | 2538 / 13259 | 1058 / 16220 | 734 / 55366 | 671 / 99116 | 663 / 135315 | 662 / 143638 | 669 / 151500 |
| Hungarian+PBS-MLA* | **2527** / 13674 | 1050 / **14050** | 657 / 34995 | 640 / 72337 | **606** / 97607 | 603 / 106820 | 612 / 115340 |
| Hungarian+wPBS-MLA* | 2530 / 13903 | 1046 / 14099 | 679 / 34809 | 641 / **71235** | 609 / 96837 | 609 / **105750** | 607 / 113902 |
| Hungarian+PBS-MLSIPP | 2531 / 13642 | **1040** / 14058 | 670 / 34466 | 641 / 72369 | 619 / 97532 | 618 / 106911 | 611 / 115143 |
| Hungarian+wPBS-MLSIPP | 2530 / 13883 | 1041 / 14060 | **648** / 34200 | **627** / 71294 | 610 / **96363** | 604 / 105788 | 604 / 113817 |
| LNS(1s)+PBS-MLSIPP | 2529 / 13690 | 1043 / 14112 | 662 / 34379 | 643 / 73046 | 620 / 97417 | 605 / 106673 | 615 / 114879 |
| LNS(1s)+wPBS-MLSIPP | 2528 / 13920 | 1044 / 14289 | 664 / **33819** | 630 / 72054 | **606** / 97273 | 616 / 106144 | **593** / **113333** |

**30 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 0.2 | f 0.5 | f 1 | f 2 | f 5 | f 10 | f all |
|---|---|---|---|---|---|---|---|
| TP-STA* | 2565 / 19906 | 1093 / 21571 | 577 / 25164 | 513 / 61545 | 498 / 92077 | 465 / 100556 | 490 / 103944 |
| TPTS-STA* | 2532 / 12504 | 1048 / 13780 | 574 / 23587 | 497 / 56827 | 509 / 87565 | 502 / 99691 | 450 / 105387 |
| HBH+MLA* | 2532 / 12598 | 1035 / 13276 | 595 / 18461 | 470 / 45918 | 454 / 76049 | 425 / 87369 | 398 / 93523 |
| CENTRAL-CBS | 2535 / 13386 | 1038 / 13816 | 551 / 15565 | 471 / 45940 | 414 / 75717 | 401 / 87651 | 400 / 94572 |
| CENTRAL-fixed-CBS | 2532 / **12197** | 1035 / 13036 | 544 / 17695 | 438 / 47085 | 418 / 77505 | 426 / 89186 | 440 / 94876 |
| TA-Prioritized-STA* | — | — | — | — | — | — | 755 / 175268 |
| TA-Hybrid-STA* | — | — | — | — | — | — | 500 / 124571 |
| LNS(1s)+PBS-MLA* | 2532 / 12764 | 1035 / 13068 | **541** / **14053** | 426 / 35865 | 410 / 60242 | 403 / 69874 | 399 / 77972 |
| LNS(1s)+wPBS-MLA* | 2533 / 13078 | **1034** / 13276 | **541** / 14201 | 422 / 35102 | 413 / 59714 | 406 / 68791 | **391** / 76332 |
| TPTS-MLSIPP | 2532 / 12445 | 1041 / 13708 | 571 / 20179 | 477 / 57575 | 456 / 85557 | 481 / 98035 | 448 / 105279 |
| Hungarian+PBS-MLA* | **2531** / 12751 | 1035 / 13086 | 545 / 14202 | 419 / 36242 | 407 / 60016 | 406 / 69559 | 393 / 77294 |
| Hungarian+wPBS-MLA* | 2534 / 13091 | 1036 / 13121 | 544 / 14365 | 425 / 35501 | 403 / **59077** | 415 / 68602 | 400 / 76311 |
| Hungarian+PBS-MLSIPP | **2531** / 12999 | 1035 / **12970** | 543 / 14177 | 428 / 36113 | **399** / 60763 | 405 / 70012 | 399 / 77501 |
| Hungarian+wPBS-MLSIPP | 2533 / 12973 | 1036 / 13168 | **541** / 14212 | 422 / 35870 | 408 / 59314 | 414 / 68756 | 403 / **76106** |
| LNS(1s)+PBS-MLSIPP | **2531** / 12999 | 1035 / 13012 | 545 / 14392 | 428 / 36425 | 406 / 60431 | 405 / 69423 | 403 / 77184 |
| LNS(1s)+wPBS-MLSIPP | 2533 / 12973 | 1036 / 13107 | **541** / 14252 | **416** / **35087** | 405 / 59529 | **395** / **68474** | 399 / 76529 |

**40 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 0.2 | f 0.5 | f 1 | f 2 | f 5 | f 10 | f all |
|---|---|---|---|---|---|---|---|
| TP-STA* | 2541 / 20215 | 1055 / 22512 | 602 / 24974 | 428 / 43315 | 367 / 67693 | 400 / 78672 | 398 / 85370 |
| TPTS-STA* | 2517 / 12088 | **1026** / 13715 | 549 / 17714 | 410 / 42294 | 368 / 66095 | 379 / 77874 | 350 / 82314 |
| HBH+MLA* | **2514** / 12358 | **1026** / 13124 | 548 / 14918 | 403 / 28182 | 384 / 54412 | 362 / 65318 | 341 / 71850 |
| CENTRAL-CBS | **2514** / 12969 | 1027 / 12858 | 531 / 13958 | 351 / 27203 | 336 / 55211 | 330 / 65027 | 322 / 75143 |
| CENTRAL-fixed-CBS | 2517 / **11928** | 1028 / 12632 | 532 / 14017 | 363 / 28271 | 331 / 55629 | 337 / 66168 | 324 / 75241 |
| TA-Prioritized-STA* | — | — | — | — | — | — | 633 / 134614 |
| TA-Hybrid-STA* | — | — | — | — | — | — | 375 / 93452 |
| LNS(1s)+PBS-MLA* | 2516 / 12299 | 1030 / 12471 | 531 / 12863 | 333 / 21741 | 312 / 42062 | 314 / 50700 | 305 / 59494 |
| LNS(1s)+wPBS-MLA* | 2521 / 12761 | 1035 / 12908 | 533 / 13280 | 334 / 20695 | 312 / 41563 | **300** / 49651 | 300 / 57976 |
| TPTS-MLSIPP | **2514** / 11934 | **1026** / 13366 | 552 / 16561 | 397 / 34804 | 372 / 63181 | 381 / 74872 | 396 / 82628 |
| Hungarian+PBS-MLA* | 2517 / 12249 | 1029 / **12453** | **530** / 12827 | 336 / 21341 | 312 / 42123 | 314 / 50662 | 309 / 58612 |
| Hungarian+wPBS-MLA* | 2518 / 12683 | 1032 / 12628 | 535 / 13234 | 338 / 21039 | 315 / 41262 | **300** / 49624 | 305 / **57623** |
| Hungarian+PBS-MLSIPP | 2517 / 12249 | 1029 / 12683 | **530** / **12775** | 333 / 21451 | 312 / 42256 | 309 / 50878 | 304 / 59129 |
| Hungarian+wPBS-MLSIPP | 2517 / 12872 | 1032 / 12913 | 532 / 13183 | **328** / 21099 | 313 / 41146 | **300** / 49896 | 304 / 57642 |
| LNS(1s)+PBS-MLSIPP | 2516 / 12193 | 1029 / 12539 | **530** / 12997 | 330 / 20891 | 315 / 41945 | 309 / 50477 | 303 / 59354 |
| LNS(1s)+wPBS-MLSIPP | 2516 / 12764 | 1037 / 12835 | 532 / 13384 | 339 / **20552** | **303** / **40823** | 303 / **49011** | **298** / 58211 |

**50 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 0.2 | f 0.5 | f 1 | f 2 | f 5 | f 10 | f all |
|---|---|---|---|---|---|---|---|
| TP-STA* | 2536 / 20370 | 1044 / 22535 | 565 / 25888 | 483 / 42865 | 440 / 65534 | 455 / 68417 | 356 / 68709 |
| TPTS-STA* | 2509 / 11908 | 1034 / 13568 | 576 / 15771 | 364 / 30722 | 360 / 61652 | 372 / 61751 | 334 / 68083 |
| HBH+MLA* | 2520 / 12169 | 1034 / 12945 | 541 / 14429 | 348 / 21434 | 402 / 45143 | 400 / 55224 | 375 / 63669 |
| CENTRAL-CBS | **2507** / 12744 | **1022** / 12498 | **529** / 13546 | 307 / 20286 | 323 / 46581 | 313 / 55287 | 325 / 64060 |
| CENTRAL-fixed-CBS | **2507** / **11644** | 1025 / 12351 | 530 / 13560 | 336 / 21414 | 328 / 46737 | 318 / 55378 | 313 / 63471 |
| TA-Prioritized-STA* | — | — | — | — | — | — | 484 / 113130 |
| TA-Hybrid-STA* | — | — | — | — | — | — | 300 / 76599 |
| LNS(1s)+PBS-MLA* | 2513 / 12082 | 1027 / 12178 | 530 / **12536** | 298 / 16042 | 272 / 35804 | 269 / 44448 | 274 / 51543 |
| LNS(1s)+wPBS-MLA* | 2514 / 12635 | 1029 / 12803 | 537 / 13039 | 297 / 15122 | 266 / 34381 | 264 / 43222 | **254** / 50155 |
| TPTS-MLSIPP | 2508 / 11932 | **1022** / 13605 | 569 / 16438 | 391 / 28742 | 376 / 61125 | 366 / 63802 | 343 / 69904 |
| Hungarian+PBS-MLA* | 2517 / 12073 | 1028 / **12176** | 531 / 12596 | 292 / 15843 | 266 / 35542 | 262 / 43830 | 260 / 51622 |
| Hungarian+wPBS-MLA* | 2514 / 12681 | 1030 / 12656 | 537 / 13117 | **290** / 15040 | 264 / 34795 | 262 / 42892 | 255 / 50608 |
| Hungarian+PBS-MLSIPP | 2513 / 12103 | 1028 / 12207 | 530 / 12610 | 292 / 15822 | 269 / 34982 | 264 / 44192 | 260 / 51592 |
| Hungarian+wPBS-MLSIPP | 2513 / 12577 | 1029 / 12796 | 541 / 13271 | 297 / 15211 | 267 / **34304** | 276 / 43036 | 261 / 50853 |
| LNS(1s)+PBS-MLSIPP | 2514 / 12175 | 1027 / 12195 | 532 / 12591 | 293 / 15640 | **263** / 35134 | **259** / 43610 | 255 / 50838 |
| LNS(1s)+wPBS-MLSIPP | 2514 / 12591 | 1030 / 12940 | 537 / 13370 | 297 / **14962** | 271 / 34379 | 267 / **42727** | 260 / **50077** |

Best method per setting:

| Agents | Frequency | Best makespan | Best SWT |
|---:|---:|---|---|
| 10 | 0.2 | CENTRAL-CBS, CENTRAL-fixed-CBS, TPTS-MLSIPP (2535) | Hungarian+wPBS-MLSIPP (14289) |
| 10 | 0.5 | CENTRAL-CBS (1223) | LNS(1s)+wPBS-MLSIPP (54534) |
| 10 | 1 | CENTRAL-CBS (1131) | LNS(1s)+wPBS-MLA* (127518) |
| 10 | 2 | CENTRAL-CBS (1097) | LNS(1s)+wPBS-MLA* (169735) |
| 10 | 5 | CENTRAL-CBS (1106) | LNS(1s)+PBS-MLA* (195096) |
| 10 | 10 | CENTRAL-fixed-CBS (1095) | LNS(1s)+wPBS-MLA* (204290) |
| 10 | all | HBH+MLA* (1076) | LNS(1s)+wPBS-MLA* (212542) |
| 20 | 0.2 | Hungarian+PBS-MLA* (2527) | CENTRAL-fixed-CBS (12969) |
| 20 | 0.5 | Hungarian+PBS-MLSIPP (1040) | Hungarian+PBS-MLA* (14050) |
| 20 | 1 | Hungarian+wPBS-MLSIPP (648) | LNS(1s)+wPBS-MLSIPP (33819) |
| 20 | 2 | Hungarian+wPBS-MLSIPP (627) | Hungarian+wPBS-MLA* (71235) |
| 20 | 5 | Hungarian+PBS-MLA*, LNS(1s)+wPBS-MLSIPP (606) | Hungarian+wPBS-MLSIPP (96363) |
| 20 | 10 | LNS(1s)+PBS-MLA* (602) | Hungarian+wPBS-MLA* (105750) |
| 20 | all | LNS(1s)+wPBS-MLSIPP (593) | LNS(1s)+wPBS-MLSIPP (113333) |
| 30 | 0.2 | Hungarian+PBS-MLA*, Hungarian+PBS-MLSIPP, LNS(1s)+PBS-MLSIPP (2531) | CENTRAL-fixed-CBS (12197) |
| 30 | 0.5 | LNS(1s)+wPBS-MLA* (1034) | Hungarian+PBS-MLSIPP (12970) |
| 30 | 1 | LNS(1s)+PBS-MLA*, LNS(1s)+wPBS-MLA*, Hungarian+wPBS-MLSIPP, LNS(1s)+wPBS-MLSIPP (541) | LNS(1s)+PBS-MLA* (14053) |
| 30 | 2 | LNS(1s)+wPBS-MLSIPP (416) | LNS(1s)+wPBS-MLSIPP (35087) |
| 30 | 5 | Hungarian+PBS-MLSIPP (399) | Hungarian+wPBS-MLA* (59077) |
| 30 | 10 | LNS(1s)+wPBS-MLSIPP (395) | LNS(1s)+wPBS-MLSIPP (68474) |
| 30 | all | LNS(1s)+wPBS-MLA* (391) | Hungarian+wPBS-MLSIPP (76106) |
| 40 | 0.2 | HBH+MLA*, CENTRAL-CBS, TPTS-MLSIPP (2514) | CENTRAL-fixed-CBS (11928) |
| 40 | 0.5 | TPTS-STA*, HBH+MLA*, TPTS-MLSIPP (1026) | Hungarian+PBS-MLA* (12453) |
| 40 | 1 | Hungarian+PBS-MLA*, Hungarian+PBS-MLSIPP, LNS(1s)+PBS-MLSIPP (530) | Hungarian+PBS-MLSIPP (12775) |
| 40 | 2 | Hungarian+wPBS-MLSIPP (328) | LNS(1s)+wPBS-MLSIPP (20552) |
| 40 | 5 | LNS(1s)+wPBS-MLSIPP (303) | LNS(1s)+wPBS-MLSIPP (40823) |
| 40 | 10 | LNS(1s)+wPBS-MLA*, Hungarian+wPBS-MLA*, Hungarian+wPBS-MLSIPP (300) | LNS(1s)+wPBS-MLSIPP (49011) |
| 40 | all | LNS(1s)+wPBS-MLSIPP (298) | Hungarian+wPBS-MLA* (57623) |
| 50 | 0.2 | CENTRAL-CBS, CENTRAL-fixed-CBS (2507) | CENTRAL-fixed-CBS (11644) |
| 50 | 0.5 | CENTRAL-CBS, TPTS-MLSIPP (1022) | Hungarian+PBS-MLA* (12176) |
| 50 | 1 | CENTRAL-CBS (529) | LNS(1s)+PBS-MLA* (12536) |
| 50 | 2 | Hungarian+wPBS-MLA* (290) | LNS(1s)+wPBS-MLSIPP (14962) |
| 50 | 5 | LNS(1s)+PBS-MLSIPP (263) | Hungarian+wPBS-MLSIPP (34304) |
| 50 | 10 | LNS(1s)+PBS-MLSIPP (259) | LNS(1s)+wPBS-MLSIPP (42727) |
| 50 | all | LNS(1s)+wPBS-MLA* (254) | LNS(1s)+wPBS-MLSIPP (50077) |

#### Structured SMALL multi-goal (500 tasks)

220 runs: 219 completed every task; 1 error(1).

**10 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 2 | f 5 | f 10 | f all |
|---|---|---|---|---|
| TP-STA* | 2845 / 598808 | 2920 / 632753 | 2798 / 653643 | — |
| TPTS-STA* | 2892 / 595786 | 2836 / 648579 | 2846 / 663410 | — |
| HBH+MLA* | 2261 / 466538 | **2204** / 505411 | 2237 / 518170 | — |
| CENTRAL-CBS | **2251** / 469909 | 2296 / 500387 | 2314 / 519542 | — |
| CENTRAL-fixed-CBS | 2314 / 490303 | 2352 / 532107 | 2291 / 535781 | — |
| TA-Prioritized-STA* | — | — | — | 3163 / 738963 |
| TA-Hybrid-STA* | — | — | — | **2689** / **663504** |
| LNS(1s)+PBS-MLA* | 2292 / 294583 | 2248 / **311956** | 2235 / 319508 | — |
| LNS(1s)+wPBS-MLA* | 2269 / **290573** | 2270 / 312890 | 2233 / **317759** | — |
| TPTS-MLSIPP | 2895 / 615025 | 2868 / 659070 | 2835 / 659075 | — |
| Hungarian+PBS-MLA* | 2294 / 298310 | 2269 / 318293 | 2241 / 320053 | — |
| Hungarian+wPBS-MLA* | 2291 / 294401 | 2239 / 314821 | 2269 / 321483 | — |
| Hungarian+PBS-MLSIPP | 2304 / 295785 | 2238 / 317932 | **2220** / 319670 | — |
| Hungarian+wPBS-MLSIPP | 2257 / 293221 | 2239 / 314496 | 2240 / 320420 | — |
| LNS(1s)+PBS-MLSIPP | 2306 / 297887 | 2276 / 313856 | 2268 / 321035 | — |
| LNS(1s)+wPBS-MLSIPP | 2262 / 290938 | 2268 / 313176 | 2223 / 318784 | — |

**20 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 2 | f 5 | f 10 | f all |
|---|---|---|---|---|
| TP-STA* | 1988 / 380448 | 1833 / 373970 | 1870 / 411608 | — |
| TPTS-STA* | 1951 / 369343 | 2056 / 446907 | 2108 / 447984 | — |
| HBH+MLA* | 1199 / 218565 | 1218 / 240129 | 1182 / 250459 | — |
| CENTRAL-CBS | 1298 / 222186 | 1242 / 249111 | 1297 / 264455 | — |
| CENTRAL-fixed-CBS | 1270 / 218700 | 1269 / 257328 | 1243 / 253549 | — |
| TA-Prioritized-STA* | — | — | — | 1711 / 360676 |
| TA-Hybrid-STA* | — | — | — | **1276** / **300456** |
| LNS(1s)+PBS-MLA* | 1125 / 125886 | 1135 / 148847 | 1118 / 154516 | — |
| LNS(1s)+wPBS-MLA* | 1119 / 124937 | 1131 / 148712 | **1089** / **153178** | — |
| TPTS-MLSIPP | 1819 / 356921 | 1812 / 376632 | 1857 / 383936 | — |
| Hungarian+PBS-MLA* | 1165 / 127177 | 1138 / 148454 | 1116 / 156526 | — |
| Hungarian+wPBS-MLA* | 1123 / **123478** | 1108 / **146731** | 1109 / 155717 | — |
| Hungarian+PBS-MLSIPP | 1130 / 127175 | 1122 / 147836 | 1124 / 156835 | — |
| Hungarian+wPBS-MLSIPP | 1129 / 123915 | 1116 / 147307 | 1092 / 155329 | — |
| LNS(1s)+PBS-MLSIPP | 1129 / 126171 | 1129 / 149184 | 1115 / 155375 | — |
| LNS(1s)+wPBS-MLSIPP | **1117** / 124814 | **1098** / 147663 | 1108 / 153193 | — |

**30 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 2 | f 5 | f 10 | f all |
|---|---|---|---|---|
| TP-STA* | 1697 / 299847 | 1801 / 317683 | 1789 / 357657 | — |
| TPTS-STA* | 1740 / 330294 | 1836 / 338455 | 1680 / 362541 | — |
| HBH+MLA* | 1007 / 131480 | 854 / 168591 | 864 / 181264 | — |
| CENTRAL-CBS | 991 / 138382 | 1012 / 168880 | 938 / 185204 | — |
| CENTRAL-fixed-CBS | 948 / 146407 | 963 / 175150 | 965 / 182867 | — |
| TA-Prioritized-STA* | — | — | — | 1335 / 254934 |
| TA-Hybrid-STA* | — | — | — | **845** / **202746** |
| LNS(1s)+PBS-MLA* | 782 / 77564 | 755 / 98791 | 767 / 105569 | — |
| LNS(1s)+wPBS-MLA* | 772 / 77661 | 776 / 98189 | **735** / **103395** | — |
| TPTS-MLSIPP | 1783 / 308426 | 1659 / 327847 | 1534 / 305808 | — |
| Hungarian+PBS-MLA* | 783 / 78926 | 766 / 99404 | 770 / 105540 | — |
| Hungarian+wPBS-MLA* | 778 / 76796 | 759 / 98039 | 760 / 104850 | — |
| Hungarian+PBS-MLSIPP | **750** / 78939 | 762 / 100092 | 747 / 105323 | — |
| Hungarian+wPBS-MLSIPP | 781 / 77825 | **750** / 98440 | 767 / 105008 | — |
| LNS(1s)+PBS-MLSIPP | 761 / 78908 | 760 / 98509 | 784 / 105485 | — |
| LNS(1s)+wPBS-MLSIPP | 758 / **76096** | 761 / **97332** | 743 / 105130 | — |

**40 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 2 | f 5 | f 10 | f all |
|---|---|---|---|---|
| TP-STA* | 1950 / 330013 | 1843 / 372209 | 1424 / 333450 | — |
| TPTS-STA* | 1713 / 306327 | 1531 / 320248 | × | — |
| HBH+MLA* | 742 / 96670 | 704 / 129735 | 674 / 138837 | — |
| CENTRAL-CBS | 784 / 109984 | 763 / 134607 | 820 / 146440 | — |
| CENTRAL-fixed-CBS | 819 / 105632 | 792 / 136828 | 814 / 150368 | — |
| TA-Prioritized-STA* | — | — | — | 1023 / 198064 |
| TA-Hybrid-STA* | — | — | — | **648** / **157873** |
| LNS(1s)+PBS-MLA* | 614 / 56131 | 609 / 75459 | 614 / 81873 | — |
| LNS(1s)+wPBS-MLA* | 622 / **55936** | 615 / 74522 | **580** / **80709** | — |
| TPTS-MLSIPP | 1537 / 295062 | 1779 / 363038 | 1698 / 370207 | — |
| Hungarian+PBS-MLA* | 626 / 57939 | 609 / 76346 | 597 / 82784 | — |
| Hungarian+wPBS-MLA* | 623 / 57114 | 614 / 74818 | 607 / 81388 | — |
| Hungarian+PBS-MLSIPP | 609 / 57589 | 620 / 75805 | 611 / 83250 | — |
| Hungarian+wPBS-MLSIPP | **602** / 56334 | **577** / 75572 | 611 / 81028 | — |
| LNS(1s)+PBS-MLSIPP | 604 / 56447 | 608 / 75835 | 591 / 82406 | — |
| LNS(1s)+wPBS-MLSIPP | 611 / 56392 | 586 / **74176** | 598 / 80851 | — |

**50 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 2 | f 5 | f 10 | f all |
|---|---|---|---|---|
| TP-STA* | 1750 / 324578 | 1770 / 351043 | 1887 / 375604 | — |
| TPTS-STA* | 1759 / 309098 | 1540 / 289623 | 1682 / 341547 | — |
| HBH+MLA* | 636 / 79803 | 643 / 111081 | 661 / 119049 | — |
| CENTRAL-CBS | 837 / 98282 | 734 / 119160 | 761 / 132160 | — |
| CENTRAL-fixed-CBS | 787 / 95980 | 846 / 123434 | 774 / 128912 | — |
| TA-Prioritized-STA* | — | — | — | 797 / 165904 |
| TA-Hybrid-STA* | — | — | — | **538** / **130309** |
| LNS(1s)+PBS-MLA* | **505** / 46592 | 502 / 64905 | 505 / 71455 | — |
| LNS(1s)+wPBS-MLA* | 517 / 45830 | 505 / 63389 | 513 / 69716 | — |
| TPTS-MLSIPP | 1799 / 306119 | 1810 / 337780 | 1548 / 297032 | — |
| Hungarian+PBS-MLA* | 517 / 47850 | 516 / 65958 | 521 / 71922 | — |
| Hungarian+wPBS-MLA* | 510 / 46462 | 499 / 63993 | 495 / 69591 | — |
| Hungarian+PBS-MLSIPP | 514 / 47637 | 505 / 65783 | 497 / 71010 | — |
| Hungarian+wPBS-MLSIPP | 528 / 46511 | **494** / 64160 | **492** / 70090 | — |
| LNS(1s)+PBS-MLSIPP | 526 / 46378 | 515 / 64887 | 500 / 71690 | — |
| LNS(1s)+wPBS-MLSIPP | 513 / **45767** | 505 / **63234** | 496 / **69499** | — |

Best method per setting:

| Agents | Frequency | Best makespan | Best SWT |
|---:|---:|---|---|
| 10 | 2 | CENTRAL-CBS (2251) | LNS(1s)+wPBS-MLA* (290573) |
| 10 | 5 | HBH+MLA* (2204) | LNS(1s)+PBS-MLA* (311956) |
| 10 | 10 | Hungarian+PBS-MLSIPP (2220) | LNS(1s)+wPBS-MLA* (317759) |
| 10 | all | TA-Hybrid-STA* (2689) | TA-Hybrid-STA* (663504) |
| 20 | 2 | LNS(1s)+wPBS-MLSIPP (1117) | Hungarian+wPBS-MLA* (123478) |
| 20 | 5 | LNS(1s)+wPBS-MLSIPP (1098) | Hungarian+wPBS-MLA* (146731) |
| 20 | 10 | LNS(1s)+wPBS-MLA* (1089) | LNS(1s)+wPBS-MLA* (153178) |
| 20 | all | TA-Hybrid-STA* (1276) | TA-Hybrid-STA* (300456) |
| 30 | 2 | Hungarian+PBS-MLSIPP (750) | LNS(1s)+wPBS-MLSIPP (76096) |
| 30 | 5 | Hungarian+wPBS-MLSIPP (750) | LNS(1s)+wPBS-MLSIPP (97332) |
| 30 | 10 | LNS(1s)+wPBS-MLA* (735) | LNS(1s)+wPBS-MLA* (103395) |
| 30 | all | TA-Hybrid-STA* (845) | TA-Hybrid-STA* (202746) |
| 40 | 2 | Hungarian+wPBS-MLSIPP (602) | LNS(1s)+wPBS-MLA* (55936) |
| 40 | 5 | Hungarian+wPBS-MLSIPP (577) | LNS(1s)+wPBS-MLSIPP (74176) |
| 40 | 10 | LNS(1s)+wPBS-MLA* (580) | LNS(1s)+wPBS-MLA* (80709) |
| 40 | all | TA-Hybrid-STA* (648) | TA-Hybrid-STA* (157873) |
| 50 | 2 | LNS(1s)+PBS-MLA* (505) | LNS(1s)+wPBS-MLSIPP (45767) |
| 50 | 5 | Hungarian+wPBS-MLSIPP (494) | LNS(1s)+wPBS-MLSIPP (63234) |
| 50 | 10 | Hungarian+wPBS-MLSIPP (492) | LNS(1s)+wPBS-MLSIPP (69499) |
| 50 | all | TA-Hybrid-STA* (538) | TA-Hybrid-STA* (130309) |

Runs without a result: TPTS-STA* 1x (System runtime timeout).

#### Sparse SMALL-to-MEDIUM (101 x 81, 500 tasks)

500 runs: 491 completed every task; 7 error(1), 2 timeout.

**10 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 0.2 | f 0.5 | f 1 | f 2 | f 5 | f 10 | f all |
|---|---|---|---|---|---|---|---|
| TP-STA* | 3493 / 274292 | 3387 / 577051 | 3403 / 697346 | 3393 / 750939 | 3385 / 793707 | 3339 / 797985 | 3329 / 782741 |
| TPTS-STA* | 3506 / 270143 | 3368 / 578037 | **3323** / 677149 | 3360 / 734881 | 3343 / 775489 | 3386 / 799267 | 3366 / 791788 |
| HBH+MLA* | **3399** / 240805 | **3330** / 556252 | 3332 / 667694 | **3288** / 723684 | **3245** / 759341 | **3259** / 777478 | 3277 / 775067 |
| CENTRAL-CBS | 3454 / 250279 | 3371 / 576449 | 3432 / 673781 | 3346 / 742358 | 3337 / 771084 | 3286 / 776423 | 3270 / 785029 |
| CENTRAL-fixed-CBS | 3467 / 246417 | 3377 / 562149 | 3358 / 687830 | 3327 / 734702 | 3408 / 767739 | 3386 / 783141 | **3269** / 779687 |
| TA-Prioritized-STA* | — | — | — | — | — | — | 6413 / 1470749 |
| TA-Hybrid-STA* | — | — | — | — | — | — | × |
| LNS(1s)+PBS-MLA* | 3666 / 222404 | 3497 / 460892 | 3446 / 545674 | 3376 / **590073** | 3388 / **615897** | 3418 / **623546** | 3374 / **631487** |
| LNS(1s)+wPBS-MLA* | 3629 / 220266 | 3494 / 459714 | 3376 / **540465** | 3405 / 590133 | 3376 / 621204 | 3359 / 623949 | 3374 / 634791 |
| TPTS-MLSIPP | 3483 / 258893 | 3423 / 590458 | 3329 / 676616 | 3329 / 732119 | 3319 / 773931 | 3337 / 790177 | 3349 / 793750 |
| Hungarian+PBS-MLA* | 3630 / **216459** | 3458 / 458119 | 3427 / 544893 | 3440 / 593234 | 3373 / 617595 | 3415 / 626951 | 3366 / 633567 |
| Hungarian+wPBS-MLA* | 3638 / 222135 | 3443 / 458971 | 3421 / 541634 | 3378 / 592971 | 3334 / 616286 | 3391 / 627401 | 3359 / 634007 |
| Hungarian+PBS-MLSIPP | 3630 / **216459** | 3488 / **456965** | 3427 / 544893 | 3401 / 593392 | 3377 / 619477 | 3415 / 626951 | 3366 / 633567 |
| Hungarian+wPBS-MLSIPP | 3638 / 222135 | 3443 / 458971 | 3421 / 541634 | 3378 / 592969 | 3317 / 616320 | 3390 / 627390 | 3357 / 633974 |
| LNS(1s)+PBS-MLSIPP | 3666 / 222404 | 3497 / 460894 | 3446 / 545676 | 3439 / 590151 | 3396 / 617499 | 3418 / **623546** | 3374 / **631487** |
| LNS(1s)+wPBS-MLSIPP | 3629 / 220266 | 3494 / 459724 | 3376 / **540465** | 3405 / 590133 | 3357 / 620589 | 3386 / 625710 | 3363 / 634609 |

**20 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 0.2 | f 0.5 | f 1 | f 2 | f 5 | f 10 | f all |
|---|---|---|---|---|---|---|---|
| TP-STA* | 2650 / 64498 | 1907 / 233101 | 1899 / 336768 | 1783 / 391702 | 1750 / 404674 | 1779 / 415602 | 1789 / 414107 |
| TPTS-STA* | 2616 / 51568 | 1821 / 205304 | 1728 / 306902 | 1792 / 371959 | 1759 / 398503 | 1773 / 409801 | 1755 / 416182 |
| HBH+MLA* | 2636 / 54216 | **1732** / 184514 | **1692** / 287668 | 1706 / 354506 | **1638** / 380564 | 1736 / 395217 | **1644** / 405040 |
| CENTRAL-CBS | 2615 / 43875 | 1795 / 197684 | 1716 / 301347 | 1774 / 355833 | 1791 / 396041 | 1782 / 403137 | 1695 / 411178 |
| CENTRAL-fixed-CBS | 2683 / 54908 | 1884 / 194323 | 1761 / 301104 | 1760 / 358294 | × | 1736 / 403435 | 1696 / 409032 |
| TA-Prioritized-STA* | — | — | — | — | — | — | 3224 / 744965 |
| TA-Hybrid-STA* | — | — | — | — | — | — | × |
| LNS(1s)+PBS-MLA* | **2582** / 41701 | 1775 / 149692 | 1712 / 227916 | 1719 / **270201** | 1699 / **293149** | 1723 / 306745 | 1702 / 311156 |
| LNS(1s)+wPBS-MLA* | 2593 / 41821 | 1786 / 148578 | 1753 / 228815 | 1702 / 271349 | 1717 / 296067 | **1674** / 303668 | 1671 / 309674 |
| TPTS-MLSIPP | 2594 / 52014 | 1908 / 214917 | 1846 / 319900 | 1722 / 367043 | 1806 / 398685 | 1790 / 416058 | 1705 / 415315 |
| Hungarian+PBS-MLA* | 2593 / 40972 | 1762 / 149774 | 1736 / 228590 | 1737 / 272165 | 1696 / 297004 | 1716 / 304711 | 1666 / 311496 |
| Hungarian+wPBS-MLA* | 2596 / 41649 | 1788 / **147784** | 1741 / **226507** | 1735 / 272920 | 1709 / 294564 | 1698 / 301611 | 1704 / 313884 |
| Hungarian+PBS-MLSIPP | 2595 / 41036 | 1796 / 148875 | 1717 / 228414 | **1697** / 271270 | 1709 / 296703 | 1716 / 304711 | 1666 / 311512 |
| Hungarian+wPBS-MLSIPP | 2616 / 41954 | 1788 / **147784** | 1741 / **226507** | 1727 / 272959 | 1726 / 294513 | 1698 / **301610** | 1704 / 313887 |
| LNS(1s)+PBS-MLSIPP | 2596 / **40969** | 1751 / 149021 | 1729 / 227297 | 1745 / 271187 | 1724 / 293673 | 1728 / 306565 | 1694 / 311666 |
| LNS(1s)+wPBS-MLSIPP | 2596 / 41807 | 1771 / 148094 | 1753 / 228815 | 1702 / 271351 | 1717 / 296059 | 1706 / 303560 | 1688 / **309417** |

**30 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 0.2 | f 0.5 | f 1 | f 2 | f 5 | f 10 | f all |
|---|---|---|---|---|---|---|---|
| TP-STA* | 2690 / 66249 | 1598 / 129175 | 1527 / 223229 | 1462 / 263450 | 1379 / 300756 | 1436 / 310279 | 1348 / 294825 |
| TPTS-STA* | 2613 / 41266 | 1418 / 119813 | 1374 / 200817 | 1325 / 244966 | 1297 / 280482 | 1319 / 294907 | 1310 / 303972 |
| HBH+MLA* | 2629 / 41816 | 1419 / 87026 | 1372 / 178632 | 1383 / 229070 | 1329 / 260827 | 1243 / 273801 | 1212 / 276239 |
| CENTRAL-CBS | 2583 / 40007 | 1357 / 92616 | 1242 / 180692 | 1251 / 239586 | 1250 / 267049 | 1250 / 280004 | 1286 / 285715 |
| CENTRAL-fixed-CBS | **2571** / 39230 | 1374 / 94434 | 1271 / 185958 | 1256 / 236490 | 1255 / 264288 | 1291 / 274997 | 1232 / 285480 |
| TA-Prioritized-STA* | — | — | — | — | — | — | 2277 / 497576 |
| TA-Hybrid-STA* | — | — | — | — | — | — | × |
| LNS(1s)+PBS-MLA* | 2586 / 36779 | 1294 / 69276 | 1229 / 132772 | 1179 / 171607 | **1154** / **197082** | 1186 / 206726 | 1174 / 216724 |
| LNS(1s)+wPBS-MLA* | 2585 / 37071 | 1312 / **67437** | 1227 / **131751** | 1210 / 171882 | 1182 / 198243 | 1172 / 206380 | **1153** / 216160 |
| TPTS-MLSIPP | 2605 / 41648 | 1434 / 103993 | 1350 / 189463 | 1289 / 243127 | 1310 / 284106 | 1307 / 292438 | 1354 / 310306 |
| Hungarian+PBS-MLA* | 2586 / **36504** | 1283 / 68688 | 1223 / 133633 | 1220 / 173047 | 1183 / 197594 | 1187 / 206657 | 1170 / 215888 |
| Hungarian+wPBS-MLA* | 2586 / 36983 | 1299 / 68480 | **1185** / 132258 | **1176** / 172984 | 1164 / 198056 | 1172 / **205845** | 1180 / 216131 |
| Hungarian+PBS-MLSIPP | 2583 / 36713 | 1309 / 70260 | 1228 / 134628 | 1220 / 172532 | 1182 / 198459 | 1176 / 208909 | 1170 / 215888 |
| Hungarian+wPBS-MLSIPP | 2587 / 37039 | 1276 / 69934 | 1200 / 132029 | 1180 / 172543 | 1171 / 197915 | 1173 / 206147 | 1178 / 216391 |
| LNS(1s)+PBS-MLSIPP | 2585 / 36678 | 1307 / 68160 | 1200 / 133033 | 1200 / 171407 | 1161 / 197721 | **1170** / 206733 | 1176 / 217113 |
| LNS(1s)+wPBS-MLSIPP | 2586 / 36987 | **1274** / 67718 | 1231 / 133188 | 1190 / **170944** | 1173 / 197543 | 1185 / 206769 | 1159 / **215283** |

**40 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 0.2 | f 0.5 | f 1 | f 2 | f 5 | f 10 | f all |
|---|---|---|---|---|---|---|---|
| TP-STA* | 2816 / 66949 | 1358 / 85159 | 1169 / 157962 | 1082 / 190088 | 1195 / 219699 | 1157 / 238982 | 1127 / 220963 |
| TPTS-STA* | × | 1346 / 72074 | 1116 / 134635 | 1057 / 172920 | 1058 / 209925 | 1041 / 225468 | 979 / 218386 |
| HBH+MLA* | 2837 / 39113 | 1286 / 53308 | 1025 / 104053 | 976 / 152189 | 1033 / 187934 | 1029 / 197525 | 984 / 204728 |
| CENTRAL-CBS | 2685 / 37189 | 1211 / 49509 | 1008 / 113281 | 942 / 158756 | 970 / 186373 | 934 / 198837 | 896 / 205789 |
| CENTRAL-fixed-CBS | 2680 / 36426 | 1200 / 53766 | 1032 / 111141 | 953 / 156360 | 961 / 191056 | 961 / 202873 | 892 / 206031 |
| TA-Prioritized-STA* | — | — | — | — | — | — | 1656 / 385855 |
| TA-Hybrid-STA* | — | — | — | — | — | — | × |
| LNS(1s)+PBS-MLA* | 2603 / **33656** | **1113** / **39192** | **913** / 86324 | 906 / 123605 | 875 / 149592 | 884 / 159251 | 871 / 166184 |
| LNS(1s)+wPBS-MLA* | 2607 / 34233 | 1167 / 39886 | 937 / 87446 | 910 / 123900 | 880 / 149598 | **871** / 158592 | **858** / **165365** |
| TPTS-MLSIPP | 2686 / 39505 | 1231 / 66163 | 1108 / 136875 | 1112 / 174778 | 1014 / 203815 | 1072 / 218975 | 1032 / 218624 |
| Hungarian+PBS-MLA* | **2602** / 33845 | 1140 / 39522 | 932 / 87253 | 898 / **122932** | 896 / 150451 | 894 / 158969 | 885 / 166233 |
| Hungarian+wPBS-MLA* | 2603 / 34127 | 1149 / 40614 | 921 / 88227 | **895** / 123603 | 894 / **149199** | 877 / 158910 | 862 / 165677 |
| Hungarian+PBS-MLSIPP | 2605 / 33728 | 1131 / 39767 | 914 / 87072 | 904 / 123573 | 887 / 149865 | 884 / 159751 | 879 / 166246 |
| Hungarian+wPBS-MLSIPP | 2607 / 34220 | 1118 / 40598 | 928 / 87210 | 906 / 124098 | 921 / 149489 | 906 / 158775 | 875 / 166068 |
| LNS(1s)+PBS-MLSIPP | 2610 / 33881 | 1152 / 40049 | 922 / **85140** | 912 / 123496 | **868** / 149488 | 876 / **158456** | 907 / 166467 |
| LNS(1s)+wPBS-MLSIPP | 2606 / 34261 | 1153 / 39983 | 930 / 87539 | 899 / 123435 | 902 / 149664 | **871** / 158590 | 872 / 165712 |

**50 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 0.2 | f 0.5 | f 1 | f 2 | f 5 | f 10 | f all |
|---|---|---|---|---|---|---|---|
| TP-STA* | 2691 / 62997 | 1285 / 72360 | 1100 / 132432 | 1069 / 173344 | 1014 / 201674 | 1054 / 213626 | 958 / 188387 |
| TPTS-STA* | 2601 / 36896 | 1158 / 50275 | 1044 / 101643 | 916 / 146554 | 876 / 171141 | 886 / 197793 | 836 / 187285 |
| HBH+MLA* | 2639 / 37348 | 1270 / 51987 | 905 / 76496 | 877 / 127413 | 848 / 159928 | 857 / 171164 | 859 / 171947 |
| CENTRAL-CBS | 2603 / 35610 | 1186 / 40983 | × | 818 / 132373 | × | 807 / 170054 | 800 / 176212 |
| CENTRAL-fixed-CBS | 2589 / 35160 | 1158 / 42511 | 852 / 80874 | 909 / 132406 | 783 / 162272 | 795 / 171135 | 852 / 173868 |
| TA-Prioritized-STA* | — | — | — | — | — | — | 1479 / 313810 |
| TA-Hybrid-STA* | — | — | — | — | — | — | × |
| LNS(1s)+PBS-MLA* | 2588 / **32254** | **1092** / **34903** | 780 / **58813** | 766 / 95977 | 725 / 119251 | 716 / 127257 | 711 / 134324 |
| LNS(1s)+wPBS-MLA* | 2587 / 33172 | 1112 / 35508 | 767 / 59755 | 713 / 95024 | 743 / **118400** | 721 / **126986** | 713 / 133862 |
| TPTS-MLSIPP | 2604 / 36652 | 1142 / 51427 | 946 / 98437 | 900 / 154828 | 956 / 194163 | 1022 / 207674 | 875 / 195747 |
| Hungarian+PBS-MLA* | 2588 / 32270 | 1098 / 34997 | 770 / 60518 | 739 / 95001 | 735 / 119339 | 743 / 127381 | **700** / 135329 |
| Hungarian+wPBS-MLA* | **2586** / 33200 | 1116 / 35623 | 767 / 60543 | 755 / **94337** | 724 / 118442 | 725 / 127130 | 709 / 134569 |
| Hungarian+PBS-MLSIPP | 2588 / 32541 | 1112 / 35001 | 786 / 59702 | 746 / 94985 | 729 / 119018 | 711 / 127388 | **700** / 135332 |
| Hungarian+wPBS-MLSIPP | **2586** / 33221 | 1101 / 35629 | **758** / 59988 | 716 / 95166 | 730 / 118685 | 716 / 127198 | 720 / 134797 |
| LNS(1s)+PBS-MLSIPP | 2588 / 32439 | 1134 / 34906 | 773 / 59676 | **712** / 95458 | 725 / 119155 | **705** / 127453 | 726 / 134525 |
| LNS(1s)+wPBS-MLSIPP | **2586** / 33405 | 1114 / 35547 | 767 / 59753 | 731 / 94950 | **723** / 118964 | 721 / **126986** | 713 / **133860** |

Best method per setting:

| Agents | Frequency | Best makespan | Best SWT |
|---:|---:|---|---|
| 10 | 0.2 | HBH+MLA* (3399) | Hungarian+PBS-MLA*, Hungarian+PBS-MLSIPP (216459) |
| 10 | 0.5 | HBH+MLA* (3330) | Hungarian+PBS-MLSIPP (456965) |
| 10 | 1 | TPTS-STA* (3323) | LNS(1s)+wPBS-MLA*, LNS(1s)+wPBS-MLSIPP (540465) |
| 10 | 2 | HBH+MLA* (3288) | LNS(1s)+PBS-MLA* (590073) |
| 10 | 5 | HBH+MLA* (3245) | LNS(1s)+PBS-MLA* (615897) |
| 10 | 10 | HBH+MLA* (3259) | LNS(1s)+PBS-MLA*, LNS(1s)+PBS-MLSIPP (623546) |
| 10 | all | CENTRAL-fixed-CBS (3269) | LNS(1s)+PBS-MLA*, LNS(1s)+PBS-MLSIPP (631487) |
| 20 | 0.2 | LNS(1s)+PBS-MLA* (2582) | LNS(1s)+PBS-MLSIPP (40969) |
| 20 | 0.5 | HBH+MLA* (1732) | Hungarian+wPBS-MLA*, Hungarian+wPBS-MLSIPP (147784) |
| 20 | 1 | HBH+MLA* (1692) | Hungarian+wPBS-MLA*, Hungarian+wPBS-MLSIPP (226507) |
| 20 | 2 | Hungarian+PBS-MLSIPP (1697) | LNS(1s)+PBS-MLA* (270201) |
| 20 | 5 | HBH+MLA* (1638) | LNS(1s)+PBS-MLA* (293149) |
| 20 | 10 | LNS(1s)+wPBS-MLA* (1674) | Hungarian+wPBS-MLSIPP (301610) |
| 20 | all | HBH+MLA* (1644) | LNS(1s)+wPBS-MLSIPP (309417) |
| 30 | 0.2 | CENTRAL-fixed-CBS (2571) | Hungarian+PBS-MLA* (36504) |
| 30 | 0.5 | LNS(1s)+wPBS-MLSIPP (1274) | LNS(1s)+wPBS-MLA* (67437) |
| 30 | 1 | Hungarian+wPBS-MLA* (1185) | LNS(1s)+wPBS-MLA* (131751) |
| 30 | 2 | Hungarian+wPBS-MLA* (1176) | LNS(1s)+wPBS-MLSIPP (170944) |
| 30 | 5 | LNS(1s)+PBS-MLA* (1154) | LNS(1s)+PBS-MLA* (197082) |
| 30 | 10 | LNS(1s)+PBS-MLSIPP (1170) | Hungarian+wPBS-MLA* (205845) |
| 30 | all | LNS(1s)+wPBS-MLA* (1153) | LNS(1s)+wPBS-MLSIPP (215283) |
| 40 | 0.2 | Hungarian+PBS-MLA* (2602) | LNS(1s)+PBS-MLA* (33656) |
| 40 | 0.5 | LNS(1s)+PBS-MLA* (1113) | LNS(1s)+PBS-MLA* (39192) |
| 40 | 1 | LNS(1s)+PBS-MLA* (913) | LNS(1s)+PBS-MLSIPP (85140) |
| 40 | 2 | Hungarian+wPBS-MLA* (895) | Hungarian+PBS-MLA* (122932) |
| 40 | 5 | LNS(1s)+PBS-MLSIPP (868) | Hungarian+wPBS-MLA* (149199) |
| 40 | 10 | LNS(1s)+wPBS-MLA*, LNS(1s)+wPBS-MLSIPP (871) | LNS(1s)+PBS-MLSIPP (158456) |
| 40 | all | LNS(1s)+wPBS-MLA* (858) | LNS(1s)+wPBS-MLA* (165365) |
| 50 | 0.2 | Hungarian+wPBS-MLA*, Hungarian+wPBS-MLSIPP, LNS(1s)+wPBS-MLSIPP (2586) | LNS(1s)+PBS-MLA* (32254) |
| 50 | 0.5 | LNS(1s)+PBS-MLA* (1092) | LNS(1s)+PBS-MLA* (34903) |
| 50 | 1 | Hungarian+wPBS-MLSIPP (758) | LNS(1s)+PBS-MLA* (58813) |
| 50 | 2 | LNS(1s)+PBS-MLSIPP (712) | Hungarian+wPBS-MLA* (94337) |
| 50 | 5 | LNS(1s)+wPBS-MLSIPP (723) | LNS(1s)+wPBS-MLA* (118400) |
| 50 | 10 | LNS(1s)+PBS-MLSIPP (705) | LNS(1s)+wPBS-MLA*, LNS(1s)+wPBS-MLSIPP (126986) |
| 50 | all | Hungarian+PBS-MLA*, Hungarian+PBS-MLSIPP (700) | LNS(1s)+wPBS-MLSIPP (133860) |

Runs without a result: CENTRAL-CBS 1x (Pathfinding runtime timeout); CENTRAL-CBS 1x (TIMEOUT after 1000 seconds); CENTRAL-fixed-CBS 1x (TIMEOUT after 1000 seconds); TA-Hybrid-STA* 4x (Pathfinding runtime timeout); TA-Hybrid-STA* 1x (System runtime timeout); TPTS-STA* 1x (System runtime timeout).

#### Structured MEDIUM (101 x 81, 1,000 tasks)

80 runs: 56 completed every task; 15 error(1), 9 timeout.

**100 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 50 | f all |
|---|---|---|
| TP-STA* | 1106 / 465769 | — |
| TPTS-STA* | 988 / 439475 | — |
| HBH+MLA* | 905 / 377738 | — |
| CENTRAL-CBS | 794 / 366070 | — |
| CENTRAL-fixed-CBS | × | — |
| TA-Prioritized-STA* | — | **1626** / **698104** |
| TA-Hybrid-STA* | — | × |
| LNS(1s)+PBS-MLA* | 811 / 303663 | — |
| LNS(1s)+wPBS-MLA* | 788 / 302615 | — |
| TPTS-MLSIPP | 958 / 429556 | — |
| Hungarian+PBS-MLA* | 808 / 304060 | — |
| Hungarian+wPBS-MLA* | **787** / 303312 | — |
| Hungarian+PBS-MLSIPP | 815 / 304080 | — |
| Hungarian+wPBS-MLSIPP | 808 / 302521 | — |
| LNS(1s)+PBS-MLSIPP | **787** / 303793 | — |
| LNS(1s)+wPBS-MLSIPP | 798 / **302239** | — |

**200 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 50 | f all |
|---|---|---|
| TP-STA* | 796 / 321166 | — |
| TPTS-STA* | × | — |
| HBH+MLA* | 618 / 217132 | — |
| CENTRAL-CBS | × | — |
| CENTRAL-fixed-CBS | × | — |
| TA-Prioritized-STA* | — | **927** / **386776** |
| TA-Hybrid-STA* | — | × |
| LNS(1s)+PBS-MLA* | **431** / 171361 | — |
| LNS(1s)+wPBS-MLA* | 433 / 171331 | — |
| TPTS-MLSIPP | 625 / 276917 | — |
| Hungarian+PBS-MLA* | 447 / 172698 | — |
| Hungarian+wPBS-MLA* | 436 / 171180 | — |
| Hungarian+PBS-MLSIPP | 446 / **170870** | — |
| Hungarian+wPBS-MLSIPP | 439 / 171598 | — |
| LNS(1s)+PBS-MLSIPP | 439 / 172113 | — |
| LNS(1s)+wPBS-MLSIPP | 436 / 171117 | — |

**300 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 50 | f all |
|---|---|---|
| TP-STA* | 767 / 298550 | — |
| TPTS-STA* | × | — |
| HBH+MLA* | 702 / 173940 | — |
| CENTRAL-CBS | × | — |
| CENTRAL-fixed-CBS | × | — |
| TA-Prioritized-STA* | — | **706** / **291500** |
| TA-Hybrid-STA* | — | × |
| LNS(1s)+PBS-MLA* | 366 / 137892 | — |
| LNS(1s)+wPBS-MLA* | **363** / 138022 | — |
| TPTS-MLSIPP | × | — |
| Hungarian+PBS-MLA* | 371 / 137611 | — |
| Hungarian+wPBS-MLA* | 382 / 138414 | — |
| Hungarian+PBS-MLSIPP | 364 / 137472 | — |
| Hungarian+wPBS-MLSIPP | 373 / 138322 | — |
| LNS(1s)+PBS-MLSIPP | 374 / **137287** | — |
| LNS(1s)+wPBS-MLSIPP | 380 / 137802 | — |

**400 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 50 | f all |
|---|---|---|
| TP-STA* | 782 / 255725 | — |
| TPTS-STA* | × | — |
| HBH+MLA* | 540 / 145729 | — |
| CENTRAL-CBS | × | — |
| CENTRAL-fixed-CBS | × | — |
| TA-Prioritized-STA* | — | × |
| TA-Hybrid-STA* | — | × |
| LNS(1s)+PBS-MLA* | 323 / 118579 | — |
| LNS(1s)+wPBS-MLA* | 303 / 120864 | — |
| TPTS-MLSIPP | × | — |
| Hungarian+PBS-MLA* | **294** / 119276 | — |
| Hungarian+wPBS-MLA* | 298 / 121448 | — |
| Hungarian+PBS-MLSIPP | 314 / 119093 | — |
| Hungarian+wPBS-MLSIPP | 329 / 122147 | — |
| LNS(1s)+PBS-MLSIPP | 315 / **118043** | — |
| LNS(1s)+wPBS-MLSIPP | 310 / 121133 | — |

**500 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f 50 | f all |
|---|---|---|
| TP-STA* | 721 / 270393 | — |
| TPTS-STA* | × | — |
| HBH+MLA* | 522 / 134338 | — |
| CENTRAL-CBS | × | — |
| CENTRAL-fixed-CBS | × | — |
| TA-Prioritized-STA* | — | × |
| TA-Hybrid-STA* | — | × |
| LNS(1s)+PBS-MLA* | × | — |
| LNS(1s)+wPBS-MLA* | 282 / 109750 | — |
| TPTS-MLSIPP | × | — |
| Hungarian+PBS-MLA* | 255 / 105148 | — |
| Hungarian+wPBS-MLA* | 275 / 110250 | — |
| Hungarian+PBS-MLSIPP | 270 / 104659 | — |
| Hungarian+wPBS-MLSIPP | 274 / 110192 | — |
| LNS(1s)+PBS-MLSIPP | **254** / **104490** | — |
| LNS(1s)+wPBS-MLSIPP | 264 / 110103 | — |

Best method per setting:

| Agents | Frequency | Best makespan | Best SWT |
|---:|---:|---|---|
| 100 | 50 | Hungarian+wPBS-MLA*, LNS(1s)+PBS-MLSIPP (787) | LNS(1s)+wPBS-MLSIPP (302239) |
| 100 | all | TA-Prioritized-STA* (1626) | TA-Prioritized-STA* (698104) |
| 200 | 50 | LNS(1s)+PBS-MLA* (431) | Hungarian+PBS-MLSIPP (170870) |
| 200 | all | TA-Prioritized-STA* (927) | TA-Prioritized-STA* (386776) |
| 300 | 50 | LNS(1s)+wPBS-MLA* (363) | LNS(1s)+PBS-MLSIPP (137287) |
| 300 | all | TA-Prioritized-STA* (706) | TA-Prioritized-STA* (291500) |
| 400 | 50 | Hungarian+PBS-MLA* (294) | LNS(1s)+PBS-MLSIPP (118043) |
| 500 | 50 | LNS(1s)+PBS-MLSIPP (254) | LNS(1s)+PBS-MLSIPP (104490) |

Runs without a result: CENTRAL-CBS 4x (TIMEOUT after 1000 seconds); CENTRAL-fixed-CBS 5x (TIMEOUT after 1000 seconds); LNS(1s)+PBS-MLA* 1x (System runtime timeout); TA-Hybrid-STA* 5x (Pathfinding runtime timeout); TA-Prioritized-STA* 2x (Pathfinding runtime timeout); TPTS-MLSIPP 3x (System runtime timeout); TPTS-STA* 4x (System runtime timeout).

#### Structured LARGE (187 x 153, 1,000 agents)

72 runs: 0 completed every task; 61 error(1), 11 timeout.

**1000 agents** (makespan / SWT; — not run; × no result in 1000 s)

| Method | f t1000_f100 | f t2000_f100 | f t3000_f100 | f t4000_f100 | f t5000_f100 | f all |
|---|---|---|---|---|---|---|
| TP-STA* | × | × | × | × | × | — |
| TPTS-STA* | × | × | × | × | × | — |
| HBH+MLA* | × | × | × | × | × | — |
| CENTRAL-CBS | × | × | × | × | × | — |
| CENTRAL-fixed-CBS | × | × | × | × | × | — |
| TA-Prioritized-STA* | — | — | — | — | — | × |
| TA-Hybrid-STA* | — | — | — | — | — | × |
| LNS(1s)+PBS-MLA* | × | × | × | × | × | — |
| LNS(1s)+wPBS-MLA* | × | × | × | × | × | — |
| TPTS-MLSIPP | × | × | × | × | × | — |
| Hungarian+PBS-MLA* | × | × | × | × | × | — |
| Hungarian+wPBS-MLA* | × | × | × | × | × | — |
| Hungarian+PBS-MLSIPP | × | × | × | × | × | — |
| Hungarian+wPBS-MLSIPP | × | × | × | × | × | — |
| LNS(1s)+PBS-MLSIPP | × | × | × | × | × | — |
| LNS(1s)+wPBS-MLSIPP | × | × | × | × | × | — |

No method completed a run within 1000 s.

Runs without a result: CENTRAL-CBS 5x (TIMEOUT after 1000 seconds); CENTRAL-fixed-CBS 5x (TIMEOUT after 1000 seconds); HBH+MLA* 3x (Pathfinding runtime timeout); HBH+MLA* 2x (System runtime timeout); Hungarian+PBS-MLA* 5x (std::bad_alloc); Hungarian+PBS-MLSIPP 5x (std::bad_alloc); Hungarian+wPBS-MLA* 5x (System runtime timeout); Hungarian+wPBS-MLSIPP 4x (System runtime timeout); Hungarian+wPBS-MLSIPP 1x (TIMEOUT after 1000 seconds); LNS(1s)+PBS-MLA* 5x (std::bad_alloc); LNS(1s)+PBS-MLSIPP 5x (std::bad_alloc); LNS(1s)+wPBS-MLA* 5x (System runtime timeout); LNS(1s)+wPBS-MLSIPP 5x (System runtime timeout); TA-Hybrid-STA* 1x (Pathfinding runtime timeout); TA-Prioritized-STA* 1x (Pathfinding runtime timeout); TP-STA* 5x (System runtime timeout); TPTS-MLSIPP 5x (System runtime timeout); TPTS-STA* 5x (System runtime timeout).


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
    --task-type MG_MAPD \
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
| 1 | When to call task assignment | `--assign_trigger` | `ON_FREE_WAITS`<br>`EVERY_TIMESTEP`<br>`ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE` (alias `ON_NEW_TASK_OR_AGENT_BECOMES_FREE`)<br>`ON_AGENT_FREE_OR_AT_PICKUP`<br>`ONCE` | Every trigger with every assignment that plans as a separate step; `ON_FREE_WAITS` with `--replan_on WHEN_ASSIGNED` is TP/TPTS/HBH as published.<br>`ONCE` only for `LKH3_TSP` (`OFFLINE`; `LKH3_TSP_REASSIGN` reassigns after time 0, so it rejects `ONCE`); `LKH3_*` + `PP_TASK_SEQUENCE` only `ON_NEW_OR_DEFERRED_...`, or `ONCE` for `LKH3_TSP` (it replans whole tours after each assignment).<br>A known task's actual release is an event too (`OFFLINE`, `SEMI_ONLINE`).<br>`EVERY_TIMESTEP` assigns and advances every step, which is slow on large maps. |
| 2 | Task assignment | `--assign_method` | `DECOUPLED_GREEDY` (TP)<br>`DECOUPLED_GREEDY_SWAPS` (TPTS)<br>`CENTRALIZED_GREEDY` (HBH)<br>`CENTRAL_HUNGARIAN`<br>`REPEATED_HUNGARIAN`<br>`REPEATED_HUNGARIAN_LNS`<br>`LKH3_TSP`<br>`LKH3_TSP_REASSIGN` | Every method feeds every planner `PP`, `PP_TASK_SEQUENCE`, `PBS`, `wPBS`, `CBS`; `TA_HYBRID_TWO_GROUP` takes only `LKH3_TSP_REASSIGN`.<br>`LKH3_*` need `--tour <file>` and `OFFLINE`.<br>LNS budget is `--lns_time` (CPU seconds per call, early stop `--lns_no_improvement_limit`), so its results vary slightly between runs. |
| 3 | When to plan paths | `--replan_on` | `WHEN_ASSIGNED` (alias `NONE`), or a comma-separated set of `AFTER_ASSIGNMENT`, `EVERY_TIMESTEP`, `GOAL_REACHED`, `PERIODIC` (e.g. `AFTER_ASSIGNMENT,PERIODIC`) | `WHEN_ASSIGNED` = each agent is planned when it is assigned a task, inside the assignment; only TP/TPTS/HBH with `ON_FREE_WAITS`.<br>`PERIODIC` alone plans new assignments at the next period; a set with `EVERY_TIMESTEP` is `EVERY_TIMESTEP`; `GOAL_REACHED` alone is rejected (new assignments would never be planned).<br>`wPBS` needs `PERIODIC` or `EVERY_TIMESTEP`; so does every planner fed by `LKH3_*` (it plans the next tasks of a tour) except `PP_TASK_SEQUENCE`, which plans whole tours: `AFTER_ASSIGNMENT` only.<br>After `CENTRAL_HUNGARIAN` (an agent is planned to its pickup) a `GOAL_REACHED`, `PERIODIC` or `EVERY_TIMESTEP` replan plans the next leg; TA-Hybrid needs `GOAL_REACHED` or `EVERY_TIMESTEP`.<br>A planner that fails also replans at the next timestep. |
| 4 | Which agents to plan | `--agent_selection` | `ALL`<br>`CHANGED`<br>`CHANGED_OR_NOT_CARRYING` | Either, for every planner and assignment, except TP/TPTS/HBH as published (`WHEN_ASSIGNED`): `CHANGED`, since their assignment plans one agent at a time against the others' committed paths (for `ALL`, plan after the assignment: `--replan_on AFTER_ASSIGNMENT`).<br>Kept paths can block the changed agents; when `CHANGED` has reached no goal for 2 (rows + cols) steps while work remains, every agent is planned.<br>`CBS` with `ALL` (one CBS over every agent's goal sequence) is slow with 50 agents. |
| 5 | High-level planner | `--mapf` | `PP` (alias `PP_PER_TASK`)<br>`PP_TASK_SEQUENCE`<br>`CBS`<br>`PBS`<br>`wPBS`<br>`TA_HYBRID_TWO_GROUP` | `PBS`/`wPBS`/`CBS` plan the first `--task_sequence_limit` tasks (built-in default 2; the eight Hungarian/LNS PBS/wPBS configurations set 1, as the original code does); `PP` plans one task.<br>`PP_TASK_SEQUENCE` plans every agent's whole sequence and tentatively plans every remaining agent at each priority level (n(n+1)/2 searches per plan).<br>Every planner runs in every mode the assignment allows.<br>`TA_HYBRID_TWO_GROUP` only with `LKH3_TSP_REASSIGN` (its Group 2 routes free agents under the LKH3 tour's makespan bound and may swap their sequences). |
| 5.1 | wPBS window | `--wpbs_replan_window` | integer, default 10 | At least 2 for `wPBS`.<br>Also the `PERIODIC` replan interval for any planner. |
| 6 | Low-level planner | `--single_agent` | `STA`<br>`MLA`<br>`MLSIPP` (alias `SIPP`)<br>`SIPP2` (alias `2SEG_SIPP`)<br>`SEQ_STA` | Every planner takes any of the five; under `CBS` the search obeys the high-level constraints.<br>`CBS` set up as CENTRAL's plain ECBS (earliest conflict, no ICBS improvements, reference ties) with `MLA` runs the port of the reference ECBS (the same paths) when every planned agent has one released endpoint goal; any other search runs in the framework's CBS.<br>TA-Hybrid plans its carriers (Group 1) and its free agents' fallback paths with it; `SEQ_STA` is its original search. |
| 7 | Task endpoint | `--endpoint_strategy`<br>`--dummy_path 0/1` | `WAIT_OR_NEAREST_SAFE`<br>`WAIT_OR_NEAREST_FREE_NONTASK`<br>`NEAREST_AVAILABLE`<br>`NEAREST_WITH_STRICT_EXCLUSIONS`<br>`PAIRWISE_TASK_THEN_HOME`<br>`RETURN_TO_HOME` | Every strategy, with either `--dummy_path`, for every planner; the Table 1 configurations keep their original value.<br>`--dummy_path 0`: an agent stops at its last goal (or stays where it is) unless another task or agent needs that cell.<br>`PAIRWISE` skips endpoints others need unless the planner is `wPBS` (a windowed plan does not hold them).<br>`CENTRAL` parks idle agents at the strategy's endpoint. |
| 8 | Task setup | `--mode` | `ONLINE`<br>`SEMI_ONLINE` (with `--semi_online_lookahead_batches k`, default 1)<br>`OFFLINE` | `LKH3_*` and `TA_HYBRID_TWO_GROUP` only `OFFLINE` (the tour covers the complete task set); every other method runs in every mode.<br>`CENTRAL_HUNGARIAN` then also matches known tasks released within rows + cols steps (the agent waits at the pickup).<br>TP/TPTS/HBH as published can run `OFFLINE`, but do not use the advance knowledge and perform poorly. |
| 9 | Task type | `--task_type` | `MAPD`<br>`MG_MAPD` (default) | `MAPD` checks that every task in the task file has exactly 2 goals, otherwise errors.<br>Multi-goal task files need `MG_MAPD`. |

General:
- `--config configs/table1/TP.cfg` sets all nine at once; options given on the
  command line override the file, so you can change just one or two. `-a` is
  optional.
- Unsupported combinations stop before the run with an error that lists the
  values that work; the full rules are in `src/config_validate.cpp`.
- `mapd` exits with status 2 when the collision or task-order check fails;
  check the exit status in scripts.
- A planner that cannot plan every agent replans every agent; PBS, wPBS and
  CBS then fall back to prioritized planning, which is retried with the agents
  it missed planned first. TA-Hybrid's Group 1 CBS (bounded to about 2 GB of
  nodes) falls back to planning its carriers one at a time. If that fails too it prints a `Warning:` line; the
  agents it could not plan keep their collision-free committed paths, and it
  plans again at the next timestep. A TA-Hybrid free agent that not even the
  complete prioritized fallback paths place also prints a `Warning:` line; it
  keeps its path and is planned again at the next assignment.
- `--save_output` writes `output/<algorithm>[_sa-…][_mapf-…]_<map>_<task>.txt`:
  the summary, every agent's path (`# Agent paths`, one location per
  timestep), and one line per task (`# Tasks (task_id release_time pickup_loc
  delivery_loc status completion_time pickup_time carrier)`), where
  `pickup_time` is the step at which the method counts the first goal as
  reached and `carrier` is the agent at the final goal at completion (-1 if not
  completed). Most methods count the first arrival; TP and TPTS count, as the
  reference token passing does (and as for completion), the arrival from which
  the agent can stay, and CENTRAL the step at which an agent standing on the
  pickup takes the task, so their agents can be at the goal earlier. Earlier
  versions wrote only the first six columns, with the status column labelled
  `agent`.
- `./mapd --help` lists every option and its values.

### Values in detail

`./mapd --help` lists every option with its values. The values of the nine
components, and the Table 1 configurations (`configs/table1/`) that use them:

**1. When to call task assignment** — `--assign_trigger`

| Value | Meaning | Used by |
|---|---|---|
| `ON_FREE_WAITS` | an agent is free and waiting (one agent per call) | TP, TPTS, HBH |
| `EVERY_TIMESTEP` | every timestep | TA-Hybrid |
| `ON_NEW_OR_DEFERRED_TASK_OR_AGENT_BECOMES_FREE` | a task is released, an assignment method deferred a task (or CENTRAL a pickup), or an agent became free (alias `ON_NEW_TASK_OR_AGENT_BECOMES_FREE`) | CENTRAL-fixed, LNS-PBS, LNS-wPBS |
| `ON_AGENT_FREE_OR_AT_PICKUP` | an agent became free (every agent at time 0), or an agent that carries no task is at the pickup of a released task or picked one up; a new task alone is no event (the reference CENTRAL code), unless no agent carries or is matched to a task (where the reference would wait for good) | CENTRAL |
| `ONCE` | once, at time 0 | TA-Prioritized |

**2. Task assignment** — `--assign_method`

| Value | Meaning | Used by |
|---|---|---|
| `DECOUPLED_GREEDY` | each free agent takes its nearest task; planning inside the assignment, it plans the task as the reference token passing does, to the pickup and then to the delivery, each reached when it can be held (a multi-goal task's goals in between are passed through) | TP |
| `DECOUPLED_GREEDY_SWAPS` | as above; may take over a task another agent would reach later | TPTS |
| `CENTRALIZED_GREEDY` | the globally nearest (agent, task) pairs first | HBH |
| `CENTRAL_HUNGARIAN` | Ma's CENTRAL (the reference code). Phase 1: an agent that carries no task and stands on a released pickup takes that task if no carried task and no agent's path ends at its delivery. The agents that took a task are planned first, with the configured planner (two-stage planning). Phase 2: Hungarian matching of the other agents to released pickups (one task per pickup cell, pickups not on a carried task's goal) or parking endpoints (chosen by the endpoint strategy), costs = path lengths against the carriers' paths. An agent owns a task once it picks it up; until then it is planned to the pickup and rematched at every assignment | CENTRAL |
| `CENTRAL_HUNGARIAN_FIXED` | as above, but a match persists until its task is picked up; only agents without one are matched, to unmatched tasks (Ma's thesis, Sec 6.5.3) | CENTRAL-fixed |
| `REPEATED_HUNGARIAN` | repeated Hungarian matching of agents to task sequences | Hungarian-PBS, Hungarian-wPBS (variants) |
| `REPEATED_HUNGARIAN_LNS` | as above, improved by LNS (`--lns_time`, default 1 s) | LNS-PBS, LNS-wPBS |
| `LKH3_TSP` | task sequences from an LKH3 tour (`--tour`) | TA-Prioritized |
| `LKH3_TSP_REASSIGN` | as above; free agents may take over unstarted tasks | TA-Hybrid |

**3. When to plan paths** — `--replan_on` (comma-separated set)

| Value | Meaning | Used by |
|---|---|---|
| `WHEN_ASSIGNED` (alias `NONE`) | each agent is planned when it is assigned a task, inside the assignment | TP, TPTS, HBH |
| `AFTER_ASSIGNMENT` | after every task assignment | LNS-PBS, TA-Prioritized; with others below |
| `EVERY_TIMESTEP` | every timestep | — |
| `GOAL_REACHED` | an agent picked up a task or reached a later goal with goals left (plans its next leg) | CENTRAL, CENTRAL-fixed, TA-Hybrid (`AFTER_ASSIGNMENT,GOAL_REACHED`) |
| `PERIODIC` | every `--wpbs_replan_window` steps | LNS-wPBS (`AFTER_ASSIGNMENT,PERIODIC`) |

**4. Which agents to plan** — `--agent_selection`

| Value | Meaning | Used by |
|---|---|---|
| `ALL` | every agent | LNS-PBS, LNS-wPBS, TA-Prioritized |
| `CHANGED` | agents whose task goals changed, or whose final cell another task needs; the others keep their paths | TP, TPTS, HBH, TA-Hybrid |
| `CHANGED_OR_NOT_CARRYING` | `CHANGED`, and every agent that carries no task (carriers keep their paths) | CENTRAL, CENTRAL-fixed |

**5. High-level planner** — `--mapf`

| Value | Meaning | Used by |
|---|---|---|
| `PP` (alias `PP_PER_TASK`) | prioritized planning, one task at a time | TP, TPTS, HBH |
| `PP_TASK_SEQUENCE` | prioritized planning of each agent's whole task sequence; at each priority level the agent that finishes last goes first | TA-Prioritized |
| `CBS` | conflict-based search (ICBS by default) over the selected agents' goal sequences, first `--task_sequence_limit` tasks. Configured as plain ECBS (`--cbs_conflict_selection earliest`, bypass, target and rectangle reasoning off, reference ties) on agents that each have one goal to reach and hold, it runs the port of the reference CENTRAL code's ECBS (`src/ecbs_reference.cpp`), which returns that code's paths | CENTRAL, CENTRAL-fixed |
| `PBS` | PBS over each agent's first `--task_sequence_limit` tasks (1 in the LNS/Hungarian configurations) | LNS-PBS |
| `wPBS` | PBS with a planning window (5.1) | LNS-wPBS |
| `TA_HYBRID_TWO_GROUP` | TA-Hybrid's two-group planning | TA-Hybrid |

**5.1 Planning window** — `--wpbs_replan_window w` (default 10, at least 2 for
`wPBS`): wPBS plans and resolves conflicts within w steps; also the interval of
`PERIODIC` replans.

**6. Low-level planner** — `--single_agent`

| Value | Meaning | Used by |
|---|---|---|
| `STA` | space-time A* goal by goal, the reference token-passing search (COBRA's `Agent::AStar`): the last goal is reached when it can be held for good | TP, TPTS |
| `MLA` | multi-label A*, one task per group | HBH, LNS-PBS, LNS-wPBS, CENTRAL, CENTRAL-fixed (one goal per agent at a time) |
| `MLSIPP` (alias `SIPP`) | multi-label SIPP over the whole goal sequence. In TP/TPTS, of the equally fast paths it takes one that reaches the goal as early as possible and waits there, stepping aside when another agent passes, and otherwise waits as early as possible (SIPP would wait beside a busy cell, often in a corridor that the agents planned later need) | the MLSIPP variants |
| `SIPP2` (alias `2SEG_SIPP`) | SIPP, one segment per goal | — |
| `SEQ_STA` | sequential STA*: one leg per goal, each continuing to the final endpoint, which it must be able to hold | TA-Prioritized, TA-Hybrid |

In TA-Hybrid the configured search plans each carrier's next goal and its
parking (the low level of its Group 1 CBS) and the fallback paths of its free
agents; its min-cost-flow routing of free agents and its dummy-path
re-routing are its own, and `SEQ_STA` is its original search. Under `CBS` the
configured search also obeys the high-level constraints and (except
`SEQ_STA`) breaks ties between equally short paths toward fewer conflicts with
the other CBS agents' paths. The searches
measure their horizons from the latest release among their goals, so pickups
released far in the future (`OFFLINE`, a large lookahead) are planned like any
other.

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
but CENTRAL); `0` stops at the last task goal (CENTRAL, CENTRAL-fixed), or
stays where it is when idle; the agent still moves on to the strategy's
endpoint when another task or agent needs that cell (the `WAIT_*` strategies
stay at the last goal unless it is needed). An agent CENTRAL parks, or
matched to a pickup it does not own yet, gets no dummy endpoint: that
endpoint is its final goal; without dummy paths a CENTRAL carrier also ends at
its delivery (CENTRAL rematches every agent at every assignment), and reaches
it when it arrives there to stay (its path may cross the cell earlier), as
the reference code records it. CENTRAL's
destinations are pairwise distinct, and so are the endpoints of the agents a
planner plans. Under a planner that holds the final endpoint for good (all but
`wPBS`), `PAIRWISE` also skips endpoints other tasks or agents need. An idle
agent whose final cell another task needs is replanned (`CHANGED`; in
TA-Hybrid it moves to its rest endpoint). In TP, TPTS and HBH, an idle agent
that must vacate its endpoint while the strategy picks that same endpoint
(another agent's committed path crosses it, and no other endpoint is
better) steps aside and comes back to it, instead of staying in the way. See
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
| `DECOUPLED_GREEDY`, `DECOUPLED_GREEDY_SWAPS`, `CENTRALIZED_GREEDY` | `PP` inside the assignment (`--replan_on WHEN_ASSIGNED`, `ON_FREE_WAITS`); as a separate step `PP`, `PP_TASK_SEQUENCE`, `PBS`, `wPBS`, `CBS` |
| `REPEATED_HUNGARIAN`, `REPEATED_HUNGARIAN_LNS` | `PP`, `PP_TASK_SEQUENCE`, `PBS`, `wPBS`, `CBS` |
| `CENTRAL_HUNGARIAN` | `PP`, `PP_TASK_SEQUENCE`, `PBS`, `CBS` with `GOAL_REACHED`, `PERIODIC` (or `EVERY_TIMESTEP`) replans; `wPBS` |
| `LKH3_TSP` (offline) | `PP_TASK_SEQUENCE`; `PP`, `PBS`, `wPBS`, `CBS` with `PERIODIC` (or `EVERY_TIMESTEP`) replans |
| `LKH3_TSP_REASSIGN` (offline) | `TA_HYBRID_TWO_GROUP`; as `LKH3_TSP` |

Planner requirements: `wPBS` needs a `PERIODIC` (or `EVERY_TIMESTEP`) refresh,
and so do the planners that plan only the next tasks of an LKH3 tour;
`PP_TASK_SEQUENCE` plans the whole tour once, after the assignment
(replanning complete tours at every goal or period is too slow). CENTRAL
plans an agent to its pickup, so a `GOAL_REACHED`, `PERIODIC` (or
`EVERY_TIMESTEP`) replan plans the delivery. `wPBS` needs a
window of at least 2. When the collision or task-order check fails, `mapd`
exits with status 2.

Slow but supported (correct, collision-free, but minutes per run on kiva; the
combination check lists them as slow): `CBS` with `ALL` and 50 agents (one CBS
over every agent's goal sequence reaches its expansion limit at almost every
replan, and prioritized planning takes over); `PP_TASK_SEQUENCE` planning long
sequences, i.e. LKH3 tours with an endpoint other agents' tours cross
(`NEAREST_AVAILABLE`, `NEAREST_WITH_STRICT_EXCLUSIONS`, `WAIT_OR_NEAREST_SAFE`,
`PAIRWISE`; every leg must be able to hold that endpoint), or every known task
in `OFFLINE` (n(n+1)/2 whole-sequence searches per plan, `SEQ_STA` the
slowest, minutes even at kiva frequency 1); `LKH3_TSP` + `PP`/`PBS` with periodic replans at 50 agents and
frequency 0.2; LNS with every task known upfront (`OFFLINE`); 50 agents with
every task known: CENTRAL at kiva frequency 500 (about 4 min with the defaults,
longer with `--dummy_path 1`) and TP with `PBS` and `--agent_selection ALL` in
`OFFLINE` or `SEMI_ONLINE` at frequency 0.2; and CENTRAL
with a single-agent search other than `MLA` (`MLSIPP`, `SIPP2`, `STA`,
`SEQ_STA`): it runs the framework CBS, not the reference ECBS port, with
CENTRAL's expansion limit of 1000000, so a replan whose CBS has no solution
expands up to a million nodes before prioritized planning takes over (kiva a10
frequency 1 with `MLSIPP`: 626 s; with `--cbs_high_level_expansion_limit
10000` such a run takes seconds).

## Algorithm setup

All rows below also receive `--seed 0`. Built-in defaults are
`task_sequence_limit=2` (the eight Hungarian/LNS PBS/wPBS configurations set
1, the original code's `task_truncated_size`), `wpbs_replan_window=10`, `lns_time=1`,
`lns_no_improvement_limit=2000`, CBS focal weight 1.0, and the CBS high-level
expansion limit 10000 (`--cbs_high_level_expansion_limit`; a CBS search that
reaches it fails, so every agent is replanned and then prioritized planning
takes over; the CENTRAL configurations set 1000000: the reference ECBS has
no limit, and with every task released at time 0 its first plans need more
than that, beyond 1000 s with 40 or 50 agents). By default CBS uses ICBS
conflict prioritization
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
| CENTRAL-CBS | `--config configs/table1/CENTRAL.cfg` | Online, the reference code |
| CENTRAL-fixed-CBS | `--config configs/table1/CENTRAL-fixed.cfg` | Online, Ma's thesis Sec 6.5.3 (see the note below) |
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

CENTRAL runs the reference CENTRAL code's components; its ECBS is ported, so
the runs reproduce that code (on kiva small 31 of the 35 rows are identical to
the original results, one more to the reference run on Linux). CENTRAL-fixed
is CENTRAL with Extensions 2 and 4 of Ma's thesis (Sec 6.5.3): assignment
when a task is added or an agent becomes free, and a match persists until its
task is picked up (`CENTRAL_HUNGARIAN_FIXED`), which the thesis calls "less
effective but more efficient". The original results the Excel gives for
CENTRAL-fixed are the reference CENTRAL code's, so CENTRAL-fixed is expected
to be worse than them. (The reference directory `CENTRAL-fixed/` is a
min-cost-flow variant of TA-Hybrid whose flow sees unreleased tasks; it is not
used.)

The eight Hungarian/LNS PBS or wPBS MLA*/MLSIPP configurations plan one task
per agent (`task_sequence_limit = 1`, as the original code); their `(ts 1)`
rows are therefore the same runs, and `--task_sequence_limit 2` gives the
two-task variants. The limit is the number of
tasks PBS, wPBS and CBS plan per agent, and repeated Hungarian/LNS keep that
many tasks per agent (one for PP, all for PP_TASK_SEQUENCE); TP, TPTS, HBH,
CENTRAL and the TA methods do not use it.

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

For TA-Prioritized, every unplanned agent's home is represented as a permanent
reservation during prioritized planning. Consequently, the requesting agent's
own home is its only guaranteed available parking endpoint; the override
preserves that invariant instead of allowing unsafe home swaps. TA-Hybrid
chooses its parking with the configured strategy among all endpoints, pairwise
distinct within a group.

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
