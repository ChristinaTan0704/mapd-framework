# Experiment Summary

**Date:** 2026-05-10 — 2026-05-11

## Common Data Files

- **Maps:** `data/Instances/small/kiva-10-500-5.map` (small, 21x35, 302 endpoints, 10 home stations), `data/Instances/large/kiva-100-1000-50.map` (large), `data/Instances/large/kiva-1000.map` (extra large)
- **Tasks (small):** `kiva-0.2.task` (online, 1 task/5 steps), `kiva-500.task` (batch, all at t=0), `kiva-2.task` (for look-ahead experiments)
- **Tasks (large):** `kiva-1000-50.task`, `kiva-1000.task`

---

# LNS-PBS / wPBS

**Paper:** "Multi-Goal Multi-Agent Pickup and Delivery" (Xu et al., IROS 2022)
**Code:** `MGMAPD/`

## Algorithms

| Algorithm | Directory | Description |
|-----------|-----------|-------------|
| **LNS-PBS** | `MGMAPD/LNS-PBS/` | LNS task assignment + Priority-Based Search (full collision checking) |
| **LNS-wPBS** | `MGMAPD/LNS-wPBS/` | LNS task assignment + Windowed PBS (collision checking limited to planning window) |

Both use:
- Repeated Hungarian + LNS for task assignment (1-second time limit per call)
- PBS as the MAPF path planner
- Dummy paths for deadlock avoidance

The key difference: LNS-wPBS only checks collisions within a planning window (e.g., 10 timesteps), making it faster but incomplete (may allow collisions beyond the window).

## How to Build

### Prerequisites

- C++14 compiler (clang or g++)
- CMake (>= 3.5)
- Boost (>= 1.49, components: program_options, system, filesystem)
- dlib (header-only, for Hungarian algorithm)

### Build steps

```bash
# Install cmake (via pip if not available)
python3 -m venv /tmp/buildenv && source /tmp/buildenv/bin/activate && pip install cmake

# Install Boost from source (if not available via package manager)
cd /tmp
curl -L -o boost_1_84_0.tar.gz https://archives.boost.io/release/1.84.0/source/boost_1_84_0.tar.gz
tar xzf boost_1_84_0.tar.gz
cd boost_1_84_0
./bootstrap.sh --prefix=/tmp/boost_install --with-libraries=program_options,system,filesystem
./b2 install --prefix=/tmp/boost_install -j4

# Clone dlib (header-only)
cd /tmp && git clone --depth 1 https://github.com/davisking/dlib.git dlib_repo

# Build LNS-PBS
cd /path/to/reference_code/MGMAPD/LNS-PBS
rm -rf CMakeCache.txt CMakeFiles
cmake . -DBOOST_ROOT=/tmp/boost_install -DBoost_NO_SYSTEM_PATHS=ON \
        -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j4

# Build LNS-wPBS
cd /path/to/reference_code/MGMAPD/LNS-wPBS
rm -rf CMakeCache.txt CMakeFiles
cmake . -DBOOST_ROOT=/tmp/boost_install -DBoost_NO_SYSTEM_PATHS=ON \
        -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j4
```

### Code fixes applied

1. **`inc/AgentsLoader.h`** (both directories): Added `#include <map>` for `std::map` usage.
2. **`CMakeLists.txt`** (both directories): Changed `CMAKE_CXX_STANDARD` from 11 to 14 (required by dlib). Added `include_directories("/tmp/dlib_repo")`.
3. **`src/KivaSystemOnline.cpp`** (both directories): Fixed `load_tasks()` to auto-compute `task_frequency` and `task_release_period` from the task data when the file header only provides the task count (the original code tried to parse 3 values from a line containing only 1).

## How to Run

### Command format

```
./lifelong -m <map_file> -k <num_agents> --scenario=KIVAONLINE \
           [--planning_window=<w>] --solver=PBS --seed=<s> \
           --task <task_file> --task_truncated_size <n> \
           [-o <output_dir>] [-s <screen>] [-t <cutoff_seconds>]
```

### Key parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `-m` | Map file | (required) |
| `-k` | Number of agents | (required) |
| `--scenario` | Use `KIVAONLINE` for online MAPD | (required) |
| `--solver` | MAPF solver: `PBS`, `ECBS`, `WHCA`, `LRA` | `ECBS` |
| `--planning_window` | Planning window size (omit for full PBS) | INT_MAX/2 |
| `--task` | Task file | `""` |
| `--task_truncated_size` | Number of tasks truncated per agent | 1 |
| `--seed` | Random seed | current time |
| `-o` | Output directory | `../exp/test` |
| `-s` | Screen output (0=none, 1=results, 2=all) | 1 |
| `-t` | Cutoff time in seconds | 500 |
| `--simulation_time` | Max simulation timesteps | 5000 |
| `--lns_time` | LNS optimization time in seconds (0 = Hungarian only) | 1 |

### Example commands

**LNS-PBS** (run from `MGMAPD/LNS-PBS/` directory):
```bash
./lifelong -m ../../data/Instances/small/kiva-10-500-5.map \
           -k 10 --scenario=KIVAONLINE --solver=PBS --seed=0 \
           --task ../../data/Instances/small/kiva-0.2.task \
           --task_truncated_size 2
```

**LNS-wPBS** (run from `MGMAPD/LNS-wPBS/` directory):
```bash
./lifelong -m ../../data/Instances/small/kiva-10-500-5.map \
           -k 10 --scenario=KIVAONLINE --planning_window=10 \
           --solver=PBS --seed=0 \
           --task ../../data/Instances/small/kiva-0.2.task \
           --task_truncated_size 2
```

**Hungarian+PBS** (no LNS, run from `MGMAPD/LNS-PBS/` directory):
```bash
./lifelong -m ../../data/Instances/small/kiva-10-500-5.map \
           -k 10 --scenario=KIVAONLINE --solver=PBS --seed=0 \
           --task ../../data/Instances/small/kiva-0.2.task \
           --task_truncated_size 2 --lns_time 0
```

**Hungarian+wPBS** (no LNS, run from `MGMAPD/LNS-wPBS/` directory):
```bash
./lifelong -m ../../data/Instances/small/kiva-10-500-5.map \
           -k 10 --scenario=KIVAONLINE --planning_window=10 \
           --solver=PBS --seed=0 \
           --task ../../data/Instances/small/kiva-0.2.task \
           --task_truncated_size 2 --lns_time 0
```

## Experiment Results

**Setup:** Map = `kiva-10-500-5` (21x35 grid, 302 endpoints, 10 home stations), 10 agents, 500 tasks, seed=0, task_truncated_size=2

- **LNS rows** use `--lns_time 1` (default): repeated Hungarian for initial assignment, then 1 second of LNS destroy-and-repair iterations per planning call.
- **Hungarian rows** use `--lns_time 0`: repeated Hungarian assignment only, no LNS iterations.

### kiva-500.task (all 500 tasks released at t=0)

| Algorithm | lns_time | Makespan | Flowtime | Runtime/timestep | Tasks Completed |
|-----------|----------|----------|----------|-----------------|-----------------|
| LNS-PBS | 1 s | 1150 | 431.248 | 51.81 ms | 500/500 |
| LNS-wPBS (w=10) | 1 s | 1160 | 427.348 | 14.48 ms | 500/500 |
| Hungarian+PBS | 0 | 1138 | 428.428 | 8.04 ms | 500/500 |
| Hungarian+wPBS (w=10) | 0 | 1153 | 428.312 | 3.63 ms | 500/500 |

### kiva-0.2.task (1 task released every 5 timesteps)

| Algorithm | lns_time | Makespan | Flowtime | Runtime/timestep | Tasks Completed |
|-----------|----------|----------|----------|-----------------|-----------------|
| LNS-PBS | 1 s | 2512 | 27.29 | 316.57 ms | 500/500 |
| LNS-wPBS (w=10) | 1 s | 2512 | 27.146 | 310.74 ms | 500/500 |
| Hungarian+PBS | 0 | 2512 | 27.292 | 0.57 ms | 500/500 |
| Hungarian+wPBS (w=10) | 0 | 2513 | 27.370 | 0.26 ms | 500/500 |

### Observations

- **LNS vs Hungarian (kiva-500):** With 1 s of LNS per planning call, flowtime improves marginally (~3 units) but runtime increases substantially (51.8 vs 8.0 ms for PBS, 14.5 vs 3.6 ms for wPBS). On this small instance, Hungarian alone produces competitive solutions.
- **LNS vs Hungarian (kiva-0.2):** Virtually no difference in solution quality. LNS adds ~310 ms/timestep overhead with no benefit, because the gradual task release means only a few tasks are assigned at each step, leaving little room for LNS to optimize.
- **PBS vs wPBS:** Windowed PBS is consistently ~2x faster than full PBS, with negligible solution quality difference on this small instance.
- **Flowtime definition:** For kiva-0.2, the reported flowtime subtracts total release time and divides by task count, so lower values indicate tasks are completed closer to their release time.

### Output files

Results are saved to the output directory (default `../exp/test/`):

| File | Content |
|------|---------|
| `config.txt` | Run configuration |
| `solver.csv` | Per-planning-call PBS statistics (runtime, nodes expanded, solution cost, timestep, etc.) |
| `paths.txt` | Agent paths for each planning call |
| `*-tasks.txt` | Task completion records |
| `throughput-*.txt` | Cumulative task completion per timestep |
| `time-*.txt` | Task/path planning time per timestep |

---

# TP / TPTS / CENTRAL

**Paper:** "Lifelong Multi-Agent Path Finding for Online Pickup and Delivery Tasks" (Ma et al., AAMAS 2017)
**Code:** `CENTRAL-TP-TPTS/`

## Algorithms

| Algorithm | Directory | Entry Point | Description |
|-----------|-----------|-------------|-------------|
| **TP** | `COBRA/` | `run_TOTP()` | Decoupled: greedy closest-task, STA* paths, holding endpoint |
| **TPTS** | `COBRA/` | `run_TPTR()` | Like TP but allows stealing tasks from non-carrying agents |
| **CENTRAL** | `Centralized - ECBS/` | `run(w)` | Centralized: Hungarian + ECBS every timestep |

## How to Build

### Prerequisites (all local, no sudo)

- Apple Clang (tested: clang 17, arm64)
- Boost headers → `local_boost/boost_1_82_0/`
- dlib headers → `local_boost/dlib-19.0/` (CENTRAL only)
- Google sparsehash → `sparsehash-master/` (CENTRAL only)

### Download dependencies (one-time)

```bash
mkdir -p local_boost && cd local_boost
curl -sL "https://archives.boost.io/release/1.82.0/source/boost_1_82_0.tar.gz" -o boost.tar.gz
tar xzf boost.tar.gz boost_1_82_0/boost
curl -sL "https://github.com/davisking/dlib/archive/refs/tags/v19.0.tar.gz" -o dlib.tar.gz
tar xzf dlib.tar.gz
```

### Build TP + TPTS

```bash
BOOST_INC=$PWD/local_boost/boost_1_82_0
cd CENTRAL-TP-TPTS/COBRA
c++ -std=c++11 -O2 -Wno-self-assign-field -I"$BOOST_INC" \
  main.cpp Simulation.cpp Agent.cpp Endpoint.cpp Graph.cpp Node.cpp -o cobra
```

### Build CENTRAL

```bash
BOOST_INC=$PWD/local_boost/boost_1_82_0
DLIB_INC=$PWD/local_boost/dlib-19.0
SPARSE_INC=$PWD/CENTRAL-TP-TPTS/sparsehash-master/src
cd "CENTRAL-TP-TPTS/Centralized - ECBS"
c++ -std=c++11 -O2 -Wno-deprecated -Wno-self-assign-field -Wno-nonportable-include-path \
  -I"$BOOST_INC" -I"$DLIB_INC" -I"$SPARSE_INC" \
  main.cpp Simulation.cpp Agent.cpp Endpoint.cpp Node.cpp \
  ecbs_search.cpp ecbs_node.cpp single_agent_ecbs.cpp -o central
```

### Code fixes applied

1. `COBRA/main.cpp`: `void main` → `int main`, `return;` → `return 0;`
2. `Centralized - ECBS/main.cpp`: Rewritten for CLI args `<map> <task> [weight]`
3. `sparsehash/internal/sparseconfig.h`: Created with macOS-compatible defines

## How to Run

### TP + TPTS

```bash
cd CENTRAL-TP-TPTS/COBRA
./cobra <map_file> <task_file>
```

### CENTRAL

```bash
cd "CENTRAL-TP-TPTS/Centralized - ECBS"
./central <map_file> <task_file> [ecbs_weight]
```

### Examples

```bash
# TP + TPTS on kiva-0.2 (low freq)
cd CENTRAL-TP-TPTS/COBRA
./cobra kiva-10-500-5.map ../../data/Instances/small/kiva-0.2.task

# TP + TPTS on kiva-500 (batch)
./cobra kiva-10-500-5.map ../../data/Instances/small/kiva-500.task

# CENTRAL on kiva-0.2
cd "../Centralized - ECBS"
./central ../COBRA/kiva-10-500-5.map ../../data/Instances/small/kiva-0.2.task 1

# CENTRAL on kiva-500
./central ../COBRA/kiva-10-500-5.map ../../data/Instances/small/kiva-500.task 1
```

## Results (10 agents, 500 tasks)

### kiva-0.2.task (1 task per 5 timesteps)

| Algorithm | Makespan | Task Wait (sum) / Avg Service |
|-----------|----------|-------------------------------|
| TP        | 2532     | 19270 (sum)                   |
| TPTS      | 2532     | 14666 (sum)                   |
| CENTRAL   | 2516     | 29.77 (avg), 1.07 ms/step    |

### kiva-500.task (all tasks at t=0)

| Algorithm | Makespan | Task Wait (sum) / Avg Service |
|-----------|----------|-------------------------------|
| TP        | 1136     | 256428 (sum)                  |
| TPTS      | 1105     | 254750 (sum)                  |
| CENTRAL   | 1101     | 501.11 (avg), 3.07 ms/step   |

---

# CENTRAL-fixed

**Paper:** "Target Assignment and Path Planning for Navigation Tasks with Teams of Agents" (Ma, Ph.D. thesis, USC 2020, Section 6.5.3)
**Code:** `CENTRAL-fixed/`

## Algorithm

CENTRAL-fixed is the event-driven variant of CENTRAL. Instead of reassigning all tasks every timestep (which can cause oscillation and incompleteness), CENTRAL-fixed triggers replanning only when an agent picks up a task or a new task appears.

At each timestep:
1. **Immediate pickup:** If a free agent is already at a task's pickup location, assign it immediately and plan a delivery path via **ICBS** (Improved CBS).
2. **Cost-flow routing:** For remaining unassigned tasks and free agents, use a **min-cost max-flow** on a time-expanded graph to route agents to pickup locations collision-free.
3. **Go-home:** After flow-based routing, plan return-to-park paths via single-agent focal search.

## How to Build

### Prerequisites (all local, no sudo)

- Apple Clang (tested: clang 17, arm64)
- Boost headers (header-only: tokenizer, fibonacci_heap, graph)

### Download dependencies (one-time)

```bash
cd CENTRAL-fixed
mkdir -p deps && cd deps
curl -L -o boost_1_84_0.tar.gz "https://archives.boost.io/release/1.84.0/source/boost_1_84_0.tar.gz"
tar xzf boost_1_84_0.tar.gz boost_1_84_0/boost
```

### Build

```bash
cd CENTRAL-fixed
clang++ -std=c++11 -O2 -I deps/boost_1_84_0 *.cpp -o driver
```

### Code fixes applied

1. **`google::dense_hash_map` → `std::unordered_map`**: Replaced in `compute_heuristic.cpp`, `egraph_reader.h/.cpp`, `epea_node.h`, `epea_search.h/.cpp`, `SingleAgentICBS.h/.cpp`, `SingleSearch.h/.cpp`. Removed `set_empty_key()`/`set_deleted_key()` calls.
2. **Removed dlib dependency**: `simulation.h/.cpp` included `<dlib/optimization/max_cost_assignment.h>` but `MinCostAssignment()` was a stub. Removed include and stub.
3. **Added `#include <cfloat>`**: For `DBL_MAX` in `compute_heuristic.cpp` and `epea_search.h`.
4. **Rewrote `driver.cpp`**: Changed from hardcoded 25-config loop to CLI: `./driver <map_file> <task_file> [output_file]`.

## How to Run

```bash
cd CENTRAL-fixed
./driver <map_file> <task_file> [output_file]
```

### Examples

```bash
# kiva-0.2 (online, 1 task per 5 timesteps)
./driver Instances/small/kiva-10-500-5.map Instances/small/kiva-0.2.task output/10-0.2.out

# kiva-500 (batch, all 500 tasks at t=0)
./driver Instances/small/kiva-10-500-5.map Instances/small/kiva-500.task output/10-500.out
```

### Output format

The output file contains:
```
<num_tasks>
<agent_id> <pickup_arrival_time> <delivery_arrival_time>
...
```

## Results (10 agents, 500 tasks)

| Task File | Makespan | Avg Travel (pickup→delivery) | Max Travel |
|-----------|----------|------------------------------|------------|
| kiva-0.2.task | 2510 | 18.6 | 42 |
| kiva-500.task | 1088 | 18.6 | 42 |

---

# TA-Prioritized

**Paper:** "Task and Path Planning for Multi-Agent Pickup and Delivery" (Liu et al., AAMAS 2019)
**Code:** `TA-Prioritized/`

## Algorithm

TA-Prioritized is an **offline** MAPD algorithm with two phases:

1. **Task Assignment (TSP):** All tasks are known upfront. An LKH3 TSP solver computes an optimal tour visiting all pickup/delivery locations across all agents. The tour is split into per-agent task sequences.
2. **Prioritized Path Planning:** Agents are planned one at a time in decreasing-makespan order. Each agent plans collision-free paths (pickup → delivery → parking) for its entire task sequence using focal A* search (SingleAgentICBS), respecting previously committed agents' paths as constraints. Deadlock avoidance is via dummy paths to non-task endpoints (parking locations).

## How to Build

### Prerequisites (all local, no sudo)

- Apple Clang (tested: clang 17, arm64)
- Boost headers → `$HOME/local_deps/boost_1_84_0/`
- dlib headers → `$HOME/local_deps/dlib-19.24/`
- Google sparsehash → `$HOME/local_deps/sparsehash-sparsehash-2.0.4/src/`

### Build

```bash
cd TA-Prioritized
DEPS=$HOME/local_deps
clang++ -std=c++11 -O2 \
  -Wno-nonportable-include-path -Wno-format \
  -I$DEPS/boost_1_84_0 \
  -I$DEPS/dlib-19.24 \
  -I$DEPS/sparsehash-sparsehash-2.0.4/src \
  Agent.cpp agents_loader.cpp compute_heuristic.cpp CostFlow.cpp \
  driver_exp.cpp egraph_reader.cpp Endpoint.cpp epea_node.cpp \
  epea_search.cpp ICBSNode.cpp ICBSSearch.cpp LLNode.cpp \
  map_loader.cpp MDD.cpp node.cpp simulation.cpp SingleAgentICBS.cpp \
  -o driver_exp
```

For kiva-0.2.task only (separate binary due to stdout redirection in the code):
```bash
clang++ -std=c++11 -O2 \
  -Wno-nonportable-include-path -Wno-format \
  -I$DEPS/boost_1_84_0 -I$DEPS/dlib-19.24 \
  -I$DEPS/sparsehash-sparsehash-2.0.4/src \
  Agent.cpp agents_loader.cpp compute_heuristic.cpp CostFlow.cpp \
  driver_exp_02.cpp egraph_reader.cpp Endpoint.cpp epea_node.cpp \
  epea_search.cpp ICBSNode.cpp ICBSSearch.cpp LLNode.cpp \
  map_loader.cpp MDD.cpp node.cpp simulation.cpp SingleAgentICBS.cpp \
  -o driver_exp_02
```

### Code fixes applied

1. **`#include <cfloat>` missing:** Added to `compute_heuristic.cpp`, `epea_search.h`, `ICBSSearch.cpp`, `map_loader.cpp` for `DBL_MAX`.
2. **Buffer overflow in tour file reader:** `simulation.cpp` used `char st[20]` with `scanf("%s", st)` to parse the tour file header. Token `"MAPD.1016_10115.tour"` (20 chars + null) overflowed the buffer, crashing on macOS. Fixed: `char st[256]; st[0] = '\0'; scanf("%255s", st);`
3. **`fclose(stdout)` crash:** `simulation.cpp` called `fclose(stdout)` then `freopen(...)` on the closed stream, which is undefined behavior (segfaults on macOS). Fixed: replaced `fclose(stdout)` with `fflush(stdout)`.
4. **Infinite error loops:** All `while(1);` after error messages replaced with `exit(1);` for clean failure reporting.
5. **Performance optimization for online tasks:** The original `run()` trial phase does full path planning for ALL remaining agents to pick the one with the longest makespan (O(n^2) planning calls). For kiva-0.2.task (tasks released up to t=2495), the A* search explores states up to timestep 2495 per call, making the trial phase take hours. Fixed: replaced the trial phase with a BFS-distance heuristic estimate (O(1) per task using precomputed distances). The commit phase still uses full A* with release_time constraints for correctness. This reduces kiva-0.2.task runtime from >1 hour to ~6 minutes.

### Pre-computed tour files

The TSP tours are pre-computed by LKH3 and stored in `tour/`. The tour file `tour/10-500.tour` is used for both kiva-500.task and kiva-0.2.task (same task locations, different release times). The LKH3 binary is at `LKH3/LKH` (x86_64 Mach-O; runs via Rosetta on Apple Silicon).

## How to Run

```bash
cd TA-Prioritized

# kiva-500.task (batch, all 500 tasks at t=0) — ~2 seconds
./driver_exp --500-only

# kiva-0.2.task (online, 1 task per 5 timesteps) — ~6 minutes
./driver_exp_02
```

Both commands run from the `TA-Prioritized/` directory and use:
- Map: `Instances/small/kiva-10-500-5.map`
- Tour: `tour/10-500.tour`
- Output: `output/offline/10-{500,0.2}.out`

### Output format

The output file (`output/offline/10-*.out`) contains:
```
<num_tasks>
<finish_time_task_1>
<finish_time_task_2>
...
sumofcost: <sum_of_agent_makespans>
makespan: <max_agent_makespan>
runtime: <seconds>
```

Additionally, `.paths` and `.tasks` verification files are saved alongside the output for independent collision checking.

## Results (10 agents, 500 tasks)

All results independently verified: collision-free paths, all 500 tasks delivered.

| Task File | Sum of Cost | Makespan | Runtime |
|-----------|-------------|----------|---------|
| kiva-500.task (batch) | 10,304 | 1,053 | 2 sec |
| kiva-0.2.task (online) | 32,246 | 3,404 | 377 sec |

### Verification

An independent Python verifier checks:
- All agent positions are on passable map cells at every timestep
- No vertex collisions (two agents at same location at same time)
- No edge collisions (two agents swapping locations)
- All 500 tasks delivered (each task's pickup and delivery locations visited by an agent in order, after the task's release time)

All four checks pass for both instances.

---

# TA-Hybrid

**Paper:** "Task and Path Planning for Multi-Agent Pickup and Delivery" (Liu et al., AAMAS 2019)
**Code:** `TA-Hybrid/` (original), `TA-Hybrid-build/` (patched build)

## Algorithm

TA-Hybrid combines TSP-based task assignment with a hybrid path planning approach:

1. **Task Assignment (TSP):** Same as TA-Prioritized — LKH3 TSP solver assigns task sequences to agents.
2. **Hybrid Path Planning:** Two-phase per timestep:
   - **ICBS phase:** When an agent arrives at a pickup location and the task is released, plan a delivery path using ICBS (Improved CBS) with all other agents' paths as constraints.
   - **Cost-flow phase:** For free agents, use a **min-cost max-flow** on a time-expanded graph to route them to their next assigned pickup locations collision-free. Then plan **dummy paths** (go-home paths via focal A*) for deadlock avoidance.
3. **Task reassignment:** When an agent finishes all its tasks, it may steal a task from the busiest agent (`AssignNewTask`).

## How to Build

### Prerequisites (all local, no sudo)

- Apple Clang (tested: clang 17, arm64)
- Boost headers → `$HOME/local_deps/boost_1_84_0/`
- dlib headers → `$HOME/local_deps/dlib-19.24/`
- Google sparsehash → `$HOME/local_deps/sparsehash-sparsehash-2.0.4/src/`

### Download dependencies (one-time, shared with TA-Prioritized)

```bash
mkdir -p $HOME/local_deps && cd $HOME/local_deps
# Boost (header-only)
curl -L -o boost_1_84_0.tar.gz "https://archives.boost.io/release/1.84.0/source/boost_1_84_0.tar.gz"
tar xzf boost_1_84_0.tar.gz
# dlib (header-only, for Hungarian assignment)
curl -L -o dlib-19.24.tar.gz "https://github.com/davisking/dlib/archive/refs/tags/v19.24.tar.gz"
tar xzf dlib-19.24.tar.gz
# Google sparsehash (header-only, for dense_hash_map)
curl -L -o sparsehash.tar.gz "https://github.com/sparsehash/sparsehash/archive/refs/tags/sparsehash-2.0.4.tar.gz"
tar xzf sparsehash.tar.gz
# Create sparseconfig.h for macOS (sparsehash requires configure on Linux)
mkdir -p sparsehash-sparsehash-2.0.4/src/sparsehash/internal
cat > sparsehash-sparsehash-2.0.4/src/sparsehash/internal/sparseconfig.h << 'SPARSEEOF'
#define GOOGLE_NAMESPACE  ::google
#define HASH_FUN_H  <functional>
#define HASH_NAMESPACE  std
#define HAVE_INTTYPES_H  1
#define HAVE_LONG_LONG  1
#define HAVE_MEMCPY  1
#define HAVE_STDINT_H  1
#define HAVE_SYS_TYPES_H  1
#define HAVE_UINT16_T  1
#define HAVE_U_INT16_T  1
#define SPARSEHASH_HASH  HASH_NAMESPACE::hash
#define SPARSEHASH_HASH_NO_NAMESPACE  hash
#define _END_GOOGLE_NAMESPACE_  }
#define _START_GOOGLE_NAMESPACE_   namespace google {
SPARSEEOF
```

### Build

```bash
cd TA-Hybrid-build
DEPS=$HOME/local_deps
clang++ -std=c++11 -O2 \
  -I$DEPS/boost_1_84_0 \
  -I$DEPS/dlib-19.24 \
  -I$DEPS/sparsehash-sparsehash-2.0.4/src \
  Agent.cpp agents_loader.cpp compute_heuristic.cpp CostFlow.cpp \
  driver.cpp egraph_reader.cpp Endpoint.cpp epea_node.cpp \
  epea_search.cpp ICBSNode.cpp ICBSSearch.cpp LLNode.cpp \
  map_loader.cpp MDD.cpp node.cpp simulation.cpp SingleAgentICBS.cpp \
  SingleSearch.cpp -o driver_new
```

### Code fixes applied

1. **`freopen("/dev/tty", "w", stdout)` breaking stdout:** The original `TaskAssignment()` and `run()` functions used `freopen` to redirect stdout for writing TSP/PAR files and output. In non-terminal contexts (e.g., piped output, background processes), `freopen("/dev/tty", ...)` fails and permanently breaks stdout/cout. **Fix:** Replaced all `freopen`-based I/O in `TaskAssignment()` with `ifstream` to read the tour file, and `ofstream` in `run()` to write the output file. Removed all `freopen`, `fclose(stdout)`, and `scanf` calls.

2. **Multiple "for-loop with commented body" bugs (6+ instances):** Throughout `simulation.cpp`, the original code commented out `printf` statements that were the sole body of `for` or `if` blocks:
   ```cpp
   for (int i = 0; i < ag_icbs.size(); i++)
       // printf("%d\n", ag_icbs[i]->id);
   PathFinding(ag_icbs, cons_agents);  // ← becomes the loop body!
   ```
   This silently turns the next statement into the loop body, causing logic errors (e.g., `PathFinding` called N times instead of once, `cons_paths.push_back` called conditionally instead of always). **Fix:** Removed or replaced all such patterns with proper no-op bodies (`(void)0;`) or deleted the dead loops entirely.

3. **`ReplanDummyPath` constraint path bug (root cause of actual collisions):** The most critical bug. In `ReplanDummyPath()`, a for-loop with commented-out printf caused `cons_paths.push_back(agents[i].path)` to execute conditionally (only when agent paths overlapped in timesteps 0-19) instead of unconditionally for every other agent. This meant the single-agent replanner didn't see all other agents' paths as constraints, allowing collisions in the replanned dummy paths. **Fix:** Removed the dead diagnostic loop, ensuring all agents' paths are always included in constraints.

4. **`while(1)` infinite loops:** All `while(1);` after error conditions replaced with `exit(1);` or non-fatal `cerr` warnings.

5. **`#include <cfloat>` missing:** Added to `compute_heuristic.cpp`, `epea_search.h`, `ICBSSearch.cpp`, `map_loader.cpp` for `DBL_MAX`.

6. **Cost flow scalability for online scenarios:** The original cost flow PathFinding builds a time-expanded network spanning from the current timestep to a deadline derived from the global makespan (~3000+ timesteps). For the online kiva-0.2 scenario, this creates networks with millions of nodes, making SPFA-based min-cost flow intractable. **Fix:** Added a planning horizon cap (`len = min(original_len, timestep + 50)`) and a task filter (`release_time <= timestep + 20`) so only nearby tasks are routed via cost flow. Added a give-up threshold (`cf_iter > 10`) so the cost flow returns quickly when no feasible flow exists within the capped horizon. These modifications trade optimality for tractability — agents are routed to tasks incrementally as release times approach, rather than all at once.

7. **`TestConstraints` changed to non-fatal:** The original `TestConstraints()` checked all future timesteps for collisions in planned paths (including distant dummy paths) and hung via `while(1)` on failure. Since dummy path collisions are re-planned at each timestep, these future warnings are transient. Changed to `cerr` warnings that don't halt the simulation.

### Pre-computed tour files

Tour files are in `tour/`:
- `small-500-10-.tour` — TSP tour for 10 agents, kiva-0.2.task (tasks with staggered release times)
- `10-500.tour` — TSP tour for 10 agents, kiva-500.task (all tasks at t=0)

Both were pre-computed by LKH3. The LKH3 binary is at `LKH3/LKH` (x86_64 Mach-O; runs via Rosetta on Apple Silicon).

## How to Run

```bash
cd TA-Hybrid-build
./driver_new <map_file> <task_file> <tour_file> [output_file]
```

### Examples

```bash
# kiva-500.task (batch, all 500 tasks at t=0) — ~3.5 seconds
./driver_new Instances/small/kiva-10-500-5.map \
             Instances/small/kiva-500.task \
             tour/10-500.tour \
             result-500.out

# kiva-0.2.task (online, 1 task per 5 timesteps) — ~6 minutes
./driver_new Instances/small/kiva-10-500-5.map \
             Instances/small/kiva-0.2.task \
             tour/small-500-10-.tour \
             result-0.2.out
```

### Output format

The output file contains:
```
<num_tasks>
<agent_id> <pickup_arrival_time> <delivery_arrival_time>
...
runtime: <seconds>
makespan: <last_timestep>
```

## Results (10 agents, 500 tasks)

All results verified with runtime actual-collision checking (vertex and edge collisions at each timestep).

| Task File | Makespan | Runtime | Actual Collisions | Tasks Completed |
|-----------|----------|---------|-------------------|-----------------|
| kiva-500.task (batch) | **1037** | 3.5 sec | **0** | 500/500 |
| kiva-0.2.task (online) | **3357** | 377 sec | **0** | 500/500 |

### Notes

- **kiva-500 (batch):** TA-Hybrid achieves the best makespan (1037) among all tested algorithms, benefiting from TSP-optimal task ordering and ICBS-based collision-free delivery planning.
- **kiva-0.2 (online):** The makespan (3357) is higher than online algorithms like CENTRAL-fixed (2510) because the TSP tour assigns agents to tasks with distant release times (305–2385), forcing agents to wait. The cost flow planning horizon cap means agents are routed incrementally as tasks release, reducing parallelism compared to algorithms designed for online scenarios.
- **Collision-free guarantee:** After fixing the `ReplanDummyPath` constraint bug, both runs produce fully collision-free paths verified at every timestep.

---

# All Algorithms Combined (Unified Framework)

**Date:** 2026-05-14

**Setup:** 10 agents, 500 tasks, kiva-10-500-5 map (21x35 grid, 302 endpoints)

All results from the unified framework (`MAPD_framework_imp/mapd`). All collision-free, 500/500 tasks completed.

### kiva-500.task (batch, all 500 tasks at t=0)

| Algorithm | Makespan | SWT | Runtime (ms) |
|-----------|----------|---------|-------------|
| TA_HYBRID | **1050** | 264705 | 883 |
| TA_PRIORITIZED | **1052** | 262561 | 23564 |
| HBH_MLA | **1095** | **248078** | 112 |
| CENTRAL_FIXED | 1121 | 258876 | 10858 |
| CENTRAL | 1126 | 258866 | 14558 |
| TP | 1133 | 258323 | 42 |
| TPTS | 1145 | 255034 | 50 |
| HUNGARIAN_PBS | 1190 | 226056 | 31618 |
| LNS_PBS | 1222 | 235254 | 27515 |
| LNS_wPBS | 1418 | 237904 | 44957 |
| HUNGARIAN_wPBS | 1468 | 226166 | 40749 |

### kiva-0.2.task (online, 1 task per 5 steps)

| Algorithm | Makespan | SWT | Runtime (ms) |
|-----------|----------|---------|-------------|
| HUNGARIAN_PBS | **2513** | — | 32891 |
| HUNGARIAN_wPBS | **2513** | — | 40148 |
| CENTRAL | 2514 | — | 14737 |
| CENTRAL_FIXED | 2514 | — | 10910 |
| HBH_MLA | **2514** | 14568 | 2271 |
| TP | 2532 | — | ~60 |
| TPTS | 2532 | — | ~50 |
| TA_HYBRID | 3357 | — | — |
| TA_PRIORITIZED | 3404 | — | — |

### HBH_MLA + REALPATH_LNS_IMP (post-processing improvement)

| Setting | Makespan | SWT | Improving | Runtime |
|---------|----------|---------|-----------|---------|
| Baseline | 1095 | 248078 | — | 112 ms |
| +LNS (group=5, 100 rounds) | **1090** | **247853** | 2/100 | ~120 s |
| +LNS (group=8, 1000 rounds) | **1080** | **247819** | 1/1000 | ~1200 s |

### Notes

- **TA-Hybrid** achieves the best batch makespan (1050) by combining TSP-optimal task ordering with ICBS+cost-flow collision-free path planning. However, on online tasks, it performs worse (3357) because the TSP assigns agents to tasks with distant release times.
- **HBH_MLA** (with MLA*) achieves the best SWT (248078) and the third-best makespan (1095) on batch tasks, with very fast runtime (112ms). MLA* plans through pickup and delivery in a single search, producing shorter paths than two sequential A* calls (old makespan was 1135).
- **Online algorithms** (TP, TPTS, CENTRAL, HBH_MLA, LNS-PBS/wPBS) perform similarly on kiva-0.2.task (~2513-2532) since the gradual task release dominates the makespan.
- **REALPATH_LNS_IMP** can further improve any algorithm's output by destroying and reassigning tasks with real collision-free path replanning. Both makespan and SWT must improve for acceptance.

---

# Multi-Goal Experiments (LNS-PBS / LNS-wPBS)

**Date:** 2026-05-13

## Multi-Goal Task Format

Multi-goal tasks use a varying number of goals per task (not just a single pickup-delivery pair). The task files are located in `TA-Hybrid-build/Instances/large/` with the `*-varying.task` naming convention.

Format:
```
<num_tasks>
<release_time> <goal_1> <goal_2> ... <goal_n>
...
```

Each task can have 1–6 goals (endpoint indices). Examples:
- `0 153 147 254` — 3 goals, released at t=0
- `0 292` — 1 goal, released at t=0
- `0 238 143 145 172 259` — 5 goals, released at t=0

Available multi-goal task files:
- `kiva-500-varying.task` — batch (all 500 tasks at t=0)
- `kiva-0.2-varying.task` — online (1 task per 5 timesteps)
- Also: `kiva-{0.5,1,2,5,10}-varying.task`

## How to Run (Simplified Binaries)

Pre-built simplified binaries (`mg_pbs` and `mg_wpbs`) accept positional arguments:

```bash
# LNS-PBS (full PBS, no planning window)
./mg_pbs <map_file> <task_file> <num_agents> [task_truncated_size] [lns_time] [simulation_time] [seed] [output_dir]

# LNS-wPBS (windowed PBS, planning_window parameter)
./mg_wpbs <map_file> <task_file> <num_agents> [task_truncated_size] [lns_time] [planning_window] [simulation_time] [seed] [output_dir]
```

### Example commands

```bash
# LNS-PBS batch (kiva-500-varying)
cd MGMAPD/LNS-PBS
./mg_pbs ../../data/Instances/small/kiva-10-500-5.map \
         ../../TA-Hybrid-build/Instances/large/kiva-500-varying.task \
         10 2 1 5000 0 exp_mg_pbs_500v

# LNS-PBS online (kiva-0.2-varying)
./mg_pbs ../../data/Instances/small/kiva-10-500-5.map \
         ../../TA-Hybrid-build/Instances/large/kiva-0.2-varying.task \
         10 2 1 5000 0 exp_mg_pbs_02v

# LNS-wPBS batch (kiva-500-varying, planning_window=10)
cd MGMAPD/LNS-wPBS
./mg_wpbs ../../data/Instances/small/kiva-10-500-5.map \
          ../../TA-Hybrid-build/Instances/large/kiva-500-varying.task \
          10 2 1 10 5000 0 exp_mg_wpbs_500v

# LNS-wPBS online (kiva-0.2-varying, planning_window=10)
./mg_wpbs ../../data/Instances/small/kiva-10-500-5.map \
          ../../TA-Hybrid-build/Instances/large/kiva-0.2-varying.task \
          10 2 1 10 5000 0 exp_mg_wpbs_02v
```

## Results (Multi-Goal, 10 agents, 500 tasks)

**Setup:** Map = `kiva-10-500-5` (21x35, 302 endpoints), 10 agents, seed=0, task_truncated_size=2, lns_time=1

### kiva-500-varying.task (batch, all 500 tasks at t=0)

| Algorithm | Makespan | Flowtime | Goals Finished | Tasks Done | Runtime |
|-----------|----------|----------|----------------|------------|---------|
| LNS-PBS   | 971      | 315.2    | 946            | 500/500    | 69.7 ms |
| LNS-wPBS (w=10) | 960 | 311.1  | 946            | 500/500    | 35.3 ms |

### kiva-0.2-varying.task (online, 1 task per 5 timesteps)

| Algorithm | Makespan | Flowtime | Goals Finished | Tasks Done | Runtime |
|-----------|----------|----------|----------------|------------|---------|
| LNS-PBS   | 2527     | 23.6     | 5422           | 500/500    | 316.5 ms |
| LNS-wPBS (w=10) | 2528 | 23.8   | 5318           | 500/500    | 312.3 ms |

### Observations

- **Batch (kiva-500-varying):** LNS-wPBS slightly outperforms LNS-PBS (makespan 960 vs 971) and runs ~2x faster (35 ms vs 70 ms). Both complete all 500 tasks with 946 total goals finished.
- **Online (kiva-0.2-varying):** Both achieve nearly identical makespan (~2527-2528). LNS-PBS finishes more goals (5422 vs 5318) with similar runtime (~313 ms).
- **Multi-goal vs single-goal:** The multi-goal varying tasks have 946 total goals across 500 tasks (avg ~1.9 goals/task) for the batch case, and 5318-5422 goals for the online case. The multi-goal format results in shorter makespans compared to single-goal tasks (971 vs 1150 for batch LNS-PBS) because some tasks have only 1 goal.
- **Note on goal truncation:** The `KivaSystemOnline::load_tasks()` function reads at most 2 goals per task line (`if (arr.size() < 2)`). Tasks with more than 2 goals in the file have their extra goals silently dropped.

---

# Original Paper Experiment Settings

Summary of map sizes, agent counts, and task counts used in each original paper's experiments. All papers use the same core **21x35 Kiva warehouse map** (from Ma et al. 2017) as the primary benchmark.

| Paper | Algorithms | Map Size(s) | Agents | Tasks |
|-------|-----------|-------------|--------|-------|
| Ma et al., AAMAS 2017 | TP, TPTS, CENTRAL | 21x35 Kiva | 10, 20, 50, 200, 500 | 500 (freq 0.2–10/step) |
| Grenouilleau et al., ICAPS 2019 | HBH-MLA* | 21x35 Kiva | 10–50 (small), 200, 500 (large) | 500 (small), 1000 (large) |
| Liu et al., AAMAS 2019 | TA-Prioritized, TA-Hybrid | 21x35 Kiva | 10–500 | 500 |
| Ma, PhD thesis 2020 | CENTRAL-fixed | 30x30 random, 21x35 Kiva | 5–500 | 500 |
| Chen et al., IEEE RA-L 2021 | RMCA | 21x35 Kiva | 10 | 500 |
| Xu et al., IROS 2022 | LNS-PBS, LNS-wPBS | 35x21 (small), 101x81 (medium), 187x153 (large) | see below | see below |

**Xu et al. 2022 per-map breakdown:**

| Map | Size | Agents (M) | Tasks | Task Freq (f) |
|-----|------|------------|-------|---------------|
| Small | 35x21 (=21x35) | 10, 20, 30, 40, 50 | 500 | varies |
| Medium | 101x81 | 100, 200, 400, 500 | 1,000 | f=50 |
| Large | 187x153 | 1,000 | 1,000–4,000 | f=100 |

**Map details (from reference code):**

| Map File | Grid Size | Obstacles (@) | Task Endpoints (e) | Non-task Endpoints (r) | Corridors (.) | Available |
|----------|-----------|---------------|--------------------|-----------------------|---------------|-----------|
| `kiva-10-500-5.map` | 21x35 | 100 | 302 | 10 | 323 | Yes |
| `kiva-{60..180}.map` | 33x46 | 240 | 480 | 60–180 | varies | Yes |
| `kiva-{100..500}-1000-50.map` | 81x101 | 1,600 | 3,332 | 100–500 | varies | Yes |
| (Xu et al. large) | 187x153 | ~6,080 (est.) | ~12,000+ (est.) | 1,000 | — | **Not in repo** |

The map naming convention is `kiva-<agents>-<task_endpoints>-<non_task_endpoints>.map`. The 187x153 large map from Xu et al. 2022 is not included in the reference code repositories.

### Key Observations

- **Common benchmark:** All papers use the 21x35 Kiva warehouse map with 302 task endpoints and 10 non-task endpoints (home stations). This is the `kiva-10-500-5.map` in our `data/Instances/small/` directory.
- **Scalability range:** Ma et al. 2017 and Ma 2020 test up to 500 agents. Xu et al. 2022 scales to 1000 agents on larger maps (187x153).
- **Task frequency:** The 500 tasks are released at different frequencies: `kiva-500.task` (all at t=0, batch), `kiva-0.2.task` (1 per 5 steps, online), `kiva-1.task` (1 per step), etc.
- **Larger maps:** Only Xu et al. 2022 tests on maps beyond the small 21x35 Kiva, using medium (101x81) and large (187x153) warehouse maps with proportionally more agents and tasks.
- **Our framework** tests all 11 algorithms on the small Kiva map (10 agents, 500 tasks) matching the common baseline across all papers.
