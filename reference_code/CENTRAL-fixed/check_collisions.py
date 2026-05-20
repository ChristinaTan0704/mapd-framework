#!/usr/bin/env python3
"""Verify agent paths are collision-free (vertex and edge collisions)."""
import sys

def check(path_file, map_file):
    # Load map to verify positions are walkable
    with open(map_file) as f:
        header = f.readline().strip().split(',')
        rows = int(header[0]) + 2
        cols = int(header[1]) + 2
        f.readline()  # workpoint_num
        f.readline()  # agent_num
        f.readline()  # maxtime
        grid = [False] * (rows * cols)
        for i in range(1, rows - 1):
            line = f.readline().rstrip('\n')
            for j in range(1, cols - 1):
                ch = line[j - 1] if (j - 1) < len(line) else '@'
                grid[i * cols + j] = (ch != '@')
        # borders blocked
        for i in range(rows):
            grid[i * cols] = False
            grid[i * cols + cols - 1] = False
        for j in range(1, cols - 1):
            grid[j] = False
            grid[(rows - 1) * cols + j] = False

    # Load paths
    with open(path_file) as f:
        first = f.readline().strip().split()
        n_agents = int(first[0])
        makespan = int(first[1])
        paths = []
        for i in range(n_agents):
            locs = list(map(int, f.readline().strip().split()))
            paths.append(locs)

    print(f"Agents: {n_agents}, Makespan: {makespan}, Path lengths: {[len(p) for p in paths]}")

    errors = 0

    # Check 1: all positions are walkable
    for ag, path in enumerate(paths):
        for t, loc in enumerate(path):
            if loc < 0 or loc >= rows * cols or not grid[loc]:
                print(f"  ILLEGAL POSITION: agent {ag} at t={t}, loc={loc}")
                errors += 1
                if errors > 20:
                    print("  ... too many errors, stopping")
                    return errors

    # Check 2: vertex collisions
    for t in range(makespan + 1):
        occupied = {}
        for ag in range(n_agents):
            loc = paths[ag][t]
            if loc in occupied:
                print(f"  VERTEX COLLISION: agents {occupied[loc]} and {ag} at loc={loc}, t={t}")
                errors += 1
                if errors > 20:
                    print("  ... too many errors, stopping")
                    return errors
            occupied[loc] = ag

    # Check 3: edge collisions (agents swap positions)
    for t in range(1, makespan + 1):
        for a1 in range(n_agents):
            for a2 in range(a1 + 1, n_agents):
                if (paths[a1][t] == paths[a2][t - 1] and
                    paths[a1][t - 1] == paths[a2][t]):
                    print(f"  EDGE COLLISION: agents {a1} and {a2} swap "
                          f"({paths[a1][t-1]}<->{paths[a1][t]}) at t={t}")
                    errors += 1
                    if errors > 20:
                        print("  ... too many errors, stopping")
                        return errors

    # Check 4: movement validity (only adjacent cells or wait)
    for ag in range(n_agents):
        for t in range(1, len(paths[ag])):
            prev = paths[ag][t - 1]
            curr = paths[ag][t]
            if prev == curr:
                continue
            diff = curr - prev
            if diff not in (-1, 1, -cols, cols):
                print(f"  INVALID MOVE: agent {ag} from {prev} to {curr} at t={t} (diff={diff})")
                errors += 1
            elif abs(diff) == 1 and abs(curr % cols - prev % cols) != 1:
                print(f"  WRAPAROUND MOVE: agent {ag} from {prev} to {curr} at t={t}")
                errors += 1

    if errors == 0:
        print("  ALL CHECKS PASSED - paths are collision-free")
    else:
        print(f"  FOUND {errors} ERRORS")
    return errors


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <path_file> <map_file>")
        sys.exit(1)
    errors = check(sys.argv[1], sys.argv[2])
    sys.exit(1 if errors > 0 else 0)
