#!/usr/bin/env python3
"""Verify MGMAPD agent paths are collision-free."""
import sys

def check(paths_file, map_file):
    # Load map (MGMAPD format: no border padding, rows/cols as-is)
    with open(map_file) as f:
        header = f.readline().strip().split(',')
        rows = int(header[0])
        cols = int(header[1])
        f.readline()  # workpoint_num
        f.readline()  # agent_num
        f.readline()  # maxtime
        grid = [False] * (rows * cols)
        for i in range(rows):
            line = f.readline().rstrip('\n')
            for j in range(cols):
                ch = line[j] if j < len(line) else '@'
                grid[i * cols + j] = (ch != '@')

    # Load paths
    with open(paths_file) as f:
        first = f.readline().strip().split()
        r, c, n_agents = int(first[0]), int(first[1]), int(first[2])
        paths = []
        for i in range(n_agents):
            line = f.readline().strip()
            entries = [e for e in line.split(';') if e.strip()]
            locs = []
            for entry in entries:
                parts = entry.split(',')
                loc = int(parts[0])
                locs.append(loc)
            paths.append(locs)

    makespan = max(len(p) for p in paths)
    # Pad shorter paths
    for p in paths:
        while len(p) < makespan:
            p.append(p[-1])

    print(f"Agents: {n_agents}, Makespan: {makespan}, Map: {rows}x{cols}")

    errors = 0

    # Check 1: all positions walkable
    for ag, path in enumerate(paths):
        for t, loc in enumerate(path):
            if loc < 0 or loc >= rows * cols or not grid[loc]:
                print(f"  ILLEGAL POSITION: agent {ag} at t={t}, loc={loc}")
                errors += 1
                if errors > 20:
                    return errors

    # Check 2: vertex collisions
    for t in range(makespan):
        occupied = {}
        for ag in range(n_agents):
            loc = paths[ag][t]
            if loc in occupied:
                print(f"  VERTEX COLLISION: agents {occupied[loc]} and {ag} at loc={loc}, t={t}")
                errors += 1
                if errors > 20:
                    return errors
            occupied[loc] = ag

    # Check 3: edge collisions
    for t in range(1, makespan):
        for a1 in range(n_agents):
            for a2 in range(a1 + 1, n_agents):
                if (paths[a1][t] == paths[a2][t - 1] and
                    paths[a1][t - 1] == paths[a2][t]):
                    print(f"  EDGE COLLISION: agents {a1} and {a2} swap "
                          f"({paths[a1][t-1]}<->{paths[a1][t]}) at t={t}")
                    errors += 1
                    if errors > 20:
                        return errors

    # Check 4: valid moves
    for ag in range(n_agents):
        for t in range(1, len(paths[ag])):
            prev, curr = paths[ag][t - 1], paths[ag][t]
            if prev == curr:
                continue
            diff = curr - prev
            if diff not in (-1, 1, -cols, cols):
                print(f"  INVALID MOVE: agent {ag} from {prev} to {curr} at t={t}")
                errors += 1
            elif abs(diff) == 1 and abs(curr % cols - prev % cols) != 1:
                print(f"  WRAPAROUND: agent {ag} from {prev} to {curr} at t={t}")
                errors += 1

    if errors == 0:
        print("  ALL CHECKS PASSED - collision-free")
    else:
        print(f"  FOUND {errors} ERRORS")
    return errors


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <paths_file> <map_file>")
        sys.exit(1)
    errors = check(sys.argv[1], sys.argv[2])
    sys.exit(1 if errors > 0 else 0)
