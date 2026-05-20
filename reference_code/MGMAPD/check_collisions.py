#!/usr/bin/env python3
"""Check output paths for vertex and edge collisions."""
import sys
import os

def parse_paths(filepath):
    with open(filepath) as f:
        header = f.readline().strip().split()
        rows, cols, num_agents = int(header[0]), int(header[1]), int(header[2])
        agents = []
        for _ in range(num_agents):
            line = f.readline().strip().rstrip(';')
            steps = []
            for entry in line.split(';'):
                if not entry:
                    continue
                parts = entry.split(',')
                loc = int(parts[0])
                timestep = int(parts[2])
                steps.append((loc, timestep))
            agents.append(steps)
    return agents

def check_collisions(agents):
    max_t = max(s[-1][1] for s in agents if s)
    collisions = []

    def get_loc(agent_path, t):
        if t < agent_path[0][1]:
            return agent_path[0][0]
        for i, (loc, ts) in enumerate(agent_path):
            if ts == t:
                return loc
        if t > agent_path[-1][1]:
            return agent_path[-1][0]
        return None

    for t in range(max_t + 1):
        locs = {}
        for i, path in enumerate(agents):
            loc = get_loc(path, t)
            if loc is None:
                continue
            if loc < 0:
                continue
            if loc in locs:
                collisions.append(('vertex', t, locs[loc], i, loc))
            else:
                locs[loc] = i

        if t > 0:
            for i in range(len(agents)):
                for j in range(i + 1, len(agents)):
                    loc_i_prev = get_loc(agents[i], t - 1)
                    loc_i_curr = get_loc(agents[i], t)
                    loc_j_prev = get_loc(agents[j], t - 1)
                    loc_j_curr = get_loc(agents[j], t)
                    if (loc_i_prev is not None and loc_j_prev is not None and
                        loc_i_curr is not None and loc_j_curr is not None and
                        loc_i_prev >= 0 and loc_j_prev >= 0 and
                        loc_i_curr >= 0 and loc_j_curr >= 0):
                        if loc_i_prev == loc_j_curr and loc_j_prev == loc_i_curr and loc_i_prev != loc_j_prev:
                            collisions.append(('edge', t, i, j, (loc_i_prev, loc_i_curr)))

    return collisions

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 check_collisions.py <paths_file> [<paths_file2> ...]")
        sys.exit(1)

    for filepath in sys.argv[1:]:
        print(f"=== {os.path.basename(filepath)} ===")
        agents = parse_paths(filepath)
        print(f"  {len(agents)} agents, max timestep = {max(s[-1][1] for s in agents if s)}")
        collisions = check_collisions(agents)
        if collisions:
            print(f"  COLLISIONS FOUND: {len(collisions)}")
            for c in collisions[:20]:
                if c[0] == 'vertex':
                    print(f"    Vertex collision at t={c[1]}: agents {c[2]} and {c[3]} at loc {c[4]}")
                else:
                    print(f"    Edge collision at t={c[1]}: agents {c[2]} and {c[3]} swapping {c[4]}")
            if len(collisions) > 20:
                print(f"    ... and {len(collisions) - 20} more")
        else:
            print("  No collisions found.")

if __name__ == '__main__':
    main()
