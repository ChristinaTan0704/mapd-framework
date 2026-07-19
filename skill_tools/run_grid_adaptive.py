#!/usr/bin/env python3
"""run_grid_adaptive.py — run a method across the full agent grid x all freqs,
capping each cell's wall-clock at (1.20 * reference_runtime_s + margin). A cell
that can't beat ~1.2x the reference fails the runtime gate anyway, so there is no
point running it longer; this keeps the fill fast while still recording real
values for cells that complete.

Usage: run_grid_adaptive.py "<METHOD>" <out_csv> [agents_csv]
  agents_csv defaults to 10,20,30,40,50 (TA methods auto-use freq 500 only).
Output CSV: method,agents,freq,makespan,swt,runtime_s,status   (runtime in SECONDS)
"""
import csv, sys, os, subprocess, math, importlib.util

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATA = "/Users/jiaqit/Desktop/paper/reference_code/data/Instances/small"
TOUR = os.path.join(ROOT, "tour")
spec = importlib.util.spec_from_file_location(
    "cm", os.path.join(HERE, "compare_method.py")
)
cm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cm)

FLAGS = {
    "TP-STA*": ("TP", []),
    "TPTS-STA*": ("TPTS", []),
    "CENTRAL-ECBS": ("CENTRAL", []),
    "CENTRAL-ECBS-SIPP": ("CENTRAL", ["--sipp"]),
    "TA-Hybrid-STA*": ("TA_HYBRID", ["--tour", "TOUR/AGENTS-500.tour"]),
    "TA-Prioritized-STA*": ("TA_PRIORITIZED", ["--tour", "TOUR/AGENTS-500.tour"]),
    "Hungarian+PBS-MLA*": ("HUNGARIAN_PBS", []),
    "Hungarian+wPBS-MLA*": ("HUNGARIAN_wPBS", []),
    "LNS(1s)+PBS-MLA*": ("LNS_PBS", ["--lns_time", "1"]),
    "LNS(1s)+wPBS-MLA*": ("LNS_wPBS", ["--lns_time", "1"]),
    "Hungarian+PBS-MLSIPP": ("HUNGARIAN_PBS", ["--sipp"]),
    "Hungarian+wPBS-MLSIPP": ("HUNGARIAN_wPBS", ["--sipp"]),
    "LNS(1s)+PBS-MLSIPP": ("LNS_PBS", ["--lns_time", "1", "--sipp"]),
    "LNS(1s)+wPBS-MLSIPP": ("LNS_wPBS", ["--lns_time", "1", "--sipp"]),
    "TP-SIPP": ("TP", ["--single_agent", "MLA", "--sipp"]),
    "TPTS-SIPP": ("TPTS", ["--single_agent", "MLA", "--sipp"]),
    "Hungarian+PP-SIPP": ("HUNGARIAN_PBS", ["--mapf", "PP", "--sipp"]),
    "LNS(1s)+PP-SIPP": ("LNS_PBS", ["--mapf", "PP", "--lns_time", "1", "--sipp"]),
}
FREQS = ["0.2", "0.5", "1", "2", "5", "10", "500"]


def num(x):
    try:
        return float(x)
    except:
        return None


def main():
    method, outcsv = sys.argv[1], sys.argv[2]
    agents = (sys.argv[3] if len(sys.argv) > 3 else "10,20,30,40,50").split(",")
    algo, extra_t = FLAGS[method]
    og = cm.COMPARE_AGAINST[method]
    # ref runtime: store (runtime, makespan). For MGMAPD-family the CSV runtime is
    # PER-STEP ms (see compare_method.PER_STEP_MS_REF) -> convert to total seconds
    # = perstep * makespan / 1000 before using as the adaptive timeout cap.
    ref = {
        (r[1], r[2]): (num(r[5]), num(r[3]))
        for r in csv.reader(open(os.path.join(ROOT, "all_results.csv")))
        if len(r) >= 10 and r[0] == og and r[9] == "original"
    }

    out = open(outcsv, "w", newline="")
    w = csv.writer(out)
    w.writerow(["method", "agents", "freq", "makespan", "swt", "runtime_s", "status"])
    out.flush()
    for ag in agents:
        freqs = ["500"] if method.startswith("TA-") else FREQS
        extra = [e.replace("TOUR", TOUR).replace("AGENTS", ag) for e in extra_t]
        for fr in freqs:
            mp = f"{DATA}/kiva-{ag}-500-5.map"
            tk = f"{DATA}/kiva-{fr}.task"
            rr = ref.get((ag, fr))
            ref_total_s = None
            if rr and rr[0] is not None:
                if og in cm.PER_STEP_MS_REF and rr[1] is not None:
                    ref_total_s = rr[0] * rr[1] / 1000.0  # per-step ms -> total s
                else:
                    ref_total_s = rr[0]
            cap = (
                max(20, int(math.ceil(1.2 * ref_total_s)) + 10) if ref_total_s else 600
            )
            cmd = (
                [
                    "gtimeout",
                    str(cap),
                    os.path.join(ROOT, "mapd"),
                    "-m",
                    mp,
                    "-t",
                    tk,
                    "-a",
                    algo,
                ]
                + extra
                + ["-s", "1"]
            )
            try:
                p = subprocess.run(
                    cmd, capture_output=True, text=True, timeout=cap + 30
                )
                o = p.stdout
                rc = p.returncode
            except subprocess.TimeoutExpired:
                o = ""
                rc = 124
            ms = swt = rtms = None
            coll_fail = False
            for ln in o.splitlines():
                if "Finishing Timestep:" in ln:
                    ms = ln.split()[-1]
                elif "Sum of Task Waiting Time:" in ln:
                    swt = ln.split()[-1]
                elif "Total runtime:" in ln:
                    parts = ln.split()
                    rtms = parts[-2] if len(parts) >= 2 else None
                elif "COLLISION" in ln and ("FAILED" in ln or "DETECTED" in ln):
                    coll_fail = True
            status = "ok"
            if rc == 124:
                status = "timeout"
            elif coll_fail:
                status = "collision"
            elif ms is None:
                status = "error"
            rts = f"{num(rtms)/1000.0:.3f}" if rtms is not None else "N/A"
            w.writerow([method, ag, fr, ms or "N/A", swt or "N/A", rts, status])
            out.flush()
            print(
                f"  {method} {ag}/{fr}: cap={cap}s ms={ms} swt={swt} rt={rts}s [{status}]",
                flush=True,
            )
    out.close()


if __name__ == "__main__":
    main()
