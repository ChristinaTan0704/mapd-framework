#!/usr/bin/env python3
"""compare_method.py — compare a reimplementation run against the reference (original)
results and apply the acceptance gate (reimpl <= 1.20x reference on makespan, SWT, AND runtime).

Usage:
    compare_method.py "<METHOD_LABEL>" <RUN_CSV> [--threshold 0.20] \
        [--all-results /Users/jiaqit/Desktop/paper/MAPD_framework_imp/all_results.csv]

RUN_CSV columns: method,agents,freq,makespan,swt,runtime_s,status  (runtime in SECONDS)

Exit code: 0 if every cell PASSES, 1 if any cell FAILS (so a skill loop can branch on it).
The first FAIL printed is the recommended place to start debugging.
"""
import csv, sys, argparse

# reimplementation method label -> the ORIGINAL method label it is compared against
COMPARE_AGAINST = {
    "TP-STA*": "TP-STA*",
    "TPTS-STA*": "TPTS-STA*",
    "CENTRAL-ECBS": "CENTRAL-ECBS",
    "CENTRAL-ECBS-SIPP": "CENTRAL-ECBS",
    "TA-Hybrid-STA*": "TA-Hybrid-STA*",
    "TA-Prioritized-STA*": "TA-Prioritized-STA*",
    "Hungarian+PBS-MLA*": "Hungarian+PBS-MLA*",
    "Hungarian+wPBS-MLA*": "Hungarian+wPBS-MLA*",
    "LNS(1s)+PBS-MLA*": "LNS(1s)+PBS-MLA*",
    "LNS(1s)+wPBS-MLA*": "LNS(1s)+wPBS-MLA*",
    # MLSIPP = same assignment+MAPF as MLA*, different (faster) low-level: MS/SWT must match MLA*
    "Hungarian+PBS-MLSIPP": "Hungarian+PBS-MLA*",
    "Hungarian+wPBS-MLSIPP": "Hungarian+PBS-MLA*",
    "LNS(1s)+PBS-MLSIPP": "LNS(1s)+PBS-MLA*",
    "LNS(1s)+wPBS-MLSIPP": "Hungarian+wPBS-MLA*",
    # same assignment, different low-level / weaker MAPF
    "TP-SIPP": "TP-STA*",
    "TPTS-SIPP": "TPTS-STA*",
    "Hungarian+PP-SIPP": "Hungarian+PBS-MLA*",
    "LNS(1s)+PP-SIPP": "LNS(1s)+PBS-MLA*",
}


# Reference runtime UNITS differ by source binary:
#  - MGMAPD originals (lifelong_simple / KivaSystemOnline.cpp:488) print
#    "Runtime: total_cpu_ms / makespan" = PER-MAKESPAN-STEP ms. So the CSV
#    runtime for these originals is per-step ms, NOT total seconds.
#  - TP/TPTS/CENTRAL/TA originals are total wall-seconds (run_with_timing.sh).
# reimpl `mapd` prints total ms -> stored as total seconds. To compare fairly we
# convert a per-step-ms reference to total seconds: ref_total_s = perstep_ms * ref_makespan / 1000.
PER_STEP_MS_REF = {
    "Hungarian+PBS-MLA*",
    "Hungarian+wPBS-MLA*",
    "LNS(1s)+PBS-MLA*",
    "LNS(1s)+wPBS-MLA*",
}


def num(x):
    try:
        return float(x)
    except:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("method")
    ap.add_argument("run_csv")
    ap.add_argument("--threshold", type=float, default=0.20)
    ap.add_argument(
        "--all-results",
        default="/Users/jiaqit/Desktop/meta/paper/MAPD_framework_imp/all_results.csv",
    )
    a = ap.parse_args()

    og = COMPARE_AGAINST.get(a.method)
    if og is None:
        print(f"ERROR: unknown method '{a.method}'", file=sys.stderr)
        sys.exit(2)

    # load originals
    orig = {}
    for r in csv.reader(open(a.all_results)):
        if len(r) < 10 or r[9] != "original":
            continue
        orig[(r[0], r[1], r[2])] = (num(r[3]), num(r[4]), num(r[5]))

    rows = list(csv.reader(open(a.run_csv)))[1:]
    thr = a.threshold
    lim = 1.0 + thr
    metrics = ["makespan", "swt", "runtime_s"]
    fails = []
    print(
        f"method={a.method}  compare_against={og}  gate: reimpl <= {lim:.2f}x reference (each metric)"
    )
    print(
        f"{'ag':>3} {'freq':>5} | {'MS reimpl/ref':>18} {'SWT reimpl/ref':>20} {'RT reimpl/ref(s)':>20} | verdict"
    )
    print("-" * 100)
    for r in rows:
        m, ag, fr = r[0], r[1], r[2]
        rv = (num(r[3]), num(r[4]), num(r[5]))
        status = r[6] if len(r) > 6 else ""
        ov = orig.get((og, ag, fr))
        if ov is None:
            print(f"{ag:>3} {fr:>5} | NO REFERENCE for ({og},{ag},{fr}) — skipped")
            continue
        if status not in ("ok", ""):
            fails.append((ag, fr, f"run status={status}"))
            print(f"{ag:>3} {fr:>5} | run FAILED: {status}  <-- DEBUG HERE")
            continue
        cell_fail = []
        cells = []
        for i, name in enumerate(metrics):
            rvi, ovi = rv[i], ov[i]
            # Convert per-step-ms reference runtime -> total seconds for fair compare.
            if (
                i == 2
                and og in PER_STEP_MS_REF
                and ovi is not None
                and ov[0] is not None
            ):
                ovi = ovi * ov[0] / 1000.0
            if rvi is None or ovi is None:
                cells.append(f"{name}=NA")
                continue
            ratio = (rvi / ovi) if ovi else float("inf")
            tag = "" if rvi <= ovi * lim else " !!"
            if tag:
                cell_fail.append(f"{name} {ratio:.2f}x (+{(ratio-1)*100:.0f}%)")
            cells.append(f"{rvi:.3g}/{ovi:.3g}={ratio:.2f}x{tag}")
        verdict = "PASS" if not cell_fail else "FAIL"
        if cell_fail:
            fails.append((ag, fr, "; ".join(cell_fail)))
        mark = "" if not cell_fail else "  <-- DEBUG HERE"
        print(
            f"{ag:>3} {fr:>5} | {cells[0]:>18} {cells[1]:>20} {cells[2]:>20} | {verdict}{mark}"
        )

    print("-" * 100)
    if not fails:
        print(
            f"RESULT: ALL PASS for {a.method} ({len(rows)} cells within {thr*100:.0f}%)"
        )
        sys.exit(0)
    print(
        f"RESULT: {len(fails)} FAIL cell(s) for {a.method}. Start debugging at the FIRST one:"
    )
    ag, fr, why = fails[0]
    print(f"  --> agents={ag} freq={fr} : {why}")
    sys.exit(1)


if __name__ == "__main__":
    main()
