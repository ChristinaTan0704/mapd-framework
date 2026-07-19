#!/usr/bin/env python3
"""persist_results.py — write screen results into all_results.csv (reimplementation rows).

Usage: persist_results.py <RUN_CSV> [ALL_RESULTS_CSV]
For each (method,agents,freq) in RUN_CSV it updates the matching reimplementation row:
  status ok  -> makespan/swt/runtime_s + recomputed ms_gap%/swt_gap%/rt_gap% vs mapped original
  otherwise  -> all N/A
Gaps use the same per-step-ms runtime conversion as compare_method.py for *-MLA* references.
Backs up ALL_RESULTS_CSV to .bak_persist before writing. Leaves untouched any cell not in RUN_CSV.
"""
import csv, sys, shutil

COMPARE = {
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
    "Hungarian+PBS-MLSIPP": "Hungarian+PBS-MLA*",
    "Hungarian+wPBS-MLSIPP": "Hungarian+PBS-MLA*",
    "LNS(1s)+PBS-MLSIPP": "LNS(1s)+PBS-MLA*",
    "LNS(1s)+wPBS-MLSIPP": "Hungarian+wPBS-MLA*",
    "TP-SIPP": "TP-STA*",
    "TPTS-SIPP": "TPTS-STA*",
    "Hungarian+PP-SIPP": "Hungarian+PBS-MLA*",
    "LNS(1s)+PP-SIPP": "LNS(1s)+PBS-MLA*",
}
PER_STEP = {
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
    run_csv = sys.argv[1]
    allp = (
        sys.argv[2]
        if len(sys.argv) > 2
        else "/Users/jiaqit/Desktop/paper/MAPD_framework_imp/all_results.csv"
    )
    shutil.copy(allp, allp + ".bak_persist")
    rows = list(csv.reader(open(allp)))
    orig = {}
    for r in rows:
        if len(r) >= 10 and r[9] == "original":
            orig[(r[0], r[1], r[2])] = (num(r[3]), num(r[4]), num(r[5]))
    fresh = {}
    for r in list(csv.reader(open(run_csv)))[1:]:
        if len(r) < 7:
            continue
        fresh[(r[0], r[1], r[2])] = (r[3], r[4], r[5], r[6])

    def gaps(m, ms, swt, rt, ag, fr):
        og = COMPARE.get(m)
        ov = orig.get((og, ag, fr))
        if ov is None or None in ov:
            return ("-", "-", "-")
        omS, oS, oR = ov
        oR_s = oR * omS / 1000.0 if og in PER_STEP else oR

        def g(a, b):
            a = num(a)
            return "-" if a is None or b in (None, 0) else f"{(a-b)/b*100:+.1f}%"

        return (g(ms, omS), g(swt, oS), g(rt, oR_s))

    upd = na = 0
    for r in rows:
        if len(r) < 11 or r[9] != "reimplementation":
            continue
        key = (r[0], r[1], r[2])
        if key not in fresh:
            continue
        ms, swt, rt, st = fresh[key]
        if st == "ok" and num(ms) is not None:
            r[3], r[4], r[5] = ms, swt, rt
            r[6], r[7], r[8] = gaps(r[0], ms, swt, rt, r[1], r[2])
            upd += 1
        else:
            r[3] = r[4] = r[5] = r[6] = r[7] = r[8] = "N/A"
            na += 1
    with open(allp, "w", newline="") as f:
        csv.writer(f).writerows(rows)
    print(f"persisted {upd} cells ({na} N/A) from {run_csv} -> {allp}")


if __name__ == "__main__":
    main()
