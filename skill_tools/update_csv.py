#!/usr/bin/env python3
"""update_csv.py — merge a per-method full-grid run CSV into all_results.csv.

Updates (or appends) the method's `reimplementation` rows with new
makespan/swt/runtime_s and recomputes signed ms_gap%/swt_gap%/rt_gap% vs the
mapped original (COMPARE_AGAINST). Cells with non-ok status (timeout/collision/
error) are written with N/A metrics and 'n/a' gaps.

Usage: update_csv.py "<METHOD>" <run_csv> [all_results.csv]
Backs up all_results.csv to all_results.csv.bak_fill (once, if not present).
"""
import csv, sys, os, importlib.util, shutil

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    "cm", os.path.join(HERE, "compare_method.py")
)
cm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cm)


def num(x):
    try:
        return float(x)
    except:
        return None


def main():
    method, run_csv = sys.argv[1], sys.argv[2]
    allp = (
        sys.argv[3]
        if len(sys.argv) > 3
        else os.path.join(HERE, "..", "all_results.csv")
    )
    allp = os.path.abspath(allp)
    og = cm.COMPARE_AGAINST[method]

    rows = list(csv.reader(open(allp)))
    hdr = rows[0]
    # original lookup
    orig = {
        (r[0], r[1], r[2]): (num(r[3]), num(r[4]), num(r[5]))
        for r in rows[1:]
        if len(r) >= 10 and r[9] == "original"
    }

    def gap(a, b):
        if a is None or b is None or not b:
            return "n/a"
        return f"{(a-b)/b*100:+.1f}%"

    # index existing reimplementation rows for this method
    idx = {}
    for i, r in enumerate(rows):
        if i == 0:
            continue
        if r[0] == method and len(r) >= 10 and r[9] == "reimplementation":
            idx[(r[1], r[2])] = i

    if not os.path.exists(allp + ".bak_fill"):
        shutil.copy(allp, allp + ".bak_fill")

    upd = add = 0
    for rr in list(csv.reader(open(run_csv)))[1:]:
        ag, fr, ms, swt, rt, st = (
            rr[1],
            rr[2],
            rr[3],
            rr[4],
            rr[5],
            (rr[6] if len(rr) > 6 else "ok"),
        )
        ov = orig.get((og, ag, fr))
        if st == "ok" and num(ms) is not None:
            gms = gap(num(ms), ov[0]) if ov else "n/a"
            gsw = gap(num(swt), ov[1]) if ov else "n/a"
            # reimpl rt is total seconds; convert per-step-ms reference -> total s
            ref_rt = ov[2] if ov else None
            if (
                ov
                and og in cm.PER_STEP_MS_REF
                and ov[2] is not None
                and ov[0] is not None
            ):
                ref_rt = ov[2] * ov[0] / 1000.0
            grt = gap(num(rt), ref_rt) if ov else "n/a"
            newrow = [
                method,
                ag,
                fr,
                ms,
                swt,
                rt,
                gms,
                gsw,
                grt,
                "reimplementation",
                og,
            ]
        else:
            newrow = [
                method,
                ag,
                fr,
                "N/A",
                "N/A",
                "N/A",
                "n/a",
                "n/a",
                "n/a",
                "reimplementation",
                og,
            ]
        if (ag, fr) in idx:
            rows[idx[(ag, fr)]] = newrow
            upd += 1
        else:
            rows.append(newrow)
            add += 1

    with open(allp, "w", newline="") as f:
        csv.writer(f).writerows(rows)
    print(f"[update_csv] {method}: updated {upd}, added {add} reimplementation rows")


if __name__ == "__main__":
    main()
