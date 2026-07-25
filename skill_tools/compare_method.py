#!/usr/bin/env python3
"""compare_method.py — compare a reimplementation run against the reference (original)
results and apply the acceptance gate: reimpl <= 1.03x reference on makespan AND SWT
(the +3% quality gate), and reimpl <= 1.20x reference on runtime (the +20% runtime gate).

Usage:
    compare_method.py "<METHOD_LABEL>" <RUN_CSV> [--threshold 0.03] [--rt-threshold 0.20] \
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
    "Hungarian+wPBS-MLSIPP": "Hungarian+wPBS-MLA*",
    "LNS(1s)+PBS-MLSIPP": "LNS(1s)+PBS-MLA*",
    "LNS(1s)+wPBS-MLSIPP": "LNS(1s)+wPBS-MLA*",
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


def norm_key(x):
    """Canonicalize an agents/freq key so xlsx numbers (10, 0.2, 2.0) and csv
    strings ('10','0.2','2') compare equal."""
    s = str(x).strip()
    try:
        f = float(s)
        return str(int(f)) if f == int(f) else ("%g" % f)
    except Exception:
        return s


def load_originals(path):
    """Load source=original rows -> {(method, agents, freq): (ms, swt, rt)}.
    Reads by HEADER NAME so it works with both all_results.xlsx (has a leading
    `index` column) and the legacy all_results.csv. The .xlsx is the source of truth."""
    orig = {}
    if path.lower().endswith((".xlsx", ".xlsm")):
        import openpyxl

        ws = openpyxl.load_workbook(path, data_only=True).active
        rows = list(ws.iter_rows(values_only=True))
    else:
        rows = list(csv.reader(open(path)))
    if not rows:
        return orig
    idx = {str(h).strip(): i for i, h in enumerate(rows[0])}
    need = ("method", "agents", "freq", "makespan", "swt", "runtime_s", "source")
    if not all(k in idx for k in need):
        raise SystemExit(f"ERROR: {path} missing columns; found {list(idx)}")
    for r in rows[1:]:
        if r is None or len(r) <= idx["source"] or str(r[idx["source"]]) != "original":
            continue
        key = (str(r[idx["method"]]).strip(), norm_key(r[idx["agents"]]), norm_key(r[idx["freq"]]))
        orig[key] = (num(r[idx["makespan"]]), num(r[idx["swt"]]), num(r[idx["runtime_s"]]))
    return orig


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("method")
    ap.add_argument("run_csv")
    ap.add_argument("--threshold", type=float, default=0.03,
                    help="MS/SWT quality gate (default 0.03 = +3%%)")
    ap.add_argument("--rt-threshold", type=float, default=0.20,
                    help="runtime gate (default 0.20 = +20%%)")
    ap.add_argument(
        "--all-results",
        default="/Users/jiaqit/Desktop/meta/paper/MAPD_framework_imp/all_results.xlsx",
    )
    a = ap.parse_args()

    og = COMPARE_AGAINST.get(a.method)
    if og is None:
        print(f"ERROR: unknown method '{a.method}'", file=sys.stderr)
        sys.exit(2)

    # load originals (source of truth = all_results.xlsx)
    orig = load_originals(a.all_results)

    rows = list(csv.reader(open(a.run_csv)))[1:]
    thr = a.threshold
    lim = 1.0 + thr
    rt_thr = a.rt_threshold
    rt_lim = 1.0 + rt_thr
    metrics = ["makespan", "swt", "runtime_s"]
    collision_fails = []  # invalid runs (collision / non-ok status) — must be fixed FIRST
    quality_fails = []    # collision-free but outside the quality gate
    print(
        f"method={a.method}  compare_against={og}  gate: MS/SWT <= {lim:.2f}x, runtime <= {rt_lim:.2f}x reference"
    )
    print(
        f"{'ag':>3} {'freq':>5} | {'MS reimpl/ref':>18} {'SWT reimpl/ref':>20} {'RT reimpl/ref(s)':>20} | verdict"
    )
    print("-" * 100)
    for r in rows:
        m, ag, fr = r[0], r[1], r[2]
        rv = (num(r[3]), num(r[4]), num(r[5]))
        status = r[6] if len(r) > 6 else ""
        ov = orig.get((og, norm_key(ag), norm_key(fr)))
        if ov is None:
            print(f"{ag:>3} {fr:>5} | NO REFERENCE for ({og},{ag},{fr}) — skipped")
            continue
        if status not in ("ok", ""):
            collision_fails.append((ag, fr, f"run status={status}"))
            print(f"{ag:>3} {fr:>5} | INVALID run: {status}  <-- COLLISION/FAIL, FIX FIRST")
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
            metric_lim = rt_lim if i == 2 else lim   # runtime uses the 20% gate; MS/SWT use +3%
            tag = "" if rvi <= ovi * metric_lim else " !!"
            if tag:
                cell_fail.append(f"{name} {ratio:.2f}x (+{(ratio-1)*100:.0f}%)")
            cells.append(f"{rvi:.3g}/{ovi:.3g}={ratio:.2f}x{tag}")
        verdict = "PASS" if not cell_fail else "FAIL"
        if cell_fail:
            quality_fails.append((ag, fr, "; ".join(cell_fail)))
        mark = "" if not cell_fail else "  <-- DEBUG HERE"
        print(
            f"{ag:>3} {fr:>5} | {cells[0]:>18} {cells[1]:>20} {cells[2]:>20} | {verdict}{mark}"
        )

    print("-" * 100)
    # Gate order (point 3): collision-free FIRST, then quality.
    if collision_fails:
        print(
            f"RESULT: {len(collision_fails)} INVALID/COLLISION cell(s) for {a.method}. "
            f"Fix correctness BEFORE looking at quality. Start at the FIRST one:"
        )
        ag, fr, why = collision_fails[0]
        print(f"  --> agents={ag} freq={fr} : {why}")
        sys.exit(1)
    if quality_fails:
        print(
            f"RESULT: all cells collision-free, but {len(quality_fails)} exceed the "
            f"gate (MS/SWT +{thr*100:.0f}%, runtime +{rt_thr*100:.0f}%) for {a.method}. Start debugging at the FIRST one:"
        )
        ag, fr, why = quality_fails[0]
        print(f"  --> agents={ag} freq={fr} : {why}")
        sys.exit(1)
    print(
        f"RESULT: ALL PASS for {a.method} ({len(rows)} cells collision-free; "
        f"MS/SWT within +{thr*100:.0f}%, runtime within +{rt_thr*100:.0f}%)"
    )
    sys.exit(0)


if __name__ == "__main__":
    main()
