#!/usr/bin/env python3
# Regenerate comparison_original_vs_reimpl.csv from all_results.csv (orig always; reimpl blank if missing).
import csv

ROOT = "/Users/jiaqit/Desktop/2026 meta docs/paper/MAPD_framework_imp"
ALL = [
    "TP-STA*",
    "TPTS-STA*",
    "CENTRAL-ECBS",
    "CENTRAL-ECBS-SIPP",
    "TA-Hybrid-STA*",
    "TA-Prioritized-STA*",
    "Hungarian+PBS-MLA*",
    "Hungarian+wPBS-MLA*",
    "Hungarian+PBS-MLSIPP",
    "Hungarian+wPBS-MLSIPP",
    "Hungarian+PP-SIPP",
    "TP-SIPP",
    "TPTS-SIPP",
    "LNS(1s)+PBS-MLA*",
    "LNS(1s)+wPBS-MLA*",
    "LNS(1s)+PBS-MLSIPP",
    "LNS(1s)+wPBS-MLSIPP",
    "LNS(1s)+PP-SIPP",
]
CMP = {
    "TP-STA*": "TP-STA*",
    "TPTS-STA*": "TPTS-STA*",
    "CENTRAL-ECBS": "CENTRAL-ECBS",
    "CENTRAL-ECBS-SIPP": "CENTRAL-ECBS",
    "TA-Hybrid-STA*": "TA-Hybrid-STA*",
    "TA-Prioritized-STA*": "TA-Prioritized-STA*",
    "Hungarian+PBS-MLA*": "Hungarian+PBS-MLA*",
    "Hungarian+wPBS-MLA*": "Hungarian+wPBS-MLA*",
    "Hungarian+PBS-MLSIPP": "Hungarian+PBS-MLA*",
    "Hungarian+wPBS-MLSIPP": "Hungarian+PBS-MLA*",
    "Hungarian+PP-SIPP": "Hungarian+PBS-MLA*",
    "TP-SIPP": "TP-STA*",
    "TPTS-SIPP": "TPTS-STA*",
    "LNS(1s)+PBS-MLA*": "LNS(1s)+PBS-MLA*",
    "LNS(1s)+wPBS-MLA*": "LNS(1s)+wPBS-MLA*",
    "LNS(1s)+PBS-MLSIPP": "LNS(1s)+PBS-MLA*",
    "LNS(1s)+wPBS-MLSIPP": "Hungarian+wPBS-MLA*",
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


rows = list(csv.reader(open(f"{ROOT}/all_results.csv")))
orig = {}
re = {}
for r in rows:
    if len(r) < 10:
        continue
    if r[9] == "original":
        orig[(r[0], r[1], r[2])] = (num(r[3]), num(r[4]), num(r[5]))
    if r[9] == "reimplementation":
        re[(r[0], r[1], r[2])] = (num(r[3]), num(r[4]), num(r[5]))


def c(v):
    return f"{v:.0f}" if v is not None else ""


out = csv.writer(open(f"{ROOT}/comparison_original_vs_reimpl.csv", "w", newline=""))
out.writerow(
    [
        "method",
        "agents",
        "freq",
        "orig_MS",
        "reimpl_MS",
        "orig_SWT",
        "reimpl_SWT",
        "orig_RTs",
        "reimpl_RTs",
    ]
)
for m in ALL:
    og = CMP[m]
    for ag in ["10", "20", "30", "40", "50"]:
        for fr in ["0.2", "0.5", "1", "2", "5", "10", "500"]:
            ov = orig.get((og, ag, fr))
            rv = re.get((m, ag, fr))
            if ov is None and rv is None:
                continue
            oR = ov[2] if ov else None
            if ov and og in PER_STEP and oR and ov[0]:
                oR = oR * ov[0] / 1000.0
            out.writerow(
                [
                    m,
                    ag,
                    fr,
                    c(ov[0] if ov else None),
                    c(rv[0] if rv else None),
                    c(ov[1] if ov else None),
                    c(rv[1] if rv else None),
                    (f"{oR:.1f}" if oR is not None else ""),
                    (f"{rv[2]:.1f}" if rv and rv[2] is not None else ""),
                ]
            )
print("regenerated comparison_original_vs_reimpl.csv")
