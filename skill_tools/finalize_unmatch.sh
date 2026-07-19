#!/bin/bash
set -u
ROOT="/Users/jiaqit/Desktop/2026 meta docs/paper/MAPD_framework_imp"
cd "$ROOT" || exit 1
while ! grep -q "RERUN UNMATCH DONE" /tmp/rerun_unmatch.log 2>/dev/null; do sleep 60; done
cp all_results.csv all_results.csv.bak_unmatch
python3 - <<'PY'
import csv
ROOT="/Users/jiaqit/Desktop/2026 meta docs/paper/MAPD_framework_imp"
CMP={"LNS(1s)+PBS-MLA*":"LNS(1s)+PBS-MLA*","LNS(1s)+wPBS-MLA*":"LNS(1s)+wPBS-MLA*",
"LNS(1s)+PBS-MLSIPP":"LNS(1s)+PBS-MLA*","LNS(1s)+wPBS-MLSIPP":"Hungarian+wPBS-MLA*","LNS(1s)+PP-SIPP":"LNS(1s)+PBS-MLA*"}
PER_STEP={"LNS(1s)+PBS-MLA*","LNS(1s)+wPBS-MLA*"}
def num(x):
    try:return float(x)
    except:return None
rows=list(csv.reader(open(f"{ROOT}/all_results.csv")))
# 1) update originals from fresh same-machine reference (store per-step ms)
ref={}
for r in list(csv.reader(open("/tmp/unmatch_ref.csv")))[1:]:
    if len(r)>=6 and r[6]=="ok": ref[(r[0],r[1],r[2])]=(num(r[3]),num(r[5]))
for r in rows:
    if len(r)>=10 and r[9]=="original" and (r[0],r[1],r[2]) in ref:
        ms,tot=ref[(r[0],r[1],r[2])]
        if ms and tot is not None: r[5]=f"{tot*1000.0/ms:.4g}"
# rebuild original index for gap calc
orig={}
for r in rows:
    if len(r)>=10 and r[9]=="original":orig[(r[0],r[1],r[2])]=(num(r[3]),num(r[4]),num(r[5]))
# 2) upsert reimpl cells from fresh run
meas={}
for r in list(csv.reader(open("/tmp/unmatch_re.csv")))[1:]:
    if len(r)>=7 and r[6]=="ok": meas[(r[0],r[1],r[2])]=(r[3],r[4],r[5])
idx={(r[0],r[1],r[2]):i for i,r in enumerate(rows) if len(r)>=11 and r[9]=="reimplementation"}
def gap(a,b):
    a=num(a); return "-" if (a is None or not b) else f"{(a-b)/b*100:+.1f}%"
for (m,ag,fr),(ms,swt,rt) in meas.items():
    og=CMP[m];ov=orig.get((og,ag,fr))
    if ov is None: continue
    oR=ov[2]*ov[0]/1000.0 if (og in PER_STEP and ov[2] and ov[0]) else ov[2]
    row=[m,ag,fr,ms,swt,rt,gap(ms,ov[0]),gap(swt,ov[1]),gap(rt,oR),"reimplementation",og]
    if (m,ag,fr) in idx: rows[idx[(m,ag,fr)]]=row
    else: rows.append(row)
csv.writer(open(f"{ROOT}/all_results.csv","w",newline="")).writerows(rows)
print("updated originals + reimpl for unmatched cells")
PY
python3 "$ROOT/skill_tools/gen_compare.py" >/dev/null 2>&1
echo "FINALIZE UNMATCH DONE" >&2
