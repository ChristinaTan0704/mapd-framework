#!/bin/bash
# Wait for the LNS resume run to finish, then: persist LNS cells, run CENTRAL-ECBS-SIPP 20/30/40,
# insert its rows, and regenerate comparison_original_vs_reimpl.csv. Sequential (no contention).
set -u
ROOT="/Users/jiaqit/Desktop/2026 meta docs/paper/MAPD_framework_imp"
cd "$ROOT" || exit 1

# 1) wait for resume
while ! grep -q "RESUME DONE" /tmp/resume.log 2>/dev/null; do sleep 60; done
echo "[final] resume done; persisting LNS cells" >&2
{ echo "method,agents,freq,makespan,swt,runtime_s,status"; awk -F, '$7=="ok"' /tmp/resume_cells.csv; } > /tmp/resume_ok.csv
python3 "$ROOT/skill_tools/persist_results.py" /tmp/resume_ok.csv "$ROOT/all_results.csv" >/dev/null 2>&1

# 2) run CENTRAL-ECBS-SIPP 20/30/40 (sequential, after resume -> clean runtime)
echo "[final] running CENTRAL-ECBS-SIPP 20,30,40" >&2
"$ROOT/skill_tools/run_method.sh" "CENTRAL-ECBS-SIPP" "20,30,40" /tmp/cesipp.csv

# 3) insert CENTRAL-ECBS-SIPP reimpl rows (compare_against = CENTRAL-ECBS) with gaps
python3 - "$ROOT/all_results.csv" /tmp/cesipp.csv <<'PY'
import csv,sys
allp,run=sys.argv[1],sys.argv[2]
def num(x):
    try:return float(x)
    except:return None
rows=list(csv.reader(open(allp)))
orig={(r[1],r[2]):(num(r[3]),num(r[4]),num(r[5])) for r in rows if len(r)>=10 and r[9]=="original" and r[0]=="CENTRAL-ECBS"}
# drop any existing CENTRAL-ECBS-SIPP reimpl rows at 20/30/40 to avoid dups
rows=[r for r in rows if not(len(r)>=10 and r[0]=="CENTRAL-ECBS-SIPP" and r[9]=="reimplementation" and r[1] in("20","30","40"))]
def g(a,b):
    return "-" if (a is None or not b) else f"{(a-b)/b*100:+.1f}%"
add=0
for r in list(csv.reader(open(run)))[1:]:
    if len(r)<7 or r[6]!="ok":continue
    m,ag,fr,ms,swt,rt=r[0],r[1],r[2],num(r[3]),num(r[4]),num(r[5])
    ov=orig.get((ag,fr))
    if ov is None:continue
    rows.append([m,ag,fr,r[3],r[4],r[5],g(ms,ov[0]),g(swt,ov[1]),g(rt,ov[2]),"reimplementation","CENTRAL-ECBS"]);add+=1
open(allp,"w",newline="").write("");import csv as c2
c2.writer(open(allp,"w",newline="")).writerows(rows)
print(f"inserted {add} CENTRAL-ECBS-SIPP rows")
PY

# 4) regenerate comparison CSV
python3 "$ROOT/skill_tools/gen_compare.py" > /dev/null 2>&1 || true
echo "FINALIZE DONE" >&2
