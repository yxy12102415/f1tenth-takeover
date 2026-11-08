from validate_pid_tuning import ROOT,rollout
import json
from pathlib import Path
results=[]
for tuning in [[.9,.12,0.],[1.,.08,.06],[1.,.04,.06],[1.1,.08,.06]]:
    for lag,servo in [(.15,.1),(.25,.15),(.35,.2)]:
        report,trace=rollout(ROOT,True,lag,servo,tuning=tuning)
        report['tuning']=tuning;results.append(report)
        print(json.dumps(report),flush=True)
(ROOT/'run_logs/pid_parameter_comparison.json').write_text(json.dumps(results,indent=2)+'\n')
