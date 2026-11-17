"""Sequential offline/domain-83 suite. Never passes physical flags."""
import json
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parent
FAULTS=['window','release','guard_kill','window_kill','supervisor_permit_loss',
        'supervisor_main_stall','health_kill','health_stale','health_overcurrent',
        'health_overtemp']
cases=[('zero',['--zero-only','--require-localization','--zero-seconds','5','--duration','110'])]+[(f,['--fault',f,'--duration','90']) for f in FAULTS]
for name,args in cases:
    print('OFFLINE START '+name,flush=True)
    before=set((ROOT/'run_logs').glob('*/result.json'))
    result=subprocess.run(['bash',str(ROOT/'start.sh'),'--offline-self-test']+args,
                          stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=150)
    new=set((ROOT/'run_logs').glob('*/result.json'))-before
    report=json.loads(next(iter(new)).read_text()) if len(new)==1 else {}
    print(json.dumps(dict(test=name,returncode=result.returncode,passed=report.get('passed'),
                          stop_ms=report.get('coast_command_after_fault_ms'),
                          error=report.get('error'),reports=[str(p) for p in new])),flush=True)
    if result.returncode or report.get('passed') is not True:
        print(result.stdout[-5000:],flush=True)
        sys.exit(1)
print('ALL 11 OFFLINE CASES PASSED',flush=True)
