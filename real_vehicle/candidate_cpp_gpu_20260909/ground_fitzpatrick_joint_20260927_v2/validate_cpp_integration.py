"""Run the tuned PID public offline entrypoint and verify the selected binaries.
No physical flags or hardware drivers are allowed in these fixtures.
"""
import json,os,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parent
WS=ROOT.parents[1]
reports=[]
for kind,stem in [('pid','381_2_0_frenet_pid')]:
 before=set((ROOT/'run_logs').glob('*/result.json'))
 print('Public offline entry:',kind,flush=True)
 with (ROOT/'run_logs'/('cpp_integration_'+kind+'.log')).open('w') as log:
  result=subprocess.run([str(WS/(stem+'_offline.sh'))],stdout=log,stderr=subprocess.STDOUT,timeout=120)
 paths=set((ROOT/'run_logs').glob('*/result.json'))-before
 assert len(paths)==1,(kind,paths)
 path=next(iter(paths));r=json.loads(path.read_text())
 expected='closed_loop_frenet_cpp_'+kind
 assert result.returncode==0 and r['passed'],(kind,r.get('error'),str(path))
 assert r['controller']==expected,(kind,'wrong controller',r['controller'])
 assert r['offline'] is True and r['physical_motion_authorized_by_cli'] is False
 trace=json.loads((path.parent/'tracking_trace.json').read_text())['controller']
 assert trace and all(t['controller']==expected for t in trace),kind
 reports.append(dict(controller=kind,passed=True,result=str(path),fingerprint=r['code_fingerprint'],first_actual_rpm_command=r['first_actual_rpm_command'],stop_latency_ms=r['fixture_stop_after_observation_ms']))
 print(json.dumps(reports[-1]),flush=True)
(ROOT/'run_logs/cpp_integration_summary.json').write_text(json.dumps(reports,indent=2)+'\n')
