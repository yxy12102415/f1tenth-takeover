"""Sequential localhost ROS tests; no vehicle drivers, no physical topics."""
import json,os,subprocess,sys
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='83' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
r=Path(__file__).resolve().parent
cases=[
 ('pid_static',['--tracker','pid','--lead-speed','0']),
 ('mpc_static',['--tracker','dynamic_mpc','--lead-speed','0']),
 ('pid_wheels',['--tracker','pid','--lead-speed','0','--wheel-pair']),
 ('mpc_wheels',['--tracker','dynamic_mpc','--lead-speed','0','--wheel-pair']),
 ('pid_moving015',['--tracker','pid','--lead-speed','.15']),
 ('mpc_moving015',['--tracker','dynamic_mpc','--lead-speed','.15']),
 ('pid_release',['--tracker','pid','--lead-speed','0','--fault','release']),
 ('mpc_scan_timeout',['--tracker','dynamic_mpc','--lead-speed','0','--fault','scan_timeout']),
 ('mpc_no_obstacle',['--tracker','dynamic_mpc','--lead-speed','0','--no-obstacle']),
]
results=[]
for name,args in cases:
 print('CASE '+name,flush=True)
 p=subprocess.run([sys.executable,str(r/'test_generic_ros.py'),'--lead-width','.30','--report-name',name]+args,cwd=r,timeout=50)
 report=json.loads((r/'reports'/name/'report.json').read_text())
 results.append(dict(name=name,exit_code=p.returncode,**report))
 (r/'reports/suite.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps([dict(name=v['name'],passed=v['passed'],route_completed=v['route_completed'],failure=v['failure']) for v in results],indent=2))
sys.exit(0 if all(v['passed'] for v in results) else 1)
