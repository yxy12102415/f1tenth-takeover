from pathlib import Path
import json,os,subprocess,sys,shutil
R=Path(__file__).resolve().parents[1];out=R/'reports/compact_fix';results=[]
assert os.environ.get('ROS_DOMAIN_ID')=='83' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
for tracker in ['pid','dynamic_mpc']:
 for case,extra in [('static',['--lead-speed','0']),('moving',['--lead-speed','.5']),('clear',['--lead-speed','0','--no-obstacle']),('scan_timeout',['--lead-speed','0','--fault','scan_timeout'])]:
  name=tracker+'_'+case;dest=R/'reports/dynamic_preparation'/name
  old=R/'review/pre_compact_fix'/name
  if dest.exists() and not old.exists():shutil.copytree(dest,old)
  print('CASE '+name,flush=True)
  p=subprocess.run([sys.executable,str(R/'test_generic_ros.py'),'--tracker',tracker,'--lead-width','.30','--report-name','dynamic_preparation/'+name]+extra,cwd=R,timeout=50)
  v=json.loads((dest/'report.json').read_text());item=dict(name=name,passed=v['passed'],failure=v['failure'],route_completed=v['route_completed'],exit=p.returncode);results.append(item);(out/'ros_regression.json').write_text(json.dumps(results,indent=2)+'\n');print(json.dumps(item),flush=True)
print(json.dumps(results,indent=2));sys.exit(0 if all(v['passed'] for v in results) else 1)
