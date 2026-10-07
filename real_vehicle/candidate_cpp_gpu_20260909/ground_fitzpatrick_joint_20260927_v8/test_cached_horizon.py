"""Committed paths and candidate search must cover hazards beyond the old 2 m cutoff."""
import json,subprocess,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
route=np.array(json.loads((ROOT/'runtime_assets.json').read_text())['route'])
proc=subprocess.Popen([str(ROOT/'bin/core_probe'),str(ROOT/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def ask(q):
 proc.stdin.write(json.dumps(q)+'\n');proc.stdin.flush();r=json.loads(proc.stdout.readline());assert r['ok'],r;return r
try:
 lengths=[];reused=0
 for index in range(0,len(route)-70,7):
  r=ask(dict(op='plan',reset=index==0,pose=route[index].tolist(),obstacles=[],horizon=3.2))
  assert r['valid'],r
  points=np.array(r['path']);length=float(np.linalg.norm(np.diff(points[:,:2],axis=0),axis=1).sum());lengths.append(length)
  assert length>=3.19,(index,length)
  reused+=int(r['center_path_reused'])
 assert reused>10
 ask(dict(op='plan',reset=True,pose=route[0].tolist(),obstacles=[],horizon=3.2))
 obstacle=route[80,:2]
 r=ask(dict(op='plan',pose=route[20].tolist(),obstacles=[obstacle.tolist()],horizon=3.2))
 assert not r['center_path_reused'],r
 if r['valid']:
  points=np.array(r['path']);clearance=min(np.linalg.norm(v[:2]+shift*np.array([math.cos(v[2]),math.sin(v[2])])-obstacle) for v in points for shift in (-.08,0,.08));assert clearance>=.15,clearance
 report=dict(passed=True,minimum_nonterminal_path_length_m=min(lengths),center_reuse_count=reused,fresh_tail_obstacle_rechecked=True,obstacle_result=r['reason'])
 (ROOT/'run_logs/cached_horizon_contract.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
finally:proc.stdin.close();proc.wait(timeout=5)
