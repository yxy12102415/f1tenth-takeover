from pathlib import Path
import json,subprocess,time,statistics,math
R=Path(__file__).resolve().parents[1];baseline=json.loads((R/'review/guard_profile_baseline.json').read_text());assets=json.loads((R/'runtime_assets.json').read_text())
class Probe:
 def __init__(self,path):self.p=subprocess.Popen([str(R/path),str(R/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
 def ask(self,req):
  t=time.perf_counter();self.p.stdin.write(json.dumps(req)+'\n');self.p.stdin.flush();out=json.loads(self.p.stdout.readline());return out,1000*(time.perf_counter()-t)
 def close(self):self.p.stdin.close();self.p.wait(timeout=5)
old=Probe('review/pre_control_perf_opt/bin/generic_probe');new=Probe('bin/generic_probe');cases=[x['request'] for x in baseline];timing=[]
for index in range(0,len(assets['route']),8):
 for steer in [-.2,0.,.2]:cases.append(dict(stamp=100,poses=[assets['route'][index]],state_coast_cap=True,servo=steer,upper=2.))
ox,oy=assets['origin'];res=assets['resolution'];w,h=assets['width'],assets['height']
for p in [[ox+.01,oy+.01,0],[ox+w*res-.01,oy+h*res-.01,math.pi],[ox-.01,oy,0]]:cases.append(dict(stamp=100,poses=[p],state_coast_cap=True,servo=0,upper=2.))
try:
 for req in cases:
  a,_=old.ask(req);b,_=new.ask(req);assert a==b,(req,a,b)
 for row in baseline:
  at=[];bt=[]
  for repeat in range(5):
   if repeat%2:a,ta=old.ask(row['request']);b,tb=new.ask(row['request'])
   else:b,tb=new.ask(row['request']);a,ta=old.ask(row['request'])
   assert a==b;at.append(ta);bt.append(tb)
  timing.append(dict(old_ms=statistics.median(at),new_ms=statistics.median(bt),response=a))
finally:old.close();new.close()
a=statistics.median(x['old_ms'] for x in timing);b=statistics.median(x['new_ms'] for x in timing)
out=dict(passed=True,physical_motion=False,exact_cap_parity_cases=len(cases),map_boundary_cases=3,old_median_ms=a,new_median_ms=b,speedup=a/max(b,1e-9),timings=timing,scope='Exact speed-cap parity and paired captured-state microbenchmark; conservative EDT certificate then unchanged exact body checks; final emergency guard unchanged.')
(R/'review/coast_preview_optimization_regression.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k!='timings'},indent=2))
