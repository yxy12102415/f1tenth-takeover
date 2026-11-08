#!/usr/bin/env python3
"""Copy and verify commissioned 381 assets; audit geometry without ROS/devices."""
import hashlib,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(__file__).resolve().parent
WS=Path('/home/f1tenth/f1tenth_real_migration_ws_20260829')
SOURCE=WS/'candidate_cpp_gpu_20260909/ground_381_2_0_joint_dmpc_20260927_v2/runtime_assets.json'
def main():
 d=json.loads(SOURCE.read_text())
 for p,h in d['sources'].items():
  assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h, 'changed source: '+p
 (D/'assets').mkdir(exist_ok=True);(D/'reports').mkdir(exist_ok=True)
 (D/'assets/381_2_0.json').write_bytes(SOURCE.read_bytes())
 r=np.array(d['route']);s=np.r_[0,np.linalg.norm(np.diff(r[:,:2],axis=0),axis=1).cumsum()]
 grid=np.array(d['clearance']).reshape(d['height'],d['width']);res=d['resolution'];ox,oy=d['origin']
 def clear(x,y):
  col=math.floor((x-ox)/res);row=math.floor((y-oy)/res)
  return float(grid[d['height']-1-row,col]) if 0<=col<d['width'] and 0<=row<d['height'] else 0.
 def feasible(x,y,yaw):
  # Screening only: sampled body perimeter plus physical model center clearance.
  vals=[clear(x,y)-.25]
  for u in np.linspace(-.29,.29,7):
   for v in [-.155,.155]:vals.append(clear(x+u*math.cos(yaw)-v*math.sin(yaw),y+u*math.sin(yaw)+v*math.cos(yaw))-.05)
  for u in [-.29,.29]:
   for v in np.linspace(-.155,.155,5):vals.append(clear(x+u*math.cos(yaw)-v*math.sin(yaw),y+u*math.sin(yaw)+v*math.cos(yaw))-.05)
  return min(vals)>=0
 sides={}
 for side in [1,-1]:
  mask=[feasible(x-side*.82*math.sin(yaw),y+side*.82*math.cos(yaw),yaw) for x,y,yaw in r];segments=[];start=None
  for i,ok in enumerate(mask+[False]):
   if ok and start is None:start=i
   if not ok and start is not None:
    segments.append({'from_s_m':float(s[start]),'to_s_m':float(s[i-1]),'length_m':float(s[i-1]-s[start])});start=None
  sides['left' if side==1 else 'right']=sorted(segments,key=lambda x:x['length_m'],reverse=True)
 # Idealized straight equal-tangent motion, instantaneous speed, no lateral/rejoin/stop cost.
 # This is only a screen, not proof for curved offsets or a newly recorded longer route.
 limits=[]
 for lead in [.7,.8,1.]:
  ego=1.5;gap=2.25;ahead=1.2
  limits.append({'ego_speed_mps':ego,'lead_speed_mps':lead,'initial_gap_m':gap,'required_ahead_gap_m':ahead,'ideal_straight_pass_distance_m':ego*(gap+ahead)/(ego-lead)})
 report={'map':'381_2_0','route_mode':'open_stop_at_end','source_assets':str(SOURCE),'assets_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),'route_length_m':float(s[-1]),'start_pose':r[0].tolist(),'end_pose':r[-1].tolist(),'vehicle_dimensions_assumed_m':[.58,.31],'dimensions_confirmed':False,'offset_m':.82,'offset_corridor_screen':sides,'idealized_distance_screen':limits,'physical_motion_ready':False,'blockers':['Existing open route is only 5.85 m; demonstrate complete pass/rejoin/stopping room or record a longer route.','Simulation node hardcodes remaining=100 m and has no commissioned open-route terminal stop.','Real command adapter, deadman/watchdog/fault validation and Jetson timing acceptance pending.','Opponent dimensions/speed and current localization/start pose not confirmed.'],'scope':'Static sampled geometry screen only; not swept or time-indexed trajectory acceptance.'}
 (D/'reports/route_audit.json').write_text(json.dumps(report,indent=2)+'\n')
 import yaml
 from PIL import Image
 m=yaml.safe_load((WS/'maps/381_2_0.yaml').read_text());im=np.array(Image.open(m['image']))
 fig,ax=plt.subplots(figsize=(10,8));ax.imshow(im,cmap='gray',origin='upper',extent=[ox,ox+d['width']*res,oy,oy+d['height']*res],vmin=0,vmax=255)
 ax.plot(r[:,0],r[:,1],color='#2373b8',lw=2,label='Existing route (5.85 m)');ax.scatter(*r[0,:2],color='green',s=70,label='Start');ax.scatter(*r[-1,:2],color='red',s=70,label='End')
 for k in np.arange(1,s[-1],1):
  i=np.argmin(abs(s-k));ax.text(r[i,0],r[i,1]+.12,str(int(k))+' m',fontsize=9)
 ax.set(xlabel='Map X [m]',ylabel='Map Y [m]',title='381 2.0: Current Route — Overtaking Preparation');ax.legend();ax.set_aspect('equal');fig.tight_layout();fig.savefig(D/'reports/381_route_audit_en.png',dpi=160);plt.close(fig)
 print(json.dumps(report,indent=2))
if __name__=='__main__':main()
