"""Exploratory PF-derived steering response; not an actuator identification.
Uses recorded poses/commands, centered heading differences, and reports fits
for both drives. PF noise, sideslip and clock receipt delay remain confounded.
"""
import json,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
sources=[('v1','ground_381_2_0_joint_cpp_20260926_v1','20260926_215443-closed-frenet-ground-attempt'),('v2','ground_381_2_0_joint_pid_20260926_v2','20260926_222558-closed-frenet-ground-attempt')]
reports=[]
for name,release,run in sources:
 d=json.loads((ROOT.with_name(release)/'run_logs'/run/'tracking_trace.json').read_text());a=[x for x in d['controller'] if x.get('valid') and x.get('speed_command',0)>0];ct=np.array([x['received_mono'] for x in a]);delta=np.array([x['steering_command'] for x in a]);speed=np.array([x['tracker_diagnostics']['measured_speed_mps'] for x in a]);p=d['poses'];pt=np.array([x['received_mono'] for x in p]);yaw=np.unwrap([x['yaw'] for x in p]);t=np.arange(ct[0]+.5,ct[-1]-.2,.05);v=np.interp(t,ct,speed);keep=v>.8;t=t[keep];v=v[keep];r=(np.interp(t+.10,pt,yaw)-np.interp(t-.10,pt,yaw))/.20
 fits=[]
 for delay in np.arange(0,.401,.025):
  u=np.interp(t-delay,ct,delta);pred=v/.25*np.tan(u);gain=float(np.dot(pred,r)/max(1e-9,np.dot(pred,pred)));error=float(np.sqrt(np.mean((gain*pred-r)**2)));fits.append(dict(delay_s=float(delay),effective_gain=gain,rate_rmse_rad_s=error))
 report=dict(source=str(ROOT.with_name(release)/'run_logs'/run),label=name,best_approximation=min(fits,key=lambda x:x['rate_rmse_rad_s']),samples=len(t),fits=fits,limitation='Exploratory equivalent gain/delay from PF yaw, not independent actuator feedback or model identification.');reports.append(report)
 print(name,report['best_approximation'])
(ROOT/'run_logs/recorded_steering_response.json').write_text(json.dumps(reports,indent=2)+'\n')
