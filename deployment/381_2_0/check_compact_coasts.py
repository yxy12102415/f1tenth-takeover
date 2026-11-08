#!/usr/bin/env python3
"""Non-actuating full-body emergency-coast screen from every rollout state.

The current actuator stop is zero current plus centered steering, not active
braking. Delayed detection is covered by checking every subsequent driving
state as a possible zero-command initiation. Model evidence only.
"""
import argparse,json,math
from pathlib import Path
import numpy as np
from compact_straight_screen import ASSETS,D,sat_collision
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--rollout',type=Path,default=D/'reports/compact_mpc_rollout/wide_085.json');ap.add_argument('--decel',type=float,default=.4);args=ap.parse_args()
    data=json.loads(args.rollout.read_text());a=json.loads(ASSETS.read_text());grid=np.array(a['clearance']).reshape(a['height'],a['width']);ox,oy=a['origin'];res=a['resolution'];first=data['path'][0];ct,st=math.cos(first[2]),math.sin(first[2])
    bx,by=np.meshgrid(np.linspace(-.29,.29,13),np.linspace(-.155,.155,9));coverage=math.hypot(.58/12,.31/8)/2+.01
    minimum=999.;map_fail=[];body_fail=[];worst=[];cases=0
    for start in data['trace']:
        x,y,yaw,v,vy,rate,servo=[start[k] for k in ['x','y','yaw','speed','lateral_speed','actual_yaw_rate','actual_servo']];samples=[];local_min=999.;conflict=False
        for k in range(1000):
            dt=.005;v=max(0.,v-dt*args.decel);servo+=dt*(0-servo)/.20
            if v<.5:vy=0.;rate=v/.25*math.tan(servo)
            else:
                m,iz,lf,lr,cf,cr=3.74,.04712,.12,.13,90.,96.
                dvy=-(cf+cr)/(m*v)*vy+(-v-(cf*lf-cr*lr)/(m*v))*rate+cf/m*servo
                dr=-(cf*lf-cr*lr)/(iz*v)*vy-(cf*lf*lf+cr*lr*lr)/(iz*v)*rate+lf*cf/iz*servo
                vy+=dt*dvy;rate+=dt*dr
            x+=dt*(v*math.cos(yaw)-vy*math.sin(yaw));y+=dt*(v*math.sin(yaw)+vy*math.cos(yaw));yaw+=dt*rate
            if k%4==0 or v==0:
                t=(k+1)*dt;px=x+math.cos(yaw)*bx-math.sin(yaw)*by;py=y+math.sin(yaw)*bx+math.cos(yaw)*by
                col=np.floor((px-ox)/res).astype(int);row=a['height']-1-np.floor((py-oy)/res).astype(int)
                inside=(col>=0)&(col<a['width'])&(row>=0)&(row<a['height']);clearance=float(np.min(grid[row[inside],col[inside]])-coverage) if np.all(inside) else -1.
                local_min=min(local_min,clearance);lx=(x-first[0])*ct+(y-first[1])*st;ly=-(x-first[0])*st+(y-first[1])*ct
                for speed in [0.,.1,.15,.25]:
                    center=1.6+speed*start['t'];lo=center-.05;hi=center+.05+.25*t
                    conflict=conflict or bool(sat_collision(lx,ly,yaw-first[2],(lo+hi)/2,0,lead_l=hi-lo+.36,lead_w=.36,margin=.08))
                samples.append([t,x,y,yaw,v,clearance])
            if v==0:break
        cases+=1
        if local_min<minimum:minimum=local_min;worst=samples
        if local_min<.05:map_fail.append(start['t'])
        if conflict:body_fail.append(start['t'])
    report=dict(passed=not map_fail and not body_fail,cases=cases,coast_deceleration_mps2=args.decel,minimum_map_clearance_m=minimum,map_failure_start_times=map_fail,lead_body_failure_start_times=body_fail,physical_motion=False,scope=__doc__,rollout=str(args.rollout),worst_map_coast=worst)
    out=D/'reports'/('coasts_'+args.rollout.stem+'.json');out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='worst_map_coast'},indent=2));raise SystemExit(0 if report['passed'] else 1)
if __name__=='__main__':main()
