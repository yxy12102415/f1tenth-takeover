#!/usr/bin/env python3
"""No ROS/devices. Nominal compact full-pass geometry/time screen, not commissioning.

Uses actual saved-map clearance, analytic quintic lane changes, acceleration/
curvature constrained forward/backward speed profile, inflated rectangular
lead collision checks at 50Hz or faster, and endpoint zero velocity. Ideal
path execution is assumed; MPC/actuator lag/robust prediction remain required.
"""
import argparse, hashlib, json, math
from pathlib import Path
import numpy as np
D=Path(__file__).resolve().parent
ASSETS=Path('/home/f1tenth/f1tenth_real_migration_ws_20260829/candidate_cpp_gpu_20260909/ground_381_2_0_straight_dmpc_20261107_v7/runtime_assets.json')

def transition(u):
    z=np.clip(u,0.,1.)
    f=10*z**3-15*z**4+6*z**5
    d=30*z**2-60*z**3+30*z**4
    dd=60*z-180*z**2+120*z**3
    return f,d,dd

def sat_collision(x,y,yaw,lead_x,lead_y,ego_l=.58,ego_w=.31,lead_l=.58,lead_w=.31,margin=.18):
    # Half the clearance margin on each rectangle; target size is ASSUMED.
    c=np.cos(yaw);s=np.sin(yaw);dx=lead_x-x;dy=lead_y-y
    a=ego_l/2+margin/2;b=ego_w/2+margin/2
    u=lead_l/2+margin/2;v=lead_w/2+margin/2
    return ((np.abs(dx)<=a*np.abs(c)+b*np.abs(s)+u)&
            (np.abs(dy)<=a*np.abs(s)+b*np.abs(c)+v)&
            (np.abs(dx*c+dy*s)<=a+u*np.abs(c)+v*np.abs(s))&
            (np.abs(-dx*s+dy*c)<=b+u*np.abs(s)+v*np.abs(c)))

def main():
    p=argparse.ArgumentParser();p.add_argument('--assets',type=Path,default=ASSETS)
    p.add_argument('--output',type=Path,default=D/'reports/compact_straight_screen');args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    raw=args.assets.read_bytes();a=json.loads(raw);route=np.asarray(a['route']);origin=route[0,:2];theta=float(route[0,2]);ct,st=math.cos(theta),math.sin(theta)
    axis=np.array([ct,st]);normal=np.array([-st,ct]);length=float(np.linalg.norm(route[-1,:2]-origin))
    if np.max(np.abs((route[:,:2]-origin)@normal))>.002:raise ValueError('straight route required')
    grid=np.asarray(a['clearance']).reshape(a['height'],a['width']);res=a['resolution'];ox,oy=a['origin']
    # Full interior sampling, with grid coverage allowance (not just corners).
    bx,by=np.meshgrid(np.linspace(-.29,.29,17),np.linspace(-.155,.155,11));bx=bx.ravel();by=by.ravel()
    coverage=math.hypot(.58/16,.31/10)/2
    def map_clearance(x,y,yaw):
        wx=origin[0]+x*ct-y*st;wy=origin[1]+x*st+y*ct;h=yaw+theta
        px=wx[:,None]+np.cos(h[:,None])*bx-np.sin(h[:,None])*by
        py=wy[:,None]+np.sin(h[:,None])*bx+np.cos(h[:,None])*by
        col=np.floor((px-ox)/res).astype(int);row=a['height']-1-np.floor((py-oy)/res).astype(int)
        inside=(col>=0)&(col<a['width'])&(row>=0)&(row<a['height'])
        vals=np.zeros_like(px);vals[inside]=grid[row[inside],col[inside]]
        return float(vals.min()-coverage)
    counts=dict(geometry=0,curvature_rejected=0,map_rejected=0,nominal_cases=0,body_rejected=0,rejoin_gap_rejected=0,nominal_pass=0)
    successes=[];best=None
    x=np.linspace(0,length,281)
    for side in [-1,1]:
      for offset in [.45,.5,.55,.6,.65,.7,.75,.8]:
       for entry in [1.6,1.8,2.,2.2,2.4]:
        for hold in [0.,.3,.6,.9,1.2]:
         for exit_len in [1.6,1.8,2.,2.2,2.4]:
          finish=entry+hold+exit_len
          if finish>length-.35:continue
          counts['geometry']+=1
          f,dy,ddy=transition(x/entry);g,ey,eey=transition((x-entry-hold)/exit_len)
          y=side*offset*(f-g);yp=side*offset*(dy/entry-ey/exit_len);ypp=side*offset*(ddy/entry**2-eey/exit_len**2)
          yaw=np.arctan(yp);curv=ypp/(1+yp**2)**1.5
          if max(abs(curv))>math.tan(.269)/.25:counts['curvature_rejected']+=1;continue
          clearance=map_clearance(x,y,yaw)
          if clearance<.05:counts['map_rejected']+=1;continue
          ds=np.hypot(np.diff(x),np.diff(y));arc=np.r_[0,np.cumsum(ds)]
          for cap in [1.,1.25,1.5,1.85]:
           v=np.minimum(cap,np.sqrt(1.2/np.maximum(.01,abs(curv))));v[0]=v[-1]=0.
           for i in range(1,len(v)):v[i]=min(v[i],math.sqrt(v[i-1]**2+2.*1.*ds[i-1]))
           for i in range(len(v)-2,-1,-1):v[i]=min(v[i],math.sqrt(v[i+1]**2+2.*.55*ds[i]))
           t=np.r_[0,np.cumsum(2*ds/(v[:-1]+v[1:]))]
           ticks=np.linspace(0,t[-1],int(math.ceil(t[-1]/.02))+1)
           tx=np.interp(ticks,t,x);ty=np.interp(ticks,t,y);th=np.interp(ticks,t,yaw)
           return_idx=int(np.searchsorted(tx,entry+hold))
           done_idx=int(np.searchsorted(tx,finish))
           for lead_speed in [.1,.15,.3,.5]:
            for gap in [.9,1.2,1.5]:
             counts['nominal_cases']+=1;lead_x=gap+lead_speed*ticks
             if np.any(sat_collision(tx,ty,th,lead_x,np.zeros_like(tx))):counts['body_rejected']+=1;continue
             # Preserve planner's present full rejoin criterion, not silently relax it.
             rejoin_gap=float(tx[done_idx]-lead_x[done_idx])
             if rejoin_gap<1.2:counts['rejoin_gap_rejected']+=1;continue
             counts['nominal_pass']+=1
             row=dict(side=side,offset_m=offset,entry_m=entry,hold_m=hold,exit_m=exit_len,
                 merge_finished_x_m=finish,cap_mps=cap,lead_assumed_mps=lead_speed,initial_center_gap_m=gap,
                 total_time_s=float(t[-1]),map_body_clearance_lower_bound_m=clearance,
                 max_curvature_1pm=float(max(abs(curv))),gap_at_merge_m=rejoin_gap,
                 final_gap_m=float(tx[-1]-lead_x[-1]),nominal_only=True)
             successes.append(row)
             score=(lead_speed==.15 and gap==1.5,clearance,-t[-1])
             if best is None or score>best[0]:best=(score,row,x.copy(),y.copy(),yaw.copy(),v.copy(),t.copy())
    report=dict(scope=__doc__,asset_path=str(args.assets),assets_sha256=hashlib.sha256(raw).hexdigest(),length_m=length,
        ego_and_lead_dimensions_assumed_m=[.58,.31],body_margin_m=.18,acceleration_mps2=1.,deceleration_mps2=.55,
        assumed_constant_lead_speeds_mps=[.1,.15,.3,.5],counts=counts,
        physical_motion_ready=False,mpc_rollout_passed=False,actuator_dynamics_tested=False,
        selected_nominal_candidate=None if best is None else best[1],passing_cases=successes)
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    if best:
        _,_,xx,yy,h,v,t=best
        out=np.c_[origin[0]+xx*ct-yy*st,origin[1]+xx*st+yy*ct,h+theta,v,t]
        np.savetxt(args.output/'candidate_nominal_only.csv',out,delimiter=',',header='x,y,yaw,speed,time',comments='')
    print(json.dumps({k:v for k,v in report.items() if k not in ['passing_cases','scope']},indent=2))
if __name__=='__main__':main()
