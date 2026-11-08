#!/usr/bin/env python3
"""No ROS/devices: execute compact path through existing native real DMPC.

Independent dynamic bicycle plant includes motor/servo lag and optional noise.
This validates one selected path under ASSUMED lead dimensions/constant speed;
it is not a live tracker, a dynamic planner, or authorization to drive.
"""
import argparse,json,math,os,subprocess
from pathlib import Path
import numpy as np
from compact_straight_screen import D,ASSETS,transition,sat_collision
V7=ASSETS.parent

def path_for(spec,assets):
    r=np.asarray(assets['route']);start=r[0];theta=start[2];c,s=math.cos(theta),math.sin(theta)
    length=np.linalg.norm(r[-1,:2]-r[0,:2]);x=np.linspace(0,length,141)
    e,h,z=spec['entry_m'],spec['hold_m'],spec['exit_m'];off=spec['side']*spec['offset_m']
    f,dy,_=transition(x/e);g,ey,_=transition((x-e-h)/z)
    y=off*(f-g);yaw=theta+np.arctan(off*(dy/e-ey/z))
    return np.c_[start[0]+x*c-y*s,start[1]+x*s+y*c,yaw]

def project(path,q):
    edges=np.diff(path[:,:2],axis=0);ds=np.linalg.norm(edges,axis=1);arc=np.r_[0,np.cumsum(ds)]
    u=np.clip(np.sum((q-path[:-1,:2])*edges,axis=1)/(ds**2),0,1)
    nearest=path[:-1,:2]+u[:,None]*edges;dist=np.linalg.norm(nearest-q,axis=1);i=np.argmin(dist)
    return float(arc[i]+u[i]*ds[i]),float(arc[-1]),float(dist[i])

class Native:
    def __init__(self,assets):
        env=os.environ.copy();env['LD_LIBRARY_PATH']=str(V7/'lib')+':'+env.get('LD_LIBRARY_PATH','')
        self.p=subprocess.Popen([str(V7/'bin/core_probe'),str(assets)],env=env,stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
    def ask(self,q):
        self.p.stdin.write(json.dumps(q)+'\n');self.p.stdin.flush()
        for _ in range(30):
            line=self.p.stdout.readline()
            if not line:raise RuntimeError('native process exited')
            try:r=json.loads(line)
            except ValueError:continue
            if not r.get('ok'):raise RuntimeError(r.get('error','native failure'))
            return r
        raise RuntimeError('invalid native response')
    def close(self):
        self.p.stdin.close();self.p.wait(timeout=5)

def run(spec,assets_file,lag=.15,servo_lag=.20,gain=1.,lead_speed=None):
    assets=json.loads(assets_file.read_text());path=path_for(spec,assets);x,y,yaw=path[0];v=vy=rate=servo=command=0.
    first=path[0];ct,st=math.cos(first[2]),math.sin(first[2]);lead_speed=spec['lead_assumed_mps'] if lead_speed is None else lead_speed
    rows=[];failure=None;done=False;max_cte=0.;minimum_map=math.inf;warm_errors=[]
    grid=np.asarray(assets['clearance']).reshape(assets['height'],assets['width']);res=assets['resolution'];ox,oy=assets['origin']
    bx,by=np.meshgrid(np.linspace(-.29,.29,13),np.linspace(-.155,.155,9));bx=bx.ravel();by=by.ravel()
    coverage=math.hypot(.58/12,.31/8)/2
    def map_check(q):
        px=q[0]+math.cos(q[2])*bx-math.sin(q[2])*by;py=q[1]+math.sin(q[2])*bx+math.cos(q[2])*by
        c=np.floor((px-ox)/res).astype(int);r=assets['height']-1-np.floor((py-oy)/res).astype(int)
        valid=(c>=0)&(c<assets['width'])&(r>=0)&(r<assets['height'])
        if not np.all(valid):return -1.
        return float(np.min(grid[r,c])-coverage)
    base=dict(op='joint',type='dynamic_mpc',path=path.tolist(),cap=min(2.,spec['cap_mps']+.15),dt=.05,obstacles=[])
    native=Native(assets_file)
    try:
      for _ in range(4):
        try:native.ask(dict(base,pose=path[0].tolist(),speed=0.,lateral=0.,yaw_rate=0.,remaining=6.,reset=True))
        except RuntimeError as e:warm_errors.append(str(e))
      for tick in range(500):
        progress,total,cte=project(path,np.array([x,y]));max_cte=max(max_cte,cte)
        q=dict(base,reset=tick==0,pose=[x,y+.003*math.sin(tick*1.1),yaw+.006*math.sin(tick*.8)],speed=v,lateral=0.,yaw_rate=v/.25*math.tan(command),remaining=max(0.,total-progress))
        r=native.ask(q);target=r['speed'];command=r['angle'];diag=r['diag']
        if not (0<=target<=1.85001 and abs(command)<=.27001):raise RuntimeError('command_out_of_bounds')
        for j in range(10):
          dt=.005;applied=0. if v<.03 and target<.18 else target
          if target==0. and v>.03:dv=-.60 # coasting, not active speed regulation/braking
          else:dv=float(np.clip((gain*applied-v)/lag,-.8,2.))
          v=max(0.,v+dt*dv);servo+=dt*(command-servo)/servo_lag
          if v<.5:vy=0.;rate=v/.25*math.tan(servo)
          else:
            m,iz,lf,lr,cf,cr=3.74,.04712,.12,.13,90.,96.
            dvy=-(cf+cr)/(m*v)*vy+(-v-(cf*lf-cr*lr)/(m*v))*rate+cf/m*servo
            dr=-(cf*lf-cr*lr)/(iz*v)*vy-(cf*lf*lf+cr*lr*lr)/(iz*v)*rate+lf*cf/iz*servo
            vy+=dt*dvy;rate+=dt*dr
          x+=dt*(v*math.cos(yaw)-vy*math.sin(yaw));y+=dt*(v*math.sin(yaw)+vy*math.cos(yaw));yaw+=dt*rate
          lx=(x-first[0])*ct+(y-first[1])*st;ly=-(x-first[0])*st+(y-first[1])*ct
          lead_x=spec['initial_center_gap_m']+lead_speed*(tick*.05+(j+1)*dt)
          if sat_collision(lx,ly,yaw-first[2],lead_x,0.,
                           lead_l=spec.get('lead_length_m',.58),
                           lead_w=spec.get('lead_width_m',.31),margin=.08):raise RuntimeError('plant_lead_body_clearance')
        clearance=map_check([x,y,yaw]);minimum_map=min(minimum_map,clearance)
        if clearance<.05:raise RuntimeError('plant_map_body_clearance')
        if v>2.1:raise RuntimeError('plant_overspeed')
        # Preserve the actual solver sweep for independent dynamic-body replay.
        rows.append(dict(t=(tick+1)*.05,x=x,y=y,yaw=yaw,speed=v,lateral_speed=vy,actual_yaw_rate=rate,actual_servo=servo,target=target,steering=command,cte=cte,map_body_clearance=clearance,diag=diag))
        if diag.get('terminal_coasting') and v<.03:
          distance=math.hypot(x-path[-1,0],y-path[-1,1]);heading=math.atan2(math.sin(yaw-path[-1,2]),math.cos(yaw-path[-1,2]))
          if distance>.1 or abs(heading)>.2:raise RuntimeError('endpoint_pose_mismatch')
          if lx-lead_x<spec.get('final_center_gap_m',1.2):raise RuntimeError('insufficient_final_ahead_gap')
          done=True;break
    except RuntimeError as e:failure=str(e)
    finally:native.close()
    summary=dict(passed=done and failure is None,failure=failure,complete=done,seconds=len(rows)*.05,
        endpoint_error_m=math.hypot(x-path[-1,0],y-path[-1,1]),max_cte_m=max_cte,
        minimum_map_body_clearance_m=minimum_map if math.isfinite(minimum_map) else None,
        max_speed_mps=max((r['speed'] for r in rows),default=0.),max_compute_ms=max((r['diag'].get('controller_compute_ms',0.) for r in rows),default=0.),
        motor_lag_s=lag,servo_lag_s=servo_lag,motor_gain=gain,lead_assumed_mps=lead_speed,warmup_errors=warm_errors,
        physical_motion_ready=False,scope=__doc__)
    return dict(summary=summary,spec=spec,trace=rows,path=path.tolist())

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--screen',type=Path,default=D/'reports/compact_straight_reserved/report.json');ap.add_argument('--output',type=Path,default=D/'reports/compact_mpc_rollout');args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True);screen=json.loads(args.screen.read_text());spec=screen['selected_nominal_candidate']
    if spec is None:raise RuntimeError('no nominal candidate')
    result=run(spec,ASSETS);(args.output/'nominal.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result['summary'],indent=2))
if __name__=='__main__':main()
