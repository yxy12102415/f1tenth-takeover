"""Independent, non-actuating closed-loop plant with motor/servo lag.

Native planning AND complete longitudinal/lateral controller are exercised.
This is model evidence, not a physical clearance or system identification claim.
"""
import argparse, json, math, os, subprocess
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent

class Native:
    def __init__(self):
        env=os.environ.copy();env['LD_LIBRARY_PATH']=str(ROOT/'lib')+':'+env.get('LD_LIBRARY_PATH','')
        self.proc=subprocess.Popen([str(ROOT/'bin/core_probe'),str(ROOT/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,env=env)
    def ask(self,j):
        self.proc.stdin.write(json.dumps(j)+'\n');self.proc.stdin.flush()
        diagnostics=[]
        for _ in range(20):
            line=self.proc.stdout.readline()
            if not line:raise RuntimeError('native_eof: '+str(diagnostics))
            try:r=json.loads(line);break
            except ValueError:diagnostics.append(line.strip())
        else:raise RuntimeError('native_non_json: '+str(diagnostics))
        if not r.get('ok'):raise RuntimeError(r.get('error','unknown'))
        return r
    def close(self):
        self.proc.stdin.close();self.proc.wait(timeout=5)

def rollout(kind,obstacle,lag=.15,servo_lag=.10,noise=True):
    assets=json.loads((ROOT/'runtime_assets.json').read_text());route=np.array(assets['route']);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(route[:,:2],axis=0),axis=1))]
    x,y,yaw=route[0];v=vy=rate=servo=command=0.;index=0;rows=[];failure=None;complete=False
    box=[]
    if obstacle:
        for q in np.linspace(-.0786,.0786,9):
            box.extend([[2.7497+q,-.2355-.0786],[2.7497+q,-.2355+.0786],[2.7497-.0786,-.2355+q],[2.7497+.0786,-.2355+q]])
    native=Native();selected=None
    try:
        for tick in range(600):
            if tick%4==0:
                selected=native.ask(dict(op='plan',reset=tick==0,pose=[x,y,yaw],obstacles=box,horizon=max(0,arc[-1]-arc[index])))
                index=selected['progress']
                if not selected['valid']:raise RuntimeError('plan_'+selected['reason'])
            distance=math.hypot(x-route[-1,0],y-route[-1,1]);heading=math.atan2(math.sin(yaw-route[-1,2]),math.cos(yaw-route[-1,2]))
            if arc[-1]-arc[index]<=.05:
                if distance>.10 or abs(heading)>.20:raise RuntimeError('endpoint_pose_mismatch')
                complete=True;break
            r=native.ask(dict(op='joint',type=kind,reset=tick==0,path=selected['path'],pose=[x,y+(.003*math.sin(tick*1.1) if noise else 0),yaw+(.006*math.sin(tick*.8) if noise else 0)],speed=v,lateral=0,yaw_rate=v/.25*math.tan(command),remaining=float(arc[-1]-arc[index]),dt=.05,obstacles=box))
            target=r['speed'];command=r['angle']
            for j in range(10):
                dt=.005
                # Static motor friction is deliberately absent from the MPC model.
                applied=0. if v<.03 and target<.18 else target
                v=max(0.,v+dt*float(np.clip((applied-v)/lag,-.8,1.)))
                servo+=dt*(command-servo)/servo_lag
                if v<.5:vy=0.;rate=v/.25*math.tan(servo)
                else:
                    m,iz,lf,lr,cf,cr=3.74,.04712,.12,.13,90.,96.
                    dvy=-(cf+cr)/(m*v)*vy+(-v-(cf*lf-cr*lr)/(m*v))*rate+cf/m*servo
                    dr=-(cf*lf-cr*lr)/(iz*v)*vy-(cf*lf*lf+cr*lr*lr)/(iz*v)*rate+lf*cf/iz*servo
                    vy+=dt*dvy;rate+=dt*dr
                x+=dt*(v*math.cos(yaw)-vy*math.sin(yaw));y+=dt*(v*math.sin(yaw)+vy*math.cos(yaw));yaw+=dt*rate
            centers=np.array([x,y])+np.array([-.08,0,.08])[:,None]*np.array([math.cos(yaw),math.sin(yaw)])
            clearance=float(np.min(np.linalg.norm(centers[:,None,:]-np.asarray(box)[None,:,:],axis=2))) if box else None
            if clearance is not None and clearance<.15:raise RuntimeError('plant_obstacle_footprint')
            col=int(math.floor((x-assets['origin'][0])/assets['resolution'])); row=assets['height']-1-int(math.floor((y-assets['origin'][1])/assets['resolution']))
            if not(0<=col<assets['width'] and 0<=row<assets['height']) or assets['clearance'][row*assets['width']+col]<.25:raise RuntimeError('plant_static_clearance')
            diag=r['diag'];diag.pop('predicted_path',None)
            rows.append(dict(t=tick*.05,x=x,y=y,yaw=yaw,speed=v,target=target,steering=command,state=selected['mode'],clearance=clearance,diag=diag))
    except RuntimeError as e:failure=str(e)
    finally:native.close()
    report=dict(controller=kind,box=obstacle,lag=lag,servo_lag=servo_lag,noise=noise,complete=complete,failure=failure,duration_s=len(rows)*.05,endpoint_distance_m=math.hypot(x-route[-1,0],y-route[-1,1]),endpoint_heading_rad=math.atan2(math.sin(yaw-route[-1,2]),math.cos(yaw-route[-1,2])),max_compute_ms=max((r['diag']['controller_compute_ms'] for r in rows),default=0),max_speed=max((r['speed'] for r in rows),default=0),minimum_clearance_m=min((r['clearance'] for r in rows if r['clearance'] is not None),default=None))
    report['passed']=complete and failure is None
    return report,rows

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--tracker',choices=['pid','dynamic_mpc','both'],default='both');parser.add_argument('--quick',action='store_true');args=parser.parse_args()
    reports=[]
    for kind in (['pid','dynamic_mpc'] if args.tracker=='both' else [args.tracker]):
        for obstacle,lag,servo in ([(False,.15,.1)] if args.quick else [(False,.15,.1),(True,.15,.1),(True,.25,.15)]):
            result,trace=rollout(kind,obstacle,lag,servo)
            name=f'joint_rollout_{kind}_{int(obstacle)}_{lag}'
            (ROOT/'run_logs'/f'{name}.json').write_text(json.dumps(dict(summary=result,trace=trace),indent=2)+'\n')
            reports.append(result);print(json.dumps(result),flush=True)
    (ROOT/'run_logs/joint_rollout_summary.json').write_text(json.dumps(reports,indent=2)+'\n')
    if not all(r['passed'] for r in reports):raise SystemExit(1)
