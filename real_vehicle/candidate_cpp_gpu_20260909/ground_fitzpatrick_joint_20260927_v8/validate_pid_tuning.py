"""Compare native releases with the same route, box and lagged plant.
Includes the coast after route_complete, which the earlier rollout omitted.
No ROS or hardware. This is a model sensitivity check, not a measured rerun.
"""
import argparse, json, math, subprocess
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
BASE=ROOT.with_name('ground_381_2_0_joint_cpp_20260926_v1')
class Native:
    def __init__(self,root):
        self.proc=subprocess.Popen([str(root/'bin/core_probe'),str(root/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
    def ask(self,j):
        self.proc.stdin.write(json.dumps(j)+'\n');self.proc.stdin.flush()
        for _ in range(20):
            line=self.proc.stdout.readline()
            if not line:raise RuntimeError('native_eof')
            try:r=json.loads(line);break
            except ValueError:continue
        else:raise RuntimeError('native_non_json')
        if not r.get('ok'):raise RuntimeError(r.get('error','native_error'))
        return r
    def close(self):
        self.proc.stdin.close();self.proc.wait(timeout=5)

def rollout(root,obstacle,lag=.25,servo_lag=.15,coast=.6,offset=0.,heading_offset=0.,tuning=None,steering_delay=0.,steering_gain=1.,pose_delay=0.,prediction=None,box_center=(2.7497,-.2355),box_half=.0786,scan_noise=0.,box_points=None,kind="dynamic_mpc"):
    assets=json.loads((root/'runtime_assets.json').read_text());route=np.array(assets['route']);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(route[:,:2],axis=0),axis=1))]
    x,y,yaw=route[0];y+=offset;yaw+=heading_offset
    v=vy=rate=servo=0.;index=0;rows=[];failure=None;completed=None;stop_ticks=0;minimum=100.;coasting=False;quiet_ticks=0;cut=None;floor_seen=False
    box=[]
    if obstacle:
        # Consistent fixture near the box cluster recorded in the PID drive.
        cx,cy=box_center
        for q in np.linspace(-box_half,box_half,9):
            box.extend([[cx+q,cy-box_half],[cx+q,cy+box_half],[cx-box_half,cy+q],[cx+box_half,cy+q]])
    if box_points is not None:box=box_points
    native=Native(root);selected=None;commands=[];pose_history=[]
    try:
        for tick in range(1800):
            pose_history.append([x,y,yaw])
            estimate=pose_history[max(0,len(pose_history)-1-int(round(pose_delay/.05)))]
            ex,ey,eh=estimate
            observed_box=[[px+scan_noise*math.sin(tick*.71+j*.37),py+scan_noise*math.cos(tick*.53+j*.43)] for j,(px,py) in enumerate(box)]
            if tick%4==0 and completed is None and not coasting:
                selected=native.ask(dict(op='plan',reset=tick==0,pose=estimate,obstacles=observed_box,horizon=max(0,arc[-1]-arc[index])))
                index=selected['progress']
                if arc[-1]-arc[index]<=.05 and root!=ROOT:
                    distance=math.hypot(ex-route[-1,0],ey-route[-1,1]);heading=math.atan2(math.sin(eh-route[-1,2]),math.cos(eh-route[-1,2]))
                    if distance>.10 or abs(heading)>.20:raise RuntimeError('endpoint_pose_mismatch')
                    completed=dict(t=tick*.05,speed=v,distance=distance,heading=heading)
                elif not selected['valid']:raise RuntimeError('plan_'+selected['reason'])
            if coasting:
                quiet_ticks=quiet_ticks+1 if v<=.03 else 0
                if quiet_ticks>=5 and completed is None:
                    distance=math.hypot(ex-route[-1,0],ey-route[-1,1]);heading=math.atan2(math.sin(eh-route[-1,2]),math.cos(eh-route[-1,2]))
                    if distance>.10 or abs(heading)>.20:raise RuntimeError('terminal_stop_outside_endpoint')
                    completed=dict(t=tick*.05,speed=v,distance=distance,heading=heading)
            if completed is not None or coasting:
                target=command=0.;diag={};stop_ticks+=1
            else:
                r=native.ask(dict(op='joint',type=kind,reset=tick==0,path=selected['path'],pose=[ex,ey+.003*math.sin(tick*1.1),math.atan2(math.sin(eh+.006*math.sin(tick*.8)),math.cos(eh+.006*math.sin(tick*.8)))],speed=v,lateral=0,yaw_rate=v/.25*math.tan(servo),remaining=float(arc[-1]-arc[index]),dt=.05,obstacles=observed_box,pid_tuning=tuning,**({"pid_prediction_s":prediction} if prediction is not None else {})))
                target=r['speed'];command=r['angle'];diag=r['diag']
                if root==ROOT:
                    if target>=.5:floor_seen=True
                    assert not (floor_seen and 0<target<.5-1e-5),('sub_floor_drive',target)
                    if diag.get('terminal_coasting'):
                        coasting=True;cut=dict(t=tick*.05,x=x,y=y,speed=v,remaining=diag['control_remaining_m'])
            commands.append(command)
            for j in range(10):
                dt=.005
                delayed_index=int(math.floor((tick*.05+j*dt-steering_delay+1e-9)/.05))
                delayed_command=commands[min(tick,delayed_index)] if delayed_index>=0 else 0.
                applied=0. if v<.03 and target<.18 else target
                if completed is not None or coasting:v=max(0.,v-coast*dt)
                else:v=max(0.,v+dt*float(np.clip((applied-v)/lag,-.8,1.)))
                servo+=dt*(steering_gain*delayed_command-servo)/servo_lag
                if v<.5:vy=0.;rate=v/.25*math.tan(servo)
                else:
                    m,iz,lf,lr,cf,cr=3.74,.04712,.12,.13,90.,96.
                    dvy=-(cf+cr)/(m*v)*vy+(-v-(cf*lf-cr*lr)/(m*v))*rate+cf/m*servo
                    dr=-(cf*lf-cr*lr)/(iz*v)*vy-(cf*lf*lf+cr*lr*lr)/(iz*v)*rate+lf*cf/iz*servo
                    vy+=dt*dvy;rate+=dt*dr
                x+=dt*(v*math.cos(yaw)-vy*math.sin(yaw));y+=dt*(v*math.sin(yaw)+vy*math.cos(yaw));yaw+=dt*rate
                centers=np.array([x,y])+np.array([-.08,0,.08])[:,None]*np.array([math.cos(yaw),math.sin(yaw)])
                clearance=float(np.min(np.linalg.norm(centers[:,None,:]-np.asarray(box)[None,:,:],axis=2))) if box else 100.
                minimum=min(minimum,clearance)
                if clearance<.15:raise RuntimeError('plant_obstacle_footprint')
                col=int(math.floor((x-assets['origin'][0])/assets['resolution']));row=assets['height']-1-int(math.floor((y-assets['origin'][1])/assets['resolution']))
                if not(0<=col<assets['width'] and 0<=row<assets['height']) or assets['clearance'][row*assets['width']+col]<.25:raise RuntimeError('plant_static_clearance')
            rows.append(dict(t=tick*.05,x=x,y=y,yaw=yaw,speed=v,command=target,steering=command,state=selected['mode'],progress=float(arc[index]),diag=diag))
            if completed and stop_ticks>=30:break
    except RuntimeError as e:failure=str(e)
    finally:native.close()
    distance=math.hypot(x-route[-1,0],y-route[-1,1]);heading=math.atan2(math.sin(yaw-route[-1,2]),math.cos(yaw-route[-1,2]))
    post=[r['diag']['cross_track_error_m'] for r in rows if 3.2<=r['progress']<=5.2 and r['diag']]
    report=dict(release=root.name,box=obstacle,box_center=box_center,box_half=box_half,scan_noise_m=scan_noise,lag=lag,servo_lag=servo_lag,coast_deceleration=coast,steering_delay_s=steering_delay,steering_gain=steering_gain,pose_delay_s=pose_delay,initial_offset_m=offset,initial_heading_rad=heading_offset,complete=completed, coast_cut=cut,failure=failure,final_distance_m=distance,final_heading_rad=heading,minimum_obstacle_point_clearance_m=minimum if box else None,return_cross_track_peak_m=max(map(abs,post),default=0),return_cross_track_rms_m=float(np.sqrt(np.mean(np.square(post)))) if post else None,max_compute_ms=max((r['diag'].get('controller_compute_ms',0) for r in rows),default=0))
    report['passed']=completed is not None and failure is None and distance<=.1 and abs(heading)<=.2
    return report,rows

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--baseline',action='store_true');parser.add_argument('--quick',action='store_true');args=parser.parse_args()
    root=BASE if args.baseline else ROOT;reports=[]
    cases=[(False,.15,.10,.6,0.,0.),(True,.15,.10,.6,0.,0.),(True,.25,.15,.6,0.,0.),(True,.35,.20,.4,0.,0.),(True,.25,.15,.6,.02,.03),(True,.25,.15,.6,-.02,-.03)]
    for i,case in enumerate(cases[:1] if args.quick else cases):
        result,trace=rollout(root,*case);reports.append(result)
        (ROOT/'run_logs'/f'pid_tuning_{"baseline" if args.baseline else "candidate"}_{i}.json').write_text(json.dumps(dict(summary=result,trace=trace),indent=2)+'\n');print(json.dumps(result),flush=True)
    (ROOT/'run_logs'/f'pid_tuning_{"baseline" if args.baseline else "candidate"}_summary.json').write_text(json.dumps(reports,indent=2)+'\n')
    if not args.baseline and not all(r['passed'] for r in reports):raise SystemExit(1)
