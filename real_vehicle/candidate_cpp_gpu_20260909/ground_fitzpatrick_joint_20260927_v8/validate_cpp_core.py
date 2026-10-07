"""Non-actuating Python/C++ equivalence against frozen v3 algorithms."""
import json,subprocess,sys,time
from pathlib import Path
import numpy as np
from return_lattice_planner import ClosedLoopLatticePlanner
from obstacle_tracker import cluster_scan_points,PersistentObstacleTracker,confirmed_points
from local_path_tracker import LocalPathTracker
ROOT=Path(__file__).resolve().parent;WS=ROOT.parents[1]
process=subprocess.Popen([str(ROOT/'bin/core_probe'),str(ROOT/'runtime_assets.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
def ask(j):
 process.stdin.write(json.dumps(j,allow_nan=False)+'\n');process.stdin.flush();line=process.stdout.readline();assert line,'native probe died';return json.loads(line)
def planner():return ClosedLoopLatticePlanner(WS/'routes/manual_route_20260923_172359_controller_ready.csv',WS/'maps/381_2_0.yaml',static_echo_tolerance_m=.25,offsets=(-.4,-.3,-.2,-.15,-.1,0,.1,.15,.2,.3,.4),minimum_initial_avoidance_offset_m=.15)
def box(x,y):
 t=np.linspace(-.08,.08,12);return np.vstack([np.c_[x+t,np.full(12,y-.08)],np.c_[x+t,np.full(12,y+.08)],np.c_[np.full(12,x-.08),y+t],np.c_[np.full(12,x+.08),y+t]])
rng=np.random.default_rng(925);times=[];cases=0;max_path_error=0
try:
 for sequence in range(12):
  py=planner()
  for tick in range(10):
   index=min(117,tick*11+sequence%3);x,y,yaw=py.route[index];x+=float(rng.normal(0,.02));y+=float(rng.normal(0,.02));yaw+=float(rng.normal(0,.03))
   obs=box(2.75+sequence*.005,-.18) if sequence%3 else np.empty((0,2));c,cs=py.plan(x,y,obs,heading=yaw)
   n=ask(dict(pose=[x,y,yaw],obstacles=obs.tolist(),reset=tick==0,candidates=True));assert n['ok'],n
   assert c.valid==n['valid'],(sequence,tick,c.reason,n['reason']);assert py.mode==n['mode'],(sequence,tick,py.mode,n['mode']);assert py.center_path_reused==n['center_path_reused'];assert len(cs)==len(n['candidates'])
   for pc,nc in zip(cs,n['candidates']):
    assert pc.valid==nc['valid'],(sequence,tick,pc.reason,nc['reason'])
    assert pc.reason==nc['reason'],(pc.reason,nc['reason'])
    np.testing.assert_allclose([pc.score,pc.maximum_curvature_1pm,pc.lateral_jerk_cost],[nc['score'],nc['maximum_curvature_1pm'],nc['lateral_jerk_cost']],atol=1e-8,rtol=1e-8)
   if c.valid:
    np.testing.assert_allclose(c.path,n['path'],atol=1e-9,rtol=1e-9);max_path_error=max(max_path_error,float(np.max(abs(c.path-np.asarray(n['path'])))))
    np.testing.assert_allclose([c.offset_m,c.transition_m,c.return_m],[n['offset_m'],n['transition_m'],n['return_m']],atol=1e-9)
   times.append(n['compute_ms']);cases+=1
 print('Planner equivalence',cases,'max path difference',max_path_error,flush=True)
 errors=[];rejects=0;tracker=LocalPathTracker();route=planner().route
 for i in range(100):
  idx=i%90;path=route[idx:min(idx+35,len(route))];pose=path[0].copy();pose[1]+=.04*np.sin(i);pose[2]+=.06*np.sin(i*.4);speed=.5+i%10*.08;rate=.2*np.sin(i*.3)
  if i%10==0:tracker.reset()
  n=ask(dict(op='track',type='dynamic_mpc',reset=i%10==0,path=path.tolist(),pose=pose.tolist(),dt=.05,speed=speed,yaw_rate=rate))
  try:value=tracker.compute(path,*pose,.05,speed_mps=speed,yaw_rate_rad_s=rate)
  except ValueError as e:assert not n['ok'],n;rejects+=1;continue
  assert n['ok'],n;errors.append(abs(value-n['angle']));assert errors[-1]<1e-9,(i,value,n)
 print('Dynamic MPC equivalence',len(errors),'rejections',rejects,'max angle error',max(errors),flush=True)
 pytrack=PersistentObstacleTracker(confirmed_hold_s=.30)
 for i in range(20):
  points=box(2.7+i*.002,-.2)[:12] if i%5<3 else np.empty((0,2));beams=np.arange(len(points));stamp=10+i*.2
  tracks=pytrack.update(cluster_scan_points(points,beams),stamp);n=ask(dict(op='perception',points=points.tolist(),beams=beams.tolist(),stamp=stamp,reset=i==0));assert n['ok'];np.testing.assert_allclose(confirmed_points(tracks).reshape(-1,2),np.asarray(n['points']).reshape(-1,2),atol=1e-12)
 import importlib.util,math,ast
 from types import SimpleNamespace
 def load(name,file):
  spec=importlib.util.spec_from_file_location(name,file);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
 pid=load('pid_oracle',ROOT.parent/'ground_381_2_0_frenet_pid_20260923_v1/local_path_tracker.py').LocalPathTracker()
 # Extract the unchanged PP method without importing its old ROS node.
 source=(ROOT.parent/'ground_381_2_0_frenet_pp_20260923_v1/avoidance_controller.py').read_text()
 tree=ast.parse(source);method=next(n for cls in tree.body if isinstance(cls,ast.ClassDef) and cls.name=='AvoidanceController' for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='control')
 pp_module=ast.Module(body=[method],type_ignores=[])
 from avoidance_controller import endpoint_speed_limit
 scope=dict(np=np,math=math,endpoint_speed_limit=endpoint_speed_limit,MAX_STEERING=.269,MAX_STEERING_RATE=1.,ENDPOINT_STOP_REMAINING_M=.05,SPEED_PROFILE_LOOKAHEAD_M=.20,MAX_SPEED=1.5,MIN_MOVING_SPEED=.6,BLOCKAGE_CONFIRM_SPEED=.6)
 exec(compile(pp_module,'original_pp_control','exec'),scope)
 from avoidance_controller import LapProgress,load_speed_profile
 from threading import RLock
 profile_distance,profile_speed=load_speed_profile(WS/'routes/manual_route_20260923_172359_controller_ready_speed_profile_v2.csv',route)
 pp=SimpleNamespace(last_angle=0.,last_pose_tick=None,lattice_state="FOLLOWING",lap=LapProgress(planner().segment_m,False),input_lock=RLock(),profile_distance=profile_distance,profile_speed=profile_speed)
 basic={}
 for typ in ('pid','pp'):
  err=[];rejected=0
  for i in range(100):
   path=route[i%90:min(i%90+35,len(route))];pose=path[0].copy();pose[1]+=.04*math.sin(i);pose[2]+=.06*math.sin(.4*i)
   reset=i%10==0
   if reset:pid.reset();pp.last_angle=0.;pp.last_pose_tick=None
   n=ask(dict(op='track',type=typ,reset=reset,path=path.tolist(),pose=pose.tolist(),dt=.05,speed=.8))
   try:
    if typ=='pid':value=pid.compute(path,*pose,.05)
    else:value=scope['control'](pp,path,*pose,10.+i*.05)[1]
   except ValueError:assert not n['ok'];rejected+=1;continue
   assert n['ok'],n;err.append(abs(value-n['angle']));assert err[-1]<1e-9,(typ,i,value,n)
  basic[typ]=dict(cases=len(err),rejected=rejected,max_angle_error=max(err));print(typ,basic[typ],flush=True)
 summary=dict(passed=True,basic_trackers=basic,planner_cases=cases,planner_max_path_error=max_path_error,native_plan_ms=dict(median=float(np.median(times)),p95=float(np.percentile(times,95)),maximum=max(times)),dynamic_cases=len(errors),dynamic_rejections=rejects,dynamic_max_angle_error=max(errors),perception_steps=20)
 (ROOT/'run_logs/cpp_equivalence.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary),flush=True)
finally:process.stdin.close();process.wait(timeout=5)
