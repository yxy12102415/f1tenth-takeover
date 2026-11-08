#!/usr/bin/env python3
"""Diagnostic-only run. Never starts a driver, guard, or physical command bridge."""
import argparse,json,os,signal,subprocess,time,fcntl
from pathlib import Path
D=Path(__file__).resolve().parent;R=D.parents[1]
def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--duration',type=float,default=30.);args=parser.parse_args()
 if not 1<=args.duration<=600:parser.error('duration must be 1..600 seconds')
 lock=(D/'reports/shadow.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 log=D/'reports'/('shadow_'+time.strftime('%Y%m%d_%H%M%S'));log.mkdir(exist_ok=False)
 source=R/'.deps/predictive-spliner-upstream/src/sqp_avoidance_node.py';assert source.exists()
 params={
  'assets_path':str(D/'assets/381_2_0.json'),'mode':'takeover','ego_odom_topic':'/overtake_381/ego_odom','target_topic':'/overtake_381/target','target_status_topic':'/overtake_381/tracker_status','drive_topic':'/overtake_381/proposed_drive','cruise_speed':2.0,'measured_yaw_feedback':False,'command_frame':'base_link','adaptive_offset':True,'predictive_speed_profile':True,'fast_pass_geometry':True,'verified_exit_recovery':True,'checked_control_recovery':True,'planning_backend':'upstream_sqp','sqp_worker_script':str(R/'Code/sim_ws/tools/upstream_sqp_worker.py'),'sqp_upstream_source':str(source),'lead_enabled':True,'use_sim_time':False}
 tracker={'measurement_source':'lidar','static_map_assets':params['assets_path'],'scan_topic':'/overtake_381/scan','ego_odom_topic':'/overtake_381/ego_odom','output_topic':'/overtake_381/target','output_pose_topic':'/overtake_381/target_pose','status_topic':'/overtake_381/tracker_status','scan_origin_x':.27,'target_center_offset':.29,'lidar_fov_deg':120.,'tracked_lidar_fov_deg':270.,'cluster_gap':.18,'min_cluster_width':.12,'max_cluster_width':.85,'min_cluster_points':5,'max_cluster_points':140,'association_gate':.55,'reacquisition_gate':.9,'prediction_timeout':1.2,'max_target_speed':2.,'min_forward_x':-.5}
 def par(values):
  out=['--ros-args']
  for k,v in values.items():out+=['-p',k+':='+('true' if v is True else 'false' if v is False else str(v))]
  return out
 commands=[('bridge',['python3',str(D/'shadow_bridge.py')]),('tracker',['python3',str(R/'Code/sim_ws/src/f1tenth_mpc-main/mpc/scripts/opponent_ekf_tracker.py')]+par(tracker)),('controller',[str(D/'install/takeover_sim/lib/takeover_sim/takeover_controller_node')]+par(params)+['-r','__node:=overtake_381_controller','-r','/scan:=/overtake_381/scan']+sum([['-r','/takeover/'+n+':=/overtake_381/'+n] for n in ['status','reference_path','lead_prediction','track_centerline']],[]))]
 (log/'config.json').write_text(json.dumps({'domain':os.environ.get('ROS_DOMAIN_ID'),'controller':params,'tracker':tracker,'actuation':False},indent=2)+'\n')
 children=[];handles=[];ok=True
 try:
  for name,cmd in commands:
   f=(log/(name+'.log')).open('w');handles.append(f);children.append(subprocess.Popen(cmd,stdout=f,stderr=f,start_new_session=True))
  print('SHADOW ONLY: proposals on /overtake_381/proposed_drive. No actuator bridge. Logs: '+str(log),flush=True)
  end=time.monotonic()+args.duration
  while time.monotonic()<end:
   if any(p.poll() is not None for p in children):ok=False;break
   time.sleep(.2)
 except KeyboardInterrupt:pass
 finally:
  for p in children:
   if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
  for p in children:
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
  for f in handles:f.close()
 print('Shadow stopped; process health: '+str(ok),flush=True)
 return 0 if ok else 1
if __name__=='__main__':raise SystemExit(main())
