"""Recorded physical PID trajectory; no extrapolated or synthetic poses."""
import json, math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parent
SOURCE=ROOT.with_name('ground_381_2_0_joint_cpp_20260926_v1')/'run_logs/20260926_215443-closed-frenet-ground-attempt'
r=json.loads((SOURCE/'result.json').read_text());data=json.loads((SOURCE/'tracking_trace.json').read_text());a=[x for x in data['controller'] if x.get('valid') and x.get('speed_command',0)>0];t0=a[0]['received_mono'];end=next(x for x in data['controller'] if x['received_mono']>t0 and x.get('valid') and x.get('complete'));poses=[x for x in data['poses'] if t0<=x['received_mono']<=end['received_mono']+1.5]
post=[x for x in a if 3.2<=x['lap_progress_m']<=5.2];cross=[x['tracker_diagnostics']['cross_track_error_m'] for x in post]
report=dict(source=str(SOURCE),post_return_cross_min_m=min(cross),post_return_cross_max_m=max(cross),post_return_cross_peak_to_peak_m=max(cross)-min(cross),post_return_cross_rms_m=float(np.sqrt(np.mean(np.square(cross)))),last_active_measured_speed_mps=a[-1]['tracker_diagnostics']['measured_speed_mps'],last_active_speed_target_mps=a[-1]['tracker_diagnostics']['speed_target_mps'],last_active_speed_command_mps=a[-1]['speed_command'],last_active_heading_error_rad=a[-1]['tracker_diagnostics']['heading_error_rad'],last_active_bearing_error_rad=a[-1]['tracker_diagnostics']['tracking_error_rad'],last_active_steering_rad=a[-1]['steering_command'],completion_distance_m=end['endpoint_distance_m'])
(ROOT/'run_logs/physical_pid_diagnosis.json').write_text(json.dumps(report,indent=2)+'\n')
assets=json.loads((ROOT/'runtime_assets.json').read_text());route=np.asarray(assets['route']);xy=np.array([[p['x'],p['y']] for p in poses]);fig,ax=plt.subplots(figsize=(10,4.5));ax.plot(route[:,0],route[:,1],'--',color='royalblue',label='Frozen reference route');ax.plot(xy[:,0],xy[:,1],color='darkorange',label='Recorded PF trajectory (v1)');ax.scatter(*route[0,:2],s=55,color='green',label='Start');ax.scatter(*route[-1,:2],s=65,marker='x',color='red',label='Endpoint')
mid=min(a,key=lambda x:abs(x['lap_progress_m']-2.5));obs=np.array(mid['obstacle_points']);near=obs[np.linalg.norm(obs-np.array([2.77,-.22]),axis=1)<.35];ax.scatter(near[:,0],near[:,1],s=8,color='black',label='Recorded box scan points');ax.add_patch(plt.Circle(route[-1,:2],.1,fill=False,color='red',alpha=.6));ax.set_aspect('equal');ax.grid(alpha=.25);ax.set_xlabel('Map x (m)');ax.set_ylabel('Map y (m)');ax.legend(loc='upper left',fontsize=8);ax.set_title('Physical PID run 2026-09-26 21:54:43');fig.tight_layout();fig.savefig(ROOT/'run_logs/physical_pid_trajectory.png',dpi=180);plt.close(fig)
print(json.dumps(report,indent=2))
