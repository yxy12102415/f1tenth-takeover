"""Physical-input regression: retain rejection and choose earlier buffer."""
import json,math
import numpy as np
from replay_recorded_pid import ROOT,run
from replay_replanned_pid import run as replan
r,trace=run();assert r['first_failure']=='predicted_obstacle_footprint' and r['failure_index']==71
assert r['max_pose_match_error']<1e-8 and r['max_steering_difference_rad']<.001
failure=trace[-1];d=failure['output']['rejected_diagnostics'];assert d['proposed_speed_mps']>1 and len(d['predicted_path'])==11
p=np.asarray(d['predicted_path']);obs=np.asarray(failure['input']['obstacles']);centers=p[:,:2,None]+np.stack([np.cos(p[:,2]),np.sin(p[:,2])],axis=1)[:,:,None]*np.array([-.08,0,.08])[None,None,:]
minimum=float(np.linalg.norm(centers.transpose(0,2,1)[:,:,None,:]-obs[None,None,:,:],axis=3).min());assert minimum<.15
changed=replan(ROOT);early=next(p for p in changed['plans'] if p['id']==146)
assert early['recorded_offset']==-.15 and early['selected']['offset_m']==-.2
assert early['selected']['minimum_footprint_point_distance_m']>=.18
assert changed['summary']['failure']=='predicted_obstacle_footprint','Unsafe recorded poses must still be rejected'
report=dict(passed=True,reproduced_first_fault_index=r['failure_index'],max_steering_replay_error_rad=r['max_steering_difference_rad'],pose_match_error=r['max_pose_match_error'],predicted_footprint_point_min_m=minimum,early_plan_id=146,old_offset_m=-.15,new_offset_m=-.2,new_preferred_plan_footprint_point_min_m=early['selected']['minimum_footprint_point_distance_m'],late_unsafe_recorded_pose_still_rejected=True,limitations=['Failure sample speed reconstructed from adjacent wheel telemetry/valid diagnostics; dt from publication times.','Replanned recorded-input playback cannot represent a different physical vehicle trajectory.'])
(ROOT/'run_logs/pid_obstacle_regression.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
