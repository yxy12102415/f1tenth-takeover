"""Final v2/v3 comparison with motor/servo lag, pure delays and gain mismatch.
Runs native C++ planner and tracker; no ROS/hardware. Stopping coast is included.
"""
import json,argparse
from validate_pid_tuning import ROOT,BASE,rollout
cases=[dict(obstacle=False,lag=.15,servo_lag=.10),dict(obstacle=True,lag=.15,servo_lag=.10),dict(obstacle=True,lag=.25,servo_lag=.15),dict(obstacle=True,lag=.35,servo_lag=.20,coast=.4),dict(obstacle=True,offset=.02,heading_offset=.03),dict(obstacle=True,offset=-.02,heading_offset=-.03),dict(obstacle=True,servo_lag=.10,steering_delay=.10,pose_delay=.05),dict(obstacle=True,servo_lag=.15,steering_delay=.15,pose_delay=.05),dict(obstacle=True,steering_delay=.10,steering_gain=1.15,pose_delay=.05),dict(obstacle=True,steering_delay=.10,steering_gain=.85,pose_delay=.05)]
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--baseline',action='store_true');args=p.parse_args();root=BASE if args.baseline else ROOT;reports=[]
 for i,case in enumerate(cases):
  r,trace=rollout(root,**case);r['case']=i
  post=[x['diag']['cross_track_error_m'] for x in trace if 3.2<=x['progress']<=5.2 and x['diag']]
  r['return_peak_to_peak_m']=max(post)-min(post) if post else None
  reports.append(r)
  (ROOT/'run_logs'/f'recovery_{"baseline" if args.baseline else "candidate"}_{i}.json').write_text(json.dumps(dict(summary=r,trace=trace),indent=2)+'\n')
  print(json.dumps(r),flush=True)
 (ROOT/'run_logs'/f'recovery_{"baseline" if args.baseline else "candidate"}_summary.json').write_text(json.dumps(reports,indent=2)+'\n')
 if not args.baseline and not all(r['passed'] for r in reports):raise SystemExit(1)
