from validate_pid_tuning import ROOT,BASE,rollout
import json
results=[]
cases=[dict(lag=.25,servo_lag=.10,steering_delay=.10,pose_delay=.05),dict(lag=.25,servo_lag=.15,steering_delay=.15,pose_delay=.05),dict(lag=.25,servo_lag=.15,steering_delay=.10,steering_gain=1.15,pose_delay=.05)]
for tuning in [None,[1.4,.1,.9],[2.,.15,.9],[2.,.15,1.2],[1.4,.2,.8],[2.,.2,.7],[2.5,.1,1.1]]:
 for i,case in enumerate(cases):
  r,t=rollout(BASE if tuning is None else ROOT,True,tuning=tuning,**case);r['tuning']=tuning;r['case']=i;results.append(r)
  post=[x['diag']['cross_track_error_m'] for x in t if 3.2<=x['progress']<=5.2 and x['diag']];r['return_peak_to_peak_m']=max(post)-min(post) if post else None
  print(tuning,i,r['passed'],r['failure'],r['return_cross_track_rms_m'],r['return_peak_to_peak_m'],flush=True)
(ROOT/'run_logs/recovery_parameter_comparison.json').write_text(json.dumps(results,indent=2)+'\n')
