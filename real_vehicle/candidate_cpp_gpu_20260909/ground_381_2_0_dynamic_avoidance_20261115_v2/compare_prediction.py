from validate_pid_tuning import ROOT,rollout
import json
results=[]
for prediction,tuning in [(.15,[1.4,.1,.9]),(.20,[1.4,.05,.9]),(.20,[1.2,.05,.8]),(.15,[1.2,.1,.7]),(.20,[1.8,.1,.9])]:
 for i,c in enumerate([dict(lag=.25,servo_lag=.15),dict(lag=.25,servo_lag=.1,steering_delay=.1,pose_delay=.05),dict(lag=.25,servo_lag=.15,steering_delay=.15,pose_delay=.05),dict(lag=.25,servo_lag=.15,steering_delay=.1,pose_delay=.05,steering_gain=1.15)]):
  r,t=rollout(ROOT,True,tuning=tuning,prediction=prediction,**c);r['prediction_s']=prediction;r['tuning']=tuning;r['case']=i;results.append(r)
  print(prediction,tuning,i,r['passed'],r['failure'],r['return_cross_track_rms_m'],flush=True)
(ROOT/'run_logs/prediction_comparison.json').write_text(json.dumps(results,indent=2)+'\n')
