#!/usr/bin/env python3
"""Summarize a stationary-LiDAR lead speed capture; never infer full footprint."""
import argparse,json
from pathlib import Path
import numpy as np

def analyze(folder):
    rows=[]
    for line in (folder/'samples.jsonl').read_text().splitlines():
        try:d=json.loads(line)
        except json.JSONDecodeError:continue
        if d.get('target') is not None and not d.get('tracking_lost'):
            c=d['target'];rows.append([d['stamp'],c['x'],c['y'],c['width'],c['n']])
    result=dict(valid_speed_estimate=False,full_body_dimensions_measured=False,
        ego_stationary='operator assumption; no F1 actuator started',samples=len(rows),
        speed_source='relative laser-cluster motion, not wheel encoder',pwm_us=1160)
    if len(rows)<30:return dict(result,reason='insufficient_target_samples')
    a=np.array(rows);t=a[:,0]-a[0,0];x=a[:,1];y=a[:,2]
    result.update(total_forward_change_m=float(x[-1]-x[0]),last_visible_x_m=float(x[-1]),
                  visible_cluster_width_median_m=float(np.median(a[:,3])))
    # One-second regressions suppress variation of the illuminated rear surface.
    fits=[]
    for mid in np.arange(t[0]+.5,t[-1]-.5,.1):
        ids=np.flatnonzero(np.abs(t-mid)<=.5)
        if len(ids)<20 or np.max(np.diff(t[ids]))>.15:continue
        ts=t[ids]-mid;px=np.polyfit(ts,x[ids],1);py=np.polyfit(ts,y[ids],1)
        err=float(np.sqrt(np.mean((x[ids]-np.polyval(px,ts))**2)))
        if px[0]>.12 and err<.06:fits.append([mid,float(px[0]),float(py[0]),err])
    if not fits or np.max(x)-np.min(x)<.5:return dict(result,reason='no_sufficient_forward_motion')
    fits=np.array(fits);groups=np.split(fits,np.flatnonzero(np.diff(fits[:,0])>.16)+1)
    f=max(groups,key=len)
    if len(f)<3:return dict(result,reason='motion_windows_not_continuous')
    result.update(valid_speed_estimate=True,reason='approximate_motion_speed_not_verified_steady_state',
        window_count=len(f),forward_speed_median_mps=float(np.median(f[:,1])),
        forward_speed_p10_p90_mps=[float(v) for v in np.quantile(f[:,1],[.1,.9])],
        lateral_speed_median_mps=float(np.median(f[:,2])),regression_rmse_max_m=float(np.max(f[:,3])),
        motion_window_start_s=float(f[0,0]-.5),motion_window_end_s=float(f[-1,0]+.5),
        stable_cruise_speed_confirmed=False,
        longitudinal_length_measured=False)
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);args=p.parse_args()
    result=analyze(args.folder);(args.folder/'measurement.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
