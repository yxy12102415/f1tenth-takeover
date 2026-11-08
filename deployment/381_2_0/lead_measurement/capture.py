#!/usr/bin/env python3
"""Stationary F1 LiDAR measurement only. No publishers and no VESC access.

Coordinates are in the laser frame, not map coordinates. A visible cluster
extent is not the vehicle's complete footprint. Touch SELECT in the output
directory only after the operator confirms the intended isolated lead target.
"""
import argparse, json, math, os, signal, subprocess, sys, time
from pathlib import Path
import numpy as np

WS=Path('/home/f1tenth/f1tenth_real_migration_ws_20260829')
sys.path.insert(0,str(WS/'baseline_pf_pp_20260908'))
from session_guard import acquire
from sensor_only import conflicting_processes

def clusters(scan):
    out=[]; group=[]; previous=None
    def finish():
        if len(group)<4:return
        p=np.asarray(group); lo=p.min(axis=0); hi=p.max(axis=0)
        # Width/extent are visible laser returns only; not physical dimensions.
        if not .035 <= hi[1]-lo[1] <= .9 or hi[0]-lo[0] > 1.1:return
        out.append(dict(x=float(np.median(p[:,0])),y=float(np.median(p[:,1])),
                        width=float(hi[1]-lo[1]),depth=float(hi[0]-lo[0]),n=len(group)))
    for i,r in enumerate(scan.ranges):
        a=scan.angle_min+i*scan.angle_increment
        good=math.isfinite(r) and max(.15,scan.range_min)<=r<min(7.,scan.range_max)
        p=(r*math.cos(a),r*math.sin(a)) if good else None
        if p is None or not (.4<=p[0]<=6.0 and abs(p[1])<=1.0):
            finish();group=[];previous=None;continue
        if previous is not None and math.hypot(p[0]-previous[0],p[1]-previous[1])>.13:
            finish();group=[]
        group.append(p);previous=p
    finish();return out

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',required=True);ap.add_argument('--duration',type=float,default=300.)
    ap.add_argument('--raw-mode',choices=('full','front'),default='full')
    args=ap.parse_args()
    if not 5<=args.duration<=600:ap.error('duration must be 5..600 s')
    if conflicting_processes():raise RuntimeError('Existing hardware/sensor session; refused')
    lock=acquire();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    (out/'mode.json').write_text(json.dumps(dict(mode='stationary_relative_lidar_measurement',actuator_nodes_started=0,
        assumed_ego_stationary=True,physical_footprint_measured=False,raw_mode=args.raw_mode,initial_target_gate_x_m=[.4,1.1],initial_target_gate_abs_y_m=.3),indent=2)+'\n')
    import rclpy
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init();node=rclpy.create_node('stationary_lead_measurement')
    handle=(out/'lidar.log').open('w');raw=(out/'scans.jsonl').open('w');samples=(out/'samples.jsonl').open('w')
    child=None;last=[None];velocity=[np.zeros(2)];last_print=[0.];count=[0];selected=[False];lost=[False];scan_at=[None]
    def scan(m):
        received=time.monotonic();stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        age=node.get_clock().now().nanoseconds*1e-9-stamp
        if not -.02<=age<=.5:return
        cs=clusters(m);count[0]+=1;scan_at[0]=received
        # All valid ranges retained so cluster association can be audited.
        raw_row=dict(received_mono=received,stamp=stamp,angle_min=m.angle_min,angle_increment=m.angle_increment)
        if args.raw_mode=='full':raw_row['ranges']=[float(v) if math.isfinite(v) else None for v in m.ranges]
        else:
            points=[]
            for k,r in enumerate(m.ranges):
                if not math.isfinite(r):continue
                angle=m.angle_min+k*m.angle_increment;x=r*math.cos(angle);y=r*math.sin(angle)
                if .6<x<4. and abs(y)<.35:points.append([x,y])
            raw_row['front_points_xy']=points
        raw.write(json.dumps(raw_row)+'\n');raw.flush()
        row=dict(received_mono=received,stamp=stamp,source_age_s=age,clusters=cs,target=None)
        candidates=[c for c in cs if .4<c['x']<1.1 and abs(c['y'])<.3]
        target=None
        if not selected[0] and (out/'SELECT').exists():
            if len(candidates)==1:
                target=candidates[0];selected[0]=True
                print('SELECTED lead cluster '+json.dumps(target),flush=True)
            elif received-last_print[0]>1.:
                print('WAIT isolated target: '+str(len(candidates))+' candidates in selection gate',flush=True)
        elif selected[0] and not lost[0]:
            dt=stamp-last[0]['stamp']
            if not 0<dt<.5:lost[0]=True
            else:
                pos=np.array([last[0]['x'],last[0]['y']]);pred=pos+velocity[0]*dt
                near=[c for c in cs if np.linalg.norm(np.array([c['x'],c['y']])-pred)<.18+2.*dt]
                # Ambiguity is explicit; never silently switch identity.
                if len(near)==1:
                    target=near[0];v=(np.array([target['x'],target['y']])-pos)/dt
                    velocity[0]=.8*velocity[0]+.2*v
                    if np.linalg.norm(velocity[0])>3.:lost[0]=True;target=None
                elif received-last[0]['received_mono']>.3:lost[0]=True
        if target is not None:
            row['target']=dict(target,vx=float(velocity[0][0]),vy=float(velocity[0][1]));last[0]=dict(target,stamp=stamp,received_mono=received)
        row['tracking_lost']=lost[0]
        samples.write(json.dumps(row)+'\n');samples.flush()
        if received-last_print[0]>2.:
            print(json.dumps(dict(scans=count[0],selected=selected[0],lost=lost[0],target=row['target'],initial_candidates=candidates)),flush=True);last_print[0]=received
    node.create_subscription(LaserScan,'/lead_measure/raw_scan',scan,qos_profile_sensor_data)
    def stop(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,stop)
    start=time.monotonic();reason='duration'
    try:
        child=subprocess.Popen(['/opt/ros/foxy/lib/urg_node/urg_node_driver','--ros-args','-r','__node:=lead_measure_lidar',
            '-r','scan:=/lead_measure/raw_scan','-p','ip_address:=192.168.0.10','-p','laser_frame_id:=laser',
            '-p','calibrate_time:=false'],stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        print('LIDAR ONLY; no command publishers. Keep F1 stationary. Output: '+str(out),flush=True)
        while time.monotonic()-start<args.duration:
            rclpy.spin_once(node,timeout_sec=.1)
            if child.poll() is not None:raise RuntimeError('LiDAR driver exited')
            if time.monotonic()-start>12 and (scan_at[0] is None or time.monotonic()-scan_at[0]>1):raise RuntimeError('No fresh scans')
        if scan_at[0] is None or time.monotonic()-scan_at[0]>1:raise RuntimeError('No fresh scans at capture completion')
    except KeyboardInterrupt:reason='operator_finished'
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=4)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        raw.close();samples.close();handle.close();node.destroy_node();rclpy.shutdown()
        (out/'capture_result.json').write_text(json.dumps(dict(reason=reason,scans=count[0],selected=selected[0],tracking_lost=lost[0],actuator_nodes_started=0),indent=2)+'\n')
if __name__=='__main__':main()
