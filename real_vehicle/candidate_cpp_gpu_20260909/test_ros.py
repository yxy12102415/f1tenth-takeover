"""Synthetic inputs + unchanged final/actuator guards; domain 83 ONLY.

No driver, joystick, LiDAR, VESC, SLAM or physical device is opened.
Virtual sink observes motor-mode commands. Delays are observed, not WCET.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, String
from ackermann_msgs.msg import AckermannDriveStamped

ROOT=Path(__file__).resolve().parent
ROUTE=ROOT.parent/'routes/manual_route_20260904_215501_controller_ready.csv'
CPP=ROOT/'install_v1/candidate_pp/lib/candidate_pp/candidate_pp_node'

def spawn(module,params,log,command=None):
    cmd=command or [sys.executable,'-c','from '+module+' import main; main()']
    cmd+=['--ros-args']
    for k,v in params.items():cmd+=['-p',k+':='+str(v).lower() if isinstance(v,bool) else k+':='+str(v)]
    return subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)

def cpu(pid):
    parts=Path('/proc/%d/stat'%pid).read_text().split()
    return (int(parts[13])+int(parts[14]))/os.sysconf('SC_CLK_TCK')

def trial(kind,controller='cpp',seconds=7.):
    node=Node('candidate_offline_fixture',start_parameter_services=False)
    pubs={}
    for key,topic,typ in [('pose','localization_pose',Odometry),('odom','odom',Odometry),
                           ('scan','scan',LaserScan),('quality','localization_valid',Bool),
                           ('enable','enable',Bool),('deadman','deadman',Bool),('arm','arm',Bool)]:
        pubs[key]=node.create_publisher(typ,'/baseline/'+topic,1)
    commands=[];statuses=[];motor=[];coast=[];finals=[];final_statuses=[]
    node.create_subscription(AckermannDriveStamped,'/baseline/drive_nominal',lambda m:commands.append((time.monotonic(),m.drive.speed,m.drive.steering_angle)),1)
    node.create_subscription(String,'/baseline/controller_status',lambda m:statuses.append((time.monotonic(),json.loads(m.data))),1)
    node.create_subscription(AckermannDriveStamped,'/candidate/final',lambda m:finals.append((time.monotonic(),m.drive.speed)),1)
    node.create_subscription(String,'/candidate/final_status',lambda m:final_statuses.append((time.monotonic(),json.loads(m.data))),1)
    node.create_subscription(Float64,'/candidate/motor_speed',lambda m:motor.append((time.monotonic(),m.data)),1)
    node.create_subscription(Float64,'/candidate/motor_current',lambda m:coast.append((time.monotonic(),m.data)),1)
    children=[];log=(ROOT/('ros_'+controller+'_'+kind+'.log')).open('w')
    try:
        p=spawn('',{'route_file':str(ROUTE)},log,[str(CPP)]) if controller=='cpp' else spawn('baseline_pf_pp.follower',{'route_file':str(ROUTE)},log)
        children.append(p)
        children.append(spawn('f1tenth_safety_migration.final_command_guard',{
            'nominal_topic':'/baseline/drive_nominal','scan_topic':'/baseline/scan','odom_topic':'/baseline/odom',
            'planner_valid_topic':'/baseline/planner_valid','deadman_topic':'/baseline/deadman','arm_topic':'/baseline/arm',
            'final_topic':'/candidate/final','status_topic':'/candidate/final_status','require_target_belief':False,
            'require_deadman':True,'max_speed':.5,'max_steering_angle':.27,'nominal_timeout':.15,
            'planner_timeout':.15,'scan_timeout':.2,'odom_timeout':.2,'deadman_timeout':.15},log))
        children.append(spawn('f1tenth_safety_migration.hardware_actuator_guard',{
            'final_topic':'/candidate/final','motor_topic':'/candidate/motor_speed',
            'motor_current_topic':'/candidate/motor_current','servo_topic':'/candidate/servo',
            'status_topic':'/candidate/actuator_status','max_abs_speed':.5,'max_abs_rpm':3750.,
            'max_abs_steering':.27,'publish_rate':50.},log))
        start=time.monotonic();fault=None;last_fast=last_pose=last_scan=0.;record_cpu=None;armed_at=None
        x,y,yaw=map(float,ROUTE.read_text().splitlines()[1].split(','))
        while time.monotonic()-start<seconds:
            n=time.monotonic();elapsed=n-start
            # Discovery must be confirmed AT THE FINAL GUARD, not merely by
            # observing the controller in this fixture's separate subscriber.
            final_ready=(len(finals)>=10 and n-finals[-1][0]<.1 and all(v==0 for _,v in finals[-10:])
                         and len(final_statuses)>=10 and n-final_statuses[-1][0]<.1
                         and all(s['arm_state']=='DISARMED' and all(0<=s['input_age_sec'][k]<.1
                                 for k in ('nominal','scan','odom','planner','deadman'))
                                 for _,s in final_statuses[-10:]))
            if (armed_at is None and elapsed>2. and len(coast)>=10 and len(statuses)>=10 and final_ready
                    and all(s['valid'] for _,s in statuses[-10:])
                    and commands and n-commands[-1][0]<.1):
                armed_at=n
            active=armed_at is not None and n-armed_at>=.7 and kind!='healthy'
            if active and fault is None:
                assert any(v>0 for _,v in motor[-20:]),'no positive virtual RPM before fault'
                fault=n
                if kind=='controller_kill':p.kill()
                if kind=='final_guard_kill':children[1].kill()
            if elapsed>=2.0 and record_cpu is None:record_cpu=(n,cpu(p.pid))
            if n-last_fast>=.02:
                last_fast=n
                pubs['enable'].publish(Bool(data=not(active and kind=='enable_release')))
                pubs['deadman'].publish(Bool(data=not(active and kind=='deadman_release')))
                # Allow discovery and disarmed STOP before a single rising arm.
                pubs['arm'].publish(Bool(data=armed_at is not None))
                pubs['quality'].publish(Bool(data=not(active and kind=='planner_invalid')))
                if not(active and kind=='odom_timeout'):
                    m=Odometry();m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id='odom';m.child_frame_id='base_link';m.pose.pose.orientation.w=1.
                    pubs['odom'].publish(m)
            if n-last_scan>=.025 and not(active and kind=='scan_timeout'):
                last_scan=n;s=LaserScan();s.header.stamp=node.get_clock().now().to_msg();s.header.frame_id='laser'
                s.angle_min=-1.;s.angle_increment=.01;s.angle_max=1.;s.range_min=.05;s.range_max=10.;s.ranges=[3.]*201
                if active and kind=='scan_nan':s.ranges=[float('nan')]*201
                pubs['scan'].publish(s)
            if n-last_pose>=.05 and kind!='no_pose_at_start' and not(active and kind=='pose_timeout'):
                last_pose=n;m=Odometry();m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id='map';m.child_frame_id='base_link'
                m.pose.pose.position.x=x;m.pose.pose.position.y=y
                m.pose.pose.orientation.z=math.sin(yaw/2);m.pose.pose.orientation.w=math.cos(yaw/2)
                if active:
                    if kind=='pose_nan':m.pose.pose.position.x=float('nan')
                    if kind=='pose_inf':m.pose.pose.position.x=float('inf')
                    if kind=='pose_replay':m.header.stamp.sec-=1
                    if kind=='wrong_frame':m.header.frame_id='odom'
                    if kind=='bad_covariance':m.pose.covariance[0]=1.
                    if kind=='bad_quaternion':m.pose.pose.orientation.w=2.
                pubs['pose'].publish(m)
            rclpy.spin_once(node,timeout_sec=.001)
            for i,c in enumerate(children):
                if not(active and ((i==0 and kind=='controller_kill') or (i==1 and kind=='final_guard_kill'))):assert c.poll() is None,'child exited'
        if kind=='no_pose_at_start':
            assert commands and len(commands)>=10 and all(v==0 and a==0 for _,v,a in commands)
            assert not motor and coast and armed_at is None
            result={'test':kind,'controller':controller,'continuous_stop_commands':len(commands),'passed':True}
            print(json.dumps(result),flush=True);return result
        assert armed_at is not None,'offline readiness not reached'
        assert kind=='healthy' or fault is not None,'fault not injected'
        used=None if p.poll() is not None else (cpu(p.pid)-record_cpu[1])/(time.monotonic()-record_cpu[0])*100.
        assert commands and all(math.isfinite(v) and math.isfinite(a) and 0<=v<=.5 and abs(a)<=.27 for _,v,a in commands)
        result={'test':kind,'controller':controller,'controller_cpu_one_core_pct':used,'commands':len(commands)}
        if fault is not None:
            zeros=[t for t,v in coast if t>=fault and v==0.]
            assert zeros,'no downstream zero-current coast after fault'
            stopped=zeros[0]
            assert stopped-fault<.40,(kind,stopped-fault)
            assert all(v==0 for t,v in coast if t>=stopped)
            assert not any(t>stopped+.03 and v!=0 for t,v in motor),'nonzero RPM resumed after STOP'
            result['coast_latency_ms']=(stopped-fault)*1000.
        else:
            selected=[(t,s) for t,s in statuses if t>start+2.]
            ages=[s['input_age']['pose']['source_age_s']*1000 for _,s in selected if s.get('valid')]
            gaps=[(selected[i][0]-selected[i-1][0])*1000 for i in range(1,len(selected))]
            assert ages and any(v>0 for _,v in motor),{'last_statuses':selected[-3:],'motor':motor[-3:], 'commands':commands[-3:]}
            result.update(max_pose_age_at_status_ms=max(ages),max_status_receipt_gap_ms=max(gaps),
                          valid_status_fraction=sum(s['valid'] for _,s in selected)/len(selected))
        result['passed']=True;print(json.dumps(result),flush=True);return result
    finally:
        (ROOT/('diagnostic_'+controller+'_'+kind+'.json')).write_text(json.dumps({
            'last_controller_statuses':statuses[-5:], 'last_final_statuses':final_statuses[-10:],
            'last_commands':commands[-5:],'last_final_commands':finals[-5:],'motor_samples':len(motor),
            'last_motor':motor[-5:], 'coast_samples':len(coast)},indent=2)+'\n')
        for c in reversed(children):
            if c.poll() is None:c.send_signal(signal.SIGINT)
        for c in children:
            try:c.wait(timeout=5)
            except subprocess.TimeoutExpired:c.kill();c.wait()
        node.destroy_node();log.close()

def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='83' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    rclpy.init();results=[]
    try:
        trials=[('no_pose_at_start','cpp',5.),('healthy','python',12.),('healthy','cpp',12.)]
        for kind in ('pose_timeout','scan_timeout','odom_timeout','pose_nan','pose_inf','scan_nan',
                     'planner_invalid','pose_replay','wrong_frame','enable_release','deadman_release',
                     'controller_kill','final_guard_kill','bad_covariance','bad_quaternion'):
            trials.append((kind,'cpp',7.))
        for kind,controller,seconds in trials:
            try:results.append(trial(kind,controller,seconds))
            except AssertionError as exc:
                result={'test':kind,'controller':controller,'passed':False,'error':str(exc)}
                results.append(result);print(json.dumps(result),flush=True)
    finally:
        rclpy.shutdown();(ROOT/'ros_results.json').write_text(json.dumps(results,indent=2)+'\n')
    if not all(r['passed'] for r in results):raise SystemExit(1)
if __name__=='__main__':main()
