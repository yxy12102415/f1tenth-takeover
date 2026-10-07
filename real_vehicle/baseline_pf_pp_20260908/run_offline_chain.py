"""Actual legacy MCL + PP + existing guards + virtual sink, synthetic sensors.

The ground-truth pose is used ONLY here to synthesize scans / evaluate error.
Never starts device drivers; refuses domain 0. No vehicle command topic used.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
from PIL import Image
import yaml
import range_libc
import rclpy
from nav_msgs.msg import OccupancyGrid,Odometry
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan,Joy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64,String
from ackermann_msgs.msg import AckermannDriveStamped
from check_runtime import check as check_runtime
from session_guard import acquire
from readiness import ready_to_arm


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--fault',choices=['scan_timeout','odom_timeout','pf_kill','guard_kill','deadman',
                                          'scan_nan','odom_nan','scan_replay','pp_pause','none'],default='scan_timeout')
    parser.add_argument('--healthy-seconds',type=float,default=4.)
    parser.add_argument('--scan-hz',type=int,choices=[10,20],default=10)
    args=parser.parse_args()
    if int(os.environ.get('ROS_DOMAIN_ID','0'))==0:
        raise SystemExit('REFUSED: use isolated nonzero domain')
    if not math.isfinite(args.healthy_seconds) or not 4.<=args.healthy_seconds<=600.:
        raise SystemExit('healthy-seconds must be between 4 and 600')
    runtime=check_runtime()
    session_lock=acquire()
    root=Path(__file__).resolve().parent;ws=root.parent
    log=root/'run_logs'/('%s-%s'%(time.strftime('%Y%m%d_%H%M%S'),args.fault))
    log.mkdir(parents=True,exist_ok=False)
    children={};handles=[]
    (log/'runtime.json').write_text(json.dumps(runtime,indent=2)+'\n')
    rclpy.init();node=rclpy.create_node('baseline_synthetic_fixture')
    metadata=yaml.safe_load((ws/'maps/ps4_slam_20260904_163930.yaml').read_text())
    image_path=Path(metadata['image'])
    if not image_path.is_absolute():image_path=ws/'maps'/image_path
    pixels=np.asarray(Image.open(image_path).convert('L'))[::-1]
    occupancy=pixels/255. if metadata.get('negate',0) else 1.-pixels/255.
    grid=OccupancyGrid();grid.header.frame_id='map'
    grid.info.width=pixels.shape[1];grid.info.height=pixels.shape[0]
    grid.info.resolution=float(metadata['resolution'])
    grid.info.origin.position.x=float(metadata['origin'][0]);grid.info.origin.position.y=float(metadata['origin'][1])
    grid.info.origin.orientation.z=math.sin(metadata['origin'][2]/2);grid.info.origin.orientation.w=math.cos(metadata['origin'][2]/2)
    grid.data=np.where(occupancy>metadata['occupied_thresh'],100,
        np.where(occupancy<metadata['free_thresh'],0,-1)).astype(np.int8).ravel().tolist()
    def get_map(req,response):response.map=grid;return response
    service=node.create_service(GetMap,'/map_server/map',get_map)
    from f1tenth_safety_migration.route_mpc_core import load_open_route
    route=str(ws/'routes/manual_route_20260904_215501_controller_ready.csv')
    truth=load_open_route(route)[0,:3]
    angles=np.linspace(-2.35,2.35,95).astype(np.float32)
    caster=range_libc.PyCDDTCast(range_libc.PyOMap(grid),int(10./grid.info.resolution),112)
    queries=np.zeros((len(angles),3),dtype=np.float32)
    queries[:,0]=truth[0]+.27*math.cos(truth[2]);queries[:,1]=truth[1]+.27*math.sin(truth[2]);queries[:,2]=truth[2]+angles
    ranges=np.zeros(len(angles),dtype=np.float32);caster.calc_range_many(queries,ranges)
    ranges=np.clip(ranges,.05,10.)
    op=node.create_publisher(Odometry,'/baseline/odom',1)
    sp=node.create_publisher(LaserScan,'/baseline/scan',1)
    jp=node.create_publisher(Joy,'/baseline/joy',1)
    ip=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',1)
    statuses={};rpm=[];poses=[];nominal_samples=[];guard_samples=[];pf_samples=[]
    status_events=[];pose_timing=[];scan_timing=[]
    guard_received=None
    def status_cb(key,msg):
        nonlocal guard_received
        data=json.loads(msg.data)
        previous=statuses.get(key,{})
        state_keys=('reason','valid','arm_state','stop_reason')
        if any(data.get(k)!=previous.get(k) for k in state_keys):
            status_events.append({'stream':key,'received_monotonic':time.monotonic(),
                                  'received_ros':node.get_clock().now().nanoseconds*1e-9,'status':data})
        statuses[key]=data
        if key=='guard':
            guard_received=time.monotonic();guard_samples.append((guard_received,data))
        if key=='pf':pf_samples.append((time.monotonic(),data.get('compute_ms',0.)))
    for key,topic in [('pf','/baseline/localization_status'),('pp','/baseline/controller_status'),('guard','/baseline/final_status')]:
        node.create_subscription(String,topic,lambda m,k=key:status_cb(k,m),10)
    def nominal_cb(msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        nominal_samples.append((time.monotonic(),stamp,node.get_clock().now().nanoseconds*1e-9-stamp))
    node.create_subscription(AckermannDriveStamped,'/baseline/drive_nominal',nominal_cb,10)
    node.create_subscription(Float64,'/baseline/effective_rpm',lambda m:rpm.append((time.monotonic(),m.data)),10)
    def pose_cb(msg):
        now=time.monotonic();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        poses.append((now,msg.pose.pose.position.x,msg.pose.pose.position.y))
        pose_timing.append((now,stamp,node.get_clock().now().nanoseconds*1e-9-stamp))
    node.create_subscription(Odometry,'/baseline/localization_pose',pose_cb,10)
    held=False;inject=False;initial_sent=False;counter=0;last_scan_stamp=None
    armed_at=None;fault_at=None
    def timing_report():
        begin=armed_at if armed_at is not None else 0.
        end=fault_at if fault_at is not None else time.monotonic()
        commands=[s for s in nominal_samples if begin<=s[0]<end]
        def summary(values):
            a=np.asarray(values,dtype=float)*1000.
            return {'count':len(a),'p99_ms':float(np.percentile(a,99)) if len(a) else None,
                    'max_ms':float(np.max(a)) if len(a) else None}
        result={'healthy_seconds_requested':args.healthy_seconds,
                'scan_hz':args.scan_hz,
                'nominal_receive_gap':summary(np.diff([s[0] for s in commands])),
                'nominal_source_gap':summary(np.diff([s[1] for s in commands])),
                'nominal_source_to_fixture':summary([s[2] for s in commands])}
        armed=[d for t,d in guard_samples if begin<=t<end and d.get('armed')]
        result['guard_nominal_age']=summary([d['input_age_sec']['nominal'] for d in armed])
        result['guard_nonarmed_after_motion']=[{'t':t,'status':d} for t,d in guard_samples
            if moving_start is not None and moving_start<=t<end and not d.get('armed')]
        result['pf_compute_observed']=summary([v/1000. for t,v in pf_samples if begin<=t<end])
        ps=[p for p in pose_timing if begin<=p[0]<end]
        result['pose_source_gap']=summary(np.diff([p[1] for p in ps]))
        result['pose_receive_gap']=summary(np.diff([p[0] for p in ps]))
        result['pose_source_to_fixture']=summary([p[2] for p in ps])
        result['scan_source_gap']=summary(np.diff([p[1] for p in scan_timing if begin<=p[0]<end]))
        result['status_events']=status_events
        return result
    moving_start=None
    def inputs():
        nonlocal initial_sent,counter,last_scan_stamp
        counter+=1;stamp=node.get_clock().now().to_msg()
        if not (inject and args.fault=='odom_timeout'):
            od=Odometry();od.header.stamp=stamp;od.header.frame_id='odom';od.child_frame_id='base_link'
            od.pose.pose.orientation.w=1.
            if inject and args.fault=='odom_nan':od.twist.twist.linear.x=math.nan
            op.publish(od)
        if not (inject and args.fault=='scan_timeout') and counter%(20//args.scan_hz)==0:
            sc=LaserScan();sc.header.stamp=last_scan_stamp if inject and args.fault=='scan_replay' else stamp
            last_scan_stamp=sc.header.stamp;sc.header.frame_id='laser'
            sc.angle_min=float(angles[0]);sc.angle_max=float(angles[-1]);sc.angle_increment=float(angles[1]-angles[0])
            sc.range_min=.05;sc.range_max=10.;sc.ranges=ranges.tolist()
            if inject and args.fault=='scan_nan':sc.ranges[len(sc.ranges)//2]=math.nan
            sp.publish(sc)
            scan_timing.append((time.monotonic(),sc.header.stamp.sec+sc.header.stamp.nanosec*1e-9))
        if not initial_sent and ip.get_subscription_count()>0:
            pose=PoseWithCovarianceStamped();pose.header.stamp=stamp;pose.header.frame_id='map'
            pose.pose.pose.position.x=float(truth[0]);pose.pose.pose.position.y=float(truth[1])
            pose.pose.pose.orientation.z=math.sin(truth[2]/2);pose.pose.pose.orientation.w=math.cos(truth[2]/2)
            ip.publish(pose);initial_sent=True
        joy=Joy();joy.buttons=[0]*13;joy.buttons[4]=int(held and not(inject and args.fault=='deadman'));jp.publish(joy)
    def spawn(key,module,params=None,config=None):
        code='import numpy as np; np.random.seed(7); from '+module.rsplit('.',1)[0]+' import '+module.rsplit('.',1)[1]+' as m; m.main()'
        cmd=[sys.executable,'-c',code,'--ros-args']
        if config:cmd+=['--params-file',str(config)]
        for k,v in (params or {}).items():cmd+=['-p',k+':='+str(v).lower() if isinstance(v,bool) else k+':='+str(v)]
        handle=(log/(key+'.log')).open('x');handles.append(handle)
        children[key]=subprocess.Popen(cmd,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        spawn('pf','baseline_pf_pp.safe_pf',config=root/'config/pf_offline.yaml')
        spawn('pp','baseline_pf_pp.follower',{'route_file':route})
        spawn('operator','f1tenth_safety_migration.route_operator_gate',{
            'joy_topic':'/baseline/joy','arm_topic':'/baseline/arm','deadman_topic':'/baseline/deadman',
            'enable_topic':'/baseline/enable','nominal_topic':'/baseline/drive_nominal',
            'planner_valid_topic':'/baseline/planner_valid','status_topic':'/baseline/operator_status'})
        spawn('guard','f1tenth_safety_migration.final_command_guard',{
            'nominal_topic':'/baseline/drive_nominal','scan_topic':'/baseline/scan','odom_topic':'/baseline/odom',
            'planner_valid_topic':'/baseline/planner_valid','deadman_topic':'/baseline/deadman',
            'arm_topic':'/baseline/arm','final_topic':'/baseline/final','status_topic':'/baseline/final_status',
            'require_target_belief':False,'require_deadman':True,'max_speed':.5,'max_steering_angle':.27,
            'nominal_timeout':.15,'planner_timeout':.15,'scan_timeout':.2,'odom_timeout':.2,'deadman_timeout':.15})
        spawn('stale','f1tenth_safety_migration.stale_command_guard',{
            'final_topic':'/baseline/final','motor_rpm_topic':'/baseline/rpm','status_topic':'/baseline/stale_status',
            'speed_to_erpm_gain':7500.,'max_abs_rpm':3750.})
        spawn('sink','f1tenth_safety_migration.virtual_vehicle_sink',{
            'motor_rpm_topic':'/baseline/rpm','effective_rpm_topic':'/baseline/effective_rpm',
            'status_topic':'/baseline/sink_status','max_abs_rpm':3750.})
        timer=node.create_timer(.05,inputs)
        start=time.monotonic();ready_since=None
        while time.monotonic()-start<40.:
            rclpy.spin_once(node,timeout_sec=.005)
            if any(p.poll() is not None for p in children.values()):raise RuntimeError('Child exited; inspect '+str(log))
            good=ready_to_arm(statuses,guard_received,time.monotonic(),len(rpm))
            ready_since=(ready_since or time.monotonic()) if good else None
            if ready_since and time.monotonic()-ready_since>1.:break
        else:raise AssertionError('Never ready: '+json.dumps(statuses))
        assert rpm and all(v==0. for t,v in rpm),'Not continuously STOP before hold'
        held=True;armed_at=time.monotonic()
        while time.monotonic()-armed_at<args.healthy_seconds:
            rclpy.spin_once(node,timeout_sec=.005)
        moving=[(t,v) for t,v in rpm if t>=armed_at and v>0.]
        moving_start=moving[0][0] if moving else None
        assert moving,'No virtual motion: '+json.dumps(statuses)
        assert all(v>0. for t,v in rpm if t>=moving[0][0]),'Premature virtual STOP: '+json.dumps(statuses)
        fault_at=time.monotonic()
        (log/'timing.json').write_text(json.dumps(timing_report(),indent=2)+'\n')
        if args.fault=='none':
            result={'fault':'none','healthy_seconds':args.healthy_seconds,
                    'virtual_motion_samples':len(moving),'log_dir':str(log)}
            (log/'result.json').write_text(json.dumps(result,indent=2)+'\n')
            print('RESULT '+json.dumps(result),flush=True)
            return
        inject=True
        if args.fault in ('pf_kill','guard_kill'):
            os.killpg(children['pf' if args.fault=='pf_kill' else 'guard'].pid,signal.SIGKILL)
        if args.fault=='pp_pause':os.killpg(children['pp'].pid,signal.SIGSTOP)
        while time.monotonic()-fault_at<1.:
            rclpy.spin_once(node,timeout_sec=.005)
        zeros=[t for t,v in rpm if t>=fault_at and v==0.]
        assert zeros,'No virtual STOP after fault'
        assert all(v==0. for t,v in rpm if t>=zeros[0]),'STOP was not persistent'
        delay=(zeros[0]-fault_at)*1000
        assert delay<500.,'Observed STOP exceeded offline 500ms test bound'
        error=max(math.hypot(x-truth[0],y-truth[1]) for t,x,y in poses)
        result={'fault':args.fault,'virtual_stop_ms':delay,'pose_samples':len(poses),
            'synthetic_position_error_max_m':error,'pf_compute_ms_last':statuses.get('pf',{}).get('compute_ms'),
            'guard_stop_reason':statuses.get('guard',{}).get('stop_reason'),
            'virtual_motion_samples':len(moving),'log_dir':str(log)}
        (log/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print('RESULT '+json.dumps(result),flush=True)
    except Exception as exc:
        (log/'timing.json').write_text(json.dumps(timing_report(),indent=2)+'\n')
        (log/'failure.json').write_text(json.dumps({
            'fault_requested':args.fault, 'phase':'fault_injected' if inject else 'pre_fault',
            'error':str(exc), 'last_statuses':statuses},indent=2)+'\n')
        raise
    finally:
        for key in ('operator','pp','pf','guard','stale','sink'):
            process=children.get(key)
            if process and process.poll() is None:
                if key=='pp' and args.fault=='pp_pause':os.killpg(process.pid,signal.SIGCONT)
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
        for handle in handles:handle.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
