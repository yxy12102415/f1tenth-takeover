"""Stationary PF alignment ONLY. No actuator/command/controller node exists here.

Zero odometry is an explicit stationary diagnostic assumption, never navigation
odometry. Do not move the car. Real LiDAR starts only with three explicit flags.
Default is a plan; --offline-self-test starts synthetic input, not a device.
"""
import argparse
from collections import OrderedDict
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parent
FORBIDDEN={'vesc_driver_node','hardware_actuator_guard','ps4_chassis_test_command',
           'joy_node','urg_node_driver','slam_toolbox_node','route_mpc_node','route_pid_node'}


def validate_options(args):
    if args.live_lidar and args.offline_self_test:
        raise ValueError('Choose live LiDAR or offline self-test, not both')
    if args.live_lidar and not (args.stationary_confirmed and args.actuator_power_isolated):
        raise ValueError('Live stationary test requires stationary-confirmed AND actuator-power-isolated')
    if not math.isfinite(args.duration) or not 5.<=args.duration<=600.:
        raise ValueError('Duration must be 5..600 seconds')


def conflicting_processes():
    found=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:argv=(proc/'cmdline').read_bytes().decode(errors='replace').split('\0')
        except (OSError,PermissionError):continue
        if any(Path(arg).name in FORBIDDEN for arg in argv if arg):
            found.append(int(proc.name))
    return found


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live-lidar',action='store_true')
    parser.add_argument('--stationary-confirmed',action='store_true')
    parser.add_argument('--actuator-power-isolated',action='store_true')
    parser.add_argument('--offline-self-test',action='store_true')
    parser.add_argument('--rviz',action='store_true')
    parser.add_argument('--duration',type=float,default=120.)
    args=parser.parse_args()
    try:validate_options(args)
    except ValueError as exc:parser.error(str(exc))
    if not (args.live_lidar or args.offline_self_test):
        print('PLAN ONLY: LiDAR -> stamped scan relay (<=10Hz) -> PF -> map pose + matched scan visualization.\n'
              'Map service + explicitly STATIONARY zero odometry; NO VESC, Joy, PP, MPC, arm or motor topics.\n'
              'Live use requires --live-lidar --stationary-confirmed --actuator-power-isolated.\n'
              'Do not move the car. This does not validate moving localization or route following.')
        return
    if os.environ.get('ROS_DOMAIN_ID')!='83':raise RuntimeError('Use clean_env.sh; domain must be 83')
    if args.rviz and not os.environ.get('DISPLAY'):raise RuntimeError('RViz requires a desktop DISPLAY')
    if args.live_lidar:
        conflicts=conflicting_processes()
        if conflicts:raise RuntimeError('Conflicting driver/controller processes: '+str(conflicts))
    from check_runtime import check
    from session_guard import acquire
    runtime=check();lock=acquire()
    import numpy as np
    from PIL import Image
    import yaml
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid,Odometry
    from nav_msgs.srv import GetMap
    from sensor_msgs.msg import LaserScan,PointCloud2,PointField
    from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker
    from baseline_pf_pp.core import quaternion_yaw
    from baseline_pf_pp.safe_pf import latest_sensor_qos
    log=ROOT/'run_logs'/(time.strftime('%Y%m%d_%H%M%S')+'-sensor-'+('synthetic' if args.offline_self_test else 'stationary'))
    log.mkdir(parents=True,exist_ok=False)
    (log/'runtime.json').write_text(json.dumps(runtime,indent=2)+'\n')
    children=[];handles=[];node=None;report={'mode':'synthetic' if args.offline_self_test else 'stationary',
        'pose_count':0,'matched_cloud_count':0,'actuator_nodes_started':0,'moving_localization_validated':False}
    def request_exit(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,request_exit)
    try:
        rclpy.init();node=rclpy.create_node('baseline_stationary_diagnostic')
        ws=ROOT.parent
        meta=yaml.safe_load((ws/'maps/ps4_slam_20260904_163930.yaml').read_text())
        image_path=Path(meta['image'])
        if not image_path.is_absolute():image_path=ws/'maps'/image_path
        pixels=np.asarray(Image.open(image_path).convert('L'))[::-1]
        occupancy=pixels/255. if meta.get('negate',0) else 1.-pixels/255.
        grid=OccupancyGrid();grid.header.frame_id='map'
        grid.info.width=pixels.shape[1];grid.info.height=pixels.shape[0];grid.info.resolution=float(meta['resolution'])
        grid.info.origin.position.x=float(meta['origin'][0]);grid.info.origin.position.y=float(meta['origin'][1])
        grid.info.origin.orientation.z=math.sin(meta['origin'][2]/2);grid.info.origin.orientation.w=math.cos(meta['origin'][2]/2)
        grid.data=np.where(occupancy>meta['occupied_thresh'],100,np.where(occupancy<meta['free_thresh'],0,-1)).astype(np.int8).ravel().tolist()
        def get_map(req,response):response.map=grid;return response
        service=node.create_service(GetMap,'/map_server/map',get_map)
        map_out=node.create_publisher(OccupancyGrid,'/baseline/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        map_out.publish(grid)
        odom_out=node.create_publisher(Odometry,'/baseline/odom',1)
        scan_out=node.create_publisher(LaserScan,'/baseline/scan',1)
        pose_out=node.create_publisher(PoseStamped,'/baseline/visual_pose',1)
        cloud_out=node.create_publisher(PointCloud2,'/baseline/visual_scan',1)
        label_out=node.create_publisher(Marker,'/baseline/visual_status',1)
        initial_out=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',1) if args.offline_self_test else None
        scans=OrderedDict();last_forward=[0.];last_status=[None];last_pose_time=[0.]
        def key(msg):return (msg.header.stamp.sec,msg.header.stamp.nanosec)
        def scan_cb(msg):
            now=time.monotonic()
            if now-last_forward[0]<.099:return
            last_forward[0]=now;scans[key(msg)]=msg
            while len(scans)>30:scans.popitem(last=False)
            scan_out.publish(msg) # preserve original source timestamp and ranges
        raw_sub=node.create_subscription(LaserScan,'/baseline/raw_scan',scan_cb,latest_sensor_qos())
        def pose_cb(msg):
            report['pose_count']+=1
            last_pose_time[0]=time.monotonic()
            visual=PoseStamped();visual.header=msg.header;visual.pose=msg.pose.pose;pose_out.publish(visual)
            scan=scans.get(key(msg))
            if scan is None:return # never invent a transform for unmatched/stale scan
            yaw=quaternion_yaw(msg.pose.pose.orientation)
            lx=msg.pose.pose.position.x+.27*math.cos(yaw);ly=msg.pose.pose.position.y+.27*math.sin(yaw)
            points=[]
            for i,r in enumerate(scan.ranges):
                if math.isfinite(r) and scan.range_min<=r<scan.range_max:
                    a=yaw+scan.angle_min+i*scan.angle_increment
                    points.append((lx+r*math.cos(a),ly+r*math.sin(a),0.))
            cloud=PointCloud2();cloud.header=msg.header;cloud.height=1;cloud.width=len(points)
            cloud.fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
            cloud.point_step=12;cloud.row_step=12*len(points);cloud.is_dense=True
            cloud.data=b''.join(struct.pack('<fff',*p) for p in points);cloud_out.publish(cloud)
            report['matched_cloud_count']+=1
        pose_sub=node.create_subscription(Odometry,'/baseline/localization_pose',pose_cb,10)
        status_sub=node.create_subscription(String,'/baseline/localization_status',lambda m:last_status.__setitem__(0,json.loads(m.data)),10)
        def spawn(name,cmd):
            handle=(log/(name+'.log')).open('x');handles.append(handle)
            process=subprocess.Popen(cmd,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
            children.append((name,process))
        pf_config='pf_offline.yaml' if args.offline_self_test else 'pf_stationary_candidate.yaml'
        spawn('pf',[sys.executable,'-c','from baseline_pf_pp.safe_pf import main; main()',
            '--ros-args','--params-file',str(ROOT/'config'/pf_config)])
        if args.live_lidar:
            spawn('lidar',['/opt/ros/foxy/lib/urg_node/urg_node_driver','--ros-args',
                '-r','__node:=urg_node','-r','scan:=/baseline/raw_scan',
                '-p','ip_address:=192.168.0.10','-p','ip_port:=10940','-p','laser_frame_id:=laser',
                '-p','calibrate_time:=false'])
        if args.rviz:spawn('rviz',['/opt/ros/foxy/bin/rviz2','-d',str(ROOT/'config/sensor_only.rviz')])
        synthetic=None
        if args.offline_self_test:
            from f1tenth_safety_migration.route_mpc_core import load_open_route
            import range_libc
            truth=load_open_route(str(ws/'routes/manual_route_20260904_215501_controller_ready.csv'))[0,:3]
            angles=np.linspace(-2.35,2.35,95).astype(np.float32)
            caster=range_libc.PyCDDTCast(range_libc.PyOMap(grid),int(10/grid.info.resolution),112)
            queries=np.zeros((95,3),dtype=np.float32)
            queries[:,0]=truth[0]+.27*math.cos(truth[2]);queries[:,1]=truth[1]+.27*math.sin(truth[2]);queries[:,2]=truth[2]+angles
            ranges=np.zeros(95,dtype=np.float32);caster.calc_range_many(queries,ranges)
            synthetic=(truth,angles,np.clip(ranges,.05,10.))
        counter=[0];initial_sent=[False]
        def tick():
            stamp=node.get_clock().now().to_msg()
            od=Odometry();od.header.stamp=stamp;od.header.frame_id='odom';od.child_frame_id='base_link';od.pose.pose.orientation.w=1.
            odom_out.publish(od) # explicit immobile diagnostic assumption
            counter[0]+=1
            if counter[0]%4==0:
                valid=bool(last_status[0] and last_status[0].get('valid') and time.monotonic()-last_pose_time[0]<.25)
                label=Marker();label.header.frame_id='map';label.header.stamp=stamp
                label.ns='stationary_diagnostic';label.id=0;label.type=Marker.TEXT_VIEW_FACING;label.action=Marker.ADD
                label.pose.position.x=grid.info.origin.position.x;label.pose.position.y=grid.info.origin.position.y
                label.pose.position.z=.5;label.pose.orientation.w=1.;label.scale.z=.2;label.color.a=1.
                label.color.g=1. if valid else 0.;label.color.r=0. if valid else 1.
                label.text='STATIONARY ONLY / '+('PF VALID' if valid else 'INVALID OR STALE')
                label_out.publish(label)
                if not valid:
                    empty=PointCloud2();empty.header.frame_id='map';empty.header.stamp=stamp
                    empty.height=1;empty.width=0;empty.point_step=12;empty.row_step=0
                    empty.fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
                    cloud_out.publish(empty)
            if synthetic is not None:
                truth,angles,ranges=synthetic
                if counter[0]%2==0:
                    sc=LaserScan();sc.header.stamp=stamp;sc.header.frame_id='laser'
                    sc.angle_min=float(angles[0]);sc.angle_max=float(angles[-1]);sc.angle_increment=float(angles[1]-angles[0])
                    sc.range_min=.05;sc.range_max=10.;sc.ranges=ranges.tolist();scan_cb(sc)
                if not initial_sent[0] and initial_out.get_subscription_count()>0:
                    initial=PoseWithCovarianceStamped();initial.header.frame_id='map';initial.header.stamp=stamp
                    initial.pose.pose.position.x=float(truth[0]);initial.pose.pose.position.y=float(truth[1])
                    initial.pose.pose.orientation.z=math.sin(truth[2]/2);initial.pose.pose.orientation.w=math.cos(truth[2]/2)
                    initial_out.publish(initial);initial_sent[0]=True
        timer=node.create_timer(.05,tick)
        print('STATIONARY DIAGNOSTIC ONLY: no VESC / no controller. Do not move the car.\n'
              'In RViz: 2D Pose Estimate at the actual position and heading. Ctrl+C once to exit.\n'
              'Logs: '+str(log),flush=True)
        started=time.monotonic()
        while time.monotonic()-started<args.duration:
            rclpy.spin_once(node,timeout_sec=.01)
            for name,p in children:
                if p.poll() is not None:raise RuntimeError(name+' exited; inspect logs')
        report['last_status']=last_status[0]
        if args.offline_self_test:
            assert report['pose_count']>=10 and report['matched_cloud_count']>=10, 'Insufficient matched visualization samples'
            assert last_status[0] and last_status[0]['valid'], 'PF not valid'
        print(json.dumps(report),flush=True)
    except KeyboardInterrupt:report['operator_exit']=True
    except Exception as exc:
        report['error']=str(exc);raise
    finally:
        # A second Ctrl+C must not interrupt child cleanup.
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        signal.signal(signal.SIGTERM,signal.SIG_IGN)
        for name,p in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=3)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
        for handle in handles:handle.close()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        (log/'result.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
