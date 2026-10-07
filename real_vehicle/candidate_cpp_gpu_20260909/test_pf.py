"""Stationary synthetic scan experiment on saved map; no vehicle publishers.

Ground truth is ONLY fixture-generated and never connected to hardware domains.
The same CPU ray-marching scan is fed to both CPU-CDDT and GPU-RM PFs.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

ROOT=Path(__file__).resolve().parent
BASE=ROOT.parent/'baseline_pf_pp_20260908'
sys.path.insert(0,str(BASE))
from stand_map_io import load_map
import range_libc # CPU fixture, never candidate native import in this process
from test_ros import cpu

def trial(mode):
    node=Node('pf_offline_fixture',start_parameter_services=False);grid=load_map()
    node.create_service(GetMap,'/map_server/map',lambda req,res:map_response(res,grid))
    scan=node.create_publisher(LaserScan,'/baseline/scan',1)
    odom=node.create_publisher(Odometry,'/baseline/odom',1)
    initial=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',1)
    poses=[];statuses=[]
    node.create_subscription(Odometry,'/baseline/localization_pose',lambda m:poses.append((time.monotonic(),m)),1)
    node.create_subscription(String,'/baseline/localization_status',lambda m:statuses.append((time.monotonic(),json.loads(m.data))),1)
    x,y,yaw=np.loadtxt(ROOT.parent/'routes/manual_route_20260904_215501_controller_ready.csv',skiprows=1,delimiter=',')[0]
    angles=np.linspace(-2.35,2.35,1081,dtype=np.float32)
    queries=np.zeros((1081,3),dtype=np.float32)
    queries[:,0]=x+.27*math.cos(yaw);queries[:,1]=y+.27*math.sin(yaw);queries[:,2]=yaw+angles
    caster=range_libc.PyRayMarching(range_libc.PyOMap(grid),200)
    ranges=np.zeros(1081,dtype=np.float32);caster.calc_range_many(queries,ranges)
    ranges=np.clip(ranges,.05,10.)
    cmd=([sys.executable,str(ROOT/'gpu_pf_offline.py'),'--ros-args','--params-file',str(ROOT/'config/pf_gpu_offline.yaml')]
         if mode=='gpu' else [sys.executable,'-c','import numpy as np; np.random.seed(20260909); from baseline_pf_pp.safe_pf import main; main()',
                              '--ros-args','--params-file',str(BASE/'config/pf_stationary_candidate.yaml')])
    log=(ROOT/('pf_'+mode+'.log')).open('w');proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
    begin=time.monotonic();last=0.;sent=False;measure=None
    try:
        while time.monotonic()-begin<20.:
            t=time.monotonic()
            assert proc.poll() is None,'PF exited; inspect '+str(log.name)
            if t-begin>=6. and measure is None:measure=(t,cpu(proc.pid))
            if t-last>=.025:
                last=t;stamp=node.get_clock().now().to_msg()
                o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.child_frame_id='base_link';o.pose.pose.orientation.w=1.;odom.publish(o)
                s=LaserScan();s.header.stamp=stamp;s.header.frame_id='laser';s.angle_min=float(angles[0]);s.angle_max=float(angles[-1]);s.angle_increment=float(angles[1]-angles[0]);s.range_min=.05;s.range_max=10.;s.ranges=ranges.tolist();scan.publish(s)
                if not sent and initial.get_subscription_count()>0:
                    p=PoseWithCovarianceStamped();p.header.stamp=stamp;p.header.frame_id='map';p.pose.pose.position.x=x;p.pose.pose.position.y=y
                    p.pose.pose.orientation.z=math.sin(yaw/2);p.pose.pose.orientation.w=math.cos(yaw/2);initial.publish(p);sent=True
            rclpy.spin_once(node,timeout_sec=.001)
        used=(cpu(proc.pid)-measure[1])/(time.monotonic()-measure[0])*100.
        selected=[s for t,s in statuses if t>measure[0]]
        ps=[p for t,p in poses if t>measure[0]]
        assert selected and ps,'no localization samples'
        errors=[math.hypot(p.pose.pose.position.x-x,p.pose.pose.position.y-y) for p in ps]
        headings=[abs(math.atan2(math.sin(2*math.atan2(p.pose.pose.orientation.z,p.pose.pose.orientation.w)-yaw),math.cos(2*math.atan2(p.pose.pose.orientation.z,p.pose.pose.orientation.w)-yaw))) for p in ps]
        result={'mode':mode,'duration_s':20,'measurement_window_s':time.monotonic()-measure[0],
                'cpu_one_core_pct':used,'pose_samples':len(ps),
                'status_valid_fraction':sum(s['valid'] for s in selected)/len(selected),
                'median_compute_ms':float(np.median([s['compute_ms'] for s in selected])),
                'max_compute_ms':max(s['compute_ms'] for s in selected),
                'median_position_error_m':float(np.median(errors)),'max_position_error_m':max(errors),
                'max_heading_error_rad':max(headings),'synthetic_only':True,'hardware_started':False}
        print(json.dumps(result),flush=True);return result
    finally:
        if proc.poll() is None:proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=8)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        node.destroy_node();log.close()

def map_response(response,grid):response.map=grid;return response

def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='83' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    rclpy.init();results=[]
    try:
        for mode in ('cpu','gpu'):results.append(trial(mode))
    finally:
        rclpy.shutdown();(ROOT/'pf_comparison.json').write_text(json.dumps(results,indent=2)+'\n')
if __name__=='__main__':main()
