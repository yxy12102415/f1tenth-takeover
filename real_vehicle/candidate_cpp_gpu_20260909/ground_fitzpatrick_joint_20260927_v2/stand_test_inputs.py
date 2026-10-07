"""Synthetic-only stand input fixture. Never imported by live mode."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'baseline_pf_pp_20260908'))
import json
import time
import math
import os
import numpy as np
import range_libc
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan,Joy
from std_msgs.msg import Bool,String
from geometry_msgs.msg import PoseWithCovarianceStamped
from stand_map_io_optimized import ROOT,load_map
from f1tenth_safety_migration.route_mpc_core import load_open_route


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='83':raise RuntimeError('Synthetic inputs only in offline domain 83')
    rclpy.init();node=Node('stand_synthetic_inputs');held=[False];sent=[False]
    node.create_subscription(Bool,'/stand_pf/test_hold',lambda m:held.__setitem__(0,m.data),1)
    op=node.create_publisher(Odometry,'/baseline/odom',1);sp=node.create_publisher(LaserScan,'/baseline/raw_scan',1)
    jp=node.create_publisher(Joy,'/baseline/joy',1);ip=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',1)
    truth=load_open_route(str(ROOT.parent/'routes/Fitzpatrick_fixed_start_arc_straight_v2.csv'))[0,:3]
    grid=load_map();angles=np.linspace(-2.35,2.35,1081).astype(np.float32)
    caster=range_libc.PyRayMarching(range_libc.PyOMap(grid),int(10/grid.info.resolution))
    q=np.zeros((1081,3),dtype=np.float32);q[:,0]=truth[0]+.27*math.cos(truth[2]);q[:,1]=truth[1]+.27*math.sin(truth[2]);q[:,2]=truth[2]+angles
    ranges=np.zeros(1081,dtype=np.float32);caster.calc_range_many(q,ranges)
    hp=node.create_publisher(String,'/stand_pf/health',1)
    fault=['']
    node.create_subscription(String,'/stand_pf/inject_health_fault',lambda m:fault.__setitem__(0,m.data),1)
    def tick():
        stamp=node.get_clock().now().to_msg()
        ros=stamp.sec+stamp.nanosec*1.e-9
        if fault[0]!='health_stale':
            hp.publish(String(data=json.dumps({'erpm':0.,'fault':0,'voltage':12.,'motor_current':70. if fault[0]=='health_overcurrent' else 40. if fault[0]=='health_stall' else 0.,'input_current':0.,'fet_temp':65. if fault[0]=='health_overtemp' else 25.,'motor_temp':-81.9,'duty_cycle':0.,'stamp':ros,'bridge_ros':ros,'bridge_mono':time.monotonic(),'synthetic':True})))
        od=Odometry();od.header.stamp=stamp;od.header.frame_id='odom';od.child_frame_id='base_link';od.pose.pose.orientation.w=1.;op.publish(od)
        sc=LaserScan();sc.header.stamp=stamp;sc.header.frame_id='laser';sc.angle_min=float(angles[0]);sc.angle_max=float(angles[-1]);sc.angle_increment=float(angles[1]-angles[0]);sc.range_min=.05;sc.range_max=10.;sc.ranges=np.clip(ranges,.05,10.).tolist();sp.publish(sc)
        j=Joy();j.buttons=[0]*13;j.buttons[4]=int(held[0]);jp.publish(j)
        # Both supervisor and PF must be discovered; supervisor alone is not
        # evidence the PF can receive this one-shot, volatile initial pose.
        if not sent[0] and ip.get_subscription_count()>=2:
            p=PoseWithCovarianceStamped();p.header.frame_id='map';p.header.stamp=stamp
            p.pose.pose.position.x=float(truth[0]);p.pose.pose.position.y=float(truth[1]);p.pose.pose.orientation.z=math.sin(truth[2]/2);p.pose.pose.orientation.w=math.cos(truth[2]/2);ip.publish(p);sent[0]=True
    node.create_timer(.05,tick)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
