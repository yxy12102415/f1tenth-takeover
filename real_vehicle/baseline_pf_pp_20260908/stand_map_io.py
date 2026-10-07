"""Map service and 5 Hz display only. No scan relay or vehicle outputs."""
import math
from collections import OrderedDict
from pathlib import Path
import struct
import time
import numpy as np
import yaml
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid,Odometry,Path as RoutePath
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan,PointCloud2,PointField
from baseline_pf_pp.safe_pf import latest_sensor_qos
from baseline_pf_pp.core import quaternion_yaw
from f1tenth_safety_migration.route_mpc_core import load_open_route

ROOT=Path(__file__).resolve().parent


def load_map():
    ws=ROOT.parent;meta=yaml.safe_load((ws/'maps/ps4_slam_20260904_163930.yaml').read_text())
    path=Path(meta['image'])
    if not path.is_absolute():path=ws/'maps'/path
    pixels=np.asarray(Image.open(path).convert('L'))[::-1]
    occupancy=pixels/255. if meta.get('negate',0) else 1.-pixels/255.
    grid=OccupancyGrid();grid.header.frame_id='map'
    grid.info.width=pixels.shape[1];grid.info.height=pixels.shape[0];grid.info.resolution=float(meta['resolution'])
    grid.info.origin.position.x=float(meta['origin'][0]);grid.info.origin.position.y=float(meta['origin'][1])
    grid.info.origin.orientation.z=math.sin(meta['origin'][2]/2);grid.info.origin.orientation.w=math.cos(meta['origin'][2]/2)
    grid.data=np.where(occupancy>meta['occupied_thresh'],100,np.where(occupancy<meta['free_thresh'],0,-1)).astype(np.int8).ravel().tolist()
    return grid


class MapIO(Node):
    def __init__(self):
        super().__init__('stand_map_io',start_parameter_services=False);self.grid=load_map();self.scans=OrderedDict();self.last_visual=None;self.last_pose=0.
        self.create_service(GetMap,'/map_server/map',self.get_map)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_out=self.create_publisher(OccupancyGrid,'/baseline/map',latched);self.map_out.publish(self.grid)
        self.pose_out=self.create_publisher(PoseStamped,'/baseline/visual_pose',1)
        self.cloud_out=self.create_publisher(PointCloud2,'/baseline/visual_scan',1)
        self.route_out=self.create_publisher(RoutePath,'/baseline/route',latched)
        route=RoutePath();route.header.frame_id='map'
        for x,y,yaw in load_open_route(str(ROOT.parent/'routes/manual_route_20260904_215501_controller_ready.csv')):
            p=PoseStamped();p.header.frame_id='map';p.pose.position.x=float(x);p.pose.position.y=float(y)
            p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2);route.poses.append(p)
        self.route_out.publish(route)
        self.create_subscription(LaserScan,'/baseline/scan',self.scan,latest_sensor_qos())
        self.create_subscription(Odometry,'/baseline/localization_pose',self.pose,1)
        self.create_timer(.10,self.clear_stale)

    def get_map(self,request,response):response.map=self.grid;return response

    @staticmethod
    def key(msg):return (msg.header.stamp.sec,msg.header.stamp.nanosec)

    def scan(self,msg):
        self.scans[self.key(msg)]=msg
        while len(self.scans)>20:self.scans.popitem(last=False)

    def pose(self,msg):
        scan=self.scans.get(self.key(msg))
        if scan is None:return
        self.last_pose=time.monotonic()
        if self.last_visual is not None and self.last_pose-self.last_visual<.2:return
        self.last_visual=self.last_pose
        pose=PoseStamped();pose.header=msg.header;pose.pose=msg.pose.pose;self.pose_out.publish(pose)
        yaw=quaternion_yaw(msg.pose.pose.orientation)
        x=msg.pose.pose.position.x+.27*math.cos(yaw);y=msg.pose.pose.position.y+.27*math.sin(yaw)
        points=[]
        for i,r in enumerate(scan.ranges):
            if math.isfinite(r) and scan.range_min<=r<scan.range_max:
                a=yaw+scan.angle_min+i*scan.angle_increment
                points.append((x+r*math.cos(a),y+r*math.sin(a),0.))
        cloud=self.cloud(points);cloud.header=msg.header;self.cloud_out.publish(cloud)

    def cloud(self,points):
        msg=PointCloud2();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
        msg.height=1;msg.width=len(points);msg.point_step=12;msg.row_step=12*len(points);msg.is_dense=True
        msg.fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
        msg.data=b''.join(struct.pack('<fff',*p) for p in points);return msg

    def clear_stale(self):
        if time.monotonic()-self.last_pose>.20:self.cloud_out.publish(self.cloud([]))


def main():
    rclpy.init();node=MapIO()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
