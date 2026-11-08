#!/usr/bin/env python3
"""Read-only PF/body-odom/scan adapter. Publishes only diagnostic input topics."""
import copy,math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

def stamp(msg):return msg.header.stamp.sec+1e-9*msg.header.stamp.nanosec
class Bridge(Node):
 def __init__(self):
  super().__init__('overtake_381_input_bridge');self.pose=None;self.odom=None
  self.pose_pub=self.create_publisher(Odometry,'/overtake_381/ego_odom',10)
  self.scan_pub=self.create_publisher(LaserScan,'/overtake_381/scan',qos_profile_sensor_data)
  self.create_subscription(Odometry,'/baseline/localization_pose',self.on_pose,10)
  self.create_subscription(Odometry,'/baseline/odom',self.on_odom,qos_profile_sensor_data)
  self.create_subscription(LaserScan,'/baseline/scan',self.on_scan,qos_profile_sensor_data)
 def on_odom(self,msg):self.odom=msg
 def on_pose(self,msg):self.pose=msg
 def on_scan(self,msg):
  now=self.get_clock().now().nanoseconds/1e9
  if self.pose is None or self.odom is None:return
  if any(not -.02<=now-stamp(x)<=.25 for x in [msg,self.pose,self.odom]):return
  if self.pose.header.frame_id!='map' or self.odom.child_frame_id not in ('base_link','ego_racecar/base_link'):return
  if abs(stamp(msg)-stamp(self.pose))>.15 or abs(stamp(msg)-stamp(self.odom))>.15:return
  q=self.pose.pose.pose.orientation;t=self.odom.twist.twist;p=self.pose.pose.pose.position
  values=[p.x,p.y,q.x,q.y,q.z,q.w,t.linear.x,t.linear.y,t.angular.z]
  if not all(math.isfinite(x) for x in values):return
  if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.01:return
  if not -.02<=t.linear.x<=2.4 or abs(t.linear.y)>2.5 or abs(t.angular.z)>8:return
  if not msg.ranges or not math.isfinite(msg.angle_increment) or msg.angle_increment<=0:return
  out=copy.deepcopy(self.pose);out.child_frame_id='base_link';out.twist=copy.deepcopy(self.odom.twist)
  # Keep the oldest source stamp; never refresh an old localization sample.
  out.header.stamp=min([self.pose,self.odom],key=stamp).header.stamp
  self.pose_pub.publish(out);self.scan_pub.publish(msg)
def main():
 rclpy.init();node=Bridge()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
