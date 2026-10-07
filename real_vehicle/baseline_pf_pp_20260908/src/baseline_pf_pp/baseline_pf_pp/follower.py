"""Direct localized-pose Pure Pursuit, nominal-only and disabled by default."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool,String
from ackermann_msgs.msg import AckermannDriveStamped
from f1tenth_safety_migration.command_encoding import encode_route_command
from f1tenth_safety_migration.route_mpc_core import RouteMpcConfig,load_open_route
from .core import PurePursuitCore,SourceFreshness,quaternion_yaw,checked_covariance,checked_pose_timeout
from .safe_pf import isolated_domain,latest_sensor_qos


class Follower(Node):
    def __init__(self):
        isolated_domain();super().__init__('baseline_pure_pursuit',start_parameter_services=False)
        self.declare_parameter('route_file','')
        self.declare_parameter('pose_timeout_sec',.20)
        pose_timeout=checked_pose_timeout(self.get_parameter('pose_timeout_sec').value)
        self.core=PurePursuitCore(load_open_route(self.get_parameter('route_file').value),RouteMpcConfig())
        self.pose_clock=SourceFreshness(timeout=pose_timeout)
        self.scan_clock,self.odom_clock=SourceFreshness(),SourceFreshness()
        self.pose=None;self.speed=0.;self.clearance=0.;self.scan_valid=False
        self.odom_valid=False
        self.enabled=False;self.enable_time=None;self.pose_valid=False
        self.quality=False;self.quality_time=None;self.last_reason=None
        self.status_sequence=0;self.previous_tick=None
        self.nominal=self.create_publisher(AckermannDriveStamped,'/baseline/drive_nominal',1)
        self.valid=self.create_publisher(Bool,'/baseline/planner_valid',1)
        self.status=self.create_publisher(String,'/baseline/controller_status',1)
        self.create_subscription(Odometry,'/baseline/localization_pose',self.pose_cb,1)
        self.create_subscription(Bool,'/baseline/localization_valid',self.quality_cb,1)
        self.create_subscription(LaserScan,'/baseline/scan',self.scan_cb,latest_sensor_qos())
        self.create_subscription(Odometry,'/baseline/odom',self.odom_cb,1)
        self.create_subscription(Bool,'/baseline/enable',self.enable_cb,1)
        self.create_timer(.05,self.tick)

    def accept(self, clock, msg):
        return clock.accept(msg.header.stamp.sec+1e-9*msg.header.stamp.nanosec,
                            self.get_clock().now().nanoseconds*1e-9,time.monotonic())

    def pose_cb(self,msg):
        try:
            if msg.header.frame_id!='map' or msg.child_frame_id!='base_link': raise ValueError()
            yaw=quaternion_yaw(msg.pose.pose.orientation);checked_covariance(msg.pose.covariance)
            x,y=msg.pose.pose.position.x,msg.pose.pose.position.y
            if not all(math.isfinite(v) for v in (x,y)):raise ValueError()
            if self.accept(self.pose_clock,msg):self.pose=(x,y,yaw);self.pose_valid=True
        except ValueError:self.pose_valid=False

    def scan_cb(self,msg):
        if (msg.header.frame_id!='laser' or not math.isfinite(msg.angle_increment)
                or msg.angle_increment<=0. or not math.isfinite(msg.angle_min)
                or not 0.<=msg.range_min<msg.range_max or not math.isfinite(msg.range_max)):
            self.scan_valid=False;return
        values=[v for i,v in enumerate(msg.ranges) if abs(msg.angle_min+i*msg.angle_increment)<=.35
                and math.isfinite(v) and msg.range_min<=v<=msg.range_max]
        if not values:self.scan_valid=False;return
        if self.accept(self.scan_clock,msg):self.clearance=min(values);self.scan_valid=True

    def odom_cb(self,msg):
        try:
            if msg.header.frame_id!='odom' or msg.child_frame_id!='base_link':raise ValueError()
            quaternion_yaw(msg.pose.pose.orientation)
            if not all(math.isfinite(v) for v in (msg.twist.twist.linear.x,
                    msg.pose.pose.position.x,msg.pose.pose.position.y)):raise ValueError()
            if self.accept(self.odom_clock,msg):
                self.speed=abs(msg.twist.twist.linear.x);self.odom_valid=True
        except ValueError:self.odom_valid=False

    def enable_cb(self,msg):
        self.enabled=msg.data;self.enable_time=time.monotonic()
        if not msg.data:self.core.last_speed_command=0.

    def quality_cb(self,msg):
        self.quality=msg.data;self.quality_time=time.monotonic()

    def tick(self):
        now=time.monotonic();ros_now=self.get_clock().now().nanoseconds*1e-9
        gap=None if self.previous_tick is None else now-self.previous_tick
        self.previous_tick=now;self.status_sequence+=1
        reason='';speed=angle=0.;valid=False
        for key,clock in [('pose',self.pose_clock),('scan',self.scan_clock),('odom',self.odom_clock)]:
            if not clock.fresh(ros_now,now):reason=key+'_stale';break
        if not reason and (not self.pose_valid or not self.quality or self.quality_time is None
                           or now-self.quality_time>.2):reason='localization_invalid'
        if not reason and not self.scan_valid:reason='scan_invalid'
        if not reason and not self.odom_valid:reason='odom_invalid'
        if not reason and self.clearance<.45:reason='front_obstacle'
        if not reason:
            decision=self.core.step(*self.pose,self.speed)
            valid=decision.valid;reason=decision.reason
            speed,angle=decision.speed,decision.steering
            if valid and (not self.enabled or self.enable_time is None or now-self.enable_time>.15):
                speed=angle=0.;reason='waiting_for_operator';self.core.last_speed_command=0.
        if not valid:
            speed=angle=0.;self.core.last_speed_command=0.
        try:speed,angle=encode_route_command(speed,angle,.5,.27)
        except ValueError:speed=angle=0.;valid=False;reason='invalid_nominal_command'
        msg=AckermannDriveStamped();msg.header.stamp=self.get_clock().now().to_msg()
        msg.header.frame_id='base_link';msg.drive.speed=speed;msg.drive.steering_angle=angle
        self.nominal.publish(msg);self.valid.publish(Bool(data=valid))
        input_age={key:{'source_age_s':None if clock.stamp is None else ros_now-clock.stamp,
                        'receipt_age_s':None if clock.received is None else now-clock.received,
                        'timeout_s':clock.timeout}
                   for key,clock in [('pose',self.pose_clock),('scan',self.scan_clock),('odom',self.odom_clock)]}
        self.status.publish(String(data=json.dumps({'valid':valid,'reason':reason,
            'controller':'pure_pursuit','speed_command':speed,'steering_command':angle,
            'progress_index':self.core.progress_index,'complete':self.core.complete,
            'input_age':input_age,'runtime_timing':{'sequence':self.status_sequence,
                'tick_start_mono':now,'tick_gap_s':gap,'publish_mono':time.monotonic()}})))
        if reason!=self.last_reason:
            self.get_logger().info('baseline state: '+reason+' input_age='+json.dumps(input_age));self.last_reason=reason


def main(args=None):
    isolated_domain();rclpy.init(args=args);node=Follower()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
