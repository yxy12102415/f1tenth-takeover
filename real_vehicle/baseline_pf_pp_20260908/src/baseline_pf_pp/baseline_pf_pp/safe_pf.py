"""Interface adapter around the vendored MIT-RACECAR/F1TENTH MCL core.

Deliberately restricted to nonzero ROS domains during baseline validation.
No TF broadcast: explicit map/base pose avoids competing TF parents and does
not treat repeated broadcasts as new localization observations.
"""
import json
import math
import os
import time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from particle_filter.particle_filter import ParticleFiler
from .core import (SourceFreshness, quaternion_yaw, particle_estimate,
                   laser_to_base, covariance36, checked_covariance, wrap)
from .runtime_budget import freeze_remote_parameters


def isolated_domain():
    if int(os.environ.get('ROS_DOMAIN_ID','0')) == 0:
        raise RuntimeError('Baseline is offline/sensor-only: nonzero ROS_DOMAIN_ID required')


def latest_sensor_qos():
    # SensorDataQoS defaults to depth 5: queued old scans can consume the entire
    # source-age budget after a compute/scheduling stall. Keep only the latest.
    return QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


class SafeParticleFilter(ParticleFiler):
    def __init__(self):
        isolated_domain()
        super().__init__()
        freeze_remote_parameters(self)
        self.declare_parameter('laser_offset_x', .27)
        self.declare_parameter('initial_xy_std', .15)
        self.declare_parameter('initial_yaw_std', .15)
        self.offset = float(self.get_parameter('laser_offset_x').value)
        self.scan_clock, self.odom_clock = SourceFreshness(), SourceFreshness()
        self.last_consumed_pose = None
        self.current_laser_odom = None
        self.odom_input_valid = False
        self.initialized_by_operator = False
        self.good = False
        self.reason = 'initial_pose_required'
        self.pose_count = 0
        self.compute_ms = 0.
        self.pipeline_timing = {}
        self.pose_out = self.create_publisher(Odometry, '/baseline/localization_pose', 1)
        self.quality_out = self.create_publisher(Bool, '/baseline/localization_valid', 1)
        self.status_out = self.create_publisher(String, '/baseline/localization_status', 1)
        # Legacy scan subscription used reliable QoS; the real sensor uses sensor-data QoS.
        self.destroy_subscription(self.laser_sub)
        self.laser_sub = self.create_subscription(LaserScan,
            self.get_parameter('scan_topic').value, self.lidarCB, latest_sensor_qos())
        self.create_timer(.05, self.health)

    def now_pair(self):
        return self.get_clock().now().nanoseconds*1e-9, time.monotonic()

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec+1e-9*msg.header.stamp.nanosec

    def fault(self, reason):
        self.good, self.reason = False, reason

    def health(self):
        ros_now, now = self.now_pair()
        if not self.scan_clock.fresh(ros_now,now):
            self.fault('scan_stale')
        elif not self.odom_clock.fresh(ros_now,now):
            self.fault('odom_stale')
        elif not self.odom_input_valid:
            self.fault('invalid_odom')
        self.quality_out.publish(Bool(data=self.good))
        self.status_out.publish(String(data=json.dumps({
            'valid':self.good, 'reason':self.reason, 'pose_count':self.pose_count,
            'compute_ms':self.compute_ms, 'pose_frame':'map/base_link',
            'scan_stamp':self.scan_clock.stamp, 'odom_stamp':self.odom_clock.stamp,
            'pipeline_timing':self.pipeline_timing})))

    def clicked_pose(self, msg):
        # Do not accept the legacy global-reset clicked_point shortcut.
        if not hasattr(msg, 'pose') or msg.header.frame_id.lstrip('/') != 'map':
            self.fault('invalid_initial_pose'); return
        pose = msg.pose.pose
        try:
            yaw = quaternion_yaw(pose.orientation)
            if not np.isfinite([pose.position.x,pose.position.y]).all():
                raise ValueError('nonfinite initial pose')
            # RViz initial pose represents base_link; MCL particles represent laser.
            xy_std = float(self.get_parameter('initial_xy_std').value)
            yaw_std = float(self.get_parameter('initial_yaw_std').value)
            if not np.isfinite([xy_std,yaw_std]).all() or min(xy_std,yaw_std) < 0.:
                raise ValueError('initial dispersion')
            self.particles[:,0] = pose.position.x+self.offset*math.cos(yaw)+np.random.normal(0.,xy_std,self.MAX_PARTICLES)
            self.particles[:,1] = pose.position.y+self.offset*math.sin(yaw)+np.random.normal(0.,xy_std,self.MAX_PARTICLES)
            self.particles[:,2] = yaw+np.random.normal(0.,yaw_std,self.MAX_PARTICLES)
            self.weights[:] = 1./self.MAX_PARTICLES
            self.last_consumed_pose = None
            self.initialized_by_operator = True
            self.fault('waiting_for_fresh_measurement')
        except ValueError:
            self.fault('invalid_initial_pose')

    def odomCB(self, msg):
        try:
            if msg.header.frame_id.lstrip('/') != 'odom' or msg.child_frame_id.lstrip('/') != 'base_link':
                raise ValueError('odom_frame')
            yaw = quaternion_yaw(msg.pose.pose.orientation)
            x,y = msg.pose.pose.position.x,msg.pose.pose.position.y
            speed = msg.twist.twist.linear.x
            if not np.isfinite([x,y,speed]).all(): raise ValueError('odom_nonfinite')
            ros_now,now = self.now_pair()
            if not self.odom_clock.accept(self.stamp(msg),ros_now,now): return
            self.current_laser_odom = np.array([x+self.offset*math.cos(yaw),
                y+self.offset*math.sin(yaw),yaw])
            self.current_speed = speed
            self.odom_input_valid = True
        except ValueError as exc:
            self.odom_input_valid = False
            self.fault(str(exc))

    def lidarCB(self, msg):
        ros_now,now = self.now_pair()
        self.pipeline_timing={'source_stamp':self.stamp(msg),'pf_receive_ros':ros_now,
                              'pf_input_age_s':ros_now-self.stamp(msg)}
        if msg.header.frame_id.lstrip('/') != 'laser':
            self.fault('scan_frame'); return
        meta = [msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max]
        ranges = np.asarray(msg.ranges,dtype=float)
        if (not np.isfinite(meta).all() or msg.angle_increment<=0. or
                not 0.<=msg.range_min<msg.range_max or len(ranges)<3 or
                np.isnan(ranges).any() or np.isneginf(ranges).any()):
            self.fault('invalid_scan'); return
        if not self.scan_clock.accept(self.stamp(msg),ros_now,now): return
        angles = msg.angle_min+np.arange(len(ranges))*msg.angle_increment
        down = angles[::self.ANGLE_STEP].astype(np.float32)
        if self.downsampled_angles is None or not np.array_equal(down,self.downsampled_angles):
            self.first_sensor_update = True
        self.downsampled_angles = down
        observation = np.minimum(ranges[::self.ANGLE_STEP], min(msg.range_max,self.MAX_RANGE_METERS)).astype(np.float32)
        if np.any(observation<msg.range_min): self.fault('invalid_scan_range'); return
        if not self.initialized_by_operator:
            self.fault('initial_pose_required'); return
        if not self.odom_input_valid or not self.odom_clock.fresh(ros_now,now) or self.current_laser_odom is None:
            self.fault('odom_stale'); return
        current = self.current_laser_odom.copy()
        previous = current if self.last_consumed_pose is None else self.last_consumed_pose
        dx,dy = current[:2]-previous[:2]
        c,s = math.cos(previous[2]),math.sin(previous[2])
        action = np.array([c*dx+s*dy,-s*dx+c*dy,wrap(current[2]-previous[2])])
        self.last_consumed_pose = current
        started=time.monotonic()
        try:
            self.MCL(action,observation)
            laser_pose,laser_cov = particle_estimate(self.particles,self.weights)
            pose,cov = laser_to_base(laser_pose,laser_cov,self.offset)
            cov36 = covariance36(cov)
            checked_covariance(cov36)
            self.compute_ms = (time.monotonic()-started)*1000.
            ros_now,now=self.now_pair()
            self.pipeline_timing.update(pf_output_check_ros=ros_now,
                                        pf_output_age_s=ros_now-self.stamp(msg),compute_ms=self.compute_ms)
            if not self.scan_clock.fresh(ros_now,now) or not self.odom_clock.fresh(ros_now,now):
                self.fault('localization_compute_stale'); return
            out=Odometry();out.header.stamp=msg.header.stamp
            out.header.frame_id='map';out.child_frame_id='base_link'
            out.pose.pose.position.x=float(pose[0]);out.pose.pose.position.y=float(pose[1])
            out.pose.pose.orientation.z=math.sin(pose[2]/2)
            out.pose.pose.orientation.w=math.cos(pose[2]/2)
            out.pose.covariance=cov36;out.twist.twist.linear.x=float(self.current_speed)
            self.pose_out.publish(out)
            self.pose_count+=1
            self.good,self.reason=True,''
        except (ValueError,RuntimeError,FloatingPointError,ZeroDivisionError) as exc:
            self.fault('localization_invalid:'+str(exc))
        self.health()

    def update(self):
        # Legacy odom-driven repeated-scan processing is intentionally disabled.
        return


def main(args=None):
    isolated_domain();rclpy.init(args=args)
    node=SafeParticleFilter()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
