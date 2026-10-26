#!/usr/bin/env python3
"""Pure synthetic scan checks. No ROS node/graph, driver or publisher."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace, MethodType
import unittest

import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/f1tenth_mpc-main/mpc/scripts'))
from opponent_ekf_tracker import OpponentEKFTracker


def scan(cx,cy):
    msg=LaserScan();msg.angle_min=-2.35;msg.angle_increment=4.7/1080
    angles=msg.angle_min+np.arange(1080)*msg.angle_increment
    dx,dy=np.cos(angles),np.sin(angles)
    with np.errstate(divide='ignore',invalid='ignore'):
        tx=np.array([(cx-.29)/dx,(cx+.29)/dx])
        ty=np.array([(cy-.155)/dy,(cy+.155)/dy])
        lo=np.maximum(tx.min(axis=0),ty.min(axis=0))
        hi=np.minimum(tx.max(axis=0),ty.max(axis=0))
    ranges=np.where((hi>=lo)&(lo>=.2),lo,np.inf)
    msg.ranges=list(map(float,ranges))
    return msg


def fixture(cx,cy):
    f=SimpleNamespace(initialized=True,last_detection_stamp=10.,static_map=None,
        lidar_fov=math.radians(190),tracked_lidar_fov=math.radians(270),
        tracked_geometry_timeout=.3,tracked_min_forward_x=-1.5,
        tracked_max_cluster_points=300,lidar_min_range=.2,lidar_max_range=8.,
        cluster_gap=.18,min_cluster_points=5,max_cluster_points=140,
        min_cluster_width=.12,max_cluster_width=.85,min_forward_x=-.5,
        target_center_offset=.29,ego_odom=Odometry(),x=np.array([[cx],[cy],[0.],[0.]]),
        P=np.eye(4)*.01,association_gate=.55,association_sigma=3.,reacquisition_gate=.9,
        missed_lidar_scans=0,reacquisition_after_missed_scans=3,reacquisition_tracks=[])
    f.ego_odom.pose.pose.orientation.w=1.
    for name in ('cluster_width','yaw_from_odom','global_to_ego_frame','ego_to_global_frame'):
        setattr(f,name,MethodType(getattr(OpponentEKFTracker,name),f))
    f.motion_confirmed_reacquisition=lambda *args:None
    return f


class SideGeometry(unittest.TestCase):
    def test_full_side_panel_is_not_discarded_by_front_face_beam_limit(self):
        msg=scan(0.,.9)
        self.assertGreater(sum(math.isfinite(x) for x in msg.ranges),140)
        f=fixture(0.,.9)
        f.lidar_fov=math.radians(270);f.tracked_lidar_fov=0.
        self.assertIsNone(OpponentEKFTracker.extract_lidar_target(f,msg,10.05)[0])
        f.lidar_fov=math.radians(190);f.tracked_lidar_fov=math.radians(270)
        found,diagnostic=OpponentEKFTracker.extract_lidar_target(f,msg,10.05)
        self.assertIsNotNone(found)
        self.assertTrue(diagnostic['established_track_geometry'])

    def test_rear_mask_only_extends_an_established_recent_identity(self):
        msg=scan(-.7,.9);f=fixture(-.7,.9)
        self.assertIsNotNone(OpponentEKFTracker.extract_lidar_target(f,msg,10.05)[0])
        f.initialized=False
        self.assertIsNone(OpponentEKFTracker.extract_lidar_target(f,msg,10.05)[0])
        f.initialized=True;f.last_detection_stamp=9.
        self.assertIsNone(OpponentEKFTracker.extract_lidar_target(f,msg,10.05)[0])


if __name__=='__main__':unittest.main()
