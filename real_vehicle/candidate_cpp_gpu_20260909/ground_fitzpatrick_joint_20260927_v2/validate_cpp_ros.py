"""Synthetic ROS load check in domain 83. Starts NO actuator or driver node.

Stationary ground-truth pose, map-raycast 1081-beam scans and a reconstructed
box exercise the actual controller callbacks, perception, search, tracking,
watchdog and DDS. This is not a physical drive or localization validation.
"""
import argparse
import collections
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tracker',choices=['pp','pid','dynamic_mpc'],default='dynamic_mpc')
    parser.add_argument('--reference-tracker', type=Path)
    parser.add_argument('--no-box',action='store_true',help='Validate the map-only baseline before physical no-obstacle testing')
    parser.add_argument('--seconds', type=float, default=12.)
    parser.add_argument('--output', type=Path)
    args, ros_args = parser.parse_known_args()
    if os.environ.get('ROS_DOMAIN_ID') != '83' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Synthetic benchmark requires localhost domain 83')
    if args.output is None:
        parser.error('--output is required')
    import rclpy
    import range_libc
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    from stand_map_io_optimized import load_map
    from benchmark_runtime import summary

    route = ROOT.parents[1]/'routes/Fitzpatrick_fixed_start_arc_straight_v2.csv'
    profile = ROOT.parents[1]/'routes/Fitzpatrick_fixed_start_arc_straight_v2_speed_profile.csv'
    map_file = ROOT.parents[1]/'maps/Fitzpatrick.yaml'
    # These CSVs have the same geometry, with a header in the speed profile.
    pose = np.loadtxt(profile, delimiter=',', skiprows=1)[0, :3]
    grid = load_map()
    pixels = np.asarray(grid.data).reshape(grid.info.height, grid.info.width).copy()
    ox = grid.info.origin.position.x; oy = grid.info.origin.position.y
    res = grid.info.resolution
    # Box center and extent from the failed 22:09 run; lidar is synthetic.
    cx, cy, half = 2.7497, -.2355, .0786
    for iy in range(math.floor((cy-half-oy)/res), math.ceil((cy+half-oy)/res)+1):
        for ix in range(math.floor((cx-half-ox)/res), math.ceil((cx+half-ox)/res)+1):
            if not args.no_box and 0 <= iy < len(pixels) and 0 <= ix < pixels.shape[1]:
                pixels[iy, ix] = 100
    grid.data = pixels.astype(np.int8).ravel().tolist()
    angles = np.linspace(-2.35, 2.35, 1081).astype(np.float32)
    rays = np.zeros((len(angles), 3), dtype=np.float32)
    rays[:, 0] = pose[0]+.27*math.cos(pose[2])
    rays[:, 1] = pose[1]+.27*math.sin(pose[2]); rays[:, 2] = pose[2]+angles
    ranges = np.zeros(len(angles), dtype=np.float32)
    caster = range_libc.PyRayMarching(range_libc.PyOMap(grid), int(10./res))
    caster.calc_range_many(rays, ranges)
    rclpy.init(args=[])
    node = rclpy.create_node('non_actuating_controller_benchmark')
    scan_pub = node.create_publisher(LaserScan, '/baseline/scan', 1)
    odom_pub = node.create_publisher(Odometry, '/baseline/odom', 1)
    pose_pub = node.create_publisher(Odometry, '/baseline/localization_pose', 1)
    valid_pub = node.create_publisher(Bool, '/baseline/localization_valid', 1)
    enable_pub = node.create_publisher(Bool, '/baseline/enable', 1)
    records = []; enabled = [False]; fault_mode=[None]; drive_records=[]; latest_plan=[None]
    def receive(message):
        records.append(dict(receipt_mono=time.monotonic(), **json.loads(message.data)))
    node.create_subscription(String, '/baseline/controller_status', receive, 50)
    node.create_subscription(String,'/baseline/local_plan_cpp',lambda m:latest_plan.__setitem__(0,json.loads(m.data)),1)
    replay_pub=node.create_publisher(String,'/baseline/local_plan_cpp',1)
    from ackermann_msgs.msg import AckermannDriveStamped
    node.create_subscription(AckermannDriveStamped,'/baseline/drive_nominal',lambda m:drive_records.append((time.monotonic(),m.drive.speed)),50)
    def tick():
        stamp = node.get_clock().now().to_msg()
        odom = Odometry(); odom.header.stamp = stamp; odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(pose[0]); odom.pose.pose.position.y = float(pose[1])
        odom.pose.pose.orientation.z = math.sin(pose[2]/2.)
        odom.pose.pose.orientation.w = math.cos(pose[2]/2.)
        odom.twist.twist.linear.x = 1.0
        scan = LaserScan(); scan.header.stamp = stamp; scan.header.frame_id = 'laser'
        scan.angle_min = float(angles[0]); scan.angle_max = float(angles[-1])
        scan.angle_increment = float(angles[1]-angles[0]); scan.range_min = .05; scan.range_max = 10.
        scan.ranges = np.clip(ranges, .05, 10.).tolist()
        pose_pub.publish(odom)
        if fault_mode[0]=='empty_scan':scan.ranges=[]
        scan_pub.publish(scan)
        if fault_mode[0]=='invalid_odom':odom.twist.twist.linear.x=float('nan')
        if fault_mode[0]=='stale_odom':odom.header.stamp.sec-=1
        odom.header.frame_id = 'odom'; odom_pub.publish(odom)
        valid_pub.publish(Bool(data=fault_mode[0]!='localization_lost')); enable_pub.publish(Bool(data=enabled[0]))
    node.create_timer(.05, tick)
    binaries=ROOT/'install_cpp/cpp_frenet/lib/cpp_frenet'
    log=args.output.with_suffix('.log').open('w')
    processes=[]
    for role,cores in [('planner','1-2'),('controller','3')]:
        command=['taskset','-c',cores,str(binaries/('cpp_frenet_'+role)),
                 '--ros-args','-p','assets_file:='+str(ROOT/'runtime_assets.json'),
                 '-p','tracker:='+args.tracker]
        processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
    process=processes[1]
    try:
        wait_until = time.monotonic()+15.
        while time.monotonic() < wait_until:
            rclpy.spin_once(node, timeout_sec=.02)
            if records and records[-1]['valid'] and records[-1]['reason'] == 'waiting_for_operator':
                break
            if process.poll() is not None:
                raise RuntimeError('controller exited during startup; see log')
        else:
            raise RuntimeError('no valid startup plan: '+str(records[-1] if records else 'no status'))
        enabled[0] = True
        start = time.monotonic()
        while time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.02)
            if process.poll() is not None:
                raise RuntimeError('controller exited during load check')
        end = time.monotonic(); enabled[0] = False
        while time.monotonic()-end < .6:
            rclpy.spin_once(node, timeout_sec=.02)
        active = [r for r in records if start+.5 <= r['receipt_mono'] < end]
        commands = [r for r in active if r['valid'] and r['speed_command'] > 0.]
        plans = {}
        for r in active:
            if r.get('last_plan_duration_s') is not None:
                plans[r['last_plan_duration_s']] = r.get('last_plan_stages', {})
        solves = [r['tracker_diagnostics']['controller_compute_ms']/1000. for r in commands]
        bad = [r for r in active if not r['valid']]
        times = [r['receipt_mono'] for r in commands]
        release_ok = any(r['receipt_mono'] >= end and r['speed_command'] == 0.
                         and r['reason'] == 'waiting_for_operator' for r in records)
        dynamic_verified = bool(commands) and all(
            r.get('controller') == 'closed_loop_frenet_cpp_'+args.tracker
            and r.get('tracker_config', {}).get('type') == args.tracker
            and (args.tracker!='dynamic_mpc' or r['tracker_diagnostics'].get('state_dimension')==10)
            for r in commands)
        snapshots=[r for r in records if r.get("local_path_snapshot")]
        assert snapshots,"missing exact planner path snapshots"
        assert all(len(point)==3 for r in snapshots for point in r["local_path_snapshot"]),"bad path snapshot"
        fault_checks={}
        enabled[0]=True
        for fault in ('localization_lost','invalid_odom','stale_odom','empty_scan'):
            fault_mode[0]=fault;begin=time.monotonic()
            while time.monotonic()-begin<.7:rclpy.spin_once(node,timeout_sec=.01)
            zero=[v for t,v in drive_records if t>begin+.2]
            fault_checks[fault]=bool(zero) and all(v==0. for v in zero)
            fault_mode[0]=None;begin=time.monotonic()
            while time.monotonic()-begin<1.:rclpy.spin_once(node,timeout_sec=.01)
            assert records[-1]['valid'] and records[-1]['speed_command']>0.,('recovery',fault,records[-1])
        # Fresh sensors cannot hide a dead planning process.
        enabled[0]=True
        fault_start=time.monotonic()
        saved_plan=latest_plan[0]
        assert saved_plan and saved_plan["valid"]
        os.killpg(processes[0].pid,signal.SIGKILL);processes[0].wait(timeout=2)
        while time.monotonic()-fault_start<1.2:
            # Waiting notifications carry the OLD generation timestamp. They
            # must never extend the lease after the real planner is gone.
            replay=dict(saved_plan,state='WAIT_SCAN_POSE',waiting_for_scan_pose=True)
            replay_pub.publish(String(data=json.dumps(replay)))
            rclpy.spin_once(node,timeout_sec=.01)
        fault_rows=[r for r in records if r['receipt_mono']>fault_start+.6]
        planner_loss_ok=bool(fault_rows) and all(not r['valid'] and r['speed_command']==0 for r in fault_rows)
        rejection_logged = True
        if args.tracker == 'pid':
            # Planner has exited. Supply a deliberately unsafe synthetic plan
            # with fresh leases to exercise the controller's own sweep guard.
            # Domain 83 has no hardware drivers or command sink.
            begin=time.monotonic()
            while time.monotonic()-begin<.4:
                synthetic=dict(saved_plan,generated_mono=time.monotonic(),
                    plan_id=int(saved_plan['plan_id'])+1000,
                    obstacle_points=[[float(pose[0]),float(pose[1])]],
                    valid=True,reason='ok',waiting_for_scan_pose=False)
                synthetic['input_scan_stamp']=node.get_clock().now().nanoseconds*1e-9
                synthetic['input_pose_stamp']=synthetic['input_scan_stamp']
                replay_pub.publish(String(data=json.dumps(synthetic)))
                rclpy.spin_once(node,timeout_sec=.01)
            rejected=[r for r in records if r['receipt_mono']>begin and r['reason']=='predicted_obstacle_footprint']
            rejection_logged=bool(rejected) and all(
                not r['valid'] and r['speed_command']==0 and r.get('control_input_snapshot')
                and r.get('rejected_control',{}).get('diagnostics',{}).get('predicted_path')
                for r in rejected)
        enabled[0]=False
        report = dict(rejected_command_snapshot_verified=rejection_logged,path_snapshots_verified=len(snapshots),fixture='1081-beam map-only scan, fixed ground-truth pose; no drivers' if args.no_box else '1081-beam map/box scan, fixed ground-truth pose; no drivers',
                      reference_tracker=str(args.reference_tracker), seconds=args.seconds,
                      valid_commands=len(commands), invalid_reasons=dict(collections.Counter(r['reason'] for r in bad)),
                      planner=summary(list(plans)) if plans else None,
                      tracker=summary(solves) if solves else None,
                      command_intervals=summary(np.diff(times)) if len(times)>1 else None,
                      candidate_count_max=max((sum(r['candidate_reason_counts'].values()) for r in active), default=0),
                      planning_stages=list(plans.values()), release_zero_verified=release_ok,
                      controller='closed_loop_frenet_cpp_'+args.tracker, tracker_runtime_verified=dynamic_verified, planner_loss_zero_verified=planner_loss_ok, input_fault_zero_verified=fault_checks,
                      passed=len(commands)>args.seconds*10 and not bad and release_ok and dynamic_verified and planner_loss_ok and rejection_logged and all(fault_checks.values()))
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        args.output.with_suffix('.trace.json').write_text(json.dumps(records)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k != 'planning_stages'}, indent=2))
        if not report['passed']:
            raise SystemExit(1)
    finally:
        args.output.with_suffix('.trace.json').write_text(json.dumps(records)+'\n')
        enabled[0] = False
        enable_pub.publish(Bool(data=False))
        for child in reversed(processes):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=2)
        log.close(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
