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
    parser.add_argument('--controller-child', action='store_true')
    parser.add_argument('--reference-tracker', type=Path)
    parser.add_argument('--seconds', type=float, default=12.)
    parser.add_argument('--output', type=Path)
    args, ros_args = parser.parse_known_args()
    if os.environ.get('ROS_DOMAIN_ID') != '83' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Synthetic benchmark requires localhost domain 83')
    if args.controller_child:
        os.sched_setaffinity(0, {1, 2, 3})
        if args.reference_tracker:
            import importlib.util
            spec = importlib.util.spec_from_file_location('local_path_tracker', args.reference_tracker)
            module = importlib.util.module_from_spec(spec)
            sys.modules['local_path_tracker'] = module
            spec.loader.exec_module(module)
        sys.argv = [sys.argv[0]]+ros_args
        from avoidance_controller import main as controller_main
        controller_main()
        return
    if args.output is None:
        parser.error('--output is required')
    import rclpy
    import range_libc
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    from stand_map_io_optimized import load_map
    from benchmark_runtime import summary

    route = ROOT.parents[1]/'routes/manual_route_20260923_172359_controller_ready.csv'
    profile = ROOT.parents[1]/'routes/manual_route_20260923_172359_controller_ready_speed_profile_v2.csv'
    map_file = ROOT.parents[1]/'maps/381_2_0.yaml'
    # These CSVs have the same geometry, with a header in the speed profile.
    pose = np.loadtxt(profile, delimiter=',', skiprows=1)[15, :3]
    grid = load_map()
    pixels = np.asarray(grid.data).reshape(grid.info.height, grid.info.width).copy()
    ox = grid.info.origin.position.x; oy = grid.info.origin.position.y
    res = grid.info.resolution
    # Box center and extent from the failed 22:09 run; lidar is synthetic.
    cx, cy, half = 2.7497, -.2355, .0786
    for iy in range(math.floor((cy-half-oy)/res), math.ceil((cy+half-oy)/res)+1):
        for ix in range(math.floor((cx-half-ox)/res), math.ceil((cx+half-ox)/res)+1):
            if 0 <= iy < len(pixels) and 0 <= ix < pixels.shape[1]:
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
    records = []; enabled = [False]
    def receive(message):
        records.append(dict(receipt_mono=time.monotonic(), **json.loads(message.data)))
    node.create_subscription(String, '/baseline/controller_status', receive, 50)
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
        pose_pub.publish(odom); scan_pub.publish(scan)
        odom.header.frame_id = 'odom'; odom_pub.publish(odom)
        valid_pub.publish(Bool(data=True)); enable_pub.publish(Bool(data=enabled[0]))
    node.create_timer(.05, tick)
    command = [sys.executable, str(Path(__file__).resolve()), '--controller-child']
    if args.reference_tracker:
        command += ['--reference-tracker', str(args.reference_tracker.resolve())]
    command += ['--ros-args', '-p', 'route_file:='+str(route), '-p', 'map_file:='+str(map_file),
                '-p', 'speed_profile_file:='+str(profile)]
    log = args.output.with_suffix('.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
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
        solves = [r['tracker_diagnostics']['mpc_solve_ms']/1000. for r in commands]
        bad = [r for r in active if not r['valid']]
        times = [r['receipt_mono'] for r in commands]
        release_ok = any(r['receipt_mono'] >= end and r['speed_command'] == 0.
                         and r['reason'] == 'waiting_for_operator' for r in records)
        dynamic_verified = bool(commands) and all(
            r.get('controller') == 'closed_loop_frenet_dynamic_mpc'
            and r.get('tracker_config', {}).get('type') == 'dynamic_mpc'
            and r.get('tracker_diagnostics', {}).get('state_dimension') == 4
            for r in commands)
        report = dict(fixture='1081-beam map/box scan, fixed ground-truth pose; no drivers',
                      reference_tracker=str(args.reference_tracker), seconds=args.seconds,
                      valid_commands=len(commands), invalid_reasons=dict(collections.Counter(r['reason'] for r in bad)),
                      planner=summary(list(plans)) if plans else None,
                      tracker=summary(solves) if solves else None,
                      command_intervals=summary(np.diff(times)) if len(times)>1 else None,
                      candidate_count_max=max((sum(r['candidate_reason_counts'].values()) for r in active), default=0),
                      planning_stages=list(plans.values()), release_zero_verified=release_ok,
                      controller='closed_loop_frenet_dynamic_mpc', dynamic_runtime_verified=dynamic_verified,
                      passed=len(commands)>args.seconds*10 and not bad and release_ok and dynamic_verified)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        args.output.with_suffix('.trace.json').write_text(json.dumps(records)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k != 'planning_stages'}, indent=2))
        if not report['passed']:
            raise SystemExit(1)
    finally:
        args.output.with_suffix('.trace.json').write_text(json.dumps(records)+'\n')
        enabled[0] = False
        enable_pub.publish(Bool(data=False))
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=2)
        log.close(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
