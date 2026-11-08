"""Actual controller regression: brief scan wait preserves cap, expired plan stops.

Synthetic ROS domain 83 only; no driver, supervisor or hardware commands.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '83'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    parser = argparse.ArgumentParser()
    parser.add_argument('--release', type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument('--expect-old-failure', action='store_true')
    args = parser.parse_args()
    root = args.release.resolve()
    assets = json.loads((root/'runtime_assets.json').read_text())
    route = assets['route']
    length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(route,route[1:]))
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    rclpy.init()
    node = rclpy.create_node('wait_scan_non_actuating_regression')
    pose_pub = node.create_publisher(Odometry, '/baseline/localization_pose', 1)
    odom_pub = node.create_publisher(Odometry, '/baseline/odom', 1)
    scan_pub = node.create_publisher(LaserScan, '/baseline/scan', 1)
    quality_pub = node.create_publisher(Bool, '/baseline/localization_valid', 1)
    enable_pub = node.create_publisher(Bool, '/baseline/enable', 1)
    plan_pub = node.create_publisher(String, '/baseline/local_plan_cpp', 1)
    records = []
    node.create_subscription(String, '/baseline/controller_status',
        lambda m: records.append(dict(receipt=time.monotonic(), **json.loads(m.data))), 100)
    log = (root/'run_logs/wait_scan_regression.log').open('w')
    child = subprocess.Popen([str(root/'install_cpp/cpp_frenet/lib/cpp_frenet/cpp_frenet_controller'),
        '--ros-args','-p','assets_file:='+str(root/'runtime_assets.json'),'-p','tracker:=dynamic_mpc'],
        stdout=log, stderr=log, start_new_session=True)
    start = time.monotonic()
    phase = 'warmup'
    phase_start = start
    plan_id = 0
    plan = None
    wait_start = None
    recovery_start = None
    expiry_start = None
    try:
        while time.monotonic()-start < 14:
            now = time.monotonic()
            stamp = node.get_clock().now().to_msg()
            source = stamp.sec+stamp.nanosec*1e-9
            pose = Odometry();pose.header.stamp=stamp;pose.header.frame_id='map';pose.child_frame_id='base_link'
            pose.pose.pose.position.x=route[0][0];pose.pose.pose.position.y=route[0][1]
            pose.pose.pose.orientation.z=math.sin(route[0][2]/2);pose.pose.pose.orientation.w=math.cos(route[0][2]/2)
            pose.twist.twist.linear.x=.7 if now-start>1.5 else 0.
            pose_pub.publish(pose)
            pose.header.frame_id='odom';odom_pub.publish(pose)
            scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='laser'
            scan.angle_min=-2.;scan.angle_max=2.;scan.angle_increment=.04;scan.range_min=.05;scan.range_max=10.;scan.ranges=[5.]*101
            scan_pub.publish(scan);quality_pub.publish(Bool(data=True));enable_pub.publish(Bool(data=now-start>1.))
            if phase in ('warmup','recovery'):
                plan_id+=1
                plan=dict(valid=True,reason='ok',frame_id='map',path=route,obstacle_points=[],
                    generated_mono=now,input_scan_stamp=source,input_pose_stamp=source,
                    plan_id=plan_id,lap_progress_m=0.,lap_length_m=length,state='FOLLOWING')
                plan_pub.publish(String(data=json.dumps(plan)))
            else:
                wait=copy.deepcopy(plan);wait.update(state='WAIT_SCAN_POSE',waiting_for_scan_pose=True)
                plan_pub.publish(String(data=json.dumps(wait)))
            until=now+.02
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.003)
            if phase=='warmup' and records and records[-1].get('valid') and records[-1].get('speed_command',0)>.95 and records[-1].get('plan_id')==plan_id:
                phase='short_wait';phase_start=wait_start=time.monotonic()
            elif phase=='short_wait' and now-phase_start>.25:
                if args.expect_old_failure:break
                phase='recovery';phase_start=recovery_start=time.monotonic()
            elif phase=='recovery' and now-phase_start>.6:
                phase='expiry';phase_start=expiry_start=time.monotonic()
            elif phase=='expiry' and now-phase_start>.85:break
            if child.poll() is not None:raise RuntimeError('controller exited')
        (root/'run_logs/wait_scan_regression_records.json').write_text(json.dumps(records)+'\n')
        assert wait_start is not None, 'did not reach pre-wait speed'
        brief=[r for r in records if wait_start+.04<r['receipt']<wait_start+.25]
        if args.expect_old_failure:
            assert any(r.get('reason')=='joint_mpc_solver_4' for r in brief), ('old failure not reproduced',[(r.get('reason'),r.get('state'),r.get('control_input_snapshot',{}).get('cap')) for r in brief])
            report=dict(passed=True,old_failure_reproduced=True,physical_motion=False)
        else:
            assert brief and all(r['valid'] and r['speed_command']>.6 for r in brief), 'brief wait interrupted control'
            assert any(r.get('waiting_for_scan_pose') for r in brief), 'wait notification was not processed'
            assert all(r.get('control_input_snapshot',{}).get('cap',2.0)==2.0 for r in brief)
            assert expiry_start is not None
            late=[r for r in records if r['receipt']>expiry_start+.65]
            assert late and all(not r['valid'] and r['speed_command']==0 for r in late), 'expired lease did not stop'
            assert any(r['reason'] in ('planning_timeout','planned_input_stale') for r in late)
            report=dict(passed=True,brief_wait_preserves_speed_cap=True,expired_lease_stops=True,
                        minimum_brief_wait_command_mps=min(r['speed_command'] for r in brief),physical_motion=False)
        (root/'run_logs/wait_scan_regression.json').write_text(json.dumps(dict(summary=report,records=records),indent=2)+'\n')
        print(json.dumps(report),flush=True)
    finally:
        enable_pub.publish(Bool(data=False))
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=3)
        log.close();node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':main()
