"""Run actual C++ controller with test-only compute delay, isolated domain 83."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parent

def run_case(root, case, expect_old=False):
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    assets = json.loads((root/'runtime_assets.json').read_text())
    route = assets['route']
    length = sum(math.hypot(a[0]-b[0],a[1]-b[1]) for a,b in zip(route,route[1:]))
    rclpy.init(); node = rclpy.create_node('compute_delay_regression')
    pose_pub = node.create_publisher(Odometry,'/baseline/localization_pose',1)
    odom_pub = node.create_publisher(Odometry,'/baseline/odom',1)
    scan_pub = node.create_publisher(LaserScan,'/baseline/scan',1)
    quality_pub = node.create_publisher(Bool,'/baseline/localization_valid',1)
    enable_pub = node.create_publisher(Bool,'/baseline/enable',1)
    plan_pub = node.create_publisher(String,'/baseline/local_plan_cpp',1)
    records = []
    node.create_subscription(String,'/baseline/controller_status',
        lambda m: records.append(dict(receipt=time.monotonic(),**json.loads(m.data))),100)
    fd, path = tempfile.mkstemp(prefix='mpc-delay-',suffix='.txt');os.close(fd)
    control = Path(path); control.write_text('0')
    env = os.environ.copy()
    env['LD_PRELOAD'] = str(ROOT/'bin/libtest_compute_delay.so')
    env['TEST_COMPUTE_DELAY_FILE'] = path
    name = ('old_' if expect_old else '')+case
    log = (ROOT/'run_logs'/('compute_timing_'+name+'.log')).open('w')
    proc = subprocess.Popen(['taskset','-c','3',str(root/'install_cpp/cpp_frenet/lib/cpp_frenet/cpp_frenet_controller'),
        '--ros-args','-p','assets_file:='+str(root/'runtime_assets.json'),'-p','tracker:=dynamic_mpc'],
        env=env,stdout=log,stderr=log,start_new_session=True)
    start=time.monotonic(); injected=None; recovered=None; phase='warmup';plan_id=0
    try:
        while time.monotonic()-start<16:
            now=time.monotonic();stamp=node.get_clock().now().to_msg()
            source=stamp.sec+stamp.nanosec*1e-9
            msg=Odometry();msg.header.stamp=stamp;msg.header.frame_id='map';msg.child_frame_id='base_link'
            msg.pose.pose.position.x=route[0][0];msg.pose.pose.position.y=route[0][1]
            msg.pose.pose.orientation.z=math.sin(route[0][2]/2);msg.pose.pose.orientation.w=math.cos(route[0][2]/2)
            msg.twist.twist.linear.x=.7 if now-start>1.5 else 0.
            pose_pub.publish(msg);msg.header.frame_id='odom';odom_pub.publish(msg)
            scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='laser'
            scan.angle_min=-2.;scan.angle_max=2.;scan.angle_increment=.04
            scan.range_min=.05;scan.range_max=10.;scan.ranges=[5.]*101
            scan_pub.publish(scan);quality_pub.publish(Bool(data=True));enable_pub.publish(Bool(data=now-start>1.))
            plan_id+=1
            plan=dict(valid=True,reason='ok',frame_id='map',path=route,obstacle_points=[],
                generated_mono=now,input_scan_stamp=source,input_pose_stamp=source,plan_id=plan_id,
                lap_progress_m=0.,lap_length_m=length,state='FOLLOWING')
            plan_pub.publish(String(data=json.dumps(plan)))
            until=now+.01
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.002)
            if phase=='warmup' and records and records[-1].get('valid') and records[-1].get('speed_command',0)>.95:
                control.write_text('90' if case=='hard' else '45')
                injected=time.monotonic();phase='delay'
            elif phase=='delay':
                after=[r for r in records if r['receipt']>injected]
                if case=='soft' and not expect_old:
                    soft=[r for r in after if (r.get('tracker_diagnostics') or {}).get('compute_soft_budget_missed')]
                    if soft:
                        control.write_text('0');recovered=time.monotonic();phase='recovery'
                elif any(r.get('reason') in ('joint_control_deadline','joint_control_overload') for r in after):
                    break
            elif phase=='recovery' and now-recovered>.7:
                break
            if proc.poll() is not None:raise RuntimeError('controller exited')
        assert injected is not None,'did not reach driving speed'
        after=[r for r in records if r['receipt']>injected]
        if expect_old or case=='hard':
            failures=[r for r in after if r.get('reason')=='joint_control_deadline']
            assert failures and all(not r['valid'] and r['speed_command']==0 for r in failures)
            assert failures[0]['rejected_control']['diagnostics']['mpc_solver_status']==0
        elif case=='sustained':
            failures=[r for r in after if r.get('reason')=='joint_control_overload']
            assert failures and failures[0]['speed_command']==0 and not failures[0]['valid']
            assert failures[0]['rejected_control']['diagnostics']['compute_consecutive_soft_misses']==3
            assert len([r for r in after if r.get('valid') and (r.get('tracker_diagnostics') or {}).get('compute_soft_budget_missed')])>=2
        else:
            assert recovered is not None,'no accepted soft miss'
            assert after and all(r.get('valid') and r['speed_command']>.8 for r in after),'soft delay interrupted control'
            late=[r for r in after if r['receipt']>recovered+.2]
            assert late and any(r['tracker_diagnostics']['compute_consecutive_soft_misses']==0 for r in late)
            assert all(r['tracker_diagnostics']['speed_phase']=='driving' for r in after),'state reset after soft miss'
        report=dict(case=name,passed=True,physical_motion=False,
                    max_observed_compute_ms=max((r.get('tracker_diagnostics') or r.get('rejected_control',{}).get('diagnostics') or {}).get('controller_compute_ms',0) for r in after),
                    reasons=sorted(set(r.get('reason','') for r in after)))
        print(json.dumps(report),flush=True)
        return report
    finally:
        (ROOT/'run_logs'/('compute_timing_'+name+'.json')).write_text(json.dumps(records)+'\n')
        enable_pub.publish(Bool(data=False))
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        control.unlink();log.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    assert os.environ.get('ROS_DOMAIN_ID')=='83' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    parser=argparse.ArgumentParser();parser.add_argument('--old-release',type=Path);args=parser.parse_args()
    reports=[]
    if args.old_release:reports.append(run_case(args.old_release,'soft',True))
    for case in ('soft','hard','sustained'):reports.append(run_case(ROOT,case))
    (ROOT/'run_logs/compute_timing_summary.json').write_text(json.dumps(reports,indent=2)+'\n')
