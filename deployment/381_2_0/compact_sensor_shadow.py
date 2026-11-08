#!/usr/bin/env python3
"""Stationary physical LiDAR/localization/planner only. NO VESC/controller.

Zero odometry here is explicitly synthetic and requires the F1 to stay still.
This does not validate real odometry, the motor guard, or physical motion.
"""
import argparse,json,math,os,signal,subprocess,sys,time
from pathlib import Path
D=Path(__file__).resolve().parent
WS=Path('/home/f1tenth/f1tenth_real_migration_ws_20260829')
ROOT=WS/'candidate_cpp_gpu_20260909'
RELEASE=Path(os.environ.get('COMPACT_RELEASE',str(ROOT/'ground_381_2_0_compact_overtake_20261107_v1')))
def main():
    global RELEASE
    if os.environ.get('ROS_DOMAIN_ID')!='83' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise RuntimeError('isolated domain required')
    ap=argparse.ArgumentParser();ap.add_argument('--duration',type=float,default=45.);ap.add_argument('--release',type=Path,default=RELEASE);args=ap.parse_args()
    RELEASE=args.release.resolve()
    sys.path.insert(0,str(WS/'baseline_pf_pp_20260908'))
    from session_guard import acquire
    from sensor_only import conflicting_processes
    if conflicting_processes():raise RuntimeError('existing hardware session')
    lock=acquire()
    import rclpy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_msgs.msg import String,Bool
    rclpy.init();node=rclpy.create_node('compact_stationary_sensor_shadow')
    odom=node.create_publisher(Odometry,'/baseline/odom',1);enable=node.create_publisher(Bool,'/baseline/enable',1)
    seed_pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
    states={};at={};rows=[]
    def status(key,msg):
        states[key]=json.loads(msg.data);at[key]=time.monotonic()
        rows.append(dict(kind=key,receipt=at[key],data=states[key]))
    def pose(msg):
        p=msg.pose.pose;states['pose']=[p.position.x,p.position.y,2*math.atan2(p.orientation.z,p.orientation.w)];at['pose']=time.monotonic()
    node.create_subscription(String,'/baseline/local_plan_cpp',lambda m:status('plan',m),20)
    node.create_subscription(String,'/baseline/localization_status',lambda m:status('pf',m),20)
    node.create_subscription(Odometry,'/baseline/localization_pose',pose,10)
    def heartbeat():
        msg=Odometry();msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id='odom';msg.child_frame_id='base_link';msg.pose.pose.orientation.w=1.;odom.publish(msg);enable.publish(Bool(data=False))
    node.create_timer(.02,heartbeat)
    out=D/'reports'/('compact_sensor_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir()
    children=[];handles=[];seeded=False;passed=False;failure=None;stable=None
    def spawn(name,cmd):
        h=(out/(name+'.log')).open('w');handles.append(h);env=os.environ.copy()
        for k in ['OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS']:env[k]='1'
        children.append(subprocess.Popen(cmd,stdout=h,stderr=h,env=env,start_new_session=True))
    def stop(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,stop)
    try:
        spawn('map',[sys.executable,str(RELEASE/'stand_map_io_optimized.py')])
        spawn('relay',[sys.executable,str(WS/'baseline_pf_pp_20260908/stand_scan_relay.py')])
        spawn('pf',['taskset','-c','4-5',sys.executable,str(ROOT/'integration_v1/gpu_pf_zero.py'),'--ros-args','--params-file',str(ROOT/'config/pf_gpu_offline.yaml')])
        spawn('lidar',['/opt/ros/foxy/lib/urg_node/urg_node_driver','--ros-args','-r','scan:=/baseline/raw_scan','-p','ip_address:=192.168.0.10','-p','laser_frame_id:=laser','-p','calibrate_time:=false'])
        spawn('planner',[str(RELEASE/'install_cpp/cpp_frenet/lib/cpp_frenet/cpp_frenet_planner'),'--ros-args','-p','assets_file:='+str(RELEASE/'runtime_assets.json'),'-p','tracker:=dynamic_mpc'])
        print('Stationary LiDAR + PF + compact planner. No motor driver or drive publisher. '+str(out),flush=True)
        start=time.monotonic();printed=start
        while time.monotonic()-start<args.duration:
            rclpy.spin_once(node,timeout_sec=.01);now=time.monotonic()
            if any(p.poll() is not None for p in children):raise RuntimeError('sensor_component_exited')
            if node.count_publishers('/baseline/drive_nominal') or node.count_publishers('/stand_pf/motor_speed'):raise RuntimeError('unexpected_motion_publisher')
            if not seeded and node.count_subscribers('/initialpose')>=1 and now-start>5.:
                x,y,yaw=json.loads((RELEASE/'runtime_assets.json').read_text())['route'][0]
                seed=PoseWithCovarianceStamped();seed.header.stamp=node.get_clock().now().to_msg();seed.header.frame_id='map';seed.pose.pose.position.x=x;seed.pose.pose.position.y=y;seed.pose.pose.orientation.z=math.sin(yaw/2);seed.pose.pose.orientation.w=math.cos(yaw/2)
                seed.pose.covariance[0]=seed.pose.covariance[7]=seed.pose.covariance[35]=.15**2
                seed_pub.publish(seed);seeded=True
            valid=states.get('plan',{}).get('valid',False) and now-at.get('plan',0)<.5
            if not valid:stable=None
            elif stable is None:stable=now
            if stable is not None and now-stable>=5.:passed=True;break
            if now-printed>3.:
                print(json.dumps(dict(reason=states.get('plan',{}).get('reason'),pose=states.get('pose'),tracks=states.get('plan',{}).get('confirmed_obstacle_tracks'))),flush=True);printed=now
        if not passed:failure='stationary_compact_admission_not_established'
    except (Exception,KeyboardInterrupt) as e:failure=str(e) or 'interrupted'
    finally:
        for p in children:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in children:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for h in handles:h.close()
        node.destroy_node();rclpy.shutdown()
    report=dict(passed=passed,failure=failure,physical_motion=False,actuator_nodes_started=0,assumed_ego_stationary=True,scope=__doc__,latest=states)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');(out/'records.json').write_text(json.dumps(rows)+'\n')
    print(json.dumps(dict(passed=passed,failure=failure,report=str(out/'report.json')),indent=2),flush=True)
    raise SystemExit(0 if passed else 1)
if __name__=='__main__':main()
