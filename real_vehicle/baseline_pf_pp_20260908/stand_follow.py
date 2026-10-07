"""PF+PP stand integration, never ground. Default cannot connect hardware.

Live motion requires explicit mode, operator alignment confirmation, released L1, then
a new L1 hold. Independent window limits motion to one <=2s attempt.
--zero-only never starts operator gate or grants window permission.
"""
import argparse
import json
import math
import os
import re
from pathlib import Path
import select
import signal
import subprocess
import sys
import threading
import time
from stand_policy import validate
from check_runtime import check
from session_guard import acquire
from stand_readiness import SupervisorLease, localization_ready, prearm_ready, operator_released
from ground_readiness import fingerprint

ROOT=Path(__file__).resolve().parent
DRIVERS=Path('/home/f1tenth/f12026_ws/install')
CONFIG='/home/f1tenth/f12026_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--offline-self-test',action='store_true')
    p.add_argument('--stand-confirmed',action='store_true')
    p.add_argument('--zero-only',action='store_true')
    p.add_argument('--allow-wheel-motion',action='store_true')
    p.add_argument('--duration',type=float,default=120.)
    p.add_argument('--zero-seconds',type=float,default=5.)
    p.add_argument('--zero-rviz',action='store_true')
    p.add_argument('--zero-joy',action='store_true',help='Include real Joy load in zero-only validation')
    p.add_argument('--require-localization',action='store_true',help='Zero-only: require continuous valid PF/PP after manual initialization')
    p.add_argument('--zero-route-seed',action='store_true',help='Zero-only diagnostic: seed from route start, NOT verified alignment or motion permission')
    p.add_argument('--route-start-confirmed',action='store_true',help='Operator confirms actual map position AND heading at route start; replaces GUI initialization, never L1 or freshness checks')
    p.add_argument('--pose-timeout',type=float,default=.20,help='Pose-only experimental timeout, 0.20..0.30 seconds')
    p.add_argument('--fault',choices=['window','release','guard_kill','window_kill','supervisor_permit_loss','supervisor_main_stall'],default='window')
    args=p.parse_args()
    from baseline_pf_pp.core import checked_pose_timeout
    try:checked_pose_timeout(args.pose_timeout)
    except ValueError as exc:p.error(str(exc))
    if not math.isfinite(args.zero_seconds) or not 5.<=args.zero_seconds<=240.:p.error('zero-seconds must be 5..240')
    if args.zero_seconds!=5. and not args.zero_only:p.error('zero-seconds requires zero-only')
    if (args.zero_joy or args.require_localization) and not args.zero_only:p.error('zero-joy/require-localization require zero-only')
    if args.zero_only and args.duration<args.zero_seconds+25.:p.error('duration must allow zero-seconds plus 25s startup')
    if args.require_localization and args.duration<args.zero_seconds+85.:p.error('duration must allow zero-seconds plus 85s initialization')
    if args.zero_rviz and (not args.zero_only or not os.environ.get('DISPLAY')):p.error('zero-rviz requires zero-only and desktop DISPLAY')
    try:validate(args,os.environ.get('ROS_DOMAIN_ID'),os.environ.get('DISPLAY'),sys.stdin.isatty())
    except ValueError as exc:p.error(str(exc))
    if not args.offline_self_test and args.fault!='window':p.error('Fault injection is offline only')
    runtime=check();lock=acquire()
    route_seed=None
    if args.zero_route_seed or args.route_start_confirmed:
        from f1tenth_safety_migration.route_mpc_core import load_open_route
        route_seed=[float(v) for v in load_open_route(str(ROOT.parent/'routes/manual_route_20260904_215501_controller_ready.csv'))[0]]
        if len(route_seed)!=3 or not all(math.isfinite(v) for v in route_seed):raise RuntimeError('Invalid route start seed')
    if not args.offline_self_test:
        from sensor_only import conflicting_processes
        if conflicting_processes():raise RuntimeError('Vehicle/controller process already running')
        required=['/dev/sensors/vesc']+(['/dev/input/js0'] if args.allow_wheel_motion or args.zero_joy else [])
        for path in required:
            if not Path(path).exists():raise RuntimeError('Missing '+path)
        if subprocess.run(['fuser','/dev/sensors/vesc'],capture_output=True).returncode==0:raise RuntimeError('VESC busy')
        if args.allow_wheel_motion or args.zero_joy:
            result=subprocess.run(['/opt/ros/foxy/lib/joy/joy_enumerate_devices'],capture_output=True,text=True,timeout=4)
            if result.returncode or not re.search(r'^\s*0\s*:\s*Wireless Controller\s*$',result.stdout,re.M):raise RuntimeError('SDL device 0 must be the readable PS4')
    import rclpy
    from std_msgs.msg import Bool,String,Float64
    from ackermann_msgs.msg import AckermannDriveStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan,Joy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from baseline_pf_pp.safe_pf import latest_sensor_qos
    log=ROOT/'run_logs'/(time.strftime('%Y%m%d_%H%M%S')+'-pf-pp-'+('offline-'+args.fault if args.offline_self_test else 'stand-zero' if args.zero_only else 'stand-attempt'))
    log.mkdir(parents=True,exist_ok=False);(log/'runtime.json').write_text(json.dumps(runtime,indent=2)+'\n')
    children={};handles=[];statuses={};received={};currents=[];speeds=[];finals=[];virtual=[];joy=[False,None];initial=[None];async_fault=[None];events=[]
    report={'passed':False,'offline':args.offline_self_test,'zero_only':args.zero_only,'motion_permission':False,'pose_timeout_sec':args.pose_timeout}
    report.update(continuous_localization_passed=False,zero_joy=args.zero_joy,zero_rviz=args.zero_rviz)
    report.update(initialization_mode='operator_confirmed_route_start' if args.route_start_confirmed else 'unverified_route_seed' if args.zero_route_seed else 'manual_or_offline',route_seed=route_seed)
    report['code_fingerprint']=fingerprint()
    localization_monitor={'active':False,'pp_samples':0,'max_pose_source_age_s':0.}
    rclpy.init();node=rclpy.create_node('stand_pf_supervisor',start_parameter_services=False)
    permit_pub=node.create_publisher(Bool,'/stand_pf/permit',1);hold_pub=node.create_publisher(Bool,'/stand_pf/test_hold',1) if args.offline_self_test else None
    seed_pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10) if route_seed is not None else None
    permitted=False;held=False;permit_heartbeat=True;lease=SupervisorLease()
    def heartbeat():
        if permit_heartbeat:permit_pub.publish(Bool(data=lease.allow(permitted,time.monotonic())))
        if hold_pub is not None:hold_pub.publish(Bool(data=held))
    node.create_timer(.02,heartbeat)
    timing={'health_samples':0,'max_source_age_s':0.,'max_bridge_age_s':0.,'max_bridge_to_supervisor_s':0.,'max_spin_gap_s':0.,'max_abs_erpm':0.}
    pp_delivery={'samples':0,'max_tick_gap_s':0.,'max_delivery_s':0.,'max_receipt_gap_s':0.}
    previous_step=time.monotonic();strict_health=[False]
    def status(key,msg):
        try:
            data=json.loads(msg.data);previous=statuses.get(key,{})
            if any(data.get(k)!=previous.get(k) for k in ('valid','reason','arm_state','stop_reason','latched')):
                events.append({'stream':key,'received_mono':time.monotonic(),'status':data})
            previous_receipt=received.get(key)
            statuses[key]=data;received[key]=time.monotonic()
            if key=='pp' and 'runtime_timing' in data:
                rt=data['runtime_timing'];delivery=received[key]-rt['publish_mono']
                if not math.isfinite(delivery) or delivery<0.:raise ValueError('Invalid PP publication time')
                pp_delivery['samples']+=1
                pp_delivery['max_delivery_s']=max(pp_delivery['max_delivery_s'],delivery)
                if rt['tick_gap_s'] is not None:pp_delivery['max_tick_gap_s']=max(pp_delivery['max_tick_gap_s'],rt['tick_gap_s'])
                if previous_receipt is not None:pp_delivery['max_receipt_gap_s']=max(pp_delivery['max_receipt_gap_s'],received[key]-previous_receipt)
                pp_delivery['last']=dict(rt,delivery_s=delivery)
            if key=='health':
                h=data;ros=node.get_clock().now().nanoseconds*1e-9
                sample={'source_age_s':ros-h['stamp'],'bridge_age_s':h['bridge_ros']-h['stamp'],
                    'bridge_to_supervisor_s':received[key]-h['bridge_mono'],'ros_now':ros,'source_stamp':h['stamp']}
                if not all(math.isfinite(v) for v in sample.values()):raise ValueError('nonfinite health timing')
                if not math.isfinite(h['erpm']):raise ValueError('nonfinite motor speed')
                timing['max_abs_erpm']=max(timing['max_abs_erpm'],abs(h['erpm']))
                timing['last_health']=sample;timing['health_samples']+=1
                for k in ('source_age_s','bridge_age_s','bridge_to_supervisor_s'):timing['max_'+k]=max(timing['max_'+k],sample[k])
            if localization_monitor['active'] and key in ('pf','pp'):
                if data.get('valid') is not True or (key=='pp' and data.get('reason')!='waiting_for_operator'):
                    async_fault[0]='Continuous localization invalid: '+key+' '+json.dumps(data)
                if key=='pp':
                    age=data['input_age']['pose']['source_age_s']
                    if not math.isfinite(age):raise ValueError('nonfinite pose age')
                    localization_monitor['pp_samples']+=1
                    localization_monitor['max_pose_source_age_s']=max(localization_monitor['max_pose_source_age_s'],age)
        except Exception as exc:async_fault[0]='Invalid '+key+' status: '+str(exc)
    for key,topic in [('operator','/stand_pf/operator_status'),('relay','/baseline/scan_relay_status'),('pf','/baseline/localization_status'),('pp','/baseline/controller_status'),('final','/baseline/final_status'),('window','/stand_pf/window_status'),('actuator','/stand_pf/actuator_status'),('health','/stand_pf/health')]:
        node.create_subscription(String,topic,lambda m,k=key:status(k,m),1)
    def command(msg):
        if not math.isfinite(msg.drive.speed) or not math.isfinite(msg.drive.steering_angle):
            async_fault[0]='Invalid final command';return
        finals.append((time.monotonic(),msg.drive.speed,msg.drive.steering_angle))
    node.create_subscription(AckermannDriveStamped,'/baseline/final',command,10)
    def current(msg):
        currents.append((time.monotonic(),msg.data))
        if msg.data!=0.:async_fault[0]='Nonzero/nonfinite current command forbidden'
    node.create_subscription(Float64,'/stand_pf/motor_current',current,10)
    node.create_subscription(Float64,'/stand_pf/motor_speed',lambda m:speeds.append((time.monotonic(),m.data)),10)
    node.create_subscription(Float64,'/stand_pf/effective_rpm',lambda m:virtual.append((time.monotonic(),m.data)),10)
    def joy_cb(msg):joy[:]=[len(msg.buttons)>4 and msg.buttons[4]==1,time.monotonic()]
    node.create_subscription(Joy,'/baseline/joy',joy_cb,latest_sensor_qos())
    node.create_subscription(PoseWithCovarianceStamped,'/initialpose',lambda m:initial.__setitem__(0,time.monotonic()) if m.header.frame_id=='map' else None,10)
    node.create_subscription(Odometry,'/baseline/odom',lambda m:received.update(odom=time.monotonic()),1)
    node.create_subscription(LaserScan,'/baseline/scan',lambda m:received.update(scan=time.monotonic()),latest_sensor_qos())
    # A dedicated executor continuously drains the monitoring subscriptions.
    # Main-loop permission has a separate lease: draining callbacks must not
    # renew a motion permit if main-loop safety checks stop running.
    executor=rclpy.executors.SingleThreadedExecutor();executor.add_node(node)
    def spin():
        try:executor.spin()
        except BaseException as exc:async_fault[0]='Supervisor executor exited: '+str(exc)
    spin_thread=threading.Thread(target=spin,name='stand-supervisor-executor',daemon=True);spin_thread.start()
    def spawn(name,module=None,params=None,cmd=None,env=None):
        if cmd is None:
            cmd=[sys.executable,str(ROOT/module)] if module.endswith('.py') else [sys.executable,'-c','from '+module.rsplit('.',1)[0]+' import '+module.rsplit('.',1)[1]+' as m; m.main()']
        if params:
            cmd+=['--ros-args']
            for key,value in params.items():cmd+=['-p',key+':='+(str(value).lower() if isinstance(value,bool) else str(value))]
        handle=(log/(name+'.log')).open('x');handles.append(handle)
        children[name]=subprocess.Popen(cmd,env=env,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
    start=time.monotonic();ignored=set()
    def step():
        nonlocal previous_step
        now=time.monotonic();timing['max_spin_gap_s']=max(timing['max_spin_gap_s'],now-previous_step);previous_step=now
        time.sleep(.01)
        if async_fault[0]:raise RuntimeError(async_fault[0])
        if not spin_thread.is_alive():raise RuntimeError('Supervisor executor is not running')
        for name,proc in children.items():
            if name not in ignored and proc.poll() is not None:raise RuntimeError(name+' exited')
        if time.monotonic()-start>args.duration:raise RuntimeError('Bounded session timeout')
        if strict_health[0] and 'vesc' in children and 'health' in statuses:
            health=statuses['health']
            if not math.isfinite(health['erpm']) or health['fault']!=0:raise RuntimeError('VESC fault/nonfinite telemetry')
            if time.monotonic()-received['health']>.3:raise RuntimeError('VESC telemetry stale')
            age=node.get_clock().now().nanoseconds*1e-9-health['stamp']
            if not -.02<=age<=.3:
                timing['failure']={'source_age_s':age,'receipt_age_s':time.monotonic()-received['health'],'health':health}
                raise RuntimeError('VESC telemetry source stamp stale: '+json.dumps(timing['failure']))
        lease.checked(time.monotonic())
    def until(predicate,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            step()
            if predicate():return
        raise RuntimeError('Preflight timeout: '+json.dumps(statuses))
    def stable_inputs():
        return prearm_ready(statuses,received,time.monotonic(),joy)
    def terminate(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,terminate)
    try:
        spawn('map_io','stand_map_io.py')
        spawn('relay','stand_scan_relay.py')
        pf_config='pf_offline.yaml' if args.offline_self_test else 'pf_stationary_candidate.yaml'
        spawn('pf',cmd=[sys.executable,'-c','from baseline_pf_pp.safe_pf import main; main()',
            '--ros-args','--params-file',str(ROOT/'config'/pf_config)])
        spawn('pp','baseline_pf_pp.follower',{'route_file':str(ROOT.parent/'routes/manual_route_20260904_215501_controller_ready.csv'),'pose_timeout_sec':args.pose_timeout})
        spawn('final','f1tenth_safety_migration.final_command_guard',{
            'nominal_topic':'/baseline/drive_nominal','scan_topic':'/baseline/scan','odom_topic':'/baseline/odom',
            'planner_valid_topic':'/baseline/planner_valid','deadman_topic':'/baseline/deadman','arm_topic':'/baseline/arm',
            'final_topic':'/baseline/final','status_topic':'/baseline/final_status','require_target_belief':False,
            'require_deadman':True,'max_speed':.5,'max_steering_angle':.27,'nominal_timeout':.15,
            'planner_timeout':.15,'scan_timeout':.20,'odom_timeout':.20,'deadman_timeout':.15})
        spawn('window','stand_window.py')
        spawn('actuator','f1tenth_safety_migration.hardware_actuator_guard',{
            'final_topic':'/stand_pf/actuator_input','motor_topic':'/stand_pf/motor_speed',
            'motor_current_topic':'/stand_pf/motor_current','servo_topic':'/stand_pf/servo',
            'status_topic':'/stand_pf/actuator_status','max_abs_speed':0. if args.zero_only else .5,
            'max_abs_rpm':0. if args.zero_only else 3750.,'max_abs_steering':.27,'publish_rate':50.0})
        until(lambda:len(currents)>=20 and len(finals)>=10,12.)
        if any(v!=0. for _,v in currents) or any(v!=0. or s!=0. for _,v,s in finals) or speeds:raise RuntimeError('Not continuously STOP before driver')
        if node.count_publishers('/stand_pf/motor_current')!=1:raise RuntimeError('Multiple current owners')
        if args.offline_self_test:
            spawn('inputs','stand_test_inputs.py');spawn('sink','stand_virtual_sink.py')
        else:
            driver_env=os.environ.copy()
            libs=[str(DRIVERS/pkg/'lib') for pkg in ('vesc_driver','vesc_msgs','vesc_ackermann')]
            driver_env['LD_LIBRARY_PATH']=':'.join(libs+[driver_env.get('LD_LIBRARY_PATH','')])
            driver_env['PYTHONPATH']=str(DRIVERS/'vesc_msgs/lib/python3.8/site-packages')+':'+driver_env.get('PYTHONPATH','')
            cmd=[str(DRIVERS/'vesc_driver/lib/vesc_driver/vesc_driver_node'),'--ros-args','--params-file',CONFIG,
                 '-r','commands/motor/current:=/stand_pf/motor_current','-r','commands/motor/speed:=/stand_pf/motor_speed',
                 '-r','commands/servo/position:=/stand_pf/'+('servo_disabled' if args.zero_only else 'servo'),
                 '-r','sensors/core:=/stand_pf/telemetry','-r','sensors/servo_position_command:=/stand_pf/servo_feedback',
                 '-p','current_min:=0.0','-p','current_max:=0.0','-p','speed_min:=0.0','-p','speed_max:='+('0.0' if args.zero_only else '3750.0')]
            for key in ('duty_cycle','brake','position'):cmd+=['-r','commands/motor/'+key+':=/stand_pf/disabled_'+key]
            spawn('vesc',cmd=cmd,env=driver_env);spawn('telemetry','stand_telemetry.py',env=driver_env)
            spawn('odom',cmd=[str(DRIVERS/'vesc_ackermann/lib/vesc_ackermann/vesc_to_odom_node'),'--ros-args','--params-file',CONFIG,
                '-r','__node:=stand_vesc_odometry','-p','wheelbase:=0.25',
                '-r','sensors/core:=/stand_pf/telemetry','-r','sensors/servo_position_command:=/stand_pf/servo_feedback',
                '-r','odom:=/baseline/odom','-p','publish_tf:=false','-p','use_servo_cmd_to_calc_angular_velocity:='+('false' if args.zero_only else 'true')],env=driver_env)
            spawn('lidar',cmd=['/opt/ros/foxy/lib/urg_node/urg_node_driver','--ros-args','-r','scan:=/baseline/raw_scan',
                '-p','ip_address:=192.168.0.10','-p','laser_frame_id:=laser','-p','calibrate_time:=false'])
            if args.allow_wheel_motion or args.zero_joy:
                joy_env=os.environ.copy();joy_env['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS']='1'
                spawn('joy',cmd=['/opt/ros/foxy/lib/joy/joy_node','--ros-args','-r','joy:=/baseline/joy',
                    '-p','device_id:=0','-p','autorepeat_rate:=50.0'],env=joy_env)
            # Synchronous process startup can pause this supervisor for hundreds
            # of milliseconds. Discard any pre-startup health sample and wait
            # for a new sample after every child has been launched. Strict
            # runtime checks begin only then; their thresholds are unchanged.
            children_ready_at=time.monotonic()
            until(lambda:received.get('health',0.)>children_ready_at and
                all(time.monotonic()-received.get(k,0.)<.2 for k in ('odom','scan','health')) and
                (not (args.allow_wheel_motion or args.zero_joy) or joy[1] is not None),15.)
            strict_health[0]=True;timing['strict_health_started_mono']=time.monotonic()
            step()
            if abs(statuses['health']['erpm'])>=100.:raise RuntimeError('VESC not stationary')
        print('ZERO PREFLIGHT PASSED. No operator gate/permission yet. Logs: '+str(log),flush=True)
        if seed_pub is not None:
            until(lambda:node.count_subscribers('/initialpose')>=2,10.)
            seed=PoseWithCovarianceStamped();seed.header.frame_id='map'
            seed.header.stamp=node.get_clock().now().to_msg()
            x,y,yaw=route_seed
            seed.pose.pose.position.x=x;seed.pose.pose.position.y=y
            seed.pose.pose.orientation.z=math.sin(yaw/2);seed.pose.pose.orientation.w=math.cos(yaw/2)
            seed.pose.covariance[0]=.15**2;seed.pose.covariance[7]=.15**2;seed.pose.covariance[35]=.15**2
            seed_pub.publish(seed)
            print('INITIAL POSE: '+report['initialization_mode']+' '+json.dumps(route_seed)+'. Live PF/PP checks remain required.',flush=True)
        if args.zero_only:
            if args.zero_rviz:spawn('rviz',cmd=['/opt/ros/foxy/bin/rviz2','-d',str(ROOT/'config/stand_follow.rviz')])
            if args.require_localization:
                if seed_pub is not None:
                    print('ZERO ONLY: initialized from route start. No ALIGNED/L1; motion forbidden.',flush=True)
                else:
                    print('ZERO ONLY: set 2D Pose Estimate within 60s. No ALIGNED or L1; motion remains forbidden.',flush=True)
                until(lambda:initial[0] is not None and localization_ready(statuses,received,time.monotonic()),60.)
                localization_monitor['active']=True
                print('ZERO ONLY: continuous localization timing check started for '+str(args.zero_seconds)+' seconds.',flush=True)
            begin=time.monotonic()
            while time.monotonic()-begin<args.zero_seconds:
                step()
                if statuses.get('window',{}).get('latched'):raise RuntimeError('Stand window latched during zero validation: '+json.dumps(statuses['window']))
                if abs(statuses['health']['erpm'])>=100. or speeds:raise RuntimeError('Unexpected motion in zero-only mode')
                ages={k:time.monotonic()-received.get(k,0.) for k in ('scan','odom','actuator')}
                if any(v>.3 for v in ages.values()):raise RuntimeError('Zero-only sensor/guard stale: '+json.dumps(ages))
                if args.require_localization and not localization_ready(statuses,received,time.monotonic()):
                    failed_at=time.monotonic()
                    report['localization_seconds']=failed_at-begin
                    report['localization_failure']={'receipt_age_s':{k:failed_at-received.get(k,0.) for k in ('pf','pp')},
                        'pf':statuses.get('pf'),'pp':statuses.get('pp')}
                    raise RuntimeError('Continuous localization readiness lost: '+json.dumps(report['localization_failure']))
            localization_monitor['active']=False
            if async_fault[0]:raise RuntimeError(async_fault[0])
            report.update(passed=True,health=statuses.get('health'),pf=statuses.get('pf'),zero_current_samples=len(currents))
            report.update(continuous_localization_passed=args.require_localization,
                          localization_seconds=time.monotonic()-begin if args.require_localization else 0.)
            return
        if args.allow_wheel_motion:
            spawn('rviz',cmd=['/opt/ros/foxy/bin/rviz2','-d',str(ROOT/'config/stand_follow.rviz')])
            print('Four wheels MUST remain off ground. Servo may center. Do not reposition a powered stand.\n'
                  'Use 2D Pose Estimate at the actual base position/heading; verify scan/map and blue route START.\n'
                  'L1 RELEASED. Type ALIGNED only after checking both position and heading.',flush=True)
            if args.route_start_confirmed:
                print('Operator-confirmed route position/heading: no repeated GUI/ALIGNED required. Waiting for live localization; keep L1 released.',flush=True)
                until(lambda:initial[0] is not None and localization_ready(statuses,received,time.monotonic()),60.)
            else:
                while not select.select([sys.stdin],[],[],0)[0]:step()
                if sys.stdin.readline().strip()!='ALIGNED' or initial[0] is None:raise RuntimeError('Manual alignment not confirmed')
        stable_since=None;deadline=time.monotonic()+30.
        while time.monotonic()<deadline:
            step()
            stable_since=(stable_since or time.monotonic()) if stable_inputs() else None
            if stable_since and time.monotonic()-stable_since>=1.:break
        else:raise RuntimeError('PF/PP/start/PS4 continuous readiness failed: '+json.dumps(statuses))
        spawn('operator','f1tenth_safety_migration.route_operator_gate',{
            'joy_topic':'/baseline/joy','arm_topic':'/baseline/arm','deadman_topic':'/baseline/deadman',
            'enable_topic':'/baseline/enable','nominal_topic':'/baseline/drive_nominal','planner_valid_topic':'/baseline/planner_valid',
            'status_topic':'/stand_pf/operator_status'})
        until(lambda:node.count_subscribers('/stand_pf/permit')>=1 and node.count_publishers('/baseline/deadman')==1,5.)
        # Parent Joy reception does not prove the newly spawned operator has
        # discovered Joy or seen release. Require that node's explicit fresh
        # released state, while all original prearm conditions remain true.
        until(lambda:operator_released(statuses,received,time.monotonic()) and stable_inputs(),10.)
        permitted=True;report['motion_permission']=True
        print('READY: hold L1 for ONE <=2s stand attempt; release latches STOP. Never put the car on the ground.',flush=True)
        if args.offline_self_test:held=True
        until(lambda:statuses.get('window',{}).get('started') is not None,30.)
        if args.offline_self_test:until(lambda:any(v>0. for _,v in virtual),1.)
        active_at=time.monotonic()
        if args.offline_self_test and args.fault!='window':
            if args.fault=='release':held=False
            elif args.fault=='supervisor_permit_loss':permit_heartbeat=False
            elif args.fault=='supervisor_main_stall':time.sleep(.30)
            else:
                target='final' if args.fault=='guard_kill' else 'window';ignored.add(target)
                os.killpg(children[target].pid,signal.SIGKILL)
        end=time.monotonic()+3.5
        while time.monotonic()<end:step()
        permitted=False;held=False
        if args.offline_self_test:
            positives=[t for t,v in virtual if v>0.]
            if not positives:raise RuntimeError('No virtual motion reached sink')
            last_positive=max(positives);zeros=[t for t,v in virtual if t>last_positive and v==0.]
            if not zeros or virtual[-1][1]!=0.:raise RuntimeError('No persistent virtual STOP')
            span=(last_positive-min(positives))*1000.
            if span>2300.:raise RuntimeError('Exceeded independent stand time bound')
            latency=(zeros[0]-active_at)*1000.
            if args.fault!='window' and latency>500.:raise RuntimeError('Fault STOP exceeded 500ms')
            last_rpm_command=max(t for t,v in speeds if v>0.)
            coast=[t for t,v in currents if t>last_rpm_command and v==0.]
            if not coast or time.monotonic()-coast[-1]>.15:raise RuntimeError('Downstream did not continuously command zero-current coast')
            coast_latency=(coast[0]-active_at)*1000.
            if args.fault!='window' and coast_latency>500.:raise RuntimeError('Downstream coast command exceeded 500ms')
            if args.fault=='window' and statuses.get('window',{}).get('reason')!='two_second_limit':raise RuntimeError('Upstream stopped before independent limit could be verified')
            report.update(virtual_nonzero_span_ms=span,stop_after_fault_ms=latency,coast_command_after_fault_ms=coast_latency,fault=args.fault)
        elif abs(statuses['health']['erpm'])>=100.:
            raise RuntimeError('Wheels not stationary after bounded attempt: use physical cutoff')
        report.update(passed=True,window=statuses.get('window'),final=statuses.get('final'),health=statuses.get('health'))
    except KeyboardInterrupt:report['operator_exit']=True
    except Exception as exc:report['error']=str(exc);raise
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        permitted=False;held=False;permit_heartbeat=False;localization_monitor['active']=False
        permit_pub.publish(Bool(data=False))
        # Remove motion sources, keep actuator zero output until VESC exits.
        for name in ('operator','inputs','pp','final','window','rviz','lidar','relay','odom','telemetry','vesc','actuator','pf','map_io','sink','joy'):
            proc=children.get(name)
            if proc and proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=2)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=2)
        for handle in handles:handle.close()
        executor.shutdown(timeout_sec=2.);spin_thread.join(timeout=2.)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        report['last_statuses']=statuses
        report['timing']=timing
        report['pp_delivery']=pp_delivery
        report['localization_monitor']=localization_monitor
        report['supervisor_lease_expired']=lease.expired
        report['completed_unix']=time.time()
        (log/'status_events.json').write_text(json.dumps(events,indent=2)+'\n')
        (log/'command_trace.json').write_text(json.dumps({'current':currents,'rpm_command':speeds,
            'final_command':finals,'virtual_rpm':virtual,'clock':'monotonic receipt seconds'})+'\n')
        (log/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)


if __name__=='__main__':main()
