#!/usr/bin/env python3
"""Actual C++ planner/controller with synthetic LiDAR and a dynamic plant.

ROS domain 83, localhost, no hardware drivers or actuator command topics.
This is a software integration test, not a physical clearance acceptance.
"""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
import numpy as np

def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='83'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    ap=argparse.ArgumentParser();ap.add_argument('--fault',choices=['none','target_loss','release'],default='none')
    ap.add_argument('--lead-speed',type=float,default=.15)
    ap.add_argument('--lead-stop-after',type=float,default=100.)
    ap.add_argument('--lead-lateral-center',type=float,default=0.)
    ap.add_argument('--lead-lateral-jitter',type=float,default=0.)
    ap.add_argument('--report-name',default='')
    ap.add_argument('--release',type=Path)
    args=ap.parse_args()
    if args.release:os.environ['COMPACT_RELEASE']=str(args.release.resolve())
    from test_compact_contract import D,RELEASE,origin,pose
    lead_start=json.loads((RELEASE/'compact_config.json').read_text()).get('lead_initial_center_distance_m',1.6)
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool,String
    from ackermann_msgs.msg import AckermannDriveStamped
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init();node=rclpy.create_node('compact_non_actuating_integration')
    pubs={k:node.create_publisher(t,name,1 if k!='scan' else qos_profile_sensor_data) for k,t,name in [
      ('pose',Odometry,'/baseline/localization_pose'),('odom',Odometry,'/baseline/odom'),
      ('scan',LaserScan,'/baseline/scan'),('quality',Bool,'/baseline/localization_valid'),('enable',Bool,'/baseline/enable')]}
    command=[0.,0.];records=[];plans=[]
    def drive(m):command[:]=[float(m.drive.speed),float(m.drive.steering_angle)]
    node.create_subscription(AckermannDriveStamped,'/baseline/drive_nominal',drive,10)
    node.create_subscription(String,'/baseline/controller_status',lambda m:records.append(dict(receipt=time.monotonic(),**json.loads(m.data))),100)
    node.create_subscription(String,'/baseline/local_plan_cpp',lambda m:plans.append(json.loads(m.data)),100)
    out=D/'reports'/(args.report_name or ('compact_ros_'+args.fault));out.mkdir(exist_ok=True)
    children=[];handles=[];failure=None;completed=False;active=None;fault_at=None
    x,y,yaw=map(float,origin);v=vy=rate=servo=0.;next_sensor=time.monotonic();last_step=next_sensor;start=next_sensor
    angles=np.linspace(-2.35619449,2.35619449,1081);world_heading=float(origin[2]);wc,ws=math.cos(world_heading),math.sin(world_heading)
    trajectory=[]
    try:
        for name in ['planner','controller']:
            log=(out/(name+'.log')).open('w');handles.append(log)
            children.append(subprocess.Popen([str(RELEASE/'install_cpp/cpp_frenet/lib/cpp_frenet'/('cpp_frenet_'+name)),
              '--ros-args','-p','assets_file:='+str(RELEASE/'runtime_assets.json'),'-p','tracker:=dynamic_mpc'],stdout=log,stderr=log,start_new_session=True))
        while time.monotonic()-start<22:
            now=time.monotonic();elapsed=0. if active is None else now-active
            if any(p.poll() is not None for p in children):raise RuntimeError('node_exited')
            dt=min(.01,max(0.,now-last_step));last_step=now
            target,steering=command
            if active is not None:
                applied=0. if v<.03 and target<.18 else target
                dv=-.6 if target==0 and v>.03 else float(np.clip((applied-v)/.15,-.8,2.))
                v=max(0.,v+dt*dv);servo+=dt*(steering-servo)/.20
                if v<.5:vy=0.;rate=v/.25*math.tan(servo)
                else:
                    m,iz,lf,lr,cf,cr=3.74,.04712,.12,.13,90.,96.
                    dvy=-(cf+cr)/(m*v)*vy+(-v-(cf*lf-cr*lr)/(m*v))*rate+cf/m*servo
                    dr=-(cf*lf-cr*lr)/(iz*v)*vy-(cf*lf*lf+cr*lr*lr)/(iz*v)*rate+lf*cf/iz*servo
                    vy+=dt*dvy;rate+=dt*dr
                x+=dt*(v*math.cos(yaw)-vy*math.sin(yaw));y+=dt*(v*math.sin(yaw)+vy*math.cos(yaw));yaw+=dt*rate
            if now>=next_sensor:
                next_sensor=now+.02;stamp=node.get_clock().now().to_msg()
                msg=Odometry();msg.header.stamp=stamp;msg.header.frame_id='map';msg.child_frame_id='base_link'
                msg.pose.pose.position.x=x;msg.pose.pose.position.y=y
                msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
                pubs['pose'].publish(msg)
                msg.header.frame_id='odom';msg.twist.twist.linear.x=v;msg.twist.twist.angular.z=v/.25*math.tan(steering);pubs['odom'].publish(msg)
                scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='laser';scan.angle_min=float(angles[0]);scan.angle_max=float(angles[-1]);scan.angle_increment=float(angles[1]-angles[0]);scan.range_min=.05;scan.range_max=10.
                lx=x+.27*math.cos(yaw);ly=y+.27*math.sin(yaw)
                local_x=(lx-origin[0])*wc+(ly-origin[1])*ws;local_y=-(lx-origin[0])*ws+(ly-origin[1])*wc
                dx=np.cos(angles+yaw-world_heading);dy=np.sin(angles+yaw-world_heading)
                lead_coast_t=min(max(0.,elapsed-args.lead_stop_after),args.lead_speed/.4)
                lead_x=(lead_start+args.lead_speed*min(elapsed,args.lead_stop_after)
                        +args.lead_speed*lead_coast_t-.5*.4*lead_coast_t**2)
                lead_y=args.lead_lateral_center+args.lead_lateral_jitter*math.sin(elapsed*8.)
                with np.errstate(divide='ignore',invalid='ignore'):
                    ax=(lead_x-.15-local_x)/dx;bx=(lead_x+.15-local_x)/dx
                    ay=(lead_y-.09-local_y)/dy;by=(lead_y+.09-local_y)/dy
                    near=np.maximum(np.minimum(ax,bx),np.minimum(ay,by));far=np.minimum(np.maximum(ax,bx),np.maximum(ay,by))
                    ranges=np.where((far>=near)&(near>.05),near,np.inf)
                if active is not None and elapsed>1.2 and args.fault!='none':
                    if fault_at is None:fault_at=now
                    if args.fault=='target_loss':ranges[:]=np.inf
                scan.ranges=ranges.astype(np.float32).tolist();pubs['scan'].publish(scan)
                pubs['quality'].publish(Bool(data=True));pubs['enable'].publish(Bool(data=active is not None and not (args.fault=='release' and fault_at is not None)))
                trajectory.append(dict(t=now-start,x=x,y=y,yaw=yaw,speed=v,command=command[:],lead_x=lead_x))
            rclpy.spin_once(node,timeout_sec=.001)
            if active is None and now-start>3. and records and records[-1].get('valid') and records[-1].get('reason')=='waiting_for_operator':
                active=now;print('synthetic target admitted; enabling simulated plant',flush=True)
            if active is not None and elapsed>.4 and records:
                latest=records[-1]
                if args.fault=='none' and not latest.get('valid'):
                    raise RuntimeError('controller_rejected:'+latest.get('reason','unknown'))
                if latest.get('complete'):
                    completed=True;break
                if fault_at is not None and now-fault_at>3.5:
                    late=[r for r in records if r['receipt']>fault_at+.65]
                    if not late or any(r.get('speed_command',0)!=0 for r in late):raise RuntimeError('fault_failed_to_stop')
                    if args.fault=='target_loss' and not any('compact_' in r.get('reason','') for r in late):raise RuntimeError('wrong_stop_reason')
                    if v>.03:raise RuntimeError('fault_plant_not_stopped')
                    completed=True;break
        if not completed:raise RuntimeError('integration_timeout')
    except Exception as e:failure=str(e)
    finally:
        pubs['enable'].publish(Bool(data=False))
        for p in children:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in children:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for h in handles:h.close()
        node.destroy_node();rclpy.shutdown()
    report=dict(passed=completed and failure is None,failure=failure,physical_motion=False,scope=__doc__,
      max_command_mps=max((r['command'][0] for r in trajectory),default=0),final_speed_mps=v,
      final_pose=[x,y,yaw],controller_messages=len(records),planner_messages=len(plans),fault=args.fault,
      release=str(RELEASE),lead_speed_mps=args.lead_speed,lead_stop_after_s=args.lead_stop_after,
      lead_lateral_center_m=args.lead_lateral_center,lead_lateral_jitter_m=args.lead_lateral_jitter)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (out/'records.json').write_text(json.dumps(dict(controller=records,planner=plans,plant=trajectory))+'\n')
    print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 1)
if __name__=='__main__':main()
