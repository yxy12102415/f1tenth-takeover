#!/usr/bin/env python3
"""Actual C++ planner/controller with synthetic LiDAR and a dynamic plant.

ROS domain 83, localhost, no hardware drivers or actuator command topics.
This is a software integration test, not a physical clearance acceptance.
"""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
from controller_assets import assets_for_tracker
import numpy as np

def main():
    assert os.environ.get('ROS_DOMAIN_ID')=='83'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    ap=argparse.ArgumentParser();ap.add_argument('--fault',choices=['none','target_loss','scan_timeout','release'],default='none')
    ap.add_argument('--no-obstacle',action='store_true')
    ap.add_argument('--lead-speed',type=float,default=.15)
    ap.add_argument('--lead-start-distance',type=float,default=None,help='Synthetic centre distance override; does not modify runtime route/config')
    ap.add_argument('--ego-start-lateral',type=float,default=0.)
    ap.add_argument('--ego-start-heading-offset',type=float,default=0.)
    ap.add_argument('--lead-stop-after',type=float,default=100.)
    ap.add_argument('--lead-lateral-center',type=float,default=0.)
    ap.add_argument('--lead-lateral-jitter',type=float,default=0.)
    ap.add_argument('--lead-width',type=float,default=.18)
    ap.add_argument('--lead-length',type=float,default=.30)
    ap.add_argument('--wheel-pair',action='store_true',help='Raycast four separate tyre rectangles; no solid chassis return')
    ap.add_argument('--ego-returns',action='store_true',help='Synthetic returns strictly inside ego body; enables after motion begins')
    ap.add_argument('--report-name',default='')
    ap.add_argument('--release',type=Path)
    ap.add_argument('--tracker',choices=['dynamic_mpc','pid'],default='dynamic_mpc')
    ap.add_argument('--servo-response',type=float,default=.20,help='Synthetic plant time constant; not controller tuning')
    ap.add_argument('--speed-response',type=float,default=.15,help='Synthetic plant time constant; not controller tuning')
    ap.add_argument('--startup-command-deadzone',type=float,default=.18,help='Synthetic standstill motor deadzone; no hardware setting')
    ap.add_argument('--allow-body-stop',action='store_true',help='Stress-case safety acceptance: complete route, or verify zero commands/standstill after body-conflict rejection; a stop is not route completion')
    args=ap.parse_args()
    if not (.14<=args.lead_width<=.5 and .20<=args.lead_length<=.8):ap.error('Invalid synthetic body dimensions')
    if not (.10<=args.startup_command_deadzone<=.80):ap.error('Invalid synthetic startup deadzone')
    if not (.08<=args.servo_response<=.40 and .08<=args.speed_response<=.40):ap.error('Invalid plant response')
    if args.release:os.environ['COMPACT_RELEASE']=str(args.release.resolve())
    RELEASE=args.release.resolve() if args.release else Path(__file__).resolve().parent
    D=RELEASE
    assets=json.loads((RELEASE/'runtime_assets.json').read_text())
    origin=np.array(assets['route'][0])
    grid=np.array(assets['clearance']).reshape(assets['height'],assets['width'])
    def map_ranges(lx,ly,ray):
        steps=np.arange(.05,7.01,.05)[:,None]
        xx=lx+steps*np.cos(ray)[None,:];yy=ly+steps*np.sin(ray)[None,:]
        cols=np.floor((xx-assets['origin'][0])/assets['resolution']).astype(int)
        rows=assets['height']-1-np.floor((yy-assets['origin'][1])/assets['resolution']).astype(int)
        inside=(cols>=0)&(cols<assets['width'])&(rows>=0)&(rows<assets['height'])
        dist=grid[np.clip(rows,0,assets['height']-1),np.clip(cols,0,assets['width']-1)]
        hit=(~inside)|(dist<=.025)
        first=np.argmax(hit,axis=0)
        return np.where(hit.any(axis=0),steps[:,0][first],np.inf)
    lead_start=args.lead_start_distance if args.lead_start_distance is not None else json.loads((RELEASE/'compact_config.json').read_text()).get('generic_fixture_center_distance_m',1.8)
    if not (.6<=lead_start<=10.):ap.error('Synthetic lead distance outside fixture range')
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
    node.create_subscription(AckermannDriveStamped,'/baseline/drive_nominal',drive,1)
    node.create_subscription(String,'/baseline/controller_status',lambda m:records.append(dict(receipt=time.monotonic(),**json.loads(m.data))),100)
    node.create_subscription(String,'/baseline/local_plan_cpp',lambda m:plans.append(json.loads(m.data)),100)
    out=D/'reports'/(args.report_name or ('generic_ros_'+args.fault));out.mkdir(parents=True,exist_ok=True)
    children=[];handles=[];failure=None;completed=False;active=None;fault_at=None;body_stop_at=None;body_stop_verified=False
    x,y,yaw=map(float,origin);x-=math.sin(yaw)*args.ego_start_lateral;y+=math.cos(yaw)*args.ego_start_lateral;yaw+=args.ego_start_heading_offset;v=vy=rate=servo=0.;next_sensor=time.monotonic();last_step=next_sensor;start=next_sensor
    angles=np.linspace(-2.35619449,2.35619449,1081);world_heading=float(origin[2]);wc,ws=math.cos(world_heading),math.sin(world_heading)
    trajectory=[];minimum_body_separation=float("inf")
    try:
        for name in ['planner','controller']:
            log=(out/(name+'.log')).open('w');handles.append(log)
            children.append(subprocess.Popen([str(RELEASE/'install_cpp/cpp_frenet/lib/cpp_frenet'/('cpp_frenet_'+name)),
              '--ros-args','-p','assets_file:='+str(assets_for_tracker(RELEASE,args.tracker)),'-p','tracker:='+args.tracker],stdout=log,stderr=log,start_new_session=True))
        while time.monotonic()-start<22:
            now=time.monotonic();elapsed=0. if active is None else now-active
            if any(p.poll() is not None for p in children):raise RuntimeError('node_exited')
            remaining_dt=max(0.,now-last_step);last_step=now
            if remaining_dt>.5:raise RuntimeError('simulation_scheduler_stall')
            target,steering=command
            if active is not None:
                while remaining_dt>1e-9:
                    dt=min(.005,remaining_dt);remaining_dt-=dt
                    applied=0. if v<.03 and target<args.startup_command_deadzone else target
                    dv=-.6 if target==0 and v>.03 else float(np.clip((applied-v)/args.speed_response,-.8,2.))
                    v=max(0.,v+dt*dv);servo+=dt*(steering-servo)/args.servo_response
                    if v<.5:vy=0.;rate=v/.25*math.tan(servo)
                    else:
                        m,iz,lf,lr,cf,cr=3.74,.04712,.12,.13,90.,96.
                        dvy=-(cf+cr)/(m*v)*vy+(-v-(cf*lf-cr*lr)/(m*v))*rate+cf/m*servo
                        dr=-(cf*lf-cr*lr)/(iz*v)*vy-(cf*lf*lf+cr*lr*lr)/(iz*v)*rate+lf*cf/iz*servo
                        vy+=dt*dvy;rate+=dt*dr
                    x+=dt*(v*math.cos(yaw)-vy*math.sin(yaw));y+=dt*(v*math.sin(yaw)+vy*math.cos(yaw));yaw+=dt*rate
            if now>=next_sensor:
                next_sensor=now+.04;stamp=node.get_clock().now().to_msg()
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
                bodies=[(lead_x,lead_y,args.lead_length/2,args.lead_width/2)]
                if args.wheel_pair:
                    bodies=[(lead_x+sx*(args.lead_length/2-.05),lead_y+sy*(args.lead_width/2-.035),.05,.035)
                            for sx in (-1,1) for sy in (-1,1)]
                if args.no_obstacle:bodies=[]
                ranges=map_ranges(lx,ly,angles+yaw)
                with np.errstate(divide='ignore',invalid='ignore'):
                    for cx,cy,hl,hw in bodies:
                        ax=(cx-hl-local_x)/dx;bx=(cx+hl-local_x)/dx
                        ay=(cy-hw-local_y)/dy;by=(cy+hw-local_y)/dy
                        near=np.maximum(np.minimum(ax,bx),np.minimum(ay,by));far=np.minimum(np.maximum(ax,bx),np.maximum(ay,by))
                        ranges=np.minimum(ranges,np.where((far>=near)&(near>.05),near,np.inf))
                if active is not None and elapsed>1.2 and args.fault!='none':
                    if fault_at is None:fault_at=now
                    if args.fault=='target_loss':ranges[:]=np.inf
                if args.ego_returns and active is not None and elapsed>.6:
                    ego_beams=(angles>=math.radians(123.))&(angles<=math.radians(130.))
                    ranges[ego_beams]=.15+.010*np.sin(np.flatnonzero(ego_beams)*.7)
                scan.ranges=ranges.astype(np.float32).tolist()
                if not(args.fault=='scan_timeout' and fault_at is not None):pubs['scan'].publish(scan)
                pubs['quality'].publish(Bool(data=True));pubs['enable'].publish(Bool(data=active is not None and not (args.fault=='release' and fault_at is not None)))
                trajectory.append(dict(t=now-start,x=x,y=y,yaw=yaw,speed=v,command=command[:],lead_x=lead_x))
            if active is not None and not args.no_obstacle:
                ex=(x-origin[0])*wc+(y-origin[1])*ws
                ey=-(x-origin[0])*ws+(y-origin[1])*wc
                angle=yaw-world_heading;ec,es=math.cos(angle),math.sin(angle)
                ox=lead_x-ex;oy=lead_y-ey;hl=args.lead_length/2;hw=args.lead_width/2
                gaps=[abs(ox)-(.29*abs(ec)+.155*abs(es)+hl),
                      abs(oy)-(.29*abs(es)+.155*abs(ec)+hw),
                      abs(ox*ec+oy*es)-(.29+hl*abs(ec)+hw*abs(es)),
                      abs(-ox*es+oy*ec)-(.155+hl*abs(es)+hw*abs(ec))]
                separation=max(gaps)
                minimum_body_separation=min(minimum_body_separation,separation)
                if separation<=0:raise RuntimeError('independent_plant_body_collision')
            # Drain sensor/plan/status callbacks; one callback per raycast loop
            # creates a fictitious >1s actuator delay under diagnostic traffic.
            for _ in range(16):rclpy.spin_once(node,timeout_sec=0.)
            time.sleep(.001)
            if active is None and now-start>3. and records and records[-1].get('valid') and records[-1].get('reason')=='waiting_for_operator':
                active=now;print('synthetic target admitted; enabling simulated plant',flush=True)
            if active is not None and elapsed>.4 and records:
                latest=records[-1]
                if args.fault=='none' and not latest.get('valid'):
                    reason=latest.get('reason','unknown')
                    if args.allow_body_stop and (body_stop_at is not None or ('generic_' in reason or reason=='planner_invalid')):
                        if body_stop_at is None:body_stop_at=now
                    else:raise RuntimeError('controller_rejected:'+reason)
                if latest.get('complete'):
                    completed=True;break
                if body_stop_at is not None and now-body_stop_at>3.5:
                    late=[r for r in records if r['receipt']>body_stop_at+.65]
                    if not late or any(r.get('speed_command',0)!=0 for r in late):raise RuntimeError('body_stop_failed_to_hold_zero')
                    if v>.03:raise RuntimeError('body_stop_plant_not_stopped')
                    body_stop_verified=True;completed=True;break
                if fault_at is not None and now-fault_at>3.5:
                    late=[r for r in records if r['receipt']>fault_at+.65]
                    if not late or any(r.get('speed_command',0)!=0 for r in late):raise RuntimeError('fault_failed_to_stop')
                    if args.fault=='target_loss' and not any('generic_' in r.get('reason','') or 'planner_invalid' in r.get('reason','') for r in late):raise RuntimeError('wrong_stop_reason')
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
      release=str(RELEASE),no_obstacle=args.no_obstacle,lead_speed_mps=args.lead_speed,lead_stop_after_s=args.lead_stop_after,
      lead_lateral_center_m=args.lead_lateral_center,lead_lateral_jitter_m=args.lead_lateral_jitter,
      lead_start_distance_m=lead_start,ego_start_lateral_m=args.ego_start_lateral,ego_start_heading_offset_rad=args.ego_start_heading_offset,
      lead_width_m=args.lead_width,lead_length_m=args.lead_length,tyre_only_returns=args.wheel_pair)
    report['first_controller_rejection']=next((r.get('reason') for r in records if active is not None and r.get('receipt',0)>active and not r.get('valid')),None)
    report['synthetic_ego_returns']=args.ego_returns
    report['minimum_independent_sat_separation_m']=minimum_body_separation if math.isfinite(minimum_body_separation) else None
    report['tracker']=args.tracker
    report.update(plant_servo_response_s=args.servo_response,plant_speed_response_s=args.speed_response,plant_startup_command_deadzone_mps=args.startup_command_deadzone)
    report.update(allowed_guarded_body_stop=args.allow_body_stop,body_stop_verified=body_stop_verified,
                  route_completed=completed and failure is None and bool(records and records[-1].get('complete')) and not body_stop_verified)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (out/'records.json').write_text(json.dumps(dict(controller=records,planner=plans,plant=trajectory))+'\n')
    print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 1)
if __name__=='__main__':main()
