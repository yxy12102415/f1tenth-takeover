"""Non-actuating endpoint test of the actual ROS C++ controller + StandWindow.
Uses a small independent kinematic plant, synthetic planner/odom/scan streams;
no driver, actuator guard, map server, or physical motion. Domain 83 only.
"""
import json,math,os,signal,subprocess,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent

def main():
 assert os.environ.get('ROS_DOMAIN_ID')=='83' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
 import rclpy
 from nav_msgs.msg import Odometry
 from sensor_msgs.msg import LaserScan
 from std_msgs.msg import Bool,String
 from ackermann_msgs.msg import AckermannDriveStamped
 from stand_window import StandWindow
 tracker=os.environ.get('FRENET_TRACKER','dynamic_mpc'); prefix='terminal_ros_'+tracker+'_'
 assets=json.loads((ROOT/'runtime_assets.json').read_text());route=np.array(assets['route']);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(route[:,:2],axis=0),axis=1))];reports=[]
 for name in ('normal','zero_feedback','wrong_endpoint'):
  rclpy.init();node=rclpy.create_node('synthetic_terminal_'+name);pub_pose=node.create_publisher(Odometry,'/baseline/localization_pose',1);pub_odom=node.create_publisher(Odometry,'/baseline/odom',1);pub_scan=node.create_publisher(LaserScan,'/baseline/scan',1);pub_quality=node.create_publisher(Bool,'/baseline/localization_valid',1);pub_enable=node.create_publisher(Bool,'/baseline/enable',1);pub_plan=node.create_publisher(String,'/baseline/local_plan_cpp',1)
  statuses=[];nominal=[0.,0.];actual=[];window=StandWindow();window.joy(False,time.monotonic());v=servo=0.;s0=arc[-1]-1.8;x=float(np.interp(s0,arc,route[:,0]));y=float(np.interp(s0,arc,route[:,1]));yaw=float(np.interp(s0,arc,np.unwrap(route[:,2])));cut=None;started=None;ready_at=None;complete_at=None;failure=None;pid=0;tick=0
  node.create_subscription(String,'/baseline/controller_status',lambda m:statuses.append(dict(received=time.monotonic(),**json.loads(m.data))),100)
  node.create_subscription(AckermannDriveStamped,'/baseline/drive_nominal',lambda m:nominal.__setitem__(slice(None),[float(m.drive.speed),float(m.drive.steering_angle)]),10)
  log=(ROOT/'run_logs'/(prefix+name+'.log')).open('w');proc=subprocess.Popen([str(ROOT/'install_cpp/cpp_frenet/lib/cpp_frenet/cpp_frenet_controller'),'--ros-args','-p','assets_file:='+str(ROOT/'runtime_assets.json'),'-p','tracker:='+os.environ.get('FRENET_TRACKER','dynamic_mpc')],stdout=log,stderr=log,start_new_session=True)
  begin=time.monotonic();previous=begin
  try:
   while time.monotonic()-begin<18:
    now=time.monotonic();dt=min(.04,max(.001,now-previous));previous=now
    enabled=now-begin>1.5 and window.latched is None
    window.health(True,now);window.permit(enabled,now);window.joy(enabled,now);window.command(nominal[0] if enabled else 0.,nominal[1] if enabled else 0.,now)
    command,angle,reason=window.evaluate(now)
    if command>0 and started is None:started=now
    if started and window.latched and cut is None:cut=dict(time=now,speed=v,reason=window.latched,x=x,y=y)
    for _ in range(4):
     h=dt/4
     v=max(0.,v+max(-.8,min(1.,(command-v)/.2))*h) if command>0 else max(0.,v-.6*h)
     servo+=(angle-servo)*min(1.,h/.10);rate=v/.25*math.tan(servo);x+=h*v*math.cos(yaw+.5*h*rate);y+=h*v*math.sin(yaw+.5*h*rate);yaw+=h*rate
    stamp=node.get_clock().now();seconds=stamp.nanoseconds*1e-9
    shifted=.25 if name=='wrong_endpoint' and cut else 0.
    pose=Odometry();pose.header.stamp=stamp.to_msg();pose.header.frame_id='map';pose.child_frame_id='base_link';pose.pose.pose.position.x=x+shifted;pose.pose.pose.position.y=y;pose.pose.pose.orientation.z=math.sin(yaw/2);pose.pose.pose.orientation.w=math.cos(yaw/2)
    tail_start=int(np.searchsorted(arc,arc[-1]-3.0));nearest=tail_start+int(np.argmin(np.sum((route[tail_start:,:2]-[x,y])**2,axis=1)));remaining=float(arc[-1]-arc[nearest])
    measured=0. if name=='zero_feedback' and remaining<.65 and not cut else v
    pose.twist.twist.linear.x=measured;pose.twist.twist.angular.z=v/.25*math.tan(servo)
    pub_pose.publish(pose);pose.header.frame_id='odom';pub_odom.publish(pose);pub_quality.publish(Bool(data=True));pub_enable.publish(Bool(data=enabled))
    scan=LaserScan();scan.header.stamp=stamp.to_msg();scan.header.frame_id='laser';scan.angle_min=-2.;scan.angle_max=2.;scan.angle_increment=.04;scan.range_min=.05;scan.range_max=10.;scan.ranges=[5.]*101;pub_scan.publish(scan)
    pid+=1;plan=dict(valid=True,reason='ok',path=route[tail_start:].tolist(),generated_mono=now,plan_id=pid,input_pose_stamp=seconds,input_scan_stamp=seconds,lap_progress_m=float(arc[nearest]),lap_length_m=float(arc[-1]),state='FOLLOWING',obstacle_points=[],frame_id='map',route_mode='closed_single_lap_stop')
    pub_plan.publish(String(data=json.dumps(plan)));rclpy.spin_once(node,timeout_sec=.001)
    until=now+.02
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=min(.003,until-time.monotonic()))
    actual.append(dict(t=now-begin,x=x,y=y,yaw=yaw,speed=v,command=command,nominal=nominal[0],window_reason=reason,measured=measured))
    if statuses and statuses[-1].get('complete'):
     complete_at=now;break
    if cut and statuses and statuses[-1].get('reason')=='terminal_stop_outside_endpoint':failure='terminal_stop_outside_endpoint';break
    if proc.poll() is not None:raise RuntimeError('controller_exited')
   active=[r for r in actual if r['command']>=.5];assert active
   first_floor=active[0]['t'];positive_after=[r['command'] for r in actual if r['t']>=first_floor and r['command']>0]
   assert all(vv>=.5-1e-6 for vv in positive_after)
   assert cut is not None and cut['reason']=='upstream_stop'
   assert all(r['command']==0 for r in actual if r['t']+begin>=cut['time'])
   distance=math.hypot(x-route[-1,0],y-route[-1,1]);heading=math.atan2(math.sin(yaw-route[-1,2]),math.cos(yaw-route[-1,2]))
   if name=='wrong_endpoint':assert complete_at is None and failure=='terminal_stop_outside_endpoint'
   else:assert complete_at is not None and v<=.03 and distance<=.10 and abs(heading)<=.20,(name,distance,heading,v,statuses[-1])
   report=dict(case=name,passed=True,route_complete=complete_at is not None,failure=failure,final_distance_m=distance,final_heading_rad=heading,completion_speed_mps=v,coast_cut_speed_mps=cut['speed'],minimum_actual_positive_after_start_mps=min(positive_after),zero_latched=True,standstill_completion_verified=name!='wrong_endpoint')
   reports.append(report);print(json.dumps(report),flush=True)
  finally:
   pub_enable.publish(Bool(data=False))
   if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=5)
   log.close();node.destroy_node();rclpy.shutdown()
   (ROOT/'run_logs'/(prefix+name+'.trace.json')).write_text(json.dumps(dict(statuses=statuses,plant=actual))+'\n')
 (ROOT/'run_logs'/(prefix+'summary.json')).write_text(json.dumps(reports,indent=2)+'\n')
if __name__=='__main__':main()
