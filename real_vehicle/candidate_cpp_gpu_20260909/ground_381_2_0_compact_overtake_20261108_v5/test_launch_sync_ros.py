"""Synthetic ROS only: reproduce arm delay, verify no hidden acceleration, expire feedback."""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def main():
 assert os.environ.get('ROS_DOMAIN_ID')=='83' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
 import rclpy
 from nav_msgs.msg import Odometry
 from sensor_msgs.msg import LaserScan
 from std_msgs.msg import Bool,String
 from ackermann_msgs.msg import AckermannDriveStamped
 from stand_window import StandWindow
 ap=argparse.ArgumentParser();ap.add_argument('--old-release',type=Path);args=ap.parse_args()
 release=args.old_release.resolve() if args.old_release else ROOT
 assets=json.loads((release/'runtime_assets.json').read_text());route=assets['route'];length=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(route,route[1:]));x,y,yaw=route[0]
 rclpy.init();node=rclpy.create_node('launch_sync_regression')
 pub_pose=node.create_publisher(Odometry,'/baseline/localization_pose',1);pub_odom=node.create_publisher(Odometry,'/baseline/odom',1);pub_scan=node.create_publisher(LaserScan,'/baseline/scan',1);pub_quality=node.create_publisher(Bool,'/baseline/localization_valid',1);pub_enable=node.create_publisher(Bool,'/baseline/enable',1);pub_plan=node.create_publisher(String,'/baseline/local_plan_cpp',1);pub_window=node.create_publisher(String,'/stand_pf/window_status',1)
 records=[];outputs=[];nominal=[0.,0.];window=StandWindow();window.joy(False,time.monotonic())
 node.create_subscription(String,'/baseline/controller_status',lambda m:records.append(dict(receipt=time.monotonic(),**json.loads(m.data))),100)
 node.create_subscription(AckermannDriveStamped,'/baseline/drive_nominal',lambda m:nominal.__setitem__(slice(None),[m.drive.speed,m.drive.steering_angle]),10)
 label='old' if args.old_release else 'new';log=(ROOT/'run_logs'/f'launch_sync_{label}.log').open('w')
 cmd=[str(release/'install_cpp/cpp_frenet/lib/cpp_frenet/cpp_frenet_controller'),'--ros-args','-p','assets_file:='+str(release/'runtime_assets.json'),'-p','tracker:=dynamic_mpc']
 if not args.old_release:cmd+=['-p','require_motion_window:=true']
 proc=subprocess.Popen(cmd,stdout=log,stderr=log,start_new_session=True);begin=time.monotonic();plan_id=0;held=False
 try:
  while time.monotonic()-begin<8.:
   now=time.monotonic();elapsed=now-begin;enabled=elapsed>1.;permitted=elapsed>3.
   window.health(True,now);window.joy(enabled,now);window.permit(permitted,now);window.command(nominal[0],nominal[1],now);actual,angle,reason=window.evaluate(now)
   outputs.append(dict(t=elapsed,actual=actual,nominal=nominal[0],reason=reason))
   stamp=node.get_clock().now();seconds=stamp.nanoseconds*1e-9
   pose=Odometry();pose.header.stamp=stamp.to_msg();pose.header.frame_id='map';pose.child_frame_id='base_link';pose.pose.pose.position.x=x;pose.pose.pose.position.y=y;pose.pose.pose.orientation.z=math.sin(yaw/2);pose.pose.pose.orientation.w=math.cos(yaw/2);pub_pose.publish(pose);pose.header.frame_id='odom';pub_odom.publish(pose)
   pub_quality.publish(Bool(data=True));pub_enable.publish(Bool(data=enabled))
   scan=LaserScan();scan.header.stamp=stamp.to_msg();scan.header.frame_id='laser';scan.angle_min=-2.;scan.angle_max=2.;scan.angle_increment=.04;scan.range_min=.05;scan.range_max=10.;scan.ranges=[5.]*101;pub_scan.publish(scan)
   plan_id+=1;pub_plan.publish(String(data=json.dumps(dict(valid=True,reason='ok',path=route,generated_mono=now,plan_id=plan_id,input_pose_stamp=seconds,input_scan_stamp=seconds,lap_progress_m=0.,lap_length_m=length,state='FOLLOWING',obstacle_points=[],frame_id='map',route_mode='open_stop_at_end'))))
   # Continue delivery of an old source stamp after 6 s; receipt alone cannot renew it.
   pub_window.publish(String(data=json.dumps(dict(published_mono=now if elapsed<6 else begin+6,started=window.started,latched=bool(window.latched)))))
   until=now+.02
   while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=min(.003,until-time.monotonic()))
   if proc.poll() is not None:raise RuntimeError('controller exited')
  preview=[r for r in records if 1.5<r['receipt']-begin<2.9 and r['valid']]
  assert preview
  preview_max=max(r['speed_command'] for r in preview)
  if args.old_release:
   assert preview_max>.5,preview_max
   summary=dict(passed=True,old_unapplied_ramp_reproduced=True,preview_max_mps=preview_max,physical_motion=False)
  else:
   assert preview_max<=.01001,preview_max
   active=[r for r in outputs if 3.3<r['t']<5.8];assert max(r['actual'] for r in active)>.5
   late=[r for r in records if r['receipt']-begin>6.65];assert late and all(not r['valid'] and r['speed_command']==0 for r in late)
   assert any(r['reason']=='motion_window_stale' for r in late)
   first=next(r for r in outputs if r['actual']>0);assert first['actual']<=.01001
   summary=dict(passed=True,preview_max_mps=preview_max,first_actual_command_mps=first['actual'],post_launch_max_mps=max(r['actual'] for r in active),stale_source_stops=True,physical_motion=False)
  (ROOT/'run_logs'/f'launch_sync_{label}.json').write_text(json.dumps(dict(summary=summary,records=records,outputs=outputs),indent=2)+'\n');print(json.dumps(summary),flush=True)
 finally:
  pub_enable.publish(Bool(data=False))
  if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=5)
  log.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
