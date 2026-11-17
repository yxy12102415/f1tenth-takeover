"""Capture one static multiview dataset. Only a LiDAR driver and subscriber run."""
import argparse,json,math,os,signal,subprocess,time,hashlib
from pathlib import Path

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('view',choices=['background','rear','rear_left','rear_right','left_side','right_side','front_left','front','front_right']);p.add_argument('--duration',type=float,default=5.);args=p.parse_args()
 if not 4.9<=args.duration<=15.:p.error('Duration must be 5–15 seconds')
 if os.environ.get('ROS_DOMAIN_ID')!='86' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise RuntimeError('Use isolated ROS domain 86 and localhost only')
 root=Path(__file__).resolve().parent;out=root/args.view
 if out.exists():raise RuntimeError('Capture exists; preserve it and choose a new session directory')
 # Do not start a second scanner session or collect alongside a motion entry.
 for entry in Path('/proc').iterdir():
  if not entry.name.isdigit() or int(entry.name)==os.getpid():continue
  try:cmd=(entry/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='ignore')
  except OSError:continue
  if 'python' in cmd and 'stand_follow.py' in cmd:raise RuntimeError('Vehicle session still running')
  if '/urg_node/urg_node_driver' in cmd:raise RuntimeError('Existing LiDAR driver detected; stop it before capture')
 import rclpy
 from sensor_msgs.msg import LaserScan
 from rclpy.qos import qos_profile_sensor_data
 out.mkdir();rclpy.init();node=rclpy.create_node('lead_shell_readonly_capture');rows=[];driver=None;error=None;data=out/'scans.jsonl';handle=data.open('w');log=(out/'lidar.log').open('w')
 def receive(m):
  stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  if rows and stamp<=rows[-1]['stamp']:return
  row=dict(stamp=stamp,received_mono=time.monotonic(),frame_id=m.header.frame_id,angle_min=float(m.angle_min),angle_increment=float(m.angle_increment),range_min=float(m.range_min),range_max=float(m.range_max),ranges=[float(x) if math.isfinite(x) else None for x in m.ranges])
  rows.append(row);handle.write(json.dumps(row,separators=(',',':'))+'\n')
 sub=node.create_subscription(LaserScan,'/shell_capture/scan',receive,qos_profile_sensor_data)
 try:
  driver=subprocess.Popen(['/opt/ros/foxy/lib/urg_node/urg_node_driver','--ros-args','-r','scan:=/shell_capture/scan','-p','ip_address:=192.168.0.10','-p','laser_frame_id:=laser','-p','calibrate_time:=false'],stdout=log,stderr=log,start_new_session=True)
  deadline=time.monotonic()+args.duration+15
  while time.monotonic()<deadline:
   rclpy.spin_once(node,timeout_sec=.1)
   if len(rows)>1 and rows[-1]['stamp']-rows[0]['stamp']>=args.duration:break
   if driver.poll() is not None:raise RuntimeError('LiDAR driver exited')
  if len(rows)<100 or rows[-1]['stamp']-rows[0]['stamp']<args.duration-.1:raise RuntimeError('Insufficient scans/duration')
  if max(b['stamp']-a['stamp'] for a,b in zip(rows,rows[1:]))>.15:raise RuntimeError('Scan gap exceeds 150 ms; retain capture for review')
 except Exception as e:error=str(e)
 finally:
  if driver and driver.poll() is None:
   os.killpg(driver.pid,signal.SIGINT)
   try:driver.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(driver.pid,signal.SIGTERM);driver.wait(timeout=5)
  handle.close();log.close();node.destroy_node();rclpy.shutdown()
 counts=[]
 for row in rows:
  points=[]
  for k,v in enumerate(row['ranges']):
   if v is None or not row['range_min']<=v<=min(row['range_max'],10.):continue
   a=row['angle_min']+k*row['angle_increment'];x,y=v*math.cos(a),v*math.sin(a)
   if .7<=x<=2.3 and abs(y)<=.65:points.append((x,y))
  counts.append(len(points))
 report=dict(passed=error is None,error=error,view=args.view,source='real_LaserScan',scans=len(rows),duration_s=rows[-1]['stamp']-rows[0]['stamp'] if len(rows)>1 else 0,max_scan_gap_s=max((b['stamp']-a['stamp'] for a,b in zip(rows,rows[1:])),default=None),roi='laser x=[0.7,2.3], |y|<=0.65 m; presence only, not identity',roi_returns_median=sorted(counts)[len(counts)//2] if counts else 0,body_shell='User reports new shell installed; dimensions not yet measured',ego_fixed_required=True,target_identity_verified=False,actuator_nodes_started=0,scans_sha256=hashlib.sha256(data.read_bytes()).hexdigest())
 (out/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
 if error:raise RuntimeError(error)
if __name__=='__main__':main()
