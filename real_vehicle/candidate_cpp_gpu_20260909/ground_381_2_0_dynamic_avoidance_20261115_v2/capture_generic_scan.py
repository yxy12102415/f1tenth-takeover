"""Read-only URG capture: no VESC, no actuator/control topics."""
import json,math,os,signal,subprocess,time
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='86' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
r=Path(__file__).resolve().parent;out=r/'reports/live_scan_capture';out.mkdir(parents=True,exist_ok=True)
rclpy.init();n=rclpy.create_node('generic_readonly_scan_capture');scans=[]
def receive(m):
 scans.append(dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,angle_min=float(m.angle_min),angle_increment=float(m.angle_increment),range_min=float(m.range_min),range_max=float(m.range_max),ranges=[float(v) if math.isfinite(v) else None for v in m.ranges]))
n.create_subscription(LaserScan,'/generic_capture/scan',receive,qos_profile_sensor_data)
f=(out/'driver.log').open('w');p=subprocess.Popen(['/opt/ros/foxy/lib/urg_node/urg_node_driver','--ros-args','-r','scan:=/generic_capture/scan','-p','ip_address:=192.168.0.10','-p','laser_frame_id:=laser','-p','calibrate_time:=false'],stdout=f,stderr=f,start_new_session=True)
try:
 end=time.monotonic()+15
 while time.monotonic()<end and len(scans)<80:rclpy.spin_once(n,timeout_sec=.1)
 assert len(scans)>=40,'Insufficient live scans'
finally:
 os.killpg(p.pid,signal.SIGINT);p.wait(timeout=5);f.close();n.destroy_node();rclpy.shutdown()
(out/'scans.json').write_text(json.dumps(scans,separators=(',',':'))+'\n')
print('Captured',len(scans),'real scans; no motor driver or commands.')
