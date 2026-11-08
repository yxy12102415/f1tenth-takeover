#!/usr/bin/env python3
"""Isolated ROS startup/zero-output/topic audit; never opens a driver."""
import json,math,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
D=Path(__file__).resolve().parent
assert os.environ.get('ROS_DOMAIN_ID')=='187','isolated test domain required'
rclpy.init();node=Node('overtake_shadow_smoke_monitor');samples=[]
node.create_subscription(AckermannDriveStamped,'/overtake_381/proposed_drive',lambda m:samples.append([m.drive.speed,m.drive.steering_angle]),10)
f=(D/'reports/shadow_smoke.log').open('w')
p=subprocess.Popen(['bash',str(D/'shadow.sh'),'--duration','30'],stdout=f,stderr=f)
forbidden=['/drive','/drive_nominal','/baseline/final','/stand_pf/motor_speed','/commands/motor/speed']
audit={}
try:
 end=time.monotonic()+18.
 while time.monotonic()<end and p.poll() is None:rclpy.spin_once(node,timeout_sec=.1)
 audit={t:len(node.get_publishers_info_by_topic(t)) for t in forbidden}
 report={'domain':187,'no_hardware_inputs':True,'proposal_count':len(samples),'all_proposals_zero':bool(samples) and all(math.isfinite(x) and x==0. for row in samples for x in row),'forbidden_topic_publishers':audit,'process_alive':p.poll() is None,'physical_motion_tested':False,'scope':'Startup/zero-output/topic isolation only; not sensor, trajectory, timing or ground acceptance.'}
 report['passed']=report['all_proposals_zero'] and report['proposal_count']>=5 and report['process_alive'] and not any(audit.values())
finally:
 if p.poll() is None:p.send_signal(signal.SIGINT)
 try:p.wait(timeout=15)
 except subprocess.TimeoutExpired:p.kill();p.wait()
 f.close();node.destroy_node();rclpy.shutdown()
(D/'reports/shadow_smoke.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
