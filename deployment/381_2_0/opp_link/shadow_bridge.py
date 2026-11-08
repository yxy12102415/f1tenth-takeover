#!/usr/bin/env python3
"""Observe real F1 permits/downstream output; send only to inhibited firmware.

No ROS publishers, no VESC writes, no F1 permission override. This is NOT the
production two-vehicle supervisor; reciprocal stop integration is pending.
"""
import argparse
import json
import select
import time
from protocol import Link
from shadow_gate import Gate

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--ip',required=True)
    args=parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Bool,String
    link=Link(args.ip);link.discover();gate=Gate()
    rclpy.init();node=Node('opp_inhibited_shadow_bridge',start_parameter_services=False)
    previous=[None]
    def boolean(key,msg):gate.update(key,msg.data,time.monotonic())
    def status(key,msg):
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):data=None
        gate.update(key,data,time.monotonic())
    def joy(msg):
        age=node.get_clock().now().nanoseconds*1e-9-(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
        valid=-.02<=age<=.25 and len(msg.buttons)>4
        gate.update('joy',bool(msg.buttons[4]) if valid else None,time.monotonic())
    node.create_subscription(Joy,'/baseline/joy',joy,qos_profile_sensor_data)
    for key,topic in [('permit','/stand_pf/permit'),('health','/stand_pf/health_permit')]:
        node.create_subscription(Bool,topic,lambda m,k=key:boolean(k,m),1)
    for key,topic in [('window','/stand_pf/window_status'),('actuator','/stand_pf/actuator_status')]:
        node.create_subscription(String,topic,lambda m,k=key:status(k,m),1)
    def tick():
        for _ in range(100):
            if not select.select([link.sock],[],[],0)[0]:break
            try:link.receive()
            except ValueError:continue
        now=time.monotonic()
        healthy=link.last_ack_at is not None and now-link.last_ack_at<=.25 and link.latest['state'] in (2,3)
        enabled,reason=gate.evaluate(now,healthy)
        link.command(enabled,1100 if enabled else 1000)
        report=(enabled,reason,link.latest['state'])
        if report!=previous[0]:
            print(json.dumps({'would_enable':enabled,'reason':reason,'receiver_state':link.latest['state'],
                              'physical_output_enabled':False,'f1_reverse_interlock_integrated':False}),flush=True)
            previous[0]=report
    node.create_timer(.02,tick)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        link.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
