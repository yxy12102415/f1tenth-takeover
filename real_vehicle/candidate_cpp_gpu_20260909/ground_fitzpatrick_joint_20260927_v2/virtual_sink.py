"""Offline current/speed-mode sink with an independent 150 ms watchdog."""
import math
import time
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

MAX_VIRTUAL_ERPM=2.00*7500.


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='83':raise RuntimeError('Virtual sink only in offline domain')
    rclpy.init();node=Node('stand_virtual_sink');value=[0.];received=[None]
    out=node.create_publisher(Float64,'/stand_pf/effective_rpm',10)
    def speed(msg):
        value[0]=msg.data if math.isfinite(msg.data) and 0.<=msg.data<=MAX_VIRTUAL_ERPM else 0.
        received[0]=time.monotonic()
    def current(msg):value[0]=0.;received[0]=time.monotonic()
    node.create_subscription(Float64,'/stand_pf/motor_speed',speed,10)
    node.create_subscription(Float64,'/stand_pf/motor_current',current,10)
    def tick():
        if received[0] is None or time.monotonic()-received[0]>.15:value[0]=0.
        out.publish(Float64(data=value[0]))
    node.create_timer(.01,tick)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
