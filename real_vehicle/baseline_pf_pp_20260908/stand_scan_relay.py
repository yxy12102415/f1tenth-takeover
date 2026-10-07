"""Independent latest-scan relay. No display, MCL, map IO or vehicle output."""
import json
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from baseline_pf_pp.safe_pf import latest_sensor_qos, isolated_domain


class ScanRelay(Node):
    def __init__(self):
        super().__init__('stand_scan_relay',start_parameter_services=False)
        self.last_forward=None;self.last_sample={};self.count=0
        self.out=self.create_publisher(LaserScan,'/baseline/scan',latest_sensor_qos())
        self.status=self.create_publisher(String,'/baseline/scan_relay_status',1)
        self.create_subscription(LaserScan,'/baseline/raw_scan',self.scan,latest_sensor_qos())
        self.create_timer(.2,self.report)

    def scan(self,msg):
        now=time.monotonic()
        if self.last_forward is not None and now-self.last_forward<.045:return
        self.last_forward=now
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        ros=self.get_clock().now().nanoseconds*1e-9
        self.out.publish(msg)  # preserve every source field, including header
        self.count+=1
        self.last_sample={'source_stamp':stamp,'relay_receive_ros':ros,
                          'input_source_age_s':ros-stamp,'callback_publish_s':time.monotonic()-now}

    def report(self):
        self.status.publish(String(data=json.dumps(dict(self.last_sample,count=self.count))))


def main():
    isolated_domain();rclpy.init();node=ScanRelay()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
