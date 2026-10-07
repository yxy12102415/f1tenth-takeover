"""VESC message-only adapter, run with a narrowly scoped legacy message path."""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vesc_msgs.msg import VescStateStamped


def main():
    rclpy.init();node=Node('stand_telemetry_bridge',start_parameter_services=False);out=node.create_publisher(String,'/stand_pf/health',1)
    def callback(msg):
        out.publish(String(data=json.dumps({'erpm':msg.state.speed,'fault':msg.state.fault_code,
            'voltage':msg.state.voltage_input,'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            'bridge_ros':node.get_clock().now().nanoseconds*1e-9,'bridge_mono':time.monotonic()})))
    node.create_subscription(VescStateStamped,'/stand_pf/telemetry',callback,1)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
