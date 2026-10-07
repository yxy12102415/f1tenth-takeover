"""Actual driver measurements. Motor temperature is logged, not an interlock."""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vesc_msgs.msg import VescStateStamped


def main():
    rclpy.init();node=Node('stand_telemetry_bridge',start_parameter_services=False)
    out=node.create_publisher(String,'/stand_pf/health',1)
    def receive(msg):
        s=msg.state
        data=dict(erpm=s.speed,fault=s.fault_code,motor_current=s.current_motor,
            input_current=s.current_input,duty_cycle=s.duty_cycle,voltage=s.voltage_input,
            fet_temp=s.temp_fet,motor_temp=s.temp_motor,motor_temperature_used=False,
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            bridge_ros=node.get_clock().now().nanoseconds*1e-9,bridge_mono=time.monotonic())
        out.publish(String(data=json.dumps(data)))
    node.create_subscription(VescStateStamped,'/stand_pf/telemetry',receive,1)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
