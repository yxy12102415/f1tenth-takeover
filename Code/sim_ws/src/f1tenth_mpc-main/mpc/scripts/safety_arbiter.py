#!/usr/bin/env python3
import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class SafetyArbiter(Node):
    def __init__(self) -> None:
        super().__init__('safety_arbiter')

        self.declare_parameter('nominal_topic', '/drive_nominal')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('nominal_timeout', 0.2)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('max_speed', 3.0)
        self.declare_parameter('max_steer', 0.5)
        self.declare_parameter('recovery_valid_commands', 3)

        nominal_topic = self.get_parameter('nominal_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        self.nominal_timeout = float(self.get_parameter('nominal_timeout').value)
        publish_rate = max(float(self.get_parameter('publish_rate').value), 1.0)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.recovery_valid_commands = max(
            1, int(self.get_parameter('recovery_valid_commands').value)
        )

        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.create_subscription(
            AckermannDriveStamped,
            nominal_topic,
            self.nominal_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_final_command)

        self.latest_nominal = None
        self.latest_nominal_stamp = None
        self.valid_command_count = 0
        self.last_stop_reason = None

    def nominal_callback(self, msg: AckermannDriveStamped) -> None:
        speed = float(msg.drive.speed)
        steering = float(msg.drive.steering_angle)
        if not math.isfinite(speed) or not math.isfinite(steering):
            self.latest_nominal = None
            self.latest_nominal_stamp = None
            self.valid_command_count = 0
            self.publish_safe_stop('invalid nominal command')
            return
        if speed < 0.0 or speed > self.max_speed + 1e-6:
            self.latest_nominal = None
            self.latest_nominal_stamp = None
            self.valid_command_count = 0
            self.publish_safe_stop('nominal speed outside safety limits')
            return
        if abs(steering) > self.max_steer + 1e-6:
            self.latest_nominal = None
            self.latest_nominal_stamp = None
            self.valid_command_count = 0
            self.publish_safe_stop('nominal steering outside safety limits')
            return

        self.latest_nominal = msg
        self.latest_nominal_stamp = self.get_clock().now()
        self.valid_command_count += 1

    def publish_final_command(self) -> None:
        if self.latest_nominal is None or self.latest_nominal_stamp is None:
            self.publish_safe_stop('no nominal command')
            return

        age = (self.get_clock().now() - self.latest_nominal_stamp).nanoseconds * 1e-9
        if age > self.nominal_timeout:
            self.valid_command_count = 0
            self.publish_safe_stop('nominal command timeout')
            return

        if self.valid_command_count < self.recovery_valid_commands:
            self.publish_safe_stop(
                f'waiting for {self.recovery_valid_commands} fresh nominal commands'
            )
            return

        output = AckermannDriveStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = 'ego_racecar/base_link'
        output.drive = self.latest_nominal.drive
        self.drive_pub.publish(output)
        self.last_stop_reason = None

    def publish_safe_stop(self, reason: str) -> None:
        stop = AckermannDriveStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.header.frame_id = 'ego_racecar/base_link'
        stop.drive.speed = 0.0
        stop.drive.steering_angle = 0.0
        stop.drive.acceleration = 0.0
        self.drive_pub.publish(stop)
        if reason != self.last_stop_reason:
            self.get_logger().warn(f'SAFETY STOP: {reason}')
            self.last_stop_reason = reason


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyArbiter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ignore only the normal ros2-launch context invalidation race.
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.publish_safe_stop('safety arbiter shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
