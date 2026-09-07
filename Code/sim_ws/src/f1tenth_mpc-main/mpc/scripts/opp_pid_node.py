#!/usr/bin/env python3
import csv
import math
import os
from dataclasses import dataclass

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from scipy.spatial.transform import Rotation


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_odom(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    quat = Rotation.from_quat([q.x, q.y, q.z, q.w])
    return quat.as_euler("zxy", degrees=False)[0]


@dataclass
class PIDState:
    integral: float = 0.0
    previous_error: float = 0.0


class OppPIDNode(Node):
    def __init__(self) -> None:
        super().__init__("opp_pid_node")

        share_directory = get_package_share_directory("mpc")
        default_waypoints_path = os.path.join(share_directory, "waypoints", "Melbourne_map_mpc.csv")

        self.declare_parameter("waypoints_path", default_waypoints_path)
        self.declare_parameter("pose_topic", "/opp_racecar/odom")
        self.declare_parameter("drive_topic", "/opp_drive")
        self.declare_parameter("lookahead_distance", 1.2)
        self.declare_parameter("max_speed", 4.0)
        self.declare_parameter("min_speed", 0.2)
        self.declare_parameter("max_steer", 0.4189)
        self.declare_parameter("kp_steer", 1.4)
        self.declare_parameter("ki_steer", 0.0)
        self.declare_parameter("kd_steer", 0.15)
        self.declare_parameter("kp_speed", 1.0)
        self.declare_parameter("ki_speed", 0.0)
        self.declare_parameter("kd_speed", 0.05)
        self.declare_parameter("max_accel", 1.0)
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("scenario", "constant_speed")
        self.declare_parameter("scenario_period", 12.0)
        self.declare_parameter("constant_speed", 1.6)
        self.declare_parameter("turning_speed", 1.2)
        self.declare_parameter("stop_duration", 3.0)

        self.waypoints_path = self.get_parameter("waypoints_path").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.drive_topic = self.get_parameter("drive_topic").value
        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.min_speed = float(self.get_parameter("min_speed").value)
        self.max_steer = float(self.get_parameter("max_steer").value)
        self.kp_steer = float(self.get_parameter("kp_steer").value)
        self.ki_steer = float(self.get_parameter("ki_steer").value)
        self.kd_steer = float(self.get_parameter("kd_steer").value)
        self.kp_speed = float(self.get_parameter("kp_speed").value)
        self.ki_speed = float(self.get_parameter("ki_speed").value)
        self.kd_speed = float(self.get_parameter("kd_speed").value)
        self.max_accel = float(self.get_parameter("max_accel").value)
        self.dt = float(self.get_parameter("dt").value)
        self.scenario = str(self.get_parameter("scenario").value)
        self.scenario_period = max(float(self.get_parameter("scenario_period").value), 2.0)
        self.constant_speed = float(self.get_parameter("constant_speed").value)
        self.turning_speed = float(self.get_parameter("turning_speed").value)
        self.stop_duration = max(float(self.get_parameter("stop_duration").value), 0.5)
        self.scenario_start = self.get_clock().now()

        self.waypoints = self.load_waypoints(self.waypoints_path)
        self.steer_pid = PIDState()
        self.speed_pid = PIDState()
        self.get_logger().info(
            f"Loaded {len(self.waypoints)} raceline waypoints from {self.waypoints_path}; "
            f"subscribing to {self.pose_topic}, publishing {self.drive_topic}"
        )

        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.create_subscription(Odometry, self.pose_topic, self.pose_callback, 10)

    def load_waypoints(self, path: str) -> np.ndarray:
        points = []
        with open(path, "r") as file:
            reader = csv.reader(file)
            for row in reader:
                if not row or row[0].startswith("#"):
                    continue
                if len(row) < 4:
                    continue
                points.append([float(row[0]), float(row[1]), float(row[2]), float(row[3])])

        if not points:
            raise RuntimeError(f"No waypoints loaded from {path}")

        return np.array(points)

    def pose_callback(self, odom: Odometry) -> None:
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        yaw = yaw_from_odom(odom)
        current_speed = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)

        nearest_idx = self.find_nearest_index(x, y)
        target_idx = self.find_lookahead_index(nearest_idx, x, y)
        target = self.waypoints[target_idx]

        target_heading = math.atan2(target[1] - y, target[0] - x)
        heading_error = normalize_angle(target_heading - yaw)
        steering = self.pid_step(
            heading_error,
            self.steer_pid,
            self.kp_steer,
            self.ki_steer,
            self.kd_steer,
            -self.max_steer,
            self.max_steer,
        )

        elapsed = (self.get_clock().now() - self.scenario_start).nanoseconds * 1e-9
        target_speed = self.scenario_target_speed(elapsed, float(target[3]))
        speed_error = target_speed - current_speed
        speed_delta = self.pid_step(
            speed_error,
            self.speed_pid,
            self.kp_speed,
            self.ki_speed,
            self.kd_speed,
            -self.max_accel * self.dt,
            self.max_accel * self.dt,
        )
        output_min_speed = 0.0 if target_speed < self.min_speed else self.min_speed
        speed = min(max(current_speed + speed_delta, output_min_speed), self.max_speed)

        drive = AckermannDriveStamped()
        drive.drive.steering_angle = float(steering)
        drive.drive.speed = float(speed)
        self.drive_pub.publish(drive)

    def scenario_target_speed(self, elapsed: float, waypoint_speed: float) -> float:
        phase = (elapsed % self.scenario_period) / self.scenario_period
        if self.scenario == "constant_speed":
            speed = self.constant_speed
        elif self.scenario == "accel_brake":
            triangle = 1.0 - abs(2.0 * phase - 1.0)
            speed = self.min_speed + triangle * (self.max_speed - self.min_speed)
        elif self.scenario == "turning":
            speed = min(self.turning_speed, waypoint_speed)
        elif self.scenario == "stop_go":
            moving_duration = max(self.scenario_period - self.stop_duration, 0.5)
            speed = self.constant_speed if (elapsed % self.scenario_period) < moving_duration else 0.0
        else:
            self.get_logger().warn(f"Unknown scenario '{self.scenario}', using waypoint speed")
            speed = waypoint_speed
        return float(np.clip(speed, 0.0, self.max_speed))

    def pid_step(
        self,
        error: float,
        state: PIDState,
        kp: float,
        ki: float,
        kd: float,
        lower: float,
        upper: float,
    ) -> float:
        state.integral += error * self.dt
        derivative = (error - state.previous_error) / self.dt
        state.previous_error = error
        command = kp * error + ki * state.integral + kd * derivative
        return float(np.clip(command, lower, upper))

    def find_nearest_index(self, x: float, y: float) -> int:
        diff = self.waypoints[:, :2] - np.array([x, y])
        distances = np.hypot(diff[:, 0], diff[:, 1])
        return int(np.argmin(distances))

    def find_lookahead_index(self, start_idx: int, x: float, y: float) -> int:
        idx = start_idx
        for step in range(len(self.waypoints)):
            idx = (start_idx + step) % len(self.waypoints)
            dx = self.waypoints[idx, 0] - x
            dy = self.waypoints[idx, 1] - y
            if math.hypot(dx, dy) >= self.lookahead_distance:
                break
        return idx


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OppPIDNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ignore only the normal ros2-launch context invalidation race.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
