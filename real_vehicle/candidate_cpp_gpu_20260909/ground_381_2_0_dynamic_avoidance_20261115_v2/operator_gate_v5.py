"""PS4 hold-to-run arm/deadman source for one autonomous route attempt."""

import json
import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String


class RouteOperatorGate(Node):
    def __init__(self) -> None:
        super().__init__('route_operator_gate')
        self.declare_parameter('joy_topic', '/stage1_hardware/joy')
        self.declare_parameter('arm_topic', '/route_test/arm')
        self.declare_parameter('deadman_topic', '/route_test/deadman')
        self.declare_parameter('status_topic', '/route_test/operator_status')
        self.declare_parameter('enable_topic', '/route_test/enable')
        self.declare_parameter('nominal_topic', '/route_test/drive_nominal')
        self.declare_parameter('planner_valid_topic', '/route_test/planner_valid')
        self.declare_parameter('deadman_button', 4)
        self.declare_parameter('joy_timeout', 0.25)
        self.declare_parameter('mpc_input_timeout', 0.10)
        self.declare_parameter('prearm_stable_time', 0.50)
        self.declare_parameter('minimum_press_time', 1.0)
        self.declare_parameter('publish_rate', 50.0)
        self.button = int(self.get_parameter('deadman_button').value)
        self.timeout = float(self.get_parameter('joy_timeout').value)
        self.mpc_input_timeout = float(
            self.get_parameter('mpc_input_timeout').value
        )
        self.prearm_stable_time = float(
            self.get_parameter('prearm_stable_time').value
        )
        self.minimum_press_time = float(self.get_parameter('minimum_press_time').value)
        if not math.isfinite(self.minimum_press_time) or self.minimum_press_time < 1.0:
            raise ValueError('minimum_press_time must be at least 1.0 second')
        self.arm_pub = self.create_publisher(
            Bool, str(self.get_parameter('arm_topic').value), 10
        )
        self.deadman_pub = self.create_publisher(
            Bool, str(self.get_parameter('deadman_topic').value), 10
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10
        )
        self.enable_pub = self.create_publisher(
            Bool, str(self.get_parameter('enable_topic').value), 10
        )
        self.create_subscription(
            Joy, str(self.get_parameter('joy_topic').value), self._joy,
            qos_profile_sensor_data
        )
        self.create_subscription(
            AckermannDriveStamped,
            str(self.get_parameter('nominal_topic').value),
            self._nominal,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('planner_valid_topic').value),
            self._planner,
            10,
        )
        self.last_joy = None
        self.last_nominal = None
        self.last_planner = None
        self.nominal_finite = False
        self.planner_valid = False
        self.pressed = False
        self.pressed_since = None
        self.release_seen = False
        self.run_started = False
        self.arm_sent = False
        self.mpc_ready_since = None
        self.release_latched = False
        self.invalid = False
        self.last_state = None
        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._tick)

    def _joy(self, msg: Joy) -> None:
        self.last_joy = time.monotonic()
        if self.button < 0 or self.button >= len(msg.buttons):
            self.invalid = True
            self.pressed = False
            return
        self.pressed = bool(msg.buttons[self.button])
        if self.pressed and self.pressed_since is None:
            self.pressed_since = self.last_joy
        if not self.pressed:
            self.pressed_since = None
            if self.arm_sent:
                # Once armed, every real release immediately latches STOP.
                self.release_latched = True
            else:
                # A prearm finger adjustment has not permitted motor output.
                # Reset qualification and allow a new continuous hold.
                self.release_seen = True
                self.run_started = False
                self.mpc_ready_since = None

    def _nominal(self, msg: AckermannDriveStamped) -> None:
        self.last_nominal = time.monotonic()
        self.nominal_finite = all(math.isfinite(float(value)) for value in (
            msg.drive.speed, msg.drive.steering_angle
        ))

    def _planner(self, msg: Bool) -> None:
        self.last_planner = time.monotonic()
        self.planner_valid = bool(msg.data)

    def _mpc_ready(self, now: float) -> bool:
        return (
            self.last_nominal is not None and
            self.last_planner is not None and
            now - self.last_nominal <= self.mpc_input_timeout and
            now - self.last_planner <= self.mpc_input_timeout and
            self.nominal_finite and
            self.planner_valid
        )

    def _tick(self) -> None:
        now = time.monotonic()
        age = 1.0e9 if self.last_joy is None else now - self.last_joy
        fresh = age <= self.timeout and not self.invalid
        if not fresh:
            if self.arm_sent:
                self.release_latched = True
            else:
                self.pressed_since = None
                self.mpc_ready_since = None
                self.run_started = False
        held_for = 0.0 if self.pressed_since is None else max(0.0, now-self.pressed_since)
        run_requested = (
            fresh and self.release_seen and self.pressed and
            not self.release_latched
        )
        if run_requested and not self.run_started:
            self.run_started = True
        enable = run_requested and self.run_started
        mpc_ready = self._mpc_ready(now)
        if enable and not self.arm_sent:
            if mpc_ready:
                if self.mpc_ready_since is None:
                    self.mpc_ready_since = now
                if (now - self.mpc_ready_since >= self.prearm_stable_time
                        and held_for >= self.minimum_press_time):
                    self.arm_sent = True
            else:
                self.mpc_ready_since = None
        # Once arm has been sent, loss/release keeps arm requested so the final
        # guard observes the deadman fault and latches STOP. Restart is required.
        arm = self.arm_sent
        deadman = enable and self.arm_sent
        active = arm and deadman
        self.enable_pub.publish(Bool(data=enable))
        self.deadman_pub.publish(Bool(data=deadman))
        self.arm_pub.publish(Bool(data=arm))
        if self.release_latched:
            reason = 'deadman_release_latched'
        elif not fresh:
            reason = 'joy_timeout'
        elif not self.release_seen:
            reason = 'release_L1_before_arm'
        elif not self.pressed:
            reason = 'waiting_for_L1'
        elif not self.arm_sent and held_for < self.minimum_press_time:
            reason = 'qualifying_L1_hold'
        elif not self.arm_sent:
            reason = 'waiting_for_mpc_stable'
        else:
            reason = ''
        nominal_age = 1.0e9 if self.last_nominal is None else now - self.last_nominal
        planner_age = 1.0e9 if self.last_planner is None else now - self.last_planner
        stable_for = (
            0.0 if self.mpc_ready_since is None else
            max(0.0, now - self.mpc_ready_since)
        )
        self.status_pub.publish(String(data=json.dumps({
            'arm': arm, 'deadman': deadman, 'active': active, 'enable': enable,
            'release_latched': self.release_latched,
            'joy_age_sec': round(age, 6),
            'nominal_age_sec': round(nominal_age, 6),
            'planner_age_sec': round(planner_age, 6),
            'mpc_ready': mpc_ready,
            'mpc_stable_for_sec': round(stable_for, 6),
            'L1_held_for_sec': round(held_for, 6),
            'minimum_press_time_sec': self.minimum_press_time,
            'reason': reason,
        }, sort_keys=True)))
        state = (active, reason)
        if state != self.last_state:
            if active:
                self.get_logger().info('ROUTE ARMED: keep holding L1')
            else:
                self.get_logger().warn('ROUTE STOP: %s' % reason)
            self.last_state = state


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteOperatorGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.deadman_pub.publish(Bool(data=False))
            node.enable_pub.publish(Bool(data=False))
            node.arm_pub.publish(Bool(data=True if node.arm_sent else False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
