"""Independent health permit. Missing/failed monitor always closes the window."""
import json
import os
import time
from health_guard_core import HealthGuard, Limits

# Experiment stop thresholds, NOT VESC configuration changes or a current regulator.
# The current/stall fields remain in the shared Limits schema for compatibility,
# but HealthGuard treats current as telemetry-only and does not stop on it.
LIMITS = Limits(63., 40., 60., 10.5, 15750., 30., 500., .10)


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID') not in ('83','85'):
        raise RuntimeError('Only isolated test domains')
    rclpy.init(); node=Node('stand_health_interlock',start_parameter_services=False)
    core=HealthGuard(LIMITS)
    permit=node.create_publisher(Bool,'/stand_pf/health_permit',1)
    status=node.create_publisher(String,'/stand_pf/health_guard_status',1)
    def receive(msg):
        try:
            sample=json.loads(msg.data)
            if os.environ.get('ROS_DOMAIN_ID')=='85' and sample.get('synthetic'):
                core.trip('synthetic_health_forbidden');return
            core.update(sample,time.monotonic(),node.get_clock().now().nanoseconds*1e-9)
        except Exception:
            core.trip('invalid_health_payload')
    node.create_subscription(String,'/stand_pf/health',receive,1)
    def tick():
        ok=core.healthy(time.monotonic(),node.get_clock().now().nanoseconds*1e-9)
        permit.publish(Bool(data=ok))
        status.publish(String(data=json.dumps(dict(healthy=ok,reason=core.reason or ('healthy' if ok else 'waiting'),
            motor_temperature_used=False,limits=LIMITS.__dict__))))
    node.create_timer(.02,tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        permit.publish(Bool(data=False));node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
