"""Independent, one-shot, <=2 second stand window. Not a ground controller."""
import math
import time


class StandWindow:
    def __init__(self):
        self.permitted=False;self.permit_at=None;self.joy_at=None;self.command_at=None
        self.held=False;self.release_seen=False;self.started=None;self.latched=None
        self.speed=self.steering=0.

    def permit(self,value,now):
        self.permitted=bool(value);self.permit_at=now
        if self.started is not None and not value and self.latched is None:self.latched='permit_revoked'

    def joy(self,value,now):
        self.held=bool(value);self.joy_at=now
        if not value:
            self.release_seen=True
            if self.started is not None and self.latched is None:self.latched='deadman_release'

    def command(self,speed,steering,now):
        self.command_at=now
        if not all(math.isfinite(x) for x in (speed,steering)) or not 0.<=speed<=.5 or abs(steering)>.27:
            self.latched=self.latched or 'invalid_command';self.speed=self.steering=0.;return
        self.speed,self.steering=speed,steering

    def reject_source(self):
        # Pre-permission discovery delays must block readiness, not silently
        # reuse the previous command. Once permission has been granted, latch
        # even if wheels have not started. There is no post-permission reset.
        self.command_at=None;self.speed=self.steering=0.
        if self.permitted or self.started is not None:
            self.latched=self.latched or 'stale_source_stamp'

    def prearm_ready(self,now):
        return (self.latched is None and not self.permitted and self.started is None
                and self.command_at is not None and 0.<=now-self.command_at<=.10
                and self.speed==0. and self.steering==0.
                and self.release_seen and not self.held and self.joy_at is not None
                and 0.<=now-self.joy_at<=.15)

    def evaluate(self,now):
        reason=self.latched
        if reason is None:
            if not self.permitted or self.permit_at is None or now-self.permit_at>.15:reason='no_fresh_permit'
            elif not self.release_seen or not self.held or self.joy_at is None or now-self.joy_at>.15:reason='deadman_not_ready'
            elif self.command_at is None or now-self.command_at>.10:reason='final_timeout'
            elif self.started is not None and now-self.started>=2.:reason='two_second_limit'
            elif self.started is not None and self.speed==0. and self.steering==0.:reason='upstream_stop'
        if reason:
            if self.started is not None:self.latched=reason
            return 0.,0.,reason
        if self.started is None and (self.speed!=0. or self.steering!=0.):self.started=now
        return self.speed,self.steering,''


def main():
    import json
    import os
    import rclpy
    from rclpy.node import Node
    from ackermann_msgs.msg import AckermannDriveStamped
    from std_msgs.msg import Bool,String
    from sensor_msgs.msg import Joy
    from baseline_pf_pp.safe_pf import latest_sensor_qos
    from f1tenth_safety_migration.command_encoding import encode_route_command
    if os.environ.get('ROS_DOMAIN_ID') not in ('83','85'):raise RuntimeError('Stand window requires test domain')
    rclpy.init();node=Node('stand_window',start_parameter_services=False);core=StandWindow();diagnostic={};last_reason=[''];source_stamp=[None]
    out=node.create_publisher(AckermannDriveStamped,'/stand_pf/actuator_input',1)
    status=node.create_publisher(String,'/stand_pf/window_status',1)
    node.create_subscription(Bool,'/stand_pf/permit',lambda m:core.permit(m.data,time.monotonic()),1)
    def joy(msg):
        core.joy(len(msg.buttons)>4 and msg.buttons[4]==1,time.monotonic())
    node.create_subscription(Joy,'/baseline/joy',joy,latest_sensor_qos())
    def reject_source(age):
        phase='prearm_source_failure' if not core.permitted and core.started is None else 'source_failure'
        if phase not in diagnostic:
            diagnostic[phase]={'age_s':age,'permitted':core.permitted,'started':core.started,
                               'prior_latch':core.latched}
        source_stamp[0]=None;core.reject_source()
    def command(msg):
        age=node.get_clock().now().nanoseconds*1e-9-(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
        if not -.02<=age<=.10:
            reject_source(age);return
        source_stamp[0]=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        core.command(msg.drive.speed,msg.drive.steering_angle,time.monotonic())
    node.create_subscription(AckermannDriveStamped,'/baseline/final',command,1)
    def tick():
        if source_stamp[0] is not None:
            age=node.get_clock().now().nanoseconds*1e-9-source_stamp[0]
            if not -.02<=age<=.10:reject_source(age)
        speed,steering,reason=core.evaluate(time.monotonic())
        speed,steering=encode_route_command(speed,steering,.5,.27)
        msg=AckermannDriveStamped();msg.header.stamp=node.get_clock().now().to_msg()
        msg.drive.speed=speed;msg.drive.steering_angle=steering;out.publish(msg)
        last_reason[0]=reason
    def status_tick():
        age=None if source_stamp[0] is None else node.get_clock().now().nanoseconds*1e-9-source_stamp[0]
        status.publish(String(data=json.dumps({'reason':last_reason[0],'latched':bool(core.latched),
            'started':core.started,'speed':0. if last_reason[0] else core.speed,
            'steering':0. if last_reason[0] else core.steering,'diagnostic':diagnostic,
            'source_age_s':age,'ready_for_permission':core.prearm_ready(time.monotonic()) and age is not None and -.02<=age<=.10})))
    # Match the 50 Hz upstream final stream; do not double-publish identical
    # commands. The independent source timeout remains 100 ms.
    node.create_timer(.02,tick)
    node.create_timer(.05,status_tick)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        out.publish(AckermannDriveStamped());node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
