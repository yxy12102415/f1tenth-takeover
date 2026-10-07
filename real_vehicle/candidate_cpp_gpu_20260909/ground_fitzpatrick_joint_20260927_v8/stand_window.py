"""Independent, one-shot, <=90 second ground-route window; endpoint and faults stop earlier."""
import math
import time

COMMAND_TIMEOUT_S = .500
INITIAL_STEP_HOLD_S = .100


class StandWindow:
    def __init__(self):
        self.permitted=False;self.permit_at=None;self.joy_at=None;self.command_at=None
        self.health_ok=False;self.health_at=None
        self.held=False;self.release_seen=False;self.started=None;self.latched=None
        self.speed=self.steering=0.
        self.ramp_at=None
        self.floor_seen=False

    def health(self,value,now):
        self.health_ok=bool(value);self.health_at=now
        if self.started is not None and not value:self.latched=self.latched or 'health_rejected'

    def health_ready(self,now):
        return self.health_ok and self.health_at is not None and 0.<=now-self.health_at<=.50

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
        if not all(math.isfinite(x) for x in (speed,steering)) or not 0.<=speed<=2.00 or abs(steering)>.27:
            self.latched=self.latched or 'invalid_command';self.speed=self.steering=0.;return
        # The nominal controller can already be at cruise speed during the
        # disarmed handshake. Slew the ACTUAL permitted output from zero.
        # Only receipt of a fresh final command advances this ramp; evaluate()
        # never increases speed. Decreases and STOP remain immediate.
        drive_ready=(self.permitted and self.release_seen and self.held and self.health_ready(now)
                     and self.permit_at is not None and 0.<=now-self.permit_at<=.50
                     and self.joy_at is not None and 0.<=now-self.joy_at<=.50
                     and self.latched is None)
        if not drive_ready or speed==0.:
            self.speed=0.;self.steering=0. if speed==0. else steering;self.ramp_at=None
            return
        # After the one-time launch, never forward a sustained sub-floor
        # command. Stop rather than raise a genuinely infeasible speed demand.
        if self.floor_seen and speed < .5-1e-5:
            self.latched='sub_floor_command';self.speed=self.steering=0.
            return
        if self.floor_seen and speed < .5:
            speed=.5  # solver numerical tolerance only
        # Command callbacks can run twice before the first output tick.
        # Hold the initial step until evaluate() actually starts the window.
        if self.started is None or now-self.started < INITIAL_STEP_HOLD_S:
            self.speed=min(speed,.02);self.steering=steering;self.ramp_at=now
            return
        dt=.02 if self.ramp_at is None else max(0.,min(.05,now-self.ramp_at))
        self.speed=min(speed,self.speed+1.0*dt)
        if self.speed>=.5:self.floor_seen=True
        self.steering=steering;self.ramp_at=now

    def reject_source(self):
        # Pre-permission discovery delays must block readiness, not silently
        # reuse the previous command. Once permission has been granted, latch
        # even if wheels have not started. There is no post-permission reset.
        self.command_at=None;self.speed=self.steering=0.
        if self.permitted or self.started is not None:
            self.latched=self.latched or 'stale_source_stamp'

    def prearm_ready(self,now):
        return (self.health_ready(now) and self.latched is None and not self.permitted and self.started is None
                and self.command_at is not None and 0.<=now-self.command_at<=COMMAND_TIMEOUT_S
                and self.speed==0. and self.steering==0.
                and self.release_seen and not self.held and self.joy_at is not None
                and 0.<=now-self.joy_at<=.50)

    def evaluate(self,now):
        reason=self.latched
        if reason is None:
            if not self.health_ready(now):reason='health_not_ready'
            elif not self.permitted or self.permit_at is None or now-self.permit_at>.50:reason='no_fresh_permit'
            elif not self.release_seen or not self.held or self.joy_at is None or now-self.joy_at>.50:reason='deadman_not_ready'
            elif self.command_at is None or now-self.command_at>COMMAND_TIMEOUT_S:reason='final_timeout'
            elif self.started is not None and now-self.started >=90.:reason='route_time_limit'
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
    node.create_subscription(Bool,'/stand_pf/health_permit',lambda m:core.health(m.data,time.monotonic()),1)
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
        if not -.02<=age<=COMMAND_TIMEOUT_S:
            reject_source(age);return
        source_stamp[0]=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        core.command(msg.drive.speed,msg.drive.steering_angle,time.monotonic())
    node.create_subscription(AckermannDriveStamped,'/baseline/final',command,1)
    def tick():
        if source_stamp[0] is not None:
            age=node.get_clock().now().nanoseconds*1e-9-source_stamp[0]
            if not -.02<=age<=COMMAND_TIMEOUT_S:reject_source(age)
        speed,steering,reason=core.evaluate(time.monotonic())
        # Preserve bounded Pure Pursuit steering; never force a straight command.
        speed,steering=encode_route_command(speed,steering,2.00,.27)
        msg=AckermannDriveStamped();msg.header.stamp=node.get_clock().now().to_msg()
        msg.drive.speed=speed;msg.drive.steering_angle=steering;out.publish(msg)
        last_reason[0]=reason
    def status_tick():
        age=None if source_stamp[0] is None else node.get_clock().now().nanoseconds*1e-9-source_stamp[0]
        status.publish(String(data=json.dumps({'reason':last_reason[0],'latched':bool(core.latched),
            'started':core.started,'speed':0. if last_reason[0] else core.speed,
            'steering':0. if last_reason[0] else core.steering,'diagnostic':diagnostic,
            'source_age_s':age,'health_ready':core.health_ready(time.monotonic()),'ready_for_permission':core.prearm_ready(time.monotonic()) and age is not None and -.02<=age<=COMMAND_TIMEOUT_S})))
    # Match the 50 Hz upstream final stream; do not double-publish identical
    # commands. The independent source timeout is 500 ms.
    node.create_timer(.02,tick)
    node.create_timer(.02,status_tick)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        out.publish(AckermannDriveStamped());node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
