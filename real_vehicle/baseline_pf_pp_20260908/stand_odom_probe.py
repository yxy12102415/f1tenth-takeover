"""Offline-only diagnostic of the reused odometry binary; never opens VESC."""
import argparse
import os
import signal
import subprocess
import time
import rclpy
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='83':raise RuntimeError('Offline domain only')
    parser=argparse.ArgumentParser();parser.add_argument('--profile',choices=['original','zero','motion'],default='original');args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('odom_probe');got=[]
    pub=node.create_publisher(VescStateStamped,'/probe/telemetry',10)
    servo=node.create_publisher(Float64,'/probe/servo',10)
    node.create_subscription(Odometry,'/probe/odom',lambda m:got.append(m),10)
    cmd=['/home/f1tenth/f12026_ws/install/vesc_ackermann/lib/vesc_ackermann/vesc_to_odom_node',
        '--ros-args','--params-file','/home/f1tenth/f12026_ws/src/f1tenth_system/f1tenth_stack/config/vesc.yaml',
        '-r','sensors/core:=/probe/telemetry','-r','odom:=/probe/odom','-r','sensors/servo_position_command:=/probe/servo',
        '-p','publish_tf:=false','-p','use_servo_cmd_to_calc_angular_velocity:='+('true' if args.profile=='motion' else 'false')]
    name='vesc_to_odom_node'
    if args.profile!='original':
        name='stand_vesc_odometry';cmd+=['-r','__node:='+name,'-p','wheelbase:=0.25']
    proc=subprocess.Popen(cmd,start_new_session=True)
    try:
        client=node.create_client(GetParameters,'/'+name+'/get_parameters')
        if not client.wait_for_service(timeout_sec=8.):raise RuntimeError('No odom service')
        request=GetParameters.Request();request.names=['publish_tf','use_servo_cmd_to_calc_angular_velocity']
        future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=3.)
        print('ACTUAL PARAMETERS',future.result(),flush=True)
        values=[v.bool_value for v in future.result().values]
        assert values==([True,True] if args.profile=='original' else [False,args.profile=='motion'])
        end=time.monotonic()+3.
        while time.monotonic()<end:
            if args.profile=='motion':servo.publish(Float64(data=.4304))
            msg=VescStateStamped();msg.header.stamp=node.get_clock().now().to_msg();msg.state.speed=750. if args.profile=='motion' else 0.;pub.publish(msg)
            rclpy.spin_once(node,timeout_sec=.02)
        print('ODOM COUNT',len(got),flush=True)
        if args.profile=='original':assert not got
        else:
            assert len(got)>10
            assert got[-1].header.frame_id=='odom' and got[-1].child_frame_id=='base_link'
            if args.profile=='motion':assert got[-1].pose.pose.position.x>0. and abs(got[-1].twist.twist.linear.x-.1)<1e-6
            else:assert got[-1].pose.pose.position.x==0. and got[-1].twist.twist.linear.x==0.
    finally:
        os.killpg(proc.pid,signal.SIGINT);proc.wait(timeout=5.)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
