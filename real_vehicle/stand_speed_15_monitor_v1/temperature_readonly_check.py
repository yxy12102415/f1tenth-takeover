"""Bounded read-only driver check: no ROS command publishers; all inputs isolated."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid


def main():
    root = Path(__file__).resolve().parent
    sys.path.insert(0, str(root.parent / 'baseline_pf_pp_20260908'))
    from session_guard import acquire
    lock = acquire()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '85':
        raise RuntimeError('Requires isolated local hardware-check domain 85')
    if subprocess.run(['fuser', '/dev/sensors/vesc'], capture_output=True).returncode != 1:
        raise RuntimeError('Device busy or ownership check failed')
    import rclpy
    from vesc_msgs.msg import VescStateStamped
    rclpy.init()
    node = rclpy.create_node('temperature_readonly_check', start_parameter_services=False)
    samples = []
    scope = '/readonly_temperature_' + uuid.uuid4().hex
    def callback(msg):
        s = msg.state
        samples.append(dict(receipt_mono=time.monotonic(), erpm=s.speed,
                            motor_current=s.current_motor, input_current=s.current_input,
                            fet_temp=s.temp_fet, motor_temp=s.temp_motor,
                            voltage=s.voltage_input, fault=s.fault_code))
    sub = node.create_subscription(VescStateStamped, scope+'/telemetry', callback, 10)
    binary = root/'install_clean_v2/vesc_driver/lib/vesc_driver/vesc_driver_node'
    command = [str(binary), '--ros-args', '-p', 'port:=/dev/sensors/vesc',
               '-p', 'speed_min:=0.0', '-p', 'speed_max:=0.0',
               '-p', 'current_min:=0.0', '-p', 'current_max:=0.0',
               '-r', 'sensors/core:='+scope+'/telemetry']
    for topic in ('motor/speed','motor/current','motor/brake','motor/duty_cycle','motor/position','servo/position'):
        command += ['-r', 'commands/'+topic+':='+scope+'/unpublished/'+topic]
    child = None
    try:
        child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
        end = time.monotonic()+10.
        while time.monotonic()<end and len(samples)<50:
            if child.poll() is not None:
                raise RuntimeError('Read-only driver exited')
            rclpy.spin_once(node, timeout_sec=.02)
        if len(samples)<50:
            raise RuntimeError('Insufficient telemetry')
        if any(not all(math.isfinite(x[k]) for k in ('erpm','motor_current','input_current','fet_temp','voltage')) or x['erpm']!=0 or x['fault']!=0 for x in samples):
            raise RuntimeError('Unexpected motion, fault or invalid telemetry')
        print(json.dumps(dict(samples=len(samples), min_fet_temp_c=min(x['fet_temp'] for x in samples),
                              max_fet_temp_c=max(x['fet_temp'] for x in samples),
                              last=samples[-1], motor_sensor_validity='not_established',
                              command_publishers_created=0, motion_test=False), indent=2))
    finally:
        if child is not None:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
            try:
                output, _ = child.communicate(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    output, _ = child.communicate(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    output, _ = child.communicate(timeout=2)
            print(output, file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        lock.close()


if __name__ == '__main__':
    main()
