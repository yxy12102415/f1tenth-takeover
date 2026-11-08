#!/usr/bin/env python3
"""Exercise actual ROS gate callbacks with a deterministic clock; no drivers."""
import importlib.util
import json
import os
from pathlib import Path
from unittest.mock import patch

assert os.environ.get('ROS_DOMAIN_ID') == '187'
import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

D = Path(__file__).resolve().parent
R = Path(os.environ.get('COMPACT_RELEASE', str(D.parents[1]/'real_vehicle/candidate_cpp_gpu_20260909/ground_381_2_0_compact_overtake_20261108_v5')))
spec = importlib.util.spec_from_file_location('operator_gate_v5', R/'operator_gate_v5.py')
g = importlib.util.module_from_spec(spec); spec.loader.exec_module(g)


class Capture:
    def __init__(self): self.messages = []
    def publish(self, value): self.messages.append(value)


def main():
    rclpy.init(); results = []
    for case in ['recorded_adjustment_then_hold', 'release_after_arm_latches',
                 'held_at_boot_needs_release', 'prearm_timeout_requalifies',
                 'MPC_must_be_stable', 'short_060_press_retries', 'postarm_timeout_latches']:
        n = g.RouteOperatorGate(); clock = [0.]
        for name in ['arm_pub', 'deadman_pub', 'enable_pub', 'status_pub']: setattr(n, name, Capture())
        def at(t, pressed=None, ready=True):
            clock[0] = t
            if pressed is not None:
                msg = Joy(); msg.buttons = [0]*5; msg.buttons[4] = int(pressed); n._joy(msg)
            n._nominal(AckermannDriveStamped()); n._planner(Bool(data=ready)); n._tick()
        def hold(start, duration, ready=True):
            for i in range(round(duration/.1)+1): at(start+i*.1, True, ready)
        try:
            with patch.object(g.time, 'monotonic', lambda: clock[0]):
                if case == 'held_at_boot_needs_release':
                    hold(0., 2.); assert not n.arm_sent
                    at(2.1, False); hold(2.2, 1.1); assert n.arm_sent
                elif case == 'prearm_timeout_requalifies':
                    at(0., False); hold(.1, .4); at(.9)
                    assert not n.arm_sent and n.pressed_since is None
                    hold(1., .7); assert not n.arm_sent
                    hold(1.8, .4); assert n.arm_sent
                elif case == 'MPC_must_be_stable':
                    at(0., False); hold(.1, 1.2, False); assert not n.arm_sent
                    hold(1.4, .4); assert not n.arm_sent
                    hold(1.9, .2); assert n.arm_sent
                elif case == 'short_060_press_retries':
                    at(0., False); hold(.1, .6); assert not n.arm_sent
                    at(.71, False); assert not n.release_latched
                    hold(.9, 1.1); assert n.arm_sent
                elif case == 'postarm_timeout_latches':
                    at(0., False); hold(.1, 1.1); assert n.arm_sent
                    at(1.6); assert n.release_latched and not n.deadman_pub.messages[-1].data
                    at(1.7, True); assert n.release_latched and not n.deadman_pub.messages[-1].data
                else:
                    at(0., False); at(.1, True); at(.238, False)
                    assert not n.arm_sent and not n.release_latched
                    hold(.48, .8); assert not n.arm_sent
                    hold(1.38, .2); assert n.arm_sent and n.deadman_pub.messages[-1].data
                    if case == 'release_after_arm_latches':
                        at(1.59, False)
                        assert n.release_latched and not n.deadman_pub.messages[-1].data
                        at(1.60, True)
                        assert n.release_latched and not n.deadman_pub.messages[-1].data
            results.append(dict(case=case, passed=True))
        finally: n.destroy_node()
    rclpy.shutdown()
    (D/'reports/input_gate_v5_contract.json').write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps(results), flush=True)


if __name__ == '__main__': main()
