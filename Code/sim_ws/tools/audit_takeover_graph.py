#!/usr/bin/env python3
"""Read-only graph audit in the isolated simulation ROS domain; no publishers."""
import json
import time
import rclpy
from rcl_interfaces.srv import GetParameters


def main():
    rclpy.init()
    node = rclpy.create_node('takeover_read_only_graph_audit')
    try:
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        subscribers = sorted({x.node_name for x in node.get_subscriptions_info_by_topic('/opp_racecar/odom')})
        publishers = sorted({x.node_name for x in node.get_publishers_info_by_topic('/drive')})
        nominal = sorted({x.node_name for x in node.get_publishers_info_by_topic('/drive_nominal')})
        service = node.create_client(GetParameters, '/takeover_lidar_tracker/get_parameters')
        if not service.wait_for_service(timeout_sec=2.):
            raise RuntimeError('Tracker parameter service unavailable')
        request = GetParameters.Request(names=['measurement_source'])
        future = service.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
        if not future.done() or future.result() is None:
            raise RuntimeError('Tracker parameter query timed out')
        source = future.result().values[0].string_value
        result = {'opponent_truth_subscribers': subscribers,
                  'final_command_publishers': publishers,
                  'nominal_command_publishers': nominal,
                  'tracker_measurement_source': source}
        result['passed'] = (
            bool(subscribers) and set(subscribers) <= {'takeover_lead_controller', 'takeover_evaluator'}
            and publishers == ['takeover_safety_arbiter']
            and nominal == ['takeover_controller'] and source == 'lidar')
        print(json.dumps(result, indent=2))
        if not result['passed']:
            raise SystemExit('Graph audit failed')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
