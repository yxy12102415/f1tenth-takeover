import threading
import unittest
from types import SimpleNamespace, MethodType
from unittest.mock import Mock, patch

import numpy as np

from avoidance_controller import AvoidanceController
from local_path_tracker import LocalPathTracker


def node_fixture():
    x = np.linspace(0., 3., 61)
    node = SimpleNamespace(
        command_lock=threading.RLock(), input_lock=threading.RLock(),
        tracker_lock=threading.RLock(), tracker=LocalPathTracker(),
        enabled=True, enable_receipt=10., complete=False,
        latest_pose=(10., .2, .04, 0.),
        last_safe_path=np.c_[x, x*0., x*0.], last_result=None,
        plan_started=9.8, last_plan_finished=9.5, last_tracker_tick=None,
        last_angle=0., measured_speed=1., measured_lateral_speed=0.,
        measured_yaw_rate=0., lattice_state='FOLLOWING',
        lap=SimpleNamespace(route_length_m=3., distance_m=.2),
        profile_distance=np.array([0., 3.]), profile_speed=np.array([1., 1.]),
        base_ready=Mock(return_value=True), publish=Mock(),
        publish_stop=Mock(), publish_command=Mock(),
        last_control_receipt=9.99)
    for name in ('control', 'reset_tracker', 'plan_fresh'):
        setattr(node, name, MethodType(getattr(AvoidanceController, name), node))
    return node


class ContinuousTrackingTest(unittest.TestCase):
    def test_new_pose_changes_command_during_inflight_search(self):
        node = node_fixture()
        with patch('avoidance_controller.time.monotonic', return_value=10.):
            AvoidanceController.control_tick(node)
        first = node.publish.call_args.args
        self.assertLess(first[1], 0.)
        self.assertEqual(first[3], 'tracking_during_replan')
        node.latest_pose = (10.05, .25, -.04, 0.)
        with patch('avoidance_controller.time.monotonic', return_value=10.05):
            AvoidanceController.control_tick(node)
        self.assertGreater(node.publish.call_args.args[1], first[1])
        self.assertEqual(node.plan_started, 9.8)
        node.publish_stop.assert_not_called()

    def test_fresh_commands_cannot_hide_stalled_search(self):
        for callback in (AvoidanceController.control_tick, AvoidanceController.watchdog):
            node = node_fixture()
            node.last_control_receipt = 10.30
            with patch('avoidance_controller.time.monotonic', return_value=10.31):
                callback(node)
            node.publish_stop.assert_called_with('planning_timeout')
            node.publish_command.assert_not_called()

    def test_idle_planner_must_also_refresh(self):
        node = node_fixture()
        node.plan_started = None
        with patch('avoidance_controller.time.monotonic', return_value=10.01):
            AvoidanceController.control_tick(node)
        node.publish_stop.assert_called_with('planning_timeout')

    def test_retry_cannot_renew_an_already_expired_path(self):
        node = node_fixture()
        node.plan_started = 9.99
        node.last_plan_finished = 8.
        with patch('avoidance_controller.time.monotonic', return_value=10.):
            AvoidanceController.control_tick(node)
        node.publish_stop.assert_called_with('planning_timeout')

    def test_stale_inputs_and_released_l1_do_not_drive(self):
        node = node_fixture()
        node.enabled = False
        with patch('avoidance_controller.time.monotonic', return_value=10.):
            AvoidanceController.control_tick(node)
        self.assertEqual(node.publish.call_args.args[:2], (0., 0.))
        node.base_ready.return_value = False
        with patch('avoidance_controller.time.monotonic', return_value=10.):
            AvoidanceController.control_tick(node)
        node.publish_stop.assert_called_with('inputs_not_ready')

    def test_completed_route_stays_zero_even_with_expired_plan(self):
        node = node_fixture()
        node.complete = True
        node.plan_started = None
        node.last_plan_finished = 0.
        with patch('avoidance_controller.time.monotonic', return_value=10.):
            AvoidanceController.control_tick(node)
        self.assertEqual(node.publish.call_args.args[:4], (0., 0., True, 'route_complete'))


if __name__ == '__main__':
    unittest.main()
