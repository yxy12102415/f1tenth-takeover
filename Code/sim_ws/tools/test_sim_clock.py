#!/usr/bin/env python3
"""Exercise actual bridge stepping methods without ROS nodes or a Gym process."""
import ast
from pathlib import Path
from types import MethodType, SimpleNamespace
import unittest
import numpy as np

source = Path(__file__).resolve().parents[1] / 'src/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py'
tree = ast.parse(source.read_text())
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'GymBridge')
functions = [n for n in cls.body if isinstance(n, ast.FunctionDef)
             and n.name in ('drive_timer_callback', '_physics_stamp')]
namespace = {'np': np, 'Time': SimpleNamespace, 'ClockMessage': SimpleNamespace}
exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), 'exec'), namespace)


def fixture():
    commands, clocks, sensors = [], [], []
    def step(value):
        commands.append(value)
        return {}, 0, False, {}
    f = SimpleNamespace(publish_sim_clock=True, physics_ticks=0,
        last_ego_drive_time=None, has_opp=True, ego_drive_published=False,
        opp_drive_published=False, ego_steer=0., ego_requested_speed=0.,
        opp_steer=0., opp_requested_speed=0., env=SimpleNamespace(step=step),
        clock_pub=SimpleNamespace(publish=clocks.append), _update_sim_state=lambda: None)
    for name in ('drive_timer_callback', '_physics_stamp'):
        setattr(f, name, MethodType(namespace[name], f))
    f.timer_callback=lambda: sensors.append(f._physics_stamp())
    return f, commands, clocks, sensors


class SimulationClock(unittest.TestCase):
    def test_zero_commands_bootstrap_ros_time_without_control_timer_deadlock(self):
        f, commands, clocks, sensors=fixture()
        f.drive_timer_callback()
        self.assertEqual(f.physics_ticks,1)
        self.assertEqual(clocks[0].clock.nanosec,10000000)
        self.assertEqual(commands[0].shape,(2,2))
        self.assertTrue(np.all(commands[0]==0.))
        self.assertEqual(sensors,[])
        f.drive_timer_callback()
        self.assertEqual(sensors[0].nanosec,20000000)

    def test_stamps_follow_steps_and_integer_rollover_not_elapsed_wall_time(self):
        f, commands, clocks, sensors=fixture()
        for _ in range(101):
            f.drive_timer_callback()
        self.assertEqual(len(commands),101)
        self.assertEqual((clocks[-1].clock.sec,clocks[-1].clock.nanosec),(1,10000000))
        self.assertEqual(len(sensors),50)
        before=f._physics_stamp()
        # No physics call means no simulated-time advancement, however long
        # an OS scheduling pause might be. There is no wall-time catch-up.
        self.assertEqual(vars(before),vars(f._physics_stamp()))

    def test_legacy_mode_does_not_publish_a_clock_or_step_without_commands(self):
        f, commands, clocks, sensors=fixture()
        f.publish_sim_clock=False
        f.drive_timer_callback()
        self.assertEqual((commands,clocks,sensors),([],[],[]))


if __name__=='__main__':
    unittest.main()
