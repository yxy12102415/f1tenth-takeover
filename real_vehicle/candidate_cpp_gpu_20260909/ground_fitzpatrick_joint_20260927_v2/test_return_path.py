import json
import math
from pathlib import Path
import unittest
import numpy as np

from avoidance_controller import (open_endpoint_reached, endpoint_pose_error,
    endpoint_pose_matches, terminal_tracker_stop_allowed)
from return_lattice_planner import ClosedLoopLatticePlanner
from test_closed_lattice_controller import ROUTE, MAP, square_box_points


class ReturnTests(unittest.TestCase):
    def test_logged_thirty_cm_finish_is_rejected(self):
        previous = Path(__file__).resolve().parent.parent/'ground_381_2_0_frenet_dynamic_mpc_20260923_v2/run_logs/20260923_234947-closed-frenet-ground-attempt'
        trace = json.loads((previous/'tracking_trace.json').read_text())
        done = next(x for x in trace['controller'] if x.get('complete'))
        pose = min(trace['poses'], key=lambda x: abs(x['received_mono']-done['received_mono']))
        route = np.loadtxt(ROUTE,delimiter=',',skiprows=1)
        error = endpoint_pose_error(route,pose['x'],pose['y'],pose['yaw'])
        self.assertGreater(error[0],.30)
        self.assertFalse(open_endpoint_reached(False,done['lap_length_m'],done['lap_progress_m'],*error))
        self.assertFalse(terminal_tracker_stop_allowed('mpc_infeasible',.04,*error))
        planner=ClosedLoopLatticePlanner(ROUTE,MAP)
        planner.progress_index=done['progress_index']
        selected,_=planner.plan(pose['x'],pose['y'],heading=pose['yaw'])
        self.assertFalse(selected.valid)

    def test_endpoint_requires_position_heading_and_finite_inputs(self):
        self.assertTrue(open_endpoint_reached(False,6,5.96,.06,.1))
        for distance,heading in [(.11,0),(.06,.21),(math.nan,0),(0,math.inf)]:
            self.assertFalse(endpoint_pose_matches(distance,heading))
            self.assertFalse(open_endpoint_reached(False,6,5.96,distance,heading))
        self.assertFalse(open_endpoint_reached(False,6,5.96))

    def test_terminal_center_candidates_reach_reference_endpoint(self):
        planner = ClosedLoopLatticePlanner(ROUTE,MAP)
        x,y,yaw = planner.route[-25]
        selected,candidates = planner.plan(x-.05*math.sin(yaw),y+.05*math.cos(yaw),heading=yaw)
        centers = [c for c in candidates if abs(c.offset_m)<1e-9]
        self.assertTrue(selected.valid)
        for candidate in centers:
            np.testing.assert_allclose(candidate.path[-1,:2],planner.route[-1,:2],atol=1e-9)
        self.assertLess(abs(math.atan2(math.sin(selected.path[-1,2]-planner.route[-1,2]),
                                      math.cos(selected.path[-1,2]-planner.route[-1,2]))),.05)

    def test_repeated_plans_keep_same_spatial_rejoin(self):
        planner = ClosedLoopLatticePlanner(ROUTE,MAP)
        x,y,yaw = planner.route[75]
        original,_ = planner.plan(x+.12*math.sin(yaw),y-.12*math.cos(yaw),heading=yaw+.08)
        self.assertTrue(original.valid)
        self.assertEqual(original.offset_m,0.)
        for index in [5,10,15]:
            x,y,yaw=original.path[index]
            selected,_=planner.plan(x,y,heading=yaw)
            self.assertTrue(selected.valid)
            self.assertTrue(planner.center_path_reused)
            np.testing.assert_allclose(selected.path[-1],original.path[-1],atol=1e-12)
            distance = np.linalg.norm(selected.path[:,None,:2]-original.path[None,:,:2],axis=2)
            self.assertLess(float(np.max(np.min(distance,axis=1))),1e-12)

    def test_new_obstacle_invalidates_cached_swept_path(self):
        planner=ClosedLoopLatticePlanner(ROUTE,MAP)
        x,y,yaw=planner.route[25]
        selected,_=planner.plan(x,y,heading=yaw)
        self.assertTrue(selected.valid)
        box=square_box_points(selected.path[15,:2],half_size=.08)
        self.assertIsNone(planner._reuse_center(x,y,box))
        replacement,_=planner.plan(x,y,box,heading=yaw)
        self.assertFalse(planner.center_path_reused)
        if replacement.valid:
            self.assertFalse(replacement.footprint_collision)
            self.assertGreaterEqual(replacement.minimum_obstacle_clearance_m,.15)


if __name__=='__main__': unittest.main()
