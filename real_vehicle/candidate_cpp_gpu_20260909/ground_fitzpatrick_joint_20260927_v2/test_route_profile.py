import math
from pathlib import Path
import unittest
import numpy as np
from stand_window import StandWindow
from motion_policy import evidence_status,FAULTS,zero_health_ready
from stand_follow import RUNTIME_ROUTE, RUNTIME_SPEED_PROFILE, physical_stop_accepted
from avoidance_controller import (ENDPOINT_STOP_REMAINING_M,
                                  SPEED_PROFILE_LOOKAHEAD_M,
                                  load_speed_profile)

class RouteTests(unittest.TestCase):
    def test_runtime_route_seed_is_the_physical_start(self):
        route=np.loadtxt(RUNTIME_ROUTE,delimiter=',',skiprows=1)
        physical=np.loadtxt(RUNTIME_ROUTE.parent/'manual_route_20260923_172359.csv',delimiter=',',skiprows=1)
        np.testing.assert_allclose(route[0,:2],physical[0,:2],atol=1e-12,rtol=0.)
        self.assertLess(abs(math.atan2(math.sin(route[0,2]-physical[0,2]),
                                      math.cos(route[0,2]-physical[0,2]))),.06)

    def test_runtime_speed_profile_matches_route_and_limits(self):
        route=np.loadtxt(RUNTIME_ROUTE,delimiter=',',skiprows=1)
        distance,speed=load_speed_profile(RUNTIME_SPEED_PROFILE,route)
        self.assertEqual(len(distance),len(route))
        self.assertEqual((speed[0],speed[-1]),(0.,0.))
        # The only nonzero values below the cruise floor belong to the smooth
        # acceleration/deceleration ramps next to the required zero endpoints.
        cruise=speed[(distance>=.20)&(distance<=distance[-1]-.20)]
        self.assertGreaterEqual(cruise.min(),.60)
        self.assertLessEqual(speed.max(),1.50)
        self.assertEqual(SPEED_PROFILE_LOOKAHEAD_M,.20)
        # Preview starts from a nonzero speed but is capped before terminal
        # zero; the independent endpoint limiter owns the last five centimeters.
        start_query=min(distance[-1]-ENDPOINT_STOP_REMAINING_M,
                        SPEED_PROFILE_LOOKAHEAD_M)
        end_query=min(distance[-1]-ENDPOINT_STOP_REMAINING_M,
                      distance[-1]-.15+SPEED_PROFILE_LOOKAHEAD_M)
        self.assertGreater(float(np.interp(start_query,distance,speed)),0.)
        self.assertGreater(float(np.interp(end_query,distance,speed)),0.)

    def running(self):
        w=StandWindow();w.health(True,0.);w.joy(False,0.);w.joy(True,0.);w.permit(True,0.);w.command(.5,.2,0.)
        self.assertEqual(w.evaluate(0.),(.02,.2,''));return w
    def test_zero_requires_continuous_health(self):
        states={'health_guard':{'healthy':True},'window':{'health_ready':True,'latched':False}}
        times={'health_guard':1.,'window':1.}
        self.assertTrue(zero_health_ready(states,times,1.01))
        self.assertTrue(zero_health_ready(states,times,1.49))
        self.assertFalse(zero_health_ready(states,times,1.51))
        states['health_guard']['healthy']=False
        self.assertFalse(zero_health_ready(states,times,1.01))
    def test_nominal_prewarm_cannot_consume_actual_ramp(self):
        w=StandWindow();w.health(True,0.);w.joy(False,0.)
        for i in range(30):
            t=i*.02;w.health(True,t);w.joy(False,t);w.command(.5,.1,t)
            self.assertEqual(w.evaluate(t)[0],0.)
        w.health(True,.6);w.joy(True,.6);w.permit(True,.6);w.command(.5,.1,.6)
        self.assertAlmostEqual(w.evaluate(.6)[0],.02)
        # A timer evaluation must never ramp up an unchanged input.
        self.assertAlmostEqual(w.evaluate(.61)[0],.02)
        w.command(.5,.1,.62);self.assertAlmostEqual(w.evaluate(.62)[0],.02)
        w.command(.5,.1,.72);self.assertAlmostEqual(w.evaluate(.72)[0],.07)
        w.command(.01,.1,.73);self.assertEqual(w.evaluate(.73)[0],.01)
        w.command(0.,0.,.74);self.assertEqual(w.evaluate(.74)[0],0.)
    def test_command_burst_before_first_output(self):
        w=StandWindow();w.health(True,0.);w.joy(False,0.);w.joy(True,0.);w.permit(True,0.)
        w.command(.5,.2,0.);w.command(.5,.2,.018);w.command(.5,.2,.035)
        self.assertEqual(w.evaluate(.036),(.02,.2,''))
        w.command(.5,.2,.055);self.assertAlmostEqual(w.evaluate(.055)[0],.02)
        w.command(.5,.2,.14);self.assertAlmostEqual(w.evaluate(.14)[0],.07)
    def test_turning_is_preserved(self):
        w=self.running()
        for a in (-.27,-.1,0.,.1,.27):
            w.command(.5,a,.01);v,angle,reason=w.evaluate(.01)
            self.assertGreater(v,0.);self.assertLessEqual(v,.5);self.assertEqual((angle,reason),(a,''))
    def test_independent_duration_latches(self):
        w=self.running()
        for i in range(1,2701):
            t=i/100;w.health(True,t);w.joy(True,t);w.permit(True,t);w.command(.5,.2,t)
            expected=.02 if t<.10 else min(.5,.02+t-.09)
            self.assertAlmostEqual(w.evaluate(t)[0],expected if t<25 else 0.,places=8)
        self.assertEqual(w.latched,'route_time_limit')
    def test_endpoint_zero_latches(self):
        w=self.running();w.command(0.,0.,.01)
        self.assertEqual(w.evaluate(.01),(0.,0.,'upstream_stop'))
        w.command(.5,.2,.02);self.assertEqual(w.evaluate(.02)[0],0.)
    def test_release_and_stale_stop(self):
        w=self.running();w.joy(False,.01);self.assertEqual(w.evaluate(.01)[2],'deadman_release')
        w=self.running();w.health(True,.501);w.joy(True,.501);w.permit(True,.501)
        self.assertEqual(w.evaluate(.501)[2],'final_timeout')
    def test_invalid_limits(self):
        for v,a in ((1.751,0.),(-.01,0.),(1.75,.271),(1.75,-.271),(math.nan,0.),(1.75,math.nan)):
            w=self.running();w.command(v,a,.01);self.assertEqual(w.evaluate(.01)[2],'invalid_command')
    def test_evidence_cannot_reuse_old_mode_or_stale_zero(self):
        zero=dict(passed=True,code_fingerprint='x',completed_unix=1000.,pose_timeout_sec=.50,
                  power_mode_record='pmode:0008',offline=False,zero_only=True,motion_permission=False,
                  continuous_localization_passed=True,localization_seconds=10.1,physical_mode='ground',
                  zero_joy=True,zero_rviz=True,initialization_mode='operator_confirmed_route_start',
                  supervisor_lease_expired=False,timing={'max_abs_erpm':0.})
        self.assertTrue(evidence_status([zero],'x',1001.)[2])
        self.assertFalse(evidence_status([dict(zero,physical_mode='stand')],'x',1001.)[2])
        self.assertFalse(evidence_status([zero],'x',1901.)[2])
        self.assertFalse(evidence_status([zero],'other',1001.)[2])

    def test_route_completion_withdraws_permit_without_planner_failure(self):
        self.assertTrue(physical_stop_accepted(
            True, 'permit_revoked', 'teleop_deadman_released'))
        self.assertTrue(physical_stop_accepted(
            True, 'upstream_stop', 'planner_invalid'))
        self.assertFalse(physical_stop_accepted(
            False, 'permit_revoked', 'teleop_deadman_released'))
        self.assertFalse(physical_stop_accepted(
            False, 'upstream_stop', 'planner_invalid'))

if __name__=='__main__':unittest.main()
