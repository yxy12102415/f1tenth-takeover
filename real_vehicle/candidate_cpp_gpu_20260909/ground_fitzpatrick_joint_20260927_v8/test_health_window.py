import unittest
from stand_window import StandWindow
from health_guard_core import HealthGuard
from health_interlock import LIMITS


class Tests(unittest.TestCase):
    def running(self):
        w=StandWindow();w.health(True,0.);w.joy(False,0.);w.joy(True,0.)
        w.permit(True,0.);w.command(.5,0.,0.)
        self.assertEqual(w.evaluate(0.)[0],.02)
        return w

    def test_missing_health_never_drives(self):
        w=StandWindow();w.joy(False,0.);w.joy(True,0.);w.permit(True,0.);w.command(.5,0.,0.)
        self.assertEqual(w.evaluate(0.)[0],0.)

    def test_health_loss_latches(self):
        w=self.running();w.health(False,.01)
        self.assertEqual(w.evaluate(.01)[0],0.)
        w.health(True,.02);w.command(.5,0.,.02)
        self.assertEqual(w.evaluate(.02)[0],0.)

    def test_health_process_exit_timeout(self):
        w=self.running();w.joy(True,.501);w.permit(True,.501);w.command(.5,0.,.501)
        self.assertEqual(w.evaluate(.501)[0],0.)
        self.assertEqual(w.latched,'health_not_ready')

    def test_deadman_still_mandatory(self):
        w=self.running();w.joy(False,.01)
        self.assertEqual(w.evaluate(.01)[0],0.)

    def test_invalid_motor_temperature_not_used(self):
        for motor_temp in (-81.9, float('nan'), None):
            g=HealthGuard(LIMITS)
            self.assertTrue(g.update(dict(stamp=100.,erpm=0.,motor_current=0.,input_current=0.,
                fet_temp=25.,voltage=12.,fault=0,duty_cycle=0.,motor_temp=motor_temp),1.,100.))
            self.assertTrue(g.healthy(1.01,100.01))

    def test_hot_fet_cannot_be_ignored(self):
        g=HealthGuard(LIMITS)
        self.assertFalse(g.update(dict(stamp=100.,erpm=0.,motor_current=0.,input_current=0.,
            fet_temp=65.,voltage=12.,fault=0,duty_cycle=0.,motor_temp=-81.9),1.,100.))

    def test_high_current_low_rpm_combination_is_not_a_trip(self):
        g=HealthGuard(LIMITS)
        for dt in (0.,.05):
            self.assertTrue(g.update(dict(stamp=100.+dt,erpm=100.,motor_current=40.,input_current=5.,
                fet_temp=25.,voltage=12.,fault=0,duty_cycle=.1),1.+dt,100.+dt))
        self.assertTrue(g.update(dict(stamp=100.11,erpm=100.,motor_current=40.,input_current=5.,
                fet_temp=25.,voltage=12.,fault=0,duty_cycle=.1),1.11,100.11))

    def test_current_thresholds_removed(self):
        g=HealthGuard(LIMITS)
        self.assertTrue(g.update(dict(stamp=100.,erpm=100.,motor_current=100.,input_current=50.,
                fet_temp=25.,voltage=12.,fault=0,duty_cycle=.1),1.,100.))

    def test_erpm_limit_matches_1_5_mps_profile(self):
        sample=dict(stamp=100.,erpm=11250.,motor_current=5.,input_current=2.,
                    fet_temp=25.,voltage=12.,fault=0,duty_cycle=.2)
        g=HealthGuard(LIMITS)
        self.assertTrue(g.update(sample,1.,100.))
        sample['stamp']=100.02;sample['erpm']=15750.01
        self.assertFalse(g.update(sample,1.02,100.02))
        self.assertEqual(g.reason,'overspeed')


if __name__=='__main__':unittest.main()
