import unittest
from health_guard_core import HealthGuard, Limits


def fixture_limits():
    # Synthetic test numbers, NOT a reviewed live motor-current allowance.
    return Limits(20., 10., 60., 10.5, 12000., 10., 500., .1)


def sample(**changes):
    value = dict(stamp=100., erpm=0., motor_current=0., input_current=0.,
                 fet_temp=25., voltage=12., duty_cycle=0., fault=0)
    value.update(changes)
    return value


class Tests(unittest.TestCase):
    def setUp(self):
        self.guard = HealthGuard(fixture_limits())

    def test_default_blocks(self):
        self.assertFalse(self.guard.healthy(1., 100.))

    def test_valid_stationary_health_not_motion_permission(self):
        self.assertTrue(self.guard.update(sample(), 1., 100.))
        self.assertTrue(self.guard.healthy(1.02, 100.02))

    def test_limits_invalid(self):
        with self.assertRaises(ValueError):
            Limits(float('nan'), 1., 60., 10., 1000., 1., 500., .1)

    def test_missing_and_nonfinite(self):
        for key in sample():
            for value in (None, float('nan'), float('inf'), True):
                with self.subTest(key=key, value=value):
                    guard = HealthGuard(fixture_limits())
                    self.assertFalse(guard.update(sample(**{key: value}), 1., 100.))

    def test_hard_trips_latch(self):
        for field, value in [('motor_current',21.), ('input_current',-11.),
                             ('fet_temp',60.), ('fet_temp',0.), ('voltage',10.),
                             ('erpm',12001.), ('duty_cycle',1.1), ('fault',1)]:
            with self.subTest(field=field, value=value):
                guard = HealthGuard(fixture_limits())
                self.assertFalse(guard.update(sample(**{field:value}), 1., 100.))
                reason = guard.reason
                self.assertFalse(guard.update(sample(stamp=100.02), 1.02, 100.02))
                self.assertEqual(guard.reason, reason)

    def test_high_current_low_rpm(self):
        self.assertTrue(self.guard.update(sample(motor_current=15.), 1., 100.))
        self.assertFalse(self.guard.update(sample(stamp=100.11, motor_current=15.), 1.11, 100.11))
        self.assertEqual(self.guard.reason,'high_current_low_rpm')

    def test_brief_startup_not_stall(self):
        self.assertTrue(self.guard.update(sample(motor_current=15.), 1., 100.))
        self.assertTrue(self.guard.update(sample(stamp=100.05, motor_current=5., erpm=1000.), 1.05, 100.05))
        self.assertIsNone(self.guard.stall_since)

    def test_lost_stream(self):
        self.guard.update(sample(), 1., 100.)
        self.assertFalse(self.guard.healthy(1.151,100.151))

    def test_delayed_callback_gap_not_healed_by_fresh_sample(self):
        self.guard.update(sample(), 1., 100.)
        self.assertFalse(self.guard.update(sample(stamp=100.2), 1.2, 100.2))

    def test_replayed_stamp(self):
        self.guard.update(sample(),1.,100.)
        self.assertFalse(self.guard.update(sample(),1.01,100.01))

    def test_stale_future_and_clock_jump(self):
        for stamp in (99.,101.):
            guard = HealthGuard(fixture_limits())
            self.assertFalse(guard.update(sample(stamp=stamp),1.,100.))
        self.guard.update(sample(),1.,100.)
        self.assertFalse(self.guard.healthy(.9,100.01))


if __name__ == '__main__':
    unittest.main()
