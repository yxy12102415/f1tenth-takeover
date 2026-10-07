import threading,unittest
from main_lease import SupervisorLease
class Tests(unittest.TestCase):
    def test_no_check_or_stall_latches(self):
        t=[10.];l=SupervisorLease(clock=lambda:t[0])
        self.assertFalse(l.allow(False));self.assertFalse(l.expired)
        self.assertFalse(l.allow(True));l.checked();self.assertFalse(l.allow(True))
        l=SupervisorLease(clock=lambda:t[0]);l.checked();t[0]=10.499;self.assertTrue(l.allow(True))
        t[0]=10.501;self.assertFalse(l.allow(True));l.checked();self.assertFalse(l.allow(True))
        self.assertEqual(l.timeout,.50)
    def test_previous_incident_within_new_limit(self):
        t=[0.];l=SupervisorLease(clock=lambda:t[0]);l.checked()
        t[0]=.111723488;self.assertTrue(l.allow(True))
        t[0]=.50;self.assertTrue(l.allow(True))
        t[0]=.500001;self.assertFalse(l.allow(True))
        l.checked();self.assertFalse(l.allow(True))
    def test_clock_read_is_inside_lock(self):
        def clock():
            self.assertTrue(l.lock.locked());return 10.
        l=SupervisorLease(clock=clock);l.checked();self.assertTrue(l.allow(True))
    def test_refresh_between_call_and_lock_is_safe(self):
        t=[10.];l=SupervisorLease(clock=lambda:t[0]);l.checked()
        result=[]
        l.lock.acquire()
        thread=threading.Thread(target=lambda:result.append(l.allow(True)))
        thread.start()
        # Model a newer completed main-loop update while the checker waits.
        t[0]=10.001;l.checked_at=10.001
        l.lock.release();thread.join(1)
        self.assertFalse(thread.is_alive());self.assertEqual(result,[True]);self.assertFalse(l.expired)
    def test_old_callers_snapshot_reproduces_false_timeout(self):
        # Prior implementation checked an outside-lock argument against newer state.
        old_now=10.;new_checked=10.001
        self.assertFalse(0.<=old_now-new_checked<=.10)
        t=[new_checked];l=SupervisorLease(clock=lambda:t[0]);l.checked()
        self.assertTrue(l.allow(True))
    def test_nonfinite_clock_stops(self):
        t=[10.];l=SupervisorLease(clock=lambda:t[0]);l.checked();t[0]=float('nan')
        self.assertFalse(l.allow(True))
if __name__=='__main__':unittest.main()
