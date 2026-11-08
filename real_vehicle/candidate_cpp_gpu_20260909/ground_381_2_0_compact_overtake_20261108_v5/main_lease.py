"""Main-loop lease: sample monotonic time inside the same lock as its state."""
import math,threading,time
class SupervisorLease:
    def __init__(self,timeout=.50,clock=time.monotonic):
        if not math.isfinite(timeout) or timeout<=0:raise ValueError('invalid lease timeout')
        self.timeout=timeout;self.clock=clock;self.checked_at=None;self.expired=False
        self.lock=threading.Lock();self.failure=None;self.max_active_age=0.
    def checked(self):
        with self.lock:self.checked_at=self.clock()
    def allow(self,requested):
        with self.lock:
            if not requested:return False
            now=self.clock()
            age=None if self.checked_at is None else now-self.checked_at
            if age is not None and math.isfinite(age):self.max_active_age=max(self.max_active_age,age)
            if self.checked_at is None or not math.isfinite(now) or not math.isfinite(self.checked_at) or not 0.<=age<=self.timeout:
                if not self.expired:self.failure=dict(now=now,checked_at=self.checked_at,age=age)
                self.expired=True
            return not self.expired
    def diagnostics(self):
        with self.lock:return dict(timeout_s=self.timeout,expired=self.expired,max_active_age_s=self.max_active_age,failure=self.failure,time_sampled_under_lock=True)
