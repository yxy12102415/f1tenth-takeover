"""Pure readiness checks and fail-closed main-loop permission lease."""
import math
import threading


class SupervisorLease:
    def __init__(self,timeout=.10):
        self.timeout=timeout;self.checked_at=None;self.expired=False;self.lock=threading.Lock()

    def checked(self,now):
        with self.lock:
            self.checked_at=now

    def allow(self,requested,now):
        with self.lock:
            if not requested:return False
            if self.checked_at is None or not math.isfinite(now) or not 0.<=now-self.checked_at<=self.timeout:
                self.expired=True
            return not self.expired


def localization_ready(statuses,received,now):
    return (all(0.<=now-received.get(k,-1e9)<.15 for k in ('pf','pp'))
            and statuses.get('pf',{}).get('valid') is True
            and statuses.get('pp',{}).get('valid') is True
            and statuses.get('pp',{}).get('reason')=='waiting_for_operator')


def prearm_ready(statuses,received,now,joy):
    a=statuses.get('final',{}).get('input_age_sec',{})
    return (localization_ready(statuses,received,now)
            and statuses.get('final',{}).get('arm_state')=='DISARMED'
            and all(0.<=a.get(k,1e9)<.15 for k in ('nominal','scan','odom','planner'))
            and all(0.<=now-received.get(k,-1e9)<.10 for k in ('final','window'))
            and statuses.get('window',{}).get('ready_for_permission') is True
            and not statuses.get('window',{}).get('latched',True)
            and joy[1] is not None and 0.<=now-joy[1]<.10 and not joy[0])


def operator_released(statuses,received,now):
    op=statuses.get('operator',{})
    return (0.<=now-received.get('operator',-1e9)<.10
            and op.get('reason')=='waiting_for_L1'
            and op.get('arm') is False and op.get('enable') is False
            and op.get('deadman') is False and op.get('release_latched') is False
            and 0.<=op.get('joy_age_sec',1e9)<.10)
