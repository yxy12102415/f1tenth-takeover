"""Offline fixture pre-arm discovery check; does not modify vehicle guards."""
import math


def ready_to_arm(statuses,guard_received,now,zero_samples):
    if zero_samples<10 or guard_received is None or not 0.<=now-guard_received<.10:return False
    pf,pp,guard=(statuses.get(k,{}) for k in ('pf','pp','guard'))
    if not pf.get('valid') or not pp.get('valid') or pp.get('reason')!='waiting_for_operator':return False
    if guard.get('arm_state')!='DISARMED' or guard.get('latched'):return False
    ages=guard.get('input_age_sec',{})
    for key,bound in [('scan',.15),('odom',.10),('nominal',.10),('planner',.10),('deadman',.10)]:
        value=ages.get(key,float('inf'))
        if not math.isfinite(value) or not 0.<=value<bound:return False
    return True
