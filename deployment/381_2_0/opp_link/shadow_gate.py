"""Read-only candidate gate for a future F1/OPP integration.

It cannot grant an F1 permit. The real supervisor still needs a reciprocal
OPP acknowledgement interlock before either vehicle can be commissioned.
"""
import math

class Gate:
    def __init__(self):
        self.values = {}
        self.release_seen = False
        self.started = False
        self.latched = None

    def update(self, key, value, received):
        self.values[key] = (value, received)
        if key == 'joy' and value is False:
            self.release_seen = True
            if self.started: self.latched = self.latched or 'l1_released'
        if self.started and key in ('permit', 'health') and value is not True:
            self.latched = self.latched or key+'_revoked'

    def evaluate(self, now, acknowledgement_fresh):
        reason = self.latched
        required = ('joy','permit','health','window','actuator')
        for key in required:
            if key not in self.values or not 0 <= now-self.values[key][1] <= .25:
                reason = reason or key+'_stale'
        if not reason:
            v = {k:self.values[k][0] for k in required}
            try:
                w,a = v['window'],v['actuator']
                mono = float(w['published_mono'])
                age = float(a['input_age_sec']); rpm = float(a['rpm'])
                if not self.release_seen or v['joy'] is not True: reason='l1_not_ready'
                elif v['permit'] is not True or v['health'] is not True: reason='f1_not_permitted'
                elif (not math.isfinite(mono) or not 0 <= now-mono <= .25 or
                      w['latched'] is not False or w['started'] is None or w['reason'] or
                      w['health_ready'] is not True): reason='window_stopped'
                elif (a['stop_reason'] or a['motor_mode']!='speed' or not math.isfinite(age) or
                      not 0 <= age <= .5 or not math.isfinite(rpm) or not 0 < rpm <= 15000):
                    reason='actuator_stopped'
                elif not acknowledgement_fresh: reason='opp_ack_stale'
            except (KeyError, TypeError, ValueError): reason='invalid_status'
        if reason:
            if self.started: self.latched=self.latched or reason
            return False,reason
        self.started=True
        return True,''
