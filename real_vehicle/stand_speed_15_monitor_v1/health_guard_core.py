"""Offline-tested health interlock, not a motion launcher or current regulator.

Thresholds must be supplied explicitly by a reviewed experiment profile.
Passing this interlock alone never grants vehicle-motion permission.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Limits:
    max_motor_current_a: float
    max_input_current_a: float
    max_fet_temp_c: float
    min_voltage_v: float
    max_abs_erpm: float
    stall_current_a: float
    stall_erpm: float
    stall_duration_s: float
    source_timeout_s: float = .15
    receipt_timeout_s: float = .15

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in self.__dict__.values()):
            raise ValueError('Every limit must be finite and positive')
        if self.stall_current_a > self.max_motor_current_a:
            raise ValueError('Stall threshold exceeds hard current threshold')
        if self.stall_erpm > self.max_abs_erpm:
            raise ValueError('Stall RPM exceeds hard RPM threshold')


class HealthGuard:
    def __init__(self, limits):
        self.limits = limits
        self.last = None
        self.stall_since = None
        self.reason = None

    def trip(self, reason):
        self.reason = self.reason or reason
        return False

    def update(self, sample, received_mono, now_ros):
        fields = ('stamp', 'erpm', 'motor_current', 'input_current',
                  'fet_temp', 'voltage', 'duty_cycle', 'fault')
        if not all(isinstance(sample.get(k), (int, float)) and
                   not isinstance(sample.get(k), bool) and math.isfinite(sample[k]) for k in fields):
            return self.trip('missing_or_nonfinite_health')
        if not all(math.isfinite(v) for v in (received_mono, now_ros)):
            return self.trip('invalid_clock')
        if self.last is not None:
            if received_mono < self.last[0] or sample['stamp'] <= self.last[1]['stamp']:
                return self.trip('nonincreasing_health_stamp')
            if received_mono - self.last[0] > self.limits.receipt_timeout_s:
                return self.trip('health_receipt_gap')
        if not -.02 <= now_ros - sample['stamp'] <= self.limits.source_timeout_s:
            return self.trip('health_source_timeout')
        checks = (
            (sample['fault'] != 0, 'vesc_fault'),
            (abs(sample['motor_current']) > self.limits.max_motor_current_a, 'motor_overcurrent'),
            (abs(sample['input_current']) > self.limits.max_input_current_a, 'input_overcurrent'),
            (not 0 < sample['fet_temp'] < self.limits.max_fet_temp_c, 'fet_temperature_invalid_or_hot'),
            (sample['voltage'] < self.limits.min_voltage_v, 'low_voltage'),
            (abs(sample['erpm']) > self.limits.max_abs_erpm, 'overspeed'),
            (abs(sample['duty_cycle']) > 1., 'invalid_duty'),
        )
        for failed, reason in checks:
            if failed:
                return self.trip(reason)
        if abs(sample['motor_current']) >= self.limits.stall_current_a and abs(sample['erpm']) < self.limits.stall_erpm:
            if self.stall_since is None:
                self.stall_since = received_mono
            if received_mono - self.stall_since >= self.limits.stall_duration_s:
                return self.trip('high_current_low_rpm')
        else:
            self.stall_since = None
        self.last = (received_mono, dict(sample))
        return self.reason is None

    def healthy(self, now_mono, now_ros):
        if self.reason is not None:
            return False
        if self.last is None:
            return False  # Waiting/disarmed, never assume absent data is zero.
        if not all(math.isfinite(v) for v in (now_mono, now_ros)):
            return self.trip('invalid_clock')
        received, sample = self.last
        if not 0 <= now_mono - received <= self.limits.receipt_timeout_s:
            return self.trip('health_receipt_timeout')
        if not -.02 <= now_ros - sample['stamp'] <= self.limits.source_timeout_s:
            return self.trip('health_source_timeout')
        return True
