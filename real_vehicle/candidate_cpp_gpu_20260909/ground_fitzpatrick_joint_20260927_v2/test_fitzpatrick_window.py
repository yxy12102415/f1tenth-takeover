"""Non-actuating outgoing-command speed and STOP regression."""
import json
import ast
import sys
from pathlib import Path
from stand_window import StandWindow

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parents[1]/'src/f1tenth_safety_migration'))
from f1tenth_safety_migration.safety_core import HardwareActuatorConfig, HardwareActuatorCore

# Read the actual launcher parameter dictionary; catch mismatched m/s and RPM
# limits rather than testing a separately typed duplicate configuration.
tree = ast.parse((ROOT/'stand_follow.py').read_text())
actuator_parameters = next(n for n in ast.walk(tree) if isinstance(n, ast.Dict)
    and any(isinstance(k, ast.Constant) and k.value == 'max_abs_rpm' for k in n.keys))
values = {}
for key, value in zip(actuator_parameters.keys, actuator_parameters.values):
    if isinstance(value, ast.IfExp): value = value.orelse
    values[ast.literal_eval(key)] = ast.literal_eval(value)
guard = HardwareActuatorCore(HardwareActuatorConfig(
    max_abs_speed=values['max_abs_speed'], max_abs_rpm=values['max_abs_rpm'],
    final_command_timeout=values['final_command_timeout']))
guard.update(2., 0., 0.)
assert guard.evaluate(0.).rpm == 15000.
guard.update(2.01, 0., .02)
assert guard.evaluate(.02).motor_mode == 'coast'
guard.update(0., 0., .04)
assert guard.evaluate(.04).motor_mode == 'coast'


def ready_command(w, speed, now):
    w.health(True, now)
    w.permit(True, now)
    w.joy(True, now)
    w.command(speed, 0., now)
    return w.evaluate(now)


results = []
for final_speed, expected_reason in ((.49, 'sub_floor_command'),
                                     (0., 'upstream_stop'),
                                     (2.01, 'invalid_command')):
    w = StandWindow()
    w.joy(False, 0.)
    speeds = []
    for i in range(160):
        v, _, why = ready_command(w, 2., i*.02)
        assert not why, why
        speeds.append(v)
    assert speeds[0] <= .02 and speeds[-1] == 2.
    assert max(b-a for a, b in zip(speeds, speeds[1:])) <= .020001
    # Normal near-end deceleration may use .5, but not a .49 creep.
    assert ready_command(w, .5, 3.2)[0] == .5
    v, _, reason = ready_command(w, final_speed, 3.22)
    assert v == 0. and reason == expected_reason, (v, reason)
    assert ready_command(w, 2., 3.24)[0] == 0., 'STOP must remain latched'
    results.append(dict(input_mps=final_speed, stop_reason=reason))

report = dict(passed=True, output_cap_mps=2., startup_ramp_preserved=True,
              hardware_guard_at_2mps_erpm=15000.,
              post_launch_sub_floor_rejected=True, terminal_zero_latched=True,
              cases=results, physical_motion=False)
out = Path(__file__).resolve().parent/'run_logs/fitzpatrick_window_contract.json'
out.write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
