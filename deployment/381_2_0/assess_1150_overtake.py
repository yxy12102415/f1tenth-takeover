#!/usr/bin/env python3
"""Offline replay of the compact pass against the measured 1150 us lead speed.

Reads saved native-controller plant trajectories. No ROS, hardware, or command
publishers. This tests the existing path, not every possible overtaking path.
Tracking rollouts use the old synthetic lead; only this replay substitutes the
new lead motion. Constant lead speed is an assumption, not a guaranteed bound.
"""
import json
import math
from pathlib import Path

import numpy as np

from compact_straight_screen import ASSETS, D, sat_collision


def coast_map_check(data, assets, decel=.4):
    """Existing zero-current, centered-steering plant from every driving state."""
    grid = np.asarray(assets['clearance']).reshape(assets['height'], assets['width'])
    ox, oy = assets['origin']
    bx, by = np.meshgrid(np.linspace(-.29, .29, 13), np.linspace(-.155, .155, 9))
    coverage = math.hypot(.58 / 12, .31 / 8) / 2 + .01
    minimum, failed = math.inf, []
    for start in data['trace']:
        x, y, yaw, v, vy, rate, servo = [start[k] for k in
            ['x', 'y', 'yaw', 'speed', 'lateral_speed', 'actual_yaw_rate', 'actual_servo']]
        local_min = math.inf
        for k in range(1500):
            dt = .005
            v = max(0., v - dt * decel)
            servo += dt * -servo / .20
            if v < .5:
                vy, rate = 0., v / .25 * math.tan(servo)
            else:
                m, iz, lf, lr, cf, cr = 3.74, .04712, .12, .13, 90., 96.
                dvy = -(cf + cr) / (m*v)*vy + (-v-(cf*lf-cr*lr)/(m*v))*rate + cf/m*servo
                dr = -(cf*lf-cr*lr)/(iz*v)*vy - (cf*lf*lf+cr*lr*lr)/(iz*v)*rate + lf*cf/iz*servo
                vy += dt*dvy
                rate += dt*dr
            x += dt*(v*math.cos(yaw)-vy*math.sin(yaw))
            y += dt*(v*math.sin(yaw)+vy*math.cos(yaw))
            yaw += dt*rate
            if k % 4 == 0 or v == 0:
                px = x + math.cos(yaw)*bx - math.sin(yaw)*by
                py = y + math.sin(yaw)*bx + math.cos(yaw)*by
                col = np.floor((px-ox)/assets['resolution']).astype(int)
                row = assets['height']-1-np.floor((py-oy)/assets['resolution']).astype(int)
                inside = (col >= 0) & (col < assets['width']) & (row >= 0) & (row < assets['height'])
                clearance = float(np.min(grid[row, col])-coverage) if np.all(inside) else -1.
                local_min = min(local_min, clearance)
            if v == 0:
                break
        minimum = min(minimum, local_min)
        if local_min < .05:
            failed.append(start['t'])
    return dict(deceleration_assumed_mps2=decel, minimum_map_clearance_m=minimum,
                failed_start_times_s=failed, passed=not failed,
                note='Map-only coast check; passing would still require two-car coast checks.')


def assess(path, measured_speed):
    data = json.loads(path.read_text())
    first = data['path'][0]
    trace = data['trace']
    t = np.r_[0., [r['t'] for r in trace]]
    dense = np.arange(0., t[-1], .005)
    pos = np.array([[first[0], first[1], first[2]]] +
                   [[r['x'], r['y'], r['yaw']] for r in trace])
    x, y, yaw = [np.interp(dense, t, pos[:, k]) for k in range(3)]
    c, s = math.cos(first[2]), math.sin(first[2])
    lx, ly = (x-first[0])*c+(y-first[1])*s, -(x-first[0])*s+(y-first[1])*c
    cases = []
    for speed in [measured_speed, .5, .55]:
        lead_x = 1.6 + speed*dense
        physical = sat_collision(lx, ly, yaw-first[2], lead_x, 0.,
                                 lead_l=.30, lead_w=.18, margin=0.)
        guarded = sat_collision(lx, ly, yaw-first[2], lead_x, 0.,
                                lead_l=.36, lead_w=.36, margin=.08)
        cases.append(dict(assumed_constant_lead_speed_mps=speed,
                          first_physical_body_overlap_s=float(dense[physical][0]) if np.any(physical) else None,
                          first_clearance_failure_s=float(dense[guarded][0]) if np.any(guarded) else None,
                          final_center_ahead_gap_m=float(lx[-1]-lead_x[-1])))
    return dict(rollout=str(path), cap_mps=data['spec']['cap_mps'],
                tracking_completed=data['summary']['complete'],
                tracking_failure=data['summary']['failure'],
                duration_s=float(t[-1]), lead_initial_center_gap_assumed_m=1.6,
                replay_cases=cases,
                coast_map=coast_map_check(data, json.loads(ASSETS.read_text())))


def main():
    out = D/'reports/lead_1150_overtake_feasibility'
    out.mkdir(exist_ok=True)
    measurement = json.loads((D/'reports/lead_speed_1150_20261107_v1/speed_summary.json').read_text())
    paths = sorted(out.glob('tracking_*.json'))
    if len(paths) != 3:
        raise RuntimeError('Expected complete native tracking runs for caps 1.25, 1.5 and 1.85')
    cases = []
    for path in paths:
        row = assess(path, measurement['peak_05s_forward_mps'])
        cases.append(row)
        print(json.dumps({k: v for k, v in row.items() if k != 'coast_map'}), flush=True)
        print('coast_map_passed:', row['coast_map']['passed'], flush=True)
    report = dict(scope=__doc__, physical_motion_started=False, physical_motion_ready=False,
                  route_length_m=5.6, measured_lead_peak_mps=measurement['peak_05s_forward_mps'],
                  lead_pwm_us=1150, cases=cases,
                  conclusion='Current compact maneuver is not commissioned. No safety settings changed.')
    (out/'assessment.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
